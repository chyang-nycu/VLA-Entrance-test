#!/usr/bin/env python3
"""Phase 3C: position-servo architecture, waypoint IK, granular state machine.

Architecture replacement for Phase 3/3B's torque-PD + resolved-rate DLS-IK
(both historical, kept unmodified in controller.py / run_grasp_test.py).
Right-arm joints are driven by bounded MuJoCo <position> servos (see
gripper_scene.write_grasp_scene_3c); IK is solved once per motion segment
(controller_3c.solve_ik_waypoint), not re-solved every control step.

State machine: RESET -> PREGRASP -> SETTLE_PREGRASP -> APPROACH ->
SETTLE_APPROACH -> CLOSE -> VERIFY_BILATERAL_CONTACT -> LIFT -> HOLD ->
LOWER -> OPEN -> DONE/FAILED.

Initialization boundary: identical rule and identical enforcement mechanism
as Phase 3/3B (CubeInitGuard, imported unchanged from run_grasp_test.py) --
cube qpos/qvel settable only before the trial's first mj_step, hard-raises
after. This module's own run_trial_3c is source-scanned for the same
invariant (see _assert_run_trial_3c_has_no_direct_cube_state_write below),
independently of Phase 3/3B's self-audit.
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import (
    GRIPPER_ACTUATORS,
    GRIPPER_JOINTS,
    GRIPPER_KD as GRIPPER_KD_PHASE3,
    GRIPPER_KP as GRIPPER_KP_PHASE3,
    GRIPPER_MAX_QVEL,
    GRIPPER_MAX_STEP,
    RIGHT_ARM_ACTUATORS,
    RIGHT_ARM_JOINTS,
    TCP_SITE,
    JointMap,
    bounded_pd_step,
)

# Phase 3C, Attempt 3C-2: the fingers lost grip mid-LIFT with Phase 3/3B's
# historical GRIPPER_KP/KD (40, 2) -- direct evidence (see
# reports/phase3c-position-servo-baseline.md): both pads made contact and
# the cube lifted momentarily, then contact was lost partway through LIFT
# and the cube fell back to rest, well before the 2.0 s hold requirement.
# The finger *force limit* (15 N) was not the bottleneck -- a back-of-envelope
# friction check (mu=1.2, cube mass 0.05 kg) needs well under 1 N of normal
# force per pad to support the cube even under a large safety-factor
# acceleration, so 15 N was never close to binding. The bottleneck was
# tracking stiffness/damping: the finger position-tracking PD was too soft
# to hold its commanded (overtraveled, squeezing) target through the
# arm's LIFT transient, so effective squeeze force sagged below what was
# needed at exactly the moment load increased. Kept separate from
# controller.py's GRIPPER_KP/KD (imported above under different names) so
# Phase 3/3B's historical gripper behavior is completely unaffected.
GRIPPER_KP_3C = 150.0
GRIPPER_KD_3C = 10.0
from tasks.g1_pick_place.controller_3c import IK_POS_TOL, solve_ik_waypoint
from tasks.g1_pick_place.gripper_scene import (
    CUBE_HALF,
    CUBE_POS,
    FINGER_OPEN_Y,
    TABLE_TOP_Z,
    write_grasp_scene_3c,
)
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard, _contacts_between

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"

TIMESTEP = 0.002
PREGRASP_DZ = 0.10
LIFT_DZ = 0.12
LIFT_HOLD_S = 2.5
CONTACT_MARGIN = 0.03

# Segment (fixed-duration drive) durations, then a bounded settle extension
# for the two gated segments (PREGRASP, APPROACH).
DRIVE_S = {
    "PREGRASP": 0.8,
    "APPROACH": 0.8,
    "CLOSE": 0.8,
    "LIFT": 1.0,
    "HOLD": LIFT_HOLD_S,
    "LOWER": 1.0,
    "OPEN": 0.6,
    "RELEASE_SETTLE": 0.8,
}
SETTLE_MAX_EXTRA_S = 1.5
SETTLE_TCP_POS_TOL = 0.01  # 1 cm -- looser than the IK solve tolerance itself,
# this is a *dynamic* settle criterion (has the servo caught up to its fixed
# set-point), not the *kinematic* solve tolerance (IK_POS_TOL).
SETTLE_ARM_QVEL_TOL = 0.15  # rad/s

GRASP_CORRIDOR_XY_M = 0.02  # cube must stay within this xy displacement from
# spawn before LIFT is attempted; chosen conservatively below the fingers'
# physical reach margin (FINGER_OPEN_Y - CUBE_HALF - pad half-thickness
# ~= 0.075 - 0.035 - 0.006 = 0.034 m).
FINGER_CLOSE_VEL_TOL = 0.03  # m/s, "closing velocity near zero" gate

VARIANT_OFFSETS = [
    (0.03, 0.0),
    (-0.03, 0.0),
    (0.0, 0.03),
    (0.0, -0.03),
    (0.02, -0.02),
]


def _finger_targets(gripper_map: JointMap, open_: bool) -> np.ndarray:
    if open_:
        return np.array([0.0, 0.0])
    return np.array([gripper_map.jnt_range[0, 0], gripper_map.jnt_range[1, 1]])


def diagnose_reachability(
    model: mujoco.MjModel,
    arm_map: JointMap,
    site_id: int,
    base_qpos: np.ndarray,
    cube_pos: np.ndarray,
) -> dict:
    """Solve IK for PREGRASP, APPROACH (grasp), CLOSED-LIFT, and HOLD before
    any simulation is run. Chains each solve from the previous waypoint's
    solution (as the real trial will move continuously through them), and
    reports residual error + solved joint config for each, so an unreachable
    target is caught before wasting a simulation run.
    """
    scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(arm_map.names))
    waypoints = {
        "PREGRASP": cube_pos + np.array([0.0, 0.0, PREGRASP_DZ]),
        "APPROACH": cube_pos.copy(),
        "CLOSED_LIFT": cube_pos + np.array([0.0, 0.0, LIFT_DZ]),
        "HOLD": cube_pos + np.array([0.0, 0.0, LIFT_DZ]),
    }
    report = {}
    q_prev = base_qpos.copy()
    for name, target in waypoints.items():
        q, resid, iters = solve_ik_waypoint(model, scratch, q_prev, arm_map, site_id, target, nominal_q)
        reachable = resid < IK_POS_TOL
        report[name] = {
            "target_pos": target.tolist(),
            "solved_joint_target": q.tolist(),
            "residual_m": resid,
            "iterations": iters,
            "reachable_within_tol": bool(reachable),
        }
        scratch.qpos[:] = q_prev
        arm_map.set_qpos(scratch, q)
        q_prev = scratch.qpos.copy()
    report["all_reachable"] = all(v["reachable_within_tol"] for v in report.values() if isinstance(v, dict))
    return report


def run_trial_3c(
    model_path: Path,
    arm_map_names: tuple = tuple(RIGHT_ARM_JOINTS),
    cube_xy_offset: tuple[float, float] = (0.0, 0.0),
    diagnostics: dict | None = None,
    gripper_kp: float = GRIPPER_KP_3C,
    gripper_kd: float = GRIPPER_KD_3C,
) -> dict:
    """One Phase 3C nominal/variant trial through the full state machine.

    Returns a result dict with the same 5 acceptance-criteria keys as
    Phase 3/3B (both_pads_contact_cube, height_gain_ge_0_08m,
    lifted_ge_2s_continuous, finite_and_bounded, released_after_open) plus
    `failure_state`/`failure_reason` when the trial is stopped early by a
    state-machine gate, and `states` recording which states were entered.
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    gripper_map = JointMap.build(model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    ik_scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(RIGHT_ARM_JOINTS))

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    cube_qpos_adr = int(model.jnt_qposadr[cube_joint_id])
    cube_dof_adr = int(model.jnt_dofadr[cube_joint_id])

    # --- RESET ---
    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    guard = CubeInitGuard(data, cube_qpos_adr, cube_dof_adr)
    guard.set_initial_pose([cube_x, cube_y, cube_z])
    mujoco.mj_forward(model, data)
    steps_run = [0]

    rest_z = float(data.xpos[cube_body_id][2])
    cube_pos = np.array([cube_x, cube_y, cube_z])
    pregrasp_target = cube_pos + np.array([0.0, 0.0, PREGRASP_DZ])
    lift_target = cube_pos + np.array([0.0, 0.0, LIFT_DZ])

    reachability = diagnose_reachability(model, arm_map, site_id, data.qpos.copy(), cube_pos)

    telemetry = {
        "states_entered": ["RESET"],
        "failure_state": None,
        "failure_reason": None,
        "both_pads_contact_cube": False,
        "max_cube_z": rest_z,
        "max_continuous_lifted_s": 0.0,
        "finite_and_bounded": True,
        "released_after_open": None,
        "cube_rest_z": rest_z,
        "left_contact_ever": False,
        "right_contact_ever": False,
        "cube_xy_before_lift": None,
        "arm_force_saturation": {},
        "arm_tracking_error_rms": {},
        "arm_tracking_error_max": {},
        "settle_extra_s": {},
    }
    lifted_since = None

    def _step_once(arm_ctrl_target: np.ndarray, finger_open: bool, phase: str) -> None:
        nonlocal lifted_since
        arm_map.set_ctrl(data, arm_ctrl_target)
        bounded_pd_step(
            gripper_map, data, _finger_targets(gripper_map, finger_open),
            gripper_kp, gripper_kd, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL,
        )
        if not np.all(np.isfinite(data.ctrl)):
            telemetry["finite_and_bounded"] = False
        mujoco.mj_step(model, data)
        steps_run[0] += 1
        if steps_run[0] == 1:
            guard.lock()

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            telemetry["finite_and_bounded"] = False

        force = data.actuator_force[arm_map.actuator_id].copy()
        frange = arm_map.ctrl_range if False else None  # placeholder, unused
        cube_z = float(data.xpos[cube_body_id][2])
        telemetry["max_cube_z"] = max(telemetry["max_cube_z"], cube_z)

        left_contact = _contacts_between(data, model, cube_geom_id, left_pad_id)
        right_contact = _contacts_between(data, model, cube_geom_id, right_pad_id)
        telemetry["left_contact_ever"] = telemetry["left_contact_ever"] or left_contact
        telemetry["right_contact_ever"] = telemetry["right_contact_ever"] or right_contact
        if left_contact and right_contact:
            telemetry["both_pads_contact_cube"] = True

        is_lifted = cube_z > (rest_z + CONTACT_MARGIN)
        if is_lifted:
            if lifted_since is None:
                lifted_since = data.time
            telemetry["max_continuous_lifted_s"] = max(
                telemetry["max_continuous_lifted_s"], data.time - lifted_since
            )
        else:
            lifted_since = None

        if diagnostics is not None:
            bucket = diagnostics.setdefault(
                phase, {"actuator_force": [], "qpos_err": [], "tcp_err": [], "left_contact": [], "right_contact": []}
            )
            bucket["actuator_force"].append(force.tolist())
            bucket["left_contact"].append(bool(left_contact))
            bucket["right_contact"].append(bool(right_contact))

    def _drive_segment(target_pos: np.ndarray, finger_open: bool, phase: str, duration_s: float, joint_target: np.ndarray):
        n = max(1, int(round(duration_s / TIMESTEP)))
        qpos_errs = []
        force_frac = []
        force_limit = np.abs(model.actuator_forcerange[arm_map.actuator_id, 1])
        for _ in range(n):
            _step_once(joint_target, finger_open, phase)
            qpos_errs.append(np.abs(joint_target - arm_map.get_qpos(data)))
            force_frac.append(np.abs(data.actuator_force[arm_map.actuator_id]) / np.maximum(force_limit, 1e-9))
        qpos_errs = np.array(qpos_errs)
        force_frac = np.array(force_frac)
        telemetry["arm_tracking_error_rms"][phase] = float(np.sqrt(np.mean(qpos_errs ** 2)))
        telemetry["arm_tracking_error_max"][phase] = float(np.max(qpos_errs))
        telemetry["arm_force_saturation"][phase] = (force_frac > 0.99).mean(axis=0).tolist()

    def _settle(target_pos: np.ndarray, finger_open: bool, phase: str, joint_target: np.ndarray) -> bool:
        max_steps = int(round(SETTLE_MAX_EXTRA_S / TIMESTEP))
        settled = False
        n_steps = 0
        for _ in range(max_steps):
            _step_once(joint_target, finger_open, phase)
            n_steps += 1
            tcp_pos = data.site_xpos[site_id]
            pos_err = float(np.linalg.norm(target_pos - tcp_pos))
            joint_speed = float(np.max(np.abs(arm_map.get_qvel(data))))
            if pos_err <= SETTLE_TCP_POS_TOL and joint_speed <= SETTLE_ARM_QVEL_TOL:
                settled = True
                break
        telemetry["settle_extra_s"][phase] = n_steps * TIMESTEP
        return settled

    def _fail(state: str, reason: str) -> dict:
        telemetry["failure_state"] = state
        telemetry["failure_reason"] = reason
        return _finalize()

    def _finalize() -> dict:
        height_gain = telemetry["max_cube_z"] - rest_z
        criteria = {
            "both_pads_contact_cube": telemetry["both_pads_contact_cube"],
            "height_gain_ge_0_08m": height_gain >= 0.08,
            "lifted_ge_2s_continuous": telemetry["max_continuous_lifted_s"] >= 2.0,
            "finite_and_bounded": telemetry["finite_and_bounded"],
            "released_after_open": bool(telemetry["released_after_open"]) if telemetry["released_after_open"] is not None else False,
        }
        return {
            "cube_xy_offset": list(cube_xy_offset),
            "cube_spawn_pos": [cube_x, cube_y, cube_z],
            "cube_rest_z": rest_z,
            "height_gain_m": height_gain,
            "max_continuous_lifted_s": telemetry["max_continuous_lifted_s"],
            "left_contact_ever": telemetry["left_contact_ever"],
            "right_contact_ever": telemetry["right_contact_ever"],
            "states_entered": telemetry["states_entered"],
            "failure_state": telemetry["failure_state"],
            "failure_reason": telemetry["failure_reason"],
            "arm_tracking_error_rms": telemetry["arm_tracking_error_rms"],
            "arm_tracking_error_max": telemetry["arm_tracking_error_max"],
            "arm_force_saturation": telemetry["arm_force_saturation"],
            "settle_extra_s": telemetry["settle_extra_s"],
            "reachability": reachability,
            "criteria": criteria,
            "pass": all(criteria.values()) and telemetry["failure_state"] is None,
        }

    # --- PREGRASP ---
    telemetry["states_entered"].append("PREGRASP")
    q_target, resid, _ = solve_ik_waypoint(model, ik_scratch, data.qpos.copy(), arm_map, site_id, pregrasp_target, nominal_q)
    _drive_segment(pregrasp_target, True, "PREGRASP", DRIVE_S["PREGRASP"], q_target)

    telemetry["states_entered"].append("SETTLE_PREGRASP")
    settled = _settle(pregrasp_target, True, "SETTLE_PREGRASP", q_target)
    if not settled:
        return _fail("SETTLE_PREGRASP", "TCP did not settle within tolerance before APPROACH")

    # --- APPROACH ---
    telemetry["states_entered"].append("APPROACH")
    q_target2, resid2, _ = solve_ik_waypoint(model, ik_scratch, data.qpos.copy(), arm_map, site_id, cube_pos, nominal_q)
    _drive_segment(cube_pos, True, "APPROACH", DRIVE_S["APPROACH"], q_target2)

    telemetry["states_entered"].append("SETTLE_APPROACH")
    settled2 = _settle(cube_pos, True, "SETTLE_APPROACH", q_target2)
    if not settled2:
        return _fail("SETTLE_APPROACH", "TCP did not settle within tolerance before CLOSE")

    cube_xy_before_close = np.array([float(data.xpos[cube_body_id][0]), float(data.xpos[cube_body_id][1])])

    # --- CLOSE (arm ctrl held at the settled approach joint target) ---
    telemetry["states_entered"].append("CLOSE")
    _drive_segment(cube_pos, False, "CLOSE", DRIVE_S["CLOSE"], q_target2)

    # --- VERIFY_BILATERAL_CONTACT ---
    telemetry["states_entered"].append("VERIFY_BILATERAL_CONTACT")
    left_now = _contacts_between(data, model, cube_geom_id, left_pad_id)
    right_now = _contacts_between(data, model, cube_geom_id, right_pad_id)
    finger_qvel = gripper_map.get_qvel(data)
    closing_settled = bool(np.all(np.abs(finger_qvel) < FINGER_CLOSE_VEL_TOL))
    cube_xy_now = np.array([float(data.xpos[cube_body_id][0]), float(data.xpos[cube_body_id][1])])
    cube_xy_spawn = np.array([cube_x, cube_y])
    displacement = float(np.linalg.norm(cube_xy_now - cube_xy_spawn))
    telemetry["cube_xy_before_lift"] = cube_xy_now.tolist()
    within_corridor = displacement <= GRASP_CORRIDOR_XY_M

    if not (left_now and right_now):
        return _fail("VERIFY_BILATERAL_CONTACT", f"no simultaneous bilateral contact at verify time (left={left_now}, right={right_now})")
    if not closing_settled:
        return _fail("VERIFY_BILATERAL_CONTACT", f"finger closing velocity not settled: {finger_qvel.tolist()}")
    if not within_corridor:
        return _fail("VERIFY_BILATERAL_CONTACT", f"cube displaced {displacement:.4f} m > corridor {GRASP_CORRIDOR_XY_M} m before lift")
    telemetry["both_pads_contact_cube"] = True

    # --- LIFT ---
    telemetry["states_entered"].append("LIFT")
    q_lift, resid3, _ = solve_ik_waypoint(model, ik_scratch, data.qpos.copy(), arm_map, site_id, lift_target, nominal_q)
    _drive_segment(lift_target, False, "LIFT", DRIVE_S["LIFT"], q_lift)

    # --- HOLD ---
    telemetry["states_entered"].append("HOLD")
    _drive_segment(lift_target, False, "HOLD", DRIVE_S["HOLD"], q_lift)

    # --- LOWER ---
    telemetry["states_entered"].append("LOWER")
    q_lower, resid4, _ = solve_ik_waypoint(model, ik_scratch, data.qpos.copy(), arm_map, site_id, cube_pos, nominal_q)
    _drive_segment(cube_pos, False, "LOWER", DRIVE_S["LOWER"], q_lower)

    # --- OPEN ---
    telemetry["states_entered"].append("OPEN")
    _drive_segment(cube_pos, True, "OPEN", DRIVE_S["OPEN"], q_lower)
    _drive_segment(cube_pos, True, "RELEASE_SETTLE", DRIVE_S["RELEASE_SETTLE"], q_lower)
    telemetry["released_after_open"] = not (
        _contacts_between(data, model, cube_geom_id, left_pad_id)
        or _contacts_between(data, model, cube_geom_id, right_pad_id)
    )

    telemetry["states_entered"].append("DONE")
    return _finalize()


# --- initialization-boundary self-audit (independent of Phase 3/3B's) -------
def _assert_run_trial_3c_has_no_direct_cube_state_write() -> None:
    src = inspect.getsource(run_trial_3c)
    forbidden = ["data.qpos[cube_qpos_adr", "data.qvel[cube_dof_adr", "xfrc_applied[cube_body_id"]
    for pattern in forbidden:
        if pattern in src:
            raise AssertionError(
                f"run_trial_3c() contains a direct cube-state write ({pattern!r}) "
                "outside CubeInitGuard -- initialization boundary violated"
            )


_assert_run_trial_3c_has_no_direct_cube_state_write()


def main() -> int:
    raise SystemExit("run_grasp_test_3c.run_trial_3c is driven by phase3c_tuning.py's attempt harness")


if __name__ == "__main__":
    raise SystemExit(main())
