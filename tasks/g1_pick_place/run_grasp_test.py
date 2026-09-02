#!/usr/bin/env python3
"""Phase 3 nominal grasp-and-lift trial + 5 deterministic position variants.

Sequence: pregrasp -> approach -> close -> lift -> hold -> lower -> open.

Initialization boundary (HANDOFF.md, clarified for Phase 3B): the cube's
free-joint qpos may only be set once, via CubeInitGuard.set_initial_pose(),
strictly before the trial's first mujoco.mj_step() call -- this is choosing
a deterministic initial condition, equivalent to hardcoding the position in
the MJCF. After that first step, CubeInitGuard.lock() is called and any
further attempt to set the cube's qpos/qvel through the guard raises
RuntimeError. No other code path in this module writes to the cube's
qpos/qvel/xfrc_applied at any time -- run_trial's own source is scanned for
that invariant by tests/test_phase3_grasp.py.
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import G1GraspController
from tasks.g1_pick_place.gripper_scene import (
    CUBE_HALF,
    CUBE_POS,
    TABLE_TOP_Z,
    write_grasp_scene,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"


class CubeInitGuard:
    """Enforces the initialization boundary for one trial's cube body.

    set_initial_pose()/set_initial_velocity() may be called any number of
    times before lock(); after lock() (called right after the trial's first
    mj_step), any further call raises RuntimeError instead of silently
    writing simulation state.
    """

    def __init__(self, data: mujoco.MjData, qpos_adr: int, dof_adr: int) -> None:
        self._data = data
        self._qpos_adr = qpos_adr
        self._dof_adr = dof_adr
        self._locked = False

    def set_initial_pose(self, pos, quat=(1.0, 0.0, 0.0, 0.0)) -> None:
        if self._locked:
            raise RuntimeError(
                "cube initialization boundary violated: qpos write attempted "
                "after the trial's first physics step"
            )
        self._data.qpos[self._qpos_adr : self._qpos_adr + 3] = pos
        self._data.qpos[self._qpos_adr + 3 : self._qpos_adr + 7] = quat

    def set_initial_velocity(self, vel=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) -> None:
        if self._locked:
            raise RuntimeError(
                "cube initialization boundary violated: qvel write attempted "
                "after the trial's first physics step"
            )
        self._data.qvel[self._dof_adr : self._dof_adr + 6] = vel

    def lock(self) -> None:
        self._locked = True

    @property
    def locked(self) -> bool:
        return self._locked

TIMESTEP = 0.002
PREGRASP_DZ = 0.10
LIFT_DZ = 0.12
LIFT_HOLD_S = 2.5
CONTACT_MARGIN = 0.03  # cube counted "lifted off table" once z exceeds rest height by this much

PHASE_DURATIONS_S = {
    "to_pregrasp": 1.0,
    "approach": 1.0,
    "close": 0.6,
    "lift": 1.0,
    "hold": LIFT_HOLD_S,
    "lower": 1.0,
    "open": 0.6,
    "release_settle": 0.8,
}

NOMINAL_REST_Z = CUBE_POS[2]

VARIANT_OFFSETS = [
    (0.03, 0.0),
    (-0.03, 0.0),
    (0.0, 0.03),
    (0.0, -0.03),
    (0.02, -0.02),
]


def _finger_targets(gripper_map, open_: bool) -> np.ndarray:
    if open_:
        return np.array([0.0, 0.0])
    return np.array([gripper_map.jnt_range[0, 0], gripper_map.jnt_range[1, 1]])


def _contacts_between(data: mujoco.MjData, model: mujoco.MjModel, geom_a: int, geom_b: int) -> bool:
    for i in range(data.ncon):
        c = data.contact[i]
        pair = (int(c.geom1), int(c.geom2))
        if geom_a in pair and geom_b in pair:
            return True
    return False


def run_trial(
    model_path: Path,
    cube_xy_offset: tuple[float, float] = (0.0, 0.0),
    controller: G1GraspController | None = None,
    diagnostics: dict | None = None,
    settle_before_close: bool = False,
    settle_pos_tol: float = 0.005,
    settle_vel_tol: float = 0.05,
    max_settle_extra_s: float = 1.0,
) -> dict:
    """Run one nominal/variant grasp trial.

    `controller` lets Phase 3B inject non-default gains (e.g. per-joint
    scaling) without duplicating this trial's logic -- the same code path
    used for grading is the one used for tuning. `diagnostics`, if given,
    collects per-step per-joint instrumentation (torque pre/post clip,
    saturation, tracking error, TCP error) keyed by phase name, for the
    Phase 3B instrumentation/attempt records. `settle_before_close` is
    Attempt 3B-2's approach->close gating: instead of a fixed-duration
    approach phase, keep tracking the grasp waypoint until the TCP is
    within `settle_pos_tol` and joint speeds are within `settle_vel_tol`
    (or `max_settle_extra_s` extra time elapses), so the arm is not asked
    to close while still oscillating.
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    ctrl = controller if controller is not None else G1GraspController(model=model)

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    cube_qpos_adr = int(model.jnt_qposadr[cube_joint_id])
    cube_dof_adr = int(model.jnt_dofadr[cube_joint_id])

    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    guard = CubeInitGuard(data, cube_qpos_adr, cube_dof_adr)
    guard.set_initial_pose([cube_x, cube_y, cube_z])
    mujoco.mj_forward(model, data)

    rest_z = float(data.xpos[cube_body_id][2])
    grasp_target = np.array([cube_x, cube_y, cube_z])
    pregrasp_target = grasp_target + np.array([0.0, 0.0, PREGRASP_DZ])
    lift_target = grasp_target + np.array([0.0, 0.0, LIFT_DZ])

    telemetry = {
        "both_pads_contact_cube": False,
        "max_cube_z": rest_z,
        "max_continuous_lifted_s": 0.0,
        "finite_and_bounded": True,
        "released_after_open": None,
        "cube_rest_z": rest_z,
        "time_series": [],
        "cube_xy_before_close": None,
        "left_contact_ever": False,
        "right_contact_ever": False,
        "settle_extra_s": 0.0,
    }

    lifted_since = None
    steps_run = [0]

    def _record_diag(phase: str, arm_diag: dict, tcp_err: np.ndarray) -> None:
        if diagnostics is None:
            return
        bucket = diagnostics.setdefault(
            phase,
            {
                "torque_pre_clip": [],
                "torque_post_clip": [],
                "saturated": [],
                "joint_error": [],
                "tcp_err": [],
            },
        )
        bucket["torque_pre_clip"].append(arm_diag["torque_pre_clip"].tolist())
        bucket["torque_post_clip"].append(arm_diag["torque_post_clip"].tolist())
        bucket["saturated"].append(arm_diag["saturated"].tolist())
        bucket["joint_error"].append(arm_diag["joint_error"].tolist())
        bucket["tcp_err"].append(tcp_err.tolist())

    def step_toward(target_pos, finger_open: bool, duration_s: float, start_pos=None, phase: str = ""):
        nonlocal lifted_since
        n = max(1, int(round(duration_s / TIMESTEP)))
        finger_target = _finger_targets(ctrl.gripper_map, finger_open)
        start = start_pos if start_pos is not None else ctrl.tcp_pos(data)
        for i in range(n):
            alpha = (i + 1) / n
            waypoint = start + alpha * (target_pos - start)
            joint_target = ctrl.ik_target_for(data, waypoint)
            arm_diag: dict = {} if diagnostics is not None else None
            arm_torque = ctrl.track_arm(data, joint_target, diag=arm_diag)
            grip_torque = ctrl.track_gripper(data, finger_target)

            if not (np.all(np.isfinite(arm_torque)) and np.all(np.isfinite(grip_torque))):
                telemetry["finite_and_bounded"] = False
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                telemetry["finite_and_bounded"] = False

            mujoco.mj_step(model, data)
            steps_run[0] += 1
            if steps_run[0] == 1:
                guard.lock()

            if arm_diag is not None:
                tcp_err = target_pos - ctrl.tcp_pos(data)
                _record_diag(phase, arm_diag, tcp_err)

            cube_z = float(data.xpos[cube_body_id][2])
            telemetry["max_cube_z"] = max(telemetry["max_cube_z"], cube_z)

            left_contact = _contacts_between(data, model, cube_geom_id, left_pad_id)
            right_contact = _contacts_between(data, model, cube_geom_id, right_pad_id)
            telemetry["left_contact_ever"] = telemetry["left_contact_ever"] or left_contact
            telemetry["right_contact_ever"] = telemetry["right_contact_ever"] or right_contact
            both_contact = left_contact and right_contact
            if both_contact:
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

            if steps_run[0] % 50 == 0:
                telemetry["time_series"].append(
                    {"t": float(data.time), "cube_z": cube_z, "both_pads_contact": bool(both_contact)}
                )

    step_toward(pregrasp_target, finger_open=True, duration_s=PHASE_DURATIONS_S["to_pregrasp"], phase="to_pregrasp")

    if settle_before_close:
        # Attempt 3B-2: hold the approach target open-loop-in-time but
        # closed-loop-in-tolerance -- keep tracking until TCP position error
        # and right-arm joint speed both settle, instead of a fixed 1.0 s
        # window, before ever commanding the fingers closed.
        n_fixed = max(1, int(round(PHASE_DURATIONS_S["approach"] / TIMESTEP)))
        start = ctrl.tcp_pos(data)
        for i in range(n_fixed):
            alpha = (i + 1) / n_fixed
            waypoint = start + alpha * (grasp_target - start)
            joint_target = ctrl.ik_target_for(data, waypoint)
            arm_diag: dict = {} if diagnostics is not None else None
            arm_torque = ctrl.track_arm(data, joint_target, diag=arm_diag)
            grip_torque = ctrl.track_gripper(data, _finger_targets(ctrl.gripper_map, True))
            if not (np.all(np.isfinite(arm_torque)) and np.all(np.isfinite(grip_torque))):
                telemetry["finite_and_bounded"] = False
            mujoco.mj_step(model, data)
            steps_run[0] += 1
            if steps_run[0] == 1:
                guard.lock()
            if arm_diag is not None:
                _record_diag("approach", arm_diag, grasp_target - ctrl.tcp_pos(data))
            cube_z = float(data.xpos[cube_body_id][2])
            telemetry["max_cube_z"] = max(telemetry["max_cube_z"], cube_z)

        extra_steps = 0
        max_extra_steps = int(round(max_settle_extra_s / TIMESTEP))
        while extra_steps < max_extra_steps:
            joint_target = ctrl.ik_target_for(data, grasp_target)
            arm_diag = {} if diagnostics is not None else None
            ctrl.track_arm(data, joint_target, diag=arm_diag)
            ctrl.track_gripper(data, _finger_targets(ctrl.gripper_map, True))
            mujoco.mj_step(model, data)
            steps_run[0] += 1
            extra_steps += 1
            if arm_diag is not None:
                _record_diag("approach_settle", arm_diag, grasp_target - ctrl.tcp_pos(data))
            pos_err = float(np.linalg.norm(grasp_target - ctrl.tcp_pos(data)))
            joint_speed = float(np.max(np.abs(ctrl.arm_map.get_qvel(data))))
            if pos_err <= settle_pos_tol and joint_speed <= settle_vel_tol:
                break
        telemetry["settle_extra_s"] = extra_steps * TIMESTEP
    else:
        step_toward(grasp_target, finger_open=True, duration_s=PHASE_DURATIONS_S["approach"], phase="approach")

    telemetry["cube_xy_before_close"] = [float(data.xpos[cube_body_id][0]), float(data.xpos[cube_body_id][1])]

    step_toward(grasp_target, finger_open=False, duration_s=PHASE_DURATIONS_S["close"], phase="close")
    step_toward(lift_target, finger_open=False, duration_s=PHASE_DURATIONS_S["lift"], phase="lift")
    step_toward(lift_target, finger_open=False, duration_s=PHASE_DURATIONS_S["hold"], phase="hold")
    step_toward(grasp_target, finger_open=False, duration_s=PHASE_DURATIONS_S["lower"], phase="lower")
    step_toward(grasp_target, finger_open=True, duration_s=PHASE_DURATIONS_S["open"], phase="open")

    settle_n = int(round(PHASE_DURATIONS_S["release_settle"] / TIMESTEP))
    open_finger_target = _finger_targets(ctrl.gripper_map, True)
    for _ in range(settle_n):
        ctrl.track_gripper(data, open_finger_target)
        arm_hold_target = ctrl.ik_target_for(data, grasp_target)
        ctrl.track_arm(data, arm_hold_target)
        mujoco.mj_step(model, data)
        steps_run[0] += 1
        if steps_run[0] == 1:
            guard.lock()
    telemetry["released_after_open"] = not (
        _contacts_between(data, model, cube_geom_id, left_pad_id)
        or _contacts_between(data, model, cube_geom_id, right_pad_id)
    )

    height_gain = telemetry["max_cube_z"] - rest_z
    criteria = {
        "both_pads_contact_cube": telemetry["both_pads_contact_cube"],
        "height_gain_ge_0_08m": height_gain >= 0.08,
        "lifted_ge_2s_continuous": telemetry["max_continuous_lifted_s"] >= 2.0,
        "finite_and_bounded": telemetry["finite_and_bounded"],
        "released_after_open": bool(telemetry["released_after_open"]),
    }

    cube_xy_spawn = np.array([cube_x, cube_y])
    cube_xy_before_close = np.array(telemetry["cube_xy_before_close"])
    return {
        "cube_xy_offset": list(cube_xy_offset),
        "cube_spawn_pos": [cube_x, cube_y, cube_z],
        "cube_rest_z": rest_z,
        "height_gain_m": height_gain,
        "max_continuous_lifted_s": telemetry["max_continuous_lifted_s"],
        "cube_xy_displacement_before_close_m": float(np.linalg.norm(cube_xy_before_close - cube_xy_spawn)),
        "left_contact_ever": telemetry["left_contact_ever"],
        "right_contact_ever": telemetry["right_contact_ever"],
        "settle_extra_s": telemetry["settle_extra_s"],
        "criteria": criteria,
        "pass": all(criteria.values()),
        "time_series_sample": telemetry["time_series"],
    }


# --- initialization-boundary self-audit -------------------------------------
# The only place this module may write cube qpos/qvel is inside
# CubeInitGuard's own methods, called from run_trial before the first
# mj_step. This scans run_trial's own source (not CubeInitGuard's) to catch
# a regression that reintroduces a direct write outside the guard.
def _assert_run_trial_has_no_direct_cube_state_write() -> None:
    src = inspect.getsource(run_trial)
    forbidden = ["data.qpos[cube_qpos_adr", "data.qvel[cube_dof_adr", "xfrc_applied[cube_body_id]"]
    for pattern in forbidden:
        if pattern in src:
            raise AssertionError(
                f"run_trial() contains a direct cube-state write ({pattern!r}) "
                "outside CubeInitGuard -- initialization boundary violated"
            )


_assert_run_trial_has_no_direct_cube_state_write()


def main() -> int:
    scene_path = write_grasp_scene()

    nominal = run_trial(scene_path, cube_xy_offset=(0.0, 0.0))
    result = {"scene": str(scene_path.relative_to(ROOT)), "nominal": nominal}

    if not nominal["pass"]:
        result["variants"] = []
        result["variant_success_rate"] = None
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        (LOG_DIR / "phase3_grasp_trials.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"nominal_pass": False, "criteria": nominal["criteria"]}, indent=2))
        return 1

    variants = []
    for offset in VARIANT_OFFSETS:
        variants.append(run_trial(scene_path, cube_xy_offset=offset))
    result["variants"] = variants
    result["variant_success_rate"] = sum(1 for v in variants if v["pass"]) / len(variants)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase3_grasp_trials.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "nominal_pass": nominal["pass"],
                "variant_success_rate": result["variant_success_rate"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
