#!/usr/bin/env python3
"""Phase 4B: Task 1 complete pick-and-place -- "pick up the red cube and
place it in the blue target area."

Extends Phase 3C's grasp state machine (run_grasp_test_3c.run_trial_3c,
already verified: nominal PASS, 5/5 deterministic) with physical transport,
lowering, release, retreat, and an objective, state-based (not rendering-
based) task-success detector.

This module does not modify run_grasp_test_3c.py, controller_3c.py,
controller.py, or run_grasp_test.py. The RESET..OPEN segment below is
written against the identical constants those modules already export
(DRIVE_S, PREGRASP_DZ, LIFT_DZ, GRASP_CORRIDOR_XY_M, FINGER_CLOSE_VEL_TOL,
SETTLE_TCP_POS_TOL, SETTLE_ARM_QVEL_TOL, SETTLE_MAX_EXTRA_S,
GRIPPER_KP_3C/KD_3C) -- so the grasp portion of this trial reproduces
Phase 3C's already-verified physics exactly, not a reimplementation with
new numbers. Only the post-grasp portion (TRANSPORT_ABOVE_TARGET onward) is
new to this phase.

Full state machine: RESET -> PREGRASP -> SETTLE_PREGRASP -> APPROACH ->
SETTLE_APPROACH -> CLOSE -> VERIFY_BILATERAL_CONTACT -> LIFT -> HOLD ->
TRANSPORT_ABOVE_TARGET -> SETTLE_ABOVE_TARGET -> LOWER_TO_TARGET ->
SETTLE_LOWER -> OPEN -> VERIFY_RELEASE -> RETREAT -> VERIFY_TASK_SUCCESS ->
DONE/FAILED.

Initialization boundary: identical rule and mechanism as Phase 3/3B/3C
(CubeInitGuard, imported unchanged) -- cube qpos/qvel settable only before
the trial's first mj_step, hard-raises after. This module's own
run_trial_pick_place is source-scanned for the same invariant, independent
of the other phases' self-audits.

Fixed-base, torso-constrained upper-body manipulation baseline (unchanged
scope decision, HANDOFF.md Phase 4A) -- the pelvis and torso remain rigidly
welded to the world in this phase's scene (write_grasp_scene_4b, itself
built on the same _build_grasp_tree(extra_trunk_weld=True) as Phase 3C).
"""

from __future__ import annotations

import inspect
import json
import re
from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import (
    GRIPPER_ACTUATORS,
    GRIPPER_JOINTS,
    GRIPPER_MAX_QVEL,
    GRIPPER_MAX_STEP,
    RIGHT_ARM_ACTUATORS,
    RIGHT_ARM_JOINTS,
    TCP_SITE,
    JointMap,
    bounded_pd_step,
)
from tasks.g1_pick_place.controller_3c import (
    IK_POS_TOL,
    ORIENT_TOL_RAD,
    orientation_residual_rad,
    solve_ik_waypoint,
    solve_ik_waypoint_oriented,
)
from tasks.g1_pick_place.gripper_scene import (
    CUBE_HALF,
    CUBE_POS,
    FINGER_PAD_HALF,
    TABLE_TOP_Z,
    TARGET_HALF_XY,
    TARGET_HALF_Z,
    TARGET_POS,
    TARGET_RELEASE_Z,
    TARGET_XY_SUCCESS_MARGIN_M,
    write_grasp_scene_4b,
)
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard, _contacts_between
from tasks.g1_pick_place.run_grasp_test_3c import (
    CONTACT_MARGIN,
    DRIVE_S,
    GRASP_CORRIDOR_XY_M,
    GRIPPER_KD_3C,
    GRIPPER_KP_3C,
    LIFT_DZ,
    PREGRASP_DZ,
    SETTLE_ARM_QVEL_TOL,
    SETTLE_MAX_EXTRA_S,
    SETTLE_TCP_POS_TOL,
    _finger_targets,
    diagnose_reachability,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"

TIMESTEP = 0.002

# --- New waypoints for this phase (grasp-phase waypoints are unchanged,
# imported from run_grasp_test_3c above) -----------------------------------
RETREAT_DZ = 0.15  # higher than LIFT_DZ (0.12): distinct clearance waypoint,
# not merely "back to the same hover height", per HANDOFF.md's requirement
# that RETREAT get its own reachability analysis.

TARGET_XY = TARGET_POS
TRANSPORT_ABOVE_TARGET_POS = np.array([TARGET_XY[0], TARGET_XY[1], TARGET_RELEASE_Z + LIFT_DZ])
LOWER_TO_TARGET_POS = np.array([TARGET_XY[0], TARGET_XY[1], TARGET_RELEASE_Z])
RETREAT_POS = np.array([TARGET_XY[0], TARGET_XY[1], TARGET_RELEASE_Z + RETREAT_DZ])

# Stage A tuning budget (<=3 evidence-driven attempts, transport/lower/
# release trajectory parameters ONLY -- values below are the final, Attempt-3
# configuration; see reports/phase4b-task1-pick-place.md for the full
# attempt log, including the two prior configurations that failed).
#
# Attempt 1 (1.2 s / 1.0 s, single-shot segments, no waypoints): failed at
# TRANSPORT_ABOVE_TARGET -- a one-shot position-servo step across the
# ~0.13 m lateral move produced a large initial tracking error and a hard
# acceleration transient that broke bilateral grip contact.
#
# Attempt 2 (1.2 s / 1.0 s, 20/8 linearly-ramped Cartesian sub-waypoints):
# fixed TRANSPORT_ABOVE_TARGET, but LOWER_TO_TARGET (also a large-enough
# move at this off-nominal arm posture) still intermittently lost contact
# right at the end of the descent.
#
# Attempt 3 (this configuration): more sub-waypoints and more time for both
# segments. Evidence: with Attempt 2's parameters, bilateral contact was
# maintained through 100% of TRANSPORT_ABOVE_TARGET but was lost during the
# final ~15% of LOWER_TO_TARGET; slower/finer motion for both segments
# eliminated bilateral contact loss entirely (0/0 lost-contact steps) and
# the nominal trial passes deterministically (5/5 identical reruns, see
# tests/test_phase4b_pick_place.py). No gripper gain, arm servo gain, or
# grasp-approach parameter was changed at any point in this budget.
TRANSPORT_DRIVE_S = 2.0
LOWER_DRIVE_S = 2.0
RETREAT_DRIVE_S = 0.8  # unchanged from Attempt 1: RETREAT happens with the
# gripper already open (post-release), so grip-stability is not a concern
# for this segment and it was never implicated in any attempt's failure.
TRANSPORT_N_WAYPOINTS = 40
LOWER_N_WAYPOINTS = 60

# Phase 4E, Attempt 2 (Section B, evidence-based): LIFT was still using
# Stage A Attempt 1's one-shot _drive_segment (a single position-servo step
# straight to the post-lift joint target) -- never updated when Attempt 3
# proved this pattern unsafe for TRANSPORT/LOWER, because at the time LIFT
# itself was not implicated (RESET..HOLD was inherited unchanged from Phase
# 3C, "not a reimplementation with new numbers"). Direct evidence from
# tasks/g1_pick_place/phase4e_diagnose_grip.py against the pre-4E scene:
# the worst-instant bilateral contact-force safety factor (measured force /
# N_min = m*g/(2*mu)) during LIFT was 0.146x -- i.e. actual grip force
# momentarily dropped BELOW the theoretical minimum needed to support the
# cube's weight by friction alone, a real, physically-caused slip event,
# not a measurement artifact (CLOSE: 0.33x, HOLD: 1.07x -- LIFT's one-shot
# vertical step is uniquely bad). Smoothing LIFT the same way Stage A
# Attempt 3 already fixed TRANSPORT/LOWER (ramped sub-waypoints instead of
# a single jump) removes the acceleration transient without touching any
# gain.
LIFT_DRIVE_S_4E = 1.5
LIFT_N_WAYPOINTS_4E = 30

# Phase 4E, Attempt 2 (Section B): raised from Phase 3C's GRIPPER_KP_3C/KD_3C
# (150/10) -- HOLD's worst-instant safety factor at those gains was only
# 1.07x (see reports/phase4e-gripper-integrity-repair.md, Attempt 1 table),
# far below a defensible margin for a "safety factor". Chosen value and its
# resulting safety factor are documented as measured in that report, not
# asserted here.
GRIPPER_KP_4E = 320.0
GRIPPER_KD_4E = 20.0

# --- Objective task-success detector parameters ----------------------------
# Cube treated as "at rest" once its linear/angular speed drop below these --
# chosen as a tight-but-not-numerically-fragile bound: the cube's own contact
# solver reference (solref "0.002 1") settles within a few ms once resting on
# a rigid support, so any lingering speed above these values after the
# dwell window indicates real residual motion (sliding/rocking), not just
# solver noise.
CUBE_LINEAR_SPEED_TOL = 0.02  # m/s
CUBE_ANGULAR_SPEED_TOL = 0.05  # rad/s
# Height tolerance for "resting on the target pad, not still falling/settling
# through it": pad top + cube half, +/- this band.
CUBE_SUPPORTED_HEIGHT_TOL = 0.01  # m
# Cube must not move more than this during RETREAT (arm withdrawal) for the
# retreat to count as "without disturbing the cube" -- tight relative to the
# cube's own 0.07 m footprint.
RETREAT_DISTURBANCE_TOL_M = 0.005
# All task-success conditions must hold *continuously* for this long (not
# merely be true once) before DONE is declared -- HANDOFF.md: "Do not count
# success immediately when the cube first crosses the target boundary."
TASK_SUCCESS_DWELL_S = 0.5
TASK_SUCCESS_MAX_WAIT_S = 1.5  # same budget as the grasp-phase SETTLE_MAX_EXTRA_S


# --- Phase 4C: slip-metric math, factored out as pure functions so the
# frame transform itself can be unit-tested against synthetic rigid-body
# motions independent of any simulation trial (tests/test_phase4c_slip_audit.py).
def tcp_local_cube_offset(tcp_pos: np.ndarray, tcp_rot: np.ndarray, cube_pos: np.ndarray) -> np.ndarray:
    """Cube position expressed in the TCP's own (rotating) frame:
    R_tcp^T @ (cube_pos - tcp_pos). Invariant under any rigid-body motion of
    the TCP (translation and/or rotation) that leaves the cube fixed
    relative to the gripper -- only genuine relative displacement between
    the cube and the closed fingers changes this value.
    """
    return tcp_rot.T @ (np.asarray(cube_pos) - np.asarray(tcp_pos))


def relative_slip_m(local_offset_now: np.ndarray, grasp_reference_offset: np.ndarray) -> float:
    """Magnitude of the change in the TCP-local cube offset since the grasp
    reference was captured. Zero iff the cube has not moved relative to the
    closed fingers since grasp verification -- this is "slip", not the raw
    offset itself and not any post-release separation.
    """
    return float(np.linalg.norm(np.asarray(local_offset_now) - np.asarray(grasp_reference_offset)))


def _solve_waypoint(
    model: mujoco.MjModel,
    scratch: mujoco.MjData,
    base_qpos: np.ndarray,
    arm_map: JointMap,
    site_id: int,
    target: np.ndarray,
    nominal_q: np.ndarray,
    use_oriented_ik: bool,
) -> tuple[np.ndarray, float, int, float]:
    """Phase 4F: dispatch between the plain (Phase 3C, position-only) and
    oriented (Phase 4F, Attempt 1) waypoint solvers, always returning a
    uniform 4-tuple. use_oriented_ik=False (the default for every existing
    caller of run_trial_pick_place/diagnose_pick_place_reachability, i.e.
    every Phase 4B/4C/4D/4E test) reproduces the exact pre-Phase-4F code
    path and numeric outputs -- the orientation residual is still computed
    and returned for telemetry in that case (a read-only measurement of the
    resulting q, not a control input), so nothing about the ACTUAL solved
    joint target changes. use_oriented_ik=True is Phase 4F's own opt-in
    path, used only by its own evidence harness and tests.
    """
    if use_oriented_ik:
        return solve_ik_waypoint_oriented(model, scratch, base_qpos, arm_map, site_id, target, nominal_q)
    q, resid, iters = solve_ik_waypoint(model, scratch, base_qpos, arm_map, site_id, target, nominal_q)
    scratch.qpos[:] = base_qpos
    arm_map.set_qpos(scratch, q)
    mujoco.mj_kinematics(model, scratch)
    orient_resid = orientation_residual_rad(scratch.site_xmat[site_id])
    return q, resid, iters, orient_resid


def diagnose_pick_place_reachability(
    model: mujoco.MjModel,
    arm_map: JointMap,
    site_id: int,
    base_qpos: np.ndarray,
    cube_pos: np.ndarray,
    target_xy: tuple[float, float] = TARGET_XY,
) -> dict:
    """Pre-simulation reachability analysis for the full pick-and-place
    chain: the 4 grasp waypoints (identical to run_grasp_test_3c's
    diagnose_reachability) plus TRANSPORT_ABOVE_TARGET, LOWER_TO_TARGET, and
    RETREAT, each solved warm-started from the previous waypoint's solution
    (as the real trial moves through them continuously). Recorded for every
    waypoint regardless of outcome.
    """
    scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(arm_map.names))
    waypoints = {
        "PREGRASP": cube_pos + np.array([0.0, 0.0, PREGRASP_DZ]),
        "APPROACH": cube_pos.copy(),
        "CLOSED_LIFT": cube_pos + np.array([0.0, 0.0, LIFT_DZ]),
        "HOLD": cube_pos + np.array([0.0, 0.0, LIFT_DZ]),
        "TRANSPORT_ABOVE_TARGET": np.array([target_xy[0], target_xy[1], TARGET_RELEASE_Z + LIFT_DZ]),
        "LOWER_TO_TARGET": np.array([target_xy[0], target_xy[1], TARGET_RELEASE_Z]),
        "RETREAT": np.array([target_xy[0], target_xy[1], TARGET_RELEASE_Z + RETREAT_DZ]),
    }
    report = {}
    q_prev = base_qpos.copy()
    for name, target in waypoints.items():
        q, resid, iters, orient_resid = solve_ik_waypoint_oriented(model, scratch, q_prev, arm_map, site_id, target, nominal_q)
        reachable = resid < IK_POS_TOL
        oriented_ok = orient_resid < ORIENT_TOL_RAD
        report[name] = {
            "target_pos": target.tolist(),
            "solved_joint_target": q.tolist(),
            "residual_m": resid,
            "orientation_residual_rad": orient_resid,
            "orientation_residual_deg": float(np.degrees(orient_resid)),
            "iterations": iters,
            "reachable_within_tol": bool(reachable),
            "orientation_within_tol": bool(oriented_ok),
        }
        scratch.qpos[:] = q_prev
        arm_map.set_qpos(scratch, q)
        q_prev = scratch.qpos.copy()
    report["all_reachable"] = all(v["reachable_within_tol"] for v in report.values() if isinstance(v, dict))
    # Phase 4F: a stricter, additive reachability signal (kept separate from
    # the pre-existing "all_reachable" above, which several Phase 4B tests
    # already assert position-only -- not redefined here, to avoid silently
    # changing what an existing, passing assertion means).
    report["all_position_and_orientation_reachable"] = all(
        v["reachable_within_tol"] and v["orientation_within_tol"] for v in report.values() if isinstance(v, dict)
    )
    return report


def run_trial_pick_place(
    model_path: Path,
    cube_xy_offset: tuple[float, float] = (0.0, 0.0),
    diagnostics: dict | None = None,
    gripper_kp: float = GRIPPER_KP_4E,
    gripper_kd: float = GRIPPER_KD_4E,
    transport_drive_s: float = TRANSPORT_DRIVE_S,
    lower_drive_s: float = LOWER_DRIVE_S,
    retreat_drive_s: float = RETREAT_DRIVE_S,
    lift_drive_s: float = LIFT_DRIVE_S_4E,
    lift_n_waypoints: int = LIFT_N_WAYPOINTS_4E,
    frame_callback: "Callable[[str, mujoco.MjModel, mujoco.MjData], None] | None" = None,
    use_oriented_ik: bool = False,
    cube_body_name: str = "cube",
    cube_geom_name: str = "cube_geom",
    cube_joint_name: str = "cube_joint",
    distractor: dict | None = None,
) -> dict:
    """One Task 1 pick-and-place trial through the full state machine.

    `frame_callback`, if given, is invoked as `frame_callback(phase, model,
    data)` after every physics step -- purely for Phase 4C video/still
    capture (tasks/g1_pick_place/record_nominal_episode.py). It has no
    effect on control, physics, or any pass/fail decision; default is None,
    which reproduces the exact pre-4C code path unchanged.

    Returns a result dict with `criteria_grasp` (the same 5 keys Phase 3C
    used: both_pads_contact_cube, height_gain_ge_0_08m,
    lifted_ge_2s_continuous, finite_and_bounded, released_after_open) and
    `criteria_placement` (the remaining task-specific conditions:
    cube_in_target_xy, cube_supported_not_held, cube_linear_speed_ok,
    cube_angular_speed_ok, held_continuously_full_dwell,
    retreated_without_disturbing_cube) -- grasp success and placement
    success are reported separately, and `task_pass` requires both groups.

    `cube_body_name`/`cube_geom_name`/`cube_joint_name` (Task 2, Phase 6.1):
    identify which body/geom/joint in `model_path`'s scene is the cube this
    trial grasps and places -- default to the literal Task-1 names ("cube"/
    "cube_geom"/"cube_joint"), so every existing caller (every Phase 4B-5E
    call site) is byte-for-byte unaffected. Passing a different name lets
    the same, otherwise-unmodified controller act on a *different* object in
    a scene that contains more than one (e.g. Task 2's second, green cube),
    with waypoints still computed from that object's own live pose, never a
    hardcoded position.

    `distractor` (Task 2, Phase 6.1), if given, is
    `{"body_name": str, "geom_name": str, "joint_name": str,
    "xy_offset": (float, float)}` for a second object present in the same
    scene but never targeted by this trial. It is initialized once at RESET
    (through its own `CubeInitGuard`, identical physical-integrity rule as
    the primary cube -- no weld/teleport/direct write after the first
    physics step) and its live pose is tracked every step purely for
    telemetry (`distractor_max_displacement_m`, `distractor_final_xy`,
    `distractor_in_target_xy`) -- it never affects control, IK targets, or
    any pass/fail decision computed in this function. Default `None`
    reproduces the exact pre-Task-2 code path (no second body looked up, no
    extra guard, telemetry keys simply absent).
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    gripper_map = JointMap.build(model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    ik_scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(RIGHT_ARM_JOINTS))

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cube_body_name)
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, cube_geom_name)
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, cube_joint_name)
    cube_qpos_adr = int(model.jnt_qposadr[cube_joint_id])
    cube_dof_adr = int(model.jnt_dofadr[cube_joint_id])

    distractor_body_id = distractor_qpos_adr = distractor_dof_adr = None
    distractor_guard = None
    distractor_state = {"max_displacement_m": 0.0, "initial_xy": None}
    if distractor is not None:
        distractor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, distractor["body_name"])
        distractor_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, distractor["joint_name"])
        distractor_qpos_adr = int(model.jnt_qposadr[distractor_joint_id])
        distractor_dof_adr = int(model.jnt_dofadr[distractor_joint_id])

    # --- RESET ---
    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    guard = CubeInitGuard(data, cube_qpos_adr, cube_dof_adr)
    guard.set_initial_pose([cube_x, cube_y, cube_z])
    if distractor is not None:
        d_x = CUBE_POS[0] + distractor["xy_offset"][0]
        d_y = CUBE_POS[1] + distractor["xy_offset"][1]
        d_z = CUBE_POS[2]
        distractor_guard = CubeInitGuard(data, distractor_qpos_adr, distractor_dof_adr)
        distractor_guard.set_initial_pose([d_x, d_y, d_z])
        distractor_state["initial_xy"] = [d_x, d_y]
    mujoco.mj_forward(model, data)
    steps_run = [0]

    rest_z = float(data.xpos[cube_body_id][2])
    cube_pos = np.array([cube_x, cube_y, cube_z])
    pregrasp_target = cube_pos + np.array([0.0, 0.0, PREGRASP_DZ])
    lift_target = cube_pos + np.array([0.0, 0.0, LIFT_DZ])

    reachability = diagnose_pick_place_reachability(model, arm_map, site_id, data.qpos.copy(), cube_pos)

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
        "arm_tracking_error_rms": {},
        "arm_tracking_error_max": {},
        "settle_extra_s": {},
        "contact_lost_during_transport": False,
        "height_unsafe_during_transport": False,
        "max_cube_slip_m": 0.0,
        "grasp_reference_offset_tcp_frame": None,
        "max_slip_during_lift": 0.0,
        "max_slip_during_transport": 0.0,
        "max_slip_during_lower": 0.0,
        "slip_at_release": None,
        "post_release_tcp_cube_separation_m": 0.0,
        "post_lower_cube_xy": None,
        "post_release_cube_pose": None,
        "final_cube_xy": None,
        "final_cube_z": None,
        "final_cube_linear_speed": None,
        "final_cube_angular_speed": None,
        "retreat_disturbance_m": None,
        "task_success_dwell_achieved_s": 0.0,
        "downward_slip_during_hold_m": 0.0,
        "min_bilateral_normal_force_n": float("inf"),
        "max_abs_contact_z_offset_from_cube_center_m": 0.0,
        # Phase 4F additions.
        "orientation_residual_rad": {},
        "lateral_centering_error_at_approach_m": None,
        "orientation_residual_at_approach_deg": None,
        "downward_slip_during_transport_m": 0.0,
        "opposing_face_contact_left": False,
        "opposing_face_contact_right": False,
    }
    lifted_since = None
    # Cube offset from the TCP, expressed in the TCP's OWN (rotating) frame at
    # the moment of grasp verification -- not a raw world-frame vector. The
    # wrist rotates by double-digit degrees over the course of a large
    # transport move (a purely kinematic side effect of position-only IK's
    # redundancy resolution, not a defect); a world-frame offset difference
    # would count that whole-body rotation as "slip" even with zero actual
    # relative motion between the cube and the closed fingers. Comparing in
    # the TCP's local frame isolates genuine relative displacement.
    grasp_offset_ref = {"value": None}

    def _step_once(arm_ctrl_target: np.ndarray, finger_open: bool, phase: str, carrying: str | None = None) -> None:
        nonlocal lifted_since
        arm_map.set_ctrl(data, arm_ctrl_target)
        bounded_pd_step(
            gripper_map, data, _finger_targets(gripper_map, finger_open),
            gripper_kp, gripper_kd, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL,
        )
        if not np.all(np.isfinite(data.ctrl)):
            telemetry["finite_and_bounded"] = False
        mujoco.mj_step(model, data)
        if frame_callback is not None:
            frame_callback(phase, model, data)
        steps_run[0] += 1
        if steps_run[0] == 1:
            guard.lock()
            if distractor_guard is not None:
                distractor_guard.lock()

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            telemetry["finite_and_bounded"] = False

        if distractor_body_id is not None:
            d_xy_now = data.xpos[distractor_body_id][:2].copy()
            d_disp = float(np.linalg.norm(d_xy_now - np.array(distractor_state["initial_xy"])))
            distractor_state["max_displacement_m"] = max(distractor_state["max_displacement_m"], d_disp)
            distractor_state["final_xy"] = d_xy_now.tolist()
            distractor_state["final_z"] = float(data.xpos[distractor_body_id][2])

        cube_xyz = data.xpos[cube_body_id].copy()
        cube_z_now = float(cube_xyz[2])
        telemetry["max_cube_z"] = max(telemetry["max_cube_z"], cube_z_now)
        tcp_rot = data.site_xmat[site_id].reshape(3, 3)

        left_contact = _contacts_between(data, model, cube_geom_id, left_pad_id)
        right_contact = _contacts_between(data, model, cube_geom_id, right_pad_id)
        telemetry["left_contact_ever"] = telemetry["left_contact_ever"] or left_contact
        telemetry["right_contact_ever"] = telemetry["right_contact_ever"] or right_contact
        both_now = left_contact and right_contact
        if both_now:
            telemetry["both_pads_contact_cube"] = True

        if carrying in ("full", "grip_only") and not both_now:
            telemetry["contact_lost_during_transport"] = True

        # Phase 4E Section C evidence: while the cube is actually grasped,
        # record (a) the minimum instantaneous bilateral normal force seen
        # (not merely a contact-exists boolean) and (b) how far the contact
        # point strays, in world Z, from the cube's own center -- both used
        # to check "cube center remains inside the vertical overlap region
        # of both finger pads" and "normal contact forces remain positive
        # and finite" directly against real per-step contact data.
        if carrying in ("full", "grip_only"):
            for pad_id in (left_pad_id, right_pad_id):
                for ci in range(data.ncon):
                    con = data.contact[ci]
                    pair = (int(con.geom1), int(con.geom2))
                    if cube_geom_id not in pair or pad_id not in pair:
                        continue
                    force6 = np.zeros(6)
                    mujoco.mj_contactForce(model, data, ci, force6)
                    normal_n = float(abs(force6[0]))
                    telemetry["min_bilateral_normal_force_n"] = min(
                        telemetry["min_bilateral_normal_force_n"], normal_n
                    )
                    z_offset = abs(float(con.pos[2]) - cube_z_now)
                    telemetry["max_abs_contact_z_offset_from_cube_center_m"] = max(
                        telemetry["max_abs_contact_z_offset_from_cube_center_m"], z_offset
                    )
                    # Phase 4F Section C, "both fingers contact opposing
                    # cube side faces": a contact whose normal is
                    # (anti)parallel to the wrist's own jaw-closing axis
                    # (local Y, the axis the two fingers slide along -- see
                    # gripper_scene.py's FINGER_REACH_X/y_ref) is a genuine
                    # side-face contact; one whose normal points mostly
                    # along the jaw's local Z or X would mean a pad is
                    # instead catching a top/bottom/front edge or corner.
                    # con.frame's first row is the contact normal in world
                    # frame (MuJoCo convention); tcp_rot's second column is
                    # the wrist's local Y axis in world frame.
                    contact_normal_world = np.array(con.frame[0:3])
                    jaw_axis_world = tcp_rot[:, 1]
                    side_face_alignment = abs(float(np.dot(contact_normal_world, jaw_axis_world)))
                    if side_face_alignment > 0.7:
                        if pad_id == left_pad_id:
                            telemetry["opposing_face_contact_left"] = True
                        else:
                            telemetry["opposing_face_contact_right"] = True

        if carrying == "full" and cube_z_now < (rest_z + CONTACT_MARGIN):
            telemetry["height_unsafe_during_transport"] = True

        if grasp_offset_ref["value"] is not None:
            tcp_pos = data.site_xpos[site_id]
            tcp_rot = data.site_xmat[site_id].reshape(3, 3)
            local_offset_now = tcp_local_cube_offset(tcp_pos, tcp_rot, cube_xyz)
            slip = relative_slip_m(local_offset_now, grasp_offset_ref["value"])
            # Phase 4C audit finding: the pre-4C code updated max_cube_slip_m
            # on every step once grasp_offset_ref was set, with no upper
            # bound -- it never stopped once OPEN/RELEASE_SETTLE/
            # VERIFY_RELEASE/RETREAT/VERIFY_TASK_SUCCESS began, i.e. long
            # after the cube was intentionally released and physically
            # separated from the closed fingers. That post-release TCP-cube
            # separation (large and expected -- the cube falls away from an
            # open, retreating gripper) was silently included in "grasp
            # slip", inflating it far past any real relative motion inside a
            # closed grip. Retained here unmodified as a legacy field (never
            # reported as the authoritative number going forward -- see
            # reports/phase4c-task1-evidence.md) so its old value stays
            # reproducible for the correction/addendum.
            telemetry["max_cube_slip_m"] = max(telemetry["max_cube_slip_m"], slip)
            if carrying in ("grip_only", "full"):
                # Only while the gripper is still commanded closed and the
                # cube is physically grasped -- this is genuine slip.
                if phase == "HOLD" or phase.startswith("LIFT"):
                    telemetry["max_slip_during_lift"] = max(telemetry["max_slip_during_lift"], slip)
                elif phase.startswith("TRANSPORT_ABOVE_TARGET") or phase == "SETTLE_ABOVE_TARGET":
                    telemetry["max_slip_during_transport"] = max(telemetry["max_slip_during_transport"], slip)
                elif phase.startswith("LOWER_TO_TARGET") or phase == "SETTLE_LOWER":
                    telemetry["max_slip_during_lower"] = max(telemetry["max_slip_during_lower"], slip)
                    # Overwritten every held step in this bucket, so its
                    # final value is the slip at the last instant the cube
                    # was still grasped -- immediately before OPEN.
                    telemetry["slip_at_release"] = slip
            else:
                # carrying is None here (OPEN/RELEASE_SETTLE/VERIFY_RELEASE/
                # RETREAT/VERIFY_TASK_SUCCESS): the gripper is opening or
                # already open and the cube is not held. This is TCP-cube
                # separation after release, not slip -- reported under its
                # own name and never folded into a "grasp slip" metric.
                separation = float(np.linalg.norm(cube_xyz - tcp_pos))
                telemetry["post_release_tcp_cube_separation_m"] = max(
                    telemetry["post_release_tcp_cube_separation_m"], separation
                )

        is_lifted = cube_z_now > (rest_z + CONTACT_MARGIN)
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
                phase, {"left_contact": [], "right_contact": [], "cube_z": []}
            )
            bucket["left_contact"].append(bool(left_contact))
            bucket["right_contact"].append(bool(right_contact))
            bucket["cube_z"].append(cube_z_now)

    def _drive_segment(
        target_pos: np.ndarray, finger_open: bool, phase: str, duration_s: float,
        joint_target: np.ndarray, carrying: str | None = None,
    ) -> None:
        n = max(1, int(round(duration_s / TIMESTEP)))
        qpos_errs = []
        for _ in range(n):
            _step_once(joint_target, finger_open, phase, carrying=carrying)
            qpos_errs.append(np.abs(joint_target - arm_map.get_qpos(data)))
        qpos_errs = np.array(qpos_errs)
        telemetry["arm_tracking_error_rms"][phase] = float(np.sqrt(np.mean(qpos_errs ** 2)))
        telemetry["arm_tracking_error_max"][phase] = float(np.max(qpos_errs))

    def _drive_smooth(
        target_pos: np.ndarray, finger_open: bool, phase: str, total_duration_s: float,
        n_waypoints: int, carrying: str | None = None,
    ) -> np.ndarray:
        """Drive through `n_waypoints` linearly-interpolated Cartesian
        sub-waypoints from the current TCP position to `target_pos`, each
        solved once via solve_ik_waypoint (chained from the previous
        waypoint's solution). Unlike _drive_segment's fixed set-point (fine
        for the grasp phase's small moves), the commanded *joint* target is
        itself ramped linearly, step by step, from the previous sub-
        waypoint's solution to the new one -- Stage A Attempt 2 evidence
        (reports/phase4b-task1-pick-place.md) showed that jumping straight
        to each new sub-waypoint's solved joint target (a position-servo
        step input) produced small jerks that, repeated over ~20
        sub-waypoints, accumulated several cm of cube slip inside the
        closed gripper -- a real physical effect, not fixed by adding more
        sub-waypoints alone. Ramping the reference removes the step input
        without touching any gain. Returns the final sub-waypoint's joint
        target (for a subsequent _settle() call against the true final
        target).
        """
        start_pos = data.site_xpos[site_id].copy()
        q_prev_target = arm_map.get_qpos(data).copy()
        seg_duration = total_duration_s / n_waypoints
        n_substeps = max(1, int(round(seg_duration / TIMESTEP)))
        q_final = None
        for i in range(n_waypoints):
            alpha = (i + 1) / n_waypoints
            waypoint = start_pos + alpha * (target_pos - start_pos)
            q_i, _, _, _ = _solve_waypoint(
                model, ik_scratch, data.qpos.copy(), arm_map, site_id, waypoint, nominal_q, use_oriented_ik
            )
            qpos_errs = []
            for s in range(n_substeps):
                beta = (s + 1) / n_substeps
                ramped_target = q_prev_target + beta * (q_i - q_prev_target)
                _step_once(ramped_target, finger_open, f"{phase}_wp{i}", carrying=carrying)
                qpos_errs.append(np.abs(ramped_target - arm_map.get_qpos(data)))
            qpos_errs = np.array(qpos_errs)
            telemetry["arm_tracking_error_rms"][f"{phase}_wp{i}"] = float(np.sqrt(np.mean(qpos_errs ** 2)))
            telemetry["arm_tracking_error_max"][f"{phase}_wp{i}"] = float(np.max(qpos_errs))
            q_prev_target = q_i
            q_final = q_i
        return q_final

    def _settle(
        target_pos: np.ndarray, finger_open: bool, phase: str, joint_target: np.ndarray,
        carrying: str | None = None,
    ) -> bool:
        max_steps = int(round(SETTLE_MAX_EXTRA_S / TIMESTEP))
        settled = False
        n_steps = 0
        for _ in range(max_steps):
            _step_once(joint_target, finger_open, phase, carrying=carrying)
            n_steps += 1
            tcp_pos = data.site_xpos[site_id]
            pos_err = float(np.linalg.norm(target_pos - tcp_pos))
            joint_speed = float(np.max(np.abs(arm_map.get_qvel(data))))
            if pos_err <= SETTLE_TCP_POS_TOL and joint_speed <= SETTLE_ARM_QVEL_TOL:
                settled = True
                break
        telemetry["settle_extra_s"][phase] = n_steps * TIMESTEP
        return settled

    def _task_success_dwell(joint_target: np.ndarray) -> bool:
        """Hold the RETREAT arm pose and repeatedly evaluate every
        task-success condition; the dwell only counts as achieved once all
        conditions have held *continuously* (not merely once) for
        TASK_SUCCESS_DWELL_S. Any single-step failure resets the streak.
        """
        max_steps = int(round(TASK_SUCCESS_MAX_WAIT_S / TIMESTEP))
        need_steps = int(round(TASK_SUCCESS_DWELL_S / TIMESTEP))
        streak = 0
        n_steps = 0
        post_lower_xy = np.array(telemetry["post_lower_cube_xy"])
        for _ in range(max_steps):
            _step_once(joint_target, True, "VERIFY_TASK_SUCCESS", carrying=None)
            n_steps += 1

            cube_xyz = data.xpos[cube_body_id].copy()
            cube_vel = data.qvel[cube_dof_adr : cube_dof_adr + 6].copy()
            lin_speed = float(np.linalg.norm(cube_vel[:3]))
            ang_speed = float(np.linalg.norm(cube_vel[3:]))
            xy_err = float(np.linalg.norm(cube_xyz[:2] - np.array(TARGET_XY)))
            supported_not_held = bool(
                abs(float(cube_xyz[2]) - TARGET_RELEASE_Z) <= CUBE_SUPPORTED_HEIGHT_TOL
                and not _contacts_between(data, model, cube_geom_id, left_pad_id)
                and not _contacts_between(data, model, cube_geom_id, right_pad_id)
            )
            retreat_disturbance = float(np.linalg.norm(cube_xyz[:2] - post_lower_xy))

            telemetry["final_cube_xy"] = cube_xyz[:2].tolist()
            telemetry["final_cube_z"] = float(cube_xyz[2])
            telemetry["final_cube_linear_speed"] = lin_speed
            telemetry["final_cube_angular_speed"] = ang_speed
            telemetry["retreat_disturbance_m"] = retreat_disturbance

            ok = (
                xy_err <= TARGET_XY_SUCCESS_MARGIN_M
                and supported_not_held
                and lin_speed <= CUBE_LINEAR_SPEED_TOL
                and ang_speed <= CUBE_ANGULAR_SPEED_TOL
                and retreat_disturbance <= RETREAT_DISTURBANCE_TOL_M
            )
            if ok:
                streak += 1
            else:
                streak = 0
            if streak >= need_steps:
                telemetry["task_success_dwell_achieved_s"] = streak * TIMESTEP
                return True
        telemetry["task_success_dwell_achieved_s"] = streak * TIMESTEP
        return False

    def _fail(state: str, reason: str) -> dict:
        telemetry["failure_state"] = state
        telemetry["failure_reason"] = reason
        return _finalize(task_success=False)

    def _finalize(task_success: bool) -> dict:
        height_gain = telemetry["max_cube_z"] - rest_z
        criteria_grasp = {
            "both_pads_contact_cube": telemetry["both_pads_contact_cube"],
            "height_gain_ge_0_08m": height_gain >= 0.08,
            "lifted_ge_2s_continuous": telemetry["max_continuous_lifted_s"] >= 2.0,
            "finite_and_bounded": telemetry["finite_and_bounded"],
            "released_after_open": bool(telemetry["released_after_open"]) if telemetry["released_after_open"] is not None else False,
        }
        xy_err_final = (
            float(np.linalg.norm(np.array(telemetry["final_cube_xy"]) - np.array(TARGET_XY)))
            if telemetry["final_cube_xy"] is not None else None
        )
        criteria_placement = {
            "cube_in_target_xy": bool(xy_err_final is not None and xy_err_final <= TARGET_XY_SUCCESS_MARGIN_M),
            "cube_supported_not_held": bool(
                telemetry["final_cube_z"] is not None
                and abs(telemetry["final_cube_z"] - TARGET_RELEASE_Z) <= CUBE_SUPPORTED_HEIGHT_TOL
            ),
            "cube_linear_speed_ok": bool(
                telemetry["final_cube_linear_speed"] is not None
                and telemetry["final_cube_linear_speed"] <= CUBE_LINEAR_SPEED_TOL
            ),
            "cube_angular_speed_ok": bool(
                telemetry["final_cube_angular_speed"] is not None
                and telemetry["final_cube_angular_speed"] <= CUBE_ANGULAR_SPEED_TOL
            ),
            "held_continuously_full_dwell": telemetry["task_success_dwell_achieved_s"] >= TASK_SUCCESS_DWELL_S,
            "retreated_without_disturbing_cube": bool(
                telemetry["retreat_disturbance_m"] is not None
                and telemetry["retreat_disturbance_m"] <= RETREAT_DISTURBANCE_TOL_M
            ),
            "no_transport_contact_loss": not telemetry["contact_lost_during_transport"],
            "no_transport_height_violation": not telemetry["height_unsafe_during_transport"],
        }
        grasp_pass = all(criteria_grasp.values())
        placement_pass = all(criteria_placement.values())

        # Phase 4E Section C: strengthened grasp-stability acceptance
        # criteria, evaluated in ADDITION to criteria_grasp above (which are
        # unchanged from Phase 3C). None of these can be satisfied
        # trivially by a trial that failed earlier (all the source
        # telemetry only accumulates while carrying is "grip_only"/"full",
        # i.e. after a real bilateral grasp was verified).
        max_slip_while_grasped_m = max(
            telemetry["max_slip_during_lift"],
            telemetry["max_slip_during_transport"],
            telemetry["max_slip_during_lower"],
        )
        criteria_grasp_stability_4e = {
            "max_slip_while_grasped_le_10mm": max_slip_while_grasped_m <= 0.010,
            "downward_slip_during_hold_le_5mm": telemetry["downward_slip_during_hold_m"] <= 0.005,
            "cube_center_within_pad_vertical_overlap": (
                telemetry["max_abs_contact_z_offset_from_cube_center_m"] <= FINGER_PAD_HALF[2]
            ),
            "bilateral_contact_throughout_hold": not telemetry["contact_lost_during_transport"],
            "normal_forces_positive_and_finite": bool(
                np.isfinite(telemetry["min_bilateral_normal_force_n"])
                and telemetry["min_bilateral_normal_force_n"] > 0.0
            ),
        }
        grasp_stability_pass_4e = all(criteria_grasp_stability_4e.values())

        # Phase 4F Section C: the 11 strengthened acceptance criteria the
        # user specified, evaluated in addition to (not replacing) criteria_
        # grasp/criteria_placement/criteria_grasp_stability_4e above. Several
        # are the same measured quantities re-checked against an unchanged
        # bar (max slip, downward slip during HOLD, vertical overlap,
        # contact-force positivity, height gain, hold duration, release/
        # placement, finite/bounded forces, no post-init cube manipulation
        # -- the last enforced structurally by CubeInitGuard/the source
        # self-audit below, not re-checked here); two are genuinely new
        # measurements this phase adds: both_fingers_on_opposing_faces (the
        # contact-normal/jaw-axis alignment check added to _step_once above)
        # and downward_slip_through_transport_le_10mm.
        criteria_grasp_stability_4f = {
            "both_fingers_on_opposing_faces": bool(
                telemetry["opposing_face_contact_left"] and telemetry["opposing_face_contact_right"]
            ),
            "bilateral_contact_persists_lift_and_hold": not telemetry["contact_lost_during_transport"],
            "max_slip_while_grasped_le_10mm": max_slip_while_grasped_m <= 0.010,
            "downward_slip_during_hold_le_5mm": telemetry["downward_slip_during_hold_m"] <= 0.005,
            "downward_slip_through_transport_le_10mm": telemetry["downward_slip_during_transport_m"] <= 0.010,
            "cube_center_within_pad_vertical_overlap": (
                telemetry["max_abs_contact_z_offset_from_cube_center_m"] <= FINGER_PAD_HALF[2]
            ),
            "height_gain_ge_0_08m": height_gain >= 0.08,
            "continuous_off_table_hold_ge_2s": telemetry["max_continuous_lifted_s"] >= 2.0,
            "physical_release_and_settled_placement": (
                bool(telemetry["released_after_open"]) if telemetry["released_after_open"] is not None else False
            ),
            "normal_forces_positive_and_finite": bool(
                np.isfinite(telemetry["min_bilateral_normal_force_n"])
                and telemetry["min_bilateral_normal_force_n"] > 0.0
            ),
            "no_cube_state_manipulation_after_init": True,  # structurally enforced: CubeInitGuard + the
            # module-level source self-audit below raise/fail import if violated, so a trial that
            # ran at all satisfies this by construction, not by a runtime measurement here.
        }
        grasp_stability_pass_4f = all(criteria_grasp_stability_4f.values())
        distractor_result = None
        if distractor is not None:
            d_final_xy = distractor_state.get("final_xy")
            distractor_result = {
                "initial_xy": distractor_state["initial_xy"],
                "final_xy": d_final_xy,
                "max_displacement_m": distractor_state["max_displacement_m"],
                "displacement_within_10mm": distractor_state["max_displacement_m"] <= 0.010,
                "in_target_xy": bool(
                    d_final_xy is not None
                    and float(np.linalg.norm(np.array(d_final_xy) - np.array(TARGET_XY))) <= TARGET_XY_SUCCESS_MARGIN_M
                ),
            }
        return {
            "cube_xy_offset": list(cube_xy_offset),
            "distractor": distractor_result,
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
            "settle_extra_s": telemetry["settle_extra_s"],
            "reachability": reachability,
            "max_cube_slip_m": telemetry["max_cube_slip_m"],
            "grasp_reference_offset_tcp_frame": telemetry["grasp_reference_offset_tcp_frame"],
            "max_slip_during_lift": telemetry["max_slip_during_lift"],
            "max_slip_during_transport": telemetry["max_slip_during_transport"],
            "max_slip_during_lower": telemetry["max_slip_during_lower"],
            "slip_at_release": telemetry["slip_at_release"],
            "post_release_tcp_cube_separation_m": telemetry["post_release_tcp_cube_separation_m"],
            "contact_lost_during_transport": telemetry["contact_lost_during_transport"],
            "height_unsafe_during_transport": telemetry["height_unsafe_during_transport"],
            "final_cube_xy": telemetry["final_cube_xy"],
            "final_cube_z": telemetry["final_cube_z"],
            "final_xy_target_error_m": xy_err_final,
            "final_cube_linear_speed": telemetry["final_cube_linear_speed"],
            "final_cube_angular_speed": telemetry["final_cube_angular_speed"],
            "retreat_disturbance_m": telemetry["retreat_disturbance_m"],
            "task_success_dwell_achieved_s": telemetry["task_success_dwell_achieved_s"],
            "criteria_grasp": criteria_grasp,
            "criteria_placement": criteria_placement,
            "grasp_pass": grasp_pass,
            "placement_pass": placement_pass and task_success,
            "task_pass": grasp_pass and placement_pass and task_success and telemetry["failure_state"] is None,
            "downward_slip_during_hold_m": telemetry["downward_slip_during_hold_m"],
            "min_bilateral_normal_force_n": (
                telemetry["min_bilateral_normal_force_n"]
                if np.isfinite(telemetry["min_bilateral_normal_force_n"]) else None
            ),
            "max_abs_contact_z_offset_from_cube_center_m": telemetry["max_abs_contact_z_offset_from_cube_center_m"],
            "max_slip_while_grasped_m": max_slip_while_grasped_m,
            "criteria_grasp_stability_4e": criteria_grasp_stability_4e,
            "grasp_stability_pass_4e": grasp_stability_pass_4e,
            "orientation_residual_rad": telemetry["orientation_residual_rad"],
            "orientation_residual_at_approach_deg": telemetry["orientation_residual_at_approach_deg"],
            "lateral_centering_error_at_approach_m": telemetry["lateral_centering_error_at_approach_m"],
            "downward_slip_during_transport_m": telemetry["downward_slip_during_transport_m"],
            "opposing_face_contact_left": telemetry["opposing_face_contact_left"],
            "opposing_face_contact_right": telemetry["opposing_face_contact_right"],
            "criteria_grasp_stability_4f": criteria_grasp_stability_4f,
            "grasp_stability_pass_4f": grasp_stability_pass_4f,
        }

    # --- PREGRASP ---
    telemetry["states_entered"].append("PREGRASP")
    q_target, _, _, orient_pregrasp = _solve_waypoint(
        model, ik_scratch, data.qpos.copy(), arm_map, site_id, pregrasp_target, nominal_q, use_oriented_ik
    )
    telemetry["orientation_residual_rad"]["PREGRASP"] = orient_pregrasp
    _drive_segment(pregrasp_target, True, "PREGRASP", DRIVE_S["PREGRASP"], q_target)

    telemetry["states_entered"].append("SETTLE_PREGRASP")
    if not _settle(pregrasp_target, True, "SETTLE_PREGRASP", q_target):
        return _fail("SETTLE_PREGRASP", "TCP did not settle within tolerance before APPROACH")

    # --- APPROACH ---
    telemetry["states_entered"].append("APPROACH")
    q_target2, _, _, orient_approach = _solve_waypoint(
        model, ik_scratch, data.qpos.copy(), arm_map, site_id, cube_pos, nominal_q, use_oriented_ik
    )
    telemetry["orientation_residual_rad"]["APPROACH"] = orient_approach
    _drive_segment(cube_pos, True, "APPROACH", DRIVE_S["APPROACH"], q_target2)

    telemetry["states_entered"].append("SETTLE_APPROACH")
    if not _settle(cube_pos, True, "SETTLE_APPROACH", q_target2):
        return _fail("SETTLE_APPROACH", "TCP did not settle within tolerance before CLOSE")

    # --- Phase 4F Section B: pre-CLOSE grasp-alignment measurement. Recorded
    # here (not folded into _settle's own tolerance, which stays unchanged
    # for PREGRASP/other callers) the same way Phase 4E's own strengthened
    # criteria (criteria_grasp_stability_4e) were added: measured and
    # reported as an explicit pass/fail criterion in _finalize below, not as
    # a new early-abort gate. reports/phase4f-orientation-grasp-
    # stabilization.md documents why: all 3 authorized repair attempts,
    # backed by real IK-residual sweeps (including a from-scratch co-primary
    # weighted solve, not just this null-space objective), found that
    # driving orient_approach below ORIENT_TOL_RAD at this exact Cartesian
    # point requires 30-70 mm of TCP position error -- i.e. missing the cube
    # entirely -- so a hard abort here would make every nominal trial fail
    # at SETTLE_APPROACH deterministically, before CLOSE is ever attempted,
    # which would hide rather than reveal the actual grasp behavior this
    # phase exists to evidence. The measured residual is reported honestly
    # as a failing criterion instead (criteria_grasp_stability_4f below).
    _approach_tcp_pos = data.site_xpos[site_id].copy()
    lateral_centering_error = float(np.linalg.norm(_approach_tcp_pos[:2] - cube_pos[:2]))
    telemetry["lateral_centering_error_at_approach_m"] = lateral_centering_error
    telemetry["orientation_residual_at_approach_deg"] = float(np.degrees(orient_approach))

    # --- CLOSE ---
    telemetry["states_entered"].append("CLOSE")
    _drive_segment(cube_pos, False, "CLOSE", DRIVE_S["CLOSE"], q_target2)

    # --- VERIFY_BILATERAL_CONTACT ---
    telemetry["states_entered"].append("VERIFY_BILATERAL_CONTACT")
    left_now = _contacts_between(data, model, cube_geom_id, left_pad_id)
    right_now = _contacts_between(data, model, cube_geom_id, right_pad_id)
    finger_qvel = gripper_map.get_qvel(data)
    closing_settled = bool(np.all(np.abs(finger_qvel) < 0.03))
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
    _tcp_rot0 = data.site_xmat[site_id].reshape(3, 3)
    grasp_offset_ref["value"] = tcp_local_cube_offset(
        data.site_xpos[site_id].copy(), _tcp_rot0, data.xpos[cube_body_id].copy()
    )
    telemetry["grasp_reference_offset_tcp_frame"] = grasp_offset_ref["value"].tolist()

    # --- LIFT --- ("grip_only": the cube is expected to be below the lift
    # height for most of this segment -- that is the point of LIFT, not a
    # safety violation. Height gain/hold-duration are already checked by
    # criteria_grasp below, reused unchanged from Phase 3C.)
    telemetry["states_entered"].append("LIFT")
    q_lift = _drive_smooth(lift_target, False, "LIFT", lift_drive_s, lift_n_waypoints, carrying="grip_only")
    if telemetry["contact_lost_during_transport"]:
        return _fail("LIFT", "lost bilateral contact during LIFT")

    # --- HOLD ---
    telemetry["states_entered"].append("HOLD")
    # Phase 4E Section C: "downward slip from the start to end of HOLD" --
    # the arm is commanded stationary throughout HOLD (lift_target does not
    # change), so any world-Z drop in the cube during this segment is,
    # unambiguously, the cube slipping down inside the closed grip, not a
    # TCP-frame-rotation artifact (there is no TCP rotation during HOLD).
    cube_z_hold_start = float(data.xpos[cube_body_id][2])
    _drive_segment(lift_target, False, "HOLD", DRIVE_S["HOLD"], q_lift, carrying="grip_only")
    cube_z_hold_end = float(data.xpos[cube_body_id][2])
    telemetry["downward_slip_during_hold_m"] = max(0.0, cube_z_hold_start - cube_z_hold_end)
    if telemetry["contact_lost_during_transport"]:
        return _fail("HOLD", "lost bilateral contact during HOLD")

    # --- TRANSPORT_ABOVE_TARGET (smooth multi-waypoint Cartesian move) ---
    telemetry["states_entered"].append("TRANSPORT_ABOVE_TARGET")
    cube_z_transport_start = float(data.xpos[cube_body_id][2])
    q_transport = _drive_smooth(
        TRANSPORT_ABOVE_TARGET_POS, False, "TRANSPORT_ABOVE_TARGET", transport_drive_s,
        TRANSPORT_N_WAYPOINTS, carrying="full",
    )
    if telemetry["contact_lost_during_transport"] or telemetry["height_unsafe_during_transport"]:
        return _fail("TRANSPORT_ABOVE_TARGET", "lost bilateral contact or unsafe height during transport")

    # --- SETTLE_ABOVE_TARGET ---
    telemetry["states_entered"].append("SETTLE_ABOVE_TARGET")
    if not _settle(TRANSPORT_ABOVE_TARGET_POS, False, "SETTLE_ABOVE_TARGET", q_transport, carrying="full"):
        return _fail("SETTLE_ABOVE_TARGET", "TCP did not settle above target before lowering")
    if telemetry["contact_lost_during_transport"] or telemetry["height_unsafe_during_transport"]:
        return _fail("SETTLE_ABOVE_TARGET", "lost bilateral contact or unsafe height while settling above target")
    # Phase 4F Section C, "vertical/downward slip through transport": the
    # cube's own net world-Z drop across TRANSPORT_ABOVE_TARGET + SETTLE_
    # ABOVE_TARGET (both "full"-carry segments; the arm is not intentionally
    # descending in either) -- any drop here is slip inside the grip, same
    # reasoning as HOLD's downward_slip_during_hold_m.
    cube_z_transport_end = float(data.xpos[cube_body_id][2])
    telemetry["downward_slip_during_transport_m"] = max(0.0, cube_z_transport_start - cube_z_transport_end)

    # --- LOWER_TO_TARGET (height decreasing intentionally: grip-only check;
    # smooth multi-waypoint descent, same rationale as TRANSPORT_ABOVE_TARGET) ---
    telemetry["states_entered"].append("LOWER_TO_TARGET")
    q_lower = _drive_smooth(
        LOWER_TO_TARGET_POS, False, "LOWER_TO_TARGET", lower_drive_s,
        LOWER_N_WAYPOINTS, carrying="grip_only",
    )
    if telemetry["contact_lost_during_transport"]:
        return _fail("LOWER_TO_TARGET", "lost bilateral contact before reaching target")

    # --- SETTLE_LOWER ---
    telemetry["states_entered"].append("SETTLE_LOWER")
    if not _settle(LOWER_TO_TARGET_POS, False, "SETTLE_LOWER", q_lower, carrying="grip_only"):
        return _fail("SETTLE_LOWER", "TCP did not settle at target before OPEN")
    if telemetry["contact_lost_during_transport"]:
        return _fail("SETTLE_LOWER", "lost bilateral contact while settling at target")

    telemetry["post_lower_cube_xy"] = [float(data.xpos[cube_body_id][0]), float(data.xpos[cube_body_id][1])]

    # --- OPEN ---
    telemetry["states_entered"].append("OPEN")
    _drive_segment(LOWER_TO_TARGET_POS, True, "OPEN", DRIVE_S["OPEN"], q_lower)
    _drive_segment(LOWER_TO_TARGET_POS, True, "RELEASE_SETTLE", DRIVE_S["RELEASE_SETTLE"], q_lower)

    # --- VERIFY_RELEASE ---
    telemetry["states_entered"].append("VERIFY_RELEASE")
    released = not (
        _contacts_between(data, model, cube_geom_id, left_pad_id)
        or _contacts_between(data, model, cube_geom_id, right_pad_id)
    )
    telemetry["released_after_open"] = released
    telemetry["post_release_cube_pose"] = data.xpos[cube_body_id].copy().tolist()
    if not released:
        return _fail("VERIFY_RELEASE", "cube still in contact with a finger pad after open + settle")

    # --- RETREAT ---
    telemetry["states_entered"].append("RETREAT")
    q_retreat, _, _, _ = _solve_waypoint(
        model, ik_scratch, data.qpos.copy(), arm_map, site_id, RETREAT_POS, nominal_q, use_oriented_ik
    )
    _drive_segment(RETREAT_POS, True, "RETREAT", retreat_drive_s, q_retreat)

    # --- VERIFY_TASK_SUCCESS ---
    telemetry["states_entered"].append("VERIFY_TASK_SUCCESS")
    dwell_ok = _task_success_dwell(q_retreat)
    if not dwell_ok:
        return _fail(
            "VERIFY_TASK_SUCCESS",
            f"task-success conditions did not hold continuously for {TASK_SUCCESS_DWELL_S}s "
            f"within {TASK_SUCCESS_MAX_WAIT_S}s (best streak {telemetry['task_success_dwell_achieved_s']:.3f}s)",
        )

    telemetry["states_entered"].append("DONE")
    return _finalize(task_success=True)


# --- initialization-boundary self-audit (independent of the other phases') -
# Unlike run_grasp_test_3c.py, this module legitimately *reads* cube
# velocity (data.qvel[cube_dof_adr:cube_dof_adr+6].copy(), for the
# linear/angular speed criteria) -- a plain substring check on
# "data.qvel[cube_dof_adr" would false-positive on that read. These patterns
# therefore require an assignment (`=`, not `==`) immediately after the
# index, so a read-only slice like `...].copy()` never matches.
def _assert_run_trial_pick_place_has_no_direct_cube_state_write() -> None:
    src = inspect.getsource(run_trial_pick_place)
    forbidden_write_patterns = [
        r"data\.qpos\[cube_qpos_adr[^\]]*\]\s*=(?!=)",
        r"data\.qvel\[cube_dof_adr[^\]]*\]\s*=(?!=)",
        r"xfrc_applied\[cube_body_id\]\s*=(?!=)",
    ]
    for pattern in forbidden_write_patterns:
        if re.search(pattern, src):
            raise AssertionError(
                f"run_trial_pick_place() contains a direct cube-state write (matches {pattern!r}) "
                "outside CubeInitGuard -- initialization boundary violated"
            )


_assert_run_trial_pick_place_has_no_direct_cube_state_write()


# --- Stage A / Stage B evaluation harness -----------------------------------
ARM_KP_4B = 400.0  # unchanged from Phase 3C/4A
ARM_KV_4B = 25.0
N_TRIALS_PER_VARIANT = 3  # matches Phase 4A's convention; the simulation is
# deterministic (no RNG anywhere in this pipeline, inherited from Phase 3C),
# so repeats are for auditability, not because run-to-run variance is
# expected -- any difference found would itself be a notable finding.

# Stage B: the three variants Phase 4A found reachable for the *grasp* itself
# (reports/phase4a-grasp-variants.md). The other two 0.03 m offsets
# (x_plus_0.03, y_minus_0.03) were rejected there by a pre-run IK residual
# check (27.1 mm and 8.43 mm respectively, both over the 8 mm tolerance) and
# confirmed unreachable in simulation -- they are not re-attempted here and
# are excluded from Stage B's primary success denominator, per HANDOFF.md.
STAGE_B_VARIANTS = [
    {"id": "nominal", "offset": (0.0, 0.0)},
    {"id": "x_minus_0.03", "offset": (-0.03, 0.0)},
    {"id": "y_plus_0.03", "offset": (0.0, 0.03)},
]
EXCLUDED_UNREACHABLE_VARIANTS = [
    {"id": "x_plus_0.03", "offset": (0.03, 0.0), "phase4a_approach_residual_m": 0.02708},
    {"id": "y_minus_0.03", "offset": (0.0, -0.03), "phase4a_approach_residual_m": 0.00843},
]


def _run_variant(scene_path: Path, variant_id: str, offset: tuple, n_trials: int) -> dict:
    trials = []
    for i in range(n_trials):
        r = run_trial_pick_place(scene_path, cube_xy_offset=offset)
        trials.append(
            {
                "trial_index": i,
                "task_pass": r["task_pass"],
                "grasp_pass": r["grasp_pass"],
                "placement_pass": r["placement_pass"],
                "criteria_grasp": r["criteria_grasp"],
                "criteria_placement": r["criteria_placement"],
                "failure_state": r["failure_state"],
                "failure_reason": r["failure_reason"],
                "height_gain_m": r["height_gain_m"],
                "max_continuous_lifted_s": r["max_continuous_lifted_s"],
                "max_cube_slip_m": r["max_cube_slip_m"],
                "grasp_reference_offset_tcp_frame": r["grasp_reference_offset_tcp_frame"],
                "max_slip_during_lift": r["max_slip_during_lift"],
                "max_slip_during_transport": r["max_slip_during_transport"],
                "max_slip_during_lower": r["max_slip_during_lower"],
                "slip_at_release": r["slip_at_release"],
                "post_release_tcp_cube_separation_m": r["post_release_tcp_cube_separation_m"],
                "contact_lost_during_transport": r["contact_lost_during_transport"],
                "final_cube_xy": r["final_cube_xy"],
                "final_xy_target_error_m": r["final_xy_target_error_m"],
                "final_cube_linear_speed": r["final_cube_linear_speed"],
                "final_cube_angular_speed": r["final_cube_angular_speed"],
                "retreat_disturbance_m": r["retreat_disturbance_m"],
                "task_success_dwell_achieved_s": r["task_success_dwell_achieved_s"],
            }
        )
    n_grasp_pass = sum(1 for t in trials if t["grasp_pass"])
    n_placement_pass = sum(1 for t in trials if t["placement_pass"])
    n_task_pass = sum(1 for t in trials if t["task_pass"])
    n_contact_retained = sum(1 for t in trials if not t["contact_lost_during_transport"])
    return {
        "id": variant_id,
        "offset_m": list(offset),
        "n_trials": n_trials,
        "trials": trials,
        "n_grasp_pass": n_grasp_pass,
        "n_placement_pass": n_placement_pass,
        "n_task_pass": n_task_pass,
        "transport_contact_retention_rate": n_contact_retained / n_trials,
        "max_cube_slip_m": max(t["max_cube_slip_m"] for t in trials),
        "max_slip_during_lift": max(t["max_slip_during_lift"] for t in trials),
        "max_slip_during_transport": max(t["max_slip_during_transport"] for t in trials),
        "max_slip_during_lower": max(t["max_slip_during_lower"] for t in trials),
        "slip_at_release": max(t["slip_at_release"] for t in trials if t["slip_at_release"] is not None),
        "post_release_tcp_cube_separation_m": max(t["post_release_tcp_cube_separation_m"] for t in trials),
        "final_xy_target_error_m": trials[-1]["final_xy_target_error_m"],
        "settling_time_s": trials[-1]["task_success_dwell_achieved_s"],
        "variant_task_success": n_task_pass >= (n_trials + 1) // 2,  # majority rule, same convention as Phase 4A
    }


def main() -> int:
    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B)
    record: dict = {
        "scope": "fixed-base, torso-constrained upper-body manipulation baseline",
        "scene": str(scene.relative_to(ROOT)),
        "shared_configuration": {
            "arm_kp": ARM_KP_4B, "arm_kv": ARM_KV_4B,
            "gripper_kp": GRIPPER_KP_3C, "gripper_kd": GRIPPER_KD_3C,
            "transport_drive_s": TRANSPORT_DRIVE_S, "transport_n_waypoints": TRANSPORT_N_WAYPOINTS,
            "lower_drive_s": LOWER_DRIVE_S, "lower_n_waypoints": LOWER_N_WAYPOINTS,
            "retreat_drive_s": RETREAT_DRIVE_S,
        },
        "target_xy": list(TARGET_XY),
        "target_xy_success_margin_m": TARGET_XY_SUCCESS_MARGIN_M,
    }

    # --- Stage A: nominal only, 5 reruns (determinism check, same convention
    # as Phase 3C) -----------------------------------------------------------
    stage_a_trials = [run_trial_pick_place(scene) for _ in range(5)]
    record["stage_a"] = {
        "n_trials": 5,
        "all_task_pass": all(t["task_pass"] for t in stage_a_trials),
        "deterministic": len({t["height_gain_m"] for t in stage_a_trials}) == 1
        and len({t["final_xy_target_error_m"] for t in stage_a_trials}) == 1,
        "height_gain_m": stage_a_trials[0]["height_gain_m"],
        "final_xy_target_error_m": stage_a_trials[0]["final_xy_target_error_m"],
        "max_cube_slip_m": stage_a_trials[0]["max_cube_slip_m"],
        "max_slip_during_lift": stage_a_trials[0]["max_slip_during_lift"],
        "max_slip_during_transport": stage_a_trials[0]["max_slip_during_transport"],
        "max_slip_during_lower": stage_a_trials[0]["max_slip_during_lower"],
        "slip_at_release": stage_a_trials[0]["slip_at_release"],
        "post_release_tcp_cube_separation_m": stage_a_trials[0]["post_release_tcp_cube_separation_m"],
        "settling_time_s": stage_a_trials[0]["task_success_dwell_achieved_s"],
    }

    # --- Stage B: 3 reachable variants, shared config, no per-variant tuning
    stage_b_variants = [
        _run_variant(scene, v["id"], v["offset"], N_TRIALS_PER_VARIANT) for v in STAGE_B_VARIANTS
    ]
    n_variants_task_success = sum(1 for v in stage_b_variants if v["variant_task_success"])
    record["stage_b"] = {
        "variants": stage_b_variants,
        "excluded_unreachable_variants": EXCLUDED_UNREACHABLE_VARIANTS,
        "n_variants_task_success": n_variants_task_success,
        "n_variants_evaluated": len(STAGE_B_VARIANTS),
        "supported_envelope_success_rate": n_variants_task_success / len(STAGE_B_VARIANTS),
        "original_five_variant_coverage": f"{n_variants_task_success}/5",
    }

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase4b_pick_place_trials.json").write_text(
        json.dumps(record, indent=2, default=str) + "\n", encoding="utf-8"
    )
    print(json.dumps(
        {
            "stage_a_pass": record["stage_a"]["all_task_pass"],
            "stage_b_supported_envelope_success_rate": record["stage_b"]["supported_envelope_success_rate"],
        },
        indent=2,
    ))
    return 0 if record["stage_a"]["all_task_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
