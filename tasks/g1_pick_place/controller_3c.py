#!/usr/bin/env python3
"""Phase 3C IK: position-priority waypoint IK with null-space posture and
joint-limit avoidance, solved once per motion segment (not at every control
step, unlike Phase 3/3B's continuously-resolved solve_dls_ik).

This module deliberately does not modify controller.py (Phase 3/3B's
torque-PD + resolved-rate DLS-IK is historical and stays untouched, still
imported by tests/test_phase3_controller.py and tests/test_phase3_grasp.py).
It reuses controller.py's pure data structures (JointMap, the joint/actuator
name lists) rather than duplicating them.
"""

from __future__ import annotations

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import JointMap  # noqa: F401 (re-exported)
from tasks.g1_pick_place.gripper_scene import CUBE_HALF, FINGER_PAD_HALF

# Explicit convergence tolerance and iteration cap for the waypoint solve.
#
# Reachability diagnosis (reports/phase3c-position-servo-baseline.md) found
# the grasp waypoint sits near a genuine wrist kinematic singularity (wrist
# roll/pitch/yaw axes nearly aligned when wrist_pitch approaches 0; smallest
# task-Jacobian singular value ~5e-4 there). Task-only DLS-IK plateaus at
# ~5.7-6.2 mm residual regardless of damping (0.008-0.08) or iteration
# budget (150-300) at that specific configuration -- not a bug, a real
# reachability floor for this arm/attachment geometry at this Cartesian
# point. IK_POS_TOL is therefore set from that evidence, not an arbitrary
# round number, with a small margin above the observed plateau.
IK_POS_TOL = 8e-3  # 8 mm, evidence-based (see above; the grasp waypoint itself
# plateaus at ~7.45 mm even warm-started from the converged pregrasp
# solution -- small relative to the cube's 35 mm half-extent and the
# gripper's 15 mm squeeze-overtravel margin, so accepted rather than
# rejected as unreachable)
IK_MAX_ITERS = 200
IK_DAMPING = 0.02
IK_MAX_DQ_STEP = 0.05  # rad per internal IK iteration, same safety idea as Phase 3/3B

# Null-space secondary objectives. Posture gain kept modest: a larger gain
# (0.4) was found to fight the primary task near the wrist singularity above
# and roughly quadrupled the residual there without changing the underlying
# reachability limit.
NULLSPACE_POSTURE_GAIN = 0.15
JOINT_LIMIT_MARGIN_FRAC = 0.12  # start repelling within this fraction of the joint's range from a bound
JOINT_LIMIT_GAIN = 2.0


def solve_ik_waypoint(
    model: mujoco.MjModel,
    scratch_data: mujoco.MjData,
    base_qpos: np.ndarray,
    joint_map: JointMap,
    site_id: int,
    target_pos: np.ndarray,
    nominal_q: np.ndarray,
    damping: float = IK_DAMPING,
    max_iters: int = IK_MAX_ITERS,
    pos_tol: float = IK_POS_TOL,
) -> tuple[np.ndarray, float, int]:
    """Position-priority IK for one Cartesian waypoint.

    Primary task: drive the TCP site to `target_pos` (position only -- no
    orientation constraint). Orientation is deliberately left unconstrained
    here: Phase 3/3B's failures were joint-tracking oscillation, not
    orientation error, and the parallel gripper is symmetric about its
    closing axis, so constraining wrist roll/yaw would over-constrain the
    solve without addressing the actual defect (see
    reports/phase3c-position-servo-baseline.md for the empirical check of
    wrist orientation drift that motivated this choice).

    Secondary (null-space) objectives, in order: (a) joint-limit avoidance
    -- a repulsive term active only within JOINT_LIMIT_MARGIN_FRAC of a
    joint's range bound, so the solve isn't pulled toward mechanical limits;
    (b) a nominal-posture attraction toward `nominal_q` (the arm's neutral
    pose) for any remaining redundancy.

    Solved once (per call) to convergence or `max_iters`, not re-solved at
    control-loop rate -- the caller uses the returned joint target as a
    fixed set-point for an entire motion segment.

    Returns (joint_target, residual_pos_error_m, iterations_used).
    """
    scratch_data.qpos[:] = base_qpos
    q = joint_map.get_qpos(scratch_data).copy()
    lo = joint_map.jnt_range[:, 0]
    hi = joint_map.jnt_range[:, 1]
    span = np.maximum(hi - lo, 1e-6)
    margin = JOINT_LIMIT_MARGIN_FRAC * span

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    residual = float("inf")
    iters_used = 0

    for it in range(max_iters):
        joint_map.set_qpos(scratch_data, q)
        mujoco.mj_kinematics(model, scratch_data)
        mujoco.mj_comPos(model, scratch_data)
        site_pos = scratch_data.site_xpos[site_id].copy()
        err = target_pos - site_pos
        residual = float(np.linalg.norm(err))
        iters_used = it
        if residual < pos_tol:
            break

        mujoco.mj_jacSite(model, scratch_data, jacp, jacr, site_id)
        J = jacp[:, joint_map.dof_adr]
        JJt = J @ J.T + (damping ** 2) * np.eye(3)
        J_pinv = J.T @ np.linalg.solve(JJt, np.eye(3))
        dq_task = J_pinv @ err

        to_hi = hi - q
        to_lo = q - lo
        push = np.zeros_like(q)
        near_hi = to_hi < margin
        near_lo = to_lo < margin
        push[near_hi] -= JOINT_LIMIT_GAIN * (margin[near_hi] - to_hi[near_hi]) / margin[near_hi]
        push[near_lo] += JOINT_LIMIT_GAIN * (margin[near_lo] - to_lo[near_lo]) / margin[near_lo]

        dq_posture = NULLSPACE_POSTURE_GAIN * (nominal_q - q)

        N = np.eye(len(q)) - J_pinv @ J
        dq_null = N @ (dq_posture + push)

        dq = dq_task + dq_null
        step_norm = np.linalg.norm(dq)
        if step_norm > IK_MAX_DQ_STEP:
            dq = dq * (IK_MAX_DQ_STEP / step_norm)

        q = np.clip(q + dq, lo, hi)

    return q, residual, iters_used


# --- Phase 4F: orientation-aware waypoint IK -------------------------------
#
# Phase 4E's frame-by-frame video review (reports/phase4e-gripper-integrity-
# repair.md) and the user's own follow-up review found the grasp still
# slides ~20.5 mm inside the closed grip even after Phase 4E's trajectory-
# smoothing and gain fixes. The primary hypothesis: solve_ik_waypoint()
# above deliberately leaves orientation unconstrained (see its own
# docstring), so the arm's redundant joints are free to roll the wrist
# about its own approach axis (local X) while still satisfying the TCP
# *position* target exactly. Because the two finger pads are offset from
# the TCP origin along the wrist's local Y axis by FINGER_CONTACT_Y (=
# CUBE_HALF + FINGER_PAD_HALF[1]), a roll angle theta about local X moves
# each pad's actual world-Z contact point by approximately
# FINGER_CONTACT_Y * sin(theta) relative to where it would be at zero roll
# -- i.e. relative to the cube's vertical center, since APPROACH's target
# is the cube's own center. A pad contacting off-center in Z grips less of
# the cube's mass on that side and applies a net moment that lets the cube
# rotate/slide downward inside the grip -- exactly the reported symptom.
#
# This module's required-axis choice: constrain only the wrist's local Z
# axis (the finger pads' own "tall" axis, FINGER_PAD_HALF[2] = 0.030 m) to
# stay aligned with world +Z (vertical). This is deliberately a 2-DOF
# constraint (aligning one 3D axis eliminates 2 rotational degrees of
# freedom), not a full 3-DOF orientation lock: rotation about the aligned
# axis itself (yaw about vertical) is left free, because it does not move
# either pad's Z-height and the position task already fixes where the TCP
# origin sits in the horizontal plane. This also implicitly keeps the jaw
# axis (local Y, along which the two fingers slide -- see FINGER_REACH_X /
# TCP_POS in gripper_scene.py) close to horizontal, since local Y and local
# Z are orthogonal by construction: an aligned local Z means local Y stays
# in the horizontal plane, which is exactly "opposing side faces" contact
# for an upright cube whose side faces are vertical.
REQUIRED_AXIS_LOCAL = np.array([0.0, 0.0, 1.0])
DESIRED_AXIS_WORLD = np.array([0.0, 0.0, 1.0])

FINGER_CONTACT_Y = CUBE_HALF + FINGER_PAD_HALF[1]

# Orientation tolerance, derived the same way IK_POS_TOL was in Phase 3C
# (evidence-based, not an arbitrary round number): choose the largest tilt
# that keeps the resulting pad-height error, FINGER_CONTACT_Y * sin(tol),
# at or below 5 mm -- half of Phase 4E's own vertical-overlap margin
# (FINGER_PAD_HALF[2] = 30 mm) and equal to this phase's own tightened
# downward-slip-during-HOLD acceptance bar (<=5 mm, see
# reports/phase4f-orientation-grasp-stabilization.md, Section C). A looser
# tolerance would let exactly the geometric error this phase exists to
# eliminate through unchecked.
ORIENT_MAX_PAD_HEIGHT_ERROR_M = 0.005
ORIENT_TOL_RAD = float(np.arcsin(min(1.0, ORIENT_MAX_PAD_HEIGHT_ERROR_M / FINGER_CONTACT_Y)))

# Position/orientation task weights, individually documented (per HANDOFF.md
# Phase 4F Section A: "use a weighted pose error with separately documented
# position and orientation weights"). POS_WEIGHT is implicit at 1.0 -- the
# position task is solved to its own convergence via the unweighted primary
# Jacobian pinv exactly as solve_ik_waypoint() does above, so it is never
# degraded by the orientation objective. ORIENT_WEIGHT scales the secondary
# orientation-alignment step that is then projected into the position
# task's null space (N below) before being added -- so it can only use
# redundancy the position task does not need, never trade position accuracy
# for orientation accuracy. 0.6 was Attempt 1's starting value (see the
# report for the measured residuals that motivated Attempt 2's revision).
ORIENT_WEIGHT = 0.6
ORIENT_DAMPING = 0.05


def orientation_residual_rad(site_xmat_flat: np.ndarray) -> float:
    """Angle (radians) between the wrist site's local Z axis (world frame)
    and vertical (+Z world). 0 = pads perfectly level; used both inside the
    IK solve above and as a live telemetry/overlay value in
    run_pick_place.py.
    """
    R = np.asarray(site_xmat_flat).reshape(3, 3)
    cur_axis = R @ REQUIRED_AXIS_LOCAL
    cos_ang = float(np.clip(np.dot(cur_axis, DESIRED_AXIS_WORLD), -1.0, 1.0))
    return float(np.arccos(cos_ang))


def solve_ik_waypoint_oriented(
    model: mujoco.MjModel,
    scratch_data: mujoco.MjData,
    base_qpos: np.ndarray,
    joint_map: JointMap,
    site_id: int,
    target_pos: np.ndarray,
    nominal_q: np.ndarray,
    damping: float = IK_DAMPING,
    max_iters: int = IK_MAX_ITERS,
    pos_tol: float = IK_POS_TOL,
    orient_weight: float = ORIENT_WEIGHT,
) -> tuple[np.ndarray, float, int, float]:
    """Phase 4F: solve_ik_waypoint() extended with a null-space, required-
    axis orientation objective (see module comment above for the axis
    choice and its justification). TCP position priority is preserved
    exactly: dq_task below is bit-for-bit the same computation as
    solve_ik_waypoint()'s primary task, using the same unweighted pinv;
    orientation only consumes redundancy already unused by the position
    task (via the null-space projector N), same priority pattern already
    used for joint-limit avoidance and nominal-posture attraction.

    Returns (joint_target, residual_pos_error_m, iterations_used,
    residual_orient_rad) -- the orientation residual is always computed and
    reported (for telemetry/overlay/rejection), even though it is a
    secondary, not primary, objective.
    """
    scratch_data.qpos[:] = base_qpos
    q = joint_map.get_qpos(scratch_data).copy()
    lo = joint_map.jnt_range[:, 0]
    hi = joint_map.jnt_range[:, 1]
    span = np.maximum(hi - lo, 1e-6)
    margin = JOINT_LIMIT_MARGIN_FRAC * span

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    residual = float("inf")
    orient_residual = float("inf")
    iters_used = 0

    for it in range(max_iters):
        joint_map.set_qpos(scratch_data, q)
        mujoco.mj_kinematics(model, scratch_data)
        mujoco.mj_comPos(model, scratch_data)
        site_pos = scratch_data.site_xpos[site_id].copy()
        site_mat = scratch_data.site_xmat[site_id].copy()
        err = target_pos - site_pos
        residual = float(np.linalg.norm(err))
        orient_residual = orientation_residual_rad(site_mat)
        iters_used = it
        if residual < pos_tol and orient_residual < ORIENT_TOL_RAD:
            break

        mujoco.mj_jacSite(model, scratch_data, jacp, jacr, site_id)
        J = jacp[:, joint_map.dof_adr]
        JJt = J @ J.T + (damping ** 2) * np.eye(3)
        J_pinv = J.T @ np.linalg.solve(JJt, np.eye(3))
        dq_task = J_pinv @ err

        cur_axis = site_mat.reshape(3, 3) @ REQUIRED_AXIS_LOCAL
        rot_err = np.cross(cur_axis, DESIRED_AXIS_WORLD)
        Jr = jacr[:, joint_map.dof_adr]
        JrJrt = Jr @ Jr.T + (ORIENT_DAMPING ** 2) * np.eye(3)
        Jr_pinv = Jr.T @ np.linalg.solve(JrJrt, np.eye(3))
        dq_orient = orient_weight * (Jr_pinv @ rot_err)

        to_hi = hi - q
        to_lo = q - lo
        push = np.zeros_like(q)
        near_hi = to_hi < margin
        near_lo = to_lo < margin
        push[near_hi] -= JOINT_LIMIT_GAIN * (margin[near_hi] - to_hi[near_hi]) / margin[near_hi]
        push[near_lo] += JOINT_LIMIT_GAIN * (margin[near_lo] - to_lo[near_lo]) / margin[near_lo]

        dq_posture = NULLSPACE_POSTURE_GAIN * (nominal_q - q)

        N = np.eye(len(q)) - J_pinv @ J
        dq_null = N @ (dq_orient + dq_posture + push)

        dq = dq_task + dq_null
        step_norm = np.linalg.norm(dq)
        if step_norm > IK_MAX_DQ_STEP:
            dq = dq * (IK_MAX_DQ_STEP / step_norm)

        q = np.clip(q + dq, lo, hi)

    return q, residual, iters_used, orient_residual
