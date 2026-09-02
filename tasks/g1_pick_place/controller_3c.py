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
