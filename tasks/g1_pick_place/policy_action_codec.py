#!/usr/bin/env python3
"""Phase 5D: causally-valid, reference-relative TCP-delta VLA action codec.

Root cause of Phase 5C's ~9.7cm max policy-action replay error (see
reports/phase5c-replay-fidelity.md, Section D): the stored high-level action
was a single STATIC PER-PHASE GOAL (`cartesian_target`, e.g. the same
TRANSPORT_ABOVE_TARGET_POS repeated across ~40 consecutive 10Hz transitions),
not the expert's actual next-interval command. A decoder with no
waypoint-index information necessarily re-ramps toward that same distant
point inside every 100ms interval, producing a straighter/faster path than
the true multi-waypoint trajectory.

This module fixes the ACTION SEMANTICS, not just the decoder: at each policy
transition t, the primary action is the expert's actual commanded TCP
reference DELTA over interval [t, t+1] -- derived purely from the expert's
own commanded joint-target trajectory (`run_pick_place._drive_smooth`'s
`ramped_target`, already captured once per physics step as
`execution/arm_joint_target` in Phase 5C's collector), never from privileged
cube/target state and never from a future policy observation.

Three attempts were made (see reports/phase5d-policy-action-redesign.md
Section "Attempt history" for the full measured numbers):

1. Single 10Hz reference-relative TCP delta, decoded via ONE IK solve plus
   ONE linear joint-space ramp across the full 100ms interval. Fixed the
   bulk of Phase 5C's error (9.7cm -> 2.36cm max) but still missed the
   10mm target: `_drive_segment` phases (PREGRASP, RETREAT, ...) issue a
   fixed joint-space set-point ONCE and hold it for the WHOLE segment
   (often many policy transitions), so the true measured TCP converges to
   that target gradually across the ENTIRE segment's wall-clock duration --
   but attempt 1's single ramp forces the decoder's own commanded
   reference to fully reach the same target within just ONE 100ms
   interval, giving the position-servo systematically less "settle time"
   than the true trial had at that same wall-clock point, and adjusting
   the ramp's shape/speed only traded one large-single-delta transition's
   error for another's (measured directly, not guessed).
2. Adjusting only the ramp speed/shape (faster ramps, immediate steps) was
   tried across a sweep and never found one interpolation shape that
   worked for every large single-transition delta simultaneously (a fast
   ramp fixed PREGRASP's transition but made a later large RETREAT-phase
   delta worse, and vice versa) -- diagnosed as a genuine information gap
   (one 100ms-wide delta cannot describe an expert trajectory that
   contains a large sub-100ms-scale reference jump), not a tuning problem.
3. **Shipped**: a fixed-size sub-action CHUNK. Each 10Hz policy transition
   stores `SUB_ACTIONS_PER_TRANSITION` (5) TCP sub-deltas, each covering
   `SUBSTEPS_PER_TRANSITION // 5` (10) physics steps (a 50Hz sub-action
   rate) -- decoded with the same one-IK-solve-plus-one-linear-ramp
   primitive as attempts 1/2, just applied once per sub-chunk instead of
   once per whole 100ms interval. This gives the decoder genuine
   information about where within the 100ms window a large reference
   change actually happened (instead of guessing an interpolation shape),
   and met the <=10mm target on both successful episodes (measured 8.09mm
   nominal, 5.99mm x_minus_0.03 -- see the report for the full sweep that
   selected H=5 over H=1/2/10).

Two reference frames, both verified empirically in this module's own tests
(tests/test_phase5d_policy_actions.py), not assumed:

- Position delta: WORLD frame. This robot is fixed-base (pelvis+torso welded
  to the world via an equality constraint); Phase 5A independently measured
  that weld's softness at ~0.19mm, so world frame and robot-base frame
  coincide to within that sub-millimeter margin. World frame is used
  directly rather than introducing a separate "base frame" transform that
  would differ from world by less than measurement noise.
- Orientation delta: TCP-LOCAL (body) frame, using MuJoCo's native
  `mju_subQuat`/`mju_quatIntegrate` operator pair. Verified by direct
  numerical test: `mju_subQuat(qa, qb)` returns the rotation vector `r` such
  that `qa = qb (x) axisAngle2Quat(r)` (right/body-frame composition -- NOT
  world/left composition), and `mju_quatIntegrate(qb, r, 1.0)` reconstructs
  `qa` to machine precision. This is the standard MuJoCo convention used
  internally for integrating quaternion-valued degrees of freedom.

The decoder composes each sub-delta with the CURRENT COMMANDED REFERENCE
(not noisy measured TCP state) to avoid compounding tracking-error noise
across many replayed steps, interpolates ONCE per sub-chunk (a single
linear joint-space ramp, mirroring `_drive_smooth`'s per-waypoint ramp) and
never restarts a new ramp toward a distant goal mid-sub-chunk. The gripper
command remains a single scalar per 10Hz transition (held constant across
all 5 sub-chunks of that transition), not chunked -- only the TCP delta is.
"""

from __future__ import annotations

import hashlib
import json

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import JointMap
from tasks.g1_pick_place.controller_3c import solve_ik_waypoint

ACTION_SCHEMA_VERSION = "3.0.0"
DECODER_VERSION = "v3-chunked-ramp-h5"

SUBSTEPS_PER_TRANSITION = 50  # 10 Hz policy / 500 Hz physics
SUB_ACTIONS_PER_TRANSITION = 5  # H: attempt-3 chunk size, selected by measured sweep (see module docstring)
SUB_STEPS_PER_SUBACTION = SUBSTEPS_PER_TRANSITION // SUB_ACTIONS_PER_TRANSITION  # 10 (50 Hz sub-action rate)
SUB_ACTION_HZ = 1.0 / 0.002 / SUB_STEPS_PER_SUBACTION  # 50.0

POSITION_FRAME = (
    "world (measured/verified to coincide with the robot-base/pelvis frame "
    "to within ~0.19mm, the pelvis weld's softness independently measured "
    "in Phase 5A -- this is a fixed-base robot, so no separate base-frame "
    "transform is applied)"
)
ORIENTATION_FRAME = (
    "TCP-local (body) frame: tcp_delta_orientation[t] = mju_subQuat(quat(t+1), "
    "quat(t)), i.e. the rotation vector r such that quat(t+1) = quat(t) (x) "
    "axisAngle2Quat(r) using MuJoCo's mju_mulQuat composition (right/body-frame "
    "composition, verified numerically in tests/test_phase5d_policy_actions.py, "
    "not assumed). Decoded back via mju_quatIntegrate(quat(t), r, 1.0)."
)


def mat_to_quat_wxyz(mat9: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(mat9).flatten())
    return q


def sub_quat(qa: np.ndarray, qb: np.ndarray) -> np.ndarray:
    """Rotation vector r (TCP/body-local to qb) such that qa = qb (x) axisAngle2Quat(r)."""
    r = np.zeros(3)
    mujoco.mju_subQuat(r, np.asarray(qa, dtype=float), np.asarray(qb, dtype=float))
    return r


def quat_integrate(qb: np.ndarray, r: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """Inverse of sub_quat: returns qb (x) axisAngle2Quat(r * scale)."""
    q = np.asarray(qb, dtype=float).copy()
    mujoco.mju_quatIntegrate(q, np.asarray(r, dtype=float), float(scale))
    return q


def forward_kinematics_tcp(
    model: mujoco.MjModel,
    template_qpos: np.ndarray,
    arm_map: JointMap,
    site_id: int,
    arm_joint_target: np.ndarray,
    scratch: mujoco.MjData | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Forward-kinematics-only TCP pose for a COMMANDED joint target -- not
    a measured/tracked state. `template_qpos` supplies every non-arm DOF
    (gripper, cube, welded-body qpos); only the arm slice is overwritten,
    since the arm is a serial kinematic chain from the fixed base to the
    TCP site and no other DOF affects that chain's forward kinematics.
    Uses `mj_kinematics` only (no dynamics, no contact) -- this is a pure
    geometric query, deterministic and side-effect-free on `scratch`.
    """
    d = scratch if scratch is not None else mujoco.MjData(model)
    d.qpos[:] = template_qpos
    arm_map.set_qpos(d, arm_joint_target)
    mujoco.mj_kinematics(model, d)
    pos = d.site_xpos[site_id].copy()
    quat = mat_to_quat_wxyz(d.site_xmat[site_id].reshape(3, 3))
    return pos, quat


def encode_delta(
    pos_t: np.ndarray, quat_t: np.ndarray, pos_t1: np.ndarray, quat_t1: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """(tcp_delta_position, tcp_delta_orientation) for interval [t, t+1]."""
    delta_pos = np.asarray(pos_t1, dtype=np.float32) - np.asarray(pos_t, dtype=np.float32)
    delta_rot = sub_quat(quat_t1, quat_t).astype(np.float32)
    return delta_pos, delta_rot


def decode_target(
    pos_t: np.ndarray, quat_t: np.ndarray, delta_pos: np.ndarray, delta_rot: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of encode_delta: reconstructs (pos_t1, quat_t1) from the
    commanded reference at t plus the recorded delta.
    """
    pos_t1 = np.asarray(pos_t, dtype=np.float64) + np.asarray(delta_pos, dtype=np.float64)
    quat_t1 = quat_integrate(quat_t, delta_rot, 1.0)
    return pos_t1, quat_t1


def decoder_config_dict(
    n_substeps: int = SUBSTEPS_PER_TRANSITION, chunk_h: int = SUB_ACTIONS_PER_TRANSITION,
) -> dict:
    sub_steps = n_substeps // chunk_h
    return {
        "decoder_version": DECODER_VERSION,
        "action_schema_version": ACTION_SCHEMA_VERSION,
        "position_frame": "world",
        "orientation_frame": "tcp_local_body_mju_subQuat_quatIntegrate",
        "interpolation": (
            "chunked_linear_joint_space_ramp: each 10Hz transition carries "
            "H sub-deltas; for each sub-chunk, solve IK once for the "
            "sub-chunk's TCP-position endpoint (commanded_reference + "
            "sub_delta), then linearly ramp the commanded joint target from "
            "the previous sub-chunk's solved joint target to the newly "
            "solved one, evaluated once per physics substep within that "
            "sub-chunk -- no re-targeting within a sub-chunk."
        ),
        "substeps_per_transition": int(n_substeps),
        "action_chunk_h": int(chunk_h),
        "sub_steps_per_subaction": int(sub_steps),
        "sub_action_hz": float((1.0 / 0.002) / sub_steps),
        "chunk_shape": "[T, H, 3] for tcp_delta_position and tcp_delta_orientation each; gripper_command stays [T] (not chunked)",
        "orientation_used_by_ik": False,
        "orientation_used_by_ik_note": (
            "The canonical controller's IK (solve_ik_waypoint, "
            "use_oriented_ik=false) is position-only; tcp_delta_orientation "
            "is faithfully recorded from the expert's commanded reference "
            "trajectory but is not fed into this decoder's IK call, "
            "consistent with not modifying Task 1's controller in this phase."
        ),
        "position_composition": "commanded_reference_position + sub_delta_position (world frame, additive), per sub-chunk",
        "orientation_composition": "quat_integrate(commanded_reference_quat, sub_delta_orientation, 1.0), per sub-chunk",
        "gripper_command": "held constant across all H sub-chunks of a transition",
        "accumulation_source": "commanded reference state (decoder-internal), never re-read from measured/noisy state mid-episode",
    }


def decoder_configuration_hash(
    n_substeps: int = SUBSTEPS_PER_TRANSITION, chunk_h: int = SUB_ACTIONS_PER_TRANSITION,
) -> str:
    canon = json.dumps(decoder_config_dict(n_substeps, chunk_h), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canon.encode("utf-8")).hexdigest()


def ramp_joint_targets(
    model: mujoco.MjModel,
    ik_scratch: mujoco.MjData,
    template_qpos: np.ndarray,
    arm_map: JointMap,
    site_id: int,
    nominal_q: np.ndarray,
    current_joint_target: np.ndarray,
    target_pos: np.ndarray,
    n_substeps: int,
) -> tuple[list, np.ndarray]:
    """Solve IK ONCE for `target_pos`, then linearly ramp the commanded
    joint target from `current_joint_target` to that solution across
    `n_substeps` physics steps (one ramped target per step). Mirrors
    `run_pick_place._drive_smooth`'s per-waypoint ramp, applied once per
    full policy interval instead of once per sub-waypoint -- satisfies
    "interpolate once... do not restart a new ramp toward a distant phase
    goal inside the same interval."

    Returns (list_of_n_substeps_joint_targets, q_target).
    """
    base_qpos = np.asarray(template_qpos, dtype=float).copy()
    q_target, _resid, _iters = solve_ik_waypoint(
        model, ik_scratch, base_qpos, arm_map, site_id, np.asarray(target_pos, dtype=float), nominal_q,
    )
    ramp = []
    for s in range(n_substeps):
        beta = (s + 1) / n_substeps
        ramp.append(current_joint_target + beta * (q_target - current_joint_target))
    return ramp, q_target
