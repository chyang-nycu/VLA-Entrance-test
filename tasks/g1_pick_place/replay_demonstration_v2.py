#!/usr/bin/env python3
"""Phase 5C: two-rate demonstration replay (Section C).

Three independent modes:

1. Exact execution replay (`replay_exact_execution`): restores ONLY the
   recorded initial condition (cube spawn offset, via the same CubeInitGuard
   pre-lock boundary every module in this project uses), then feeds the
   dataset's `execution/applied_ctrl` vector directly into `data.ctrl` at
   EVERY physics step and calls `mujoco.mj_step` -- it never re-solves IK,
   never re-runs `bounded_pd_step`, and never re-derives a target from a
   coarser sample. Since MuJoCo physics is deterministic given identical
   ctrl and identical initial state (this pipeline has no RNG anywhere),
   this is expected to reproduce the original trajectory to near machine
   precision, unlike Phase 5B's 10 Hz zero-order-hold action replay.

2. Policy-action replay (`replay_policy_actions`): uses ONLY the 10 Hz
   `policy/high_level_actions` stream (the same information an actually
   deployed VLA policy would emit) and reconstructs motion through a
   decoder that ramps -- exactly the way `run_pick_place._drive_smooth`
   ramps -- from the current TCP position to the recorded static per-
   interval Cartesian target, using `K_WAYPOINTS_PER_POLICY_STEP` internal
   IK sub-waypoints per 100 ms interval (a decoder-side hyperparameter,
   chosen independently of which internal waypoint count the original
   collection-time controller happened to use for a given phase -- a real
   policy has no notion of "phase"). This is the fix for Phase 5B's replay
   bug, which held the 10 Hz target constant for the whole interval instead
   of ramping to it.

3. Observation-only visualization replay (`visualize_episode`): unchanged in
   spirit from Phase 5B -- plays back stored RGB/state with zero physics
   stepped.

All three load the canonical manifest and check the dataset's stored
`canonical_manifest_sha256` against it; a mismatch raises
(`canonical_config.ManifestMismatchError`).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import mujoco
import numpy as np

from tasks.g1_pick_place.camera_observation import write_grasp_scene_5a
from tasks.g1_pick_place.canonical_config import ManifestMismatchError, load_manifest
from tasks.g1_pick_place.controller import (
    GRIPPER_ACTUATORS, GRIPPER_JOINTS, GRIPPER_MAX_QVEL, GRIPPER_MAX_STEP,
    RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap, bounded_pd_step,
)
from tasks.g1_pick_place.controller_3c import solve_ik_waypoint
from tasks.g1_pick_place.gripper_scene import CUBE_POS
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard
from tasks.g1_pick_place.run_grasp_test_3c import _finger_targets
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E

ROOT = Path(__file__).resolve().parents[2]

TIMESTEP = 0.002
POLICY_HZ = 10.0
SUBSTEPS_PER_TRANSITION = int(round((1.0 / TIMESTEP) / POLICY_HZ))  # 50

# Exact execution replay targets (Section D): if MuJoCo is not bit-exact for
# this replay approach, these are measured/justified, not loosened after the
# fact -- see reports/phase5c-replay-fidelity.md for the actual achieved
# numbers and the floating-point-accumulation argument for why they are not
# exactly zero.
EXACT_JOINT_TOL_RAD = 1e-4
EXACT_TCP_TOL_M = 1e-3
EXACT_CUBE_TOL_M = 1e-3

# Policy-action replay decoder hyperparameters (decoder-side, not part of the
# dataset -- a real deployment would choose these independently of how the
# original collection-time controller happened to be configured per-phase).
K_WAYPOINTS_PER_POLICY_STEP = 5
POLICY_TCP_TOL_M = 0.010


def _mat_to_quat_wxyz(mat9: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(mat9).flatten())
    return q


def _check_manifest(f: h5py.File) -> None:
    manifest = load_manifest()
    stored = f.attrs.get("canonical_manifest_sha256")
    if stored != manifest["hash"]["value"]:
        raise ManifestMismatchError(
            f"dataset's canonical_manifest_sha256 ({stored!r}) does not match the current "
            f"data/task1_canonical_config.json hash ({manifest['hash']['value']!r}) -- "
            "refusing to replay against a config the dataset was not collected under."
        )


def _build_env():
    scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    gripper_map = JointMap.build(model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    cube_qpos_adr = int(model.jnt_qposadr[cube_joint_id])
    cube_dof_adr = int(model.jnt_dofadr[cube_joint_id])
    return model, data, arm_map, gripper_map, site_id, cube_body_id, cube_qpos_adr, cube_dof_adr


def _reset_and_init(model, data, cube_qpos_adr, cube_dof_adr, cube_xy_offset):
    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    guard = CubeInitGuard(data, cube_qpos_adr, cube_dof_adr)
    guard.set_initial_pose([cube_x, cube_y, cube_z])
    mujoco.mj_forward(model, data)
    return guard


def replay_exact_execution(hdf5_path: Path, variant_id: str) -> dict:
    """Mode 1 (Section C.1): replay the recorded per-physics-step
    `execution/applied_ctrl` vector directly, never re-deriving it."""
    with h5py.File(hdf5_path, "r") as f:
        _check_manifest(f)
        g = f["episodes"][variant_id]
        cube_xy_offset = tuple(g.attrs["cube_xy_offset"])
        stored_success = bool(g.attrs["success"])
        applied_ctrl = g["execution"]["applied_ctrl"][:]
        exec_joint_pos = g["execution"]["joint_positions"][:]
        exec_tcp_pose = g["execution"]["tcp_pose"][:]
        exec_cube_pos = g["execution"]["cube_pos"][:]
        final_obs_joint_pos = g["policy"]["observations"]["joint_positions"][-1]
        final_obs_tcp_pose = g["policy"]["observations"]["tcp_pose"][-1]

    model, data, arm_map, gripper_map, site_id, cube_body_id, cube_qpos_adr, cube_dof_adr = _build_env()
    guard = _reset_and_init(model, data, cube_qpos_adr, cube_dof_adr, cube_xy_offset)

    n = applied_ctrl.shape[0]
    replayed_joint_pos = np.zeros((n, arm_map.actuator_id.shape[0]), dtype=np.float64)
    replayed_tcp_pose = np.zeros((n, 7), dtype=np.float64)
    replayed_cube_pos = np.zeros((n, 3), dtype=np.float64)

    n_arm = len(arm_map.actuator_id)
    for i in range(n):
        arm_map.set_ctrl(data, applied_ctrl[i, :n_arm])
        gripper_map.set_ctrl(data, applied_ctrl[i, n_arm:])
        mujoco.mj_step(model, data)
        if i == 0:
            guard.lock()
        replayed_joint_pos[i] = arm_map.get_qpos(data)
        replayed_tcp_pose[i] = np.concatenate([
            data.site_xpos[site_id].copy(), _mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3)),
        ])
        replayed_cube_pos[i] = data.xpos[cube_body_id].copy()

    joint_err = np.abs(replayed_joint_pos - exec_joint_pos[:n])
    tcp_err = np.linalg.norm(replayed_tcp_pose[:, :3] - exec_tcp_pose[:n, :3], axis=1)
    cube_err = np.linalg.norm(replayed_cube_pos - exec_cube_pos[:n], axis=1)

    max_joint_err = float(joint_err.max())
    max_tcp_err = float(tcp_err.max())
    max_cube_err = float(cube_err.max())

    first_divergence_step = None
    for i in range(n):
        if joint_err[i].max() > EXACT_JOINT_TOL_RAD or tcp_err[i] > EXACT_TCP_TOL_M or cube_err[i] > EXACT_CUBE_TOL_M:
            first_divergence_step = i
            break

    final_joint_err = float(np.abs(replayed_joint_pos[-1] - final_obs_joint_pos).max())
    final_tcp_err = float(np.linalg.norm(replayed_tcp_pose[-1, :3] - final_obs_tcp_pose[:3]))

    within_tolerance = (
        max_joint_err <= EXACT_JOINT_TOL_RAD
        and max_tcp_err <= EXACT_TCP_TOL_M
        and max_cube_err <= EXACT_CUBE_TOL_M
    )

    return {
        "mode": "exact_execution",
        "variant_id": variant_id,
        "n_execution_rows_replayed": n,
        "max_joint_error_rad": max_joint_err,
        "max_tcp_error_m": max_tcp_err,
        "max_cube_error_m": max_cube_err,
        "final_state_vs_final_observation": {
            "joint_error_rad": final_joint_err,
            "tcp_error_m": final_tcp_err,
        },
        "first_divergence_execution_step": first_divergence_step,
        "tolerances": {"joint_rad": EXACT_JOINT_TOL_RAD, "tcp_m": EXACT_TCP_TOL_M, "cube_m": EXACT_CUBE_TOL_M},
        "within_tolerance": within_tolerance,
        "stored_success": stored_success,
        "note": (
            "This replays the literal recorded applied_ctrl vector at every physics "
            "step (no IK re-solve, no PD re-derivation) -- residual error, if any, is "
            "floating-point accumulation over the step count, not information loss "
            "from downsampling (contrast with Phase 5B's 10 Hz ZOH replay)."
        ),
    }


def _ramp_to_target(
    model, data, arm_map, gripper_map, ik_scratch, site_id, nominal_q,
    q_prev_target: np.ndarray, target_pos: np.ndarray, finger_open: bool,
    n_waypoints: int, n_substeps: int,
) -> np.ndarray:
    """Decoder-side ramp, structurally identical to
    run_pick_place._drive_smooth's inner loop (reusing the same public
    solve_ik_waypoint / bounded_pd_step / _finger_targets this project has
    always used -- no new IK or PD logic), applied over ONE 100 ms policy
    interval instead of a whole phase.
    """
    start_pos = data.site_xpos[site_id].copy()
    for i in range(n_waypoints):
        alpha = (i + 1) / n_waypoints
        waypoint = start_pos + alpha * (target_pos - start_pos)
        q_i, _, _ = solve_ik_waypoint(model, ik_scratch, data.qpos.copy(), arm_map, site_id, waypoint, nominal_q)
        for s in range(n_substeps):
            beta = (s + 1) / n_substeps
            ramped_target = q_prev_target + beta * (q_i - q_prev_target)
            arm_map.set_ctrl(data, ramped_target)
            bounded_pd_step(
                gripper_map, data, _finger_targets(gripper_map, finger_open),
                GRIPPER_KP_4E, GRIPPER_KD_4E, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL,
            )
            mujoco.mj_step(model, data)
        q_prev_target = q_i
    return q_prev_target


def replay_policy_actions(hdf5_path: Path, variant_id: str) -> dict:
    """Mode 2 (Section C.2): decode ONLY the 10 Hz high_level_actions
    stream through the shared ramp decoder above."""
    with h5py.File(hdf5_path, "r") as f:
        _check_manifest(f)
        g = f["episodes"][variant_id]
        cube_xy_offset = tuple(g.attrs["cube_xy_offset"])
        stored_success = bool(g.attrs["success"])
        cartesian_targets = g["policy"]["high_level_actions"]["cartesian_target"][:]
        gripper_open_cmds = g["policy"]["high_level_actions"]["gripper_command_open"][:]
        stored_tcp_pose = g["policy"]["observations"]["tcp_pose"][:]
        stored_cube_pos = g["privileged"]["cube_pos"][:]

    model, data, arm_map, gripper_map, site_id, cube_body_id, cube_qpos_adr, cube_dof_adr = _build_env()
    guard = _reset_and_init(model, data, cube_qpos_adr, cube_dof_adr, cube_xy_offset)
    ik_scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(arm_map.names))

    n_transitions = cartesian_targets.shape[0]
    n_substeps = SUBSTEPS_PER_TRANSITION // K_WAYPOINTS_PER_POLICY_STEP
    q_prev_target = arm_map.get_qpos(data).copy()

    replayed_tcp_pos = [data.site_xpos[site_id].copy()]
    replayed_cube_pos = [data.xpos[cube_body_id].copy()]

    for k in range(n_transitions):
        q_prev_target = _ramp_to_target(
            model, data, arm_map, gripper_map, ik_scratch, site_id, nominal_q,
            q_prev_target, cartesian_targets[k], bool(gripper_open_cmds[k]),
            K_WAYPOINTS_PER_POLICY_STEP, n_substeps,
        )
        if k == 0:
            guard.lock()
        replayed_tcp_pos.append(data.site_xpos[site_id].copy())
        replayed_cube_pos.append(data.xpos[cube_body_id].copy())

    replayed_tcp_pos = np.stack(replayed_tcp_pos)
    replayed_cube_pos = np.stack(replayed_cube_pos)

    n = min(replayed_tcp_pos.shape[0], stored_tcp_pose.shape[0])
    tcp_err = np.linalg.norm(replayed_tcp_pos[:n] - stored_tcp_pose[:n, :3], axis=1)
    cube_err = np.linalg.norm(replayed_cube_pos[:n] - stored_cube_pos[:n], axis=1)
    max_tcp_err = float(tcp_err.max())
    final_tcp_err = float(tcp_err[-1])
    final_cube_err = float(cube_err[-1])

    within_tolerance = max_tcp_err <= POLICY_TCP_TOL_M

    return {
        "mode": "policy_action",
        "variant_id": variant_id,
        "n_transitions_replayed": n_transitions,
        "decoder_k_waypoints_per_policy_step": K_WAYPOINTS_PER_POLICY_STEP,
        "max_tcp_error_m": max_tcp_err,
        "final_tcp_error_m": final_tcp_err,
        "final_cube_error_m": final_cube_err,
        "tolerance_tcp_m": POLICY_TCP_TOL_M,
        "within_tolerance": within_tolerance,
        "stored_success": stored_success,
        "note": (
            "Decodes ONLY the 10 Hz cartesian_target/gripper_command_open stream "
            "through a ramp reusing the same solve_ik_waypoint/bounded_pd_step this "
            "project's real controller uses, re-ramping toward the recorded static "
            "per-interval target every 100 ms instead of holding it constant "
            "(Phase 5B's bug). This does NOT reimplement the settle/verify gating "
            "state machine, so it cannot independently reproduce an early-abort "
            "failure_state -- for a trial that fails before any ramped segment "
            "begins (e.g. x_plus_0.03, which fails at SETTLE_APPROACH), there are "
            "no LIFT/TRANSPORT/LOWER intervals to ramp at all, so this metric is "
            "near-degenerate for that episode; see the per-episode report for that "
            "case's specific numbers."
        ),
    }


def visualize_episode(hdf5_path: Path, variant_id: str, out_path: Path | None = None, n_tiles: int = 6) -> Path:
    """Observation-only playback: unchanged in spirit from Phase 5B -- loads
    stored RGB frames and stored privileged/phase state, zero physics
    stepped."""
    import imageio.v2 as imageio

    with h5py.File(hdf5_path, "r") as f:
        _check_manifest(f)
        g = f["episodes"][variant_id]
        rgb = g["policy"]["observations"]["rgb"][:]
        phases = [p.decode("utf-8") for p in g["privileged"]["phase"][:]]

    n = rgb.shape[0]
    idxs = np.linspace(0, n - 1, min(n_tiles, n)).astype(int)
    tiles = [rgb[i] for i in idxs]
    tile_h, tile_w = tiles[0].shape[:2]
    sheet = np.zeros((tile_h, tile_w * len(tiles), 3), dtype=np.uint8)
    for k, tile in enumerate(tiles):
        sheet[:, k * tile_w:(k + 1) * tile_w] = tile

    out_path = out_path or (ROOT / "artifacts" / "phase5b_sample_frames" / f"{variant_id}_v2_visualize_contact_sheet.png")
    imageio.imwrite(out_path, sheet)
    print(f"wrote {out_path} (phases at tiles: {[phases[i] for i in idxs]})")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["exact", "policy", "visualize"])
    parser.add_argument("--dataset", default=str(ROOT / "data" / "task1_prototype_v2.hdf5"))
    parser.add_argument("--variant", required=True)
    args = parser.parse_args()

    if args.mode == "exact":
        result = replay_exact_execution(Path(args.dataset), args.variant)
    elif args.mode == "policy":
        result = replay_policy_actions(Path(args.dataset), args.variant)
    else:
        out = visualize_episode(Path(args.dataset), args.variant)
        result = {"visualization_path": str(out)}
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
