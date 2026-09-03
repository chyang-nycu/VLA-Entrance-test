#!/usr/bin/env python3
"""Phase 5B: demonstration replay (Section E).

Two independent modes:

1. Action replay (`replay_actions`): restores ONLY the recorded initial
   condition (cube spawn offset, via the same CubeInitGuard pre-lock
   boundary every other module in this project uses -- never overwritten
   again after that single set), then applies the episode's RECORDED
   actions through real MuJoCo physics, comparing the resulting
   joint/TCP/cube trajectory against what was originally recorded.

   Recorded actions are stored at the 10 Hz POLICY_HZ rate (Section A);
   each recorded `arm_joint_position_target`/`gripper_target` is the value
   that was active at the END of its 50-physics-step block, not a
   per-physics-step trace. Replaying it as a zero-order hold across the
   whole block (the only thing a stored 10 Hz action stream supports) is
   therefore NOT expected to bit-reproduce the original trajectory during
   segments that used fine per-step waypoint ramping (LIFT/TRANSPORT_ABOVE_
   TARGET/LOWER_TO_TARGET, see run_pick_place._drive_smooth) -- this is a
   real, expected, and honestly measured consequence of downsampling the
   action stream for dataset storage, not a replay bug. This function
   reports the resulting deviation rather than assuming it is zero.

2. Observation-only visualization replay (`visualize_episode`): plays back
   stored RGB/state WITHOUT stepping physics at all -- for inspecting what
   a policy would have seen, independent of any replay fidelity question.

Both modes load the canonical manifest and check the dataset's stored
`canonical_manifest_sha256` against it; a mismatch raises
(`canonical_config.ManifestMismatchError`) rather than silently replaying
against a config the dataset was not actually collected under.
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
from tasks.g1_pick_place.gripper_scene import CUBE_POS
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E

ROOT = Path(__file__).resolve().parents[2]

# Replay tolerances (documented, not asserted as physics ground truth): a
# zero-order-hold replay of a 10 Hz downsampled action stream against a
# 500 Hz fine-grained original is expected to deviate somewhat during
# ramped segments; these bounds are generous relative to the workspace
# scale (cube half-extent 35mm, target pad half-extent 50mm) so that a
# genuine replay-pipeline bug (e.g. wrong gains, wrong initial condition)
# would still be caught, while normal zero-order-hold deviation is not
# misreported as a failure.
REPLAY_JOINT_TOL_RAD = 0.35
REPLAY_TCP_TOL_M = 0.08
REPLAY_CUBE_TOL_M = 0.08


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


def replay_actions(hdf5_path: Path, variant_id: str) -> dict:
    with h5py.File(hdf5_path, "r") as f:
        _check_manifest(f)
        substeps = int(f.attrs["substeps_per_transition"])
        g = f["episodes"][variant_id]
        cube_xy_offset = tuple(g.attrs["cube_xy_offset"])
        stored_success = bool(g.attrs["success"])
        arm_targets = g["actions"]["arm_joint_position_target"][:]
        gripper_targets = g["actions"]["gripper_target"][:]
        stored_joint_pos = g["policy_observations"]["joint_positions"][:]
        stored_tcp_pose = g["policy_observations"]["tcp_pose"][:]
        stored_cube_pos = g["privileged"]["cube_pos"][:]

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

    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    guard = CubeInitGuard(data, cube_qpos_adr, cube_dof_adr)
    guard.set_initial_pose([cube_x, cube_y, cube_z])
    mujoco.mj_forward(model, data)
    # ONLY this one set_initial_pose call is permitted -- guard.lock() below
    # makes any further cube-state write raise, exactly like every real
    # trial in this project.

    n_transitions = arm_targets.shape[0]
    replayed_joint_pos = [arm_map.get_qpos(data).copy()]
    replayed_tcp_pose = [np.concatenate([data.site_xpos[site_id].copy(), _mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3))])]
    replayed_cube_pos = [data.xpos[cube_body_id].copy()]

    first_step = True
    for i in range(n_transitions):
        arm_map.set_ctrl(data, arm_targets[i])
        for _ in range(substeps):
            bounded_pd_step(gripper_map, data, gripper_targets[i], GRIPPER_KP_4E, GRIPPER_KD_4E, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL)
            mujoco.mj_step(model, data)
            if first_step:
                guard.lock()
                first_step = False
        replayed_joint_pos.append(arm_map.get_qpos(data).copy())
        replayed_tcp_pose.append(np.concatenate([data.site_xpos[site_id].copy(), _mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3))]))
        replayed_cube_pos.append(data.xpos[cube_body_id].copy())

    replayed_joint_pos = np.stack(replayed_joint_pos)
    replayed_tcp_pose = np.stack(replayed_tcp_pose)
    replayed_cube_pos = np.stack(replayed_cube_pos)

    n = min(replayed_joint_pos.shape[0], stored_joint_pos.shape[0])
    joint_err = np.abs(replayed_joint_pos[:n] - stored_joint_pos[:n])
    tcp_err = np.linalg.norm(replayed_tcp_pose[:n, :3] - stored_tcp_pose[:n, :3], axis=1)
    cube_err = np.linalg.norm(replayed_cube_pos[:n] - stored_cube_pos[:n], axis=1)

    max_joint_err = float(joint_err.max())
    max_tcp_err = float(tcp_err.max())
    max_cube_err = float(cube_err.max())
    final_cube_err = float(cube_err[-1])

    within_tolerance = (
        max_joint_err <= REPLAY_JOINT_TOL_RAD
        and max_tcp_err <= REPLAY_TCP_TOL_M
        and max_cube_err <= REPLAY_CUBE_TOL_M
    )

    return {
        "variant_id": variant_id,
        "n_transitions_replayed": n_transitions,
        "max_joint_error_rad": max_joint_err,
        "max_tcp_error_m": max_tcp_err,
        "max_cube_error_m": max_cube_err,
        "final_cube_error_m": final_cube_err,
        "tolerances": {
            "joint_rad": REPLAY_JOINT_TOL_RAD,
            "tcp_m": REPLAY_TCP_TOL_M,
            "cube_m": REPLAY_CUBE_TOL_M,
        },
        "within_tolerance": within_tolerance,
        "stored_success": stored_success,
        "note": (
            "Deviation is expected: recorded actions are a 10 Hz zero-order-hold "
            "downsample of the original 500 Hz fine-grained waypoint-ramped control "
            "(see run_pick_place._drive_smooth); this is not a replay-pipeline bug."
        ),
    }


def visualize_episode(hdf5_path: Path, variant_id: str, out_path: Path | None = None, n_tiles: int = 6) -> Path:
    """Observation-only playback: loads stored RGB frames and stored
    privileged/phase state, tiles a handful of evenly-spaced frames into one
    contact-sheet PNG for inspection. No physics is stepped.
    """
    import imageio.v2 as imageio

    with h5py.File(hdf5_path, "r") as f:
        _check_manifest(f)
        g = f["episodes"][variant_id]
        rgb = g["policy_observations"]["rgb"][:]
        phases = [p.decode("utf-8") for p in g["privileged"]["phase"][:]]

    n = rgb.shape[0]
    idxs = np.linspace(0, n - 1, min(n_tiles, n)).astype(int)
    tiles = [rgb[i] for i in idxs]
    tile_h, tile_w = tiles[0].shape[:2]
    sheet = np.zeros((tile_h, tile_w * len(tiles), 3), dtype=np.uint8)
    for k, tile in enumerate(tiles):
        sheet[:, k * tile_w:(k + 1) * tile_w] = tile

    out_path = out_path or (ROOT / "artifacts" / "phase5b_sample_frames" / f"{variant_id}_visualize_contact_sheet.png")
    imageio.imwrite(out_path, sheet)
    print(f"wrote {out_path} (phases at tiles: {[phases[i] for i in idxs]})")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["replay", "visualize"])
    parser.add_argument("--dataset", default=str(ROOT / "data" / "task1_prototype.hdf5"))
    parser.add_argument("--variant", required=True)
    args = parser.parse_args()

    if args.mode == "replay":
        result = replay_actions(Path(args.dataset), args.variant)
    else:
        out = visualize_episode(Path(args.dataset), args.variant)
        result = {"visualization_path": str(out)}
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
