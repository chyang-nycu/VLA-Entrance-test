#!/usr/bin/env python3
"""Phase 5D: replay for the redesigned reference-relative TCP-delta actions.

Three modes, same distinction as Phase 5C:

1. Exact execution replay -- IDENTICAL logic to Phase 5C's (the `execution/`
   group's schema and semantics are unchanged in this phase), duplicated
   here (not imported) only so this module is self-contained and its own
   regression test can assert byte-level behavioral equivalence against
   Phase 5C's implementation independently.

2. Policy-action replay (`replay_policy_actions`) -- THE fix this phase
   exists for. Maintains its own internal "commanded reference" TCP
   pose (position + quaternion), seeded ONCE from the measured state at
   t=0 (unavoidable -- there is no earlier reference), then evolved PURELY
   by composing each recorded action's delta onto the previous commanded
   reference (never re-read from measured/noisy physics state mid-episode,
   per Section B). Each interval is decoded with exactly ONE IK solve and
   ONE linear joint-space ramp across the full 50-physics-step interval
   (`policy_action_codec.ramp_joint_targets`) -- no mid-interval
   re-targeting.

3. Observation-only visualization replay -- unchanged in spirit.

All three verify BOTH the canonical manifest hash and this dataset's own
`decoder_configuration_hash` against the live decoder config, raising on
either mismatch.
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
from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS, TARGET_XY_SUCCESS_MARGIN_M
from tasks.g1_pick_place.policy_action_codec import (
    SUB_ACTIONS_PER_TRANSITION, decode_target, decoder_configuration_hash, mat_to_quat_wxyz,
    ramp_joint_targets,
)
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard
from tasks.g1_pick_place.run_grasp_test_3c import _finger_targets

ROOT = Path(__file__).resolve().parents[2]

TIMESTEP = 0.002
POLICY_HZ = 10.0
SUBSTEPS_PER_TRANSITION = int(round((1.0 / TIMESTEP) / POLICY_HZ))  # 50

EXACT_JOINT_TOL_RAD = 1e-4
EXACT_TCP_TOL_M = 1e-3
EXACT_CUBE_TOL_M = 1e-3

POLICY_TCP_TOL_M = 0.010

ARM_KP_4B = 400.0
ARM_KV_4B = 25.0
GRIPPER_KP_4E = 320.0
GRIPPER_KD_4E = 20.0


class DecoderMismatchError(Exception):
    pass


def _check_hashes(f: h5py.File) -> None:
    manifest = load_manifest()
    stored_manifest = f.attrs.get("canonical_manifest_sha256")
    if stored_manifest != manifest["hash"]["value"]:
        raise ManifestMismatchError(
            f"dataset's canonical_manifest_sha256 ({stored_manifest!r}) does not match the "
            f"current data/task1_canonical_config.json hash ({manifest['hash']['value']!r})."
        )
    stored_decoder = f.attrs.get("decoder_configuration_hash")
    live_decoder = decoder_configuration_hash(SUBSTEPS_PER_TRANSITION)
    if stored_decoder != live_decoder:
        raise DecoderMismatchError(
            f"dataset's decoder_configuration_hash ({stored_decoder!r}) does not match the "
            f"live policy_action_codec decoder configuration ({live_decoder!r}) -- refusing "
            "to replay policy actions against a decoder the dataset was not built for."
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
    """Unchanged from Phase 5C -- see module docstring point 1."""
    with h5py.File(hdf5_path, "r") as f:
        _check_hashes(f)
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
            data.site_xpos[site_id].copy(), mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3)),
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
        "final_state_vs_final_observation": {"joint_error_rad": final_joint_err, "tcp_error_m": final_tcp_err},
        "first_divergence_execution_step": first_divergence_step,
        "tolerances": {"joint_rad": EXACT_JOINT_TOL_RAD, "tcp_m": EXACT_TCP_TOL_M, "cube_m": EXACT_CUBE_TOL_M},
        "within_tolerance": within_tolerance,
        "stored_success": stored_success,
        "note": "Unchanged from Phase 5C -- literal applied_ctrl replay, no IK re-solve.",
    }


def replay_policy_actions(hdf5_path: Path, variant_id: str) -> dict:
    """Mode 2: decode ONLY policy/actions/{tcp_delta_position,
    tcp_delta_orientation, gripper_command} through the reference-relative
    single-ramp decoder. Does not reimplement the state-machine's SETTLE/
    VERIFY gating (same disclosed limitation as Phase 5C) -- physical
    outcome is judged by final cube/TCP position agreement, not an
    independently re-run success detector.
    """
    with h5py.File(hdf5_path, "r") as f:
        _check_hashes(f)
        g = f["episodes"][variant_id]
        cube_xy_offset = tuple(g.attrs["cube_xy_offset"])
        stored_success = bool(g.attrs["success"])
        tcp_delta_position = g["policy"]["actions"]["tcp_delta_position"][:]
        tcp_delta_orientation = g["policy"]["actions"]["tcp_delta_orientation"][:]
        gripper_command = g["policy"]["actions"]["gripper_command"][:]
        stored_tcp_pose = g["policy"]["observations"]["tcp_pose"][:]
        stored_joint_pos = g["policy"]["observations"]["joint_positions"][:]
        stored_cube_pos = g["privileged"]["cube_pos"][:]
        state_machine_phase = [p.decode("utf-8") for p in g["policy"]["actions"]["state_machine_phase"][:]]

    model, data, arm_map, gripper_map, site_id, cube_body_id, cube_qpos_adr, cube_dof_adr = _build_env()
    guard = _reset_and_init(model, data, cube_qpos_adr, cube_dof_adr, cube_xy_offset)
    ik_scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(arm_map.names))

    n_transitions = tcp_delta_position.shape[0]
    h = tcp_delta_position.shape[1] if tcp_delta_position.ndim == 3 else 1
    sub_steps = SUBSTEPS_PER_TRANSITION // h

    current_joint_target = arm_map.get_qpos(data).copy()
    commanded_ref_pos = data.site_xpos[site_id].copy().astype(np.float64)
    commanded_ref_quat = mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3)).astype(np.float64)

    replayed_tcp_pos = [data.site_xpos[site_id].copy()]
    replayed_joint_pos = [arm_map.get_qpos(data).copy()]
    replayed_cube_pos = [data.xpos[cube_body_id].copy()]

    for k in range(n_transitions):
        finger_open = bool(gripper_command[k] < 0.5)
        for sub in range(h):
            sub_delta_pos = tcp_delta_position[k, sub] if h > 1 else tcp_delta_position[k]
            sub_delta_rot = tcp_delta_orientation[k, sub] if h > 1 else tcp_delta_orientation[k]
            target_pos = commanded_ref_pos + sub_delta_pos.astype(np.float64)
            ramp, q_target = ramp_joint_targets(
                model, ik_scratch, data.qpos.copy(), arm_map, site_id, nominal_q,
                current_joint_target, target_pos, sub_steps,
            )
            for s in range(sub_steps):
                arm_map.set_ctrl(data, ramp[s])
                bounded_pd_step(
                    gripper_map, data, _finger_targets(gripper_map, finger_open),
                    GRIPPER_KP_4E, GRIPPER_KD_4E, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL,
                )
                mujoco.mj_step(model, data)
                if k == 0 and sub == 0 and s == 0:
                    guard.lock()

            current_joint_target = q_target
            # Commanded reference evolves PURELY from the composed delta --
            # never re-read from measured/noisy data.site_xpos, per Section B.
            commanded_ref_pos, commanded_ref_quat = decode_target(
                commanded_ref_pos, commanded_ref_quat, sub_delta_pos, sub_delta_rot,
            )

        replayed_tcp_pos.append(data.site_xpos[site_id].copy())
        replayed_joint_pos.append(arm_map.get_qpos(data).copy())
        replayed_cube_pos.append(data.xpos[cube_body_id].copy())

    replayed_tcp_pos = np.stack(replayed_tcp_pos)
    replayed_joint_pos = np.stack(replayed_joint_pos)
    replayed_cube_pos = np.stack(replayed_cube_pos)

    n = min(replayed_tcp_pos.shape[0], stored_tcp_pose.shape[0])
    tcp_err = np.linalg.norm(replayed_tcp_pos[:n] - stored_tcp_pose[:n, :3], axis=1)
    joint_err = np.max(np.abs(replayed_joint_pos[:n] - stored_joint_pos[:n]), axis=1)
    cube_err = np.linalg.norm(replayed_cube_pos[:n] - stored_cube_pos[:n], axis=1)

    max_tcp_err = float(tcp_err.max())
    max_joint_err = float(joint_err.max())
    max_cube_err = float(cube_err.max())
    final_tcp_err = float(tcp_err[-1])
    final_joint_err = float(joint_err[-1])
    final_cube_err = float(cube_err[-1])

    replayed_final_xy_err = float(np.linalg.norm(replayed_cube_pos[-1, :2] - np.array(TARGET_POS)))
    target_placement_within_margin = bool(replayed_final_xy_err <= TARGET_XY_SUCCESS_MARGIN_M)

    # tcp_err is indexed by BOUNDARY: tcp_err[0] compares the initial state
    # to itself (~0 always); tcp_err[j] for j>=1 compares the state AFTER
    # transition (j-1) was applied. So the first boundary exceeding
    # tolerance at index j corresponds to TRANSITION (j-1), not transition j.
    first_divergence_boundary = None
    for k in range(n):
        if tcp_err[k] > POLICY_TCP_TOL_M:
            first_divergence_boundary = k
            break
    first_divergence_transition = (
        first_divergence_boundary - 1 if first_divergence_boundary and first_divergence_boundary >= 1 else first_divergence_boundary
    )

    # Accumulated delta drift: does error grow monotonically, or stay bounded?
    # Reported as the error at quartiles of the episode plus whether the
    # running max is non-decreasing throughout (a monotonic-drift signature).
    quartile_idxs = sorted(set(int(round(q * (n - 1))) for q in (0.0, 0.25, 0.5, 0.75, 1.0)))
    drift_profile = {f"tcp_err_at_frac_{q:.2f}_m": float(tcp_err[quartile_idxs[i]]) for i, q in enumerate((0.0, 0.25, 0.5, 0.75, 1.0))}
    running_max = np.maximum.accumulate(tcp_err)
    monotonic_nondecreasing = bool(np.all(np.diff(running_max) >= -1e-12))

    within_tolerance = max_tcp_err <= POLICY_TCP_TOL_M

    return {
        "mode": "policy_action",
        "variant_id": variant_id,
        "n_transitions_replayed": n_transitions,
        "max_tcp_error_m": max_tcp_err,
        "max_joint_error_rad": max_joint_err,
        "max_cube_error_m": max_cube_err,
        "final_tcp_error_m": final_tcp_err,
        "final_joint_error_rad": final_joint_err,
        "final_cube_error_m": final_cube_err,
        "replayed_final_xy_target_error_m": replayed_final_xy_err,
        "target_placement_within_margin": target_placement_within_margin,
        "tolerance_tcp_m": POLICY_TCP_TOL_M,
        "within_tolerance": within_tolerance,
        "stored_success": stored_success,
        "first_divergence_transition": first_divergence_transition,
        "first_divergence_phase": state_machine_phase[first_divergence_transition] if first_divergence_transition is not None and first_divergence_transition < len(state_machine_phase) else None,
        "accumulated_delta_drift": {
            "quartile_tcp_error_profile": drift_profile,
            "running_max_is_monotonic_nondecreasing": monotonic_nondecreasing,
            "interpretation": (
                "monotonic-nondecreasing running-max is consistent with (but does "
                "not prove) unbounded accumulation; a profile that rises then falls "
                "indicates bounded, self-correcting error instead"
            ),
        },
        "state_machine_note": (
            "This replay does not reimplement the state machine's SETTLE/VERIFY "
            "gating (same disclosed limitation as Phase 5C) -- 'state-machine "
            "phase agreement' is reported via the recorded state_machine_phase "
            "metadata field (ground truth from the original collection) aligned "
            "to each transition index, not independently re-derived. Physical "
            "success/placement outcome is judged via final cube position "
            "agreement (target_placement_within_margin) rather than an "
            "independently re-run success-detector boolean."
        ),
        "action_chunk_h": h,
        "sub_action_hz": (1.0 / TIMESTEP) / sub_steps,
        "note": (
            "Decodes ONLY policy/actions/{tcp_delta_position,tcp_delta_orientation,"
            "gripper_command} through a chunked ramp decoder (H sub-actions per "
            "10Hz transition) that composes each sub-delta onto an "
            "internally-tracked commanded reference (never re-read from "
            "measured state mid-episode) -- the direct fix for Phase 5C's "
            "repeated-static-phase-goal bug, and for Phase 5D attempts 1-2's "
            "single-whole-interval-delta shortfall (see "
            "reports/phase5d-policy-action-redesign.md)."
        ),
    }


def visualize_episode(hdf5_path: Path, variant_id: str, out_path: Path | None = None, n_tiles: int = 6) -> Path:
    import imageio.v2 as imageio

    with h5py.File(hdf5_path, "r") as f:
        _check_hashes(f)
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

    out_path = out_path or (ROOT / "artifacts" / "phase5b_sample_frames" / f"{variant_id}_v3_visualize_contact_sheet.png")
    imageio.imwrite(out_path, sheet)
    print(f"wrote {out_path} (phases at tiles: {[phases[i] for i in idxs]})")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["exact", "policy", "visualize"])
    parser.add_argument("--dataset", default=str(ROOT / "data" / "task1_prototype_v3.hdf5"))
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
