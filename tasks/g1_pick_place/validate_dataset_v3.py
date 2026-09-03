#!/usr/bin/env python3
"""Phase 5D: redesigned-action-schema HDF5 dataset validator.

Extends Phase 5C's validate_dataset_v2.py checks (required groups/
attributes, matching time dimensions, monotonic timestamps, finite values,
quaternion norms, image sanity, nonempty instruction, independently-
recomputed success, failure-episode training exclusion, reopen-after-close,
no post-init cube jump, transition alignment) to the v3
`policy/{observations,actions}` schema, and adds:

- decoder_configuration_hash present and matching the live
  policy_action_codec configuration (analogous to canonical_manifest_sha256)
- tcp_delta_position/tcp_delta_orientation have the expected [T, H, 3] chunk
  shape and gripper_command the expected [T] shape (not chunked)
- no-phase-goal-repetition: for any transition whose state_machine_phase
  indicates a smooth multi-waypoint segment (LIFT/TRANSPORT/LOWER), the
  per-transition action is NOT byte-identical to more than
  MAX_IDENTICAL_SMOOTH_RUN consecutive neighbors -- the direct regression
  check against Phase 5C's "static per-phase goal repeated ~40 times" bug
  (structurally impossible to recur here since the schema no longer has an
  absolute per-phase goal field at all, but this also catches a decoder/
  collector bug that accidentally reintroduced constant deltas)
- causality: next_arm_joint_target[k] (the auxiliary debug field) is
  reachable purely from execution/arm_joint_target within transition k's own
  interval, never from privileged cube/target state (checked structurally:
  privileged/ fields are never read by the action-derivation code path this
  validator re-invokes for the success recomputation, and next_arm_joint_target
  is cross-checked directly against execution/arm_joint_target's own last row)

Fails loudly (SystemExit(1)) on any required-check failure.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import h5py
import numpy as np

from tasks.g1_pick_place.camera_observation import CAM_HEIGHT, CAM_WIDTH, write_grasp_scene_5a
from tasks.g1_pick_place.canonical_config import load_manifest
from tasks.g1_pick_place.policy_action_codec import SUB_ACTIONS_PER_TRANSITION, decoder_configuration_hash
from tasks.g1_pick_place.record_demonstrations_v2 import SUBSTEPS_PER_TRANSITION
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[2]

REQUIRED_ROOT_ATTRS = [
    "schema_version", "action_schema_version", "action_frame", "decoder_version",
    "decoder_configuration_hash", "mujoco_version", "robot_embodiment",
    "unitree_mujoco_pinned_commit", "project_git_commit", "task_id", "task_instruction",
    "transition_convention", "policy_control_hz", "physics_hz", "execution_hz",
    "substeps_per_transition", "rgb_hz", "terminal_convention", "camera_params_json",
    "coordinate_conventions", "canonical_manifest_sha256",
]
REQUIRED_EPISODE_ATTRS = [
    "instruction", "variant_id", "seed", "success", "termination_reason",
    "train_eligible", "transition_count", "execution_row_count",
    "canonical_manifest_sha256", "decoder_configuration_hash",
]
REQUIRED_OBS_DATASETS = ["rgb", "joint_positions", "joint_velocities", "tcp_pose", "gripper_state", "timestamps"]
REQUIRED_ACTION_DATASETS = [
    "tcp_delta_position", "tcp_delta_orientation", "gripper_command",
    "next_arm_joint_target", "state_machine_phase",
]
REQUIRED_EXEC_DATASETS = [
    "transition_index", "timestamps", "arm_joint_target", "gripper_target", "applied_ctrl",
    "joint_positions", "joint_velocities", "tcp_pose", "cube_pos", "cube_quat", "phase",
]
REQUIRED_PRIV_DATASETS = ["cube_pos", "cube_quat", "target_pos", "phase", "commanded_ref_tcp_pos", "commanded_ref_tcp_quat"]

MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M = 0.05
MAX_IDENTICAL_SMOOTH_RUN = 3  # for LIFT/TRANSPORT/LOWER-labeled transitions only
SMOOTH_PHASE_PREFIXES = ("LIFT", "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET")


def _fail(msg: str, errors: list[str]) -> None:
    errors.append(msg)


def validate_file(path: Path) -> dict:
    errors: list[str] = []
    advisories: list[str] = []
    manifest = load_manifest()
    live_decoder_hash = decoder_configuration_hash(SUBSTEPS_PER_TRANSITION, SUB_ACTIONS_PER_TRANSITION)

    with h5py.File(path, "r") as f:
        for attr in REQUIRED_ROOT_ATTRS:
            if attr not in f.attrs:
                _fail(f"missing root attribute: {attr}", errors)
        if "canonical_manifest_sha256" in f.attrs and f.attrs["canonical_manifest_sha256"] != manifest["hash"]["value"]:
            _fail(
                f"root canonical_manifest_sha256 {f.attrs['canonical_manifest_sha256']!r} does not match "
                f"current manifest hash {manifest['hash']['value']!r}",
                errors,
            )
        if "decoder_configuration_hash" in f.attrs and f.attrs["decoder_configuration_hash"] != live_decoder_hash:
            _fail(
                f"root decoder_configuration_hash {f.attrs['decoder_configuration_hash']!r} does not match "
                f"live policy_action_codec configuration hash {live_decoder_hash!r}",
                errors,
            )

        if "episodes" not in f:
            _fail("missing top-level 'episodes' group", errors)
            return {"errors": errors, "advisories": advisories}

        substeps = int(f.attrs.get("substeps_per_transition", SUBSTEPS_PER_TRANSITION))
        h_chunk = SUB_ACTIONS_PER_TRANSITION
        train_eligible_count = 0
        train_ineligible_count = 0

        for ep_name in f["episodes"]:
            g = f["episodes"][ep_name]
            prefix = f"episode '{ep_name}': "

            for attr in REQUIRED_EPISODE_ATTRS:
                if attr not in g.attrs:
                    _fail(prefix + f"missing attribute {attr}", errors)
            if "decoder_configuration_hash" in g.attrs and g.attrs["decoder_configuration_hash"] != live_decoder_hash:
                _fail(prefix + "episode decoder_configuration_hash does not match live decoder config", errors)
            if "instruction" in g.attrs and not str(g.attrs["instruction"]).strip():
                _fail(prefix + "empty instruction string", errors)

            if "policy" not in g or "observations" not in g["policy"] or "actions" not in g["policy"]:
                _fail(prefix + "missing policy/observations or policy/actions group", errors)
                continue
            if "execution" not in g:
                _fail(prefix + "missing execution group", errors)
                continue

            obs = g["policy"]["observations"]
            act = g["policy"]["actions"]
            execu = g["execution"]
            priv = g.get("privileged")

            for ds_name in REQUIRED_OBS_DATASETS:
                if ds_name not in obs:
                    _fail(prefix + f"missing dataset policy/observations/{ds_name}", errors)
            for ds_name in REQUIRED_ACTION_DATASETS:
                if ds_name not in act:
                    _fail(prefix + f"missing dataset policy/actions/{ds_name}", errors)
            for ds_name in REQUIRED_EXEC_DATASETS:
                if ds_name not in execu:
                    _fail(prefix + f"missing dataset execution/{ds_name}", errors)
            if priv is not None:
                for ds_name in REQUIRED_PRIV_DATASETS:
                    if ds_name not in priv:
                        _fail(prefix + f"missing dataset privileged/{ds_name}", errors)

            n_obs = obs["rgb"].shape[0]
            n_act = act["gripper_command"].shape[0]
            n_exec = execu["transition_index"].shape[0]
            if n_obs != n_act + 1:
                _fail(prefix + f"observation/action count mismatch: {n_obs} observations, {n_act} actions", errors)
            if int(g.attrs.get("transition_count", -1)) != n_act:
                _fail(prefix + "transition_count attr != actual action count", errors)
            if int(g.attrs.get("execution_row_count", -1)) != n_exec:
                _fail(prefix + "execution_row_count attr != actual execution row count", errors)
            if n_exec != n_act * substeps:
                _fail(prefix + f"execution row count {n_exec} != n_transitions*substeps_per_transition", errors)

            if "tcp_delta_position" in act and act["tcp_delta_position"].shape != (n_act, h_chunk, 3):
                _fail(prefix + f"tcp_delta_position shape {act['tcp_delta_position'].shape} != expected ({n_act},{h_chunk},3)", errors)
            if "tcp_delta_orientation" in act and act["tcp_delta_orientation"].shape != (n_act, h_chunk, 3):
                _fail(prefix + f"tcp_delta_orientation shape {act['tcp_delta_orientation'].shape} != expected ({n_act},{h_chunk},3)", errors)
            if "gripper_command" in act and act["gripper_command"].shape != (n_act,):
                _fail(prefix + f"gripper_command shape {act['gripper_command'].shape} != expected ({n_act},)", errors)

            for ds_name in REQUIRED_OBS_DATASETS:
                if ds_name in obs and obs[ds_name].shape[0] != n_obs:
                    _fail(prefix + f"policy/observations/{ds_name} has wrong row count", errors)
            for ds_name in REQUIRED_EXEC_DATASETS:
                if ds_name in execu and execu[ds_name].shape[0] != n_exec:
                    _fail(prefix + f"execution/{ds_name} has wrong row count", errors)

            # --- transition alignment ---
            trans_idx = execu["transition_index"][:]
            if n_exec > 0:
                expected_trans_idx = np.repeat(np.arange(n_act), substeps)
                if not np.array_equal(trans_idx, expected_trans_idx):
                    _fail(prefix + "execution/transition_index is not the expected contiguous block structure", errors)

            obs_t = obs["timestamps"][:]
            exec_t = execu["timestamps"][:]
            if n_exec > 0 and n_obs == n_act + 1:
                for k in range(n_act):
                    block = exec_t[k * substeps:(k + 1) * substeps]
                    if block.size == 0:
                        continue
                    lo, hi = obs_t[k], obs_t[k + 1]
                    if block.min() <= lo - 1e-9 or block.max() > hi + 1e-9:
                        _fail(prefix + f"execution timestamps for transition {k} fall outside that transition's interval", errors)
                        break

            if n_exec > 0 and n_obs > 0:
                if not np.allclose(execu["joint_positions"][-1], obs["joint_positions"][-1], atol=1e-5):
                    _fail(prefix + "final execution joint_positions disagrees with final policy observation", errors)
                if not np.allclose(execu["tcp_pose"][-1, :3], obs["tcp_pose"][-1, :3], atol=1e-5):
                    _fail(prefix + "final execution TCP position disagrees with final policy observation", errors)

            # next_arm_joint_target[k] must equal execution/arm_joint_target's
            # own last row of transition k -- causality: reachable purely from
            # data generated within [k, k+1], never from privileged state.
            if n_exec > 0 and n_act > 0 and "next_arm_joint_target" in act:
                expected_next = execu["arm_joint_target"][substeps - 1::substeps][:n_act]
                if not np.allclose(act["next_arm_joint_target"][:], expected_next, atol=1e-6):
                    _fail(prefix + "next_arm_joint_target does not match execution/arm_joint_target's own recorded value at that transition's end -- action may be shifted or leaked from elsewhere", errors)

            # --- no-phase-goal-repetition regression check ---
            if "state_machine_phase" in act and "tcp_delta_position" in act and n_act > 0:
                phases = [p.decode("utf-8") for p in act["state_machine_phase"][:]]
                deltas = act["tcp_delta_position"][:]  # [T, H, 3]
                run_len = 1
                for k in range(1, n_act):
                    same_phase = any(phases[k].startswith(pfx) for pfx in SMOOTH_PHASE_PREFIXES) and phases[k] == phases[k - 1]
                    identical = np.array_equal(deltas[k], deltas[k - 1])
                    if same_phase and identical:
                        run_len += 1
                        if run_len > MAX_IDENTICAL_SMOOTH_RUN:
                            _fail(
                                prefix + f"tcp_delta_position is byte-identical across {run_len} consecutive "
                                f"'{phases[k]}' transitions -- looks like Phase 5C's static-per-phase-goal "
                                "bug may have recurred",
                                errors,
                            )
                            break
                    else:
                        run_len = 1

            timestamps = obs["timestamps"][:]
            if not np.all(np.diff(timestamps) > 0):
                _fail(prefix + "policy observation timestamps are not strictly monotonic increasing", errors)
            if n_exec > 1 and not np.all(np.diff(exec_t) > 0):
                _fail(prefix + "execution timestamps are not strictly monotonic increasing", errors)

            for arr_name, arr in (
                ("obs/joint_positions", obs["joint_positions"][:]),
                ("obs/joint_velocities", obs["joint_velocities"][:]),
                ("obs/tcp_pose", obs["tcp_pose"][:]),
                ("obs/gripper_state", obs["gripper_state"][:]),
                ("actions/tcp_delta_position", act["tcp_delta_position"][:]),
                ("actions/tcp_delta_orientation", act["tcp_delta_orientation"][:]),
                ("actions/gripper_command", act["gripper_command"][:]),
                ("execution/arm_joint_target", execu["arm_joint_target"][:]),
                ("execution/applied_ctrl", execu["applied_ctrl"][:]),
            ):
                if not np.all(np.isfinite(arr)):
                    _fail(prefix + f"{arr_name} contains non-finite values", errors)

            if "gripper_command" in act:
                gc = act["gripper_command"][:]
                if not np.all((gc == 0.0) | (gc == 1.0)):
                    _fail(prefix + "gripper_command contains values other than 0.0/1.0", errors)

            quats = obs["tcp_pose"][:, 3:7]
            norms = np.linalg.norm(quats, axis=1)
            if not np.allclose(norms, 1.0, atol=1e-3):
                _fail(prefix + f"TCP quaternion norms deviate from 1.0 (max |1-norm|={np.max(np.abs(norms-1)):.2e})", errors)
            if priv is not None and "cube_quat" in priv:
                cq_norms = np.linalg.norm(priv["cube_quat"][:], axis=1)
                if not np.allclose(cq_norms, 1.0, atol=1e-3):
                    _fail(prefix + f"cube quaternion norms deviate from 1.0 (max |1-norm|={np.max(np.abs(cq_norms-1)):.2e})", errors)

            rgb = obs["rgb"]
            if rgb.dtype != np.uint8:
                _fail(prefix + f"rgb dtype is {rgb.dtype}, expected uint8", errors)
            rgb_arr = rgb[:]
            if rgb_arr.shape[1:] != (CAM_HEIGHT, CAM_WIDTH, 3):
                _fail(prefix + f"rgb shape {rgb_arr.shape[1:]} != expected ({CAM_HEIGHT},{CAM_WIDTH},3)", errors)
            if rgb_arr.std() < 1.0:
                _fail(prefix + f"rgb frames appear blank (std={rgb_arr.std():.3f})", errors)

            success_attr = bool(g.attrs["success"])
            train_eligible_attr = bool(g.attrs["train_eligible"])
            if success_attr:
                train_eligible_count += 1
            else:
                train_ineligible_count += 1
                if train_eligible_attr:
                    _fail(prefix + "success=False but train_eligible=True", errors)

            offset = tuple(g.attrs["cube_xy_offset"])
            scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
            recomputed = run_trial_pick_place(
                scene_path, cube_xy_offset=offset, gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
            )
            if bool(recomputed["task_pass"]) != success_attr:
                _fail(
                    prefix + f"stored success={success_attr} disagrees with independently recomputed "
                    f"task_pass={recomputed['task_pass']} for offset {offset}",
                    errors,
                )

            if priv is not None and "cube_pos" in priv:
                cube_pos_arr = priv["cube_pos"][:]
                deltas_cube = np.linalg.norm(np.diff(cube_pos_arr, axis=0), axis=1)
                if np.any(deltas_cube > MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M):
                    _fail(prefix + f"cube_pos jumped {deltas_cube.max():.4f} m between consecutive recorded transitions", errors)

        if train_eligible_count == 0:
            advisories.append("no train-eligible (success=True) episodes in dataset")
        if train_ineligible_count == 0:
            advisories.append("no excluded (success=False) failure episode present in dataset")

    try:
        with h5py.File(path, "r") as f2:
            _ = list(f2["episodes"].keys())
    except Exception as e:
        _fail(f"dataset failed to reopen after close: {e}", errors)

    return {"errors": errors, "advisories": advisories}


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "data" / "task1_prototype_v3.hdf5"
    if not path.exists():
        print(f"dataset not found: {path}")
        return 1
    result = validate_file(path)
    print(json.dumps(result, indent=2))
    if result["errors"]:
        print(f"VALIDATION FAILED: {len(result['errors'])} error(s)")
        return 1
    print("VALIDATION PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
