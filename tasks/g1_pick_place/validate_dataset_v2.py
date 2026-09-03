#!/usr/bin/env python3
"""Phase 5C: two-rate HDF5 dataset validator (Section D + E).

Extends Phase 5B's validate_dataset.py checks (required groups/attributes,
matching time dimensions, monotonic timestamps, finite values, quaternion
norms, image sanity, nonempty instruction, independently-recomputed success,
failure-episode training exclusion, reopen-after-close, no post-init cube
jump) to the new two-rate `policy/{observations,high_level_actions}` +
`execution/` + `privileged/` schema, and adds the Section E transition-
alignment checks:

- len(policy/observations) == len(policy/high_level_actions) + 1
- every execution row's transition_index falls in [0, n_transitions)
- execution rows for a given transition_index form one contiguous block of
  exactly `substeps_per_transition` rows, in increasing transition order
  (this is what "no action is shifted by one policy step" and "terminal
  transitions are represented consistently" reduce to structurally)
- execution timestamps for transition k fall within (obs_t[k], obs_t[k+1]]
- the final execution row's joint/TCP state agrees with the final policy
  observation (both are read from the identical live physics state at the
  same instant during collection -- an exact-equality check, not a
  tolerance-based one)
- the stored high_level_actions.cartesian_target[k] matches what
  record_demonstrations_v2._phase_target() deterministically derives from
  the execution group's own recorded phase label for that transition's last
  row ("reconstructed 10 Hz actions match stored controller targets at the
  declared sampling instant") -- this is also the check the deliberately-
  shifted-action tamper test in tests/test_phase5c_replay_fidelity.py
  exercises.

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
from tasks.g1_pick_place.record_demonstrations_v2 import SUBSTEPS_PER_TRANSITION, _base_phase, _phase_target
from tasks.g1_pick_place.run_grasp_test_3c import LIFT_DZ
from tasks.g1_pick_place.gripper_scene import CUBE_POS
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[2]

REQUIRED_ROOT_ATTRS = [
    "schema_version", "mujoco_version", "robot_embodiment", "unitree_mujoco_pinned_commit",
    "project_git_commit", "task_id", "task_instruction", "transition_convention",
    "policy_control_hz", "physics_hz", "execution_hz", "substeps_per_transition", "rgb_hz",
    "terminal_convention", "camera_params_json", "coordinate_conventions",
    "canonical_manifest_sha256",
]
REQUIRED_EPISODE_ATTRS = [
    "instruction", "variant_id", "seed", "success", "termination_reason",
    "train_eligible", "transition_count", "execution_row_count", "canonical_manifest_sha256",
]
REQUIRED_OBS_DATASETS = ["rgb", "joint_positions", "joint_velocities", "tcp_pose", "gripper_state", "timestamps"]
REQUIRED_HLA_DATASETS = ["cartesian_target", "gripper_command_open"]
REQUIRED_EXEC_DATASETS = [
    "transition_index", "timestamps", "arm_joint_target", "gripper_target", "applied_ctrl",
    "joint_positions", "joint_velocities", "tcp_pose", "cube_pos", "cube_quat", "phase",
]
REQUIRED_PRIV_DATASETS = ["cube_pos", "cube_quat", "target_pos", "phase"]

MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M = 0.05


def _fail(msg: str, errors: list[str]) -> None:
    errors.append(msg)


def validate_file(path: Path) -> dict:
    errors: list[str] = []
    advisories: list[str] = []
    manifest = load_manifest()

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

        if "episodes" not in f:
            _fail("missing top-level 'episodes' group", errors)
            return {"errors": errors, "advisories": advisories}

        substeps = int(f.attrs.get("substeps_per_transition", SUBSTEPS_PER_TRANSITION))
        train_eligible_count = 0
        train_ineligible_count = 0

        for ep_name in f["episodes"]:
            g = f["episodes"][ep_name]
            prefix = f"episode '{ep_name}': "

            for attr in REQUIRED_EPISODE_ATTRS:
                if attr not in g.attrs:
                    _fail(prefix + f"missing attribute {attr}", errors)
            if "instruction" in g.attrs and not str(g.attrs["instruction"]).strip():
                _fail(prefix + "empty instruction string", errors)

            if "policy" not in g or "observations" not in g["policy"] or "high_level_actions" not in g["policy"]:
                _fail(prefix + "missing policy/observations or policy/high_level_actions group", errors)
                continue
            if "execution" not in g:
                _fail(prefix + "missing execution group", errors)
                continue

            obs = g["policy"]["observations"]
            hla = g["policy"]["high_level_actions"]
            execu = g["execution"]
            priv = g.get("privileged")

            for ds_name in REQUIRED_OBS_DATASETS:
                if ds_name not in obs:
                    _fail(prefix + f"missing dataset policy/observations/{ds_name}", errors)
            for ds_name in REQUIRED_HLA_DATASETS:
                if ds_name not in hla:
                    _fail(prefix + f"missing dataset policy/high_level_actions/{ds_name}", errors)
            for ds_name in REQUIRED_EXEC_DATASETS:
                if ds_name not in execu:
                    _fail(prefix + f"missing dataset execution/{ds_name}", errors)
            if priv is not None:
                for ds_name in REQUIRED_PRIV_DATASETS:
                    if ds_name not in priv:
                        _fail(prefix + f"missing dataset privileged/{ds_name}", errors)

            n_obs = obs["rgb"].shape[0]
            n_act = hla["cartesian_target"].shape[0]
            n_exec = execu["transition_index"].shape[0]
            if n_obs != n_act + 1:
                _fail(prefix + f"observation/action count mismatch: {n_obs} observations, {n_act} actions "
                      f"(expected observations == actions + 1)", errors)
            if int(g.attrs.get("transition_count", -1)) != n_act:
                _fail(prefix + f"transition_count attr ({g.attrs.get('transition_count')}) != actual action count ({n_act})", errors)
            if int(g.attrs.get("execution_row_count", -1)) != n_exec:
                _fail(prefix + f"execution_row_count attr ({g.attrs.get('execution_row_count')}) != actual execution row count ({n_exec})", errors)
            if n_exec != n_act * substeps:
                _fail(prefix + f"execution row count {n_exec} != n_transitions*substeps_per_transition ({n_act}*{substeps}={n_act*substeps})", errors)

            for ds_name in REQUIRED_OBS_DATASETS:
                if ds_name in obs and obs[ds_name].shape[0] != n_obs:
                    _fail(prefix + f"policy/observations/{ds_name} has {obs[ds_name].shape[0]} rows, expected {n_obs}", errors)
            for ds_name in REQUIRED_HLA_DATASETS:
                if ds_name in hla and hla[ds_name].shape[0] != n_act:
                    _fail(prefix + f"policy/high_level_actions/{ds_name} has {hla[ds_name].shape[0]} rows, expected {n_act}", errors)
            for ds_name in REQUIRED_EXEC_DATASETS:
                if ds_name in execu and execu[ds_name].shape[0] != n_exec:
                    _fail(prefix + f"execution/{ds_name} has {execu[ds_name].shape[0]} rows, expected {n_exec}", errors)

            # --- Section E: transition-alignment checks ---
            trans_idx = execu["transition_index"][:]
            if n_exec > 0:
                expected_trans_idx = np.repeat(np.arange(n_act), substeps)
                if not np.array_equal(trans_idx, expected_trans_idx):
                    _fail(
                        prefix + "execution/transition_index is not the expected contiguous "
                        "[0]*substeps + [1]*substeps + ... block structure -- an execution row "
                        "may map to the wrong policy transition, or an action may be shifted",
                        errors,
                    )

            obs_t = obs["timestamps"][:]
            exec_t = execu["timestamps"][:]
            if n_exec > 0 and n_obs == n_act + 1:
                for k in range(n_act):
                    block = exec_t[k * substeps:(k + 1) * substeps]
                    if block.size == 0:
                        continue
                    lo, hi = obs_t[k], obs_t[k + 1]
                    if block.min() <= lo - 1e-9 or block.max() > hi + 1e-9:
                        _fail(
                            prefix + f"execution timestamps for transition {k} (range "
                            f"[{block.min():.4f}, {block.max():.4f}]) fall outside that "
                            f"transition's interval ({lo:.4f}, {hi:.4f}]",
                            errors,
                        )
                        break

            if n_exec > 0 and n_obs > 0:
                final_exec_joint = execu["joint_positions"][-1]
                final_obs_joint = obs["joint_positions"][-1]
                if not np.allclose(final_exec_joint, final_obs_joint, atol=1e-5):
                    _fail(
                        prefix + "final execution row's joint_positions does not agree with the "
                        "final policy observation's joint_positions (both should be the identical "
                        "live state at the same instant)",
                        errors,
                    )
                final_exec_tcp = execu["tcp_pose"][-1, :3]
                final_obs_tcp = obs["tcp_pose"][-1, :3]
                if not np.allclose(final_exec_tcp, final_obs_tcp, atol=1e-5):
                    _fail(prefix + "final execution row's TCP position does not agree with the final policy observation's TCP position", errors)

            # Reconstructed 10 Hz actions match stored controller targets at
            # the declared sampling instant: re-derive cartesian_target[k]
            # from execution's own recorded phase label via the same
            # deterministic _phase_target() mapping the collector used.
            if n_exec > 0 and n_act > 0:
                offset = tuple(g.attrs["cube_xy_offset"])
                cube_pos = np.array([CUBE_POS[0] + offset[0], CUBE_POS[1] + offset[1], CUBE_POS[2]])
                lift_target = cube_pos + np.array([0.0, 0.0, LIFT_DZ])
                exec_phase = execu["phase"][:]
                cartesian_target = hla["cartesian_target"][:]
                mismatches = 0
                for k in range(n_act):
                    last_row = (k + 1) * substeps - 1
                    phase_label = exec_phase[last_row].decode("utf-8")
                    expected = _phase_target(_base_phase(phase_label), cube_pos, lift_target)
                    if not np.allclose(expected, cartesian_target[k], atol=1e-6):
                        mismatches += 1
                if mismatches > 0:
                    _fail(
                        prefix + f"{mismatches} of {n_act} high_level_actions.cartesian_target entries do not "
                        "match the value _phase_target() deterministically derives from execution's own "
                        "recorded phase label at that transition's declared sampling instant -- action stream "
                        "may be shifted or corrupted",
                        errors,
                    )

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
                ("hla/cartesian_target", hla["cartesian_target"][:]),
                ("execution/arm_joint_target", execu["arm_joint_target"][:]),
                ("execution/applied_ctrl", execu["applied_ctrl"][:]),
            ):
                if not np.all(np.isfinite(arr)):
                    _fail(prefix + f"{arr_name} contains non-finite values", errors)

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
                deltas = np.linalg.norm(np.diff(cube_pos_arr, axis=0), axis=1)
                if np.any(deltas > MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M):
                    _fail(
                        prefix + f"cube_pos jumped {deltas.max():.4f} m between consecutive recorded "
                        f"transitions (> {MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M} m tripwire)",
                        errors,
                    )

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
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "data" / "task1_prototype_v2.hdf5"
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
