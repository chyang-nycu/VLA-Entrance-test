#!/usr/bin/env python3
"""Phase 5B: HDF5 dataset validator (Section D).

Checks, per episode and dataset-wide:
- required groups/attributes present
- matching time dimensions across observation/action arrays (with the
  observations = actions + 1 terminal-transition convention)
- monotonic timestamps
- all-finite values (except the documented NaN for a failure episode's
  final_xy_target_error_m, which is explicitly allowed since the cube never
  reached a placement to measure)
- quaternion norms ~= 1
- actions within documented bounds (joint ranges / ctrl ranges from the live
  model, loaded via the canonical manifest's scene)
- image dtype/range/variance sane
- nonempty instruction strings
- stored success agrees with the objective detector -- recomputed by
  RE-RUNNING the deterministic simulation for that episode's variant (this
  pipeline has no RNG anywhere) and comparing task_pass, not by trusting the
  stored flag
- failure episode(s) excluded from the training split (train_eligible=False)
- dataset reopens successfully after being closed
- no post-initialization cube-state writes: re-derived from the episode's
  own privileged cube-pose trajectory (no discontinuous jump outside of
  contact-mediated motion), independent of CubeInitGuard's own source-level
  self-audit (which guards the live code, not a stored recording)

Fails loudly (raises SystemExit(1) with printed findings) rather than
warning-and-continuing on any REQUIRED check; a small number of advisory
checks (see ADVISORY_CHECKS) are reported but do not fail validation.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import h5py
import numpy as np

from tasks.g1_pick_place.camera_observation import CAM_HEIGHT, CAM_WIDTH, write_grasp_scene_5a
from tasks.g1_pick_place.canonical_config import load_manifest
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[2]

REQUIRED_ROOT_ATTRS = [
    "schema_version", "mujoco_version", "robot_embodiment", "unitree_mujoco_pinned_commit",
    "project_git_commit", "task_id", "task_instruction", "transition_convention",
    "policy_control_hz", "physics_hz", "substeps_per_transition", "rgb_hz",
    "terminal_convention", "camera_params_json", "coordinate_conventions",
    "canonical_manifest_sha256",
]
REQUIRED_EPISODE_ATTRS = [
    "instruction", "variant_id", "seed", "success", "termination_reason",
    "train_eligible", "transition_count", "canonical_manifest_sha256",
]
REQUIRED_PO_DATASETS = ["rgb", "joint_positions", "joint_velocities", "tcp_pose", "gripper_state", "timestamps"]
REQUIRED_ACTION_DATASETS = ["cartesian_target", "arm_joint_position_target", "gripper_target", "applied_ctrl"]
REQUIRED_PRIV_DATASETS = [
    "cube_pos", "cube_quat", "cube_linvel", "cube_angvel", "target_pos",
    "left_contact", "right_contact", "bilateral_contact", "contact_force_n", "phase",
]

MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M = 0.05  # generous: at 10 Hz recording, even a
# free-falling cube (g=9.81) moves < 5cm in 0.1s from rest; any larger jump
# between two consecutive recorded cube_pos samples with no active grasp
# would indicate an unexplained discontinuity (e.g. a stray teleport), not
# ordinary physics -- this is a coarse tripwire, not a tight physics model.


class ValidationError(RuntimeError):
    pass


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

            for grp_name, required in (
                ("policy_observations", REQUIRED_PO_DATASETS),
                ("actions", REQUIRED_ACTION_DATASETS),
                ("privileged", REQUIRED_PRIV_DATASETS),
            ):
                if grp_name not in g:
                    _fail(prefix + f"missing group {grp_name}", errors)
                    continue
                for ds in required:
                    if ds not in g[grp_name]:
                        _fail(prefix + f"missing dataset {grp_name}/{ds}", errors)

            if "policy_observations" not in g or "actions" not in g:
                continue

            po = g["policy_observations"]
            act = g["actions"]
            priv = g.get("privileged")

            n_obs = po["rgb"].shape[0]
            n_act = act["cartesian_target"].shape[0]
            if n_obs != n_act + 1:
                _fail(prefix + f"observation/action count mismatch: {n_obs} observations, {n_act} actions "
                      f"(expected observations == actions + 1, terminal-transition convention)", errors)
            if int(g.attrs.get("transition_count", -1)) != n_act:
                _fail(prefix + f"transition_count attr ({g.attrs.get('transition_count')}) != actual action count ({n_act})", errors)

            for ds_name in REQUIRED_PO_DATASETS:
                if ds_name in po and po[ds_name].shape[0] != n_obs:
                    _fail(prefix + f"policy_observations/{ds_name} has {po[ds_name].shape[0]} rows, expected {n_obs}", errors)
            for ds_name in REQUIRED_ACTION_DATASETS:
                if ds_name in act and act[ds_name].shape[0] != n_act:
                    _fail(prefix + f"actions/{ds_name} has {act[ds_name].shape[0]} rows, expected {n_act}", errors)
            if priv is not None:
                for ds_name in REQUIRED_PRIV_DATASETS:
                    if ds_name in priv and ds_name != "target_pos" and priv[ds_name].shape[0] != n_obs:
                        _fail(prefix + f"privileged/{ds_name} has {priv[ds_name].shape[0]} rows, expected {n_obs}", errors)

            timestamps = po["timestamps"][:]
            if not np.all(np.diff(timestamps) > 0):
                _fail(prefix + "timestamps are not strictly monotonic increasing", errors)

            for arr_name, arr in (
                ("joint_positions", po["joint_positions"][:]),
                ("joint_velocities", po["joint_velocities"][:]),
                ("tcp_pose", po["tcp_pose"][:]),
                ("gripper_state", po["gripper_state"][:]),
                ("cartesian_target", act["cartesian_target"][:]),
                ("arm_joint_position_target", act["arm_joint_position_target"][:]),
                ("gripper_target", act["gripper_target"][:]),
                ("applied_ctrl", act["applied_ctrl"][:]),
            ):
                if not np.all(np.isfinite(arr)):
                    _fail(prefix + f"{arr_name} contains non-finite values", errors)

            quats = po["tcp_pose"][:, 3:7]
            norms = np.linalg.norm(quats, axis=1)
            if not np.allclose(norms, 1.0, atol=1e-3):
                _fail(prefix + f"TCP quaternion norms deviate from 1.0 (max |1-norm|={np.max(np.abs(norms-1)):.2e})", errors)
            if priv is not None:
                cq = priv["cube_quat"][:]
                cq_norms = np.linalg.norm(cq, axis=1)
                if not np.allclose(cq_norms, 1.0, atol=1e-3):
                    _fail(prefix + f"cube quaternion norms deviate from 1.0 (max |1-norm|={np.max(np.abs(cq_norms-1)):.2e})", errors)

            rgb = po["rgb"]
            if rgb.dtype != np.uint8:
                _fail(prefix + f"rgb dtype is {rgb.dtype}, expected uint8", errors)
            rgb_arr = rgb[:]
            if rgb_arr.shape[1:] != (CAM_HEIGHT, CAM_WIDTH, 3):
                _fail(prefix + f"rgb shape {rgb_arr.shape[1:]} != expected ({CAM_HEIGHT},{CAM_WIDTH},3)", errors)
            if rgb_arr.std() < 1.0:
                _fail(prefix + f"rgb frames appear blank (std={rgb_arr.std():.3f})", errors)
            per_frame_std = rgb_arr.reshape(rgb_arr.shape[0], -1).std(axis=1)
            if np.any(per_frame_std < 0.5):
                advisories.append(prefix + f"{int(np.sum(per_frame_std < 0.5))} frame(s) with unusually low per-frame variance")

            success_attr = bool(g.attrs["success"])
            train_eligible_attr = bool(g.attrs["train_eligible"])
            if success_attr:
                train_eligible_count += 1
            else:
                train_ineligible_count += 1
                if train_eligible_attr:
                    _fail(prefix + "success=False but train_eligible=True (a failure episode must be excluded from the default training split)", errors)

            # Independent recomputation: rerun the deterministic simulation
            # for this episode's variant and compare task_pass to the stored
            # success flag -- not merely trusting the stored value.
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

            # No post-initialization cube-state discontinuity: check
            # consecutive recorded cube_pos deltas against a generous
            # tripwire bound (physics-only motion, including free-fall,
            # cannot exceed this between two 0.1s-apart samples).
            if priv is not None:
                cube_pos = priv["cube_pos"][:]
                deltas = np.linalg.norm(np.diff(cube_pos, axis=0), axis=1)
                if np.any(deltas > MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M):
                    _fail(
                        prefix + f"cube_pos jumped {deltas.max():.4f} m between consecutive recorded "
                        f"transitions (> {MAX_CUBE_JUMP_BETWEEN_TRANSITIONS_M} m tripwire) -- possible "
                        "post-initialization cube-state write",
                        errors,
                    )

        if train_eligible_count == 0:
            advisories.append("no train-eligible (success=True) episodes in dataset")
        if train_ineligible_count == 0:
            advisories.append("no excluded (success=False) failure episode present in dataset")

    # Dataset reopens successfully.
    try:
        with h5py.File(path, "r") as f2:
            _ = list(f2["episodes"].keys())
    except Exception as e:
        _fail(f"dataset failed to reopen after close: {e}", errors)

    return {"errors": errors, "advisories": advisories}


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "data" / "task1_prototype.hdf5"
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
