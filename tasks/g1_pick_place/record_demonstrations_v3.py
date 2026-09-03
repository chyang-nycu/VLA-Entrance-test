#!/usr/bin/env python3
"""Phase 5D: redesigned, causally-valid VLA policy-action dataset collector.

Builds directly on Phase 5C's collector (`record_demonstrations_v2.collect_episode`,
imported unmodified -- this module does not re-run or duplicate any control/
success-detection logic, it re-derives new action fields from the SAME
episode arrays Phase 5C already collects). The 500Hz `execution/` group and
10Hz `policy/observations/` group are carried over byte-for-byte; only
`policy/actions/` (formerly `policy/high_level_actions/`) is redesigned per
tasks.g1_pick_place.policy_action_codec.

New action semantics (see policy_action_codec.py's module docstring for the
full derivation and frame verification): at transition t, the primary action
is the expert's actual commanded TCP-reference DELTA over [t, t+1], derived
purely from `execution/arm_joint_target` (the expert's own commanded joint
trajectory) via forward kinematics -- never from privileged cube/target
state, never from a future policy observation. This directly fixes Phase
5C's root cause (a repeated static per-phase goal).
"""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import h5py
import mujoco
import numpy as np

from tasks.g1_pick_place.camera_observation import (
    CAM_FOVY_DEG, CAM_HEIGHT, CAM_WIDTH, HEAD_CAM_PARENT_BODY, write_grasp_scene_5a,
)
from tasks.g1_pick_place.canonical_config import (
    load_manifest, manifest_hash, verify_environment_matches_manifest,
)
from tasks.g1_pick_place.controller import (
    GRIPPER_ACTUATORS, GRIPPER_JOINTS, RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap,
)
from tasks.g1_pick_place.gripper_scene import TARGET_POS, TARGET_RELEASE_Z
from tasks.g1_pick_place.policy_action_codec import (
    ACTION_SCHEMA_VERSION, DECODER_VERSION, ORIENTATION_FRAME, POSITION_FRAME,
    SUB_ACTIONS_PER_TRANSITION, decoder_config_dict, decoder_configuration_hash, encode_delta,
    forward_kinematics_tcp,
)
from tasks.g1_pick_place.record_demonstrations_v2 import (
    EPISODES, SUBSTEPS_PER_TRANSITION, TIMESTEP, collect_episode,
)
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E

ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
LOG_DIR = ROOT / "logs"

SCHEMA_VERSION = "3.0.0"
POLICY_HZ = 10.0
EXECUTION_HZ = 1.0 / TIMESTEP


def _derive_policy_actions(scene_path: Path, ep: dict) -> dict:
    """Re-derives the v3 policy/actions fields from a Phase-5C-style
    episode's arrays via forward kinematics of the expert's own commanded
    joint-target trajectory. Reads `execution/arm_joint_target` and
    `policy/observations/joint_positions[0]` (the reset pose, measured
    before any command is issued -- unambiguously equal to the commanded
    reference at t=0) only; no privileged cube/target state is touched.

    Attempt-3 (shipped) design: each 10Hz transition is subdivided into
    SUB_ACTIONS_PER_TRANSITION (H=5) sub-chunks of
    SUBSTEPS_PER_TRANSITION//H (10) physics steps each (a 50Hz sub-action
    rate) -- see policy_action_codec.py's module docstring for why a
    single whole-interval delta (attempts 1/2) could not meet the <=10mm
    replay target on large single-transition reference jumps.
    """
    arrays = ep["arrays"]
    n = ep["n_transitions"]
    h = SUB_ACTIONS_PER_TRANSITION
    sub_steps = SUBSTEPS_PER_TRANSITION // h

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    fk_scratch = mujoco.MjData(model)
    mujoco.mj_resetData(model, fk_scratch)
    template_qpos = fk_scratch.qpos.copy()

    # Boundary 0 is the reset state (measured == commanded, zero physics
    # steps taken yet). Sub-boundary (k, sub) for sub=1..H is the commanded
    # joint target in effect at the LAST physics step of transition k's
    # sub-chunk `sub` -- i.e. generated entirely within transition k's own
    # 100ms interval, never from data beyond it. Boundary k*H+sub indexes
    # the flattened (n*H + 1)-length boundary sequence.
    exec_arm_joint_target = arrays["exec_arm_joint_target"]
    joint_boundaries = [arrays["obs_joint_positions"][0]]
    for k in range(n):
        for sub in range(1, h + 1):
            idx = k * SUBSTEPS_PER_TRANSITION + sub * sub_steps - 1
            joint_boundaries.append(exec_arm_joint_target[idx])

    pos_boundaries = []
    quat_boundaries = []
    for jb in joint_boundaries:
        pos, quat = forward_kinematics_tcp(model, template_qpos, arm_map, site_id, jb, scratch=fk_scratch)
        pos_boundaries.append(pos)
        quat_boundaries.append(quat)

    tcp_delta_position = np.zeros((n, h, 3), dtype=np.float32)
    tcp_delta_orientation = np.zeros((n, h, 3), dtype=np.float32)
    bidx = 0
    for k in range(n):
        for sub in range(h):
            bidx += 1
            dp, dr = encode_delta(
                pos_boundaries[bidx - 1], quat_boundaries[bidx - 1], pos_boundaries[bidx], quat_boundaries[bidx],
            )
            tcp_delta_position[k, sub] = dp
            tcp_delta_orientation[k, sub] = dr

    gripper_open = arrays["act_gripper_command_open"]  # bool, len n
    gripper_command = np.where(gripper_open, 0.0, 1.0).astype(np.float32)  # 0.0=open, 1.0=closed

    # next_arm_joint_target stays at TRANSITION granularity (the commanded
    # target at the end of the FULL 100ms interval, i.e. every H-th boundary).
    next_arm_joint_target = np.stack(joint_boundaries[h::h][:n]).astype(np.float32)
    state_machine_phase = np.array([str(p) for p in arrays["obs_phase"][1:]], dtype=object)  # len n, metadata only

    return {
        "tcp_delta_position": tcp_delta_position,
        "tcp_delta_orientation": tcp_delta_orientation,
        "gripper_command": gripper_command,
        "next_arm_joint_target": next_arm_joint_target,
        "state_machine_phase": state_machine_phase,
        "commanded_ref_tcp_pos_boundaries": np.stack(pos_boundaries).astype(np.float32),
        "commanded_ref_tcp_quat_boundaries": np.stack(quat_boundaries).astype(np.float32),
    }


def _write_hdf5(episodes: list[dict], scene_path: Path, out_path: Path) -> None:
    manifest = load_manifest()
    m_hash = manifest_hash()
    d_hash = decoder_configuration_hash(SUBSTEPS_PER_TRANSITION)
    cam = manifest["camera"]

    with h5py.File(out_path, "w") as f:
        f.attrs["schema_version"] = SCHEMA_VERSION
        f.attrs["schema_version_note"] = (
            "3.0.0 is this DATASET's own schema version (redesigned "
            "reference-relative TCP-delta policy actions, Phase 5D). "
            "data/task1_canonical_config.json's manifest FORMAT version "
            "(1.0.0) and its scene/controller/gains/thresholds/camera "
            "content are unchanged from Phase 5B/5C."
        )
        f.attrs["action_schema_version"] = ACTION_SCHEMA_VERSION
        f.attrs["action_frame"] = json.dumps({"position": POSITION_FRAME, "orientation": ORIENTATION_FRAME})
        f.attrs["decoder_version"] = DECODER_VERSION
        f.attrs["decoder_configuration_hash"] = d_hash
        f.attrs["decoder_configuration_json"] = json.dumps(decoder_config_dict(SUBSTEPS_PER_TRANSITION))
        f.attrs["mujoco_version"] = manifest["mujoco_version"]
        f.attrs["robot_embodiment"] = manifest["robot_embodiment"]
        f.attrs["unitree_mujoco_pinned_commit"] = manifest["unitree_mujoco_pinned_commit"]
        f.attrs["project_git_commit"] = manifest["project_git_commit_before_phase5b"]
        f.attrs["task_id"] = manifest["task_id"]
        f.attrs["task_instruction"] = manifest["task_instruction"]
        f.attrs["transition_convention"] = "observation_t -> action_t -> physics_substeps -> observation_t+1"
        f.attrs["policy_control_hz"] = POLICY_HZ
        f.attrs["physics_hz"] = 1.0 / TIMESTEP
        f.attrs["execution_hz"] = EXECUTION_HZ
        f.attrs["substeps_per_transition"] = SUBSTEPS_PER_TRANSITION
        f.attrs["rgb_hz"] = POLICY_HZ
        f.attrs["terminal_convention"] = (
            "The final recorded policy observation in each episode is terminal "
            "and has no paired action; len(policy/observations) == "
            "len(policy/actions) + 1 for every episode. execution/ holds "
            "exactly n_transitions * substeps_per_transition rows."
        )
        f.attrs["camera_params_json"] = json.dumps(cam)
        f.attrs["coordinate_conventions"] = (
            "World frame: MuJoCo world frame (right-handed, Z-up). "
            "Quaternions: (w, x, y, z), MuJoCo convention. "
            "RGB: uint8, HWC, channel order R,G,B. Positions in meters. "
            "Policy action frames: see action_frame attribute and data/schema_v3.md."
        )
        f.attrs["canonical_manifest_sha256"] = m_hash

        ep_group = f.create_group("episodes")
        for ep in episodes:
            g = ep_group.create_group(ep["variant_id"])
            g.attrs["instruction"] = manifest["task_instruction"]
            g.attrs["variant_id"] = ep["variant_id"]
            g.attrs["seed"] = 0
            g.attrs["success"] = bool(ep["task_pass"])
            g.attrs["termination_reason"] = "DONE" if ep["task_pass"] else str(ep["failure_state"])
            g.attrs["failure_reason"] = str(ep["failure_reason"]) if ep["failure_reason"] else ""
            g.attrs["train_eligible"] = bool(ep["task_pass"])
            g.attrs["transition_count"] = ep["n_transitions"]
            g.attrs["execution_row_count"] = ep["n_execution_rows"]
            g.attrs["canonical_manifest_sha256"] = m_hash
            g.attrs["decoder_configuration_hash"] = d_hash
            g.attrs["cube_xy_offset"] = list(ep["cube_xy_offset"])
            g.attrs["final_xy_target_error_m"] = (
                ep["final_xy_target_error_m"] if ep["final_xy_target_error_m"] is not None else float("nan")
            )

            arrays = ep["arrays"]
            act = ep["policy_actions"]

            policy = g.create_group("policy")
            policy.attrs["instruction"] = manifest["task_instruction"]
            obs = policy.create_group("observations")
            obs.create_dataset("rgb", data=arrays["obs_rgb"], compression="gzip", compression_opts=4)
            obs.create_dataset("joint_positions", data=arrays["obs_joint_positions"])
            obs.create_dataset("joint_velocities", data=arrays["obs_joint_velocities"])
            obs.create_dataset("tcp_pose", data=np.concatenate([arrays["obs_tcp_pos"], arrays["obs_tcp_quat"]], axis=1))
            obs.create_dataset("gripper_state", data=arrays["obs_gripper_state"])
            obs.create_dataset("timestamps", data=arrays["obs_t"])
            obs.attrs["rgb_channel_order"] = "RGB"
            obs.attrs["rgb_note"] = "Raw frames, no video-text overlay. Stored ONLY at policy (10 Hz) rate."
            obs.attrs["tcp_pose_layout"] = "[x, y, z, qw, qx, qy, qz]"

            actions = policy.create_group("actions")
            actions.create_dataset("tcp_delta_position", data=act["tcp_delta_position"])
            actions.create_dataset("tcp_delta_orientation", data=act["tcp_delta_orientation"])
            actions.create_dataset("gripper_command", data=act["gripper_command"])
            actions.create_dataset("next_arm_joint_target", data=act["next_arm_joint_target"])
            actions.create_dataset(
                "state_machine_phase",
                data=np.array([p.encode("utf-8") for p in act["state_machine_phase"]]),
            )
            actions.attrs["tcp_delta_position_note"] = (
                "Shape [T, H=5, 3]. Expert's commanded TCP-reference "
                "translation for each of H=5 sub-chunks (50Hz, 10 physics "
                "steps each) covering transition t's 100ms interval, world "
                "frame, derived via forward kinematics of "
                "execution/arm_joint_target (never from measured/tracked "
                "state, never from privileged cube/target state). PRIMARY "
                "declared VLA action field. Chunked (attempt 3) because a "
                "single whole-interval delta (attempts 1-2) could not "
                "describe large sub-100ms reference jumps closely enough to "
                "meet the <=10mm replay target -- see "
                "reports/phase5d-policy-action-redesign.md."
            )
            actions.attrs["tcp_delta_orientation_note"] = (
                "Shape [T, H=5, 3]. Expert's commanded TCP-reference "
                "rotation per sub-chunk, TCP-local body frame, MuJoCo "
                "mju_subQuat convention: quat_next = quat_prev (x) "
                "axisAngle2Quat(delta). PRIMARY declared VLA action field "
                "alongside tcp_delta_position."
            )
            actions.attrs["gripper_command_note"] = (
                "Shape [T]. 0.0 = open, 1.0 = closed. NOT chunked -- held "
                "constant across all H sub-chunks of a transition. PRIMARY "
                "declared VLA action field."
            )
            actions.attrs["action_chunk_note"] = (
                "H=5 sub-actions per 10Hz transition, 10 physics steps "
                "(50Hz) per sub-action. Execution order: sub-chunk 0 first, "
                "then 1..4, each fully ramped (one IK solve + one linear "
                "joint-space ramp) before the next begins -- no "
                "mid-sub-chunk re-targeting. Padding/mask: not needed in "
                "this stored dataset -- execution/ is truncated to whole "
                "50-physics-step transitions only (see terminal_convention), "
                "so every stored transition has a full, valid H=5 chunk; a "
                "runtime consumer operating past this dataset's own episode "
                "length would need to supply its own padding policy (e.g. "
                "zero-delta, repeat last gripper_command), which this "
                "dataset does not need to specify. A VLA policy predicts "
                "tcp_delta_position, tcp_delta_orientation (both [H,3] per "
                "step) and gripper_command (scalar per step)."
            )
            actions.attrs["next_arm_joint_target_note"] = (
                "Shape [T, 7]. OPTIONAL embodiment-specific auxiliary/debug "
                "field (the commanded joint target at the END of the full "
                "100ms transition, i.e. after all H sub-chunks) -- NOT the "
                "declared VLA action; provided for debugging and "
                "alternative joint-space behavior cloning only."
            )
            actions.attrs["state_machine_phase_note"] = "Shape [T]. METADATA ONLY -- not a required/declared policy output."
            actions.attrs["declared_vla_action_fields"] = json.dumps(
                ["tcp_delta_position", "tcp_delta_orientation", "gripper_command"]
            )

            execu = g.create_group("execution")
            execu.create_dataset("transition_index", data=arrays["exec_transition_index"])
            execu.create_dataset("timestamps", data=arrays["exec_t"])
            execu.create_dataset("arm_joint_target", data=arrays["exec_arm_joint_target"])
            execu.create_dataset("gripper_target", data=arrays["exec_gripper_target"])
            execu.create_dataset("applied_ctrl", data=arrays["exec_applied_ctrl"])
            execu.create_dataset("joint_positions", data=arrays["exec_joint_positions"])
            execu.create_dataset("joint_velocities", data=arrays["exec_joint_velocities"])
            execu.create_dataset("tcp_pose", data=np.concatenate([arrays["exec_tcp_pos"], arrays["exec_tcp_quat"]], axis=1))
            execu.create_dataset("cube_pos", data=arrays["exec_cube_pos"])
            execu.create_dataset("cube_quat", data=arrays["exec_cube_quat"])
            execu.create_dataset("phase", data=np.array([p.encode("utf-8") for p in arrays["exec_phase"]]))
            execu.attrs["rate_hz"] = EXECUTION_HZ
            execu.attrs["applied_ctrl_layout"] = "[arm(7), gripper(2)] concatenated, actuator order = RIGHT_ARM_ACTUATORS + GRIPPER_ACTUATORS"
            execu.attrs["applied_ctrl_note"] = (
                "Unchanged from Phase 5C -- privileged/debug/replay evidence "
                "only, never duplicated as a VLA policy target."
            )
            execu.attrs["transition_index_note"] = (
                "transition_index[i] = k means execution row i belongs to "
                "policy transition k."
            )

            priv = g.create_group("privileged")
            priv.create_dataset("cube_pos", data=arrays["obs_cube_pos"])
            priv.create_dataset("cube_quat", data=arrays["obs_cube_quat"])
            priv.create_dataset("target_pos", data=np.array([TARGET_POS[0], TARGET_POS[1], TARGET_RELEASE_Z], dtype=np.float32))
            priv.create_dataset("phase", data=np.array([p.encode("utf-8") for p in arrays["obs_phase"]]))
            priv.create_dataset("commanded_ref_tcp_pos", data=act["commanded_ref_tcp_pos_boundaries"])
            priv.create_dataset("commanded_ref_tcp_quat", data=act["commanded_ref_tcp_quat_boundaries"])
            priv.attrs["note"] = (
                "Simulator-only ground truth at POLICY (10 Hz) rate. NOT "
                "part of the declared VLA policy-input group. "
                "commanded_ref_tcp_pos/quat are the forward-kinematics "
                "boundary values the policy/actions/tcp_delta_* fields were "
                "derived from -- privileged debug evidence, not a policy "
                "input or action."
            )


def main() -> int:
    manifest = load_manifest()
    verify_environment_matches_manifest(
        scene_generator_name="write_grasp_scene_5a",
        use_oriented_ik=False,
        arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B,
        gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
        camera_parent_body=HEAD_CAM_PARENT_BODY,
        camera_resolution_wh=(CAM_WIDTH, CAM_HEIGHT),
        manifest=manifest,
    )

    scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    episodes = []
    summary = {
        "episodes": [], "manifest_sha256": manifest_hash(), "schema_version": SCHEMA_VERSION,
        "decoder_configuration_hash": decoder_configuration_hash(SUBSTEPS_PER_TRANSITION),
        "action_schema_version": ACTION_SCHEMA_VERSION,
    }
    for spec in EPISODES:
        wall_start = time.perf_counter()
        ep = collect_episode(scene_path, spec["variant_id"], spec["cube_xy_offset"])
        ep["policy_actions"] = _derive_policy_actions(scene_path, ep)
        derive_s = time.perf_counter() - wall_start - ep["wall_total_s"]
        episodes.append(ep)

        act = ep["policy_actions"]
        dp_mag = np.linalg.norm(act["tcp_delta_position"], axis=-1)  # [n, H]
        dr_mag = np.linalg.norm(act["tcp_delta_orientation"], axis=-1)  # [n, H]
        summary["episodes"].append({
            "variant_id": ep["variant_id"],
            "success": ep["task_pass"],
            "failure_state": ep["failure_state"],
            "failure_reason": ep["failure_reason"],
            "n_transitions": ep["n_transitions"],
            "n_observations": ep["n_observations"],
            "n_execution_rows": ep["n_execution_rows"],
            "physics_steps_total": ep["physics_steps_total"],
            "wall_total_s": ep["wall_total_s"],
            "action_derivation_s": derive_s,
            "render_time_s": ep["render_time_s"],
            "final_xy_target_error_m": ep["final_xy_target_error_m"],
            "tcp_delta_position_stats_m": {
                "max": float(dp_mag.max()) if len(dp_mag) else 0.0,
                "mean": float(dp_mag.mean()) if len(dp_mag) else 0.0,
            },
            "tcp_delta_orientation_stats_rad": {
                "max": float(dr_mag.max()) if len(dr_mag) else 0.0,
                "mean": float(dr_mag.mean()) if len(dr_mag) else 0.0,
            },
        })
        print(f"[{ep['variant_id']}] success={ep['task_pass']} transitions={ep['n_transitions']} "
              f"max_delta_pos={dp_mag.max() if len(dp_mag) else 0:.5f}m wall={ep['wall_total_s']:.2f}s")

    out_path = DATA_DIR / "task1_prototype_v3.hdf5"
    _write_hdf5(episodes, scene_path, out_path)

    file_bytes = out_path.stat().st_size
    sha256 = hashlib.sha256(out_path.read_bytes()).hexdigest()
    summary["hdf5_path"] = str(out_path.relative_to(ROOT))
    summary["hdf5_size_bytes"] = file_bytes
    summary["hdf5_sha256"] = sha256

    (LOG_DIR / "phase5d_v3_collection_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
