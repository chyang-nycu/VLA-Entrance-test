#!/usr/bin/env python3
"""Phase 5E: scaled Task 1 VLA demonstration collection.

Collects the 32 episodes locked in data/task1_collection_spec.json (24
success-intended + 8 diagnostic probes) using the UNCHANGED Phase 5D v3
action schema/decoder (tasks.g1_pick_place.policy_action_codec) and the
UNCHANGED canonical manifest (data/task1_canonical_config.json). Task 1
physics/controller/gains/geometry/camera/success-thresholds are not touched
-- this module only calls tasks.g1_pick_place.run_pick_place.run_trial_pick_place
with varying cube_xy_offset, exactly like Phase 5B-5D's collectors.

Per-episode policy-action derivation reuses
tasks.g1_pick_place.record_demonstrations_v3._derive_policy_actions
UNMODIFIED: that function derives every v3 action field purely from
execution/arm_joint_target (the real controller's own commanded trajectory)
via forward kinematics, so it needs no cube/target-position-dependent
"high-level action" bookkeeping at all -- unlike Phase 5C's v2 collector,
this module does not need (and does not build) a _phase_target-style
per-phase Cartesian-goal table.

IMPORTANT DEVIATION FROM THE LITERAL AUTHORIZATION (documented in full in
data/task1_collection_spec.json's DEVIATION_DISCLOSURE_target_position_not_varied
key and reports/phase5e-scaled-data-collection.md Section A): target
position is NOT varied in this phase. The rendered blue target pad is fixed
scene geometry (tasks.g1_pick_place.gripper_scene, TARGET_POS baked into the
MJCF at generation time, no offset parameter) -- varying only a controller-
side target belief while the visible pad stays fixed would produce
physically dishonest episodes (RGB shows the pad in one place, the
recorded/true placement target is silently elsewhere), and moving the
rendered pad itself would be a Task 1 GEOMETRY change, which this phase's
authorization explicitly prohibits. Every episode therefore uses
target_xy_offset = (0.0, 0.0); only cube_xy_offset (correctly parameterized
in run_trial_pick_place since Phase 4E) is varied.

Every attempted configuration (accepted, rejected by reachability, failed
during interaction, successful) is recorded -- none is silently discarded.
A config that fails the cheap position-only IK reachability pre-filter
(diagnose_pick_place_reachability -> all_reachable) is recorded as
outcome="rejected_by_reachability" with NO physics trial run for it (0
simulated seconds); every other config gets a full trial and a full HDF5
episode group, whether it ultimately succeeds or fails.
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
    CAM_HEIGHT, CAM_WIDTH, HEAD_CAM_PARENT_BODY, write_grasp_scene_5a,
)
from tasks.g1_pick_place.canonical_config import (
    load_manifest, manifest_hash, verify_environment_matches_manifest,
)
from tasks.g1_pick_place.controller import (
    GRIPPER_ACTUATORS, GRIPPER_JOINTS, RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap,
)
from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS, TARGET_RELEASE_Z
from tasks.g1_pick_place.policy_action_codec import (
    ACTION_SCHEMA_VERSION, DECODER_VERSION, ORIENTATION_FRAME, POSITION_FRAME,
    SUB_ACTIONS_PER_TRANSITION, decoder_config_dict, decoder_configuration_hash,
)
from tasks.g1_pick_place.record_demonstrations_v2 import (
    SUBSTEPS_PER_TRANSITION, TIMESTEP, _base_phase, _capture_probe_observation, _finger_open_for_phase,
    _mat_to_quat_wxyz,
)
from tasks.g1_pick_place.run_grasp_test_3c import _finger_targets
from tasks.g1_pick_place.record_demonstrations_v3 import _derive_policy_actions
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E,
    diagnose_pick_place_reachability, run_trial_pick_place,
)

ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
LOG_DIR = ROOT / "logs"
ARTIFACT_DIR = ROOT / "artifacts" / "phase5e_dataset_summary"

SCHEMA_VERSION = "1.0.0"  # this DATASET's own scaled-collection schema version
POLICY_HZ = 10.0
EXECUTION_HZ = 1.0 / TIMESTEP

SPEC_PATH = DATA_DIR / "task1_collection_spec.json"
HDF5_PATH = DATA_DIR / "task1_demonstrations_v1.hdf5"
# Threshold above which the raw HDF5 is left untracked (see Section H: "Do
# not commit the HDF5 file if it becomes large").
COMMIT_SIZE_THRESHOLD_BYTES = 20 * 1024 * 1024


def check_reachability(scene_path: Path, cube_xy_offset: tuple[float, float]) -> dict:
    """Cheap, physics-step-free pre-filter (Section A: 'use pre-run IK
    diagnostics'). Builds a throwaway MjData at the reset pose -- no
    mujoco.mj_step is ever called here, so this costs milliseconds, not
    seconds, regardless of outcome.
    """
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    mujoco.mj_resetData(model, data)
    cube_pos = np.array([CUBE_POS[0] + cube_xy_offset[0], CUBE_POS[1] + cube_xy_offset[1], CUBE_POS[2]])
    return diagnose_pick_place_reachability(model, arm_map, site_id, data.qpos.copy(), cube_pos)


def collect_episode(scene_path: Path, cube_xy_offset: tuple[float, float]) -> dict:
    """Runs one real Task 1 trial via run_trial_pick_place, recording the
    same policy(10Hz)/execution(500Hz) two-rate arrays Phase 5C/5D's
    collectors do. Does not build any cartesian_target/high_level_actions
    bookkeeping -- unneeded for the v3 action schema (see module docstring).
    """
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    gripper_map = JointMap.build(model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")

    renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)

    probe = _capture_probe_observation(scene_path, cube_xy_offset)
    policy_obs = {k: [probe[k]] for k in (
        "t", "rgb", "joint_positions", "joint_velocities", "gripper_state",
        "tcp_pos", "tcp_quat", "cube_pos", "cube_quat", "phase",
    )}
    gripper_open_flags: list[bool] = []

    exec_rows = {
        "t": [], "transition_index": [], "arm_joint_target": [], "gripper_target": [],
        "applied_ctrl": [], "joint_positions": [], "joint_velocities": [], "tcp_pos": [],
        "tcp_quat": [], "cube_pos": [], "cube_quat": [], "phase": [],
    }

    call_count = [0]
    render_time_accum = [0.0]

    def frame_callback(phase: str, m: mujoco.MjModel, data: mujoco.MjData) -> None:
        call_count[0] += 1
        base = _base_phase(phase)
        finger_open = _finger_open_for_phase(base)
        transition_index = (call_count[0] - 1) // SUBSTEPS_PER_TRANSITION

        arm_ctrl = data.ctrl[arm_map.actuator_id].copy()
        gripper_ctrl = data.ctrl[gripper_map.actuator_id].copy()
        gripper_target_now = _finger_targets(gripper_map, finger_open)
        exec_rows["t"].append(float(data.time))
        exec_rows["transition_index"].append(int(transition_index))
        exec_rows["arm_joint_target"].append(arm_ctrl.copy())
        exec_rows["gripper_target"].append(gripper_target_now)
        exec_rows["applied_ctrl"].append(np.concatenate([arm_ctrl, gripper_ctrl]))
        exec_rows["joint_positions"].append(arm_map.get_qpos(data))
        exec_rows["joint_velocities"].append(arm_map.get_qvel(data))
        exec_rows["tcp_pos"].append(data.site_xpos[site_id].copy())
        exec_rows["tcp_quat"].append(_mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3)))
        exec_rows["cube_pos"].append(data.xpos[cube_body_id].copy())
        exec_rows["cube_quat"].append(data.xquat[cube_body_id].copy())
        exec_rows["phase"].append(phase)

        if call_count[0] % SUBSTEPS_PER_TRANSITION != 0:
            return

        gripper_open_flags.append(bool(finger_open))

        t0 = time.perf_counter()
        renderer.update_scene(data, camera="head_cam")
        rgb = renderer.render().copy()
        render_time_accum[0] += time.perf_counter() - t0

        policy_obs["t"].append(float(data.time))
        policy_obs["rgb"].append(rgb)
        policy_obs["joint_positions"].append(arm_map.get_qpos(data))
        policy_obs["joint_velocities"].append(arm_map.get_qvel(data))
        policy_obs["gripper_state"].append(gripper_map.get_qpos(data))
        policy_obs["tcp_pos"].append(data.site_xpos[site_id].copy())
        policy_obs["tcp_quat"].append(_mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3)))
        policy_obs["cube_pos"].append(data.xpos[cube_body_id].copy())
        policy_obs["cube_quat"].append(data.xquat[cube_body_id].copy())
        policy_obs["phase"].append(phase)

    wall_start = time.perf_counter()
    result = run_trial_pick_place(
        scene_path, cube_xy_offset=cube_xy_offset,
        gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
        frame_callback=frame_callback,
    )
    wall_total_s = time.perf_counter() - wall_start
    renderer.close()

    n_obs = len(policy_obs["t"])
    n_act = len(gripper_open_flags)
    assert n_obs == n_act + 1, f"transition alignment invariant violated: {n_obs} observations, {n_act} actions"
    n_exec_expected = n_act * SUBSTEPS_PER_TRANSITION
    for k in exec_rows:
        exec_rows[k] = exec_rows[k][:n_exec_expected]

    arrays = {}
    for k in ("t", "joint_positions", "joint_velocities", "gripper_state", "tcp_pos", "tcp_quat", "cube_pos", "cube_quat"):
        arrays[f"obs_{k}"] = np.stack(policy_obs[k]).astype(np.float64 if k == "t" else np.float32)
    arrays["obs_rgb"] = np.stack(policy_obs["rgb"]).astype(np.uint8)
    arrays["obs_phase"] = np.array(policy_obs["phase"], dtype=object)
    arrays["act_gripper_command_open"] = np.array(gripper_open_flags, dtype=np.bool_)

    for k in ("t", "arm_joint_target", "gripper_target", "applied_ctrl", "joint_positions",
              "joint_velocities", "tcp_pos", "tcp_quat", "cube_pos", "cube_quat"):
        arrays[f"exec_{k}"] = np.stack(exec_rows[k]).astype(np.float64 if k == "t" else np.float32)
    arrays["exec_transition_index"] = np.array(exec_rows["transition_index"], dtype=np.int32)
    arrays["exec_phase"] = np.array(exec_rows["phase"], dtype=object)

    return {
        "cube_xy_offset": cube_xy_offset,
        "arrays": arrays,
        "n_transitions": n_act,
        "n_observations": n_obs,
        "n_execution_rows": n_exec_expected,
        "task_pass": bool(result["task_pass"]),
        "failure_state": result["failure_state"],
        "failure_reason": result["failure_reason"],
        "final_xy_target_error_m": result["final_xy_target_error_m"],
        "wall_total_s": wall_total_s,
        "render_time_s": render_time_accum[0],
        "physics_steps_total": call_count[0],
    }


def sample_success_config(seed: int, dx_lo: float, dx_hi: float, dy_lo: float, dy_hi: float, templates: list[str]) -> dict:
    """Deterministic sampler used to REGENERATE (not to re-derive from the
    locked spec at collection time -- the spec already stores the sampled
    values) the same values data/task1_collection_spec.json's
    generation script used. Kept here, alongside the collector, purely for
    auditability / regeneration-from-scratch; collect_dataset.main() reads
    the ALREADY-SAMPLED values out of the locked spec file, never re-samples.
    """
    rng = np.random.default_rng(seed)
    cube_dx = float(rng.uniform(dx_lo, dx_hi))
    cube_dy = float(rng.uniform(dy_lo, dy_hi))
    template_id = int(rng.integers(0, len(templates)))
    return {"cube_xy_offset": [round(cube_dx, 5), round(cube_dy, 5)], "instruction_template_id": template_id}


def _write_episode_group(ep_group: h5py.Group, cfg: dict, ep: dict, scene_path: Path, manifest: dict) -> None:
    m_hash = manifest_hash()
    d_hash = decoder_configuration_hash(SUBSTEPS_PER_TRANSITION)
    act = _derive_policy_actions(scene_path, ep)

    g = ep_group.create_group(f"seed_{cfg['seed']}")
    g.attrs["seed"] = cfg["seed"]
    g.attrs["split"] = cfg["split"]
    g.attrs["instruction_canonical"] = manifest["task_instruction"]
    g.attrs["instruction_utterance"] = cfg["instruction_utterance"]
    g.attrs["instruction_template_id"] = cfg["instruction_template_id"]
    g.attrs["cube_xy_offset"] = list(cfg["cube_xy_offset"])
    g.attrs["target_xy_offset"] = list(cfg["target_xy_offset"])
    g.attrs["probe_kind"] = cfg.get("probe_kind", "success_envelope_sample")
    g.attrs["success"] = bool(ep["task_pass"])
    g.attrs["termination_reason"] = "DONE" if ep["task_pass"] else str(ep["failure_state"])
    g.attrs["failure_stage"] = str(ep["failure_state"]) if ep["failure_state"] else ""
    g.attrs["failure_reason"] = str(ep["failure_reason"]) if ep["failure_reason"] else ""
    g.attrs["train_eligible"] = bool(ep["task_pass"]) and cfg["split"] != "diagnostics"
    g.attrs["transition_count"] = ep["n_transitions"]
    g.attrs["execution_row_count"] = ep["n_execution_rows"]
    g.attrs["canonical_manifest_sha256"] = m_hash
    g.attrs["decoder_configuration_hash"] = d_hash
    g.attrs["collection_spec_hash"] = cfg["_spec_hash"]
    g.attrs["reachability_all_reachable"] = bool(cfg["_reachability"]["all_reachable"])
    g.attrs["reachability_residuals_json"] = json.dumps(
        {k: v["residual_m"] for k, v in cfg["_reachability"].items() if isinstance(v, dict)}
    )
    g.attrs["final_xy_target_error_m"] = (
        ep["final_xy_target_error_m"] if ep["final_xy_target_error_m"] is not None else float("nan")
    )

    arrays = ep["arrays"]
    policy = g.create_group("policy")
    policy.attrs["instruction_canonical"] = manifest["task_instruction"]
    policy.attrs["instruction_utterance"] = cfg["instruction_utterance"]
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
        "state_machine_phase", data=np.array([p.encode("utf-8") for p in act["state_machine_phase"]]),
    )
    actions.attrs["declared_vla_action_fields"] = json.dumps(
        ["tcp_delta_position", "tcp_delta_orientation", "gripper_command"]
    )
    actions.attrs["action_chunk_note"] = "H=5 sub-actions per 10Hz transition, 50Hz sub-action rate -- identical semantics/decoder to Phase 5D. See data/schema_v3.md."

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

    priv = g.create_group("privileged")
    priv.create_dataset("cube_pos", data=arrays["obs_cube_pos"])
    priv.create_dataset("cube_quat", data=arrays["obs_cube_quat"])
    priv.create_dataset("target_pos", data=np.array([TARGET_POS[0], TARGET_POS[1], TARGET_RELEASE_Z], dtype=np.float32))
    priv.create_dataset("phase", data=np.array([p.encode("utf-8") for p in arrays["obs_phase"]]))
    priv.create_dataset("commanded_ref_tcp_pos", data=act["commanded_ref_tcp_pos_boundaries"])
    priv.create_dataset("commanded_ref_tcp_quat", data=act["commanded_ref_tcp_quat_boundaries"])
    priv.attrs["note"] = "Simulator-only ground truth at POLICY (10 Hz) rate. NOT part of the declared VLA policy-input group."


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

    spec_bytes = SPEC_PATH.read_bytes()
    spec = json.loads(spec_bytes)
    spec_hash = hashlib.sha256(spec_bytes).hexdigest()
    if spec["canonical_manifest_sha256"] != manifest_hash():
        raise RuntimeError(
            f"data/task1_collection_spec.json's canonical_manifest_sha256 "
            f"({spec['canonical_manifest_sha256']!r}) does not match the live "
            f"canonical manifest hash ({manifest_hash()!r})."
        )

    scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    configs = spec["sampling_distribution"]["seed_to_config"]
    attempted = []
    written_episodes = []
    sim_time_total_s = 0.0
    wall_time_total_s = 0.0
    collection_wall_start = time.perf_counter()

    with h5py.File(HDF5_PATH, "w") as f:
        f.attrs["schema_version"] = SCHEMA_VERSION
        f.attrs["schema_version_note"] = (
            "1.0.0 is this SCALED-COLLECTION dataset's own schema version "
            "(Phase 5E). Reuses Phase 5D's action_schema_version/decoder_"
            "version/decoder_configuration_hash UNCHANGED (see below) -- "
            "only the number of episodes and their per-episode metadata "
            "(seed, split, instruction variants, reachability) are new."
        )
        f.attrs["action_schema_version"] = ACTION_SCHEMA_VERSION
        f.attrs["action_frame"] = json.dumps({"position": POSITION_FRAME, "orientation": ORIENTATION_FRAME})
        f.attrs["decoder_version"] = DECODER_VERSION
        f.attrs["decoder_configuration_hash"] = decoder_configuration_hash(SUBSTEPS_PER_TRANSITION)
        f.attrs["decoder_configuration_json"] = json.dumps(decoder_config_dict(SUBSTEPS_PER_TRANSITION))
        f.attrs["mujoco_version"] = manifest["mujoco_version"]
        f.attrs["robot_embodiment"] = manifest["robot_embodiment"]
        f.attrs["unitree_mujoco_pinned_commit"] = manifest["unitree_mujoco_pinned_commit"]
        f.attrs["task_id"] = manifest["task_id"]
        f.attrs["instruction_canonical"] = manifest["task_instruction"]
        f.attrs["transition_convention"] = "observation_t -> action_t -> physics_substeps -> observation_t+1"
        f.attrs["policy_control_hz"] = POLICY_HZ
        f.attrs["physics_hz"] = 1.0 / TIMESTEP
        f.attrs["execution_hz"] = EXECUTION_HZ
        f.attrs["substeps_per_transition"] = SUBSTEPS_PER_TRANSITION
        f.attrs["rgb_hz"] = POLICY_HZ
        f.attrs["canonical_manifest_sha256"] = manifest_hash()
        f.attrs["collection_spec_sha256"] = spec_hash
        f.attrs["collection_spec_path"] = "data/task1_collection_spec.json"

        ep_group = f.create_group("episodes")

        for cfg in configs:
            cube_xy_offset = tuple(cfg["cube_xy_offset"])
            reach = check_reachability(scene_path, cube_xy_offset)
            row = {
                "seed": cfg["seed"], "split": cfg["split"], "cube_xy_offset": list(cube_xy_offset),
                "target_xy_offset": list(cfg["target_xy_offset"]), "instruction_utterance": cfg["instruction_utterance"],
                "instruction_template_id": cfg["instruction_template_id"],
                "all_reachable": bool(reach["all_reachable"]), "probe_kind": cfg.get("probe_kind", "success_envelope_sample"),
            }

            if not reach["all_reachable"]:
                row["outcome"] = "rejected_by_reachability"
                row["task_pass"] = False
                row["failure_stage"] = "REACHABILITY_PREFILTER"
                row["failure_reason"] = "position-only IK reachability failed before any physics step was run"
                row["wall_s"] = 0.0
                row["sim_time_s"] = 0.0
                attempted.append(row)
                print(f"[seed {cfg['seed']}] REJECTED by reachability (no trial run)")
                continue

            cfg_full = dict(cfg)
            cfg_full["_spec_hash"] = spec_hash
            cfg_full["_reachability"] = reach

            wall_start = time.perf_counter()
            ep = collect_episode(scene_path, cube_xy_offset)
            wall_s = time.perf_counter() - wall_start
            wall_time_total_s += wall_s
            sim_time_total_s += ep["n_execution_rows"] * TIMESTEP

            _write_episode_group(ep_group, cfg_full, ep, scene_path, manifest)
            written_episodes.append(cfg["seed"])

            row["outcome"] = "success" if ep["task_pass"] else "failed_interaction"
            row["task_pass"] = bool(ep["task_pass"])
            row["failure_stage"] = str(ep["failure_state"]) if ep["failure_state"] else ""
            row["failure_reason"] = str(ep["failure_reason"]) if ep["failure_reason"] else ""
            row["n_transitions"] = ep["n_transitions"]
            row["wall_s"] = wall_s
            row["sim_time_s"] = ep["n_execution_rows"] * TIMESTEP
            row["final_xy_target_error_m"] = ep["final_xy_target_error_m"]
            attempted.append(row)
            print(f"[seed {cfg['seed']}] split={cfg['split']} outcome={row['outcome']} "
                  f"failure_stage={row['failure_stage']!r} wall={wall_s:.2f}s")

        f.attrs["attempted_configs_json"] = json.dumps(attempted)

    collection_wall_s = time.perf_counter() - collection_wall_start

    n_success = sum(1 for r in attempted if r["outcome"] == "success")
    n_failed_interaction = sum(1 for r in attempted if r["outcome"] == "failed_interaction")
    n_rejected = sum(1 for r in attempted if r["outcome"] == "rejected_by_reachability")
    n_diagnostic_total = n_failed_interaction + n_rejected

    file_bytes = HDF5_PATH.stat().st_size
    sha256 = hashlib.sha256(HDF5_PATH.read_bytes()).hexdigest()
    exceeds_commit_threshold = file_bytes > COMMIT_SIZE_THRESHOLD_BYTES
    if exceeds_commit_threshold:
        print(f"NOTE: {HDF5_PATH.name} is {file_bytes} bytes (> {COMMIT_SIZE_THRESHOLD_BYTES} byte threshold) "
              "-- left untracked per .gitignore's *.hdf5 rule (no exception added for this file); "
              "only its SHA-256/size/regeneration command are recorded.")

    summary = {
        "collection_spec_sha256": spec_hash,
        "canonical_manifest_sha256": manifest_hash(),
        "decoder_configuration_hash": decoder_configuration_hash(SUBSTEPS_PER_TRANSITION),
        "n_configs_attempted": len(configs),
        "n_success": n_success,
        "n_failed_interaction": n_failed_interaction,
        "n_rejected_by_reachability": n_rejected,
        "n_diagnostic_total": n_diagnostic_total,
        "intended_n_success": 24,
        "intended_n_diagnostic": 8,
        "matches_intended_split": (n_success == 24 and n_diagnostic_total == 8),
        "attempted_configs": attempted,
        "hdf5_path": str(HDF5_PATH.relative_to(ROOT)),
        "hdf5_size_bytes": file_bytes,
        "hdf5_sha256": sha256,
        "hdf5_exceeds_commit_size_threshold": exceeds_commit_threshold,
        "hdf5_regeneration_command": "python3 -m tasks.g1_pick_place.collect_dataset",
        "collection_wall_time_s": collection_wall_s,
        "sim_time_total_s": sim_time_total_s,
        "wall_time_physics_trials_only_s": wall_time_total_s,
    }
    (LOG_DIR / "phase5e_collection_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "attempted_configs"}, indent=2))
    print(f"n_success={n_success} n_failed_interaction={n_failed_interaction} n_rejected={n_rejected} "
          f"(intended 24 success / 8 diagnostic)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
