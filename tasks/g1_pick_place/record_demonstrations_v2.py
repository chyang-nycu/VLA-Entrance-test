#!/usr/bin/env python3
"""Phase 5C: two-rate (policy + execution) VLA demonstration collector.

Root cause of Phase 5B's ~4.9 cm nominal replay error (quantified in
reports/phase5c-replay-fidelity.md, Section A): `run_pick_place._drive_smooth`
re-solves and RAMPS the commanded arm joint target on every physics step
(TIMESTEP=0.002s, 500 Hz) during LIFT/TRANSPORT_ABOVE_TARGET/LOWER_TO_TARGET
-- the arm actuators are native MuJoCo `<position>` servos, so `data.ctrl`
for the arm literally IS the instantaneous target. Phase 5B's dataset only
stored a snapshot of that target once per 50-step (10 Hz) block; replaying it
as a zero-order hold across the whole 100 ms block discarded the entire
intra-block ramp, which is exactly where the 4.9 cm came from (see the report
for the direct A/B measurement isolating this).

This module does not change that: it adds a second, execution-rate group
(`execution/`, one row per physics step -- i.e. the same 500 Hz the ramp
actually runs at) that records literally the same `applied_ctrl` vector
`run_pick_place._step_once` sends to `mujoco.mj_step` on every step, so
"exact execution replay" can feed it back byte-for-byte instead of
re-deriving it from a coarser sample. The 10 Hz `policy/` group is
unchanged in spirit from Phase 5B's `policy_observations/` -- observations
at T+1, one static per-phase Cartesian goal as the "action" at T -- and is
what a real onboard VLA policy would actually see/emit.

This module does not reimplement any control or success-detection logic; it
only observes the real trial (tasks.g1_pick_place.run_pick_place.run_trial_pick_place)
via `frame_callback`, exactly like Phase 5B's collector. Same canonical
manifest, same scene/controller/gains as Phase 5B -- verified against
data/task1_canonical_config.json at the start of `main()`, fail-loud on any
mismatch.
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
from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS, TARGET_RELEASE_Z
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard, _contacts_between
from tasks.g1_pick_place.run_grasp_test_3c import LIFT_DZ, PREGRASP_DZ, _finger_targets
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E, RETREAT_POS,
    TRANSPORT_ABOVE_TARGET_POS, LOWER_TO_TARGET_POS, run_trial_pick_place,
)

ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
LOG_DIR = ROOT / "logs"
ARTIFACT_DIR = ROOT / "artifacts" / "phase5b_sample_frames"  # shared with Phase 5B; no new dir needed for v2 stills

SCHEMA_VERSION = "2.0.0"
TIMESTEP = 0.002  # tasks.g1_pick_place.run_pick_place.TIMESTEP
POLICY_HZ = 10.0
SUBSTEPS_PER_TRANSITION = int(round((1.0 / TIMESTEP) / POLICY_HZ))  # 50
EXECUTION_HZ = 1.0 / TIMESTEP  # 500 Hz -- see module docstring for why this,
# not a coarser rate, is the honest choice: the arm's commanded set-point
# changes every physics step during ramped segments, so anything coarser
# would silently reintroduce the exact information loss this phase exists
# to fix.

EPISODES = [
    {"variant_id": "nominal", "cube_xy_offset": (0.0, 0.0)},
    {"variant_id": "x_minus_0.03", "cube_xy_offset": (-0.03, 0.0)},
    {"variant_id": "x_plus_0.03", "cube_xy_offset": (0.03, 0.0)},
]


def _base_phase(phase: str) -> str:
    return phase.split("_wp", 1)[0]


def _phase_target(base: str, cube_pos: np.ndarray, lift_target: np.ndarray) -> np.ndarray:
    mapping = {
        "PREGRASP": cube_pos + np.array([0.0, 0.0, PREGRASP_DZ]),
        "SETTLE_PREGRASP": cube_pos + np.array([0.0, 0.0, PREGRASP_DZ]),
        "APPROACH": cube_pos,
        "SETTLE_APPROACH": cube_pos,
        "CLOSE": cube_pos,
        "LIFT": lift_target,
        "HOLD": lift_target,
        "TRANSPORT_ABOVE_TARGET": TRANSPORT_ABOVE_TARGET_POS,
        "SETTLE_ABOVE_TARGET": TRANSPORT_ABOVE_TARGET_POS,
        "LOWER_TO_TARGET": LOWER_TO_TARGET_POS,
        "SETTLE_LOWER": LOWER_TO_TARGET_POS,
        "OPEN": LOWER_TO_TARGET_POS,
        "RELEASE_SETTLE": LOWER_TO_TARGET_POS,
        "RETREAT": RETREAT_POS,
        "VERIFY_TASK_SUCCESS": RETREAT_POS,
    }
    return mapping.get(base, cube_pos)


_GRIPPER_OPEN_PHASES = {
    "PREGRASP", "SETTLE_PREGRASP", "APPROACH", "SETTLE_APPROACH",
    "OPEN", "RELEASE_SETTLE", "RETREAT", "VERIFY_TASK_SUCCESS",
}


def _finger_open_for_phase(base: str) -> bool:
    return base in _GRIPPER_OPEN_PHASES


def _mat_to_quat_wxyz(mat9: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(mat9).flatten())
    return q


def _capture_probe_observation(model_path: Path, cube_xy_offset: tuple[float, float]) -> dict:
    """Identical to Phase 5B's own probe (tasks.g1_pick_place.record_demonstrations):
    a read-only, zero-physics-step reconstruction of the trial's own t=0
    state, valid because this pipeline has no RNG anywhere.
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
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
    # This probe never calls mj_step -- guard.lock() is never invoked, and
    # this MjData/guard instance is discarded immediately after.

    renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)
    renderer.update_scene(data, camera="head_cam")
    rgb = renderer.render().copy()
    renderer.close()

    tcp_pos = data.site_xpos[site_id].copy()
    tcp_quat = _mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3))
    cube_vel = data.qvel[cube_dof_adr:cube_dof_adr + 6].copy()

    return {
        "t": float(data.time),
        "rgb": rgb,
        "joint_positions": arm_map.get_qpos(data),
        "joint_velocities": arm_map.get_qvel(data),
        "gripper_state": gripper_map.get_qpos(data),
        "tcp_pos": tcp_pos,
        "tcp_quat": tcp_quat,
        "cube_pos": data.xpos[cube_body_id].copy(),
        "cube_quat": data.xquat[cube_body_id].copy(),
        "phase": "RESET",
    }


def collect_episode(scene_path: Path, variant_id: str, cube_xy_offset: tuple[float, float]) -> dict:
    """Runs one real Task 1 trial via run_trial_pick_place, recording BOTH:
    - a 10 Hz policy-rate stream (observations at T+1, static per-phase
      Cartesian/gripper high-level action at T, RGB at T+1 only) -- same
      semantics as Phase 5B's collector;
    - a 500 Hz execution-rate stream: one row per physics step, capturing
      the literal `data.ctrl` vector applied that step (arm target == ctrl
      for these native <position> actuators; gripper ctrl is the resulting
      bounded-PD torque command) plus joint/TCP/cube state and the
      transition index each step belongs to.

    frame_callback is invoked by run_trial_pick_place after EVERY physics
    step (unlike Phase 5B's, which only acted every 50th call) -- the
    execution row is appended unconditionally; the policy-rate snapshot
    (including the render) is still gated to every 50th call, identical to
    Phase 5B, so RGB storage volume and render time are unchanged from
    Phase 5B for the same episode.
    """
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    gripper_map = JointMap.build(model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    cube_dof_adr = int(model.jnt_dofadr[cube_joint_id])
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")

    cube_pos = np.array([CUBE_POS[0] + cube_xy_offset[0], CUBE_POS[1] + cube_xy_offset[1], CUBE_POS[2]])
    lift_target = cube_pos + np.array([0.0, 0.0, LIFT_DZ])

    renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)

    probe = _capture_probe_observation(scene_path, cube_xy_offset)
    policy_obs = {k: [probe[k]] for k in (
        "t", "rgb", "joint_positions", "joint_velocities", "gripper_state",
        "tcp_pos", "tcp_quat", "cube_pos", "cube_quat", "phase",
    )}
    high_level_actions = {"cartesian_target": [], "gripper_command_open": []}

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
        target = _phase_target(base, cube_pos, lift_target)
        finger_open = _finger_open_for_phase(base)
        transition_index = (call_count[0] - 1) // SUBSTEPS_PER_TRANSITION

        # --- execution rate (every call): record the LITERAL applied ctrl.
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

        # --- policy rate (every SUBSTEPS_PER_TRANSITION-th call): unchanged
        # from Phase 5B, including the RGB render.
        if call_count[0] % SUBSTEPS_PER_TRANSITION != 0:
            return

        high_level_actions["cartesian_target"].append(target.copy())
        high_level_actions["gripper_command_open"].append(bool(finger_open))

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
    n_act = len(high_level_actions["cartesian_target"])
    assert n_obs == n_act + 1, f"transition alignment invariant violated: {n_obs} observations, {n_act} actions"
    n_exec_expected = n_act * SUBSTEPS_PER_TRANSITION
    # Trailing steps beyond the last complete 50-step block (e.g. a mid-block
    # early failure) are recorded in exec_rows but NOT part of any complete
    # policy transition -- truncate the execution trace to exactly the
    # complete transitions, identical in spirit to Phase 5B's own "trailing
    # steps ... are not recorded as a partial transition" rule for observations.
    for k in exec_rows:
        exec_rows[k] = exec_rows[k][:n_exec_expected]

    arrays = {}
    for k in ("t", "joint_positions", "joint_velocities", "gripper_state", "tcp_pos", "tcp_quat", "cube_pos", "cube_quat"):
        arrays[f"obs_{k}"] = np.stack(policy_obs[k]).astype(np.float64 if k == "t" else np.float32)
    arrays["obs_rgb"] = np.stack(policy_obs["rgb"]).astype(np.uint8)
    arrays["obs_phase"] = np.array(policy_obs["phase"], dtype=object)
    arrays["act_cartesian_target"] = np.stack(high_level_actions["cartesian_target"]).astype(np.float32)
    arrays["act_gripper_command_open"] = np.array(high_level_actions["gripper_command_open"], dtype=np.bool_)

    for k in ("t", "arm_joint_target", "gripper_target", "applied_ctrl", "joint_positions",
              "joint_velocities", "tcp_pos", "tcp_quat", "cube_pos", "cube_quat"):
        arrays[f"exec_{k}"] = np.stack(exec_rows[k]).astype(np.float64 if k == "t" else np.float32)
    arrays["exec_transition_index"] = np.array(exec_rows["transition_index"], dtype=np.int32)
    arrays["exec_phase"] = np.array(exec_rows["phase"], dtype=object)

    return {
        "variant_id": variant_id,
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


def _write_hdf5(episodes: list[dict], out_path: Path) -> None:
    manifest = load_manifest()
    m_hash = manifest_hash()
    cam = manifest["camera"]

    with h5py.File(out_path, "w") as f:
        f.attrs["schema_version"] = SCHEMA_VERSION
        f.attrs["schema_version_note"] = (
            "2.0.0 is this DATASET's own schema version (two-rate policy/"
            "execution representation, Phase 5C). It is unrelated to "
            "data/task1_canonical_config.json's own 'schema_version' field "
            "(1.0.0, the manifest file FORMAT version), which is unchanged -- "
            "the manifest's scene/controller/gains/thresholds/camera content "
            "is byte-identical to Phase 5B, per this phase's authorization."
        )
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
            "len(policy/high_level_actions) + 1 for every episode. The "
            "execution/ group holds exactly n_transitions * substeps_per_transition "
            "rows (trailing steps past the last complete 50-step block, e.g. "
            "from a mid-block early failure, are not recorded, same rule as "
            "Phase 5B applied to observations)."
        )
        f.attrs["camera_params_json"] = json.dumps(cam)
        f.attrs["coordinate_conventions"] = (
            "World frame: MuJoCo world frame (right-handed, Z-up). "
            "Quaternions: (w, x, y, z), MuJoCo convention. "
            "RGB: uint8, HWC, channel order R,G,B. Positions in meters."
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
            g.attrs["cube_xy_offset"] = list(ep["cube_xy_offset"])
            g.attrs["final_xy_target_error_m"] = (
                ep["final_xy_target_error_m"] if ep["final_xy_target_error_m"] is not None else float("nan")
            )

            arrays = ep["arrays"]

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
            obs.attrs["rgb_note"] = "Raw frames, no video-text overlay (that overlay is evidence-video-only). Stored ONLY at policy (10 Hz) rate -- not duplicated in execution/."
            obs.attrs["tcp_pose_layout"] = "[x, y, z, qw, qx, qy, qz]"

            hla = policy.create_group("high_level_actions")
            hla.create_dataset("cartesian_target", data=arrays["act_cartesian_target"])
            hla.create_dataset("gripper_command_open", data=arrays["act_gripper_command_open"])
            hla.attrs["note"] = (
                "cartesian_target is the static per-phase Cartesian goal in effect "
                "for physics steps [T*50+1 .. (T+1)*50] (the same quantity Phase 5B "
                "stored as 'cartesian_target', unchanged semantics). This is NOT the "
                "fine-grained internal sub-waypoint the low-level controller actually "
                "ramps through every physics step -- see execution/arm_joint_target "
                "for that. gripper_command_open is the binary open/close command a "
                "policy would emit; the low-level gripper PD target/gains are "
                "decoder-side, not part of this high-level action."
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
                "The literal data.ctrl vector run_pick_place._step_once sent to "
                "mujoco.mj_step on this exact physics step -- for the arm (native "
                "<position> actuators) this equals arm_joint_target; for the gripper "
                "it is the bounded_pd_step output torque, which differs numerically "
                "from gripper_target (the desired open/close position). Exact "
                "execution replay uses this field directly, not a re-derivation."
            )
            execu.attrs["transition_index_note"] = (
                "transition_index[i] = k means execution row i belongs to policy "
                "transition k, i.e. it is one of the 50 physics steps between "
                "policy/observations[k] and policy/observations[k+1]."
            )

            priv = g.create_group("privileged")
            priv.create_dataset("cube_pos", data=arrays["obs_cube_pos"])
            priv.create_dataset("cube_quat", data=arrays["obs_cube_quat"])
            priv.create_dataset("target_pos", data=np.array([TARGET_POS[0], TARGET_POS[1], TARGET_RELEASE_Z], dtype=np.float32))
            priv.create_dataset("phase", data=np.array([p.encode("utf-8") for p in arrays["obs_phase"]]))
            priv.attrs["note"] = (
                "Simulator-only ground truth at POLICY (10 Hz) rate, mirroring "
                "Phase 5B's privileged/ group for continuity. NOT part of the "
                "declared VLA policy-input group (policy/observations/). "
                "execution/cube_pos and execution/cube_quat carry the same "
                "quantity at the finer 500 Hz execution rate, for replay comparison."
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
    summary = {"episodes": [], "manifest_sha256": manifest_hash(), "schema_version": SCHEMA_VERSION}
    for spec in EPISODES:
        ep = collect_episode(scene_path, spec["variant_id"], spec["cube_xy_offset"])
        episodes.append(ep)
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
            "render_time_s": ep["render_time_s"],
            "final_xy_target_error_m": ep["final_xy_target_error_m"],
        })
        print(f"[{ep['variant_id']}] success={ep['task_pass']} transitions={ep['n_transitions']} exec_rows={ep['n_execution_rows']} wall={ep['wall_total_s']:.2f}s")

    out_path = DATA_DIR / "task1_prototype_v2.hdf5"
    _write_hdf5(episodes, out_path)

    file_bytes = out_path.stat().st_size
    sha256 = hashlib.sha256(out_path.read_bytes()).hexdigest()
    summary["hdf5_path"] = str(out_path.relative_to(ROOT))
    summary["hdf5_size_bytes"] = file_bytes
    summary["hdf5_sha256"] = sha256

    (LOG_DIR / "phase5c_v2_collection_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
