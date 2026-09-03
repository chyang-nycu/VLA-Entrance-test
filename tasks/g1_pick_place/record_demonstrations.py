#!/usr/bin/env python3
"""Phase 5B: Task 1 VLA demonstration dataset collector.

Collects exactly three episodes (nominal, x_minus_0.03, x_plus_0.03) into
`data/task1_prototype.hdf5`, using the SAME deterministic pipeline every
prior phase has used (tasks.g1_pick_place.run_pick_place.run_trial_pick_place,
non-oriented IK, Phase 4E-lineage scene via write_grasp_scene_5a) -- this
module does not reimplement any control or success-detection logic; it only
observes the real trial via `frame_callback` and records what actually
happened.

Transition convention (Section A): observation_t -> action_t -> physics
substeps -> observation_t+1.

- Physics frequency: 1 / TIMESTEP = 500 Hz (tasks.g1_pick_place.run_pick_place.TIMESTEP).
- Policy/control (recording) frequency: POLICY_HZ = 10 Hz.
- Substeps per recorded transition: PHYSICS_HZ / POLICY_HZ = 50 raw mj_step
  calls between consecutive recorded observations.
- RGB frequency: same as policy frequency (one frame per recorded
  observation, not per physics step -- rendering at 500 Hz was measured in
  Phase 5A as far slower than real-time and is not needed for a policy that
  only sees the world at the recording rate).
- observation_0 is captured from a read-only probe (a separate MjModel/
  MjData built from the identical scene and cube offset, stepped zero
  times) at the exact pre-first-action RESET state -- this project's
  pipeline has no RNG anywhere, so this probe reproduces the real trial's
  own t=0 state bit-for-bit. observation_{k} for k>=1 is captured at the
  physics step where a 50-step block completes (call count 50*k), i.e. the
  state that block's actions produced. action_{k-1} is the actuator control
  vector *applied during the final physics substep of that same block* --
  the same callback invocation that produces observation_k also records
  action_{k-1}, so by construction len(observations) == len(actions) + 1
  and neither array can be shifted relative to the other (asserted in
  tests/test_phase5b_dataset.py).
- Terminal-transition convention: the last recorded observation is terminal
  (no paired action follows it). Episode metadata separately records
  `success` (bool, from the trial's own `task_pass`) and
  `termination_reason` ("DONE" on success, else the trial's own
  `failure_state`). Trailing physics steps after the last complete 50-step
  block (fewer than 50) are not recorded as a partial transition.

Policy-observation vs. privileged split (Section B): `policy_observations/`
holds only what an onboard VLA policy could actually see (RGB, arm joint
positions/velocities, TCP pose, gripper state). `privileged/` holds
simulator-only ground truth (cube pose/velocity, target pose, contact
booleans/force, state-machine phase string) and is a SEPARATE top-level
group, never nested under `policy_observations/`.
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
ARTIFACT_DIR = ROOT / "artifacts" / "phase5b_sample_frames"

SCHEMA_VERSION = "1.0.0"
TIMESTEP = 0.002  # tasks.g1_pick_place.run_pick_place.TIMESTEP, duplicated here as a
# read-only constant for documentation; the recorder derives timing from the
# real trial's own callback cadence, not by re-deriving physics.
POLICY_HZ = 10.0
SUBSTEPS_PER_TRANSITION = int(round((1.0 / TIMESTEP) / POLICY_HZ))  # 50

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
    """Read-only reconstruction of the trial's own t=0 (post-RESET,
    pre-first-action) state -- a separate MjData, zero physics steps taken,
    so it never touches the real trial's own CubeInitGuard instance. Valid
    because this pipeline is fully deterministic (no RNG anywhere).
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
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")

    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    guard = CubeInitGuard(data, cube_qpos_adr, cube_dof_adr)
    guard.set_initial_pose([cube_x, cube_y, cube_z])
    mujoco.mj_forward(model, data)
    # This probe never calls mj_step -- guard.lock() is never invoked, and
    # this MjData/guard instance is discarded immediately after, so it never
    # interacts with the real trial's own state-machine loop or its own
    # CubeInitGuard instance.

    renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)
    renderer.update_scene(data, camera="head_cam")
    rgb = renderer.render().copy()
    renderer.close()

    tcp_pos = data.site_xpos[site_id].copy()
    tcp_quat = _mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3))
    left_c = _contacts_between(data, model, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom"), left_pad_id)
    right_c = _contacts_between(data, model, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom"), right_pad_id)
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
        "cube_linvel": cube_vel[:3],
        "cube_angvel": cube_vel[3:],
        "left_contact": bool(left_c),
        "right_contact": bool(right_c),
        "contact_force_n": 0.0,
        "phase": "RESET",
    }


def collect_episode(scene_path: Path, variant_id: str, cube_xy_offset: tuple[float, float]) -> dict:
    """Runs one real Task 1 trial via run_trial_pick_place, recording a
    downsampled (POLICY_HZ) observation/action stream via frame_callback.
    Returns a dict of numpy arrays plus the trial's own result dict
    (untouched, for episode metadata and cross-checking).
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
    observations = {k: [probe[k]] for k in (
        "t", "rgb", "joint_positions", "joint_velocities", "gripper_state",
        "tcp_pos", "tcp_quat", "cube_pos", "cube_quat", "cube_linvel",
        "cube_angvel", "left_contact", "right_contact", "contact_force_n", "phase",
    )}
    actions = {"cartesian_target": [], "arm_joint_position_target": [], "gripper_target": [], "applied_ctrl": []}

    call_count = [0]
    sim_render_start = time.perf_counter()
    render_time_accum = [0.0]

    def frame_callback(phase: str, m: mujoco.MjModel, data: mujoco.MjData) -> None:
        call_count[0] += 1
        if call_count[0] % SUBSTEPS_PER_TRANSITION != 0:
            return
        base = _base_phase(phase)
        target = _phase_target(base, cube_pos, lift_target)
        finger_open = _finger_open_for_phase(base)

        arm_ctrl = data.ctrl[arm_map.actuator_id].copy()
        gripper_ctrl = data.ctrl[gripper_map.actuator_id].copy()
        gripper_target = _finger_targets(gripper_map, finger_open)

        actions["cartesian_target"].append(target.copy())
        actions["arm_joint_position_target"].append(arm_ctrl.copy())
        actions["gripper_target"].append(gripper_target)
        actions["applied_ctrl"].append(np.concatenate([arm_ctrl, gripper_ctrl]))

        t0 = time.perf_counter()
        renderer.update_scene(data, camera="head_cam")
        rgb = renderer.render().copy()
        render_time_accum[0] += time.perf_counter() - t0

        cube_xyz = data.xpos[cube_body_id].copy()
        cube_quat = data.xquat[cube_body_id].copy()
        cube_vel = data.qvel[cube_dof_adr:cube_dof_adr + 6].copy()
        left_c = _contacts_between(data, model, cube_geom_id, left_pad_id)
        right_c = _contacts_between(data, model, cube_geom_id, right_pad_id)

        force_n = 0.0
        for pad_id in (left_pad_id, right_pad_id):
            for ci in range(data.ncon):
                con = data.contact[ci]
                pair = (int(con.geom1), int(con.geom2))
                if cube_geom_id in pair and pad_id in pair:
                    force6 = np.zeros(6)
                    mujoco.mj_contactForce(model, data, ci, force6)
                    force_n = max(force_n, float(abs(force6[0])))

        observations["t"].append(float(data.time))
        observations["rgb"].append(rgb)
        observations["joint_positions"].append(arm_map.get_qpos(data))
        observations["joint_velocities"].append(arm_map.get_qvel(data))
        observations["gripper_state"].append(gripper_map.get_qpos(data))
        observations["tcp_pos"].append(data.site_xpos[site_id].copy())
        observations["tcp_quat"].append(_mat_to_quat_wxyz(data.site_xmat[site_id].reshape(3, 3)))
        observations["cube_pos"].append(cube_xyz)
        observations["cube_quat"].append(cube_quat)
        observations["cube_linvel"].append(cube_vel[:3])
        observations["cube_angvel"].append(cube_vel[3:])
        observations["left_contact"].append(bool(left_c))
        observations["right_contact"].append(bool(right_c))
        observations["contact_force_n"].append(force_n)
        observations["phase"].append(phase)

    wall_start = time.perf_counter()
    result = run_trial_pick_place(
        scene_path, cube_xy_offset=cube_xy_offset,
        gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
        frame_callback=frame_callback,
    )
    wall_total_s = time.perf_counter() - wall_start
    renderer.close()

    n_obs = len(observations["t"])
    n_act = len(actions["cartesian_target"])
    assert n_obs == n_act + 1, f"transition alignment invariant violated: {n_obs} observations, {n_act} actions"

    arrays = {}
    for k in ("t", "joint_positions", "joint_velocities", "gripper_state", "tcp_pos", "tcp_quat",
              "cube_pos", "cube_quat", "cube_linvel", "cube_angvel", "left_contact", "right_contact",
              "contact_force_n"):
        arrays[f"obs_{k}"] = np.stack(observations[k]).astype(np.float64 if k == "t" else (np.bool_ if "contact" in k and k != "contact_force_n" else np.float32))
    arrays["obs_rgb"] = np.stack(observations["rgb"]).astype(np.uint8)
    arrays["obs_phase"] = np.array(observations["phase"], dtype=object)
    for k in ("cartesian_target", "arm_joint_position_target", "gripper_target", "applied_ctrl"):
        arrays[f"act_{k}"] = np.stack(actions[k]).astype(np.float32)

    return {
        "variant_id": variant_id,
        "cube_xy_offset": cube_xy_offset,
        "arrays": arrays,
        "n_transitions": n_act,
        "n_observations": n_obs,
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
        f.attrs["mujoco_version"] = manifest["mujoco_version"]
        f.attrs["robot_embodiment"] = manifest["robot_embodiment"]
        f.attrs["unitree_mujoco_pinned_commit"] = manifest["unitree_mujoco_pinned_commit"]
        f.attrs["project_git_commit"] = manifest["project_git_commit_before_phase5b"]
        f.attrs["task_id"] = manifest["task_id"]
        f.attrs["task_instruction"] = manifest["task_instruction"]
        f.attrs["transition_convention"] = "observation_t -> action_t -> physics_substeps -> observation_t+1"
        f.attrs["policy_control_hz"] = POLICY_HZ
        f.attrs["physics_hz"] = 1.0 / TIMESTEP
        f.attrs["substeps_per_transition"] = SUBSTEPS_PER_TRANSITION
        f.attrs["rgb_hz"] = POLICY_HZ
        f.attrs["terminal_convention"] = (
            "The final recorded observation in each episode is terminal and has no "
            "paired action; len(observations) == len(actions) + 1 for every episode "
            "(enforced at collection time, re-checked by validate_dataset.py)."
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
            g.attrs["canonical_manifest_sha256"] = m_hash
            g.attrs["cube_xy_offset"] = list(ep["cube_xy_offset"])
            g.attrs["final_xy_target_error_m"] = (
                ep["final_xy_target_error_m"] if ep["final_xy_target_error_m"] is not None else float("nan")
            )

            arrays = ep["arrays"]
            po = g.create_group("policy_observations")
            po.create_dataset("rgb", data=arrays["obs_rgb"], compression="gzip", compression_opts=4)
            po.create_dataset("joint_positions", data=arrays["obs_joint_positions"])
            po.create_dataset("joint_velocities", data=arrays["obs_joint_velocities"])
            po.create_dataset("tcp_pose", data=np.concatenate([arrays["obs_tcp_pos"], arrays["obs_tcp_quat"]], axis=1))
            po.create_dataset("gripper_state", data=arrays["obs_gripper_state"])
            po.create_dataset("timestamps", data=arrays["obs_t"])
            po.attrs["rgb_channel_order"] = "RGB"
            po.attrs["rgb_note"] = "Raw frames, no video-text overlay (that overlay is evidence-video-only)."
            po.attrs["tcp_pose_layout"] = "[x, y, z, qw, qx, qy, qz]"

            act = g.create_group("actions")
            act.create_dataset("cartesian_target", data=arrays["act_cartesian_target"])
            act.create_dataset("arm_joint_position_target", data=arrays["act_arm_joint_position_target"])
            act.create_dataset("gripper_target", data=arrays["act_gripper_target"])
            act.create_dataset("applied_ctrl", data=arrays["act_applied_ctrl"])
            act.attrs["applied_ctrl_layout"] = "[arm(7), gripper(2)] concatenated, actuator order = RIGHT_ARM_ACTUATORS + GRIPPER_ACTUATORS"
            act.attrs["note"] = (
                "arm_joint_position_target IS the applied arm ctrl (native MuJoCo <position> "
                "actuators: ctrl == target). gripper_target is the software PD position "
                "target (tasks.g1_pick_place.run_grasp_test_3c._finger_targets); "
                "applied_ctrl's gripper channels are the resulting bounded PD torque "
                "command (tasks.g1_pick_place.controller.bounded_pd_step), which differs "
                "numerically from gripper_target -- this is documented, not a bug."
            )

            priv = g.create_group("privileged")
            priv.create_dataset("cube_pos", data=arrays["obs_cube_pos"])
            priv.create_dataset("cube_quat", data=arrays["obs_cube_quat"])
            priv.create_dataset("cube_linvel", data=arrays["obs_cube_linvel"])
            priv.create_dataset("cube_angvel", data=arrays["obs_cube_angvel"])
            priv.create_dataset("target_pos", data=np.array([TARGET_POS[0], TARGET_POS[1], TARGET_RELEASE_Z], dtype=np.float32))
            priv.create_dataset("left_contact", data=arrays["obs_left_contact"])
            priv.create_dataset("right_contact", data=arrays["obs_right_contact"])
            priv.create_dataset("bilateral_contact", data=np.logical_and(arrays["obs_left_contact"], arrays["obs_right_contact"]))
            priv.create_dataset("contact_force_n", data=arrays["obs_contact_force_n"])
            priv.create_dataset("phase", data=np.array([p.encode("utf-8") for p in arrays["obs_phase"]]))
            priv.attrs["note"] = "Simulator-only ground truth. NOT part of the declared VLA policy-input group (policy_observations/)."


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

    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    episodes = []
    summary = {"episodes": [], "manifest_sha256": manifest_hash()}
    for spec in EPISODES:
        ep = collect_episode(scene_path, spec["variant_id"], spec["cube_xy_offset"])
        episodes.append(ep)
        rgb = ep["arrays"]["obs_rgb"]
        n = rgb.shape[0]
        for tag, idx in (("first", 0), ("middle", n // 2), ("final", n - 1)):
            import imageio.v2 as imageio
            out = ARTIFACT_DIR / f"{ep['variant_id']}_{tag}.png"
            imageio.imwrite(out, rgb[idx])
        summary["episodes"].append({
            "variant_id": ep["variant_id"],
            "success": ep["task_pass"],
            "failure_state": ep["failure_state"],
            "failure_reason": ep["failure_reason"],
            "n_transitions": ep["n_transitions"],
            "n_observations": ep["n_observations"],
            "physics_steps_total": ep["physics_steps_total"],
            "wall_total_s": ep["wall_total_s"],
            "render_time_s": ep["render_time_s"],
            "final_xy_target_error_m": ep["final_xy_target_error_m"],
        })
        print(f"[{ep['variant_id']}] success={ep['task_pass']} transitions={ep['n_transitions']} wall={ep['wall_total_s']:.2f}s")

    out_path = DATA_DIR / "task1_prototype.hdf5"
    _write_hdf5(episodes, out_path)

    file_bytes = out_path.stat().st_size
    sha256 = hashlib.sha256(out_path.read_bytes()).hexdigest()
    summary["hdf5_path"] = str(out_path.relative_to(ROOT))
    summary["hdf5_size_bytes"] = file_bytes
    summary["hdf5_sha256"] = sha256

    (LOG_DIR / "phase5b_collection_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
