#!/usr/bin/env python3
"""Phase 5A smoke test: runs one fresh, unmodified nominal Task 1 episode
and captures the new onboard head camera (tasks/g1_pick_place/
camera_observation.py) throughout, producing:

- artifacts/phase5a_head_camera.mp4 (onboard view only, with a state-label
  text overlay)
- artifacts/phase5a_head_camera_frames/<PHASE>.png (one frame per required
  named phase, plus the first-bilateral-contact event)
- logs/phase5a_camera_smoke.json (all smoke-test measurements: visibility
  checks, physics-invariance check, performance)

Configuration choice (documented, not silently substituted): Task 1's most
recently authorized configuration is Phase 4F's orientation-constrained IK
(`write_grasp_scene_4f()`, `use_oriented_ik=True`) -- but that configuration
does not reach OPEN/VERIFY_TASK_SUCCESS in this project's own logged
behavior (`logs/phase4f_orientation_grasp.json`: `failure_state:
"SETTLE_LOWER"`; confirmed by re-running it here with this phase's camera
attached, same outcome). HANDOFF.md Phase 5A requires onboard visibility
evidence at OPEN and VERIFY_TASK_SUCCESS specifically, which is impossible
to obtain from a run that never reaches those states without altering the
trial. This script therefore uses Phase 4B/4C's configuration
(`write_grasp_scene_4b()`, `use_oriented_ik=False` -- the default), which is
the configuration that actually completes the full state machine
(deterministic task_pass=True, confirmed unregressed through Phase 4C-4F's
own repeated reruns). Both configurations share the identical cube, target,
gripper, and arm-gain constants; only the optional Phase 4F orientation
secondary objective differs, and that objective was itself an unsuccessful
attempt at reducing slip, not a defining part of "Task 1". This choice does
not touch, retune, or re-authorize either configuration -- it only selects
which of two already-existing, already-tested configurations this camera
smoke test observes. See reports/phase5a-onboard-camera.md, Section A, for
the full record of this decision.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from tasks.g1_pick_place.camera_observation import (
    CAM_FOVY_DEG,
    CAM_HEIGHT,
    CAM_WIDTH,
    HEAD_CAM_NAME,
    blue_target_mask,
    camera_extrinsic,
    camera_intrinsics,
    red_cube_mask,
    write_grasp_scene_5a,
)
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts"
FRAMES_DIR = ARTIFACTS / "phase5a_head_camera_frames"
LOGS = ROOT / "logs"
TIMESTEP = 0.002
FRAME_STRIDE = 17  # same cadence used by every prior phase's evidence video (~29.4 fps)
VIDEO_FPS = 1.0 / (TIMESTEP * FRAME_STRIDE)

REQUIRED_PHASES = [
    "PREGRASP", "APPROACH", "CLOSE", "LIFT", "HOLD",
    "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET", "OPEN", "VERIFY_TASK_SUCCESS",
]
# Phases where the cube must be visible per HANDOFF.md Section C. RESET and
# FIRST_BILATERAL_CONTACT are handled separately (RESET happens before any
# state-machine phase string exists; first-bilateral-contact is a contact
# EVENT, not a named phase).
REQUIRED_CUBE_VISIBLE = ["RESET", "PREGRASP", "APPROACH", "CLOSE", "FIRST_BILATERAL_CONTACT", "LIFT", "HOLD",
                          "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET", "OPEN", "VERIFY_TASK_SUCCESS"]
# Target is only required visible before placement completes (HANDOFF.md:
# "blue target before placement" / "cube at or above target during
# lowering") -- once the cube settles onto the pad, the pad is expected to
# be mostly covered by the cube itself, which is the physically-correct
# outcome of a successful placement, not a camera defect.
REQUIRED_TARGET_VISIBLE = ["RESET", "PREGRASP", "APPROACH", "CLOSE", "LIFT", "HOLD",
                            "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET"]


def _draw_overlay(frame: np.ndarray, phase: str) -> np.ndarray:
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    label = f"onboard head_cam | state: {phase}"
    draw.rectangle([2, 2, 2 + 6 * len(label) + 4, 14], fill=(0, 0, 0))
    draw.text((4, 3), label, fill=(255, 255, 0))
    return np.array(img)


def main() -> int:
    ARTIFACTS.mkdir(exist_ok=True)
    FRAMES_DIR.mkdir(exist_ok=True)

    scene = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
    model = mujoco.MjModel.from_xml_path(str(scene))
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, HEAD_CAM_NAME)
    renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")

    # --- RESET frame: captured from fresh MjData before any state-machine
    # step exists (the "RESET" phase string is never itself passed to
    # frame_callback -- see run_pick_place.py). ---
    data0 = mujoco.MjData(model)
    mujoco.mj_resetData(model, data0)
    mujoco.mj_forward(model, data0)
    renderer.update_scene(data0, camera=cam_id)
    reset_frame = renderer.render().copy()
    reset_extrinsic = camera_extrinsic(model, data0, cam_id)

    named_frames: dict[str, np.ndarray] = {"RESET": reset_frame}
    video_frames: list[np.ndarray] = [_draw_overlay(reset_frame.copy(), "RESET")]
    # Raw (no text overlay) frames at the SAME stride as video_frames, kept
    # separately for the temporal-variance / consecutive-frame-identity
    # diagnostic below -- that check must measure genuine rendered-scene
    # change, not overlay-text change, and must compare temporally adjacent
    # frames (this stride sequence), not the semantically-unrelated named
    # milestone frames in `named_frames` (which are snapshots at different,
    # non-adjacent instants and are not expected to differ from each other
    # in any particular way).
    raw_video_frames: list[np.ndarray] = [reset_frame.copy()]
    cam_positions: list[list[float]] = [reset_extrinsic["position_world"]]
    step_count = [0]
    first_contact_seen = [False]

    def _has_bilateral_contact(cb_data: mujoco.MjData) -> bool:
        left_ok = right_ok = False
        for ci in range(cb_data.ncon):
            con = cb_data.contact[ci]
            pair = (int(con.geom1), int(con.geom2))
            if cube_geom_id in pair and left_pad_id in pair:
                left_ok = True
            if cube_geom_id in pair and right_pad_id in pair:
                right_ok = True
        return left_ok and right_ok

    def frame_callback(phase: str, cb_model: mujoco.MjModel, cb_data: mujoco.MjData) -> None:
        step_count[0] += 1
        if not first_contact_seen[0] and _has_bilateral_contact(cb_data):
            renderer.update_scene(cb_data, camera=cam_id)
            named_frames["FIRST_BILATERAL_CONTACT"] = renderer.render().copy()
            first_contact_seen[0] = True
        for tp in REQUIRED_PHASES:
            if phase.startswith(tp) and tp not in named_frames:
                renderer.update_scene(cb_data, camera=cam_id)
                named_frames[tp] = renderer.render().copy()
        if step_count[0] % FRAME_STRIDE == 0:
            renderer.update_scene(cb_data, camera=cam_id)
            frame = renderer.render().copy()
            raw_video_frames.append(frame)
            video_frames.append(_draw_overlay(frame.copy(), phase))
            cam_positions.append(camera_extrinsic(cb_model, cb_data, cam_id)["position_world"])

    t_render_start = time.perf_counter()
    result = run_trial_pick_place(scene, frame_callback=frame_callback, use_oriented_ik=False)
    t_render_end = time.perf_counter()
    sim_plus_render_s = t_render_end - t_render_start
    n_rendered = len(video_frames)

    print(f"task_pass={result['task_pass']} height_gain_m={result['height_gain_m']}")
    print(f"phases captured: {list(named_frames.keys())}")

    # --- Physics-invariance check: rerun with NO rendering at all and
    # confirm the physics trajectory (height gain, task outcome) is
    # unaffected -- rendering must be read-only w.r.t. MjData. ---
    t_sim_only_start = time.perf_counter()
    result_no_render = run_trial_pick_place(scene, use_oriented_ik=False)
    t_sim_only_end = time.perf_counter()
    sim_only_s = t_sim_only_end - t_sim_only_start
    physics_unaffected_by_rendering = (
        result["height_gain_m"] == result_no_render["height_gain_m"]
        and result["task_pass"] == result_no_render["task_pass"]
        and result["final_xy_target_error_m"] == result_no_render["final_xy_target_error_m"]
    )

    # --- Visibility checks (smoke-test diagnostics only, per HANDOFF.md
    # Section D -- never task-success logic). ---
    visibility = {}
    for phase_name, frame in named_frames.items():
        visibility[phase_name] = {
            "shape": list(frame.shape),
            "dtype": str(frame.dtype),
            "finite": bool(np.all(np.isfinite(frame))),
            "in_range_0_255": bool(frame.min() >= 0 and frame.max() <= 255),
            "not_blank": bool(frame.std() > 1.0),
            "red_cube_px": int(red_cube_mask(frame).sum()),
            "blue_target_px": int(blue_target_mask(frame).sum()),
        }

    cube_visible_all_required = all(
        visibility[p]["red_cube_px"] > 0 for p in REQUIRED_CUBE_VISIBLE if p in visibility
    )
    target_visible_all_required = all(
        visibility[p]["blue_target_px"] > 0 for p in REQUIRED_TARGET_VISIBLE if p in visibility
    )
    missing_required_phases = [p for p in REQUIRED_CUBE_VISIBLE if p not in visibility]

    # Temporal variance / consecutive-frame checks: measured on the actual
    # stride-sampled VIDEO sequence (raw_video_frames, temporally adjacent
    # by construction -- every FRAME_STRIDE simulation steps), not on the
    # named milestone snapshots (which are deliberately non-adjacent and
    # not expected to differ by any particular amount from one another).
    diffs = [
        float(np.abs(raw_video_frames[i].astype(int) - raw_video_frames[i - 1].astype(int)).mean())
        for i in range(1, len(raw_video_frames))
    ]
    temporal_variance_nonzero = float(np.var(np.stack(raw_video_frames).astype(float))) > 0.0
    no_two_consecutive_identical = all(d > 0.0 for d in diffs)
    n_identical_consecutive_pairs = sum(1 for d in diffs if d == 0.0)

    # Camera pose vs. parent body: in this fixed-base (pelvis+torso welded)
    # configuration, torso_link never moves, so the onboard camera's world
    # position is expected to be EXACTLY constant across the whole episode
    # -- this is the correct behavior for a rigid head mount on a fixed
    # trunk, not a bug. Checked directly, not assumed.
    cam_pos_arr = np.array(cam_positions)
    # 1mm tolerance: the weld constraint is soft, not bit-exact rigid (see
    # camera_pose_note below) -- measured max deviation is ~0.19mm, two
    # orders of magnitude under this bound.
    cam_pose_constant = bool(np.allclose(cam_pos_arr, cam_pos_arr[0], atol=1e-3))
    cam_pose_max_deviation_m = float(np.abs(cam_pos_arr - cam_pos_arr[0]).max())

    # --- Performance ---
    n_frames_in_stride_loop = step_count[0] // FRAME_STRIDE
    achieved_render_fps = n_frames_in_stride_loop / sim_plus_render_s
    frame_nbytes = int(reset_frame.nbytes)
    episode_s = step_count[0] * TIMESTEP
    candidate_rates_hz = [5, 10, 20, 30]
    storage_estimate_15s = {
        f"{hz}hz": {
            "raw_uint8_mb": round(15 * hz * frame_nbytes / (1024 * 1024), 3),
            "jpeg_est_mb_at_8x": round(15 * hz * frame_nbytes / 8 / (1024 * 1024), 3),
        }
        for hz in candidate_rates_hz
    }
    recommended_rate_hz = 10
    recommendation_reason = (
        f"Achieved combined sim+render throughput was {achieved_render_fps:.2f} fps in this trial "
        f"(sim-only: {step_count[0] / sim_only_s:.1f} steps/s equivalent episode-time ratio "
        f"{episode_s / sim_only_s:.2f}x realtime; sim+render: {episode_s / sim_plus_render_s:.2f}x realtime). "
        "10 Hz leaves close to 2x headroom under the measured ~19 fps combined ceiling for a real-time "
        "onboard capture loop (leaving margin for compression/disk I/O/multi-camera setups later), while "
        "20 Hz would consume nearly all of the measured budget and is only safely usable for OFFLINE "
        "(non-real-time) dataset generation, where rendering can be decoupled from live stepping."
    )

    intrinsics = camera_intrinsics()

    log = {
        "scope": "Phase 5A onboard RGB observation smoke test",
        "configuration_used": "write_grasp_scene_4b + use_oriented_ik=False (see module docstring for why, not Phase 4F's config)",
        "task_pass": bool(result["task_pass"]),
        "height_gain_m": result["height_gain_m"],
        "camera_specification": {
            "parent_body": "torso_link",
            "pos_local": None,  # filled below
            "quat_local": None,
            "fovy_deg": CAM_FOVY_DEG,
            "resolution_wh": [CAM_WIDTH, CAM_HEIGHT],
            "znear_far_note": "MuJoCo <visual><map znear/zfar> are multiples of model.stat.extent, not absolute meters",
            "rgb_dtype_channel_order": "uint8, HWC, RGB (mujoco.Renderer.render() default)",
            "world_camera_convention": "camera looks down local -Z; local +Y is image-up; local +X is image-right (MuJoCo convention)",
            "extrinsic_at_reset": reset_extrinsic,
            "intrinsics": intrinsics,
        },
        "visibility": visibility,
        "cube_visible_at_all_required_phases": cube_visible_all_required,
        "target_visible_at_all_required_pre_placement_phases": target_visible_all_required,
        "missing_required_phases": missing_required_phases,
        "temporal_variance_nonzero": temporal_variance_nonzero,
        "no_two_consecutive_frames_identical": no_two_consecutive_identical,
        "n_identical_consecutive_pairs": n_identical_consecutive_pairs,
        "n_video_frames_checked": len(raw_video_frames),
        "frame_to_frame_mean_abs_diff_stats": {
            "min": float(np.min(diffs)), "max": float(np.max(diffs)), "mean": float(np.mean(diffs)),
        },
        "camera_pose_constant_under_fixed_base": cam_pose_constant,
        "camera_pose_max_deviation_m": cam_pose_max_deviation_m,
        "camera_pose_note": (
            "torso_link is rigidly welded to the world via a MuJoCo equality constraint in this "
            "fixed-base configuration. That constraint is soft (finite stiffness), not infinitely "
            "rigid, so the onboard camera position is constant to within solver precision "
            f"(max deviation {cam_pose_max_deviation_m * 1000:.4f} mm across the whole episode), "
            "not bit-exact zero. This is the correct, expected behavior for a rigid head mount "
            "on a fixed trunk -- the camera moves consistently with (i.e. negligibly, matching) "
            "its parent body, not independently of it."
        ),
        "rendering_does_not_alter_physics": physics_unaffected_by_rendering,
        "task_still_completes_with_unchanged_controller": bool(result["task_pass"]) and bool(result_no_render["task_pass"]),
        "performance": {
            "sim_only_wall_s": sim_only_s,
            "sim_plus_render_wall_s": sim_plus_render_s,
            "episode_sim_time_s": episode_s,
            "n_render_calls_in_stride_loop": n_frames_in_stride_loop,
            "achieved_combined_fps": achieved_render_fps,
            "per_frame_bytes_raw_uint8": frame_nbytes,
            "storage_estimate_per_15s_episode": storage_estimate_15s,
            "recommended_observation_rate_hz": recommended_rate_hz,
            "recommendation_reason": recommendation_reason,
        },
    }
    log["camera_specification"]["pos_local"] = None
    from tasks.g1_pick_place.camera_observation import CAM_POS_LOCAL, CAM_QUAT
    log["camera_specification"]["pos_local"] = CAM_POS_LOCAL.tolist()
    log["camera_specification"]["quat_local_wxyz"] = CAM_QUAT.tolist()

    LOGS.mkdir(exist_ok=True)
    with open(LOGS / "phase5a_camera_smoke.json", "w") as f:
        json.dump(log, f, indent=2)
    print(f"wrote {LOGS / 'phase5a_camera_smoke.json'}")

    # --- Save named-phase frames ---
    for phase_name, frame in named_frames.items():
        Image.fromarray(frame).save(FRAMES_DIR / f"{phase_name}.png")
    print(f"wrote {len(named_frames)} frames to {FRAMES_DIR}")

    # --- Write video (onboard view only) ---
    video_path = ARTIFACTS / "phase5a_head_camera.mp4"
    writer = imageio.get_writer(
        str(video_path), fps=VIDEO_FPS, codec="libx264", quality=None,
        ffmpeg_params=["-crf", "23", "-pix_fmt", "yuv420p"],
    )
    for f in video_frames:
        writer.append_data(f)
    writer.close()
    reader = imageio.get_reader(str(video_path))
    decoded = sum(1 for _ in reader)
    reader.close()
    if decoded != len(video_frames):
        raise RuntimeError(f"decoded frame count {decoded} != encoded frame count {len(video_frames)}")
    duration_s = decoded / VIDEO_FPS
    print(
        f"wrote {video_path}: {CAM_WIDTH}x{CAM_HEIGHT}, fps={VIDEO_FPS:.2f}, "
        f"frames={decoded}, duration={duration_s:.2f}s, size={video_path.stat().st_size} bytes, decode-verified"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
