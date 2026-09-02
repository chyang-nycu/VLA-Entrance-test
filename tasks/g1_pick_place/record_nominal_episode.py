#!/usr/bin/env python3
"""Phase 4C: renders the complete nominal Task 1 episode to
artifacts/phase4b_task1_nominal.mp4 (filename kept from Phase 4B; the video
is produced in Phase 4C) plus three still frames (grasp, transport-above-
target, final released-in-target), using MuJoCo's offscreen Renderer and a
fixed (non-tracking) third-person MjvCamera.

This does not change control, physics, or the trial's pass/fail outcome --
it only observes via `frame_callback`, a purely additive, opt-in hook on
run_trial_pick_place() (default None, unused elsewhere). Uses the exact
unchanged Phase 4B configuration (arm_kp=400.0, arm_kv=25.0,
gripper_kp=150.0, gripper_kd=10.0).
"""

from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, run_trial_pick_place
from tasks.g1_pick_place.gripper_scene import write_grasp_scene_4b

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts"
TIMESTEP = 0.002
VIDEO_FPS = 1.0 / (TIMESTEP * 17)  # ~29.41 fps -- normal (real-time) playback rate
FRAME_STRIDE = 17
WIDTH, HEIGHT = 640, 480

INSTRUCTION_TEXT = "Task 1: pick up the red cube and place it in the blue target area"


def _make_camera() -> mujoco.MjvCamera:
    """A single fixed (non-tracking) third-person camera, framed on the
    midpoint between the cube's start and the target pad, constant for the
    whole episode -- not re-aimed or re-positioned at any point.
    """
    cam = mujoco.MjvCamera()
    mid_x = (CUBE_POS[0] + TARGET_POS[0]) / 2.0
    mid_y = (CUBE_POS[1] + TARGET_POS[1]) / 2.0
    cam.lookat[:] = [mid_x, mid_y, 0.85]
    cam.distance = 1.7
    cam.azimuth = 200.0
    cam.elevation = -22.0
    return cam


def main() -> int:
    ARTIFACTS.mkdir(exist_ok=True)
    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    model = mujoco.MjModel.from_xml_path(str(scene))
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    cam = _make_camera()

    frames: list[np.ndarray] = []
    stills: dict[str, np.ndarray] = {}
    step_count = [0]
    last_bilateral_step: dict = {"frame": None}

    def frame_callback(phase: str, cb_model: mujoco.MjModel, cb_data: mujoco.MjData) -> None:
        step_count[0] += 1
        if step_count[0] % FRAME_STRIDE == 0:
            renderer.update_scene(cb_data, camera=cam)
            frames.append(renderer.render().copy())

        # Still 1: first step of VERIFY_BILATERAL_CONTACT is decided outside
        # _step_once, so approximate "successful grasp" as the first LIFT
        # step (grasp already verified by then).
        if phase == "LIFT" and "grasp" not in stills:
            renderer.update_scene(cb_data, camera=cam)
            stills["grasp"] = renderer.render().copy()
        # Still 2: midway through TRANSPORT_ABOVE_TARGET.
        if phase.startswith("TRANSPORT_ABOVE_TARGET_wp") and "transport" not in stills:
            wp_idx = int(phase.rsplit("wp", 1)[1])
            if wp_idx >= 20:  # roughly midway of the transport sub-waypoints
                renderer.update_scene(cb_data, camera=cam)
                stills["transport"] = renderer.render().copy()
        # Still 3: last step of VERIFY_TASK_SUCCESS (final released, in-target rest).
        if phase == "VERIFY_TASK_SUCCESS":
            renderer.update_scene(cb_data, camera=cam)
            stills["released"] = renderer.render().copy()

    result = run_trial_pick_place(scene, frame_callback=frame_callback)
    print(f"task_pass={result['task_pass']} states_entered={result['states_entered']}")
    print(f"rendered {len(frames)} frames at stride {FRAME_STRIDE} (~{VIDEO_FPS:.2f} fps)")

    if not result["task_pass"]:
        raise RuntimeError("nominal trial did not pass -- refusing to present a failing episode as the demo")

    video_path = ARTIFACTS / "phase4b_task1_nominal.mp4"
    writer = imageio.get_writer(
        str(video_path), fps=VIDEO_FPS, codec="libx264", quality=None,
        ffmpeg_params=["-crf", "23", "-pix_fmt", "yuv420p"],
    )
    for f in frames:
        writer.append_data(f)
    writer.close()

    for name, img in stills.items():
        imageio.imwrite(str(ARTIFACTS / f"phase4c_still_{name}.png"), img)

    size_bytes = video_path.stat().st_size
    print(f"wrote {video_path} ({size_bytes / 1e6:.2f} MB)")
    for name in stills:
        print(f"wrote artifacts/phase4c_still_{name}.png")

    # Verify decodability + report resolution/fps/frame count/duration.
    reader = imageio.get_reader(str(video_path))
    meta = reader.get_meta_data()
    decoded_frames = 0
    for _ in reader:
        decoded_frames += 1
    reader.close()
    duration_s = decoded_frames / VIDEO_FPS
    print(
        f"decode-verified: {WIDTH}x{HEIGHT}, fps={VIDEO_FPS:.2f}, "
        f"frames={decoded_frames}, duration={duration_s:.2f}s, size={size_bytes} bytes, "
        f"meta_fps={meta.get('fps')}"
    )
    if decoded_frames != len(frames):
        raise RuntimeError(f"decoded frame count {decoded_frames} != encoded frame count {len(frames)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
