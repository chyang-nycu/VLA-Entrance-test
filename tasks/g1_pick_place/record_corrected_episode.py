#!/usr/bin/env python3
"""Phase 4E, Section E: renders the corrected nominal Task 1 episode to
artifacts/phase4e_task1_corrected.mp4 (full state machine, fixed third-
person camera, burned-in overlay of task state / cube world-Z gain / live
TCP-frame slip / left-right normal contact force) plus a close-up
side-view clip (artifacts/phase4e_task1_closeup.mp4) where downward slip
would be visually obvious.

Uses the current (post-Phase-4E-repair) write_grasp_scene_4b/
run_trial_pick_place configuration unchanged -- this script only observes
via frame_callback, a purely additive, opt-in hook; it does not change
control, physics, or the trial's pass/fail outcome.

Does NOT overwrite artifacts/phase4d_failure_reproduction.mp4 (Phase 4D
failure evidence) or artifacts/phase4b_task1_nominal.mp4 (Phase 4C's
demo of the since-diagnosed-flawed grasp) -- both are historical and
preserved unchanged.
"""

from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B,
    ARM_KV_4B,
    run_trial_pick_place,
    tcp_local_cube_offset,
    write_grasp_scene_4b,
)

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts"
TIMESTEP = 0.002
FRAME_STRIDE = 17
VIDEO_FPS = 1.0 / (TIMESTEP * FRAME_STRIDE)  # ~29.41 fps -- normal (real-time) playback rate
WIDTH, HEIGHT = 640, 480
CLOSEUP_WIDTH, CLOSEUP_HEIGHT = 480, 360

INSTRUCTION_TEXT = "Task 1: pick up the red cube and place it in the blue target area"


def _make_camera() -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    mid_x = (CUBE_POS[0] + TARGET_POS[0]) / 2.0
    mid_y = (CUBE_POS[1] + TARGET_POS[1]) / 2.0
    cam.lookat[:] = [mid_x, mid_y, 0.85]
    cam.distance = 1.7
    cam.azimuth = 200.0
    cam.elevation = -22.0
    return cam


def _make_closeup_camera() -> mujoco.MjvCamera:
    """Tight view of the cube/gripper from the same general direction as
    the main camera (so the finger pads are not occluded by the wrist),
    framed so any downward slip of the cube relative to the fingers is
    directly visible."""
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [CUBE_POS[0] + 0.02, CUBE_POS[1], CUBE_POS[2] + 0.03]
    cam.distance = 0.55
    cam.azimuth = 205.0
    cam.elevation = -18.0
    return cam


def _draw_overlay(frame: np.ndarray, lines: list[str]) -> np.ndarray:
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    y = 6
    for line in lines:
        draw.rectangle([4, y - 2, 4 + 7 * len(line) + 6, y + 12], fill=(0, 0, 0))
        draw.text((7, y), line, fill=(255, 255, 0))
        y += 14
    return np.array(img)


def main() -> int:
    ARTIFACTS.mkdir(exist_ok=True)
    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    model = mujoco.MjModel.from_xml_path(str(scene))
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    closeup_renderer = mujoco.Renderer(model, height=CLOSEUP_HEIGHT, width=CLOSEUP_WIDTH)
    cam = _make_camera()
    closeup_cam = _make_closeup_camera()

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_tcp")

    frames: list[np.ndarray] = []
    closeup_frames: list[np.ndarray] = []
    stills: dict[str, np.ndarray] = {}
    step_count = [0]
    state = {"rest_z": None, "grasp_ref": None}

    def _contact_force(cb_model: mujoco.MjModel, cb_data: mujoco.MjData, pad_id: int) -> float:
        total = 0.0
        for ci in range(cb_data.ncon):
            con = cb_data.contact[ci]
            pair = (int(con.geom1), int(con.geom2))
            if cube_geom_id in pair and pad_id in pair:
                f6 = np.zeros(6)
                mujoco.mj_contactForce(cb_model, cb_data, ci, f6)
                total += abs(float(f6[0]))
        return total

    def frame_callback(phase: str, cb_model: mujoco.MjModel, cb_data: mujoco.MjData) -> None:
        step_count[0] += 1
        if state["rest_z"] is None:
            state["rest_z"] = float(cb_data.xpos[cube_body_id][2])

        carrying_phase = phase.startswith("LIFT") or phase in (
            "HOLD",
        ) or phase.startswith("TRANSPORT_ABOVE_TARGET") or phase == "SETTLE_ABOVE_TARGET" \
            or phase.startswith("LOWER_TO_TARGET") or phase == "SETTLE_LOWER"
        if state["grasp_ref"] is None and carrying_phase:
            tcp_pos = cb_data.site_xpos[site_id]
            tcp_rot = cb_data.site_xmat[site_id].reshape(3, 3)
            cube_xyz = cb_data.xpos[cube_body_id]
            state["grasp_ref"] = tcp_local_cube_offset(tcp_pos, tcp_rot, cube_xyz)

        if step_count[0] % FRAME_STRIDE == 0:
            cube_z_now = float(cb_data.xpos[cube_body_id][2])
            height_gain = cube_z_now - state["rest_z"]
            if state["grasp_ref"] is not None:
                tcp_pos = cb_data.site_xpos[site_id]
                tcp_rot = cb_data.site_xmat[site_id].reshape(3, 3)
                cube_xyz = cb_data.xpos[cube_body_id]
                local_now = tcp_local_cube_offset(tcp_pos, tcp_rot, cube_xyz)
                slip = float(np.linalg.norm(local_now - state["grasp_ref"]))
            else:
                slip = 0.0
            left_f = _contact_force(cb_model, cb_data, left_pad_id)
            right_f = _contact_force(cb_model, cb_data, right_pad_id)
            lines = [
                INSTRUCTION_TEXT,
                f"state: {phase}",
                f"cube height gain: {height_gain:+.4f} m",
                f"TCP-frame slip: {slip * 1000:.2f} mm",
                f"contact force L/R: {left_f:.3f} / {right_f:.3f} N",
            ]

            renderer.update_scene(cb_data, camera=cam)
            frames.append(_draw_overlay(renderer.render().copy(), lines))

            closeup_renderer.update_scene(cb_data, camera=closeup_cam)
            closeup_frames.append(_draw_overlay(closeup_renderer.render().copy(), lines))

        if phase == "LIFT_wp0" and "grasp" not in stills:
            renderer.update_scene(cb_data, camera=cam)
            stills["grasp"] = renderer.render().copy()
        if phase.startswith("TRANSPORT_ABOVE_TARGET_wp") and "transport" not in stills:
            wp_idx = int(phase.rsplit("wp", 1)[1])
            if wp_idx >= 20:
                renderer.update_scene(cb_data, camera=cam)
                stills["transport"] = renderer.render().copy()
        if phase == "VERIFY_TASK_SUCCESS":
            renderer.update_scene(cb_data, camera=cam)
            stills["released"] = renderer.render().copy()

    result = run_trial_pick_place(scene, frame_callback=frame_callback)
    print(f"task_pass={result['task_pass']} grasp_stability_pass_4e={result['grasp_stability_pass_4e']}")
    print(f"rendered {len(frames)} frames at stride {FRAME_STRIDE} (~{VIDEO_FPS:.2f} fps)")

    def _write_video(path: Path, all_frames: list[np.ndarray]) -> tuple[int, int]:
        writer = imageio.get_writer(
            str(path), fps=VIDEO_FPS, codec="libx264", quality=None,
            ffmpeg_params=["-crf", "23", "-pix_fmt", "yuv420p"],
        )
        for f in all_frames:
            writer.append_data(f)
        writer.close()
        reader = imageio.get_reader(str(path))
        decoded = 0
        for _ in reader:
            decoded += 1
        reader.close()
        return decoded, path.stat().st_size

    video_path = ARTIFACTS / "phase4e_task1_corrected.mp4"
    decoded_n, size_bytes = _write_video(video_path, frames)
    if decoded_n != len(frames):
        raise RuntimeError(f"decoded frame count {decoded_n} != encoded frame count {len(frames)}")
    duration_s = decoded_n / VIDEO_FPS
    print(
        f"wrote {video_path}: {WIDTH}x{HEIGHT}, fps={VIDEO_FPS:.2f}, "
        f"frames={decoded_n}, duration={duration_s:.2f}s, size={size_bytes} bytes, decode-verified"
    )

    closeup_path = ARTIFACTS / "phase4e_task1_closeup.mp4"
    decoded_c, size_c = _write_video(closeup_path, closeup_frames)
    if decoded_c != len(closeup_frames):
        raise RuntimeError(f"decoded close-up frame count {decoded_c} != encoded frame count {len(closeup_frames)}")
    duration_c = decoded_c / VIDEO_FPS
    print(
        f"wrote {closeup_path}: {CLOSEUP_WIDTH}x{CLOSEUP_HEIGHT}, fps={VIDEO_FPS:.2f}, "
        f"frames={decoded_c}, duration={duration_c:.2f}s, size={size_c} bytes, decode-verified"
    )

    for name, img in stills.items():
        out = ARTIFACTS / f"phase4e_still_{name}.png"
        imageio.imwrite(str(out), img)
        print(f"wrote {out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
