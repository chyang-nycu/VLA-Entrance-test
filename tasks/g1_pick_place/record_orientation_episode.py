#!/usr/bin/env python3
"""Phase 4F, Section E: renders the final (Attempt 3) orientation-
stabilization configuration's nominal Task 1 episode to two synchronized
videos:

- artifacts/phase4f_task1_full.mp4: normal full-task third-person view.
- artifacts/phase4f_bilateral_contact_view.mp4: a diagnostic view
  perpendicular to the gripper's jaw-closing axis (world Y, per the
  measured wrist orientation in reports/phase4f-orientation-grasp-
  stabilization.md), so the left finger, cube, and right finger are all
  visible side-by-side rather than one occluding another.

Both carry a burned-in overlay: task state, live position residual
(distance from the TCP to whatever this phase's segment is currently
targeting), live required-axis orientation residual (wrist local Z vs.
world vertical), total TCP-frame grasp slip, vertical/downward slip since
grasp, and left/right normal contact force.

Uses write_grasp_scene_4f() and run_trial_pick_place(..., use_oriented_ik=
True) -- Phase 4F's own opt-in configuration; does not affect or overwrite
write_grasp_scene_4b()'s scene or any earlier phase's artifacts
(phase4b_task1_nominal.mp4, phase4d_failure_reproduction.mp4,
phase4e_task1_corrected.mp4, phase4e_task1_closeup.mp4 all untouched).
"""

from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from tasks.g1_pick_place.controller_3c import orientation_residual_rad
from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS, write_grasp_scene_4f
from tasks.g1_pick_place.run_grasp_test_3c import LIFT_DZ, PREGRASP_DZ
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B,
    ARM_KV_4B,
    LOWER_TO_TARGET_POS,
    RETREAT_POS,
    TRANSPORT_ABOVE_TARGET_POS,
    run_trial_pick_place,
    tcp_local_cube_offset,
)

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts"
TIMESTEP = 0.002
FRAME_STRIDE = 17
VIDEO_FPS = 1.0 / (TIMESTEP * FRAME_STRIDE)
WIDTH, HEIGHT = 640, 480
DIAG_WIDTH, DIAG_HEIGHT = 480, 360

INSTRUCTION_TEXT = "Task 1: pick up the red cube and place it in the blue target area (Phase 4F, attempt 3)"

CUBE_POS_ARR = np.array(CUBE_POS)
PREGRASP_TARGET = CUBE_POS_ARR + np.array([0.0, 0.0, PREGRASP_DZ])
LIFT_TARGET = CUBE_POS_ARR + np.array([0.0, 0.0, LIFT_DZ])

_PHASE_TARGETS = [
    ("PREGRASP", PREGRASP_TARGET),
    ("APPROACH", CUBE_POS_ARR),
    ("CLOSE", CUBE_POS_ARR),
    ("LIFT", LIFT_TARGET),
    ("HOLD", LIFT_TARGET),
    ("TRANSPORT_ABOVE_TARGET", TRANSPORT_ABOVE_TARGET_POS),
    ("SETTLE_ABOVE_TARGET", TRANSPORT_ABOVE_TARGET_POS),
    ("LOWER_TO_TARGET", LOWER_TO_TARGET_POS),
    ("SETTLE_LOWER", LOWER_TO_TARGET_POS),
    ("RETREAT", RETREAT_POS),
]


def _target_for_phase(phase: str) -> np.ndarray:
    for prefix, target in _PHASE_TARGETS:
        if phase.startswith(prefix):
            return target
    return CUBE_POS_ARR


def _make_full_camera() -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    mid_x = (CUBE_POS[0] + TARGET_POS[0]) / 2.0
    mid_y = (CUBE_POS[1] + TARGET_POS[1]) / 2.0
    cam.lookat[:] = [mid_x, mid_y, 0.85]
    cam.distance = 1.7
    cam.azimuth = 200.0
    cam.elevation = -22.0
    return cam


def _make_diagnostic_camera() -> mujoco.MjvCamera:
    """Perpendicular to the jaw-closing axis (measured: wrist local Y is
    within ~5 deg of world Y at the grasp waypoint -- see
    reports/phase4f-orientation-grasp-stabilization.md, Section A). A view
    looking along world Y (camera azimuth rotated 90 deg from the main
    camera, roughly along +/-X) sees the two fingers side by side with the
    cube between them, rather than one behind the other."""
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [CUBE_POS[0], CUBE_POS[1], CUBE_POS[2] + 0.02]
    cam.distance = 0.45
    cam.azimuth = 155.0
    cam.elevation = -12.0
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
    scene = write_grasp_scene_4f(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml")
    model = mujoco.MjModel.from_xml_path(str(scene))
    full_renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    diag_renderer = mujoco.Renderer(model, height=DIAG_HEIGHT, width=DIAG_WIDTH)
    full_cam = _make_full_camera()
    diag_cam = _make_diagnostic_camera()

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_tcp")

    full_frames: list[np.ndarray] = []
    diag_frames: list[np.ndarray] = []
    step_count = [0]
    state = {"rest_z": None, "grasp_ref": None, "grasp_z0": None}

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

        carrying_phase = (
            phase.startswith("LIFT") or phase == "HOLD"
            or phase.startswith("TRANSPORT_ABOVE_TARGET") or phase == "SETTLE_ABOVE_TARGET"
            or phase.startswith("LOWER_TO_TARGET") or phase == "SETTLE_LOWER"
        )
        tcp_pos_now = cb_data.site_xpos[site_id].copy()
        tcp_rot_now = cb_data.site_xmat[site_id].reshape(3, 3).copy()
        if state["grasp_ref"] is None and carrying_phase:
            cube_xyz0 = cb_data.xpos[cube_body_id]
            state["grasp_ref"] = tcp_local_cube_offset(tcp_pos_now, tcp_rot_now, cube_xyz0)
            state["grasp_z0"] = float(cube_xyz0[2])

        if step_count[0] % FRAME_STRIDE == 0:
            cube_xyz = cb_data.xpos[cube_body_id].copy()
            cube_z_now = float(cube_xyz[2])
            height_gain = cube_z_now - state["rest_z"]
            pos_resid = float(np.linalg.norm(tcp_pos_now - _target_for_phase(phase)))
            orient_resid_deg = float(np.degrees(orientation_residual_rad(tcp_rot_now.flatten())))
            if state["grasp_ref"] is not None:
                local_now = tcp_local_cube_offset(tcp_pos_now, tcp_rot_now, cube_xyz)
                slip = float(np.linalg.norm(local_now - state["grasp_ref"]))
                vertical_slip = max(0.0, state["grasp_z0"] - cube_z_now)
            else:
                slip = 0.0
                vertical_slip = 0.0
            left_f = _contact_force(cb_model, cb_data, left_pad_id)
            right_f = _contact_force(cb_model, cb_data, right_pad_id)
            lines = [
                INSTRUCTION_TEXT,
                f"state: {phase}",
                f"cube height gain: {height_gain:+.4f} m",
                f"position residual: {pos_resid * 1000:.2f} mm",
                f"orientation residual (required axis): {orient_resid_deg:.1f} deg",
                f"total grasp slip: {slip * 1000:.2f} mm",
                f"vertical/downward slip: {vertical_slip * 1000:.2f} mm",
                f"contact force L/R: {left_f:.3f} / {right_f:.3f} N",
            ]

            full_renderer.update_scene(cb_data, camera=full_cam)
            full_frames.append(_draw_overlay(full_renderer.render().copy(), lines))

            # Diagnostic camera tracks the cube's CURRENT position (not a
            # static lookat at its rest pose) so the gripper/cube stay
            # framed throughout lift/transport/lower, not just at APPROACH.
            diag_cam.lookat[:] = [cube_xyz[0], cube_xyz[1], cube_xyz[2] + 0.01]
            diag_renderer.update_scene(cb_data, camera=diag_cam)
            diag_frames.append(_draw_overlay(diag_renderer.render().copy(), lines))

    result = run_trial_pick_place(scene, frame_callback=frame_callback, use_oriented_ik=True)
    print(f"task_pass={result['task_pass']} grasp_stability_pass_4f={result['grasp_stability_pass_4f']} "
          f"failure_state={result['failure_state']}")
    print(f"rendered {len(full_frames)} frames at stride {FRAME_STRIDE} (~{VIDEO_FPS:.2f} fps)")

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

    full_path = ARTIFACTS / "phase4f_task1_full.mp4"
    decoded_n, size_bytes = _write_video(full_path, full_frames)
    if decoded_n != len(full_frames):
        raise RuntimeError(f"decoded frame count {decoded_n} != encoded frame count {len(full_frames)}")
    duration_s = decoded_n / VIDEO_FPS
    print(
        f"wrote {full_path}: {WIDTH}x{HEIGHT}, fps={VIDEO_FPS:.2f}, "
        f"frames={decoded_n}, duration={duration_s:.2f}s, size={size_bytes} bytes, decode-verified"
    )

    diag_path = ARTIFACTS / "phase4f_bilateral_contact_view.mp4"
    decoded_d, size_d = _write_video(diag_path, diag_frames)
    if decoded_d != len(diag_frames):
        raise RuntimeError(f"decoded diagnostic frame count {decoded_d} != encoded frame count {len(diag_frames)}")
    duration_d = decoded_d / VIDEO_FPS
    print(
        f"wrote {diag_path}: {DIAG_WIDTH}x{DIAG_HEIGHT}, fps={VIDEO_FPS:.2f}, "
        f"frames={decoded_d}, duration={duration_d:.2f}s, size={size_d} bytes, decode-verified"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
