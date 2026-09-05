"""Phase 8, Step 5: Task 3 door-pull video, third-person and close-up.

Mirrors task2_language_selection.record_instruction_video's exact pattern
(purely-observational frame_callback, no effect on control or outcome).

The burned-in label is derived from the trial's own actual door_pass
result rather than hardcoded, so the video never claims a status the
rollout itself didn't earn (reports/phase8-slip-diagnosis.md). As of the
Phase 8 gain fix (arm_kp=2200, gripper_kp=1200) the shipped configuration
passes all criteria (max slip 8.22mm <= the strict 10mm target), so this
now records as a genuine successful expert demonstration; the earlier
failing-configuration recording is preserved separately as
phase8_task3_*_prefail.mp4 for the record.
"""

from __future__ import annotations

import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from tasks.g1_pick_place.door_open import TASK_DIR, run_trial_door_open
from tasks.g1_pick_place.workspace_map import handle_pose

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts"

WIDTH, HEIGHT = 640, 480
TIMESTEP = 0.002
FRAME_STRIDE = 17
FPS = 1.0 / (TIMESTEP * FRAME_STRIDE)


def _cameras(geometry: dict) -> tuple[mujoco.MjvCamera, mujoco.MjvCamera]:
    pivot_xy = tuple(geometry["pivot_xy"])
    radius = geometry["radius_m"]
    phi0, theta, z = geometry["phi0_deg"], geometry["theta_deg"], geometry["handle_z"]
    closed = handle_pose(pivot_xy, radius, phi0, z)
    opened = handle_pose(pivot_xy, radius, phi0 + theta, z)
    mid = (closed + opened) / 2.0

    # Azimuth/elevation chosen by direct visual inspection (rendered and
    # eyeballed a sweep of candidates, not guessed): az=160/340/200 all
    # look through the robot's own torso and hide the door entirely; a
    # side-profile at az=70 is the one that actually shows the arm, the
    # handle, and the frame post simultaneously.
    third = mujoco.MjvCamera()
    third.lookat[:] = mid
    third.distance = 1.2
    third.azimuth = 70.0
    third.elevation = -15.0

    closeup = mujoco.MjvCamera()
    closeup.lookat[:] = mid
    closeup.distance = 0.5
    closeup.azimuth = 70.0
    closeup.elevation = -10.0
    return third, closeup


def record(scene_path: Path, geometry: dict, out_dir: Path) -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    third_cam, close_cam = _cameras(geometry)
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    # Determine the honest label from a real (unrendered) run of this exact
    # scene/geometry before recording, rather than asserting a status ahead
    # of time. The trial is deterministic (tests/test_door_open.py::
    # test_result_is_deterministic_across_reruns), so this pre-check result
    # applies unchanged to the rendered run below.
    precheck = run_trial_door_open(scene_path, geometry)
    label = (
        "SUCCESSFUL EXPERT DEMONSTRATION (all strict criteria met)"
        if precheck["door_pass"]
        else "PROTOTYPE / FAILURE-ANALYSIS DEMO (strict <=10mm slip target not met)"
    )

    third_frames: list[np.ndarray] = []
    close_frames: list[np.ndarray] = []
    step_count = [0]

    def overlay(frame: np.ndarray, phase: str, hinge_deg: float) -> np.ndarray:
        img = Image.fromarray(frame)
        draw = ImageDraw.Draw(img)
        draw.text((8, 8), "Task 3: open the cabinet door", fill=(255, 255, 0))
        draw.text((8, 24), label, fill=(255, 80, 80))
        draw.text((8, 40), f"phase: {phase}   hinge: {hinge_deg:.1f} deg", fill=(255, 255, 0))
        return np.array(img)

    def cb(phase: str, m: mujoco.MjModel, d: mujoco.MjData) -> None:
        step_count[0] += 1
        if step_count[0] % FRAME_STRIDE != 0:
            return
        hid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
        hinge_deg = float(np.degrees(d.qpos[m.jnt_qposadr[hid]]))
        renderer.update_scene(d, camera=third_cam)
        third_frames.append(overlay(renderer.render().copy(), phase, hinge_deg))
        renderer.update_scene(d, camera=close_cam)
        close_frames.append(overlay(renderer.render().copy(), phase, hinge_deg))

    result = run_trial_door_open(scene_path, geometry, frame_callback=cb)
    renderer.close()
    assert result["door_pass"] == precheck["door_pass"], "non-deterministic trial -- label may be wrong"

    out_dir.mkdir(parents=True, exist_ok=True)
    third_path = out_dir / "phase8_task3_third_person.mp4"
    close_path = out_dir / "phase8_task3_closeup.mp4"
    for frames, path in ((third_frames, third_path), (close_frames, close_path)):
        writer = imageio.get_writer(str(path), fps=FPS, codec="libx264", quality=8)
        for f in frames:
            writer.append_data(f)
        writer.close()

    return {
        "door_pass": result["door_pass"],
        "max_hinge_deg": float(np.degrees(result["telemetry"]["max_hinge_qpos"])),
        "max_handle_slip_m": result["telemetry"]["max_handle_slip_m"],
        "label": label,
        "n_frames": len(third_frames),
        "third_person_path": str(third_path),
        "closeup_path": str(close_path),
    }


def main() -> int:
    geometry = json.loads((ROOT / "logs" / "phase7b_selected_door_geometry.json").read_text())
    scene_path = TASK_DIR / "g1_grasp_scene_door.xml"
    meta = record(scene_path, geometry, ARTIFACTS)
    print(json.dumps(meta, indent=2))
    (ARTIFACTS / "phase8_task3_video_manifest.json").write_text(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
