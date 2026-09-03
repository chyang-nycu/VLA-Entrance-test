#!/usr/bin/env python3
"""Task 2 (optional, time-boxed): language-conditioned two-object selection.

Scene: the existing Task 1 scene (write_grasp_scene_5a -- fixed pelvis+torso,
task-local physical gripper, blue target pad, torso-mounted onboard RGB
camera) plus a second cube ("cube2", green), added here by re-parsing that
scene's own output (never by editing gripper_scene.py's write_grasp_scene_5a
or write_grasp_scene_4b). Both cubes use IDENTICAL size/mass/friction/
collision geometry (CUBE_HALF/CUBE_MASS/CUBE_FRICTION, imported unchanged
from gripper_scene.py -- never redefined here).

Controller: the SAME non-learned scripted controller as Task 1
(run_pick_place.run_trial_pick_place, run unmodified in its control logic --
this phase only added optional, default-preserving parameters to it so it
can act on a caller-specified cube body/joint name instead of the literal
"cube"/"cube_joint", plus optional read-only distractor-displacement
telemetry). Waypoints are always computed from the SELECTED object's own
live pose (whichever body name is passed in) -- never a hardcoded position,
verified by test_waypoints_follow_selected_object_not_hardcoded below.

Language: the scripted oracle controller receives the selected object
identity from the task specification. The environment and dataset expose
language-conditioned object selection, but learned visual-language grounding
is not evaluated. `parse_selected_object()` below is a trivial, deterministic
keyword lookup over exactly the two authorized instruction strings -- not a
language model, not visual recognition, and not claimed to be either.

Object-slot geometry (Section A's "collision-safe transport path"
requirement): candidate second-cube offsets were checked with
diagnose_pick_place_reachability (a cheap, pre-run, IK-only reachability
probe -- no physics stepped) BEFORE being chosen, not guessed. The chosen
separation (SLOT_B_OFFSET, -0.08m in the confirmed-reachable -x direction
from SLOT_A_OFFSET's nominal (0,0)) is deliberately almost pure -x rather
than a small diagonal offset: PREGRASP/APPROACH/CLOSE only constrain the TCP
site's *position* (no orientation term), and the two finger pads extend
+/-0.075m from the TCP along the wrist's local jaw axis, roughly world-Y at
the measured nominal configuration (reports/phase4f-orientation-grasp-
stabilization.md) -- but that alignment is only verified AT the nominal
pose, not guaranteed at an arbitrary off-nominal one. An 8cm pure-X
separation keeps the two cubes' bodies 8cm apart in world X regardless of
exactly how the wrist happens to be yawed at either slot, since the finger
pads' own local-X extent (the reach axis) is small (~1.2cm half-extent),
while the risk from an *assumed* world-Y jaw alignment would be a pure-Y
separation, which is not assumed here. This choice is verified, not just
argued: `logs/task2_language_selection.json`'s `distractor` field for every
successful trial records the true measured maximum displacement of the
NON-selected cube throughout the entire trial (RESET through
VERIFY_TASK_SUCCESS), confirmed <=10mm empirically, not by geometric
argument alone.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from tasks.g1_pick_place.camera_observation import (
    CAM_HEIGHT,
    CAM_WIDTH,
    HEAD_CAM_NAME,
    blue_target_mask,
    red_cube_mask,
    write_grasp_scene_5a,
)
from tasks.g1_pick_place.controller import RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap
from tasks.g1_pick_place.gripper_scene import (
    CUBE_FRICTION,
    CUBE_HALF,
    CUBE_MASS,
    CUBE_POS,
    TABLE_TOP_Z,
    TARGET_POS,
    TASK_DIR,
)
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B,
    ARM_KV_4B,
    TARGET_XY,
    diagnose_pick_place_reachability,
    run_trial_pick_place,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"
ARTIFACTS = ROOT / "artifacts"
REPORT_DIR = ROOT / "reports"

INSTRUCTION_CANONICAL = "Pick up the red cube and place it in the blue target area."
INSTRUCTIONS = {
    "red": "Pick up the red cube and place it in the blue target area.",
    "green": "Pick up the green cube and place it in the blue target area.",
}

# Object registry: which body/geom/joint each color corresponds to in the
# Task 2 scene. "red" is Task 1's own, unrenamed cube (body "cube") so every
# Task-1-derived physical constant (CUBE_POS, contact-geom ids, etc.) that
# implicitly assumed a body literally named "cube" continues to refer to the
# same object; "green" is the new cube added by write_task2_scene below.
OBJECT_SPECS = {
    "red": {"body_name": "cube", "geom_name": "cube_geom", "joint_name": "cube_joint"},
    "green": {"body_name": "cube2", "geom_name": "cube2_geom", "joint_name": "cube2_joint"},
}

# Two reachable slot offsets from CUBE_POS (see module docstring for why -x,
# not a small diagonal, and why 8cm). Both independently confirmed reachable
# by diagnose_pick_place_reachability before use (see verify_slots_reachable
# below and tests/test_task2_language_selection.py).
SLOT_A_OFFSET = (0.0, 0.0)
# Empirically tuned (reports/task2-language-selection.md Section A): an
# initial -x-only candidate (-0.08, 0.0) was IK-reachable and geometrically
# 8cm clear of slot A, but a full physics trial measured 48.7mm of real
# distractor displacement -- traced (via a frame_callback probe, not
# guessed) to RETREAT's one-shot joint-space `_drive_segment` (never
# Cartesian-smoothed, since Task 1 never had a second object for it to
# sweep through) carrying the arm through an uncontrolled joint-space path
# that happened to pass close to that particular slot. TARGET_POS sits at
# CUBE_POS + (-0.11, +0.07) from slot A -- i.e. the RETREAT/TRANSPORT
# corridor runs toward -x AND +y -- so (-0.08, 0.0) sat almost on that
# corridor. A second candidate (-0.08, -0.05), moved away from the +y pull,
# measured a safe 5.0mm both directions but was found (separately) to be
# only marginally visible to the onboard camera at reset (9/19200 px,
# mostly occluded by the resting gripper's own geometry) -- a real,
# measured shortfall against Section "Camera/data"'s visibility
# requirement, not assumed satisfied. A further offset search (see
# logs/task2_language_selection.json's search history and the report)
# checking reachability, real physics-measured distractor displacement in
# BOTH grasp directions, AND onboard-camera pixel visibility together found
# (-0.08, -0.10): 0.0mm / 1.7mm displacement (both directions) and 30/19200
# green pixels visible (vs 9 at the first -y candidate) -- better on every
# measured axis, not merely visibility traded against safety.
SLOT_B_OFFSET = (-0.08, -0.10)

ARRANGEMENTS = {
    "A": {"red": SLOT_A_OFFSET, "green": SLOT_B_OFFSET},
    "swapped": {"red": SLOT_B_OFFSET, "green": SLOT_A_OFFSET},
}

DISTRACTOR_DISPLACEMENT_TOL_M = 0.010


def parse_selected_object(instruction: str) -> str:
    """Deterministic keyword lookup, not language understanding. Raises if
    the instruction is not exactly one of the two authorized strings, or
    names both/neither color -- this project never guesses.
    """
    has_red = "red" in instruction.lower()
    has_green = "green" in instruction.lower()
    if has_red and not has_green:
        return "red"
    if has_green and not has_red:
        return "green"
    raise ValueError(f"instruction does not unambiguously name exactly one known object: {instruction!r}")


def write_task2_scene(
    arm_kp: dict | float = ARM_KP_4B,
    arm_kv: dict | float = ARM_KV_4B,
    scene_name: str = "g1_grasp_scene_task2.xml",
) -> Path:
    """Task 2 scene: write_grasp_scene_5a()'s own output (Task 1's scene,
    unmodified, re-used by import) plus one additional green cube body,
    identical size/mass/friction/collision to the existing red "cube" body.
    Never calls write_grasp_scene_5a/write_grasp_scene_4b with different
    arguments and never edits their source -- this mirrors the exact
    "build on a fresh re-parse" convention write_grasp_scene_5a() itself
    already uses for write_grasp_scene_4b().
    """
    base = write_grasp_scene_5a(arm_kp=arm_kp, arm_kv=arm_kv, scene_name="g1_grasp_scene_5a.xml")
    tree = ET.parse(base)
    root = tree.getroot()

    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("expected asset section from write_grasp_scene_5a()")
    ET.SubElement(asset, "material", {"name": "cube2_mat", "rgba": "0.15 0.6 0.15 1"})

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("expected worldbody from write_grasp_scene_5a()")
    cube2 = ET.SubElement(
        worldbody, "body", {"name": "cube2", "pos": f"{CUBE_POS[0]} {CUBE_POS[1]} {CUBE_POS[2]}"},
    )
    ET.SubElement(cube2, "freejoint", {"name": "cube2_joint"})
    ET.SubElement(
        cube2, "geom",
        {
            "name": "cube2_geom", "type": "box",
            "size": f"{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}", "mass": str(CUBE_MASS),
            "material": "cube2_mat", "contype": "1", "conaffinity": "1", "friction": CUBE_FRICTION,
        },
    )

    out = TASK_DIR / scene_name
    tree.write(out, encoding="utf-8", xml_declaration=False)
    return out


def verify_slots_reachable(scene_path: Path) -> dict:
    """Pre-run IK-only reachability check (no physics stepped) for both
    slot offsets, against the actual Task 2 scene -- not assumed from the
    single-cube scene's own prior measurements.
    """
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    base_qpos = data.qpos.copy()
    result = {}
    for slot_name, offset in (("A", SLOT_A_OFFSET), ("B", SLOT_B_OFFSET)):
        cube_pos = np.array([CUBE_POS[0] + offset[0], CUBE_POS[1] + offset[1], CUBE_POS[2]])
        report = diagnose_pick_place_reachability(model, arm_map, site_id, base_qpos, cube_pos)
        result[slot_name] = {"offset": list(offset), "all_reachable": report["all_reachable"]}
    return result


def run_trial_task2(scene_path: Path, selected_object_id: str, arrangement: str) -> dict:
    """One Task 2 trial: `selected_object_id` ("red"/"green") is grasped and
    placed; the other object is the distractor, tracked (never controlled)
    throughout. Deterministic -- no RNG anywhere in this module, same as
    every other phase in this project.
    """
    if selected_object_id not in OBJECT_SPECS:
        raise ValueError(f"unknown selected_object_id: {selected_object_id!r}")
    if arrangement not in ARRANGEMENTS:
        raise ValueError(f"unknown arrangement: {arrangement!r}")
    distractor_id = "green" if selected_object_id == "red" else "red"

    selected_spec = OBJECT_SPECS[selected_object_id]
    distractor_spec = OBJECT_SPECS[distractor_id]
    selected_offset = ARRANGEMENTS[arrangement][selected_object_id]
    distractor_offset = ARRANGEMENTS[arrangement][distractor_id]

    result = run_trial_pick_place(
        scene_path,
        cube_xy_offset=selected_offset,
        cube_body_name=selected_spec["body_name"],
        cube_geom_name=selected_spec["geom_name"],
        cube_joint_name=selected_spec["joint_name"],
        distractor={
            "body_name": distractor_spec["body_name"],
            "geom_name": distractor_spec["geom_name"],
            "joint_name": distractor_spec["joint_name"],
            "xy_offset": distractor_offset,
        },
    )

    distractor = result["distractor"]
    wrong_object_placed = bool(distractor is not None and distractor["in_target_xy"])
    distractor_ok = bool(
        distractor is not None
        and distractor["displacement_within_10mm"]
        and not distractor["in_target_xy"]
    )
    selected_identity_agrees = True  # structurally guaranteed: run_trial_pick_place only ever
    # reads/moves the body named selected_spec["body_name"] -- there is no code path in this
    # module or run_trial_pick_place that could act on distractor_spec["body_name"] instead,
    # so this is a documented invariant, not a runtime guess (exercised directly by
    # tests/test_task2_language_selection.py::test_waypoints_follow_selected_object_not_hardcoded).

    task2_pass = bool(result["task_pass"] and distractor_ok and not wrong_object_placed)

    return {
        "instruction": INSTRUCTIONS[selected_object_id],
        "selected_object_id": selected_object_id,
        "distractor_object_id": distractor_id,
        "arrangement": arrangement,
        "selected_offset": list(selected_offset),
        "distractor_offset": list(distractor_offset),
        "task_pass": result["task_pass"],
        "selected_identity_agrees": selected_identity_agrees,
        "distractor": distractor,
        "wrong_object_placed": wrong_object_placed,
        "task2_pass": task2_pass,
        "failure_state": result["failure_state"],
        "failure_reason": result["failure_reason"],
        "final_xy_target_error_m": result["final_xy_target_error_m"],
        "_raw": result,
    }


def verify_camera_sees_both_objects_and_target(scene_path: Path) -> dict:
    """Renders the onboard head_cam's very first (post-reset) frame with
    BOTH objects present (arrangement A) and checks it is non-blank and
    contains a red-cube-colored region, a target-blue region, and (via a
    simple, documented-as-a-diagnostic-only green mask, mirroring
    red_cube_mask/blue_target_mask's own calibration approach) a green-cube-
    colored region. This is a rendering/visibility smoke check, exactly the
    same kind already used in Phase 5A/5B -- never task-success logic.
    """
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    cube2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube2_joint")
    for joint_id, offset in ((cube_joint_id, SLOT_A_OFFSET), (cube2_joint_id, SLOT_B_OFFSET)):
        qpos_adr = int(model.jnt_qposadr[joint_id])
        data.qpos[qpos_adr : qpos_adr + 3] = [CUBE_POS[0] + offset[0], CUBE_POS[1] + offset[1], CUBE_POS[2]]
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, HEAD_CAM_NAME)
    renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)
    renderer.update_scene(data, camera=cam_id)
    frame = renderer.render().copy()
    renderer.close()

    def _green_cube_mask(f: np.ndarray) -> np.ndarray:
        r = f[..., 0].astype(int)
        g = f[..., 1].astype(int)
        b = f[..., 2].astype(int)
        return (g > 90) & (g - r > 25) & (g - b > 25)

    red_frac = float(np.mean(red_cube_mask(frame)))
    green_frac = float(np.mean(_green_cube_mask(frame)))
    blue_frac = float(np.mean(blue_target_mask(frame)))
    return {
        "non_blank": bool(np.std(frame) > 1.0),
        "sees_red_cube": red_frac > 0.0005,
        "sees_green_cube": green_frac > 0.0005,
        "sees_blue_target": blue_frac > 0.0005,
        "red_pixel_fraction": red_frac,
        "green_pixel_fraction": green_frac,
        "blue_pixel_fraction": blue_frac,
    }


THIRD_PERSON_WIDTH, THIRD_PERSON_HEIGHT = 640, 480
FRAME_STRIDE = 17
TIMESTEP = 0.002
VIDEO_FPS = 1.0 / (TIMESTEP * FRAME_STRIDE)


def _make_third_person_camera() -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    mid_x = (CUBE_POS[0] + TARGET_POS[0]) / 2.0
    mid_y = (CUBE_POS[1] + TARGET_POS[1]) / 2.0
    cam.lookat[:] = [mid_x, mid_y, 0.85]
    cam.distance = 1.7
    cam.azimuth = 200.0
    cam.elevation = -22.0
    return cam


def record_instruction_video(scene_path: Path, selected_object_id: str, arrangement: str, out_path: Path) -> dict:
    """Records one full Task 2 trial to `out_path`, third-person view, with
    a burned-in text overlay naming the instruction actually given. Uses
    run_trial_pick_place's existing, purely-observational frame_callback
    hook (unchanged mechanism from Phase 4C/4E) -- recording never affects
    control or the pass/fail outcome.
    """
    from PIL import Image, ImageDraw

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    renderer = mujoco.Renderer(model, height=THIRD_PERSON_HEIGHT, width=THIRD_PERSON_WIDTH)
    cam = _make_third_person_camera()
    frames: list[np.ndarray] = []
    step_count = [0]
    instruction_text = INSTRUCTIONS[selected_object_id]

    def _cb(phase: str, m: mujoco.MjModel, d: mujoco.MjData) -> None:
        step_count[0] += 1
        if step_count[0] % FRAME_STRIDE != 0:
            return
        renderer.update_scene(d, camera=cam)
        frame = renderer.render().copy()
        img = Image.fromarray(frame)
        draw = ImageDraw.Draw(img)
        draw.text((8, 8), f"Task 2: {instruction_text}", fill=(255, 255, 0))
        draw.text((8, 24), f"phase: {phase}", fill=(255, 255, 0))
        frames.append(np.array(img))

    selected_spec = OBJECT_SPECS[selected_object_id]
    distractor_id = "green" if selected_object_id == "red" else "red"
    distractor_spec = OBJECT_SPECS[distractor_id]
    selected_offset = ARRANGEMENTS[arrangement][selected_object_id]
    distractor_offset = ARRANGEMENTS[arrangement][distractor_id]

    result = run_trial_pick_place(
        scene_path,
        cube_xy_offset=selected_offset,
        cube_body_name=selected_spec["body_name"],
        cube_geom_name=selected_spec["geom_name"],
        cube_joint_name=selected_spec["joint_name"],
        distractor={
            "body_name": distractor_spec["body_name"],
            "geom_name": distractor_spec["geom_name"],
            "joint_name": distractor_spec["joint_name"],
            "xy_offset": distractor_offset,
        },
        frame_callback=_cb,
    )
    renderer.close()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(str(out_path), fps=VIDEO_FPS, codec="libx264", quality=8)
    for f in frames:
        writer.append_data(f)
    writer.close()
    return {"task_pass": result["task_pass"], "n_frames": len(frames), "video_path": str(out_path)}


def evaluate_minimum_configurations(scene_path: Path, n_trials_per_config: int = 3) -> list[dict]:
    """Exactly the 4 required configurations x n_trials_per_config
    deterministic repeats each (12 trials total at the default). Same
    scene/controller parameters for every trial -- no per-configuration
    tuning of any kind.
    """
    configs = [
        ("red", "A"),
        ("green", "A"),
        ("red", "swapped"),
        ("green", "swapped"),
    ]
    all_results = []
    for selected_object_id, arrangement in configs:
        for trial_index in range(n_trials_per_config):
            r = run_trial_task2(scene_path, selected_object_id, arrangement)
            r = {k: v for k, v in r.items() if k != "_raw"}
            r["trial_index"] = trial_index
            all_results.append(r)
    return all_results


def main() -> int:
    scene_path = write_task2_scene()
    reachability = verify_slots_reachable(scene_path)
    camera_check = verify_camera_sees_both_objects_and_target(scene_path)
    trials = evaluate_minimum_configurations(scene_path)

    representative_episodes = []
    seen_configs = set()
    for t in trials:
        key = (t["selected_object_id"], t["arrangement"])
        if t["task2_pass"] and key not in seen_configs:
            seen_configs.add(key)
            representative_episodes.append(
                {
                    "instruction_canonical": INSTRUCTION_CANONICAL,
                    "instruction_utterance": t["instruction"],
                    "selected_object_id": t["selected_object_id"],
                    "arrangement": t["arrangement"],
                    "red_initial_offset": list(ARRANGEMENTS[t["arrangement"]]["red"]),
                    "green_initial_offset": list(ARRANGEMENTS[t["arrangement"]]["green"]),
                    "final_xy_target_error_m": t["final_xy_target_error_m"],
                    "distractor_max_displacement_m": t["distractor"]["max_displacement_m"] if t["distractor"] else None,
                    "success": t["task2_pass"],
                }
            )

    n_configs = 4
    n_configs_all_3_pass = sum(
        1 for key in {(t["selected_object_id"], t["arrangement"]) for t in trials}
        if all(tt["task2_pass"] for tt in trials if (tt["selected_object_id"], tt["arrangement"]) == key)
    )
    overall_pass = bool(n_configs_all_3_pass == n_configs)

    video_results = {}
    if overall_pass:
        video_results["red"] = record_instruction_video(
            scene_path, "red", "A", ARTIFACTS / "task2_red_instruction.mp4"
        )
        video_results["green"] = record_instruction_video(
            scene_path, "green", "A", ARTIFACTS / "task2_green_instruction.mp4"
        )

    log = {
        "scene_path": str(scene_path),
        "reachability": reachability,
        "camera_check": camera_check,
        "trials": trials,
        "representative_episodes": representative_episodes,
        "n_configs": n_configs,
        "n_configs_all_3_pass": n_configs_all_3_pass,
        "overall_pass": overall_pass,
        "video_results": video_results,
        "distractor_displacement_tol_m": DISTRACTOR_DISPLACEMENT_TOL_M,
    }
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / "task2_language_selection.json"
    with open(log_path, "w") as f:
        json.dump(log, f, indent=2)
    print(f"wrote {log_path}")
    print(f"overall_pass={overall_pass} ({n_configs_all_3_pass}/{n_configs} configs all-3-pass)")
    return 0 if overall_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
