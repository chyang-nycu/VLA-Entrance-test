"""Phase 5B pre-collection canonical-baseline check (run once, evidence
saved to logs/phase5b_baseline_check.json). Not a permanent module -- its
logic is folded into record_demonstrations.py's own pre-flight check;
this script exists only to produce the standalone baseline evidence log
the user's spec asked for before any episode collection begins.
"""
from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.camera_observation import (
    CAM_HEIGHT, CAM_WIDTH, HEAD_CAM_PARENT_BODY, write_grasp_scene_5a,
)
from tasks.g1_pick_place.canonical_config import (
    load_manifest, manifest_hash, verify_environment_matches_manifest,
)
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[2]
results = {}

manifest = load_manifest()
results["manifest_hash"] = manifest_hash()

scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
tree = ET.parse(scene_path)
root = tree.getroot()

# Check 1: corrected visual gripper present (Phase 4E finger pads + palm backing)
finger_pad_names = {g.get("name") for g in root.iter("geom") if g.get("name") in ("left_finger_pad", "right_finger_pad")}
results["check1_corrected_gripper_present"] = finger_pad_names == {"left_finger_pad", "right_finger_pad"}

# Check 2: vendor decorative hand absent
decorative_present = any(g.get("mesh") == "right_rubber_hand" for g in root.iter("geom"))
results["check2_vendor_decorative_hand_absent"] = not decorative_present

# Check 3: physical Task 1 completes (real state machine, nominal variant)
env_result = run_trial_pick_place(scene_path, cube_xy_offset=(0.0, 0.0))
results["check3_task1_completes"] = bool(env_result["task_pass"])
results["check3_failure_state"] = env_result["failure_state"]

# Check 4: onboard RGB renders non-blank frames
model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=CAM_HEIGHT, width=CAM_WIDTH)
renderer.update_scene(data, camera="head_cam")
frame = renderer.render()
results["check4_rgb_shape"] = list(frame.shape)
results["check4_rgb_dtype"] = str(frame.dtype)
results["check4_rgb_nonblank"] = bool(frame.std() > 1.0)
renderer.close()

# Check 5: live-computed config hash matches manifest (parent body verified via ET)
cam_parent = None
for body in root.iter("body"):
    for cam in body.findall("camera"):
        if cam.get("name") == "head_cam":
            cam_parent = body.get("name")
try:
    verify_environment_matches_manifest(
        scene_generator_name="write_grasp_scene_5a",
        use_oriented_ik=False,
        arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B,
        gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
        camera_parent_body=cam_parent,
        camera_resolution_wh=(CAM_WIDTH, CAM_HEIGHT),
        manifest=manifest,
    )
    results["check5_config_hash_matches_manifest"] = True
    results["check5_error"] = None
except Exception as e:
    results["check5_config_hash_matches_manifest"] = False
    results["check5_error"] = str(e)

results["camera_parent_body_live"] = cam_parent
results["all_checks_pass"] = all([
    results["check1_corrected_gripper_present"],
    results["check2_vendor_decorative_hand_absent"],
    results["check3_task1_completes"],
    results["check4_rgb_nonblank"],
    results["check5_config_hash_matches_manifest"],
])

out_path = ROOT / "logs" / "phase5b_baseline_check.json"
with open(out_path, "w") as f:
    json.dump(results, f, indent=2)
print(json.dumps(results, indent=2))
