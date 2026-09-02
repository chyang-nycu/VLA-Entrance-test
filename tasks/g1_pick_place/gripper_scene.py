#!/usr/bin/env python3
"""Phase 3 task-local G1 grasp scene: fixed-pelvis + parallel gripper + table + cube.

Derives an MJCF from the pinned vendor G1 model by ElementTree deep-copy,
mirroring the pattern used in g1_manipulation_audit.py (write_contact_scene,
write_site_probe_scene). Vendor source files are never modified; only the
in-memory copy is edited before being written to a task-local path.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "vendor" / "unitree_mujoco"
G1_DIR = VENDOR / "unitree_robots" / "g1"
MODEL_XML = G1_DIR / "g1_29dof.xml"
TASK_DIR = ROOT / "tasks" / "g1_pick_place"

WRIST_BODY = "right_wrist_yaw_link"
PELVIS_BODY = "pelvis"

# Cube convention reused from the Phase 2 contact probe (box, mass 0.05 kg,
# friction "1 0.01 0.001"), placed on a task-local table within the reachable
# workspace measured for right_wrist_yaw_link during Phase 3 inspection
# (candidate grasp pose puts the wrist ~0.20-0.30 m forward, ~-0.15 m
# lateral, ~0.80-0.85 m up).
CUBE_HALF = 0.035
CUBE_MASS = 0.05
CUBE_FRICTION = "1 0.01 0.001"
TABLE_TOP_Z = 0.70
CUBE_POS = (0.33, -0.15, TABLE_TOP_Z + CUBE_HALF)

# Finger geometry, in the right_wrist_yaw_link local frame. Fingers slide
# along local Y (perpendicular to the local-X reach/approach direction used
# by the TCP offset), symmetric about y=0.
FINGER_PAD_HALF = (0.012, 0.006, 0.022)
FINGER_REACH_X = 0.10
FINGER_OPEN_Y = 0.075
FINGER_CONTACT_Y = CUBE_HALF + FINGER_PAD_HALF[1]  # pad face just touches cube
# Joints are allowed to travel a little past the nominal contact point so a
# "closed" command keeps pressing (real squeeze force via contact, robust to
# a few mm of arm/IK misalignment) instead of stopping exactly at first
# contact. Tuning iteration 3: iteration 2 reached contact but the cube got
# shoved sideways instead of gripped, because the closed target equaled the
# exact nominal contact point (near-zero net squeeze force under any small
# misalignment). FINGER_SQUEEZE_MARGIN adds real overtravel/force.
FINGER_SQUEEZE_MARGIN = 0.015
FINGER_CLOSED_Y = FINGER_CONTACT_Y - FINGER_SQUEEZE_MARGIN
FINGER_FRICTION = "1.2 0.01 0.001"
FINGER_FORCE_LIMIT = 15.0  # N, conservative for a 0.05 kg cube
TCP_POS = (FINGER_REACH_X, 0.0, 0.0)


def _sub(parent: ET.Element, tag: str, **attrs: str) -> ET.Element:
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def write_grasp_scene() -> Path:
    scene = TASK_DIR / "g1_grasp_scene.xml"
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    root.set("model", "g1_phase3_grasp")

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", str((G1_DIR / "meshes").resolve()))

    # --- environment: floor, lights, table (task-local, not vendor asset) ---
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    _sub(
        asset, "texture", type="2d", name="groundplane", builtin="checker",
        mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3",
        markrgb="0.8 0.8 0.8", width="300", height="300",
    )
    _sub(
        asset, "material", name="groundplane", texture="groundplane",
        texuniform="true", texrepeat="5 5", reflectance="0.2",
    )
    _sub(asset, "material", name="table_mat", rgba="0.45 0.32 0.2 1")
    _sub(asset, "material", name="cube_mat", rgba="0.8 0.2 0.1 1")
    _sub(asset, "material", name="finger_mat", rgba="0.15 0.15 0.15 1")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("G1 model has no worldbody")
    _sub(worldbody, "light", pos="0 0 1.5", dir="0 0 -1", directional="true")
    _sub(worldbody, "light", pos="0.4 -0.2 1.3", dir="-0.2 0.1 -1", directional="true")
    _sub(worldbody, "geom", name="floor", size="0 0 0.05", type="plane", material="groundplane")

    table = _sub(
        worldbody, "body", name="table",
        pos=f"{CUBE_POS[0]} {CUBE_POS[1]} {TABLE_TOP_Z / 2.0}",
    )
    _sub(
        table, "geom", name="table_top", type="box",
        size=f"0.22 0.22 {TABLE_TOP_Z / 2.0}", material="table_mat",
        contype="1", conaffinity="1",
    )

    cube = _sub(worldbody, "body", name="cube", pos=f"{CUBE_POS[0]} {CUBE_POS[1]} {CUBE_POS[2]}")
    _sub(cube, "freejoint", name="cube_joint")
    _sub(
        cube, "geom", name="cube_geom", type="box",
        size=f"{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}", mass=str(CUBE_MASS),
        material="cube_mat", contype="1", conaffinity="1", friction=CUBE_FRICTION,
    )

    # --- fixed pelvis: model-level equality weld, not a runtime hack ---
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    _sub(
        equality, "weld", name="pelvis_fixed", body1=PELVIS_BODY,
        solref="0.002 1", solimp="0.9999 0.9999 0.001 0.5 2",
    )

    # --- physical parallel gripper attached under right_wrist_yaw_link ---
    wrist = None
    for body in root.iter("body"):
        if body.get("name") == WRIST_BODY:
            wrist = body
            break
    if wrist is None:
        raise RuntimeError(f"{WRIST_BODY} not found in vendor model")

    _sub(
        wrist, "site", name="grasp_tcp", pos=f"{TCP_POS[0]} {TCP_POS[1]} {TCP_POS[2]}",
        size="0.01", rgba="0 1 0 1",
    )

    for side, y_ref, jrange in (
        ("left", FINGER_OPEN_Y, f"{-(FINGER_OPEN_Y - FINGER_CLOSED_Y):.4f} 0"),
        ("right", -FINGER_OPEN_Y, f"0 {(FINGER_OPEN_Y - FINGER_CLOSED_Y):.4f}"),
    ):
        finger = _sub(
            wrist, "body", name=f"{side}_finger",
            pos=f"{FINGER_REACH_X} {y_ref} 0",
        )
        _sub(
            finger, "joint", name=f"{side}_finger_joint", type="slide",
            axis="0 1 0", range=jrange, damping="2.0", frictionloss="0.05",
        )
        _sub(
            finger, "geom", name=f"{side}_finger_pad", type="box",
            size=f"{FINGER_PAD_HALF[0]} {FINGER_PAD_HALF[1]} {FINGER_PAD_HALF[2]}",
            material="finger_mat", contype="1", conaffinity="1",
            friction=FINGER_FRICTION, mass="0.03",
        )

    # Exclude wrist-vs-finger and wrist-vs-vendor-hand-mesh contact pairs so
    # the rigid mounting frame does not register spurious self-contact; the
    # cube is the only intended contact partner for the finger pads.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    _sub(contact, "exclude", name="wrist_left_finger_exclude", body1=WRIST_BODY, body2="left_finger")
    _sub(contact, "exclude", name="wrist_right_finger_exclude", body1=WRIST_BODY, body2="right_finger")

    # --- actuators: motors for the two finger slide joints, bounded force ---
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    _sub(
        actuator, "motor", name="left_finger", joint="left_finger_joint",
        ctrllimited="true", ctrlrange=f"{-FINGER_FORCE_LIMIT} {FINGER_FORCE_LIMIT}",
    )
    _sub(
        actuator, "motor", name="right_finger", joint="right_finger_joint",
        ctrllimited="true", ctrlrange=f"{-FINGER_FORCE_LIMIT} {FINGER_FORCE_LIMIT}",
    )

    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


if __name__ == "__main__":
    path = write_grasp_scene()
    print(f"wrote {path}")
