#!/usr/bin/env python3
"""Phase 2 G1 manipulation audit helpers.

This script reads the official vendor model but writes all generated files under
the project workspace.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "vendor" / "unitree_mujoco"
G1_DIR = VENDOR / "unitree_robots" / "g1"
SCENE = G1_DIR / "scene.xml"
MODEL_XML = G1_DIR / "g1_29dof.xml"
TASK_DIR = ROOT / "tasks" / "g1_pick_place"
LOG_DIR = ROOT / "logs"
REPORT_DIR = ROOT / "reports"


def name(model: mujoco.MjModel, obj_type: mujoco.mjtObj, obj_id: int) -> str:
    return mujoco.mj_id2name(model, obj_type, obj_id) or f"<unnamed:{obj_id}>"


def floats(values: np.ndarray) -> list[float]:
    return [float(v) for v in values]


def body_geom_names(model: mujoco.MjModel, body_id: int) -> list[str]:
    start = int(model.body_geomadr[body_id])
    count = int(model.body_geomnum[body_id])
    return [name(model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(start, start + count)]


def body_joint_names(model: mujoco.MjModel, body_id: int) -> list[str]:
    start = int(model.body_jntadr[body_id])
    count = int(model.body_jntnum[body_id])
    return [name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(start, start + count)]


def joint_range(model: mujoco.MjModel, joint_id: int) -> list[float] | None:
    if not bool(model.jnt_limited[joint_id]):
        return None
    return floats(model.jnt_range[joint_id])


def actuator_inventory(model: mujoco.MjModel) -> list[dict[str, object]]:
    rows = []
    for actuator_id in range(model.nu):
        actuator_name = name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        trn_id = int(model.actuator_trnid[actuator_id][0])
        joint_name = name(model, mujoco.mjtObj.mjOBJ_JOINT, trn_id)
        body_id = int(model.jnt_bodyid[trn_id])
        rows.append(
            {
                "index": actuator_id,
                "actuator_name": actuator_name,
                "controlled_joint": joint_name,
                "control_range": floats(model.actuator_ctrlrange[actuator_id])
                if bool(model.actuator_ctrllimited[actuator_id])
                else None,
                "joint_range": joint_range(model, trn_id),
                "associated_body": name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
            }
        )
    return rows


def relevant_bodies(model: mujoco.MjModel) -> dict[str, list[dict[str, object]]]:
    patterns = {
        "wrist_end_effector_bodies": re.compile(r"(wrist|hand|palm)", re.I),
        "hand_finger_bodies": re.compile(r"(hand|thumb|index|middle|finger|palm)", re.I),
    }
    found: dict[str, list[dict[str, object]]] = {key: [] for key in patterns}
    for body_id in range(model.nbody):
        body_name = name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        for key, pattern in patterns.items():
            if pattern.search(body_name):
                found[key].append(
                    {
                        "body": body_name,
                        "joints": body_joint_names(model, body_id),
                        "geoms": body_geom_names(model, body_id),
                    }
                )
    return found


def site_inventory(model: mujoco.MjModel) -> list[dict[str, object]]:
    return [
        {
            "index": site_id,
            "site": name(model, mujoco.mjtObj.mjOBJ_SITE, site_id),
            "body": name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.site_bodyid[site_id])),
            "pos": floats(model.site_pos[site_id]),
        }
        for site_id in range(model.nsite)
    ]


def geom_inventory(model: mujoco.MjModel, body_pattern: str) -> list[dict[str, object]]:
    pattern = re.compile(body_pattern, re.I)
    rows = []
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        body_name = name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not pattern.search(body_name):
            continue
        rows.append(
            {
                "geom": name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id),
                "body": body_name,
                "type": int(model.geom_type[geom_id]),
                "contype": int(model.geom_contype[geom_id]),
                "conaffinity": int(model.geom_conaffinity[geom_id]),
                "size": floats(model.geom_size[geom_id]),
            }
        )
    return rows


def variant_inventory() -> list[dict[str, object]]:
    variants = []
    for path in sorted(G1_DIR.glob("*.xml")):
        text = path.read_text(encoding="utf-8")
        row: dict[str, object] = {
            "file": str(path.relative_to(ROOT)),
            "mentions_hand": bool(re.search(r"hand|thumb|index|middle|palm|finger", text, re.I)),
            "mentions_gripper": bool(re.search(r"gripper", text, re.I)),
            "mentions_inspire": bool(re.search(r"inspire", text, re.I)),
            "mentions_dex3": bool(re.search(r"dex3|dex", text, re.I)),
            "motor_count_text": len(re.findall(r"<motor\b", text)),
            "joint_count_text": len(re.findall(r"<joint\b", text)),
        }
        try:
            compiled = mujoco.MjModel.from_xml_path(str(path))
            row.update(
                {
                    "compiled": True,
                    "compiled_nu": int(compiled.nu),
                    "compiled_nbody": int(compiled.nbody),
                    "compiled_ngeom": int(compiled.ngeom),
                    "compiled_nsite": int(compiled.nsite),
                    "compiled_hand_bodies": [
                        name(compiled, mujoco.mjtObj.mjOBJ_BODY, body_id)
                        for body_id in range(compiled.nbody)
                        if re.search(
                            r"hand|thumb|index|middle|palm|finger",
                            name(compiled, mujoco.mjtObj.mjOBJ_BODY, body_id),
                            re.I,
                        )
                    ],
                }
            )
        except Exception as exc:
            row.update({"compiled": False, "compile_error": repr(exc)})
        variants.append(row)
    return variants


def write_contact_scene() -> Path:
    scene = TASK_DIR / "g1_contact_probe_scene.xml"
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    root.set("model", "g1_contact_probe")

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", str((G1_DIR / "meshes").resolve()))

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        {
            "type": "2d",
            "name": "groundplane",
            "builtin": "checker",
            "mark": "edge",
            "rgb1": "0.2 0.3 0.4",
            "rgb2": "0.1 0.2 0.3",
            "markrgb": "0.8 0.8 0.8",
            "width": "300",
            "height": "300",
        },
    )
    ET.SubElement(
        asset,
        "material",
        {
            "name": "groundplane",
            "texture": "groundplane",
            "texuniform": "true",
            "texrepeat": "5 5",
            "reflectance": "0.2",
        },
    )
    ET.SubElement(asset, "material", {"name": "probe_cube_mat", "rgba": "0.8 0.2 0.1 1"})

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("G1 model has no worldbody")
    ET.SubElement(worldbody, "light", {"pos": "0 0 1.5", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(
        worldbody,
        "geom",
        {"name": "floor", "size": "0 0 0.05", "type": "plane", "material": "groundplane"},
    )
    cube = ET.SubElement(worldbody, "body", {"name": "probe_cube", "pos": "0.30 -0.12 0.84"})
    ET.SubElement(
        cube,
        "geom",
        {
            "name": "probe_cube_geom",
            "type": "box",
            "size": "0.035 0.035 0.035",
            "mass": "0.05",
            "material": "probe_cube_mat",
            "contype": "1",
            "conaffinity": "1",
            "friction": "1 0.01 0.001",
        },
    )

    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


def write_site_probe_scene() -> Path:
    scene = TASK_DIR / "g1_site_probe_scene.xml"
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    root.set("model", "g1_site_probe")
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", str((G1_DIR / "meshes").resolve()))
    for body in root.iter("body"):
        if body.get("name") == "right_wrist_yaw_link":
            ET.SubElement(
                body,
                "site",
                {
                    "name": "right_wrist_tcp_probe",
                    "pos": "0.09 -0.003 0",
                    "size": "0.01",
                    "rgba": "0 1 0 1",
                },
            )
            break
    else:
        raise RuntimeError("right_wrist_yaw_link not found")
    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


def set_joint(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, value: float) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_addr = int(model.jnt_qposadr[joint_id])
    data.qpos[qpos_addr] = value


def qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_addr = int(model.jnt_qposadr[joint_id])
    return float(data.qpos[qpos_addr])


def run_contact_test(scene: Path) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    # A fixed-pelvis kinematic sweep isolates whether the stock wrist collision
    # geometry can report contact with an object. The cube is static and never
    # welded, attached, or moved after model load.
    initial_pose: dict[str, float] = {}
    target_pose = {
        "right_shoulder_pitch_joint": -0.5,
        "right_elbow_joint": 0.8,
    }

    right_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "probe_cube_geom")
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "probe_cube")

    contact_events = []
    min_distance = math.inf
    start_had_contact = False
    for step in range(101):
        mujoco.mj_resetData(model, data)
        scale = step / 100.0
        for joint, target in target_pose.items():
            set_joint(model, data, joint, scale * target)
        mujoco.mj_forward(model, data)

        wrist_pos = np.array(data.xpos[right_body_id])
        cube_pos = np.array(data.xpos[cube_body_id])
        min_distance = min(min_distance, float(np.linalg.norm(wrist_pos - cube_pos)))
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if cube_geom_id not in (int(contact.geom1), int(contact.geom2)):
                continue
            other_geom_id = int(contact.geom2 if int(contact.geom1) == cube_geom_id else contact.geom1)
            other_body_id = int(model.geom_bodyid[other_geom_id])
            other_body = name(model, mujoco.mjtObj.mjOBJ_BODY, other_body_id)
            if "right_" not in other_body:
                continue
            if step == 0:
                start_had_contact = True
            contact_events.append(
                {
                    "step": step,
                    "time": float(data.time),
                    "cube_geom": name(model, mujoco.mjtObj.mjOBJ_GEOM, cube_geom_id),
                    "other_geom": name(model, mujoco.mjtObj.mjOBJ_GEOM, other_geom_id),
                    "other_body": other_body,
                    "dist": float(contact.dist),
                    "pos": floats(contact.pos),
                }
            )
        if contact_events:
            break

    return {
        "scene": str(scene.relative_to(ROOT)),
        "cube_body": "probe_cube",
        "cube_geom": "probe_cube_geom",
        "cube_initial_pos": [0.30, -0.12, 0.84],
        "motion": {
            "method": "fixed-pelvis kinematic joint sweep",
            "steps": 101,
            "target_joint_pose": target_pose,
            "initial_joint_pose": initial_pose,
        },
        "right_wrist_body": "right_wrist_yaw_link",
        "right_wrist_final_pos": floats(data.xpos[right_body_id]),
        "cube_final_pos": floats(data.xpos[cube_body_id]),
        "min_wrist_cube_body_distance": min_distance,
        "contact_detected": bool(contact_events),
        "start_had_contact": start_had_contact,
        "contacts": contact_events[:20],
        "final_right_arm_qpos": {
            joint: qpos(model, data, joint)
            for joint in [
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_roll_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ]
        },
    }


def run_free_standing_stability_probe() -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    mujoco.mj_forward(model, data)
    start_pos = floats(data.xpos[pelvis_id])
    commands = {"right_shoulder_pitch": 0.2, "right_elbow": 0.2, "left_shoulder_pitch": 0.2}
    actuator_ids = {
        actuator: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
        for actuator in commands
    }
    for _ in range(1000):
        for actuator, ctrl in commands.items():
            data.ctrl[actuator_ids[actuator]] = ctrl
        mujoco.mj_step(model, data)
    end_pos = floats(data.xpos[pelvis_id])
    return {
        "scene": str(SCENE.relative_to(ROOT)),
        "method": "free-standing dynamics with small constant arm torques",
        "duration_seconds": float(data.time),
        "commands": commands,
        "pelvis_start_pos": start_pos,
        "pelvis_end_pos": end_pos,
        "pelvis_translation_norm": float(np.linalg.norm(np.array(end_pos) - np.array(start_pos))),
        "stable_for_mvp_manipulation": False,
    }


def main() -> int:
    TASK_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_DIR.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    inventory = {
        "repo_commit": "4134cb5dc7ff1ba7f484deda48b5274b58694519",
        "scene": str(SCENE.relative_to(ROOT)),
        "model_xml": str(MODEL_XML.relative_to(ROOT)),
        "mujoco_version": mujoco.__version__,
        "model_counts": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "nbody": int(model.nbody),
            "njnt": int(model.njnt),
            "ngeom": int(model.ngeom),
            "nsite": int(model.nsite),
            "nsensor": int(model.nsensor),
        },
        "actuators": actuator_inventory(model),
        "bodies": relevant_bodies(model),
        "sites": site_inventory(model),
        "hand_collision_geoms": geom_inventory(model, r"hand|thumb|index|middle|finger|palm|wrist"),
        "variants": variant_inventory(),
    }
    (LOG_DIR / "g1_actuators.json").write_text(json.dumps(inventory, indent=2) + "\n", encoding="utf-8")

    contact_scene = write_contact_scene()
    site_scene = write_site_probe_scene()
    contacts = run_contact_test(contact_scene)
    contacts["site_probe_scene"] = str(site_scene.relative_to(ROOT))
    contacts["free_standing_stability_probe"] = run_free_standing_stability_probe()
    (LOG_DIR / "g1_contacts.json").write_text(json.dumps(contacts, indent=2) + "\n", encoding="utf-8")

    print(json.dumps({"inventory": str(LOG_DIR / "g1_actuators.json"), "contacts": contacts}, indent=2))
    return 0 if contacts["contact_detected"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
