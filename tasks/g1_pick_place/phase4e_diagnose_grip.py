#!/usr/bin/env python3
"""Phase 4E, Section B: evidence-first grasp-stability diagnosis.

Runs one real nominal Task 1 trial against a given scene file with a
frame_callback that records, at every step, the actual contact state
between each finger pad and the cube (mj_contactForce -- not merely a
contact-exists boolean), the contact position in world Z relative to the
cube's own center Z, and the cube's world position. Also reads the model's
own cube mass/friction/gravity directly (not hardcoded assumptions) to
compute the theoretical minimum bilateral normal force and the actual
observed safety factor.

This is a diagnostic tool, not a test -- it prints/returns evidence used to
justify the Section B redesign in reports/phase4e-gripper-integrity-repair.md.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tasks.g1_pick_place import gripper_scene as gs  # noqa: E402
from tasks.g1_pick_place import run_pick_place as rp  # noqa: E402


def diagnose(scene_path: Path, gripper_kp: float, gripper_kd: float) -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene_path))

    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")

    cube_mass = float(model.body_mass[cube_body_id])
    gravity = float(-model.opt.gravity[2])
    cube_friction = model.geom_friction[cube_geom_id].copy().tolist()
    left_friction = model.geom_friction[left_pad_id].copy().tolist()
    right_friction = model.geom_friction[right_pad_id].copy().tolist()
    # MuJoCo's documented default combination rule for two geoms of equal
    # priority (both 0 here, vendor and task-local geoms never set
    # priority): the CONTACT friction is the element-wise MAXIMUM of the two
    # geoms' friction arrays (not an average) -- see MuJoCo docs, "Contact
    # parameters". mu_effective is therefore max(cube, pad) sliding friction.
    mu_effective = max(cube_friction[0], left_friction[0])

    weight_n = cube_mass * gravity
    n_min_bilateral = weight_n / (2.0 * mu_effective)

    log = {
        "cube_mass_kg": cube_mass,
        "gravity_mps2": gravity,
        "weight_n": weight_n,
        "cube_friction_xml": cube_friction,
        "finger_pad_friction_xml": left_friction,
        "friction_combination_rule": "MuJoCo default: element-wise max for equal-priority geoms",
        "mu_effective": mu_effective,
        "n_min_bilateral_formula": "m*g/(2*mu)",
        "n_min_bilateral_n": n_min_bilateral,
        "per_phase": {},
        "contact_z_offset_from_cube_center_m": {"min": None, "max": None, "samples": []},
    }

    per_phase_forces: dict[str, dict[str, list[float]]] = {}
    z_offsets: list[float] = []

    def cb(phase: str, m: mujoco.MjModel, d: mujoco.MjData) -> None:
        bucket = per_phase_forces.setdefault(phase, {"left": [], "right": []})
        cube_z = float(d.xpos[cube_body_id][2])
        for i in range(d.ncon):
            c = d.contact[i]
            pair = (int(c.geom1), int(c.geom2))
            if cube_geom_id not in pair:
                continue
            f = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, f)
            normal_n = float(abs(f[0]))
            if left_pad_id in pair:
                bucket["left"].append(normal_n)
                if normal_n > 1e-6:
                    z_offsets.append(float(c.pos[2]) - cube_z)
            elif right_pad_id in pair:
                bucket["right"].append(normal_n)
                if normal_n > 1e-6:
                    z_offsets.append(float(c.pos[2]) - cube_z)

    result = rp.run_trial_pick_place(scene_path, gripper_kp=gripper_kp, gripper_kd=gripper_kd, frame_callback=cb)

    for phase, sides in per_phase_forces.items():
        entry = {}
        for side in ("left", "right"):
            vals = sides[side]
            entry[side] = {
                "n_samples": len(vals),
                "min_n": min(vals) if vals else None,
                "max_n": max(vals) if vals else None,
                "mean_n": float(np.mean(vals)) if vals else None,
            }
        log["per_phase"][phase] = entry

    if z_offsets:
        log["contact_z_offset_from_cube_center_m"] = {
            "min": float(min(z_offsets)),
            "max": float(max(z_offsets)),
            "mean": float(np.mean(z_offsets)),
            "n_samples": len(z_offsets),
        }

    log["trial_result_summary"] = {
        "task_pass": result["task_pass"],
        "height_gain_m": result["height_gain_m"],
        "max_slip_during_lift": result["max_slip_during_lift"],
        "max_slip_during_transport": result["max_slip_during_transport"],
        "max_slip_during_lower": result["max_slip_during_lower"],
        "slip_at_release": result["slip_at_release"],
        "failure_state": result["failure_state"],
        "failure_reason": result["failure_reason"],
    }

    for phase in ("CLOSE", "VERIFY_BILATERAL_CONTACT", "LIFT", "HOLD"):
        entry = log["per_phase"].get(phase)
        if entry:
            l_min = entry["left"]["min_n"]
            r_min = entry["right"]["min_n"]
            if l_min is not None and r_min is not None:
                worst = min(l_min, r_min)
                log["per_phase"][phase]["bilateral_min_safety_factor"] = (
                    worst / n_min_bilateral if n_min_bilateral > 0 else None
                )

    return log


if __name__ == "__main__":
    scene_name = sys.argv[1] if len(sys.argv) > 1 else "g1_grasp_scene_4b.xml"
    kp = float(sys.argv[2]) if len(sys.argv) > 2 else rp.GRIPPER_KP_4E
    kd = float(sys.argv[3]) if len(sys.argv) > 3 else rp.GRIPPER_KD_4E
    scene = gs.write_grasp_scene_4b(arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name=scene_name)
    out = diagnose(scene, kp, kd)
    print(json.dumps(out, indent=2))
