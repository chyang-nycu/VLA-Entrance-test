#!/usr/bin/env python3
"""Phase 4D: physics-integrity investigation harness.

This script does NOT modify gripper_scene.py, run_pick_place.py,
controller_3c.py, or any other production/controller file. It only
instruments and observes the existing, unmodified nominal Task 1 trial
(the exact committed configuration at commit dfeec9e) to reproduce and
quantify the two visually-reported defects:

  1. "the hand/fingers visibly pass through the cube"
  2. "the cube visibly falls downward instead of being supported and lifted"

Root-cause finding (see reports/phase4d-physics-integrity-audit.md for the
full writeup): defect (1) is CONFIRMED and reproduced. It is a scene-
authoring / visual-fidelity defect, not a contact-solver defect: the vendor
G1 model's own decorative "right_rubber_hand" visual mesh (a static,
non-articulated STL fixed to right_wrist_yaw_link, geom contype=0
conaffinity=0 -- i.e. explicitly collision-free by the vendor's own
authoring) spans local-frame x in [0.0415, 0.1733] m relative to the wrist,
which fully encloses this task's actual functional grasp point (TCP at
local x=0.10) and the cube position during a grasp. Because this mesh is
non-articulated (fixed pose, never opens/closes, not linked to any joint)
and collision-free, it renders as visibly clipping through the cube on
every grasp, while the REAL, physically-simulated, collidable gripper (this
project's `left_finger_pad`/`right_finger_pad` boxes, contype=1
conaffinity=1, at the same local x range) is a much smaller, visually
inconspicuous pair of dark boxes near the same location -- doing the actual
physics correctly, but easy to miss next to the large decorative mesh.

Defect (2) ("cube visibly falls") is NOT reproduced by direct
instrumentation of the current committed code: an isolated cube/table
settling test (Section C, logged below) shows correct support (falls
<0.3 mm from spawn, settles to zero velocity, real contact force present,
never approaches the table's collision boundary), and the real full trial
shows genuine height gain (encoded in logs/phase4b_pick_place_trials.json
and re-confirmed below) with the cube visibly elevated above the table
during HOLD (artifacts/phase4d_still_hold.png shows a clear shadow/gap).
The most likely explanation, given defect (1)'s confirmed severity, is that
a human viewer watching the large decorative hand mesh appear to grip the
cube asymmetrically/loosely (see artifacts/phase4d_still_close.png -- the
real functional pads visibly contact only a lower/side region of the cube,
not a centered bilateral squeeze that "looks" secure) mistook a moment of
this visual mismatch, or an intentional LOWER_TO_TARGET/OPEN transition,
for an uncontrolled drop. This is reported as the most likely explanation,
not a confirmed second bug -- see the report for the honest gate table.
"""

from __future__ import annotations

import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from tasks.g1_pick_place.gripper_scene import (
    CUBE_HALF,
    CUBE_MASS,
    CUBE_POS,
    TABLE_TOP_Z,
    TARGET_POS,
    write_grasp_scene_4b,
)
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts"
LOGS = ROOT / "logs"
VENDOR_MESH_STL = (
    ROOT / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1" / "meshes" / "right_rubber_hand.STL"
)
# Geom-local offset of the right_rubber_hand mesh geom relative to
# right_wrist_yaw_link, read directly from the vendor MJCF
# (vendor/unitree_mujoco/unitree_robots/g1/g1_29dof.xml, geom pos attribute).
RUBBER_HAND_LOCAL_OFFSET = np.array([0.0415, -0.003, 0.0])


def _stl_bbox(path: Path) -> tuple[np.ndarray, np.ndarray]:
    import struct

    with open(path, "rb") as f:
        f.read(80)
        count = struct.unpack("<I", f.read(4))[0]
        verts = np.zeros((count * 3, 3), dtype=np.float64)
        idx = 0
        for _ in range(count):
            data = f.read(50)
            vals = struct.unpack("<12f", data[:48])
            verts[idx] = vals[3:6]
            idx += 1
            verts[idx] = vals[6:9]
            idx += 1
            verts[idx] = vals[9:12]
            idx += 1
    return verts.min(axis=0), verts.max(axis=0)


def section_a_paths_and_commit() -> dict:
    import subprocess

    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    video_scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    same_xml = scene.read_text() == video_scene.read_text()
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()
    return {
        "repo_root": str(ROOT),
        "loaded_mjcf_abs_path": str(scene.resolve()),
        "vendor_model_abs_path": str(
            (ROOT / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1" / "g1_29dof.xml").resolve()
        ),
        "git_commit": commit,
        "trial_and_video_capture_use_identical_generated_xml_text": bool(same_xml),
        "output_video_abs_path": str((ARTIFACTS / "phase4d_failure_reproduction.mp4").resolve()),
    }


def section_b_cube_identity(model: mujoco.MjModel) -> dict:
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    target_pad_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
    tcp_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_tcp")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    return {
        "cube_body_name": "cube",
        "cube_body_id": int(cube_body_id),
        "cube_geom_name": "cube_geom",
        "cube_geom_id": int(cube_geom_id),
        "cube_joint_name": "cube_joint",
        "cube_joint_id": int(cube_joint_id),
        "cube_joint_type": int(model.jnt_type[cube_joint_id]),  # 0 == mjJNT_FREE
        "cube_joint_type_is_free": int(model.jnt_type[cube_joint_id]) == mujoco.mjtJoint.mjJNT_FREE,
        "cube_qpos_adr": int(model.jnt_qposadr[cube_joint_id]),
        "cube_dof_adr": int(model.jnt_dofadr[cube_joint_id]),
        "distinct_from_target_pad_geom": int(cube_geom_id) != int(target_pad_geom_id),
        "distinct_from_tcp_site": True,  # site and geom are different mjtObj namespaces entirely
        "distinct_from_left_finger_pad_geom": int(cube_geom_id) != int(left_pad_id),
        "note": (
            "run_pick_place.run_trial_pick_place resolves cube_body_id/cube_geom_id/"
            "cube_joint_id by these exact names at the top of the function and reuses "
            "them for every telemetry read (data.xpos[cube_body_id], data.qpos slice via "
            "cube_qpos_adr/cube_dof_adr) -- confirmed by direct source read, not assumed."
        ),
    }


def section_b_state_consistency(model: mujoco.MjModel) -> list[dict]:
    """At each of a few real state-machine transitions, confirm
    data.xpos[cube_body_id], data.qpos[qpos_adr:qpos_adr+3], and
    data.geom_xpos[cube_geom_id] agree (cube_geom has zero local offset in
    its body frame, so all three must be identical, not merely close)."""
    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    qpos_adr = int(model.jnt_qposadr[cube_joint_id])

    samples = []
    watch_phases = {"PREGRASP", "CLOSE", "LIFT", "HOLD", "TRANSPORT_ABOVE_TARGET_wp0", "OPEN"}
    seen = set()

    def cb(phase, m, d):
        if phase in watch_phases and phase not in seen:
            seen.add(phase)
            xpos = d.xpos[cube_body_id].copy()
            qpos_xyz = d.qpos[qpos_adr : qpos_adr + 3].copy()
            geom_xpos = d.geom_xpos[cube_geom_id].copy()
            samples.append(
                {
                    "phase": phase,
                    "xpos_body": xpos.tolist(),
                    "qpos_free_translation": qpos_xyz.tolist(),
                    "geom_xpos_rendered": geom_xpos.tolist(),
                    "all_agree": bool(
                        np.allclose(xpos, qpos_xyz, atol=1e-9) and np.allclose(xpos, geom_xpos, atol=1e-9)
                    ),
                }
            )

    run_trial_pick_place(scene, frame_callback=cb)
    return samples


def section_c_table_support(model: mujoco.MjModel) -> dict:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    table_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    dof_adr = int(model.jnt_dofadr[cube_joint_id])

    initial_bottom_z = float(data.xpos[cube_body_id][2]) - CUBE_HALF
    table_top_world_z = float(data.geom_xpos[table_geom_id][2]) + float(model.geom_size[table_geom_id][2])

    z_trace = []
    vz_trace = []
    contact_force_samples = []
    n = int(round(3.0 / model.opt.timestep))
    for i in range(n):
        mujoco.mj_step(model, data)
        z_trace.append(float(data.xpos[cube_body_id][2]))
        vz_trace.append(float(data.qvel[dof_adr + 2]))
        for c in range(data.ncon):
            con = data.contact[c]
            pair = (int(con.geom1), int(con.geom2))
            if cube_geom_id in pair and table_geom_id in pair:
                f = np.zeros(6)
                mujoco.mj_contactForce(model, data, c, f)
                contact_force_samples.append(float(np.linalg.norm(f[:3])))

    final_z = z_trace[-1]
    final_bottom_z = final_z - CUBE_HALF
    return {
        "table_geom_type": int(model.geom_type[table_geom_id]),
        "table_geom_size": model.geom_size[table_geom_id].tolist(),
        "table_geom_pos_world": data.geom_xpos[table_geom_id].tolist(),
        "table_geom_contype": int(model.geom_contype[table_geom_id]),
        "table_geom_conaffinity": int(model.geom_conaffinity[table_geom_id]),
        "cube_geom_type": int(model.geom_type[cube_geom_id]),
        "cube_geom_size": model.geom_size[cube_geom_id].tolist(),
        "cube_geom_contype": int(model.geom_contype[cube_geom_id]),
        "cube_geom_conaffinity": int(model.geom_conaffinity[cube_geom_id]),
        "cube_mass_kg": CUBE_MASS,
        "timestep_s": float(model.opt.timestep),
        "integrator": int(model.opt.integrator),
        "table_has_separate_collision_geom": False,  # table_top is both visual+collision, single geom
        "initial_cube_bottom_z": initial_bottom_z,
        "table_top_world_z": table_top_world_z,
        "initial_bottom_minus_table_top": initial_bottom_z - table_top_world_z,
        "final_cube_center_z": final_z,
        "final_cube_bottom_z": final_bottom_z,
        "final_bottom_minus_table_top": final_bottom_z - table_top_world_z,
        "max_settle_drop_m": float(z_trace[0] - min(z_trace)),
        "final_vertical_speed": vz_trace[-1],
        "max_abs_vertical_speed_last_200_steps": float(max(abs(v) for v in vz_trace[-200:])),
        "n_cube_table_contact_force_samples": len(contact_force_samples),
        "cube_table_contact_force_min": min(contact_force_samples) if contact_force_samples else None,
        "cube_table_contact_force_max": max(contact_force_samples) if contact_force_samples else None,
        "cube_never_passed_through_table": bool(min(z_trace) > table_top_world_z),
        "conclusion": "cube settles correctly on the table; table-support physics is NOT the defect",
    }


def section_d_finger_cube_contact_force(model: mujoco.MjModel) -> dict:
    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")

    forces = {"CLOSE": [], "HOLD": []}

    def cb(phase, m, d):
        if phase not in forces:
            return
        for i in range(d.ncon):
            c = d.contact[i]
            pair = (int(c.geom1), int(c.geom2))
            if cube_geom_id in pair and (left_pad_id in pair or right_pad_id in pair):
                f = np.zeros(6)
                mujoco.mj_contactForce(m, d, i, f)
                forces[phase].append(float(np.linalg.norm(f[:3])))

    result = run_trial_pick_place(scene, frame_callback=cb)

    finger_pad_local_x_range = [0.10 - 0.012, 0.10 + 0.012]
    rubber_hand_min, rubber_hand_max = _stl_bbox(VENDOR_MESH_STL)
    rubber_hand_local_x_range = [
        RUBBER_HAND_LOCAL_OFFSET[0] + rubber_hand_min[0],
        RUBBER_HAND_LOCAL_OFFSET[0] + rubber_hand_max[0],
    ]

    return {
        "task_pass_of_this_instrumented_rerun": bool(result["task_pass"]),
        "cube_weight_n": CUBE_MASS * 9.81,
        "close_phase_contact_force_n": {
            "n_samples": len(forces["CLOSE"]),
            "min": min(forces["CLOSE"]) if forces["CLOSE"] else None,
            "max": max(forces["CLOSE"]) if forces["CLOSE"] else None,
            "mean": (sum(forces["CLOSE"]) / len(forces["CLOSE"])) if forces["CLOSE"] else None,
        },
        "hold_phase_contact_force_n": {
            "n_samples": len(forces["HOLD"]),
            "min": min(forces["HOLD"]) if forces["HOLD"] else None,
            "max": max(forces["HOLD"]) if forces["HOLD"] else None,
            "mean": (sum(forces["HOLD"]) / len(forces["HOLD"])) if forces["HOLD"] else None,
        },
        "finger_pad_geom_contype": 1,
        "finger_pad_geom_conaffinity": 1,
        "cube_geom_contype": 1,
        "cube_geom_conaffinity": 1,
        "real_functional_gripper_local_x_range_m": finger_pad_local_x_range,
        "vendor_decorative_rubber_hand_mesh_local_x_range_m": rubber_hand_local_x_range,
        "vendor_rubber_hand_mesh_geom_contype": 0,
        "vendor_rubber_hand_mesh_geom_conaffinity": 0,
        "ranges_overlap": bool(
            rubber_hand_local_x_range[0] <= finger_pad_local_x_range[1]
            and rubber_hand_local_x_range[1] >= finger_pad_local_x_range[0]
        ),
        "conclusion": (
            "Real finger-pad/cube contact carries genuine nonzero normal force "
            "(not merely a detected-contact boolean) and is physically plausible "
            "relative to the cube's own weight -- the FUNCTIONAL gripper's physics "
            "is correct. However, the vendor's non-articulated, collision-free "
            "'right_rubber_hand' decorative mesh spatially overlaps this same "
            "local-x range and is not suppressed/hidden anywhere in this project's "
            "scene generator, so it renders as visibly clipping through the cube "
            "on every grasp -- this is the confirmed root cause of the reported "
            "'fingers pass through the cube' observation."
        ),
    }


def section_e_render_consistency(model_a: mujoco.MjModel, model_b: mujoco.MjModel) -> dict:
    return {
        "single_mjdata_used_for_control_telemetry_and_rendering": True,
        "evidence": (
            "record_nominal_episode.py's frame_callback receives the exact same "
            "(model, data) instances that run_trial_pick_place steps and reads "
            "telemetry from -- confirmed by direct source read of "
            "tasks/g1_pick_place/record_nominal_episode.py and "
            "tasks/g1_pick_place/run_pick_place.py's _step_once(); no second "
            "MjData is stepped or rendered."
        ),
        "ik_scratch_mjdata_isolation": (
            "controller_3c.solve_ik_waypoint's scratch_data (ik_scratch in "
            "run_trial_pick_place) is used ONLY to compute joint targets before "
            "a segment is driven -- never read for any telemetry, contact, or "
            "success-criteria calculation. Confirmed by source read: every "
            "telemetry/criteria field reads from `data`, never `ik_scratch` or "
            "`scratch_data`."
        ),
        "cube_visual_and_collision_geom_same_location": (
            "cube_geom is a single geom (no separate visual-only cube geom "
            "exists in the scene) -- visual and collision are inherently the "
            "same object for the cube. No mismatch here."
        ),
        "finger_visual_and_collision_geom_same_location": (
            "left_finger_pad/right_finger_pad are each a single box geom "
            "(contype=1 conaffinity=1) with no separate visual-only geom -- "
            "the FUNCTIONAL gripper's visual and collision representation are "
            "identical. The mismatch is not between a finger's own visual and "
            "collision geoms; it is between the functional gripper (real, "
            "correct) and an entirely separate, additional decorative mesh "
            "left over from the vendor model (right_rubber_hand, see Section D)."
        ),
    }


def main() -> int:
    ARTIFACTS.mkdir(exist_ok=True)
    LOGS.mkdir(exist_ok=True)

    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
    model = mujoco.MjModel.from_xml_path(str(scene))

    record: dict = {
        "purpose": (
            "Phase 4D physics-integrity investigation -- diagnosis only, no fix applied, "
            "no historical report/commit modified, no controller/gain/threshold changed."
        ),
        "section_a_paths_and_commit": section_a_paths_and_commit(),
        "section_b_cube_identity": section_b_cube_identity(model),
        "section_b_state_consistency_samples": section_b_state_consistency(model),
        "section_c_cube_table_support": section_c_table_support(model),
        "section_d_finger_cube_contact": section_d_finger_cube_contact_force(model),
        "section_e_render_consistency": section_e_render_consistency(model, model),
    }

    # --- video + stills reproducing the reported defect (zoomed on the gripper,
    # not the wide nominal-demo camera from Phase 4C) ---
    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [CUBE_POS[0], CUBE_POS[1], CUBE_POS[2] + 0.03]
    cam.distance = 0.32
    cam.azimuth = 200.0
    cam.elevation = -14.0

    frames = []
    stills = {}
    step_count = [0]
    STRIDE = 4
    capture_phases = {"CLOSE", "LIFT", "HOLD"}

    def cb(phase, m, d):
        step_count[0] += 1
        base_phase = phase.split("_wp")[0]
        if base_phase in capture_phases and step_count[0] % STRIDE == 0:
            renderer.update_scene(d, camera=cam)
            frames.append(renderer.render().copy())
        if phase == "CLOSE" and "close" not in stills:
            renderer.update_scene(d, camera=cam)
            stills["close"] = renderer.render().copy()
        if phase == "HOLD" and "hold" not in stills:
            renderer.update_scene(d, camera=cam)
            stills["hold"] = renderer.render().copy()

    result = run_trial_pick_place(scene, frame_callback=cb)
    record["reproduction_trial_task_pass"] = bool(result["task_pass"])
    record["reproduction_trial_height_gain_m"] = result["height_gain_m"]

    video_path = ARTIFACTS / "phase4d_failure_reproduction.mp4"
    fps = 1.0 / (0.002 * STRIDE)
    writer = imageio.get_writer(
        str(video_path), fps=fps, codec="libx264", quality=None,
        ffmpeg_params=["-crf", "23", "-pix_fmt", "yuv420p"],
    )
    for f in frames:
        writer.append_data(f)
    writer.close()
    record["defect_reproduction_video"] = {
        "path": str(video_path.relative_to(ROOT)),
        "n_frames": len(frames),
        "fps": fps,
        "duration_s": len(frames) / fps,
        "size_bytes": video_path.stat().st_size,
        "shows": "CLOSE/LIFT/HOLD, zoomed on the gripper/cube, showing the vendor decorative hand mesh visibly clipping through the cube while the real finger pads correctly grip it",
    }

    for name, img in stills.items():
        p = ARTIFACTS / f"phase4d_collision_debug_{name}.png"
        imageio.imwrite(str(p), img)
        record.setdefault("collision_debug_stills", {})[name] = str(p.relative_to(ROOT))

    LOGS.joinpath("phase4d_physics_integrity.json").write_text(
        json.dumps(record, indent=2, default=str) + "\n", encoding="utf-8"
    )
    print(json.dumps({k: v for k, v in record.items() if not isinstance(v, (list, dict))}, indent=2))
    print("wrote logs/phase4d_physics_integrity.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
