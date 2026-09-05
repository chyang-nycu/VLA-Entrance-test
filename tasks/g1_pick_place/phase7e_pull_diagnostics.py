"""Phase 7E: per-step diagnostic instrumentation for the Task 3 door pull.

Answers, with real measured data rather than inference: what does
conditioning/contact-force/orientation/slip actually look like AS A
FUNCTION OF TIME during PULL_ARC, and does slip correlate with any of them.

Non-invasive: uses run_trial_door_open's existing frame_callback hook
(the same mechanism Task 1 uses for video capture, run_pick_place.py)
rather than modifying the trial function itself. Every quantity here is
independently recomputed from `data` at each physics step -- none of it
reads run_trial_door_open's own internal telemetry, so this is a genuine
second measurement, not a re-labeling of the first.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap
from tasks.g1_pick_place.controller_3c import orientation_residual_rad
from tasks.g1_pick_place.run_pick_place import relative_slip_m, tcp_local_cube_offset
from tasks.g1_pick_place.door_open import (
    GRIPPER_KP_DOOR,
    HANDLE_GRASP_CORRIDOR_RAD,
    run_trial_door_open,
)
from tasks.g1_pick_place.workspace_map import handle_pose

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"

_WP_PATTERN = re.compile(r"_wp(\d+)$")


def _reconstruct_commanded_target(phase: str, geometry: dict, phi0_live_deg: float) -> np.ndarray | None:
    """The Cartesian target the arm is being driven toward during `phase`,
    reconstructed from the geometry alone (not read from any internal
    trial state) -- so "TCP position error" below is error against an
    independently-derived target, not a self-referential comparison.
    """
    pivot_xy = tuple(geometry["pivot_xy"])
    radius = geometry["radius_m"]
    theta = geometry["theta_deg"]
    z = geometry.get("handle_z", 0.9)
    phi0 = geometry["phi0_deg"]

    m = _WP_PATTERN.search(phase)
    if phase.startswith("PULL_ARC") and m is not None:
        i = int(m.group(1))
        n_waypoints = 75  # PULL_ARC_N_WAYPOINTS, shipped value
        alpha = (i + 1) / n_waypoints
        phi = phi0_live_deg + alpha * ((phi0 + theta) - phi0_live_deg)
        return handle_pose(pivot_xy, radius, phi, z)
    if phase.startswith("PREGRASP_HANDLE"):
        return handle_pose(pivot_xy, radius, phi0_live_deg - 8.0, z)
    if phase.startswith("APPROACH_HANDLE") or phase == "CLOSE":
        return handle_pose(pivot_xy, radius, phi0_live_deg, z)
    if phase.startswith("SETTLE_OPEN") or phase in ("OPEN", "RELEASE_SETTLE"):
        return handle_pose(pivot_xy, radius, phi0 + theta, z)
    if phase == "RETREAT":
        open_pos = handle_pose(pivot_xy, radius, phi0 + theta, z)
        return open_pos + np.array([-0.10, 0.0, 0.0])
    return None


def run_with_diagnostics(scene_path: Path, geometry: dict, initial_hinge_angle_rad: float = 0.0) -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    handle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "door_handle_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
    hinge_qpos_adr = model.jnt_qposadr[hid]
    hinge_dof_adr = model.jnt_dofadr[hid]

    phi0_live_deg = geometry["phi0_deg"] + np.degrees(initial_hinge_angle_rad)

    log: list[dict] = []
    grasp_ref = {"value": None}

    def cb(phase: str, model_: mujoco.MjModel, data: mujoco.MjData) -> None:
        tcp_pos = data.site_xpos[site_id].copy()
        tcp_rot = data.site_xmat[site_id].reshape(3, 3)
        handle_pos = data.geom_xpos[handle_id].copy()
        hinge_qpos = float(data.qpos[hinge_qpos_adr])
        hinge_qvel = float(data.qvel[hinge_dof_adr])

        jacp = np.zeros((3, model_.nv))
        jacr = np.zeros((3, model_.nv))
        mujoco.mj_jacSite(model_, data, jacp, jacr, site_id)
        J = jacp[:, arm_map.dof_adr]
        svals = np.linalg.svd(J, compute_uv=False)
        sigma_min, sigma_max = float(svals.min()), float(svals.max())
        cond = sigma_max / sigma_min if sigma_min > 1e-12 else float("inf")
        manip = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
        orient_deg = float(np.degrees(orientation_residual_rad(tcp_rot.flatten())))

        # Per-pad contact force at the handle, plus its normal/tangential
        # split relative to the panel's own swing direction (tangent to
        # the circle about the pivot) -- normal component is "grip
        # force", tangential is the force actually driving/resisting the
        # door's rotation ("door reaction").
        pivot = np.array(geometry["pivot_xy"] + [handle_pos[2]])
        radial = handle_pos - pivot
        radial_xy = radial[:2] / (np.linalg.norm(radial[:2]) + 1e-12)
        tangent_xy = np.array([-radial_xy[1], radial_xy[0]])

        left_force = right_force = 0.0
        left_tangent = right_tangent = 0.0
        left_contact = right_contact = False
        for ci in range(data.ncon):
            con = data.contact[ci]
            pair = (int(con.geom1), int(con.geom2))
            if handle_id not in pair:
                continue
            force6 = np.zeros(6)
            mujoco.mj_contactForce(model_, data, ci, force6)
            normal_n = float(abs(force6[0]))
            frame_normal_world = np.array(con.frame[0:3])
            tangential_n = float(abs(np.dot(frame_normal_world[:2], tangent_xy)) * np.linalg.norm(force6[:3]))
            if left_pad_id in pair:
                left_force = max(left_force, normal_n)
                left_tangent = max(left_tangent, tangential_n)
                left_contact = True
            elif right_pad_id in pair:
                right_force = max(right_force, normal_n)
                right_tangent = max(right_tangent, tangential_n)
                right_contact = True

        both_contact = left_contact and right_contact
        min_force = min(left_force, right_force) if both_contact else 0.0

        if phase.startswith("PULL_ARC") and grasp_ref["value"] is None and both_contact:
            grasp_ref["value"] = tcp_local_cube_offset(tcp_pos, tcp_rot, handle_pos)
        slip_mm = None
        if grasp_ref["value"] is not None:
            local_now = tcp_local_cube_offset(tcp_pos, tcp_rot, handle_pos)
            slip_mm = relative_slip_m(local_now, grasp_ref["value"]) * 1000.0

        target = _reconstruct_commanded_target(phase, geometry, phi0_live_deg)
        tcp_err_mm = float(np.linalg.norm(tcp_pos - target)) * 1000.0 if target is not None else None

        log.append({
            "t": float(data.time),
            "phase": phase,
            "hinge_deg": float(np.degrees(hinge_qpos)),
            "hinge_qvel": hinge_qvel,
            "tcp_err_mm": tcp_err_mm,
            "condition_number": cond,
            "sigma_min": sigma_min,
            "manipulability": manip,
            "orientation_residual_deg": orient_deg,
            "left_force_n": left_force,
            "right_force_n": right_force,
            "min_bilateral_force_n": min_force,
            "both_contact": both_contact,
            "left_tangent_n": left_tangent,
            "right_tangent_n": right_tangent,
            "slip_mm": slip_mm,
        })

    result = run_trial_door_open(scene_path, geometry, initial_hinge_angle_rad=initial_hinge_angle_rad, frame_callback=cb)
    return {"result": result, "log": log}


def main() -> int:
    geometry = json.loads((LOG_DIR / "phase7b_selected_door_geometry.json").read_text())
    scene_path = ROOT / "tasks" / "g1_pick_place" / "g1_grasp_scene_door.xml"
    out = run_with_diagnostics(scene_path, geometry)
    print(f"logged {len(out['log'])} steps; door_pass={out['result']['door_pass']}")
    (LOG_DIR / "phase7e_pull_diagnostics.json").write_text(
        json.dumps(out, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
