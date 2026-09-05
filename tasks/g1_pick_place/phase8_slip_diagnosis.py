"""Phase 8: Task 3 slip-onset diagnosis.

Answers: what causes door-pull grasp slip that appears BEFORE bilateral
contact force visibly declines? Phase 7E showed force insufficiency is a
demonstrated partial cause (isolated by a real intervention), but slip
already exceeds the 10mm target before the force decline is visible --
this phase investigates that earlier mechanism directly, one hypothesis
at a time, with a genuine intervention per hypothesis rather than more
correlation.

Instrumentation strategy: monkeypatch `door_open._door_step_once` -- every
drive/settle/arc helper in door_open.py calls it by bare name, resolved
against the module's global namespace at CALL time, so replacing the name
on the module intercepts every call site without touching
run_trial_door_open's own code. This gives the EXACT commanded joint
target passed to each step (not a reconstruction from phase-name
parsing, unlike Phase 7E's frame_callback approach), plus everything else
requested: actual TCP/joint state, contact forces (normal + tangential),
Jacobian conditioning, finger positions, and actuator saturation.
"""

from __future__ import annotations

import json
from contextlib import contextmanager
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place import controller_3c as c3c
from tasks.g1_pick_place import door_open as do
from tasks.g1_pick_place import run_pick_place as rpp
from tasks.g1_pick_place.controller import bounded_pd_step
from tasks.g1_pick_place.gripper_scene import RIGHT_ARM_JOINT_ACTUATOR_PAIRS

ARM_FORCE_LIMITS = np.array([fl for _, _, fl in RIGHT_ARM_JOINT_ACTUATOR_PAIRS])
from tasks.g1_pick_place.controller_3c import orientation_residual_rad
from tasks.g1_pick_place.run_grasp_test_3c import _finger_targets
from tasks.g1_pick_place.workspace_map import handle_pose

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"
ARTIFACTS = ROOT / "artifacts"

# Effective bilateral friction coefficient: MuJoCo's documented default
# combination rule for two geoms of unequal friction is element-wise
# MAXIMUM. Handle friction == CUBE_FRICTION (reused, gripper_scene.py);
# finger pad friction is FINGER_FRICTION -- both first components are 1.0
# and 1.2 respectively (matching Phase 4E's own recorded mu=1.2 for the
# cube case, which used the identical pad/object friction pair).
MU_EFFECTIVE = 1.2


def _rotmat_to_euler_xyz(R: np.ndarray) -> np.ndarray:
    """Intrinsic X-Y-Z Euler angles (roll, pitch, yaw) in radians, purely
    for human-readable orientation logging -- NOT used by any controller
    or criterion, which all use the coordinate-free orientation_residual_rad
    (angle between wrist-local Z and world vertical) instead.
    """
    sy = -R[2, 0]
    sy = np.clip(sy, -1.0, 1.0)
    pitch = np.arcsin(sy)
    if abs(sy) < 0.9999999:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])


@contextmanager
def _patched(module, name, value):
    original = getattr(module, name)
    setattr(module, name, value)
    try:
        yield
    finally:
        setattr(module, name, original)


def run_instrumented(
    scene_path: Path,
    geometry: dict,
    *,
    initial_hinge_angle_rad: float = 0.0,
    gripper_kp: float | None = None,
    gripper_kd: float | None = None,
    orient_weight: float | None = None,
    nullspace_posture_gain: float | None = None,
    use_oriented_ik: bool = True,
    frame_callback=None,
) -> dict:
    """Run one door trial with full per-step instrumentation, optionally
    overriding exactly the ONE knob a given hypothesis test needs. All
    overrides default to None/unchanged so the baseline call reproduces
    the shipped trial exactly.
    """
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    arm_map_probe = None  # filled on first step
    handle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "door_handle_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
    hinge_qpos_adr = model.jnt_qposadr[hid]
    hinge_dof_adr = model.jnt_dofadr[hid]
    site_id_probe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_tcp")

    log: list[dict] = []
    grasp_ref = {"value": None}
    scratch = mujoco.MjData(model)

    real_step = do._door_step_once

    def instrumented_step(model_, data, arm_map, gripper_map, site_id, arm_ctrl_target, finger_open, phase, telemetry, steps_run, guard, frame_cb, carrying=None):
        commanded_q = np.asarray(arm_ctrl_target, dtype=float).copy()
        actual_q_before = arm_map.get_qpos(data).copy()

        # Delegate to the real implementation for correctness (physics,
        # telemetry bookkeeping) -- we only observe around it.
        gripper_diag: dict = {}
        real_bounded_pd_step = bounded_pd_step

        def spying_pd_step(joint_map, data_, target, kp, kd, max_step, max_qvel, diag=None):
            return real_bounded_pd_step(joint_map, data_, target, kp, kd, max_step, max_qvel, diag=gripper_diag)

        with _patched(do, "bounded_pd_step", spying_pd_step):
            real_step(model_, data, arm_map, gripper_map, site_id, arm_ctrl_target, finger_open, phase, telemetry, steps_run, guard, frame_cb, carrying=carrying)

        actual_q_after = arm_map.get_qpos(data).copy()
        joint_err = commanded_q - actual_q_after

        tcp_pos = data.site_xpos[site_id].copy()
        tcp_rot = data.site_xmat[site_id].reshape(3, 3)
        handle_pos = data.geom_xpos[handle_id].copy()
        hinge_qpos = float(data.qpos[hinge_qpos_adr])
        hinge_qvel = float(data.qvel[hinge_dof_adr])

        # Commanded TCP position: forward-kinematics the COMMANDED joint
        # target (not the achieved one) through the same site.
        scratch.qpos[:] = data.qpos
        arm_map.set_qpos(scratch, commanded_q)
        mujoco.mj_kinematics(model_, scratch)
        commanded_tcp = scratch.site_xpos[site_id].copy()
        tcp_err_mm = float(np.linalg.norm(tcp_pos - commanded_tcp)) * 1000.0

        # Jacobian conditioning at the ACTUAL configuration.
        jacp = np.zeros((3, model_.nv))
        jacr = np.zeros((3, model_.nv))
        mujoco.mj_jacSite(model_, data, jacp, jacr, site_id)
        J = jacp[:, arm_map.dof_adr]
        svals = np.linalg.svd(J, compute_uv=False)
        sigma_min, sigma_max = float(svals.min()), float(svals.max())
        cond = sigma_max / sigma_min if sigma_min > 1e-12 else float("inf")

        orient_resid_deg = float(np.degrees(orientation_residual_rad(tcp_rot.flatten())))
        rpy_deg = np.degrees(_rotmat_to_euler_xyz(tcp_rot))

        # Contact forces + tangential decomposition (relative to the
        # panel's own swing direction at the handle).
        pivot = np.array(list(geometry["pivot_xy"]) + [handle_pos[2]])
        radial = handle_pos - pivot
        radial_xy = radial[:2] / (np.linalg.norm(radial[:2]) + 1e-12)
        tangent_xy = np.array([-radial_xy[1], radial_xy[0]])
        left_n = right_n = left_t = right_t = 0.0
        left_c = right_c = False
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
                left_n, left_t, left_c = max(left_n, normal_n), max(left_t, tangential_n), True
            elif right_pad_id in pair:
                right_n, right_t, right_c = max(right_n, normal_n), max(right_t, tangential_n), True
        both_contact = left_c and right_c
        min_normal = min(left_n, right_n) if both_contact else 0.0
        max_tangent = max(left_t, right_t)
        n_required = max_tangent / (2.0 * MU_EFFECTIVE)

        if phase.startswith("PULL_ARC") and grasp_ref["value"] is None and both_contact:
            grasp_ref["value"] = rpp.tcp_local_cube_offset(tcp_pos, tcp_rot, handle_pos)
        slip_mm = None
        handle_in_tcp_frame = tcp_rot.T @ (handle_pos - tcp_pos)
        if grasp_ref["value"] is not None:
            local_now = rpp.tcp_local_cube_offset(tcp_pos, tcp_rot, handle_pos)
            slip_mm = rpp.relative_slip_m(local_now, grasp_ref["value"]) * 1000.0

        finger_q = gripper_map.get_qpos(data).copy()

        # Arm actuator saturation: applied force vs. each joint's real
        # physical force limit (RIGHT_ARM_JOINT_ACTUATOR_PAIRS' own
        # force_limit, 25 N*m for shoulder/elbow/wrist_roll, 5 N*m for
        # wrist_pitch/yaw -- using a uniform 25 below is a conservative
        # denominator for the wider-limit joints only; reported as a
        # diagnostic ratio, not a pass/fail criterion).
        arm_force = np.array([data.actuator_force[i] for i in arm_map.actuator_id])

        log.append({
            "t": float(data.time), "phase": phase, "carrying": carrying,
            "hinge_deg": float(np.degrees(hinge_qpos)), "hinge_qvel": hinge_qvel,
            "tcp_actual": tcp_pos.tolist(), "tcp_commanded": commanded_tcp.tolist(),
            "tcp_err_mm": tcp_err_mm,
            "orientation_residual_deg": orient_resid_deg,
            "roll_deg": float(rpy_deg[0]), "pitch_deg": float(rpy_deg[1]), "yaw_deg": float(rpy_deg[2]),
            "finger_left_qpos": float(finger_q[0]), "finger_right_qpos": float(finger_q[1]),
            "left_normal_n": left_n, "right_normal_n": right_n,
            "left_tangent_n": left_t, "right_tangent_n": right_t,
            "both_contact": both_contact, "min_bilateral_normal_n": min_normal,
            "max_tangent_n": max_tangent, "n_required_n": n_required,
            "friction_margin_n": min_normal - n_required,
            "handle_in_tcp_frame": handle_in_tcp_frame.tolist(),
            "slip_mm": slip_mm,
            "condition_number": cond, "sigma_min": sigma_min,
            "commanded_q": commanded_q.tolist(), "actual_q": actual_q_after.tolist(),
            "joint_err_max_deg": float(np.degrees(np.max(np.abs(joint_err)))),
            "joint_err_rms_deg": float(np.degrees(np.sqrt(np.mean(joint_err ** 2)))),
            "arm_actuator_force": arm_force.tolist(),
            "arm_force_saturation_frac": float(np.max(np.abs(arm_force) / ARM_FORCE_LIMITS)) if np.all(np.isfinite(arm_force)) else None,
            "gripper_saturated_any": bool(np.any(gripper_diag.get("saturated", [False]))) if gripper_diag else None,
        })

    patches = [(do, "_door_step_once", instrumented_step)]
    if gripper_kp is not None:
        patches.append((do, "GRIPPER_KP_DOOR", gripper_kp))
    if gripper_kd is not None:
        patches.append((do, "GRIPPER_KD_DOOR", gripper_kd))
    if nullspace_posture_gain is not None:
        patches.append((c3c, "NULLSPACE_POSTURE_GAIN", nullspace_posture_gain))
    if orient_weight is not None:
        original_oriented = c3c.solve_ik_waypoint_oriented

        def weighted_oriented(*args, **kwargs):
            kwargs.setdefault("orient_weight", orient_weight)
            return original_oriented(*args, **kwargs)

        patches.append((rpp, "solve_ik_waypoint_oriented", weighted_oriented))

    from contextlib import ExitStack
    with ExitStack() as stack:
        for module, name, value in patches:
            stack.enter_context(_patched(module, name, value))
        result = do.run_trial_door_open(
            scene_path, geometry, initial_hinge_angle_rad=initial_hinge_angle_rad,
            use_oriented_ik=use_oriented_ik, frame_callback=frame_callback,
        )

    return {"result": result, "log": log}


def summarize(out: dict) -> dict:
    log = [l for l in out["log"] if l["phase"].startswith("PULL_ARC")]
    if not log:
        return {"door_pass": out["result"]["door_pass"], "max_slip_mm": None}
    slips = [l["slip_mm"] for l in log if l["slip_mm"] is not None]
    mid = log[len(log) // 2]
    return {
        "door_pass": out["result"]["door_pass"],
        "max_slip_mm": max(slips) if slips else None,
        "slip_at_midpoint_mm": mid.get("slip_mm"),
        "final_hinge_deg": log[-1]["hinge_deg"],
        "contact_retention_frac": sum(1 for l in log if l["both_contact"]) / len(log),
        "max_tcp_err_mm": max(l["tcp_err_mm"] for l in log),
        "max_joint_err_deg": max(l["joint_err_max_deg"] for l in log),
    }
