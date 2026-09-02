#!/usr/bin/env python3
"""Phase 3B: controller stabilization budget (HANDOFF.md).

Runs, in order, stopping as soon as one passes:
  0. Baseline instrumentation of the existing (unmodified) Phase 3 controller.
  1. Attempt 3B-1: per-joint gain scaling by actuator torque authority.
  2. Attempt 3B-2 (only if 3B-1 fails): trajectory smoothing + settle-before-close.
  3. Attempt 3B-3 (only if 3B-2 fails): one bounded adjustment from evidence.

Writes every attempt's full metrics (not just the winner) to
logs/phase3b_controller_tuning.json.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place import controller as ctrl_mod
from tasks.g1_pick_place.controller import G1GraspController, compute_per_joint_gains
from tasks.g1_pick_place.gripper_scene import write_grasp_scene
from tasks.g1_pick_place.run_grasp_test import run_trial

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"


def resolve_ctrl_to_torque_mapping(model: mujoco.MjModel) -> dict:
    """Verify (not assume) that ctrl units equal joint N*m torque for the
    right-arm motors: gear must be [1,0,0,0,0,0] and the transmission must
    be a plain joint (not tendon/site), per motor."""
    rows = []
    all_1to1 = True
    for jname, aname in zip(ctrl_mod.RIGHT_ARM_JOINTS, ctrl_mod.RIGHT_ARM_ACTUATORS):
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
        gear = model.actuator_gear[aid].tolist()
        trntype = int(model.actuator_trntype[aid])
        is_1to1 = (
            trntype == int(mujoco.mjtTrn.mjTRN_JOINT)
            and gear[0] == 1.0
            and all(g == 0.0 for g in gear[1:])
        )
        all_1to1 = all_1to1 and is_1to1
        rows.append(
            {
                "joint": jname,
                "actuator": aname,
                "gear": gear,
                "trntype": trntype,
                "ctrl_equals_joint_torque_1to1": is_1to1,
            }
        )
    return {"per_joint": rows, "ctrl_equals_joint_torque_for_all_arm_joints": all_1to1}


def joint_saturation_and_error_stats(diagnostics: dict, joint_names: list[str]) -> dict:
    """Aggregate per-joint saturation fraction and RMS/max tracking error
    across every recorded phase in `diagnostics`."""
    n_joints = len(joint_names)
    sat_count = np.zeros(n_joints)
    n_steps = 0
    joint_err_sq_sum = np.zeros(n_joints)
    joint_err_max = np.zeros(n_joints)
    tcp_err_sq_sum = 0.0
    tcp_err_max = 0.0
    ff_over_limit_count = np.zeros(n_joints)

    for phase, bucket in diagnostics.items():
        sat = np.array(bucket["saturated"])  # (steps, n_joints)
        if sat.size == 0:
            continue
        n_steps += sat.shape[0]
        sat_count += sat.sum(axis=0)
        jerr = np.array(bucket["joint_error"])
        joint_err_sq_sum += (jerr ** 2).sum(axis=0)
        joint_err_max = np.maximum(joint_err_max, np.abs(jerr).max(axis=0))
        tcp_err = np.array(bucket["tcp_err"])
        tcp_err_norms = np.linalg.norm(tcp_err, axis=1)
        tcp_err_sq_sum += float((tcp_err_norms ** 2).sum())
        tcp_err_max = max(tcp_err_max, float(tcp_err_norms.max()))

    n_steps = max(n_steps, 1)
    return {
        "n_steps_recorded": int(n_steps),
        "per_joint_saturation_fraction": {
            name: float(sat_count[i] / n_steps) for i, name in enumerate(joint_names)
        },
        "per_joint_rms_tracking_error_rad": {
            name: float(np.sqrt(joint_err_sq_sum[i] / n_steps)) for i, name in enumerate(joint_names)
        },
        "per_joint_max_tracking_error_rad": {
            name: float(joint_err_max[i]) for i, name in enumerate(joint_names)
        },
        "tcp_rms_error_m": float(np.sqrt(tcp_err_sq_sum / n_steps)),
        "tcp_max_error_m": float(tcp_err_max),
    }


def gravity_bias_fraction(model: mujoco.MjModel, ctrl: G1GraspController) -> dict:
    """At a representative pregrasp-ish pose, what fraction of each joint's
    torque authority is consumed by gravity/Coriolis feedforward alone?"""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # Bend the arm toward a representative reach pose (same family used in
    # Phase 2's workspace probe) so gravity loading is non-trivial, then
    # settle briefly under gravity compensation only (kp=kd=0 equivalent:
    # we just read qfrc_bias directly, no need to simulate).
    ff = data.qfrc_bias[ctrl.arm_map.dof_adr]
    limits = ctrl.arm_map.ctrl_range[:, 1]
    return {
        name: float(abs(ff[i]) / limits[i]) for i, name in enumerate(ctrl.arm_map.names)
    }


def summarize_result(label: str, extra: dict, result: dict, diag: dict | None) -> dict:
    entry = {
        "label": label,
        **extra,
        "trajectory_duration_s": None,
        "height_gain_m": result["height_gain_m"],
        "max_continuous_lifted_s": result["max_continuous_lifted_s"],
        "cube_xy_displacement_before_close_m": result["cube_xy_displacement_before_close_m"],
        "left_contact_ever": result["left_contact_ever"],
        "right_contact_ever": result["right_contact_ever"],
        "criteria": result["criteria"],
        "pass": result["pass"],
    }
    if diag is not None:
        entry["instrumentation"] = joint_saturation_and_error_stats(diag, ctrl_mod.RIGHT_ARM_JOINTS)
    return entry


def main() -> int:
    scene = write_grasp_scene()
    model = mujoco.MjModel.from_xml_path(str(scene))
    log = {"scene": str(scene.relative_to(ROOT))}

    # --- pre-3B-1: resolve ctrl->torque mapping, must not assume it ---
    mapping = resolve_ctrl_to_torque_mapping(model)
    log["ctrl_to_torque_mapping"] = mapping
    if not mapping["ctrl_equals_joint_torque_for_all_arm_joints"]:
        raise RuntimeError(
            "Cannot apply torque_limit_i/25 gain scaling: ctrl is not 1:1 joint "
            f"torque for all right-arm joints: {mapping}"
        )

    baseline_ctrl = G1GraspController(model=model)  # exact Phase 3 defaults
    log["gravity_bias_fraction_of_torque_limit_at_forward_pose"] = gravity_bias_fraction(model, baseline_ctrl)

    # --- baseline: existing (failed) Phase 3 controller, instrumented ---
    baseline_diag: dict = {}
    baseline_result = run_trial(scene, cube_xy_offset=(0.0, 0.0), controller=baseline_ctrl, diagnostics=baseline_diag)
    log["baseline"] = summarize_result(
        "baseline (Phase 3, uniform Kp=180/Kd=18)",
        {"arm_kp": float(ctrl_mod.ARM_KP), "arm_kd": float(ctrl_mod.ARM_KD)},
        baseline_result,
        baseline_diag,
    )
    print("baseline:", json.dumps({"pass": baseline_result["pass"], "criteria": baseline_result["criteria"]}))

    attempts = []

    # --- Attempt 3B-1: per-joint gain scaling by torque authority ---
    per_joint_kp, per_joint_kd = compute_per_joint_gains(baseline_ctrl.arm_map.ctrl_range)
    attempt1_ctrl = G1GraspController(model=model, arm_kp=per_joint_kp, arm_kd=per_joint_kd)
    diag1: dict = {}
    result1 = run_trial(scene, cube_xy_offset=(0.0, 0.0), controller=attempt1_ctrl, diagnostics=diag1)
    entry1 = summarize_result(
        "3B-1 (per-joint Kp/Kd scaled by torque authority)",
        {
            "arm_kp": {n: float(v) for n, v in zip(ctrl_mod.RIGHT_ARM_JOINTS, per_joint_kp)},
            "arm_kd": {n: float(v) for n, v in zip(ctrl_mod.RIGHT_ARM_JOINTS, per_joint_kd)},
        },
        result1,
        diag1,
    )
    attempts.append(entry1)
    print("3B-1:", json.dumps({"pass": result1["pass"], "criteria": result1["criteria"]}))

    winner = None
    if result1["pass"]:
        winner = entry1
        winner["controller_kwargs"] = {"arm_kp": per_joint_kp.tolist(), "arm_kd": per_joint_kd.tolist()}

    # --- Attempt 3B-2: trajectory smoothing + settle-before-close (only if needed) ---
    if winner is None:
        settle_pos_tol = 0.008
        settle_vel_tol = 0.15
        max_settle_extra_s = 1.5
        attempt2_ctrl = G1GraspController(model=model, arm_kp=per_joint_kp, arm_kd=per_joint_kd)
        diag2: dict = {}
        result2 = run_trial(
            scene, cube_xy_offset=(0.0, 0.0), controller=attempt2_ctrl, diagnostics=diag2,
            settle_before_close=True, settle_pos_tol=settle_pos_tol,
            settle_vel_tol=settle_vel_tol, max_settle_extra_s=max_settle_extra_s,
        )
        entry2 = summarize_result(
            "3B-2 (3B-1 gains + settle-before-close gating)",
            {
                "arm_kp": {n: float(v) for n, v in zip(ctrl_mod.RIGHT_ARM_JOINTS, per_joint_kp)},
                "arm_kd": {n: float(v) for n, v in zip(ctrl_mod.RIGHT_ARM_JOINTS, per_joint_kd)},
                "settle_pos_tol_m": settle_pos_tol,
                "settle_vel_tol_rad_s": settle_vel_tol,
                "max_settle_extra_s": max_settle_extra_s,
                "settle_extra_s_used": result2["settle_extra_s"],
            },
            result2,
            diag2,
        )
        attempts.append(entry2)
        print("3B-2:", json.dumps({"pass": result2["pass"], "criteria": result2["criteria"]}))
        if result2["pass"]:
            winner = entry2
            winner["controller_kwargs"] = {"arm_kp": per_joint_kp.tolist(), "arm_kd": per_joint_kd.tolist()}
            winner["settle_kwargs"] = {
                "settle_before_close": True,
                "settle_pos_tol": settle_pos_tol,
                "settle_vel_tol": settle_vel_tol,
                "max_settle_extra_s": max_settle_extra_s,
            }

    # --- Attempt 3B-3: torque-weighted DLS-IK, baseline PD gains (only if needed) ---
    # Evidence from 3B-1/3B-2 (see log["attempts"] entries 0-1): softening
    # wrist_pitch/yaw's PD gains reduced *their* saturation as intended, but
    # the torque-agnostic IK reassigned kinematic load onto shoulder/elbow
    # (e.g. shoulder_yaw saturation 5.6% -> 35.4%, RMS tracking error
    # roughly 5-6x across the board, TCP RMS error 0.061m -> 0.215m) --
    # PD-side gain tuning alone cannot fix a coupling problem that
    # originates in the IK. This attempt reverts to baseline PD gains
    # (proven not to overload shoulder/elbow) and instead weights the DLS
    # solve by each joint's torque limit (normalized to the strongest
    # joint), directly discouraging the IK from leaning on low-torque
    # wrist_pitch/yaw joints to close Cartesian error.
    if winner is None:
        torque_limits = baseline_ctrl.arm_map.ctrl_range[:, 1]
        dls_weights = torque_limits / torque_limits.max()
        attempt3_ctrl = G1GraspController(
            model=model,
            arm_kp=ctrl_mod.ARM_KP,
            arm_kd=ctrl_mod.ARM_KD,
            arm_dls_weights=dls_weights,
        )
        diag3: dict = {}
        result3 = run_trial(scene, cube_xy_offset=(0.0, 0.0), controller=attempt3_ctrl, diagnostics=diag3)
        entry3 = summarize_result(
            "3B-3 (baseline uniform Kp/Kd + torque-weighted DLS-IK)",
            {
                "arm_kp": float(ctrl_mod.ARM_KP),
                "arm_kd": float(ctrl_mod.ARM_KD),
                "dls_weights": {n: float(w) for n, w in zip(ctrl_mod.RIGHT_ARM_JOINTS, dls_weights)},
            },
            result3,
            diag3,
        )
        attempts.append(entry3)
        print("3B-3:", json.dumps({"pass": result3["pass"], "criteria": result3["criteria"]}))
        if result3["pass"]:
            winner = entry3
            winner["controller_kwargs"] = {
                "arm_kp": float(ctrl_mod.ARM_KP),
                "arm_kd": float(ctrl_mod.ARM_KD),
                "arm_dls_weights": dls_weights.tolist(),
            }

    log["attempts"] = attempts
    log["winner"] = winner["label"] if winner else None
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase3b_controller_tuning.json").write_text(json.dumps(log, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"winner": log["winner"]}, indent=2))
    return 0 if winner else 1


if __name__ == "__main__":
    raise SystemExit(main())
