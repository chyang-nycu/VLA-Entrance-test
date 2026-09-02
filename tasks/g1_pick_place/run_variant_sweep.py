#!/usr/bin/env python3
"""Phase 4A: grasp setup-variant evaluation.

Fixed-base, torso-constrained upper-body manipulation baseline (Phase 3C's
pelvis+torso-welded G1, bounded position-servo right arm, waypoint IK,
physical parallel gripper). This module does NOT tune anything -- it runs
Phase 3C's single, unchanged winning configuration (ARM_KP_3C/ARM_KV_3C,
GRIPPER_KP_3C/GRIPPER_KD_3C from run_grasp_test_3c.py) against 5
deterministic cube-position variants and records the result honestly.

No per-variant gains, hand-tuned joint targets, offsets, or hidden
exceptions: every variant is driven through the identical
run_trial_3c(scene, gripper_kp=GRIPPER_KP_3C, gripper_kd=GRIPPER_KD_3C,
cube_xy_offset=...) call, one shared scene file. Cube pose is set only
during CubeInitGuard's pre-lock initialization window (enforced by
run_trial_3c/CubeInitGuard, unchanged, reused here as-is).

Simulation is fully deterministic (verified in Phase 3C: 5/5 bit-identical
reruns) -- there is no RNG anywhere in this pipeline, so "fixed seed" is
trivially satisfied. Trials are still repeated (>=3 per variant, per
HANDOFF.md) for auditability, not because run-to-run variance is expected;
any run-to-run difference found here would itself be a notable finding.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import (
    RIGHT_ARM_ACTUATORS,
    RIGHT_ARM_JOINTS,
    TCP_SITE,
    JointMap,
)
from tasks.g1_pick_place.gripper_scene import CUBE_HALF, CUBE_POS, write_grasp_scene_3c
from tasks.g1_pick_place.phase3c_tuning import ARM_KP_3C, ARM_KV_3C
from tasks.g1_pick_place.run_grasp_test_3c import (
    GRIPPER_KD_3C,
    GRIPPER_KP_3C,
    diagnose_reachability,
    run_trial_3c,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"

# Table half-extent (see gripper_scene.py: size="0.22 0.22 ...", centered on
# CUBE_POS's (x, y)) -- used for the table-containment feasibility check.
TABLE_HALF_XY = 0.22
# Conservative clearance kept from the table edge so the cube's own footprint
# (CUBE_HALF) never approaches the table boundary.
TABLE_EDGE_MARGIN = 0.05

N_TRIALS_PER_VARIANT = 3
# A variant "succeeds" if a majority of its trials meet all 5 acceptance
# criteria (>=2 of 3). Stated explicitly and applied uniformly, per
# HANDOFF.md. Since the simulation is deterministic, in practice this
# collapses to "all trials agree" -- the majority rule is the fallback in
# case that assumption is ever violated.
VARIANT_SUCCESS_TRIALS_REQUIRED = 2

# Table-plane axis 1 = x, axis 2 = y (CUBE_POS = (x, y, z) in gripper_scene.py).
# Exactly 5 variants: nominal + one +/-0.03 m pair per axis. 0.03 m is
# HANDOFF.md's default magnitude; verified below (table containment is
# trivial at this magnitude: 0.03 + CUBE_HALF = 0.065 m << TABLE_HALF_XY -
# TABLE_EDGE_MARGIN = 0.17 m) so no deviation from the default was needed.
VARIANTS = [
    {"id": "nominal", "offset": (0.0, 0.0)},
    {"id": "x_plus_0.03", "offset": (0.03, 0.0)},
    {"id": "x_minus_0.03", "offset": (-0.03, 0.0)},
    {"id": "y_plus_0.03", "offset": (0.0, 0.03)},
    {"id": "y_minus_0.03", "offset": (0.0, -0.03)},
]


def _closing_axis_world(model: mujoco.MjModel, data: mujoco.MjData, site_id: int) -> np.ndarray:
    """World-frame direction of the TCP site's local Y axis -- the fingers'
    closing/separation axis (see gripper_scene.py: each finger is offset
    +/-FINGER_OPEN_Y along the wrist-attached body's local Y). Read directly
    from site_xmat's second column; caller must have already run
    mj_kinematics/mj_comPos at the configuration of interest.
    """
    xmat = data.site_xmat[site_id].reshape(3, 3)
    return xmat[:, 1].copy()


def _table_containment(cube_xy: np.ndarray) -> dict:
    dx = abs(cube_xy[0] - CUBE_POS[0])
    dy = abs(cube_xy[1] - CUBE_POS[1])
    limit = TABLE_HALF_XY - TABLE_EDGE_MARGIN - CUBE_HALF
    contained = bool(dx <= limit and dy <= limit)
    return {
        "cube_xy": cube_xy.tolist(),
        "table_center_xy": [CUBE_POS[0], CUBE_POS[1]],
        "table_half_extent_m": TABLE_HALF_XY,
        "edge_margin_m": TABLE_EDGE_MARGIN,
        "max_allowed_offset_m": limit,
        "actual_offset_m": [float(dx), float(dy)],
        "contained": contained,
    }


def feasibility_check(
    model: mujoco.MjModel,
    arm_map: JointMap,
    site_id: int,
    base_qpos: np.ndarray,
    cube_pos: np.ndarray,
    nominal_closing_axis: np.ndarray | None,
) -> dict:
    """Pre-run feasibility check for one variant: table containment, PREGRASP
    and APPROACH IK residuals, joint-limit margin, and expected TCP/finger-pad
    alignment relative to the nominal variant's approach configuration.
    Recorded for every variant regardless of the outcome -- never silently
    skipped.
    """
    reach = diagnose_reachability(model, arm_map, site_id, base_qpos, cube_pos)
    containment = _table_containment(np.array([cube_pos[0], cube_pos[1]]))

    scratch = mujoco.MjData(model)
    q_approach = np.array(reach["APPROACH"]["solved_joint_target"])
    scratch.qpos[:] = base_qpos
    arm_map.set_qpos(scratch, q_approach)
    mujoco.mj_kinematics(model, scratch)
    mujoco.mj_comPos(model, scratch)
    closing_axis = _closing_axis_world(model, scratch, site_id)

    lo = arm_map.jnt_range[:, 0]
    hi = arm_map.jnt_range[:, 1]
    span = np.maximum(hi - lo, 1e-9)
    margin_frac = np.minimum(q_approach - lo, hi - q_approach) / span
    joint_limit_ok = bool(np.all(margin_frac > 0.02))  # >2% of range from either bound

    alignment: dict = {"closing_axis_world": closing_axis.tolist()}
    if nominal_closing_axis is not None:
        cosang = float(np.clip(np.dot(closing_axis, nominal_closing_axis), -1.0, 1.0))
        angle_deg = float(np.degrees(np.arccos(cosang)))
        alignment["angle_from_nominal_deg"] = angle_deg
        # 15 deg is a conservative bound: FINGER_OPEN_Y - CUBE_HALF - pad
        # half-thickness leaves ~34 mm of physical clearance (see
        # run_grasp_test_3c.py's GRASP_CORRIDOR_XY_M comment); a 15 deg
        # closing-axis tilt over the ~9-12 cm arm-to-cube reach displaces
        # each pad's contact point by well under that margin.
        alignment["within_tolerance_15deg"] = bool(angle_deg <= 15.0)
    else:
        alignment["angle_from_nominal_deg"] = 0.0
        alignment["within_tolerance_15deg"] = True

    reachable = bool(reach["all_reachable"])
    accepted = bool(
        reachable
        and containment["contained"]
        and joint_limit_ok
        and alignment["within_tolerance_15deg"]
    )

    return {
        "reachability": reach,
        "table_containment": containment,
        "joint_limit_margin_frac": margin_frac.tolist(),
        "joint_limit_ok": joint_limit_ok,
        "tcp_finger_pad_alignment": alignment,
        "accepted_as_reachable": accepted,
    }


def run_sweep(
    arm_kp: float = ARM_KP_3C,
    arm_kv: float = ARM_KV_3C,
    gripper_kp: float = GRIPPER_KP_3C,
    gripper_kd: float = GRIPPER_KD_3C,
    n_trials: int = N_TRIALS_PER_VARIANT,
    label: str = "sweep",
) -> dict:
    """Run the full 5-variant sweep once, under one shared configuration.
    gripper_kp/kd are threaded through to run_variant/run_trial_3c so a
    global (not per-variant) adjustment can be evaluated by calling this
    again with different values -- still one call, one config, for all 5
    variants.
    """
    scene = write_grasp_scene_3c(arm_kp=arm_kp, arm_kv=arm_kv, scene_name="g1_grasp_scene_3c.xml")

    # nominal_closing_axis is computed once (nominal variant, offset 0,0)
    # and reused as the alignment reference for every variant.
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    nominal_reach = diagnose_reachability(model, arm_map, site_id, data.qpos.copy(), np.array(CUBE_POS))
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos.copy()
    arm_map.set_qpos(scratch, np.array(nominal_reach["APPROACH"]["solved_joint_target"]))
    mujoco.mj_kinematics(model, scratch)
    mujoco.mj_comPos(model, scratch)
    nominal_closing_axis = _closing_axis_world(model, scratch, site_id)

    variants_out = []
    for v in VARIANTS:
        nca = None if v["id"] == "nominal" else nominal_closing_axis
        variants_out.append(
            run_variant_with_gains(scene, v, n_trials, nca, gripper_kp, gripper_kd)
        )

    n_variant_success = sum(1 for v in variants_out if v["variant_success"])
    total_trials = sum(v["n_trials"] for v in variants_out)
    total_pass = sum(v["n_pass"] for v in variants_out)

    return {
        "label": label,
        "scene": str(scene.relative_to(ROOT)),
        "parameters": {"arm_kp": arm_kp, "arm_kv": arm_kv, "gripper_kp": gripper_kp, "gripper_kd": gripper_kd},
        "variants": variants_out,
        "n_variants_succeeded": n_variant_success,
        "n_variants_total": len(VARIANTS),
        "variant_success_rate": n_variant_success / len(VARIANTS),
        "total_trials": total_trials,
        "total_trials_passed": total_pass,
        "trial_success_rate": total_pass / total_trials,
        "target_met_3_of_5": n_variant_success >= 3,
    }


def run_variant_with_gains(scene_path, variant, n_trials, nominal_closing_axis, gripper_kp, gripper_kd):
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)

    offset = variant["offset"]
    cube_pos = np.array([CUBE_POS[0] + offset[0], CUBE_POS[1] + offset[1], CUBE_POS[2]])
    feasibility = feasibility_check(model, arm_map, site_id, data.qpos.copy(), cube_pos, nominal_closing_axis)

    trials = []
    for i in range(n_trials):
        result = run_trial_3c(scene_path, cube_xy_offset=offset, gripper_kp=gripper_kp, gripper_kd=gripper_kd)
        pre_close_displacement = None
        if result.get("cube_xy_before_lift") is not None:
            spawn_xy = np.array(result["cube_spawn_pos"][:2])
            pre_close_displacement = float(np.linalg.norm(np.array(result["cube_xy_before_lift"]) - spawn_xy))
        trials.append(
            {
                "trial_index": i,
                "pass": result["pass"],
                "criteria": result["criteria"],
                "bilateral_contact": bool(result["left_contact_ever"] and result["right_contact_ever"]),
                "height_gain_m": result["height_gain_m"],
                "max_continuous_lifted_s": result["max_continuous_lifted_s"],
                "pre_close_cube_displacement_m": pre_close_displacement,
                "tcp_tracking_error_rms": result["arm_tracking_error_rms"],
                "tcp_tracking_error_max": result["arm_tracking_error_max"],
                "arm_force_saturation": result["arm_force_saturation"],
                "failure_state": result["failure_state"],
                "failure_reason": result["failure_reason"],
            }
        )

    n_pass = sum(1 for t in trials if t["pass"])
    n_bilateral = sum(1 for t in trials if t["bilateral_contact"])
    variant_success = n_pass >= VARIANT_SUCCESS_TRIALS_REQUIRED

    return {
        "id": variant["id"],
        "offset_m": list(offset),
        "cube_pose": cube_pos.tolist(),
        "feasibility": feasibility,
        "trials": trials,
        "n_trials": n_trials,
        "n_pass": n_pass,
        "bilateral_contact_rate": n_bilateral / n_trials,
        "max_height_gain_m": max(t["height_gain_m"] for t in trials),
        "max_continuous_hold_s": max(t["max_continuous_lifted_s"] for t in trials),
        "variant_success": variant_success,
    }


def main() -> int:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    record: dict = {"scope": "fixed-base, torso-constrained upper-body manipulation baseline"}

    sweep0 = run_sweep(label="sweep_0_unmodified_phase3c_config")
    sweeps = [sweep0]

    global_adjustments: list = []
    if sweep0["n_variants_succeeded"] < 3:
        # Placeholder path: only entered if the unmodified Phase 3C config
        # fails the 3/5 target. See run_variant_sweep's caller for the
        # actual evidence-driven adjustment(s) applied, if any were needed.
        pass

    record["sweeps"] = sweeps
    record["global_adjustments"] = global_adjustments
    record["final_sweep"] = sweeps[-1]["label"]

    (LOG_DIR / "phase4a_grasp_variants.json").write_text(
        json.dumps(record, indent=2, default=str) + "\n", encoding="utf-8"
    )
    print(json.dumps({k: v for k, v in sweeps[-1].items() if k != "variants"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
