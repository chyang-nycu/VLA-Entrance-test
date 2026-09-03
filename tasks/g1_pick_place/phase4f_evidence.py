#!/usr/bin/env python3
"""Phase 4F evidence harness: runs the final (Attempt 3) configuration's
determinism check and the informational Stage B variant sweep, and writes
logs/phase4f_orientation_grasp.json. The 3-attempt tuning history itself
(Attempt 1: null-space orientation objective; Attempt 2: increased
orientation weight; Attempt 3: measured finger-pad mounting correction) is
recorded here from the real runs already captured during development (see
reports/phase4f-orientation-grasp-stabilization.md for the full narrative
and evidence behind each number) -- this script re-verifies the FINAL
configuration fresh, rather than re-deriving the two superseded attempts
(Attempt 2 in particular left the arm unable to grasp at all, so re-running
it here would only reproduce a documented dead end).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tasks.g1_pick_place import gripper_scene as gs  # noqa: E402
from tasks.g1_pick_place import run_pick_place as rp  # noqa: E402
from tasks.g1_pick_place.controller_3c import ORIENT_TOL_RAD, ORIENT_WEIGHT, FINGER_CONTACT_Y  # noqa: E402

LOG_PATH = ROOT / "logs" / "phase4f_orientation_grasp.json"


def _trial_summary(r: dict) -> dict:
    return {
        "task_pass": r["task_pass"],
        "grasp_pass": r["grasp_pass"],
        "placement_pass": r["placement_pass"],
        "failure_state": r["failure_state"],
        "failure_reason": r["failure_reason"],
        "height_gain_m": r["height_gain_m"],
        "max_slip_while_grasped_m": r["max_slip_while_grasped_m"],
        "downward_slip_during_hold_m": r["downward_slip_during_hold_m"],
        "downward_slip_during_transport_m": r["downward_slip_during_transport_m"],
        "orientation_residual_at_approach_deg": r["orientation_residual_at_approach_deg"],
        "lateral_centering_error_at_approach_m": r["lateral_centering_error_at_approach_m"],
        "max_abs_contact_z_offset_from_cube_center_m": r["max_abs_contact_z_offset_from_cube_center_m"],
        "min_bilateral_normal_force_n": r["min_bilateral_normal_force_n"],
        "opposing_face_contact_left": r["opposing_face_contact_left"],
        "opposing_face_contact_right": r["opposing_face_contact_right"],
        "criteria_grasp_stability_4f": r["criteria_grasp_stability_4f"],
        "grasp_stability_pass_4f": r["grasp_stability_pass_4f"],
        "final_xy_target_error_m": r["final_xy_target_error_m"],
        "task_success_dwell_achieved_s": r["task_success_dwell_achieved_s"],
    }


def main() -> None:
    scene = gs.write_grasp_scene_4f(arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml")

    out = {
        "scope": "Phase 4F orientation-constrained grasp stabilization -- final configuration evidence",
        "orientation_ik_design": {
            "required_axis_local": "wrist local Z (finger-pad 'tall' axis)",
            "desired_axis_world": "+Z (vertical)",
            "orient_tol_rad": ORIENT_TOL_RAD,
            "orient_tol_deg": float(__import__("numpy").degrees(ORIENT_TOL_RAD)),
            "orient_weight_final": ORIENT_WEIGHT,
            "finger_contact_y_m": FINGER_CONTACT_Y,
            "priority": "position solved via unweighted primary Jacobian pinv (unchanged from Phase 3C); "
            "orientation is a null-space secondary objective, so it can only consume redundancy "
            "the position task does not need -- position accuracy is never traded for orientation.",
        },
        "attempts": [
            {
                "attempt": 1,
                "change": "Added solve_ik_waypoint_oriented(): null-space orientation objective, "
                "orient_weight=0.6, aligning wrist local Z to world vertical.",
                "evidence": {
                    "orientation_residual_at_approach_deg_before": 47.39,
                    "orientation_residual_at_approach_deg_after": 44.48,
                    "max_slip_while_grasped_m": 0.02193,
                    "result": "Real full-trial rerun: FAILED at SETTLE_LOWER. Orientation residual improved "
                    "only marginally (47.4 -> 44.5 deg); measured slip did not improve over Phase 4E's "
                    "0.0205 m baseline.",
                },
            },
            {
                "attempt": 2,
                "change": "Increased orient_weight from 0.6 to 2.0 (same null-space mechanism).",
                "evidence": {
                    "isolated_ik_sweep": {
                        "orient_weight_0.6": {"pos_resid_mm": 7.44, "orient_resid_deg": 44.45},
                        "orient_weight_2.0": {"pos_resid_mm": 8.30, "orient_resid_deg": 41.19},
                        "orient_weight_5.0": {"pos_resid_mm": 10.48, "orient_resid_deg": 36.71},
                        "orient_weight_10.0": {"pos_resid_mm": 13.68, "orient_resid_deg": 32.80},
                    },
                    "co_primary_stacked_dls_sweep": {
                        "note": "Separate diagnostic: orientation stacked as a CO-PRIMARY task "
                        "(not null-space-secondary) at increasing relative weight, to test whether "
                        "more aggressive weighting could reach the ORIENT_TOL_RAD bar at all.",
                        "worient_0.1": {"pos_resid_mm": 32.15, "orient_resid_deg": 17.87},
                        "worient_0.3": {"pos_resid_mm": 57.80, "orient_resid_deg": 4.57},
                        "worient_1.0": {"pos_resid_mm": 66.72, "orient_resid_deg": 0.53},
                    },
                    "real_full_trial_at_orient_weight_2.0": "FAILED at SETTLE_APPROACH -- TCP never settled "
                    "within tolerance before CLOSE (task_pass=False, height_gain_m=0.0, no grasp attempted "
                    "at all).",
                    "result": "Increasing orientation weight in EITHER form (null-space or co-primary) "
                    "only reduces orientation residual by sacrificing position accuracy -- reaching "
                    "ORIENT_TOL_RAD (~7 deg) requires 30-70 mm of TCP position error, i.e. missing the "
                    "cube. This is a genuine kinematic reachability conflict at the APPROACH waypoint "
                    "(consistent with Phase 3C's own documented wrist singularity there), not a tuning "
                    "problem. REVERTED to orient_weight=0.6 (Attempt 1's value) for Attempt 3.",
                },
            },
            {
                "attempt": 3,
                "change": "Measured finger-pad mounting correction: rotate each finger BODY (not its "
                "position) by a fixed, measured quaternion about the wrist's own local Y (jaw) axis, "
                "so the pad's local Z ('tall') axis is levelled to within ~4 deg of world vertical at "
                "the real, converged nominal APPROACH configuration -- see gripper_scene.py's "
                "FINGER_MOUNT_FIX_QUAT for the full derivation. Root-cause refinement over Attempts "
                "1-2: the wrist's local Y (jaw) axis was ALREADY well-aligned to world Y (~4-5 deg "
                "off) at APPROACH -- left/right finger height symmetry was never the actual defect. "
                "The wrist's local Z axis was ~47 deg off vertical because the arm reaches down to "
                "the cube at a steep diagonal; a pad box tilted that much contacts the cube's vertical "
                "face at a CORNER, not flush across its face -- a small, unstable contact patch that "
                "explains the reported near-drop better than a simple height mismatch.",
                "evidence": {
                    "max_abs_contact_z_offset_from_cube_center_m_before": 0.04376,
                    "max_abs_contact_z_offset_from_cube_center_m_after": 0.03654,
                    "max_slip_while_grasped_m_before": 0.02050,
                    "max_slip_while_grasped_m_after": 0.02592,
                    "result": "Real full-trial rerun: the pad-mount fix reduced the targeted contact "
                    "z-offset metric by ~17% (43.8mm -> 36.5mm), consistent with the corner-vs-flush-"
                    "face hypothesis, but did NOT reduce overall measured slip (which rose slightly, "
                    "25.9mm vs 20.5mm) -- slip during this grasp is evidently also driven by dynamic "
                    "effects during CLOSE/LIFT (impact, settling, friction transients), not solely by "
                    "static contact-patch geometry. FAILED at SETTLE_LOWER, same state as Attempt 1.",
                },
            },
        ],
        "attempt_budget_exhausted": True,
        "final_configuration": {
            "orient_weight": ORIENT_WEIGHT,
            "pad_mount_fix_applied": True,
            "note": "Attempt 3's configuration is carried forward as final: it is the only attempt that "
            "improved a real physical measurement (contact z-offset) without regressing the trial to an "
            "earlier failure point than Attempt 1's baseline, even though it does not clear the "
            "tightened acceptance bar.",
        },
    }

    print("Running final-configuration nominal trial (5x determinism check)...")
    reruns = []
    for i in range(5):
        r = rp.run_trial_pick_place(scene, use_oriented_ik=True)
        reruns.append(_trial_summary(r))
        print(f"  rerun {i}: task_pass={r['task_pass']} max_slip={r['max_slip_while_grasped_m']:.5f} "
              f"failure_state={r['failure_state']}")
    out["final_configuration_nominal_5x"] = reruns
    out["deterministic"] = len({json.dumps(r, sort_keys=True) for r in reruns}) == 1

    print("Running Stage B (informational, 3 Phase-4A-reachable variants)...")
    stage_b = {}
    for variant_id, offset in (("nominal", (0.0, 0.0)), ("x_minus_0.03", (-0.03, 0.0)), ("y_plus_0.03", (0.0, 0.03))):
        r = rp.run_trial_pick_place(scene, cube_xy_offset=offset, use_oriented_ik=True)
        stage_b[variant_id] = _trial_summary(r)
        print(f"  {variant_id}: task_pass={r['task_pass']} max_slip={r['max_slip_while_grasped_m']:.5f} "
              f"failure_state={r['failure_state']}")
    out["stage_b_informational_variants"] = stage_b
    out["stage_b_note"] = (
        "Informational only, per HANDOFF.md Section D ('Only afterward rerun the three supported "
        "Phase 4A variants... First require: nominal stable grasp 5/5... max slip <=10mm in every run') "
        "-- Stage A's gate was NOT met (max slip while grasped exceeds 10mm on the final configuration), "
        "so this Stage B run is diagnostic evidence, not an authorized pass/fail evaluation."
    )

    LOG_PATH.write_text(json.dumps(out, indent=2))
    print(f"wrote {LOG_PATH}")


if __name__ == "__main__":
    main()
