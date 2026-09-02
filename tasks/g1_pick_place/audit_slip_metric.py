#!/usr/bin/env python3
"""Phase 4C: slip-metric audit harness.

Reruns Phase 4B's exact scene/configuration (unchanged: arm_kp=400.0,
arm_kv=25.0, gripper_kp=150.0, gripper_kd=10.0) through the now-corrected
run_trial_pick_place() and records both the legacy (pre-4C) max_cube_slip_m
figure and the new phase-scoped metrics, for the nominal case (5 reruns,
determinism check, same convention as Phase 3C/4A) and the 3 Stage B
reachable variants (3 reruns each, same convention as Phase 4B). No
controller, gain, or trajectory parameter is changed from the committed
Phase 4B configuration (commit 363aa83) -- this is a measurement/reporting
audit only. Writes logs/phase4c_slip_audit.json.
"""

from __future__ import annotations

import json
from pathlib import Path

from tasks.g1_pick_place.gripper_scene import write_grasp_scene_4b
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B,
    ARM_KV_4B,
    EXCLUDED_UNREACHABLE_VARIANTS,
    STAGE_B_VARIANTS,
    run_trial_pick_place,
)

ROOT = Path(__file__).resolve().parents[2]
SLIP_FIELDS = [
    "max_cube_slip_m",
    "grasp_reference_offset_tcp_frame",
    "max_slip_during_lift",
    "max_slip_during_transport",
    "max_slip_during_lower",
    "slip_at_release",
    "post_release_tcp_cube_separation_m",
]


def _slip_record(r: dict) -> dict:
    return {k: r[k] for k in SLIP_FIELDS}


def main() -> int:
    scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")

    nominal_trials = [run_trial_pick_place(scene) for _ in range(5)]
    nominal_slip_records = [_slip_record(r) for r in nominal_trials]
    nominal_deterministic = len({r["max_cube_slip_m"] for r in nominal_slip_records}) == 1

    variants = {}
    for v in STAGE_B_VARIANTS:
        trials = [run_trial_pick_place(scene, cube_xy_offset=v["offset"]) for _ in range(3)]
        variants[v["id"]] = {
            "offset_m": list(v["offset"]),
            "trials": [_slip_record(r) for r in trials],
            "task_pass": [bool(r["task_pass"]) for r in trials],
            "grasp_pass": [bool(r["grasp_pass"]) for r in trials],
        }

    record = {
        "scope": "fixed-base, torso-constrained upper-body manipulation baseline",
        "purpose": "Phase 4C audit of Phase 4B's max_cube_slip_m metric -- measurement/reporting fix only, no controller or physics change",
        "baseline_commit": "363aa83",
        "shared_configuration_unchanged": {
            "arm_kp": ARM_KP_4B, "arm_kv": ARM_KV_4B,
            "gripper_kp": 150.0, "gripper_kd": 10.0,
        },
        "old_definition": (
            "max over every step from grasp verification (VERIFY_BILATERAL_CONTACT) "
            "to end of trial of ||R_tcp^T (cube_pos - tcp_pos) - grasp_reference_offset||, "
            "with no check that the cube was still being carried -- included "
            "OPEN/RELEASE_SETTLE/VERIFY_RELEASE/RETREAT/VERIFY_TASK_SUCCESS steps, "
            "i.e. genuine post-release TCP-cube separation, mislabeled as grasp slip."
        ),
        "corrected_definition": (
            "slip is now only accumulated while `carrying` is 'grip_only' or 'full' "
            "(i.e. LIFT/HOLD/TRANSPORT_ABOVE_TARGET/SETTLE_ABOVE_TARGET/LOWER_TO_TARGET/"
            "SETTLE_LOWER -- exactly the phases where the gripper is commanded closed and "
            "bilateral contact is being monitored), bucketed into "
            "max_slip_during_lift/transport/lower and slip_at_release (the last value "
            "recorded during SETTLE_LOWER, immediately before OPEN). Post-release motion "
            "is reported separately as post_release_tcp_cube_separation_m and is never "
            "called slip."
        ),
        "nominal": {
            "n_trials": 5,
            "deterministic": nominal_deterministic,
            "legacy_max_cube_slip_m": nominal_slip_records[0]["max_cube_slip_m"],
            "corrected": {
                "grasp_reference_offset_tcp_frame": nominal_slip_records[0]["grasp_reference_offset_tcp_frame"],
                "max_slip_during_lift": nominal_slip_records[0]["max_slip_during_lift"],
                "max_slip_during_transport": nominal_slip_records[0]["max_slip_during_transport"],
                "max_slip_during_lower": nominal_slip_records[0]["max_slip_during_lower"],
                "slip_at_release": nominal_slip_records[0]["slip_at_release"],
                "post_release_tcp_cube_separation_m": nominal_slip_records[0]["post_release_tcp_cube_separation_m"],
            },
            "trials": nominal_slip_records,
        },
        "stage_b_variants": variants,
        "excluded_unreachable_variants": EXCLUDED_UNREACHABLE_VARIANTS,
    }

    out_path = ROOT / "logs" / "phase4c_slip_audit.json"
    out_path.write_text(json.dumps(record, indent=2))
    print(f"wrote {out_path}")
    print(f"nominal legacy max_cube_slip_m = {record['nominal']['legacy_max_cube_slip_m']}")
    print(f"nominal corrected max_slip_during_lower = {record['nominal']['corrected']['max_slip_during_lower']}")
    print(f"nominal post_release_tcp_cube_separation_m = {record['nominal']['corrected']['post_release_tcp_cube_separation_m']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
