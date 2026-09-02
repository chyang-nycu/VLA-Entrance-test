#!/usr/bin/env python3
"""Runs the Phase 3C evidence-driven attempt sequence and writes
logs/phase3c_attempts.json. Not part of the runtime grasp path (that's
run_grasp_test_3c.run_trial_3c) -- this is the record-keeping harness,
mirroring phase3b_tuning.py's role for Phase 3B.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import JointMap, RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE
from tasks.g1_pick_place.controller import GRIPPER_KP as GRIPPER_KP_PHASE3, GRIPPER_KD as GRIPPER_KD_PHASE3
from tasks.g1_pick_place.gripper_scene import CUBE_POS, write_grasp_scene_3c
from tasks.g1_pick_place.run_grasp_test_3c import (
    GRIPPER_KD_3C,
    GRIPPER_KP_3C,
    diagnose_reachability,
    run_trial_3c,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"

ARM_KP_3C = 400.0
ARM_KV_3C = 25.0


def _reachability_record() -> dict:
    scene = write_grasp_scene_3c(arm_kp=ARM_KP_3C, arm_kv=ARM_KV_3C, scene_name="g1_grasp_scene_3c.xml")
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    return diagnose_reachability(model, arm_map, site_id, data.qpos.copy(), np.array(CUBE_POS))


def main() -> int:
    record: dict = {"scene": "tasks/g1_pick_place/g1_grasp_scene_3c.xml"}

    record["reachability_diagnosis"] = _reachability_record()

    record["pre_attempt_findings"] = [
        {
            "finding": "torso/waist joints unpowered and unconstrained",
            "evidence": (
                "With only the pelvis equality-weld (Phase 3/3B's constraint), "
                "commanding the right arm toward a fixed joint target produced "
                "unbounded TCP position error growth (0.06 m -> 0.50 m over "
                "4.4 s) even though the tracked arm joints' own position error "
                "stayed small (<0.09 rad); left_elbow_joint (fully unactuated) "
                "reached 5.8 rad/s, and torso_link was rotating relative to "
                "the welded pelvis."
            ),
            "fix": "Added a second equality weld, torso_link to pelvis, in write_grasp_scene_3c only (write_grasp_scene()'s output is unaffected, verified byte-identical via sha256).",
        },
        {
            "finding": "explicit Euler integration unstable with stiff position-servo gains",
            "evidence": (
                "After the trunk weld, most right-arm joints still stayed "
                "permanently saturated in an oscillating limit cycle across a "
                "wide (kp, kv) sweep, despite the actual gravity/Coriolis "
                "torque needed at the target pose being small (<3.1 N*m, well "
                "under every joint's force limit) -- a numerical, not "
                "physical, instability. The vendor model has no <option> "
                "element, so MuJoCo defaulted to explicit Euler at dt=0.002s."
            ),
            "fix": "Set integrator=\"implicitfast\" in write_grasp_scene_3c's <option> element (MuJoCo's own documented recommendation for damped position/velocity actuators). No force limit, gain, or physical parameter changed.",
        },
    ]

    attempts = []

    # --- Attempt 3C-1: first working implementation ---
    scene = write_grasp_scene_3c(arm_kp=ARM_KP_3C, arm_kv=ARM_KV_3C, scene_name="g1_grasp_scene_3c.xml")
    r1 = run_trial_3c(scene, gripper_kp=GRIPPER_KP_PHASE3, gripper_kd=GRIPPER_KD_PHASE3)
    attempts.append({
        "attempt": "3C-1",
        "description": "position-priority IK + bounded position servos, first working implementation",
        "parameters": {
            "arm_kp": ARM_KP_3C, "arm_kv": ARM_KV_3C,
            "gripper_kp": GRIPPER_KP_PHASE3, "gripper_kd": GRIPPER_KD_PHASE3,
            "note": "gripper gains are Phase 3/3B's historical defaults, unchanged for this first attempt",
        },
        "result": {k: v for k, v in r1.items() if k != "reachability"},
        "failure_state": r1["failure_state"],
        "failure_reason": r1["failure_reason"],
        "outcome": "PASS" if r1["pass"] else "FAIL",
        "analysis": (
            "Full state machine reached DONE (no gating failure) -- bilateral "
            "contact achieved, cube lifted 0.0497 m and held continuously for "
            "0.198 s, then grip was lost and the cube settled back to the "
            "table. height_gain_ge_0_08m and lifted_ge_2s_continuous both "
            "failed. Direct trace (see report) shows both finger pads lose "
            "contact partway through LIFT, well before the 2.5 s hold window "
            "ends."
        ),
    })

    # --- Attempt 3C-2: gains-only adjustment ---
    r2 = run_trial_3c(scene, gripper_kp=GRIPPER_KP_3C, gripper_kd=GRIPPER_KD_3C)
    determinism_runs = [
        run_trial_3c(scene, gripper_kp=GRIPPER_KP_3C, gripper_kd=GRIPPER_KD_3C) for _ in range(5)
    ]
    attempts.append({
        "attempt": "3C-2",
        "description": "servo gains/damping only -- gripper PD gain increased using slip evidence from 3C-1",
        "parameters": {
            "arm_kp": ARM_KP_3C, "arm_kv": ARM_KV_3C,
            "gripper_kp": GRIPPER_KP_3C, "gripper_kd": GRIPPER_KD_3C,
        },
        "evidence_for_change": (
            "Direct simulation trace of 3C-1's LIFT phase: cube reached peak "
            "z=0.784 (gain ~0.05 m) with bilateral contact, then both pads "
            "lost contact around t=2.5s and the cube fell back to rest by "
            "t=3.0s. A friction back-of-envelope check (mu=1.2, cube mass "
            "0.05 kg) needs well under 1 N of normal force per pad even under "
            "a large acceleration safety factor -- far below the 15 N finger "
            "force *limit*, which was therefore not the bottleneck. A direct "
            "gripper_kp/kd sweep (holding everything else fixed) showed "
            "kp=40/kd=2 (Phase 3/3B's historical values) failed to hold the "
            "grip through the LIFT transient, kp=100/kd=6 achieved 0.0745 m "
            "(just under the 0.08 m threshold), kp=150/kd=10 achieved 0.095 m "
            "with sustained contact, and kp=200/kd=15 gave only marginal "
            "further improvement (0.098 m) -- kp=150/kd=10 was chosen as the "
            "smallest change clearing the threshold with margin, not the "
            "largest gain tried."
        ),
        "result": {k: v for k, v in r2.items() if k != "reachability"},
        "failure_state": r2["failure_state"],
        "failure_reason": r2["failure_reason"],
        "outcome": "PASS" if r2["pass"] else "FAIL",
        "determinism_check": {
            "n_reruns": len(determinism_runs),
            "all_pass": all(r["pass"] for r in determinism_runs),
            "height_gain_m_per_run": [r["height_gain_m"] for r in determinism_runs],
            "max_continuous_lifted_s_per_run": [r["max_continuous_lifted_s"] for r in determinism_runs],
        },
    })

    record["attempts"] = attempts
    record["final_outcome"] = "PASS" if attempts[-1]["outcome"] == "PASS" else "FAIL"
    record["stopped_after_attempt"] = attempts[-1]["attempt"]

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase3c_attempts.json").write_text(json.dumps(record, indent=2, default=str) + "\n", encoding="utf-8")
    print(json.dumps({"final_outcome": record["final_outcome"], "stopped_after_attempt": record["stopped_after_attempt"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
