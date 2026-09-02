# Phase 4A Grasp Setup-Variant Evaluation

Date: 2026-09-02

**Platform framing (binding, per HANDOFF.md): this is a fixed-base, torso-constrained upper-body manipulation baseline.** The pelvis and torso are rigidly welded to the world; only the right arm and gripper move. This is not full-body or free-standing manipulation, and is not described as such anywhere in this report.

## Scope

- Repository: `vendor/unitree_mujoco` (pinned, unchanged), task-local additions under `tasks/g1_pick_place/`
- Baseline commit: `bf57b74` ("feat: complete deterministic G1 physical grasp baseline")
- Sweep script: `tasks/g1_pick_place/run_variant_sweep.py`
- Raw results: `logs/phase4a_grasp_variants.json`

**Result: 3 of 5 variants succeed (60%), 9 of 15 trials succeed (60% trial-level), using Phase 3C's unmodified configuration. Zero global adjustments were needed** — the shared configuration cleared the >=3/5 target on the first (unmodified) sweep, so HANDOFF.md's 2-adjustment budget was not touched.

## Preserved baseline (before any Phase 4A work)

Before running any variant, the committed Phase 3C nominal test suite was rerun once, unmodified: `tests.test_phase3c_grasp.Phase3CNominalGraspTest`, 8/8 pass, at commit `bf57b74`. No IK, servo, gripper, or state-machine parameter was changed before or during the variant sweep. The shared configuration used for every variant in this report is exactly Phase 3C's winning attempt 3C-2 configuration:

| Parameter | Value |
| --- | --- |
| `arm_kp` / `arm_kv` | 400.0 / 25.0 |
| `gripper_kp` / `gripper_kd` | 150.0 / 10.0 |
| IK position tolerance | 8 mm (evidence-based, unchanged) |
| Scene | `tasks/g1_pick_place/g1_grasp_scene_3c.xml` (pelvis+torso weld, `implicitfast` integrator, position-servo right arm) |

## Variant definitions

Table-plane axis 1 = x, axis 2 = y (`CUBE_POS = (0.33, -0.15, 0.735)` in `gripper_scene.py`). 0.03 m is HANDOFF.md's default magnitude; it was not deviated from — the table's half-extent is 0.22 m (footprint 0.44 x 0.44 m, centered on the nominal cube xy), so a 0.03 m offset plus the cube's own 0.035 m half-extent leaves 0.155 m of margin to the table edge even before applying the 0.05 m conservative edge margin used in the containment check below. No variant was ever at risk of falling off the table; the actual limiting factor turned out to be arm/IK reachability, not table geometry (see results).

Each variant: stable ID, explicit cube pose (computed from the offset, not hardcoded), fixed initial robot state (MuJoCo's default `mj_resetData` pose, identical for all variants), identical controller parameters (table above), and PREGRASP/APPROACH/LIFT/HOLD waypoints generated from that variant's own observed cube pose via `diagnose_reachability`/`solve_ik_waypoint` — never the nominal coordinates. Cube pose is written only inside `CubeInitGuard`'s pre-lock window, exactly as `run_trial_3c` already enforces for Phase 3C; this is unmodified and reused as-is for every variant.

The simulation is fully deterministic (established in Phase 3C: 5/5 bit-identical reruns) — there is no RNG anywhere in this pipeline, so "fixed seed" is trivially satisfied. Each variant was still run 3 times, per HANDOFF.md, for auditability; all 3 repeats were bit-identical for every variant in this sweep (confirming determinism held here too, not just for the original Phase 3C nominal case).

## Pre-run feasibility checks (all 5, recorded regardless of outcome)

| ID | Offset (x, y) m | Cube pose (x, y, z) | PREGRASP resid (m) | APPROACH resid (m) | Table contained | Joint-limit OK | TCP/pad align (deg from nominal) | Accepted reachable |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| nominal | (0.00, 0.00) | (0.330, -0.150, 0.735) | 0.00271 | 0.00788 | yes | yes | 0.00 (reference) | **yes** |
| x_plus_0.03 | (+0.03, 0.00) | (0.360, -0.150, 0.735) | 0.00475 | **0.02708** | yes | yes | 8.55 | **no** |
| x_minus_0.03 | (-0.03, 0.00) | (0.300, -0.150, 0.735) | 0.00107 | 0.00503 | yes | yes | 2.22 | **yes** |
| y_plus_0.03 | (0.00, +0.03) | (0.330, -0.120, 0.735) | 0.00357 | 0.00766 | yes | yes | 3.91 | **yes** |
| y_minus_0.03 | (0.00, -0.03) | (0.330, -0.180, 0.735) | 0.00607 | **0.00843** | yes | yes | 4.04 | **no** |

APPROACH residual acceptance threshold is `IK_POS_TOL = 8 mm` (Phase 3C's evidence-based tolerance, derived from a genuine wrist kinematic singularity near the nominal grasp point — see `reports/phase3c-position-servo-baseline.md`). Table containment used a 5 cm conservative edge margin beyond the cube's own footprint; every variant passed this trivially, as expected from the geometry above. Joint-limit margin was checked as >2% of each joint's range from either bound at the solved APPROACH configuration; all 5 variants cleared this. TCP/finger-pad alignment compares the world-frame direction of the gripper's closing axis (TCP site's local Y) at the solved APPROACH configuration against the nominal variant's own closing axis; all deviations stayed under the conservative 15° bound used here, so this check did not distinguish any variant's outcome in this sweep (all reachability failures were driven by the APPROACH IK residual, not by pad misalignment).

Two variants were rejected as unreachable **before any trial ran**: `x_plus_0.03` (residual 27.1 mm, more than 3x the tolerance — pushing the cube further from the body puts APPROACH outside what this fixed-base arm's redundancy can resolve to within tolerance) and `y_minus_0.03` (residual 8.43 mm, a narrow miss just over the 8 mm bound). Neither rejection was applied after seeing a trial fail; both are pre-run, physically-grounded IK results.

## Trial results

| ID | Trials pass/total | Bilateral-contact rate | Max height gain (m) | Max continuous hold (s) | Variant result |
| --- | --- | --- | --- | --- | --- |
| nominal | 3/3 | 1.00 | 0.1084 | 3.504 | **SUCCESS** |
| x_plus_0.03 | 0/3 | 0.00 | 0.0000 | 0.000 | FAIL (rejected pre-run; confirmed in sim) |
| x_minus_0.03 | 3/3 | 1.00 | 0.1064 | 3.516 | **SUCCESS** |
| y_plus_0.03 | 3/3 | 1.00 | 0.1064 | 3.502 | **SUCCESS** |
| y_minus_0.03 | 0/3 | 0.00 | 0.0000 | 0.000 | FAIL (rejected pre-run; confirmed in sim) |

Every trial in both rejected variants failed identically, at state `SETTLE_APPROACH`, with reason "TCP did not settle within tolerance before CLOSE" — i.e. the state machine's own dynamic settle gate independently caught the same limitation the static pre-run IK check predicted, and correctly refused to advance to CLOSE/LIFT rather than attempting (and failing) a grasp on a misplaced arm. This is the gating behavior HANDOFF.md's state machine was designed to provide, working as intended — no cube was ever displaced or mishandled in these two variants; the arm simply never got there.

The 3 succeeding variants' pre-close cube displacement, TCP tracking error, and force saturation are effectively identical to the nominal Phase 3C 3C-2 numbers already documented in `reports/phase3c-position-servo-baseline.md` (small cube-position changes here did not materially change arm-tracking behavior once a variant was inside the reachable envelope) — full per-trial detail (tracking-error-by-phase, per-phase saturation, exact contact sequence) is in `logs/phase4a_grasp_variants.json`, not reproduced in full here to avoid duplicating that file.

**Success criteria applied, unchanged from Phase 3C**: bilateral opposing-pad contact; height gain >=0.08 m; continuous off-table hold >=2.0 s; finite/bounded outputs; physical release; no prohibited cube manipulation (verified for every trial via the reused, unmodified `CubeInitGuard`/source-scan mechanism). A variant "succeeds" if a majority of its trials (>=2 of 3) meet all 5 criteria — stated explicitly and applied uniformly; since the simulation is deterministic, this rule collapsed to "all trials agree" in every variant observed here.

## Aggregate results

- **Per-variant success: 3/5 (60%)** — target (>=3/5) met.
- **Trial-level success: 9/15 (60%)**.
- **Global adjustments used: 0 of the 2 allowed.** The unmodified Phase 3C configuration met the target on the first sweep; no per-variant or global tuning was applied or needed. (Per HANDOFF.md, manufacturing an adjustment that wasn't needed would misrepresent the result — none was made.)

## Regression check: legacy test-suite hygiene

Per HANDOFF.md's Phase 4A test-suite hygiene requirement, the 3 previously-unexplained failures in `tests/test_phase3_grasp.py` (Phase 3's historical torque-PD architecture) were converted from acceptance assertions into regression diagnostics that assert the documented failure persists (`test_height_gain_remains_below_threshold_historical_failure`, `test_hold_duration_remains_below_threshold_historical_failure`, `test_nominal_trial_overall_remains_failing_historical`), each checked against the exact historical numbers from `reports/phase3-grasping-baseline.md` (height gain 0.005125 m, hold 0.0 s) with a tight numeric tolerance so any accidental future change to that unmodified legacy code path would be caught. No threshold was lowered anywhere; no Phase 3C or Phase 4A test was marked as an expected failure; `reports/phase3-grasping-baseline.md` and `reports/phase3b-controller-stabilization.md` were not edited.

Full regression run (`python -m unittest discover -s tests`), this session:

| Category | Count |
| --- | --- |
| Passing current-controller tests (Phase 2, Phase 3 structural, Phase 3C, Phase 4A) | 46 (pre-Phase-4A-test-file) + 12 (`test_phase4a_grasp_variants.py`) = 58 |
| Explicit legacy expected-failures/diagnostics (Phase 3 nominal-acceptance, now regression diagnostics) | 3, all passing as diagnostics (asserting the historical failure persists) |
| Actual unexpected failures | **0** |

The repository's default verification command now ends cleanly with no unexplained failures, while the legacy controller's genuine, historical failure remains visible, verified, and unedited.

## Limitations

- **Reachability, not grip strength, was the limiting factor for the 2 failing variants.** Both failures were caught by the fixed-base arm's kinematic reach/IK-tolerance envelope, before the gripper was ever involved. This is a different limitation from Phase 3/3B's tracking-oscillation failure and from nothing observed in Phase 3C's own single-point analysis — the 8 mm IK tolerance, evidence-based at the nominal point, does not hold uniformly across this small a workspace patch.
- **The evaluated envelope is asymmetric**: -0.03 m on x and +0.03 m on y succeed, but +0.03 m on x and -0.03 m on y fail. This is consistent with, but does not by itself fully explain, the previously-identified wrist singularity near the nominal point (a full Jacobian-conditioning map over the workspace was not computed in this phase — out of scope).
- **This remains a fixed-base, torso-constrained upper-body manipulation baseline.** No free-standing, mobile-base, or full-body capability is implied or claimed by these results.
- Per HANDOFF.md, Phase 4A does not include transport, target placement, cameras, dataset collection, or language variants. None of these were implemented in this phase.
