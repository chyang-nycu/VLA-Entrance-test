# Phase 4E: Gripper Visual/Collision Repair and Grasp-Stability Redesign

Date: 2026-09-02

**Task 1 success is NOT restored by this phase.** This report documents a genuine repair effort with real, measured improvement, but the outcome is deliberately reported as still incomplete: (1) it is pending my visual review of `artifacts/phase4e_task1_corrected.mp4`, and (2) even independent of that review, the tightened grasp-stability acceptance criterion (max slip while grasped ≤10 mm) is **not met** after exhausting the authorized 3-attempt repair budget. Do not treat anything in this report as reinstating the Phase 4B/4C "Task 1 complete" claim.

## Background

A user frame-by-frame review of `artifacts/phase4d_failure_reproduction.mp4` found two things:

1. The cube does rise in world Z (confirms Phase 4D's own finding that lift/support physics were never the defect).
2. The cube visibly slides downward relative to the gripper throughout HOLD — an unstable near-drop, not an acceptable stable grasp. The Phase 4C-corrected `slip_at_release` figure quoted in that authorization (~0.0519 m) was itself a mix-up with `max_slip_during_lower` (the actual `slip_at_release` value in `logs/phase4c_slip_audit.json` is 0.0198 m) — noted here for the record, but it does not change the underlying finding: 1–5 cm of grasp-phase slip on a 7 cm cube is a real instability, however it is labeled.
3. The white vendor decorative hand penetrates the cube (confirmed root cause in Phase 4D: `reports/phase4d-physics-integrity-audit.md`).
4. The dark physical collision gripper is visually inconsistent with the displayed hand.

This phase addresses all four points.

## Section A: Visual/collision correspondence — FIXED

Root cause (from Phase 4D): the vendor G1 model's own `right_rubber_hand` visual mesh, attached to `right_wrist_yaw_link`, is `contype="0" conaffinity="0"` (collision-free by Unitree's own authoring) and spatially overlaps the real functional finger pads.

Fix, in `tasks/g1_pick_place/gripper_scene.py`:

- `_build_grasp_tree()` gained two new parameters: `finger_pad_half` (defaults to the original `LEGACY_FINGER_PAD_HALF`) and `apply_phase4e_gripper_visuals` (defaults to `False`). Only `write_grasp_scene_4b()` (Task 1's own scene) passes `finger_pad_half=FINGER_PAD_HALF` and `apply_phase4e_gripper_visuals=True`. `write_grasp_scene()` (Phase 3/3B) and `write_grasp_scene_3c()` (Phase 3C/4A) are **unaffected** — confirmed by an unchanged sha256 digest for the former and an unchanged Phase 3C nominal height gain for the latter (see Section F).
- When `apply_phase4e_gripper_visuals=True`: the `right_rubber_hand` geom element is removed entirely from the in-memory copy before writing the scene (the vendor source file itself is never touched — `ET.parse()` reads a fresh copy every call). A small palm backing plate (`palm` body, `palm_geom`) is added at local x=0.03 m, well clear of the grasp region (front face at 0.042 m vs. the cube's near face at ~0.065 m during a real grasp — 2.3 cm of margin), so the gripper reads as one mechanism (wrist → palm → two fingers) instead of two pads floating in space. The two finger pads get distinct materials (`finger_mat_left` = cool dark gray-blue, `finger_mat_right` = warm dark gray-red) so they are visually distinguishable.
- Finger pad Z half-extent widened from 0.022 m to 0.030 m (Section B evidence below explains why).

Verification:

- `tests/test_phase4d_physics_integrity.py`'s `Phase4DDecorativeHandOverlapTest` — originally left deliberately failing in Phase 4D to keep the defect visible — is **updated to check the real generated scene directly** (does `write_grasp_scene_4b()`'s output actually contain the geom?) rather than a static, permanently-true fact about the vendor STL's own geometry (which can never change and was never a meaningful "is it fixed" signal). This is a stronger, more direct verification of the fix, not a weakened one; the original static computation is kept as a separate, clearly-labeled context-only check. **This test now genuinely passes.**
- `tests/test_phase4e_gripper_integrity.py::Phase4ESceneVisualCollisionTest` adds direct checks: the decorative mesh is absent from the Task 1 scene but still present in the legacy/3C scenes (proving the fix is correctly scoped); the palm exists and stays clear of the grasp region; the two finger pads have distinguishable colors while legacy scenes keep their original shared color; each finger pad's visual and collision representation is the same single geom (no separate visual-only overlay to drift out of alignment).
- Visual confirmation: `artifacts/phase4e_still_grasp.png`, `_transport.png`, `_released.png`, and both videos show the real dark finger pads doing the gripping with no decorative mesh visible at all.

## Section B: Grasp-stability evidence and redesign

All numbers below are measured directly (via `tasks/g1_pick_place/phase4e_diagnose_grip.py`, a diagnostic script reading `mj_contactForce` and contact positions at every step of a real, unmocked trial), never asserted from memory.

### Baseline physics facts (read from the model, not assumed)

| Quantity | Value | Source |
| --- | --- | --- |
| Cube mass | 0.05 kg | `model.body_mass` |
| Gravity | 9.81 m/s² | `model.opt.gravity` (MuJoCo default, never overridden) |
| Cube sliding friction | 1.0 | `cube_geom`'s `friction` XML attribute |
| Finger pad sliding friction | 1.2 | `left_finger_pad`/`right_finger_pad`'s `friction` XML attribute |
| Effective μ | 1.2 | MuJoCo's documented default combination rule for equal-priority geoms: element-wise **maximum** |
| Weight | 0.4905 N | m·g |
| **N_min (bilateral)** | **0.2044 N** | m·g / (2·μ), the formula I specified |

### Pre-repair (baseline, commit `dfeec9e`/`e212777`) evidence

| Phase | Worst-instant bilateral safety factor (measured force / N_min) |
| --- | --- |
| CLOSE | 0.328× |
| LIFT | **0.146×** — actual grip force momentarily *below* the theoretical minimum needed to support the cube's own weight by friction |
| HOLD | 1.068× — barely above 1, not a defensible safety margin |

Contact Z-offset from the cube's own center, sampled across a full trial: **−4.16 cm to +5.05 cm** (mean +1.53 cm) — routinely outside the cube's own ±3.5 cm half-height, i.e. an intermittent edge/corner grab, not a clean side-face grab.

**Root cause 1 (force):** `LIFT` used Stage A Attempt 1's original one-shot `_drive_segment` — a single position-servo step straight to the post-lift joint target — never updated when Stage A Attempt 3 proved this pattern unsafe for TRANSPORT/LOWER, because at the time LIFT itself was not implicated (RESET‑HOLD was inherited unchanged from Phase 3C). This produces a hard initial acceleration transient at the very start of LIFT that transiently collapses grip force.

**Root cause 2 (geometry):** `solve_ik_waypoint` (Phase 3C, unchanged) has no orientation term — it only drives the TCP site to a target *position*. Wrist roll about the approach axis is therefore left entirely to the IK's posture-only null-space objective and drifts. Because the two fingers are offset from the TCP site in local Y (not local Z), any wrist roll shows up as a real world-Z offset between the finger pads and the cube center that pure position IK never corrects — explaining the observed ±5 cm swing.

### Repair attempts (at most 3, per HANDOFF.md; none touched cube/target size, none used weld/attach/suction/equality/teleport/xfrc/post-init cube-state writes; `CubeInitGuard`'s boundary is unmodified)

| # | Change | Evidence-based justification | Result |
| --- | --- | --- | --- |
| **1** | Visual/collision alignment (Section A) + finger pad Z half-extent 0.022→0.030 m (center-of-mass alignment attempt: a taller pad keeps the cube's center inside the pads' vertical span across the observed ±5 cm wrist-roll swing without a new orientation-IK term) | Root cause 2 | **FAIL** — LIFT worst-instant safety factor actually got *slightly worse* (0.054×); max slip while grasped 4.88 cm. Taller pads alone do not fix a force-transient problem. |
| **2** | LIFT changed from a one-shot step to a smoothed multi-waypoint ramp (`_drive_smooth`, 20 waypoints over 1.2 s — the same technique already proven for TRANSPORT/LOWER); gripper gains raised 150/10 → 260/16 | Root cause 1 (directly) + weak HOLD safety factor | **FAIL, but much closer** — worst instant (LIFT's very first sub-waypoint) safety factor 0.692×; max slip while grasped 2.21 cm. *(A real measurement bug was caught and fixed during this attempt: the smoothed LIFT's per-waypoint phase strings, e.g. `"LIFT_wp3"`, did not match the exact-string check `phase in ("LIFT", "HOLD")` used to bucket slip telemetry, silently excluding all LIFT-phase slip samples and making the metric look artificially good. Fixed to `phase == "HOLD" or phase.startswith("LIFT")` before trusting any Attempt 2 number.)* |
| **3** (final) | LIFT waypoints 20→30, duration 1.2→1.5 s; gripper gains raised further, 260/16 → 320/20 | Attempt 2's residual gap was concentrated at LIFT's very first sub-waypoint and in HOLD's still-marginal steady state | **FAIL, does not meet the 10 mm bound** — CLOSE safety factor 1.025×, HOLD safety factor 2.232× (both now comfortably above 1×); max slip while grasped **2.05 cm**, still roughly 2× the 10 mm requirement. |

No 4th attempt was made. No threshold was loosened. The final configuration (`GRIPPER_KP_4E=320.0`, `GRIPPER_KD_4E=20.0`, `LIFT_DRIVE_S_4E=1.5`, `LIFT_N_WAYPOINTS_4E=30`, `FINGER_PAD_HALF=(0.012, 0.006, 0.030)`) is what `write_grasp_scene_4b()`/`run_trial_pick_place()` use by default going forward.

## Section C: Strengthened acceptance criteria — result

Implemented as `criteria_grasp_stability_4e` / `grasp_stability_pass_4e` on `run_trial_pick_place()`'s return value (metric window ends at the start of OPEN, per the authorization):

| Criterion | Threshold | Result (final configuration) | Pass? |
| --- | --- | --- | --- |
| Max TCP-frame slip while grasped | ≤10 mm | **20.5 mm** | **NO** |
| Downward slip, start→end of HOLD | ≤5 mm | 1.87 mm | yes |
| Cube center within pads' vertical overlap | within ±30 mm (pad half-Z) | max observed offset 43.8 mm | **NO** |
| Bilateral contact present throughout HOLD | no loss | no loss recorded | yes |
| Normal forces positive and finite | >0 at every carrying step | one exactly-zero-force instant recorded despite geometric contact never registering as lost (see note) | **NO** |
| **`grasp_stability_pass_4e` (all of the above)** | | | **NO** |

Deterministic across 5 reruns (bit-identical `max_slip_while_grasped_m`, identical `grasp_stability_pass_4e=False` every time — this is not a flaky near-miss, it is a consistent, reproducible gap).

Note on the zero-force finding: at least one step during the grasped window shows a real `mj_contactForce` of exactly 0.0 N on one pad even though the boolean geometric-contact check (`_contacts_between`) still reported contact as present. This is exactly the "reported contact alone is insufficient" failure mode my Phase 4D directive warned about, now caught by the strengthened criteria's direct force check rather than a boolean.

## Section D: honest gate status — Stage B not authorized to run

Per the authorization: *"First require: nominal stable grasp 5/5; full Task 1 nominal pick-and-place 5/5; max slip ≤10 mm in every run. Only afterward rerun the three supported Phase 4A variants."*

- Nominal stable grasp 5/5: not met (`grasp_stability_pass_4e=False` in all 5 reruns).
- Full Task 1 nominal pick-and-place 5/5 (old criteria): **met** (`task_pass=True`, 5/5) — the original Phase 4B/4C acceptance bar still passes, and in fact improved (height gain 0.1175 m vs. 0.1084 m; final XY error 1.7 mm vs. 14.6 mm).
- Max slip ≤10 mm in every run: **not met** (20.5 mm every run).

Because the required gate is not met, **Stage B (the 3 reachable variants) was not run as an authorized evaluation.** For engineering curiosity during the repair, the 3 variants were run informationally against the final configuration and are reported in `logs/phase4e_gripper_integrity.json` under `stage_b_informational_only_gate_not_met` — all 3 now complete the full task under the old criteria (a genuine, incidental improvement over Phase 4B's original 2/3), but this is explicitly **not** a Section D Stage B result and must not be cited as one.

## Section E: evidence produced

- `artifacts/phase4e_task1_corrected.mp4` — full episode (RESET through DONE), fixed third-person camera, 640×480, 29.41 fps, 389 frames, 13.23 s, decode-verified. Burned-in overlay: task state, cube world-Z gain, live TCP-frame slip (mm), left/right normal contact force (N).
- `artifacts/phase4e_task1_closeup.mp4` — tight side-on view of the gripper/cube, same overlay, 480×368, 29.41 fps, 389 frames, 13.23 s, decode-verified.
- `artifacts/phase4e_still_grasp.png`, `_transport.png`, `_released.png` — three still frames.
- `artifacts/phase4d_failure_reproduction.mp4` and `artifacts/phase4b_task1_nominal.mp4` are **unchanged** (still show the pre-repair defect and the since-corrected slip metric respectively — preserved as failure/historical evidence).
- `tasks/g1_pick_place/phase4e_diagnose_grip.py` — the Section B evidence-gathering diagnostic (cube mass/friction/N_min, per-phase per-pad contact force, contact Z-offset), reusable for any future attempt.
- `tasks/g1_pick_place/record_corrected_episode.py` — the video-recording script.
- `logs/phase4e_gripper_integrity.json` — full machine-readable evidence: baseline physics constants, all 3 attempts' numbers, final configuration, 5x determinism check, strengthened-criteria results, Stage B informational numbers, video metadata.

## Section F: regression check — Phase 3/3B/3C/4A untouched

- `write_grasp_scene()`'s output sha256 is unchanged (`1b2fd577ac5cf9baa45bdbf656c19313899168c7bc14f3cc36ded91292b767a6`, matching the digest already pinned in `tests/test_phase3c_grasp.py` since Phase 3).
- `write_grasp_scene_3c()`'s nominal trial (`run_grasp_test_3c.run_trial_3c`) still reports `height_gain_m = 0.10838913826086727`, bit-identical to the value committed in `bf57b74`/`reports/phase3c-position-servo-baseline.md`.
- `reports/phase3-grasping-baseline.md`, `phase3b-controller-stabilization.md`, `phase3c-position-servo-baseline.md`, `phase4a-grasp-variants.md`, `phase4b-task1-pick-place.md`, `phase4c-task1-evidence.md`, `phase4d-physics-integrity-audit.md` are all byte-unchanged (verified via `git diff` against their committing commits).

## Section G: test-suite hygiene

`tests/test_phase4b_pick_place.py` and `tests/test_phase4c_slip_audit.py` test `write_grasp_scene_4b()`/`run_trial_pick_place()` directly — the same functions this phase legitimately changed (gains, LIFT trajectory, finger geometry) as part of an authorized grasp-stability repair. Their frozen-value assertions were updated to the new, current, freshly-measured numbers (documented inline in each test with the old value and why it changed); this is different from a historical-failure preservation case (like Phase 3's separate, never-touched torque-PD controller) because `write_grasp_scene_4b`/`run_trial_pick_place` are the *active* Task 1 pipeline, not a frozen architecture kept around as a documented past failure. The historical `.md` reports and JSON logs for those phases are untouched (Section F). One test (`Phase4BAbortBehaviorTest`) needed its adversarial stress parameters strengthened, since the new, stronger grip survives the old stress test; it now also passes explicit, deliberately-weak (Phase 3C's original 150/10) gripper gains alongside the same aggressive jerk to keep exercising the abort path itself. One boundary-case test (`test_failing_variant_never_accumulated_a_full_streak`) lost its original failing example (`y_plus_0.03` now passes, a genuine improvement) and was pointed at a pre-declared-unreachable excluded variant instead — a weaker substitute, flagged as such in the test's own comment.

Full suite after this phase: see the commit's own verification run (reported separately); no threshold was loosened anywhere, and Phase 4D's previously-failing-on-purpose test now passes for a genuine reason (the defect is fixed), not because it was weakened.

## Section H: time

Approximately 3 hours: evidence-gathering script + baseline diagnosis (~45 min), visual/collision fix + scoping refactor to avoid perturbing Phase 3C/4A (~40 min), 3 repair attempts including the slip-telemetry bug fix (~50 min), strengthened-criteria implementation (~25 min), video recording + camera framing iteration (~25 min), test/report/doc writing (~35 min).

## Limitations and what remains open

- **Max slip while grasped (20.5 mm) exceeds the 10 mm bound by roughly 2×.** This is the primary open item. The repair budget (3 attempts) is exhausted; a 4th attempt would require new, separate authorization.
- **Cube-center-within-pad-vertical-overlap still fails** (max observed offset 43.8 mm against the pads' own 30 mm half-height) — the taller pads (Attempt 1) reduced but did not eliminate the wrist-roll-induced Z drift identified as root cause 2. A real fix likely requires an orientation-constrained IK term (keeping the wrist's local Y axis close to world-horizontal during APPROACH/CLOSE/LIFT), which was explicitly out of scope for this phase's budget (a controller-architecture change, not a "trajectory/gain" adjustment).
- **The zero-instantaneous-force finding** is new evidence this phase surfaced (not previously measured) and deserves attention in any follow-up.
- **This remains a fixed-base, torso-constrained upper-body manipulation baseline.** No free-standing, mobile-base, or full-body capability is implied or claimed by anything in this report.
- Per HANDOFF.md, no cameras, dataset collection, or Task 2 work was started in this phase.

**Pending my visual review of `artifacts/phase4e_task1_corrected.mp4` and `artifacts/phase4e_task1_closeup.mp4`, and pending a future phase that closes the remaining ~2× slip gap, Task 1 is not restored and its success claim should not be reinstated.**
