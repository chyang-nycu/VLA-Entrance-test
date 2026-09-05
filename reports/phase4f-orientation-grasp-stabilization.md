# Phase 4F: Orientation-Constrained Grasp Stabilization

Date: 2026-09-02

**Platform framing (binding, unchanged): this is a fixed-base, torso-constrained upper-body manipulation baseline.** The pelvis and torso remain rigidly welded to the world; only the right arm and gripper move.

**Task 1 success is NOT restored by this phase.** As I required for this phase, this report presents evidence and stops before any success claim; final approval requires my visual review of the two videos linked below.

## Scope and background

Phase 4E fixed the decorative-hand visual/collision defect and improved grasp stability from a 0.146x worst-instant safety factor to >=1.0x, but my own frame-by-frame review of `artifacts/phase4e_task1_closeup.mp4` found the grasp still slides ~20.5mm downward relative to the gripper during HOLD -- a "near-drop," not an acceptable stable grasp, against a 0.07m cube. This phase's primary hypothesis: `solve_ik_waypoint()` (Phase 3C, `controller_3c.py`) is deliberately position-only, so the arm's redundant joints are free to roll the wrist during descent, misaligning the finger-pad contact band from the cube's center.

**Historical preservation**: Phase 3/3B/3C/4A/4B/4C/4D/4E reports, logs, commits, and evidence (including `artifacts/phase4d_failure_reproduction.mp4` and `artifacts/phase4b_task1_nominal.mp4`) are byte-unchanged. `write_grasp_scene_4b()` and its default `run_trial_pick_place(...)` call (no new flags) are **also unchanged in behavior** -- confirmed by the full pre-existing test suite (139 tests through Phase 4E) still passing unmodified. Phase 4F's changes are additive and opt-in: a new `write_grasp_scene_4f()` scene function and a new `use_oriented_ik=True` parameter on `run_trial_pick_place()`, both defaulting off everywhere except Phase 4F's own evidence path.

## A. Orientation-aware IK design

`solve_ik_waypoint_oriented()` (`tasks/g1_pick_place/controller_3c.py`) extends Phase 3C's position-priority solver:

- **Position task**: bit-for-bit identical to `solve_ik_waypoint()` -- same unweighted primary Jacobian pseudo-inverse, same `IK_POS_TOL` convergence bar. Never traded away for orientation.
- **Orientation task**: a null-space secondary objective (same priority tier as the existing joint-limit-avoidance and nominal-posture terms), so it can only consume redundancy the position task does not need.
- **Required axis**: the wrist's local Z axis (the axis this project's finger-pad geometry treats as "tall", `FINGER_PAD_HALF[2] = 30mm`), aligned toward world +Z. This is deliberately a 2-DOF constraint (vector alignment), leaving rotation about the aligned axis (yaw) free.
- **Weights**: `POS_WEIGHT` is implicit at 1.0 (the unweighted primary task, as above). `ORIENT_WEIGHT = 0.6` scales the null-space correction step -- chosen as Attempt 1's starting value; Attempt 2 (below) found increasing it only trades away position accuracy, so it was kept at 0.6.
- **Orientation tolerance**: `ORIENT_TOL_RAD = arcsin(0.005 / FINGER_CONTACT_Y) ≈ 7.0 deg`, derived (same style as Phase 3C's `IK_POS_TOL`) from requiring the resulting pad-height error (`FINGER_CONTACT_Y * sin(tol)`) to stay at or below 5mm -- half of Phase 4E's own vertical-overlap margin and equal to this phase's own downward-slip-during-HOLD bar.
- Orientation residual is recorded at every waypoint (`orientation_residual_rad` telemetry, `orientation_residual_rad()` helper function) regardless of whether it is ever used to gate anything.

**A key finding changed the plan mid-phase**: at the real, converged nominal APPROACH configuration, `local Y` (the jaw axis the fingers slide on) was already well-aligned to world Y (~4-5 deg off) -- left/right finger height symmetry was never the actual defect. `local Z` was ~47 deg off vertical, because the arm reaches down to the cube at a steep diagonal, not horizontally. A finger pad box tilted ~47 deg contacts a vertical cube face at a **corner**, not flush across its face -- a small, unstable contact patch that explains the reported near-drop far better than a simple height mismatch (whose contribution, from the Y-axis analysis, is only ~3mm).

## B/D. Three-attempt evidence-driven repair budget

| Attempt | Change | Evidence | Result |
| --- | --- | --- | --- |
| **1** | Add `solve_ik_waypoint_oriented()`: null-space orientation objective, `orient_weight=0.6` | Orientation residual at APPROACH: 47.39 -> 44.48 deg (real full-trial rerun). `max_slip_while_grasped_m`: 0.02193m (no improvement over Phase 4E's 0.0205m). | **FAIL** at `SETTLE_LOWER`. Marginal orientation improvement; slip unchanged. |
| **2** | Increase `orient_weight` 0.6 -> 2.0 (same null-space mechanism); separately, a co-primary stacked-DLS diagnostic tested orientation as an equal-priority task | Isolated IK sweep: `orient_weight` 0.6/2/5/10 -> orientation 44.45/41.19/36.71/32.80 deg, but position residual grows 7.4/8.3/10.5/13.7mm. Co-primary stacked sweep: reaching <5 deg orientation requires 57-67mm position error. Real full trial at `orient_weight=2.0`: **FAILED at `SETTLE_APPROACH`** (TCP never settled before CLOSE -- no grasp attempted at all). | **FAIL**, worse than Attempt 1. Reaching `ORIENT_TOL_RAD` (~7 deg) at this Cartesian point requires 30-70mm of position error -- i.e. missing the cube. A genuine kinematic reachability conflict (consistent with Phase 3C's own documented wrist singularity there), not a tuning problem. **Reverted to `orient_weight=0.6`.** |
| **3** | Measured finger-pad mounting correction (`FINGER_MOUNT_FIX_QUAT`, `gripper_scene.py`): rotate each finger **body** (not its position) by a fixed quaternion about the wrist's own local Y (jaw) axis, computed from the real converged nominal APPROACH configuration, so local Z levels to ~4 deg from vertical. The finger's `pos` (origin, in the wrist frame) is bit-for-bit unchanged -- confirmed by `test_finger_position_unchanged_by_mount_fix` -- so the finger still brackets the TCP/cube target exactly as before; only its own shape/joint-axis orientation rotates. | `max_abs_contact_z_offset_from_cube_center_m`: 0.04376 -> 0.03654m (a real, ~17% reduction, consistent with the corner-vs-flush-face hypothesis). `max_slip_while_grasped_m`: 0.02050 -> 0.02592m (did **not** improve; rose slightly). | **FAIL** at `SETTLE_LOWER` (same failure point as Attempt 1). The targeted static-geometry metric improved; overall slip did not, indicating slip during this grasp is also driven by dynamic effects (impact/settling/friction transients during CLOSE/LIFT), not solely by static contact-patch geometry. |

No 4th attempt was made. No threshold was loosened. Attempt 3's configuration (`orient_weight=0.6` + pad-mount fix) is carried forward as **final**, since it is the only attempt that improved a real physical measurement without regressing further than Attempt 1's failure point.

## C. Acceptance criteria: final configuration vs. all 11 requirements

| # | Criterion | Result | Pass? |
| --- | --- | --- | --- |
| 1 | Both fingers contact opposing cube side faces | Contact-normal/jaw-axis alignment check: left and right both >0.7 alignment | **PASS** |
| 2 | Bilateral contact persists through LIFT and HOLD | No contact-loss event recorded | **PASS** |
| 3 | Max 3D TCP-frame slip while grasped <=10mm | 25.92mm | **FAIL** |
| 4 | Downward slip during HOLD <=5mm | 4.43mm | PASS |
| 5 | Downward slip through transport <=10mm | 0.00mm (trial fails at SETTLE_LOWER before reaching this state in a way that would exercise sustained transport loading) | PASS (not fully exercised) |
| 6 | Cube center within both pads' vertical overlap | Max offset 36.5mm > `FINGER_PAD_HALF[2]`=30mm | **FAIL** |
| 7 | Cube height gain >=0.08m | 0.1155m | PASS |
| 8 | Continuous off-table hold >=2.0s | Satisfied (grasp criteria pass) | PASS |
| 9 | Physical release and settled placement | Trial fails before OPEN/release (`SETTLE_LOWER`) | **FAIL** |
| 10 | Finite, bounded actuator forces | Confirmed finite throughout | PASS |
| 11 | No cube-state manipulation after initialization | Enforced structurally (`CubeInitGuard` + source self-audit) | PASS |

**Overall: 7 of 11 pass; the grasp-quality-critical criteria (3, 6, 9) fail.** `grasp_stability_pass_4f = False`. Deterministic across 5 reruns (bit-identical `max_slip_while_grasped_m` every time -- no RNG in this pipeline).

## D. Stage B (informational only, per HANDOFF.md)

Stage A's gate (nominal 5/5 stable grasp, full pick-and-place 5/5, max slip <=10mm every run) was **not met**, so Stage B was run for diagnostic evidence only, not as an authorized pass/fail evaluation, exactly as instructed:

| Variant | `task_pass` | `max_slip_while_grasped_m` | `failure_state` |
| --- | --- | --- | --- |
| nominal | False | 0.02592 | SETTLE_LOWER |
| x_minus_0.03 | False | 0.02018 | SETTLE_LOWER |
| y_plus_0.03 | False | 0.02565 | SETTLE_LOWER |

All three Phase-4A-reachable variants fail the same gate, at the same stage, with comparable slip magnitudes -- consistent, not variant-specific.

## E. Videos

- `artifacts/phase4f_task1_full.mp4` -- 640x480, 29.41fps, 352 frames, 11.97s, 320667 bytes. Decode-verified (frame count matches encode).
- `artifacts/phase4f_bilateral_contact_view.mp4` -- 480x360 (padded to 480x368 by the encoder's macroblock alignment), 29.41fps, 352 frames, 11.97s, 543636 bytes. Decode-verified. Camera perpendicular to the measured jaw-closing axis (world Y), tracking the cube's live position every frame; azimuth 155 deg was chosen after an 8-way sweep specifically because it is the only tested angle that does not place either finger pad fully behind the cube (confirmed by extracting and visually inspecting rendered frames, not by angle math alone). At a fully-closed grip the pads sit flush against the cube's side faces, so only their edges remain visible around the cube silhouette rather than a wide gap on both sides -- expected for a closed jaw, not an occlusion defect.
- Both videos burn in: task state, live position residual (distance from TCP to the current segment's target), live required-axis orientation residual (degrees), total TCP-frame grasp slip, vertical/downward slip since grasp, and left/right normal contact force -- overlay legibility confirmed by extracting and reading sample frames directly (e.g. one frame reads `state: HOLD, position residual: 4.81mm, orientation residual (required axis): 10.0 deg, total grasp slip: 8.94mm, vertical/downward slip: 0.00mm, contact force L/R 3.189/3.211N`).

## F. Tests

`tests/test_phase4f_orientation_grasp.py` -- 20 tests: 3 synthetic orientation-math unit tests, 3 real (non-synthetic) oriented-IK tests against the actual scene (including one that documents, not hides, the real >30 deg residual at APPROACH), 4 pad-mount-fix geometry tests (including that the legacy Phase 3C scene has no mount fix, and that the fix leaves the finger's position bit-identical), 7 real-pipeline tests against the final configuration (several honestly assert the current FAILING numbers, e.g. `test_max_slip_while_grasped_still_exceeds_tightened_bar` pins the exact 0.02592m value and instructs future maintainers to replace, not loosen, it if a future phase closes the gap), and 3 Stage-B/vendor-integrity tests.

**Full regression: 159 tests, 0 unexpected failures** (139 pre-existing from Phase 4E unchanged + 20 new). Confirmed via full `unittest discover` run. `write_grasp_scene_4b()`'s own behavior (and therefore every Phase 4B/4C/4D/4E test's numeric expectation) is untouched -- verified by the full pre-existing suite passing without modification.

## G. Limitations, reported honestly

- **The tightened <=10mm slip bar is not met.** The final configuration's genuine, measured slip is ~26mm -- closer to Phase 4E's ~20.5mm baseline than to the target, not a fix.
- **The kinematic reachability conflict at APPROACH is real and load-bearing**: Attempt 2's evidence shows this specific arm, at this specific Cartesian grasp point, cannot simultaneously hit both the position tolerance and a level wrist. Closing this gap would need either a different grasp waypoint further from the singularity, a redesigned reach trajectory that avoids it, or accepting a materially different (larger) position tolerance -- none of which were in this phase's authorized budget (Attempt 3 was restricted to waypoint/pad-center alignment, not trajectory redesign).
- **The pad-mount fix is calibrated to the nominal cube position specifically.** Stage B shows it applies unchanged to the 3 reachable variants (same failure mode, similar magnitude), which is at least consistent, but a full generalization claim was not tested beyond these three points.
- **Slip is not purely a static-geometry effect.** Attempt 3 reduced the static contact-offset metric by ~17% without reducing overall slip, meaning a real, separate dynamic contribution (impact/settling/friction transients) remains uninvestigated and unaddressed by this phase's budget.
- This remains a fixed-base, torso-constrained upper-body manipulation baseline; no full-body or free-standing capability is implied.

## Next step

Per the attempt budget I set, this phase stops here. Task 1 success requires my visual review of `artifacts/phase4f_task1_full.mp4` and `artifacts/phase4f_bilateral_contact_view.mp4`, and, given the quantitative gap documented above, most likely a further, separately-authorized phase addressing the trajectory/waypoint-level root cause (not a 4th tuning attempt within this phase's exhausted budget).
