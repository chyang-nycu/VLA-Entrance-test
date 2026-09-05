# Phase 4D: Physics-Integrity Investigation (Diagnosis Only, No Fix)

> **Evidence record — not reviewer reading.** Exhaustive per-phase audit
> trail, kept so every number in the submission is traceable. The
> reviewer-facing account is `README.md` and
> `submission/entrance_test_report.md`.

Date: 2026-09-02

**This report supersedes no prior result numerically. It marks the *interpretation* of
Phase 3C/4A/4B/4C's "PASS"/"success" claims as under review pending revalidation, per
the visible notice added to README.md/HANDOFF.md. No historical report, log, or commit
is modified. No fix is implemented in this phase.**

## Trigger

A user visual inspection of the committed Task 1 pipeline (commit `dfeec9e`) reported
two apparent critical physics defects:

1. "the hand/fingers visibly pass through the cube";
2. "the cube visibly falls downward instead of being supported and lifted."

This phase re-investigates both claims from scratch, skeptically, against the real
unmodified simulation -- not by re-reading or defending the existing test suite.

## Diagnosis summary

| Claim | Status | Root cause |
| --- | --- | --- |
| (1) hand/fingers pass through cube | **CONFIRMED, reproduced** | The vendor G1 model's own decorative `right_rubber_hand` visual mesh, fixed to `right_wrist_yaw_link`, is non-articulated and collision-free (`contype="0" conaffinity="0"` in the vendor's own MJCF) and was never suppressed, hidden, or removed by this project's scene generator. Its local-frame extent overlaps this project's real, physically-simulated finger pads. It renders as visibly clipping through any cube grasped there, on every trial. |
| (2) cube falls instead of being lifted | **NOT reproduced** by direct instrumentation of current committed code | Isolated cube/table settling test and a fresh instrumented rerun both show correct physics: the cube settles onto the table within 0.22 mm and stays there (Section C), and genuinely rises 0.108 m with real nonzero bilateral contact force during HOLD (Section D, Section G gate 6-7). Most likely explanation, not a second confirmed bug: the same decorative-mesh confusion in (1) makes the real grip look precarious/asymmetric to a viewer (see `artifacts/phase4d_collision_debug_close.png` -- the real pads contact only a lower/side region of the cube, not a centered, obviously-secure bilateral squeeze), and/or an intended `LOWER_TO_TARGET`/`OPEN` transition was misread as an uncontrolled drop. |

## A. Reproduction and evidence paths

All produced by `tasks/g1_pick_place/phase4d_reproduce_defect.py` (diagnostic-only, does
not modify any production/controller/scene-generator file) and logged in full in
`logs/phase4d_physics_integrity.json`.

| Item | Value |
| --- | --- |
| Repository root | `/Users/yangjudx/Documents/Robotics` |
| Loaded MJCF (absolute) | `tasks/g1_pick_place/g1_grasp_scene_4b.xml` (regenerated fresh by `write_grasp_scene_4b`, not a stale file -- confirmed identical XML text between two independent calls) |
| Vendor model (absolute) | `vendor/unitree_mujoco/unitree_robots/g1/g1_29dof.xml` |
| Git commit | `dfeec9e` (HEAD at start of this investigation) |
| Trial path vs. video-capture path load identical XML | **Yes**, confirmed by direct text comparison, not filename comparison |
| Output defect-reproduction video | `artifacts/phase4d_failure_reproduction.mp4` (538 frames, 125 fps, 4.30 s, 216 KB, zoomed on the gripper through CLOSE/LIFT/HOLD) |

## B. Cube identity and state tracking

Resolved once, by name, at the top of `run_trial_pick_place` (confirmed by direct source
read, not assumed):

| Field | Value |
| --- | --- |
| Body name / id | `cube` / 34 |
| Geom name / id | `cube_geom` / 77 |
| Joint name / id / type | `cube_joint` / 32 / free joint (`mjJNT_FREE`) |
| qpos address | 38 |
| qvel/dof address | 37 |

`cube_geom_id` is confirmed distinct from `target_pad_geom`, `left_finger_pad`, and the
TCP site (different `mjtObj` namespace entirely) -- the success detector, `CubeInitGuard`,
height-gain, and target-error calculations all read this same id, never a differently
named probe object. New test `Phase4DCubeIdentityTest` (3 tests, all pass) re-confirms
this from a fresh simulation, independent of the Phase 3C/4B/4C self-audits.

**A genuine, but explained, non-issue found while checking this**: sampling
`data.xpos[cube_body_id]` immediately after `mujoco.mj_step` and comparing it to
`data.qpos[qpos_adr:qpos_adr+3]` in the same instant shows a small disagreement (up to
0.336 mm during the fastest-moving LIFT phase, typically <0.06 mm elsewhere). This is
expected MuJoCo behavior, not a bug: `mj_step` integrates `qpos` to the new state but
`xpos`/`geom_xpos` are computed from the position at the *start* of that step and are not
refreshed until the next `mj_forward`/`mj_step` call. `run_trial_pick_place` always reads
`data.xpos[cube_body_id]` consistently (never mixes it with a raw `qpos` read for the same
comparison), so this one-step lag does not affect any of its telemetry or pass/fail
decisions -- confirmed by the fact that `data.xpos[cube_body_id]` and
`data.geom_xpos[cube_geom_id]` (the actually-rendered position) agree exactly (0.0 m
difference) at every sampled transition. Recorded here for completeness per the
investigation's own rigor requirement, not because it explains either reported defect.

## C. Cube/table support test (isolated, no robot motion)

3 s of pure gravity, zero robot control, real Task 1 scene (`g1_grasp_scene_4b.xml`):

| Metric | Value |
| --- | --- |
| Table geom | box, half-size `(0.22, 0.22, 0.35)` m, world pos `(0.33, -0.15, 0.35)`, contype/conaffinity `1`/`1`; single geom, both visual and collision (no separate collision-only mesh) |
| Cube geom | box, half-size `(0.035, 0.035, 0.035)` m, contype/conaffinity `1`/`1`, mass 0.05 kg |
| Timestep / integrator | 0.002 s / `implicitfast` |
| Initial cube-bottom vs. table-top | 0.0 m (spawned exactly resting) |
| Max settle drop | 0.000216 m (0.22 mm) |
| Final cube-bottom vs. table-top | -0.000216 m (i.e. 0.22 mm of natural solver settling, well inside MuJoCo's default contact margin) |
| Final vertical speed | 2.17e-14 m/s (numerically at rest) |
| Cube/table contact force samples | 5996 samples across the run; per-step **summed** normal force at rest: mean 0.4906 N vs. the cube's own weight of 0.4905 N (agreement to 4 significant figures) |
| Cube ever passed through table | **No** |

**Conclusion: table-support physics is correct.** New tests `Phase4DTableSupportTest`
(4 tests, all pass) lock in this finding for future regression detection.

## D. Finger/cube collision test (real production gripper)

Fresh instrumented rerun of the real nominal trial, reading `mujoco.mj_contactForce` at
every step (not merely a detected-contact boolean, addressing the investigation's own
"a reported contact event alone is insufficient" requirement):

| Metric | Value |
| --- | --- |
| Cube weight | 0.4905 N |
| CLOSE-phase cube/finger-pad contact force | 1778 samples, min 0.071 N, max 0.899 N, mean 0.430 N |
| HOLD-phase cube/finger-pad contact force | 11891 samples, min 0.234 N, max 0.543 N, mean 0.325 N |
| Finger pad geom contype/conaffinity | 1 / 1 (real collision) |
| Real gripper local-x range (wrist frame) | `[0.088, 0.112]` m (`FINGER_REACH_X=0.10 +/- FINGER_PAD_HALF[0]=0.012`) |
| Vendor decorative `right_rubber_hand` mesh local-x range | `[0.0415, 0.1733]` m (STL bounding box, transformed by the vendor MJCF's own geom offset `(0.0415, -0.003, 0)`) |
| Vendor mesh contype/conaffinity | **0 / 0** (collision-free, by the vendor's own authoring -- confirmed directly from `g1_29dof.xml`, not something this project disabled) |
| Ranges overlap | **Yes** |

**The functional gripper's physics is genuinely correct**: real, nonzero, physically
plausible normal force (comparable in magnitude to the cube's own weight distributed
across two pads plus squeeze preload), not a zero-force "contact detected" false
positive. The confirmed defect is that a **second, separate, decorative, collision-free
mesh** left over from the vendor model occupies the same region and is rendered every
frame. New tests `Phase4DLiftIsGenuineTest` (3 tests, all pass) and
`Phase4DDecorativeHandOverlapTest` (2 tests: one factual check that passes, and the
overlap assertion itself, which **fails on purpose** to keep this defect visible in the
default test run).

A visual zoom-in on the CLOSE phase (`artifacts/phase4d_collision_debug_close.png`)
shows this directly: the large white decorative hand mesh visibly penetrates the top and
front faces of the red cube, while the small dark, actually-functional finger pads are
barely visible near the cube's lower side/corner -- correctly gripping it via real
physics, but easy to mistake for "not gripping" or "passing through" at a glance.

## E. Viewer/render consistency

| Check | Result |
| --- | --- |
| GUI/video capture and the real trial loop use the identical generated XML | **Yes** -- `record_nominal_episode.py`'s `frame_callback` receives the exact `(model, data)` instances `run_trial_pick_place` steps and reads telemetry from; confirmed by source read, not two separately-stepped simulations |
| IK's scratch `MjData` confused with the real trial's stepped `MjData` for any metric | **No** -- `controller_3c.solve_ik_waypoint`'s `scratch_data` (`ik_scratch` in `run_pick_place.py`) is used only to compute joint targets before driving a segment; every telemetry/criteria field reads from the real `data`, never `ik_scratch` |
| Cube visual and collision geoms co-located | **Yes** (trivially -- the cube has a single geom, no separate visual-only copy) |
| Finger-pad visual and collision geoms co-located | **Yes** for the real functional gripper (single box geom per pad, no separate visual copy). The actual mismatch is not a visual/collision split *within* the finger pads -- it is a second, unrelated decorative object (Section D) occupying the same space |

## F. Test-suite audit (why 104/104 passed despite a real visible defect)

Every test in `tests/test_phase3c_grasp.py`, `tests/test_phase4a_grasp_variants.py`,
`tests/test_phase4b_pick_place.py`, and `tests/test_phase4c_slip_audit.py` was read and
classified:

Counted precisely (class-by-class, checking whether each class's own body, or a
module-level `setUpModule` fixture it references, calls `run_trial_3c` /
`run_trial_pick_place` / `run_variant_sweep`):

| Category | Count | Notes |
| --- | --- | --- |
| Real physics end-to-end (derived from a fresh, real, fully-stepped simulation -- either its own `setUpClass` call or a shared `setUpModule` fixture computed once per test run) | 50 | `Phase3CNominalGraspTest` (8, direct `run_trial_3c` call); all of `test_phase4a_grasp_variants.py` (12, shared `_SWEEP = run_sweep(...)` in `setUpModule`); `Phase4BStateMachineTest`/`AbortBehaviorTest`/`ReleaseDetectionTest`/`SuccessDetectorBoundaryTest`/`NominalPickPlaceTest`/`DeterminismTest`/`StageBVariantsTest` (23, via shared `_NOMINAL`/`_VARIANTS` `setUpModule` fixture or their own direct call); `PostReleaseIsolationTest` + `StageBVariantsUseCorrectedMetricsTest` (7, direct `run_trial_pick_place` calls). Confirmed by `grep -n "json.load\|logs/"` across all four files: **zero matches** -- none of these load a cached/logged result from `logs/phase4b_pick_place_trials.json` or `logs/phase3c_attempts.json`; every one re-simulates from scratch. This rules out "stale cached result" as the explanation for the suite passing. |
| Structural (checks MJCF content/geometry/source text, no physics stepping) | 18 | `Phase3CGripperSceneTest` (6) + `CubeInitGuard3CBoundaryTest` (2, source-scan) + `Phase4BTargetSceneTest` (8) + `Phase4BInitBoundaryTest` (2) |
| Kinematic/reachability (solves IK on a scratch `MjData`, no full trial stepped) | 5 | `Phase3CReachabilityTest` (2) + `Phase4BReachabilityTest` (3) |
| Synthetic metric/math unit test (tests a pure function in isolation) | 3 | Phase 4C's rigid-body slip-math tests (`SlipMathUnitTest`) |
| Mocked | 0 | none found |
| Log-regression (compares against a previously-recorded log/number instead of a fresh computation) | 0 *within these four files* | The only log-regression tests in the whole repo are the 3 Phase 3 historical diagnostics in `tests/test_phase3_grasp.py` (out of scope here -- a different, older file, correctly documenting a real historical failure unrelated to this investigation) |

(50 + 18 + 5 + 3 + 0 = 76, matching the exact combined test count of these four files: 18 + 12 + 36 + 10.)

**Why the suite passed despite the visible failure**: the "real physics end-to-end"
tests are genuinely real -- they check contact booleans between the correctly-named
geoms, height gain, target error, and other physically-derived scalars, and those values
are all genuinely correct (Sections C and D above). **None of the existing 104 tests, in
any category, ever inspect whether an unrelated, collision-free visual geom spatially
overlaps the cube.** That is a pure rendering/scene-authoring property with zero
coverage in the prior suite -- not a false-positive result, not a mocked value, not a
stale log. The suite was correctly testing the things it checked; it simply never checked
this. `Phase4DDecorativeHandOverlapTest`, added in this phase, closes that specific gap
and fails honestly against current code.

## G. Regression gates

| # | Gate | Status | Evidence |
| --- | --- | --- | --- |
| 1 | cube-on-table settling test passes | **PASS** | Section C |
| 2 | bilateral finger/cube contacts have nonzero normal forces | **PASS** | Section D |
| 3 | maximum penetration remains below a documented tolerance | **PASS for the functional gripper** (no interpenetration force artifacts observed); **N/A for the decorative mesh** (it is not a collision object, so "penetration" there is a pure-visual overlap, not a solver penetration depth -- see Section D for the geometric overlap instead) |
| 4 | tracked cube identity matches the rendered dynamic cube | **PASS** | Section B |
| 5 | GUI and headless runs use the same XML and controller | **PASS** | Section A, E |
| 6 | nominal cube world Z visibly and numerically rises by at least 0.08 m | **PASS** | Section D (0.108 m, fresh rerun) |
| 7 | cube remains off the table for at least 2 seconds | **PASS** | reused Phase 3C/4B criterion, re-confirmed on fresh rerun (`lifted_ge_2s_continuous`) |
| 8 | placement video visibly shows the same cube lifted and released | **PASS for the cube itself** (the existing `artifacts/phase4b_task1_nominal.mp4` and this phase's zoomed stills show the correct cube, correctly lifted and released); **the video also visibly shows the confirmed decorative-mesh overlap defect** -- viewers should expect to see the white hand mesh clip through the cube in that video too |
| 9 | success detector agrees with frame-by-frame visual inspection | **PARTIAL** -- the state-based success detector's *numeric* conclusions (grasped, lifted, released, placed) are correct and match what actually happens physically; but a naive frame-by-frame *visual* inspection of the rendered video would reasonably flag the decorative-hand/cube overlap as looking wrong, because it is wrong-looking, even though it doesn't affect the physics. This gate is not fully "green" until that visual defect is fixed. |

## H. What was NOT done in this phase

- No fix was implemented or committed. The vendor decorative mesh was not hidden,
  suppressed, removed, or given `contype`/`conaffinity` overrides anywhere.
- No historical report (`reports/phase3-grasping-baseline.md`,
  `phase3b-controller-stabilization.md`, `phase3c-position-servo-baseline.md`,
  `phase4a-grasp-variants.md`, `phase4b-task1-pick-place.md`,
  `phase4c-task1-evidence.md`) was modified. All numeric results in those reports remain
  accurate descriptions of what the simulation actually computed; only the
  *interpretation* of "Task 1 success" is under review pending a decision on whether the
  decorative-mesh overlap needs a scene fix before the task is considered visually (not
  just numerically) validated.
- No controller parameter, gain, threshold, or trajectory was changed.
- Dataset collection, cameras-as-a-feature, and Task 2 were not started.

## Recommendation (not authorized in this phase)

The most direct fix, when authorized, is to hide the vendor's `right_rubber_hand` (and
its symmetric `left_rubber_hand`, unused but present) decorative meshes in the task-local
scene generator -- e.g. by removing those specific geom elements from the deep-copied
tree in `_build_grasp_tree()`, the same "copy-and-edit, never touch vendor files"
pattern already used for every other task-local scene modification. This would not
change any physics (`contype`/`conaffinity` are already 0) -- it only removes a
misleading visual element, so it carries no risk of affecting any already-passing
numeric result. This is a recommendation for a future, separately-authorized phase, not
a change made here.
