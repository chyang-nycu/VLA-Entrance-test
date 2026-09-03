# Task 2 (Optional, Time-Boxed) — Language-Conditioned Two-Object Selection

Date: 2026-09-03. Branch: `task2-language-selection` (created from `35f15a5`,
the accepted Task 1/entrance-test submission commit on `main`).

This is optional, additive work per the entrance test's own framing
("you are not required to complete all tasks"). It does not modify or
invalidate the completed Task 1 submission: Task 1 remains exactly
reproducible at commit `35f15a5` plus this branch's own history (see
Section E, "Task 1 non-regression"). No dataset, report, schema, or commit
from Phases 1-6 is modified.

**Reporting language (required, verbatim):** The scripted oracle controller
receives the selected object identity from the task specification. The
environment and dataset expose language-conditioned object selection, but
learned visual-language grounding is not evaluated.

## Section A — scene and slot-placement evidence

Scene: `write_task2_scene()` (`tasks/g1_pick_place/task2_language_selection.py`)
re-parses `write_grasp_scene_5a()`'s own output (Task 1's scene, unmodified,
reused by import — never called with different arguments, never edited) and
adds one new body, `cube2` (green), with IDENTICAL size/mass/friction to the
existing red `cube` body (`CUBE_HALF`/`CUBE_MASS`/`CUBE_FRICTION`, imported
from `gripper_scene.py`, never redefined). `write_grasp_scene_4b`/
`write_grasp_scene_5a` are never modified; `tests/test_task2_language_selection.py::TestScene::test_task1_scene_generator_unaffected`
confirms Task 1's own generated scene has no `cube2` body.

**Slot placement was tuned empirically, not guessed**, against three
independent, measured requirements: (1) IK reachability
(`diagnose_pick_place_reachability`, IK-only, no physics), (2) real
physics-measured distractor displacement in BOTH grasp directions (the
"collision-safe transport path" requirement), and (3) onboard-camera pixel
visibility at reset. A first candidate (`-0.08, 0.0`, pure -x, 8cm from
nominal) was IK-reachable but a full physics trial measured **48.7mm** of
real distractor displacement — traced via a `frame_callback` probe (not
guessed) to `RETREAT`'s one-shot joint-space `_drive_segment` (never
Cartesian-smoothed in Task 1, since Task 1 never had a second object for it
to sweep near) carrying the arm through an uncontrolled joint-space path
that happened to pass close to that slot; `TARGET_POS` sits at
`CUBE_POS + (-0.11, +0.07)` from the nominal slot, so `(-0.08, 0.0)` sat
almost exactly on that corridor. A second candidate (`-0.08, -0.05`), moved
away from the target's `+y` pull, measured a safe 5.0mm in both directions
but was found to be only marginally visible to the onboard camera at reset
(9/19,200 green pixels, mostly occluded by the resting gripper's own
geometry). A further search — checking reachability, physics-measured
displacement in both directions, AND camera pixel count together, not
serially — found `(-0.08, -0.10)`: 0.0mm / 1.7mm displacement (both
directions, both under the 10mm target with large margin) and 30/19,200
green pixels visible (better than the first -y candidate, not merely
traded against safety). This is the shipped `SLOT_B_OFFSET`. Full
candidate-by-candidate numbers are in this module's own inline comments and
were re-derived, not retyped, from the actual measurement commands run
during this phase.

Both slots pass `diagnose_pick_place_reachability`'s `all_reachable` check
against the real Task 2 scene (not assumed from the single-cube scene's
prior measurements): `logs/task2_language_selection.json`'s `reachability`
key records `A: True`, `B: True`.

## Section B — configuration diversity

Varied: selected-object color (red/green), initial spatial arrangement
(nominal slot vs. swapped), deterministic (no RNG anywhere in this module,
same as every other phase). The controller (`run_trial_pick_place`, Phase
3C/4B/4E's own unmodified control logic) computes every waypoint from
whichever cube's *actual live pose* is passed in — never a hardcoded
position — verified directly by
`tests/test_task2_language_selection.py::TestWaypointsFollowSelectedObject::test_waypoints_follow_selected_object_not_hardcoded`,
which confirms the two slots' solved PREGRASP joint targets differ by more
than 0.05 rad (they are not the same fixed target reused regardless of
input).

No per-configuration controller tuning was introduced:
`test_same_controller_parameters_across_all_configurations` asserts
`run_trial_task2`'s own source never passes a gain/timing override to
`run_trial_pick_place` — every trial uses that function's Task-1 defaults
(`GRIPPER_KP_4E`/`KD_4E`, `TRANSPORT_DRIVE_S`, `LOWER_DRIVE_S`,
`LIFT_DRIVE_S_4E`, `RETREAT_DRIVE_S`) identically.

Every attempted configuration is recorded (all 12 trials, including their
full per-trial telemetry) in `logs/task2_language_selection.json` — nothing
is silently discarded.

## Section C — language

`instruction_canonical`: "Pick up the red cube and place it in the blue
target area." `instruction_utterance` is one of exactly two authorized
strings ("...red cube...", "...green cube..."), selected by which object is
being tested — never a third paraphrase, since only two instructions were
authorized for this phase. `parse_selected_object()` is a **trivial,
deterministic keyword lookup** ("red"/"green" substring match) over exactly
these two strings — explicitly not a language model and not visual
recognition (see the required reporting language above). It raises
`ValueError` on any instruction that does not unambiguously name exactly
one of the two known objects (tested:
`test_ambiguous_instruction_raises`, `test_neither_color_named_raises`).
The physical task (which body is grasped) is determined solely by
`selected_object_id`, never re-derived from instruction text at trial time
— `parse_selected_object` is used only to build test/report labels, not as
a code path inside `run_trial_task2` itself.

## Section D — success criteria and quality gates (per trial)

`run_trial_task2()` requires ALL of:
- `task_pass` (Task 1's own unmodified grasp+placement+retreat+dwell
  criteria, applied to the SELECTED cube's body);
- `distractor["displacement_within_10mm"]` (measured maximum XY
  displacement of the non-selected cube from its own reset pose,
  RESET through `VERIFY_TASK_SUCCESS`, tracked every physics step);
- `not distractor["in_target_xy"]` (the non-selected cube never ends inside
  the target's success margin — an explicit, separately-labeled
  `wrong_object_placed` failure mode, not lumped into `task_pass`);
- `selected_identity_agrees` (structurally guaranteed: `run_trial_pick_place`
  only ever reads/moves the body named by `selected_spec["body_name"]` —
  there is no code path that could act on the distractor's body instead;
  exercised by `test_selected_identity_agrees_with_instruction`).

All Task 1 physical-integrity constraints remain active: both cubes are
initialized exactly once, before the first physics step, through their own
`CubeInitGuard` instance (the SAME class Task 1 uses, imported unchanged,
not reimplemented) — no weld, attachment, teleport, or direct qpos/qvel
write after that point, for either cube. `run_pick_place.py`'s own
module-level self-audit (`_assert_run_trial_pick_place_has_no_direct_cube_state_write`,
a regex source-scan) still runs and passes at import time; this phase's new
`distractor_guard.set_initial_pose(...)` call goes through the exact same
guard mechanism as the primary cube, not a direct write, so it is
structurally exempt from that scan the same way the primary cube's own
`guard.set_initial_pose(...)` already is.

## Section E — minimum evaluation: 4 configurations x 3 trials (12 total)

| Selected | Arrangement | Trials (3x, deterministic) | task2_pass | selected-object success | wrong-object moved | distractor max disp. | final target error | failure stage |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| red | A | 3/3 identical | **True** | True | No | 0.0mm | 1.72mm | none |
| green | A | 3/3 identical | **True** | True | No | 1.73mm | 6.83mm | none |
| red | swapped | 3/3 identical | **True** | True | No | 1.73mm | 6.83mm | none |
| green | swapped | 3/3 identical | **True** | True | No | 0.0mm | 1.72mm | none |

**12/12 trials pass. All 4 required configurations pass all 3 deterministic
repeats.** Repeats within a configuration are bit-identical (no RNG
anywhere in this pipeline — `test_trials_within_a_configuration_are_deterministic`
asserts this directly against `logs/task2_language_selection.json`).
Distractor displacement (0.0-1.73mm) is far inside the 10mm requirement;
non-selected-cube-in-target never occurs; the selected object's own
placement error (1.7-6.8mm, depending on which physical slot it started
from) is well inside Task 1's own target-containment margin
(`TARGET_XY_SUCCESS_MARGIN_M`, unchanged).

Full per-trial telemetry (including Task 1's own complete `criteria_grasp`/
`criteria_placement`/slip/contact-force measurements for the selected cube)
is in `logs/task2_language_selection.json`.

## Section F — camera and data

`verify_camera_sees_both_objects_and_target()` renders the onboard
`head_cam`'s first (post-reset) frame with both objects present
(arrangement A) and checks: non-blank (`std > 1.0`), a red-cube-colored
region present, a green-cube-colored region present, and a target-blue
region present — the same kind of pixel-mask smoke check already used in
Phase 5A/5B (`red_cube_mask`/`blue_target_mask`, imported unchanged; a new,
analogously-calibrated `_green_cube_mask` local helper for the new object).
**All three pass**: `sees_red_cube=True`, `sees_green_cube=True`,
`sees_blue_target=True` (30/19,200 green pixels, 1050/19,200 red pixels,
28/19,200 blue pixels — see `logs/task2_language_selection.json`'s
`camera_check`). This is a rendering/visibility check only, never
task-success logic (same disclosed limitation as Phase 5A's own
`red_cube_mask`/`blue_target_mask`).

One representative episode per **successful** configuration (4 total, not
12) is recorded in `logs/task2_language_selection.json`'s
`representative_episodes`: `instruction_canonical`, `instruction_utterance`,
`selected_object_id`, both objects' initial offsets, `final_xy_target_error_m`,
`distractor_max_displacement_m`, and `success`. A full scaled HDF5 dataset
(Phase 5E's schema) was explicitly not built for this optional phase, per
the authorization ("a full scaled dataset is not required").

## Section G — videos

`artifacts/task2_red_instruction.mp4` (743,973 bytes, 389 frames, 640x480,
~29.41fps) and `artifacts/task2_green_instruction.mp4` (701,187 bytes, 389
frames, 640x480, ~29.41fps): third-person recordings of one full,
passing trial each (red-selected and green-selected, both arrangement A),
with a burned-in text overlay naming the instruction actually given and the
current state-machine phase. Both **decode-verified** (re-read frame-by-frame
via `imageio`, confirmed 389/389 frames recovered for each, matching the
frame count recorded at capture time — not merely "the file exists").
Recorded via `run_trial_pick_place`'s existing, purely-observational
`frame_callback` hook (same mechanism as Phase 4C/4E's videos) — recording
never affects control or the pass/fail outcome.

## Section H — test suite and Task 1 non-regression

`tests/test_task2_language_selection.py`: 23 new tests, covering Task 1
non-regression (2 tests: the default `run_trial_pick_place` call path is
unaffected by this phase's new optional parameters, and its nominal
placement error still matches the documented ~1-4mm range), scene
correctness (3), instruction parsing (5), waypoint-follows-selected-object
(1), physical integrity (2), camera visibility (1), the full 4x3 minimum
evaluation (8), and the wrong-object-detection logic (1).

Full-suite regression (see Section I): **311 tests, 0 unexpected failures,
604.167s** — Task 1's own existing tests all continue to pass with the same
pre-existing intentional-diagnostic exceptions as every prior phase (Phase
3/3B's 3 historical failures, Phase 4D's 1 intentional defect-documentation
failure, Phase 4F's 2 honest strict-slip-bar diagnostics) — no new
unexpected failures introduced by this phase's edits to `run_pick_place.py`.

## Section I — full regression suite result

Full suite (288 pre-existing + 23 new Task 2 tests = 311 total):
**`Ran 311 tests in 604.167s` / `OK`** — 0 unexpected failures. Confirmed
via direct re-run of
`tests/test_task2_language_selection.py::TestTask1NonRegression` (included
in this same run) that Task 1's own nominal trial is byte-for-byte
behaviorally unchanged (`task_pass=True`, `final_xy_target_error_m≈1.72mm`,
matching Phase 4E/5B-5E's own long-documented nominal result).

## Compliance checklist

- [x] Task 1 (commit `35f15a5`) preserved unmodified; branch created from
      that exact commit; every Phase 1-6 dataset/report/schema/commit
      untouched
- [x] `run_pick_place.py`'s only change is additive optional parameters
      (`cube_body_name`/`cube_geom_name`/`cube_joint_name`/`distractor`),
      all defaulting to Task 1's exact literal prior values — default call
      path verified byte-for-byte unaffected
- [x] Both cubes identical size/mass/friction/collision
- [x] Non-selected cube placed outside the selected cube's collision-safe
      transport path — verified empirically (physics-measured displacement
      <=10mm in both grasp directions), not just argued geometrically
- [x] Same non-learned scripted controller; no visual recognition or
      learned language-understanding claimed anywhere
- [x] No teleport/weld/attach/direct cube-state write after reset for
      either cube (both guarded by `CubeInitGuard`)
- [x] All 4 required configurations x 3 deterministic trials each; every
      attempted configuration recorded
- [x] Onboard camera verified to see both objects and the target at reset
- [x] Data collected: instruction, selected object ID, both object poses,
      actions (telemetry), success — one representative episode per
      successful configuration
- [x] Required files present: `task2_language_selection.py`,
      `test_task2_language_selection.py`, this report,
      `logs/task2_language_selection.json`,
      `artifacts/task2_red_instruction.mp4`,
      `artifacts/task2_green_instruction.mp4`
- [x] README/HANDOFF updated (Task 2 genuinely passed all 12 required
      trials)
- [x] Required reporting language included verbatim (top of this report)
- [x] Full regression suite run; Task 1 remains unchanged and passing
- [x] Committed on `task2-language-selection` only; not pushed; not merged
      to `main`
