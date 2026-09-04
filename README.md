# G1 MuJoCo Entrance-Test Project

Task-local manipulation experiments built on top of the official Unitree
`unitree_mujoco` simulator, targeting the Unitree G1 humanoid. All work is
staged in phases; each phase must pass its acceptance test before the next
begins.

**Current capability, precisely stated: a two-task Unitree G1 manipulation
prototype with classical expert control, language-associated VLA
demonstration interfaces, physical interaction validation, and replay
instrumentation.** A fixed-base, torso-constrained upper-body manipulation
baseline completes **Task 1** ("pick up the red cube and place it in the
blue target area") as an entrance-test prototype, with a documented
grasp-slip limitation, and **Task 2** ("language-conditioned two-object
selection", merged from `task2-language-selection`) adds a second cube
where the instruction determines which cube a privileged scripted expert
grasps and places. The pelvis and torso are rigidly welded to the world;
only the right arm and gripper move. This is not full-body or
free-standing manipulation, and not a production-ready or universally
model-ready system.

> **Task 1 acceptance status (Phase 4F human decision, 2026-09-02):**
> human visual review of `artifacts/phase4f_task1_full.mp4` and
> `artifacts/phase4f_bilateral_contact_view.mp4` found the task-level
> execution (approach, grasp, lift, transport, placement) visually and
> functionally acceptable for this entrance-test prototype, and confirmed
> the earlier decorative-hand visual/collision defect (Phase 4D) remains
> fixed. **The stricter internal engineering-quality bar introduced in
> Phase 4E/4F — max 3D grasp slip <=10mm — is NOT met** (measured: ~25.9mm,
> a real, unresolved kinematic limitation, not a faked or hidden result).
> This is a change in acceptance **policy** (prototype-level pass with a
> documented limitation), not a change to any measured data, log,
> threshold, or test. See `reports/phase4f-human-acceptance-decision.md`
> for the full decision record, and `reports/phase4d-physics-integrity-audit.md`,
> `reports/phase4e-gripper-integrity-repair.md`, and
> `reports/phase4f-orientation-grasp-stabilization.md` for the underlying
> investigation/repair history.

## Demonstration Videos

Playable directly on this page (rendered via GitHub's inline `<video>`
support — open this file on github.com, not in a local Markdown preview).
These are the same decode-verified files in `submission/videos/`; the
slide deck could only show static frames from them.

**Task 1 — pick-and-place** (fixed-base G1, physical gripper, no teleport):

<video src="https://github.com/chyang-nycu/VLA-Entrance-test/raw/main/submission/videos/task1_third_person.mp4" controls width="480">Task 1, third-person view</video>

Third-person, full episode (approach → grasp → lift → transport → lower →
release → retreat), 640x480, 12.0s — the video reviewed and accepted in the
Phase 4F human acceptance decision above.

<video src="https://github.com/chyang-nycu/VLA-Entrance-test/raw/main/submission/videos/task1_onboard_rgb.mp4" controls width="240">Task 1, onboard camera</video>

Onboard `head_cam` view (torso-mounted, near head height), 160x120
(padded to 160x128 by H.264), 13.3s — the same camera stream used as a
policy-facing observation in the VLA data pipeline.

<video src="https://github.com/chyang-nycu/VLA-Entrance-test/raw/main/submission/videos/optional_debug_before_after.mp4" controls width="480">Before/after: decorative-hand defect vs. corrected gripper</video>

Side-by-side diagnostic: **left** — the vendor's decorative hand mesh
visibly clipping through the cube (Phase 4D); **right** — the corrected
gripper after the visual/collision fix (Phase 4E). 960x360, 4.3s.

**Task 2 — object-conditioned selection** (same scene, different task
specification, different selected object; see `reports/task2-language-selection.md`):

<video src="https://github.com/chyang-nycu/VLA-Entrance-test/raw/main/submission/videos/task2_red_instruction.mp4" controls width="480">Task 2, red instructed</video>

"Pick up the **red** cube..." — red cube grasped and placed; green
distractor undisturbed throughout. 640x480, 13.2s.

<video src="https://github.com/chyang-nycu/VLA-Entrance-test/raw/main/submission/videos/task2_green_instruction.mp4" controls width="480">Task 2, green instructed</video>

Same physical arrangement, "Pick up the **green** cube..." instead — green
cube grasped and placed; red distractor undisturbed throughout. 640x480,
13.2s.

## Layout

- `vendor/unitree_mujoco/` — pinned upstream simulator (git submodule; see
  below). Never modified.
- `setup/` — host bootstrap scripts and the Phase 1 smoke-test script/report.
- `tasks/g1_pick_place/` — task-local scenes, gripper MJCF, and controllers.
  All modifications to the stock G1 model live here, never in vendor.
- `reports/` — phase audit/results reports.
- `docs/work_log.md` — running chronological log of what was done.
- `tests/` — automated `unittest` checks per phase.
- `logs/` — small JSON evidence artifacts (actuator inventories, contact
  results, trial logs).
- `artifacts/` — small evidence images (e.g. smoke-test screenshot).

## Vendor dependency

`vendor/unitree_mujoco` is a pinned git submodule of
`https://github.com/unitreerobotics/unitree_mujoco.git`, currently pinned at
commit `4134cb5dc7ff1ba7f484deda48b5274b58694519`. It is treated as read-only
upstream source. Any manipulation-specific geometry, actuators, or scenes are
added as task-local derived files under `tasks/g1_pick_place/`, never by
editing files inside `vendor/`.

## Setup

```bash
./setup/preflight_macos.sh
python3.12 -m venv .venv
.venv/bin/pip install mujoco==3.3.6 numpy imageio pillow
```

See `setup/phase1_mujoco_smoke_test.md` for the exact recorded toolchain and
package versions used so far.

## Running tests

```bash
.venv/bin/python setup/g1_mujoco_smoke.py           # Phase 1 smoke test
.venv/bin/python -m unittest tests/test_phase2_g1_audit.py -v   # Phase 2
```

```bash
.venv/bin/python -m unittest tests/test_phase3_gripper.py -v      # Phase 3 gripper structural
.venv/bin/python -m unittest tests/test_phase3_controller.py -v   # Phase 3 controller unit
.venv/bin/python -m unittest tests/test_phase3_grasp.py -v        # Phase 3/3B (historical, torque-PD; 3 FAIL by design)
.venv/bin/python -m unittest tests/test_phase3c_grasp.py -v       # Phase 3C (position-servo; passing baseline)
.venv/bin/python -m unittest tests/test_phase4a_grasp_variants.py -v  # Phase 4A (setup-variant sweep)
.venv/bin/python -m unittest tests/test_phase4b_pick_place.py -v  # Phase 4B (Task 1: complete pick-and-place)
.venv/bin/python -m unittest tests/test_phase4c_slip_audit.py -v  # Phase 4C (slip-metric audit + video capture)
.venv/bin/python -m unittest tests/test_phase4d_physics_integrity.py -v  # Phase 4D (physics-integrity audit; now fully green post-4E fix)
.venv/bin/python -m unittest tests/test_phase4e_gripper_integrity.py -v  # Phase 4E (visual/collision fix + grasp-stability repair; honest partial result)
.venv/bin/python -m unittest tests/test_phase4f_orientation_grasp.py -v  # Phase 4F (orientation-constrained IK + pad-mount fix; strict 10mm slip bar honestly still failing as a diagnostic, not project breakage)
.venv/bin/python -m unittest tests/test_phase5a_onboard_camera.py -v  # Phase 5A (onboard RGB observation camera smoke test)
.venv/bin/python -m unittest tests/test_phase5b_dataset.py -v  # Phase 5B (VLA demonstration dataset: canonical manifest, HDF5 schema, validator, replay)
.venv/bin/python -m unittest tests/test_task2_language_selection.py -v  # Task 2 (language-conditioned two-object selection)
```

`test_max_slip_while_grasped_still_exceeds_tightened_bar` and
`test_grasp_stability_pass_4f_is_honestly_false` in
`tests/test_phase4f_orientation_grasp.py` are diagnostic tests: they assert,
and pass by asserting, that the strict internal 10mm engineering-quality
slip bar is not met (locked to the measured value, `assertAlmostEqual` to
4 places) — the same regression-diagnostic pattern used for Phase 3's
historical failures in `tests/test_phase3_grasp.py`. The default test
command (`python -m unittest discover -s tests`) exits clean with zero
unexpected failures; this strict-bar limitation is a documented, honestly-
passing diagnostic, not a broken build.

## Phase status

- **Phase 1 — MuJoCo smoke test**: complete. See
  `setup/phase1_mujoco_smoke_test.md`.
- **Phase 2 — Manipulation feasibility audit**: complete. See
  `reports/phase2-manipulation-audit.md`. Conclusion: stock G1 has no
  actuated gripper/hand; a task-local physical parallel gripper is required.
- **Phase 3 — Fixed-base grasping baseline (torque-PD + resolved-rate DLS-IK)**:
  FAILED after 3 tuning iterations. See `reports/phase3-grasping-baseline.md`.
- **Phase 3B — Controller stabilization budget (same architecture, evidence-driven tuning)**:
  FAILED after 3 attempts. See `reports/phase3b-controller-stabilization.md`.
- **Phase 3C — Position-servo controller architecture (bounded position servos +
  waypoint IK + pelvis/torso weld + implicitfast integrator)**: **PASSES**,
  at attempt 3C-2, deterministic 5/5. This is the fixed-base, torso-constrained
  upper-body manipulation baseline controller. See
  `reports/phase3c-position-servo-baseline.md`.
- **Phase 4A — Grasp setup-variant evaluation**: **3/5 variants succeed
  (60%), 9/15 trials (60%)**, using Phase 3C's unmodified configuration —
  zero global adjustments needed. The 2 failing variants were correctly
  predicted unreachable by a pre-run IK feasibility check (a reachability
  limitation, not a grip-strength one). See `reports/phase4a-grasp-variants.md`
  and `HANDOFF.md` for the full 5-variant table, feasibility methodology,
  and known limitations (asymmetric success envelope, likely related to a
  wrist kinematic singularity near the nominal point; IK tolerance and
  gripper gains remain specific to the current cube mass/friction).
- **Phase 4B — Task 1 complete pick-and-place** ("pick up the red cube and
  place it in the blue target area"): **Stage A (nominal) PASSES,
  deterministic 5/5**, after 3 evidence-driven tuning attempts on
  transport/lower trajectory shape and timing only (no gripper/arm/grasp-
  approach parameter was touched). **Stage B: 2 of the 3 Phase-4A-reachable
  variants complete the full task (nominal, x-0.03); y+0.03 grasps
  successfully but narrowly misses the tight target-XY placement margin** —
  a genuine, honestly reported placement-accuracy limit, not a dropped
  grasp. Supported-envelope success 2/3 (67%); original five-variant
  coverage 2/5 (the 2 Phase-4A-unreachable variants remain excluded,
  unchanged). See `reports/phase4b-task1-pick-place.md` and `HANDOFF.md`
  for the full attempt log, target-selection evidence, and limitations
  (measured cube slip under sustained transport load, the tight nominal
  placement margin).
- **Phase 4C — Task 1 evidence hardening and video capture**: audited Phase
  4B's reported cube-slip metric and found it conflated genuine grasp-phase
  slip with post-release TCP-cube separation (the gripper opening and the
  arm retreating after a successful placement) — root cause confirmed, not
  guessed: `post_release_tcp_cube_separation_m` (0.149 m for nominal) is
  within 2 cm of the old `max_cube_slip_m` (0.156 m). **Corrected genuine
  grasp-phase slip is 3.3–5.4 cm** (phase- and variant-dependent), not
  14.97–15.62 cm. No pass/fail outcome changed — slip was never an
  acceptance criterion. Also produced a decode-verified nominal-episode
  video and 3 still frames. See `reports/phase4c-task1-evidence.md` for the
  full audit (old vs. corrected definitions, per-variant table, synthetic
  unit tests) and `logs/phase4c_slip_audit.json` for raw data.
  `reports/phase4b-task1-pick-place.md` is unedited; this is a
  measurement/reporting correction only, no controller or physics change.
- **Phase 4D — Physics-integrity investigation (diagnosis only, no fix)**:
  triggered by a user visual inspection reporting (1) the hand/fingers
  visibly pass through the cube, (2) the cube visibly falls instead of
  being lifted. **Defect (1) CONFIRMED and reproduced**: the vendor G1
  model's own decorative, non-articulated, collision-free `right_rubber_hand`
  visual mesh was never suppressed by this project's scene generator and
  spatially overlaps the real functional gripper's location, so it renders
  as clipping through the cube on every grasp — a scene-authoring/visual
  defect, not a contact-solver defect. **Defect (2) NOT reproduced**: an
  isolated table-support test and a fresh instrumented rerun both confirm
  correct physics (cube settles on the table correctly; genuinely rises
  0.108 m with real nonzero contact force). No fix was implemented in this
  phase — see `reports/phase4d-physics-integrity-audit.md` for the full
  gate-by-gate audit, test classification, and evidence (video + zoomed
  stills reproducing the defect). All prior numeric results stand
  unmodified; only the *visual/interpretive* "Task 1 success" claim is
  under review pending a scene fix and revalidation.
- **Phase 4E — Gripper visual/collision repair and grasp-stability
  redesign**: **visual/collision defect (Phase 4D) FIXED** — the vendor
  decorative hand mesh is removed from Task 1's scene only, a palm and
  distinguishably-colored fingers were added; `Phase4DDecorativeHandOverlapTest`
  now genuinely passes. **Grasp stability substantially improved but the
  tightened acceptance bar is NOT met**: 3 evidence-driven repair attempts
  (visual+pad geometry; LIFT trajectory smoothing+gain raise; further
  gain/waypoint increase) reduced max slip while grasped from as much as
  5.19 cm to 2.05 cm and raised the worst-instant bilateral safety factor
  from 0.054x to >=1.0x — but the new <=10 mm max-slip-while-grasped
  criterion is still ~2x over, and a cube-center-within-pad-vertical-
  overlap criterion also still fails. No 4th attempt was made; no threshold
  was loosened. Because the required gate was not met, Stage B (the 3
  reachable variants) was not run as an authorized evaluation — see
  `reports/phase4e-gripper-integrity-repair.md` for the full evidence,
  attempt log, and the informational-only Stage B numbers. New evidence
  video: `artifacts/phase4e_task1_corrected.mp4` (+ close-up
  `artifacts/phase4e_task1_closeup.mp4`), both with a burned-in overlay of
  task state/height gain/live slip/contact force. **Task 1 success remains
  NOT restored, pending human video review and a future phase closing the
  remaining slip gap.**
- **Phase 4F — Orientation-constrained grasp stabilization**: authorized
  after human review of `phase4e_task1_closeup.mp4` did not approve Task 1
  (grasp still slides ~20.5mm during HOLD). Added `solve_ik_waypoint_oriented()`
  (opt-in, `controller_3c.py`) — a null-space orientation objective aligning
  the wrist's local Z axis to vertical, with an evidence-derived
  `ORIENT_TOL_RAD ≈ 7 deg`. 3 evidence-driven attempts: (1) the orientation
  objective alone barely helped (47.4 -> 44.5 deg, slip unchanged); (2)
  increasing its weight found a genuine kinematic reachability conflict at
  the grasp waypoint (leveling the wrist there requires 30-70mm of position
  error — reverted); (3) a measured finger-pad mounting correction
  (`FINGER_MOUNT_FIX_QUAT`, `gripper_scene.py`) reduced the targeted
  contact-z-offset metric by ~17% but did not reduce overall slip (25.9mm
  final). **7 of 11 tightened acceptance criteria pass; the grasp-quality-
  critical ones (max slip <=10mm, pad-vertical-overlap, release/placement)
  fail.** Deliberately isolated from Phase 4B/4C/4D/4E's shared pipeline —
  `write_grasp_scene_4b()`/`run_trial_pick_place()`'s defaults are
  byte/physics-unchanged (confirmed: the full 139-test pre-existing suite
  passes unmodified); Phase 4F uses its own opt-in `write_grasp_scene_4f()`
  and `use_oriented_ik=True`. New evidence videos:
  `artifacts/phase4f_task1_full.mp4` and
  `artifacts/phase4f_bilateral_contact_view.mp4` (diagnostic view
  perpendicular to the measured jaw axis). See
  `reports/phase4f-orientation-grasp-stabilization.md` for the full
  per-attempt evidence and 11-criteria table (note: that report's own "7 of
  11 pass" summary line is an arithmetic slip — the correct count from its
  own criteria table and the raw JSON is 8 of 11; see
  `reports/phase4f-human-acceptance-decision.md`, which uses the corrected
  count and does not edit the original report).
- **Phase 4F human acceptance decision**: after reviewing
  `artifacts/phase4f_task1_full.mp4` and
  `artifacts/phase4f_bilateral_contact_view.mp4`, **Task 1 is accepted as
  "prototype task completed with a documented grasp-slip limitation."** The
  strict <=10mm engineering-quality slip bar is explicitly **not** claimed
  to pass (measured ~25.9mm); human visual/functional review of the
  complete task is what passes. No log, threshold, test, or historical
  report was changed to reach this decision — see
  `reports/phase4f-human-acceptance-decision.md`.
- **Phase 5A — Onboard RGB observation smoke test**: adds one task-local
  onboard RGB camera (`head_cam`, rigidly mounted on `torso_link` -- the
  vendor model has no separate head/neck body), as a future VLA policy
  observation source. Purely additive: does not touch Task 1's controller,
  gains, thresholds, or any historical file (confirmed: zero diff on
  `controller.py`/`controller_3c.py`/`run_pick_place.py`/`gripper_scene.py`
  and every prior scene/report/artifact). Verified across a **full fresh
  nominal Task 1 episode** (not just the reset frame): the red cube and
  blue target are both visible at every required phase (RESET, PREGRASP,
  APPROACH, first bilateral contact, LIFT, HOLD, TRANSPORT_ABOVE_TARGET,
  LOWER, OPEN, VERIFY_TASK_SUCCESS); rendering is confirmed read-only with
  respect to physics (bit-identical trial outcome with vs. without the
  camera active); camera pose tracks its fixed parent body to solver
  precision (~0.19mm over a 13.2s episode). Measured performance: ~18.5
  fps combined sim+render throughput; **10 Hz recommended** for a future
  policy observation rate (~2x headroom under the measured ceiling). See
  `reports/phase5a-onboard-camera.md` for the full camera specification,
  the camera-placement iteration record (an early pose was accidentally
  placed inside the robot's own head mesh -- documented, not hidden), and
  storage estimates.
- **Phase 5B — VLA demonstration dataset prototype**: builds and verifies
  the full HDF5 data pipeline (canonical config manifest, collector,
  validator, replay) against exactly **three** episodes — `nominal` and
  `x_minus_0.03` (both successful Task 1 completions) and `x_plus_0.03` (a
  labeled, excluded failure episode). Uses the same non-oriented,
  Phase 4E-lineage pipeline as every prior phase; does not retune Task 1's
  controller, gains, or success thresholds (cross-checked in
  `tests/test_phase5b_dataset.py::TestTask1CriteriaUnchanged`). A new
  `data/task1_canonical_config.json` manifest, hashed and verified at
  collection/validation/replay time, is the single source of truth for
  which scene/controller/camera/thresholds are authorized; the collector,
  validator, and replay tool all fail loudly on any mismatch.
  **Disclosed deviation**: the originally-planned `y_plus_0.03` failure
  variant was re-measured under the current (post-4E) config and found to
  now *pass* deterministically (the gripper-gain/trajectory-smoothing
  improvements since Phase 4B closed its placement-margin gap); `x_plus_0.03`
  (a genuine, deterministic grasp-approach/reachability failure) was
  substituted as the labeled failure episode instead, documented rather
  than silently forced to match the original plan. See
  `reports/phase5b-data-pipeline.md` and `data/schema.md` for the full
  schema, per-episode results, validator/replay output, and this
  substitution's measured record.
- **Phase 5C — VLA action/replay fidelity fix**: diagnoses and fixes the
  root cause of Phase 5B's ~4.9cm nominal replay error. Quantified: the
  entire error was the 10Hz zero-order-hold of a single per-transition
  action sample discarding the real controller's 500Hz intra-transition
  ramp (`run_pick_place._drive_smooth` re-interpolates the commanded joint
  target on every physics step). Adds a second, execution-rate (500Hz)
  data group alongside the unchanged 10Hz policy group, and three
  distinguished replay modes: **exact execution replay** (replays the
  literal per-step applied ctrl — max TCP error ~3.7e-8m, essentially
  machine precision, vs. the targets of 1e-4rad/1e-3m) and **policy-action
  replay** (decodes only the 10Hz action stream through the same
  IK/PD primitives the real controller uses). Policy-action replay's
  **maximum** TCP error during an episode does **not** meet the ≤10mm
  target (measured ~97-98mm) — diagnosed and disclosed, not adjusted to
  pass: the stored `cartesian_target` is one static per-phase goal, so a
  decoder re-ramping toward it every 100ms takes a different path shape
  than the true multi-second waypoint ramp, even though it converges to
  the same final point (final TCP error 6.3mm, within target). New
  `data/task1_prototype_v2.hdf5` (additive — `data/task1_prototype.hdf5`
  and `reports/phase5b-data-pipeline.md` are preserved unmodified as the
  original Phase 5B evidence). See `reports/phase5c-replay-fidelity.md`
  and `data/schema_v2.md` for the full audit, per-episode numbers, and
  divergence diagnosis.
- **Phase 5D — redesigned VLA policy-action representation**: fixes Phase
  5C's remaining policy-action-replay gap (max TCP error ~97-98mm) by
  redesigning the stored action semantics, not just the decoder. Each 10Hz
  transition now stores the expert's actual commanded TCP-reference
  **delta** (world-frame position, TCP-local-frame orientation, verified
  numerically not assumed) instead of a repeated static goal. Two attempts
  (a single whole-interval delta, then a ramp-speed sweep) each reduced but
  did not close the gap — diagnosed as a genuine information gap, not a
  tuning problem: a 100ms-wide delta cannot describe a trajectory
  containing a large sub-100ms-scale reference jump (e.g. the initial
  RESET→PREGRASP step). **Attempt 3 (shipped)**: a fixed-size sub-action
  chunk (H=5 sub-deltas per transition, 50Hz) — measured **8.09mm**
  (nominal) and **5.99mm** (x_minus_0.03) max TCP error, both under the
  ≤10mm target. Exact execution replay unchanged from Phase 5C (~3.7e-8m,
  machine precision). New `data/task1_prototype_v3.hdf5` (additive —
  `data/task1_prototype.hdf5` and `data/task1_prototype_v2.hdf5` are
  preserved unmodified as historical evidence). See
  `reports/phase5d-policy-action-redesign.md` and `data/schema_v3.md` for
  the full three-attempt history and per-episode numbers.
- **Phase 5E — scaled Task 1 demonstration collection**: collects 32
  episodes (24 sampled from a pilot-calibrated continuous cube-position
  envelope, all 24 successful, split 16/4/4 train/val/test by disjoint
  `cube_dx` bands; 8 fixed diagnostic probes, of which 3 were rejected
  pre-physics by IK reachability and 1 failed physically, with the
  remaining 4 turning out to succeed) using the Phase 5D v3 action
  schema/decoder completely unchanged. **Deviation disclosed up front**:
  target position is NOT varied — the rendered blue target pad is fixed
  scene geometry with no offset parameter, and varying only the
  controller's internal target while leaving the pad fixed would produce
  physically dishonest episodes, while moving the pad itself would be a
  Task 1 geometry change the authorization prohibits (a first
  implementation attempt was caught and fully reverted before use — see
  `git diff --stat tasks/g1_pick_place/run_pick_place.py` = empty).
  **Honest replay-fidelity finding**: scaling beyond Phase 5D's original 3
  validated configs shows the ≤10mm policy-action-replay target holds for
  22/29 episodes (76%) but not uniformly — 7 episodes reach up to 22.3mm,
  attributed to large single-transition reference jumps at some sampled
  cube positions exceeding what H=5/50Hz chunking can track within 10mm
  (decoder held unchanged per authorization, not redesigned this phase).
  New `data/task1_demonstrations_v1.hdf5` (62MB, left untracked — exceeds
  the commit-size threshold; checksum/regeneration command recorded
  instead) and `data/task1_collection_spec.json` (the locked sampling
  spec, committed). See `reports/phase5e-scaled-data-collection.md` for
  the full pilot, spec, quality-gate, and validation results.
- **Phase 6 — final entrance-test submission package**: documentation/
  packaging only, no code/dataset/model changes. Produces
  `submission/entrance_test_report.md` (full report, failure narrative,
  reproduction pointer), `submission/REPRODUCE.md` (every command actually
  executed and timed in this environment), `submission/DATASET_CARD.md`,
  `submission/results_summary.json` (machine-readable headline metrics,
  each tagged measured/derived/human_reviewed/passed/failed/not_attempted),
  `submission/videos/` + `submission/video_manifest.json` (3 decode-verified
  videos), and `data/task1_demonstrations_v1_quality.json` (a read-only
  per-episode replay-fidelity audit of the Phase 5E dataset, with three
  recommended training masks — does not alter the HDF5's own labels).
  **Key finding**: of the 7 Phase 5E episodes exceeding the 10mm
  policy-replay target, all 7 first diverge at the post-release RETREAT
  phase — verified to never affect cube placement or task-success
  determination. Full suite re-run: 288 tests, 0 unexpected failures. See
  `submission/entrance_test_report.md` for the complete account.
- **Task 2 — language-conditioned two-object selection** (optional,
  time-boxed; committed as `5f119ce` on branch `task2-language-selection`,
  independently audited 2026-09-03, merged into `main`):
  extends the Task 1 scene with a second, physically-identical green cube
  (`tasks/g1_pick_place/task2_language_selection.py`) and reuses Task 1's
  own unmodified scripted controller, now able to act on a caller-specified
  cube (an additive, default-preserving parameter added to
  `run_trial_pick_place`). This is a **language-conditioned task
  specification with a privileged scripted expert**: `selected_object_id`
  is supplied directly by the task specification, never parsed from text or
  visually recognized at trial time; `parse_selected_object()` is used only
  to build labels. All 4 required configurations (red/green x
  nominal/swapped), **three repeated deterministic executions per
  configuration** (no RNG anywhere in this pipeline — not distinct seeds),
  12/12 trials pass: the selected cube is grasped and placed, the
  distractor never moves more than 1.73mm (measured as the maximum
  displacement over the entire episode, not final-minus-initial) or enters
  the target, and the onboard camera confirms both objects and the target
  are visible at reset. See `reports/task2-language-selection.md` and
  `submission/entrance_test_report.md` Section 14 for the full evidence,
  including the empirical slot-placement search (a rejected 8cm-separated
  candidate slot measured 48.7mm of real distractor displacement peaking
  during RETREAT — an uncontrolled joint-space sweep, independently
  reproduced during the audit as a genuine physical disturbance, not a
  metric bug, and excluded from the 4 final passing configurations).
  Scaled dataset collection and policy integration for this task remain not
  started; require new, explicit authorization per `HANDOFF.md`.

## Ground rules (all phases)

- Never modify `vendor/unitree_mujoco`.
- Never weld, attach, teleport, or directly overwrite object state to fake a
  grasp; grasps must be produced by physical contact and actuation.
- Task-local additions only, under `tasks/g1_pick_place/`.
