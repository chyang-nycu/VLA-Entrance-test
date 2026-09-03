# G1 MuJoCo Entrance-Test Project

Task-local manipulation experiments built on top of the official Unitree
`unitree_mujoco` simulator, targeting the Unitree G1 humanoid. All work is
staged in phases; each phase must pass its acceptance test before the next
begins.

**Current capability, precisely stated: a fixed-base, torso-constrained
upper-body manipulation baseline attempting Task 1** ("pick up the red
cube and place it in the blue target area"). The pelvis and torso are
rigidly welded to the world; only the right arm and gripper move. This is
not full-body or free-standing manipulation. **Task 1 success is NOT
currently claimed** — see the notice below.

> **Previous Task 1 success results are under review following visual
> detection of collision/support inconsistencies.** Phase 4D confirmed a
> visual/collision defect (fixed in Phase 4E). Phase 4E's grasp-stability
> repair substantially reduced, but did not eliminate, a real grasp
> instability found by human video review. Phase 4F added an orientation-
> constrained IK objective and a measured finger-pad mounting correction,
> diagnosed the root cause as a genuine kinematic reachability conflict at
> the grasp waypoint, and still did not close the gap (max slip while
> grasped: ~26mm vs. a 10mm requirement). Task 1 is not considered valid
> again until a human visually approves `artifacts/phase4f_task1_full.mp4`
> and `artifacts/phase4f_bilateral_contact_view.mp4` AND the remaining slip
> gap is closed in a future, separately authorized phase. See "Phase
> 4D"/"Phase 4E"/"Phase 4F" below, `reports/phase4d-physics-integrity-audit.md`,
> `reports/phase4e-gripper-integrity-repair.md`, and
> `reports/phase4f-orientation-grasp-stabilization.md`.

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
.venv/bin/python -m unittest tests/test_phase4f_orientation_grasp.py -v  # Phase 4F (orientation-constrained IK + pad-mount fix; honest partial result)
```

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
  per-attempt evidence and 11-criteria table. **Task 1 success remains NOT
  restored**, pending human video review and a further, separately
  authorized phase.
- **Task 2 (cameras, dataset collection, language-conditioned variants,
  policy integration)**: not started; requires new, explicit authorization
  per `HANDOFF.md`.

## Ground rules (all phases)

- Never modify `vendor/unitree_mujoco`.
- Never weld, attach, teleport, or directly overwrite object state to fake a
  grasp; grasps must be produced by physical contact and actuation.
- Task-local additions only, under `tasks/g1_pick_place/`.
