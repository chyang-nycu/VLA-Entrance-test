# G1 MuJoCo Entrance-Test Project

Task-local manipulation experiments built on top of the official Unitree
`unitree_mujoco` simulator, targeting the Unitree G1 humanoid. All work is
staged in phases; each phase must pass its acceptance test before the next
begins.

**Current capability, precisely stated: a fixed-base, torso-constrained
upper-body manipulation baseline that completes Task 1** ("pick up the red
cube and place it in the blue target area") for its nominal cube position
and 2 of 3 tested reachable variants. The pelvis and torso are rigidly
welded to the world; only the right arm and gripper move. This is not
full-body or free-standing manipulation.

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
- **Task 2 (cameras, dataset collection, language-conditioned variants,
  policy integration)**: not started; requires new, explicit authorization
  per `HANDOFF.md`.

## Ground rules (all phases)

- Never modify `vendor/unitree_mujoco`.
- Never weld, attach, teleport, or directly overwrite object state to fake a
  grasp; grasps must be produced by physical contact and actuation.
- Task-local additions only, under `tasks/g1_pick_place/`.
