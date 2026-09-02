# G1 MuJoCo Entrance-Test Project

Task-local manipulation experiments built on top of the official Unitree
`unitree_mujoco` simulator, targeting the Unitree G1 humanoid. All work is
staged in phases; each phase must pass its acceptance test before the next
begins.

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

Phase 3 tests (added once Phase 3 lands): `tests/test_phase3_gripper.py`,
`tests/test_phase3_controller.py`, `tests/test_phase3_grasp.py`.

## Phase status

- **Phase 1 — MuJoCo smoke test**: complete. See
  `setup/phase1_mujoco_smoke_test.md`.
- **Phase 2 — Manipulation feasibility audit**: complete. See
  `reports/phase2-manipulation-audit.md`. Conclusion: stock G1 has no
  actuated gripper/hand; a task-local physical parallel gripper is required.
- **Phase 3 — Fixed-base grasping baseline**: in progress. See `HANDOFF.md`
  for the authoritative specification and `reports/phase3-grasping-baseline.md`
  once available.

## Ground rules (all phases)

- Never modify `vendor/unitree_mujoco`.
- Never weld, attach, teleport, or directly overwrite object state to fake a
  grasp; grasps must be produced by physical contact and actuation.
- Task-local additions only, under `tasks/g1_pick_place/`.
