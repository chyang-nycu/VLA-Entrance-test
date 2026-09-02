# Handoff

Reconstructed 2026-09-02. A prior handoff action for this document failed
silently (no `HANDOFF.md` was ever written to the repository); this file was
regenerated from the actually-verified repository state (git status, test
runs, and existing reports/logs) plus the Phase 3 specification given
directly by the project owner. It now supersedes any earlier, unrecorded
version.

## Verified state as of 2026-09-02

- Repository: `~/Documents/Robotics`, git-initialized, no commits before this
  handoff was written.
- `vendor/unitree_mujoco` pinned at commit
  `4134cb5dc7ff1ba7f484deda48b5274b58694519`
  (`https://github.com/unitreerobotics/unitree_mujoco.git`), configured as a
  git submodule. Only pre-existing modification: a macOS case-insensitive
  filesystem artifact on `unitree_robots/go2w/assets/terrain.STL` (unrelated
  to G1, left untouched).
- Phase 1 (MuJoCo smoke test on official G1 `scene.xml`): PASS. See
  `setup/phase1_mujoco_smoke_test.md`.
- Phase 2 (manipulation feasibility audit): PASS, 4/4 automated tests in
  `tests/test_phase2_g1_audit.py`. See `reports/phase2-manipulation-audit.md`.
  Key conclusion: no pinned G1 XML variant has an actuated gripper, Inspire
  hand, or Dex3 hand. Decision: build a task-local physical parallel gripper
  under `right_wrist_yaw_link`; use a fixed-base/fixed-pelvis MVP (free
  standing was unstable: 0.93 m pelvis drift in 2 s under small arm torques).

## Phase 3 — authoritative specification

Scope: fixed-base grasp-and-lift baseline for a single cube using a
task-local physical parallel gripper. This is the acceptance gate before any
Task 2 work (full pick-and-place, transport, cameras, dataset collection,
language variants, policy integration).

Requirements:

- Use the official pinned Unitree G1 29-actuator body/arm model.
- All modifications are task-local, under `tasks/g1_pick_place/`. Vendor
  files remain byte-for-byte unchanged.
- Fixed pelvis via a model-level MuJoCo constraint (not a runtime hack).
- Physical, symmetric parallel-jaw gripper attached to `right_wrist_yaw_link`:
  two actuated slide joints, collision-enabled finger pads, and a TCP site.
- No cube weld, attachment, teleport, mocap manipulation, or post-reset
  direct cube `qpos` writes — the grasp must be physically produced.
- Damped least-squares (DLS) Cartesian IK to drive the TCP.
- Bounded joint-space PD control for tracking.
- Explicit joint/qpos/qvel/actuator index mapping (no implicit ordering
  assumptions).
- Smooth motion sequence: pregrasp -> approach -> close -> lift -> hold ->
  lower -> open.
- Conservative position, velocity, force, and torque limits throughout.

Nominal trial acceptance (all must hold):

1. Both finger pads physically contact the cube.
2. Cube height increases by at least 0.08 m.
3. Cube remains lifted and off the table for at least 2 seconds.
4. All controller outputs remain finite and bounded.
5. After opening, the cube is physically released.

After nominal success: evaluate 5 deterministic cube-position variants.
Target >= 60% success; report the measured result honestly, whatever it is.

Tuning budget: at most 3 documented tuning iterations for the nominal
physical grasp. If it still fails after 3 iterations, stop and produce a
quantitative failure analysis instead of relaxing acceptance criteria or
introducing a scripted grasp constraint.

Out of scope for Phase 3: full pick-and-place transport, cameras, dataset
collection, language variants, policy integration.

Files (created/adapted for Phase 3, following existing task-local naming
conventions):

- Task-local derived G1 + gripper MJCF under `tasks/g1_pick_place/`.
- `tasks/g1_pick_place/controller.py` — DLS IK + bounded joint-space PD,
  index mapping, motion sequencing.
- `tasks/g1_pick_place/run_grasp_test.py` — runs nominal trial + 5 position
  variants, writes `logs/phase3_grasp_trials.json`.
- `tests/test_phase3_gripper.py` — gripper MJCF structural/physical checks.
- `tests/test_phase3_controller.py` — IK/PD/index-mapping unit checks.
- `tests/test_phase3_grasp.py` — nominal grasp-and-lift acceptance test.
- `logs/phase3_grasp_trials.json` — trial results.
- `reports/phase3-grasping-baseline.md` — results report.
- `docs/work_log.md` — updated with Phase 3 entries.

The task-local MJCF may be a documented derived copy of the vendor G1 model
if MuJoCo `<include>` semantics cannot attach child bodies inside the vendor
wrist body hierarchy directly. Vendor source files must still remain
byte-for-byte unchanged regardless of which approach is used.
