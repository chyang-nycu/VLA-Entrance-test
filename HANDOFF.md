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
- No cube weld, attachment, teleport, mocap manipulation, or applied `xfrc`
  used to assist grasping — the grasp must be physically produced.
- **Initialization boundary (clarified 2026-09-02):** writing the cube
  free-joint `qpos`/`qvel` during reset initialization — after
  `mj_resetData` and strictly before the first `mj_forward`/`mj_step` of a
  trial — is allowed, and is the mechanism for selecting a deterministic
  initial condition (e.g. the 5 position variants). It is equivalent to
  hardcoding the position in the MJCF. After the first physics step of a
  trial, none of the following are permitted, for any reason, for the rest
  of that trial: direct cube `qpos`/`qvel` writes, teleportation,
  weld/attachment, mocap manipulation, or applied `xfrc` to the cube. This
  boundary must be enforced by an explicit test, not left implicit.
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

## Phase 3 outcome (2026-09-02) — FAIL, recorded and not to be rewritten

Built per spec above (`gripper_scene.py`, `controller.py`,
`run_grasp_test.py`, `tests/test_phase3_{gripper,controller,grasp}.py`,
`logs/phase3_grasp_trials.json`). Gripper and controller structural/unit
tests pass (14/14). The nominal grasp-and-lift trial does **not** pass after
3 documented tuning iterations; stopped per the tuning budget rather than
attempting a 4th iteration or a scripted grasp constraint. Full quantitative
analysis: `reports/phase3-grasping-baseline.md` (do not edit retroactively —
it is a historical record of attempts 1-3). Committed at `f5ce62d`.

Root cause identified: a single `(Kp=180, Kd=18)` PD gain pair applied
uniformly across all 7 right-arm joints, whose actuator torque limits differ
5x (shoulder/elbow +/-25 N*m vs wrist pitch/yaw +/-5 N*m), produces
multi-joint Cartesian tracking oscillation (several cm) during
pregrasp/approach that physically shoves the cube out of position before the
gripper closes.

## Phase 3B — controller stabilization budget (separate from Phase 3, 2026-09-02)

Phase 3B is a new, separately-budgeted attempt to fix the Phase 3 root cause.
It does not reopen or renumber Phase 3's 3 failed iterations; those remain
the historical record in `reports/phase3-grasping-baseline.md`.

Hypothesis: uniform PD gains are inappropriate given the arm joints' 5x
differing actuator force limits; wrist actuators saturate and produce the
Cartesian oscillation that displaces the cube before closure.

Required before Attempt 3B-1 (instrumentation + baseline evidence):

1. Instrument each right-arm joint: torque limit; commanded torque before
   clipping; applied torque after clipping; saturation fraction; RMS and max
   joint tracking error; contribution to TCP position/orientation error.
2. Record baseline metrics from the existing (failed) Phase 3 controller.
3. Re-verify actuator-to-joint and qvel index mappings.
4. Verify whether gravity/Coriolis bias compensation consumes most of any
   joint's torque authority.

Up to 3 new evidence-driven attempts, in order, each only if the previous
failed:

- **3B-1**: per-joint gain scaling based on actuator torque authority (scale
  proportional gain by `torque_limit_i / 25 N*m` relative to the existing
  +/-25 N*m-joint gains as reference; scale damping more conservatively,
  e.g. by the square root of the proportional-gain scale). Must first
  resolve whether `ctrl` units are actually joint torque units (actuator
  gear/transmission) before applying this formula — do not hardcode it
  blindly if the mapping says otherwise.
- **3B-2**: reduce Cartesian trajectory speed/acceleration (especially final
  approach), smooth interpolation, require TCP to settle within a
  position/velocity tolerance before transitioning to CLOSE.
- **3B-3**: one final bounded adjustment using only the measured per-joint
  saturation/tracking evidence from the previous attempts. May not change
  cube geometry, move the cube toward the gripper, enlarge success
  tolerances, or introduce attachment.

Per-attempt required records: parameter vector; trajectory duration; TCP
RMS/max error; per-joint saturation fraction; cube displacement before
CLOSE; finger contact sequence; lift height; continuous hold duration; exact
failure state and reason. All logged in
`logs/phase3b_controller_tuning.json`; narrative + analysis in
`reports/phase3b-controller-stabilization.md`.

Acceptance criteria are unchanged from Phase 3 (5 criteria above), plus: no
post-initialization cube-state manipulation (see Initialization boundary
clause above).

Nominal variant only, first. The 5-position-variant sweep stays gated on
nominal success, same as Phase 3. If nominal succeeds: rerun >= 3 times from
the same seed to confirm determinism, rerun all structural/controller tests,
then a separate local checkpoint commit. If all 3 Phase 3B attempts fail,
stop again with quantitative evidence — do not proceed to a 4th attempt or
to Task 2.

## Phase 3B outcome (2026-09-02) — FAIL, recorded and not to be rewritten

Initialization boundary is now enforced in code (`CubeInitGuard` in
`run_grasp_test.py`, raises past the first `mj_step`; 4 passing tests in
`tests/test_phase3_grasp.py::CubeInitGuardTest`). Ctrl-to-torque mapping
resolved: all 7 right-arm motors are confirmed 1:1 joint-torque (gear
`[1,0,0,0,0,0]`, `mjTRN_JOINT`) -- the `torque_limit_i/25` formula applied
directly, no unit conversion needed. Gravity/Coriolis feedforward confirmed
not to be the bottleneck (<10% of any joint's torque budget at rest).

All 3 evidence-driven attempts FAIL the nominal trial:

- **3B-1** (per-joint Kp/Kd scaled by torque authority): fixed the targeted
  wrist joints' saturation as intended, but the torque-agnostic DLS-IK
  reassigned kinematic load onto shoulder_yaw/elbow, making global tracking
  much worse (TCP RMS error 0.061 -> 0.215 m) and losing cube contact
  entirely.
- **3B-2** (3B-1 gains + settle-before-close gating): no improvement; the
  settle gate never converged within its time budget, confirming this is a
  tracking-convergence problem, not a trajectory-speed problem.
- **3B-3** (baseline PD gains reverted + new torque-weighted DLS-IK via
  `solve_dls_ik(..., dls_weights=...)`, `None` preserves original
  behavior): closest attempt -- regained contact, near-baseline TCP
  tracking -- but wrist saturation only mildly reduced and height gain
  still far short (0.0005 m of the required 0.08 m).

Updated root cause: uniform PD gains and a torque-agnostic IK are two
separate, interacting problems; fixing either alone does not fix the
coupled 7-joint system. Full quantitative analysis, per-attempt tables, and
the blockers for any future attempt: `reports/phase3b-controller-stabilization.md`
(do not edit retroactively). All raw metrics: `logs/phase3b_controller_tuning.json`.
Committed separately from Phase 3 (see git log) -- do not proceed to a 4th
attempt or to Task 2 without a new, explicitly authorized budget.

## Phase 3C — controller-architecture replacement (authorized 2026-09-02)

Phase 3C is authorized as a separate architecture change, not a 4th Phase 3B
tuning attempt. Phase 3 and Phase 3B reports, logs, commits, and failed
results are historical and must not be edited or rewritten. The goal is no
longer to tune the torque-PD + DLS-IK architecture from Phase 3/3B; replace
it with the simplest physically honest classical controller that can
validate the task and environment.

Time budget: at most 4 focused implementation/tuning attempts, at most
~4 hours. Stop when the nominal acceptance test passes reproducibly (5/5
same-seed reruns), or stop with quantitative evidence if the budget is
exhausted.

Keep unchanged: fixed pelvis (equality weld); physical parallel gripper;
physical cube contacts/friction; the `CubeInitGuard` initialization-boundary
enforcement (no cube weld/attach/teleport/mocap/`xfrc`/post-step qpos-qvel
writes, for any reason); vendor model byte-for-byte unchanged.

Architecture changes:

1. Replace task-local right-arm torque motors with bounded MuJoCo position
   servos (or an equivalent task-local bounded position-servo layer).
   Retain explicit actuator force limits and joint limits; finite gains and
   damping; log actual actuator forces and saturation; the robot must not
   become kinematic (physics must still simulate real dynamics); arm `qpos`
   is never written directly during simulation (mirrors the cube rule).
2. Redesign IK target generation: position-priority TCP IK; constrain
   orientation only as much as necessary to align the gripper with the
   cube (don't over-constrain yaw/wrist-roll if not needed); joint-limit
   avoidance; a nominal-arm-posture null-space objective; explicit
   convergence tolerances; compute waypoints offline/per-segment rather
   than high-frequency resolved-rate IK if that's more stable.
3. Diagnose reachability before executing: solve PREGRASP, APPROACH,
   CLOSED-LIFT, and HOLD waypoint IK up front; report residual error for
   each; reject unreachable targets before simulation; log solved TCP and
   finger-pad positions relative to the cube.
4. Conservative state machine: `RESET -> PREGRASP -> SETTLE_PREGRASP ->
   APPROACH -> SETTLE_APPROACH -> CLOSE -> VERIFY_BILATERAL_CONTACT -> LIFT
   -> HOLD -> LOWER -> OPEN -> DONE/FAILED`. Must not advance from APPROACH
   unless TCP velocity and position error are both below tolerance.
5. Before attempting LIFT, require: both finger pads contact the cube;
   contacts on opposite cube sides; finger closing velocity near zero or
   grasp width stabilized; cube not already displaced outside the grasp
   corridor.

Four-attempt budget (each only if the previous failed):
- **3C-1**: position-priority IK + bounded position servos (first working
  implementation of the new architecture).
- **3C-2**: adjust only servo gains/damping, using measured tracking and
  saturation evidence.
- **3C-3**: adjust only grasp geometry/waypoint alignment, using contact
  evidence.
- **3C-4**: one final evidence-driven adjustment, changing one identified
  factor.

Do not enlarge the cube, finger pads, success tolerance, or friction merely
to force a pass, unless the original physical parameter is demonstrably
unrealistic — any such change must be explicitly justified and reported,
not applied silently.

Acceptance criteria (unchanged from Phase 3/3B): bilateral physical finger
contact; cube lift >= 0.08 m; off-table continuous hold >= 2.0 s; bounded
finite controller output; physical release after opening; no manipulation
of cube state after initialization (per the boundary clause above).

If nominal passes: rerun from the same seed >= 5 times, require 5/5
deterministic success; rerun all Phase 1-3 structural tests; capture a GUI
video if possible; new local checkpoint commit; stop before full
pick-and-place and report.

Outputs: `reports/phase3c-position-servo-baseline.md`;
`logs/phase3c_attempts.json`; updated tests (without weakening historical
Phase 3/3B acceptance-criteria tests — add new ones for the new
architecture, preserve old failure tests where practical as historical
record); update to this file and `docs/work_log.md`.

Note: an uncommitted timestamp-only diff in `logs/g1_mujoco_smoke.json`
(from an incidental Phase 1 rerun during Phase 3B verification) is
unrelated to Phase 3C and must not be included in the Phase 3C commit, nor
deleted/rewritten without explicitly reporting the action taken.

## Phase 3C outcome (2026-09-02) — PASS at attempt 3C-2

**The fixed-base grasp-and-lift acceptance gate is now met.** Nominal trial
passes all 5 criteria (both pads contact; height gain 0.1084 m >= 0.08 m;
continuous hold 3.504 s >= 2.0 s; finite/bounded outputs; physical release
after opening) and is bit-for-bit deterministic across 5/5 reruns from the
same seed. Full detail, per-attempt evidence, and limitations:
`reports/phase3c-position-servo-baseline.md` (do not edit retroactively).
All raw metrics: `logs/phase3c_attempts.json`. Video:
`artifacts/phase3c_grasp_demo.gif`.

Architecture replaced (not tuned) from Phase 3/3B: 7 right-arm torque
motors -> bounded MuJoCo `<position>` servos (real per-joint force limits
retained); continuously-resolved DLS-IK -> waypoint-based position-priority
IK (solved once per motion segment, null-space joint-limit avoidance +
posture objective, evidence-based 8mm tolerance); pelvis-only weld ->
pelvis+torso weld (Phase 2's "fixed-base" assumption was previously only
nominally true -- the torso/waist was unpowered and free to swing under
arm reaction torque); default explicit-Euler integrator -> `implicitfast`
(explicit Euler was numerically unstable with stiff position-servo gains
at this timestep, independent of the physical parameters).

Attempt 3C-1 (first working implementation, historical Phase 3/3B gripper
gains kp=40/kd=2): state machine ran cleanly to DONE, bilateral contact
achieved, but grip strength was insufficient to hold through the LIFT
transient -- height gain 0.0497 m, hold 0.198 s, then dropped. Attempt 3C-2
(gripper gain only, kp=150/kd=10, chosen as the smallest sweep value
clearing the threshold with margin, evidence: gripper force-limit was not
the bottleneck, gain was) passed outright. Attempts 3C-3/3C-4 were not
needed.

Phase 3/3B's historical failure tests (`tests/test_phase3_grasp.py`, 3
nominal-acceptance failures) still reproduce identically -- the old
torque-PD architecture was not modified, only superseded by a new code
path for Phase 3C. All regression tests across Phase 2/3/3B/3C pass or fail
exactly as documented (see the report for the full command list and
counts).

Limitations carried forward (do not skip re-reading these before any
follow-on work): the fixed-base is now pelvis+torso, more restrictive than
originally framed; the grasp waypoint sits near a real kinematic
singularity (Jacobian singular value ~5e-4) and the 8mm IK tolerance and
gripper gains are specific to this cube's position/mass/friction, not
necessarily transferable to a different one; the 5-position-variant sweep
required by Phase 3's original spec has still not been run (out of scope
for this report, would need its own reachability diagnosis per variant).
Per this file's scope, do not proceed to full pick-and-place transport,
cameras, dataset collection, language variants, or policy integration
without new, explicit authorization.
