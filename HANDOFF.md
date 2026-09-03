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

## Phase 4A — grasp setup-variant evaluation (authorized 2026-09-02)

**Scope decision, binding for all future phases unless explicitly revisited:**
the current pelvis+torso-welded configuration is accepted as the MVP
baseline. It must be described exactly as "fixed-base, torso-constrained
upper-body manipulation baseline." Never describe this project's
capability as full-body or free-standing manipulation. Do not unlock the
torso during Phase 4A.

Not in scope for Phase 4A: transport, target placement, cameras, dataset
collection, language variants. The successful Phase 3C controller
parameters (`arm_kp=400, arm_kv=25`, `gripper_kp=150, gripper_kd=10`,
position-priority IK with the evidence-based 8mm tolerance, the
pelvis+torso weld, `implicitfast` integrator) must be preserved unchanged
as the one shared configuration for the entire sweep — no per-variant
gains, hand-tuned joint targets, offsets, or hidden exceptions.

Before any variant work: rerun the committed Phase 3C nominal trial once,
record the commit hash and nominal metrics, and change nothing about the
working IK/servo/gripper/state-machine parameters before doing so.

Five deterministic cube variants: nominal; +0.03m and -0.03m on one
table-plane axis; +0.03m and -0.03m on the other table-plane axis. 0.03m is
the default magnitude unless pre-run reachability analysis proves it
invalid for a given direction, in which case the chosen magnitude must be
justified up front and applied symmetrically (same magnitude both
directions on that axis). Each variant: stable ID, explicit cube pose,
fixed seed, identical robot reset state, identical controller parameters,
and PREGRASP/APPROACH/LIFT/HOLD targets generated from the variant's
observed cube pose (not hardcoded nominal coordinates). Cube pose is
written only during reset initialization, before `CubeInitGuard` locks —
no post-step cube state manipulation, no exceptions.

Pre-run feasibility check per variant (recorded, not silently skipped even
if it fails): table containment / collision-free spawn; PREGRASP IK
residual; APPROACH IK residual; joint-limit margin; expected TCP/finger-pad
alignment; accepted-as-reachable or not.

Each variant run >= 3 times. Per-variant and aggregate metrics recorded:
success count/trials, bilateral-contact rate, max cube height gain,
continuous hold time, pre-close cube displacement, TCP error, controller
saturation, failure state/reason. Success criteria unchanged from Phase
3/3C (bilateral contact; height gain >=0.08m; hold >=2.0s; finite/bounded;
physical release; no prohibited cube manipulation). Target: >=3/5 variants
succeed; report both per-variant success and total trial success rate,
honestly, whatever it is.

No per-variant tuning. If the one shared configuration scores below 3/5,
at most 2 global, evidence-driven adjustments are allowed, each applied to
all 5 variants and the entire sweep rerun after each adjustment.

Test-suite hygiene: preserve Phase 3/3B's historical failure evidence and
reports unedited, but the repository's default verification command must
not terminate with 3 unexplained failures — either mark the 3 legacy tests
as explicit expected-failures, or convert them into regression diagnostics
asserting the legacy controller stays below its documented (failing)
threshold. Do not lower acceptance thresholds anywhere; do not mark any
Phase 3C or Phase 4A test as an expected failure; do not delete historical
tests; do not rewrite historical reports. The final summary must
distinguish passing current-controller tests, explicit legacy
expected-failures/diagnostics, and any actual unexpected failure.

Outputs: `tasks/g1_pick_place/run_variant_sweep.py`;
`tests/test_phase4a_grasp_variants.py`; `logs/phase4a_grasp_variants.json`;
`reports/phase4a-grasp-variants.md` (must include an exact 5-row variant
table and aggregate results); artifacts/video evidence where practical;
updates to `README.md`, this file, and `docs/work_log.md`.

Commit discipline: exclude the `logs/g1_mujoco_smoke.json` timestamp-only
diff and the pre-existing Go2w submodule artifact; do not track large
generated video unless already-established repository policy tracks it
(the existing small `.gif` evidence files are fine, following the Phase 3C
precedent). Show staged files and diff summary before committing. One
separate local commit, not pushed. Stop after reporting Phase 4A results —
do not begin transport automatically.

## Phase 4A outcome (2026-09-02) — 3/5 variants succeed, 0 adjustments needed

**Fixed-base, torso-constrained upper-body manipulation baseline** (binding
framing, see Scope decision above). Ran the unmodified Phase 3C winning
configuration (`arm_kp=400, arm_kv=25, gripper_kp=150, gripper_kd=10`,
unchanged) across 5 deterministic cube variants (nominal; +/-0.03m on each
table-plane axis), 3 trials each, one shared scene/config for all of them.
Full detail: `reports/phase4a-grasp-variants.md`. Raw data:
`logs/phase4a_grasp_variants.json`. Sweep script:
`tasks/g1_pick_place/run_variant_sweep.py`.

**Result: 3/5 variants succeed (nominal, x-0.03, y+0.03), 2/5 fail
(x+0.03, y-0.03) — 60% variant rate, 60% trial rate (9/15), target (>=3/5)
met on the first, unmodified sweep. Zero of the 2 allowed global
adjustments were used** — none were needed, and none were manufactured to
"use the budget."

Both failures were caught by the pre-run IK-based feasibility check
(APPROACH residual exceeding the evidence-based 8mm tolerance: 27.1mm for
x+0.03, 8.43mm for y-0.03) *before* any trial ran, and independently
confirmed by the state machine's own dynamic settle gate refusing to
advance past `SETTLE_APPROACH` in every trial of both variants — the two
failure signals agree, and no cube was ever displaced or mishandled in the
rejected variants; the arm simply could not reach those positions to
within tolerance. This is a *reachability* limitation, distinct from
Phase 3/3B's tracking-oscillation failure.

Test-suite hygiene: Phase 3's 3 historical nominal-acceptance failures
(`tests/test_phase3_grasp.py`) were converted to regression diagnostics
that assert the documented failure numbers persist (not lowered, not
marked xfail-and-forgotten, not deleted; `reports/phase3-grasping-baseline.md`
unedited). Full regression this session: 58 passing current-controller
tests (Phase 2/3-structural/3C/4A) + 3 passing legacy diagnostics + 0
unexpected failures.

Limitations: the evaluated envelope is asymmetric (-x and +y succeed, +x
and -y fail) and consistent with, but not fully explained by, the
previously-identified wrist singularity near the nominal point — a full
workspace Jacobian-conditioning map was not computed (out of scope for
this phase). Per this file's scope, transport, target placement, cameras,
dataset collection, and language variants remain unimplemented and
unauthorized for this phase.

## Phase 4B — Task 1 complete pick-and-place (authorized 2026-09-02)

**Terminology correction (binding):** this is still Task 1 ("pick up the
red cube and place it in the blue target area"), not Task 2. Task 2
(cameras, dataset collection, language-conditioned variants, policy
integration) remains unauthorized.

Preserve unchanged: all Phase 1-4A reports/commits/tests; the exact Phase
3C controller configuration (`arm_kp=400, arm_kv=25, gripper_kp=150,
gripper_kd=10`, position-priority IK, 8mm tolerance, pelvis+torso weld,
`implicitfast` integrator); the physical parallel gripper; the fixed-base,
torso-constrained scope (torso stays welded); `CubeInitGuard` and every
anti-cheating constraint; vendor integrity; the documented asymmetric
grasp-reachability limitation (x+0.03/y-0.03 not re-attempted here).

Scene: add a static blue target pad to the task-local scene only
(`tasks/g1_pick_place/gripper_scene.py`'s `write_grasp_scene_4b`) — a
jointless geom (implicitly world-fixed), no equality/weld/tendon/actuator/
force referencing the cube or the pad; placement success judged from cube
state, never from color/rendering. Target dimensions/pose documented in
`gripper_scene.py` and below.

Target selection: reachability analysis (IK residual at
TRANSPORT_ABOVE_TARGET, LOWER_TO_TARGET, RETREAT) run before simulation
over a grid of candidate offsets. Chosen target: `(-0.11, +0.07)` m offset
from the cube's nominal position — table pos `(0.22, -0.08)` — with the
largest reachability margin found (max residual 3.6mm vs. the 8mm
tolerance), >=0.10m lateral separation from the cube (meaningful
transport, not directly under the nominal lift position), within the
table's edge margin, and in the same (-x, +y) direction Phase 4A already
found reachable (away from the documented wrist singularity). Rejected
candidates and their residuals are recorded in
`reports/phase4b-task1-pick-place.md`.

State machine (extends Phase 3C's grasp stages unchanged): RESET ->
PREGRASP -> SETTLE_PREGRASP -> APPROACH -> SETTLE_APPROACH -> CLOSE ->
VERIFY_BILATERAL_CONTACT -> LIFT -> HOLD -> TRANSPORT_ABOVE_TARGET ->
SETTLE_ABOVE_TARGET -> LOWER_TO_TARGET -> SETTLE_LOWER -> OPEN ->
VERIFY_RELEASE -> RETREAT -> VERIFY_TASK_SUCCESS -> DONE/FAILED. Transport
aborts (reports FAILED, does not fake success) if bilateral contact is
lost or cube height falls below a safe threshold; cube slip relative to
the gripper (measured in the gripper's own rotating frame, not raw world
coordinates) is logged for every trial.

Objective task-success detector (all required, continuously, for a
settling dwell — never on first boundary crossing): cube lifted
>=0.08m at some point; released by both finger pads; cube center within
the target pad's footprint margin; cube supported by the table/pad, not
the gripper; cube linear and angular speed below documented thresholds;
all of the above true continuously for the dwell window; robot retreated
without disturbing the cube.

Evaluation order: Stage A (nominal only, <=3 evidence-driven tuning
attempts, transport/lower/release trajectory parameters only — no gripper
gain, arm servo gain, or grasp-approach parameter changes without
quantitative proof of a grasp regression, and none occurred). Stage B
(only after Stage A passes): the same fixed target against Phase 4A's 3
reachable variants (nominal, x-0.03, y+0.03), >=3 trials each, one shared
configuration, no per-variant tuning. The 2 pre-declared unreachable grasp
variants (x+0.03, y-0.03) are excluded from Task 1's primary success
denominator and listed separately as known-unsupported setup variants;
both supported-envelope success and original five-variant coverage are
reported.

Outputs: `tasks/g1_pick_place/run_pick_place.py`;
`tests/test_phase4b_pick_place.py`; `logs/phase4b_pick_place_trials.json`;
`reports/phase4b-task1-pick-place.md`; updates to `README.md`, this file,
and `docs/work_log.md`.

Commit discipline: exclude the `logs/g1_mujoco_smoke.json` timestamp-only
diff and the pre-existing Go2w submodule artifact. Show staged files and
diff summary before committing. One separate local commit, titled exactly
"feat: complete G1 Task 1 pick and place", not pushed. Stop after
reporting Task 1 results — do not begin cameras, dataset collection, or
language-conditioned Task 2 automatically.

## Phase 4B outcome (2026-09-02) — Task 1 complete, Stage A PASS, Stage B 2/3

**Fixed-base, torso-constrained upper-body manipulation baseline**
(binding framing, unchanged). Stage A: the nominal pick-and-place
succeeded after 3 evidence-driven tuning attempts on transport/lower
trajectory shape and timing only (final: 40-waypoint ramped transport over
2.0s, 60-waypoint ramped lower over 2.0s; no gripper/arm/grasp-approach
parameter was ever touched) — deterministic 5/5 identical reruns. Stage B:
2 of the 3 Phase-4A-reachable variants (nominal, x-0.03) complete the full
task; y+0.03 grasps successfully but narrowly misses the target-XY
placement margin (20.4mm vs. the 15mm pad margin) — a genuine, honestly
reported placement-accuracy limit, not a dropped grasp (bilateral contact
was retained for all 3 variants' full transports). Supported-envelope
success: 2/3 (67%); original five-variant coverage: 2/5 (the 2
Phase-4A-unreachable variants, x+0.03 and y-0.03, are excluded from the
denominator and listed separately, unchanged from Phase 4A). Full detail,
attempt log, and both result tables: `reports/phase4b-task1-pick-place.md`.
Raw data: `logs/phase4b_pick_place_trials.json`.

Test-suite hygiene unchanged from Phase 4A: the same 3 Phase 3 legacy
tests remain regression diagnostics (not re-touched); full suite this
session: 94 tests, 0 unexpected failures (58 pre-existing + 36 new Phase
4B tests, 3 of the 58 being the unedited legacy diagnostics).

No `.mp4`/GIF demo artifact was produced this phase: the environment has
no `ffmpeg`/`imageio-ffmpeg` available for video encoding, and installing
a new dependency was not authorized — noted honestly rather than skipped
silently or faked.

Per this file's scope, cameras, dataset collection, language-conditioned
variants, and policy integration (Task 2) remain unimplemented and
unauthorized.

## Phase 4C — Task 1 evidence hardening and video capture (authorized 2026-09-02)

Goal: audit the reported cube-slip metric, produce a viewable nominal
demonstration, and package Task 1's results so they are understandable
without running code. No retuning of the successful controller; no Task 2
work; `reports/phase4b-task1-pick-place.md` preserved unedited.

### Slip-metric audit — root cause confirmed

Phase 4B's `max_cube_slip_m = 0.1561555402975266` (nominal) conflated
genuine grasp-phase slip with **post-release TCP-cube separation**: the
code kept updating "slip" on every step for the rest of the trial once a
grasp reference was captured, with no check that the cube was still being
carried — so `OPEN`/`RELEASE_SETTLE`/`VERIFY_RELEASE`/`RETREAT`/
`VERIFY_TASK_SUCCESS` (all post-release, cube intentionally no longer
held) were silently included. Confirmed directly, not guessed:
`post_release_tcp_cube_separation_m = 0.14860671167299208` for the nominal
trial — within 2 cm of the old figure. The TCP-local-frame transform
itself (`R_tcp^T @ (cube_pos - tcp_pos)`) was already correct — rotation
was handled properly (confirmed by a synthetic unit test: pure TCP
rotation with a rigidly-attached cube produces exactly zero slip) — the
defect was entirely about the time window, not the math.

Corrected metrics, gated on the existing `carrying` signal so slip is only
counted while the gripper is closed and the cube is grasped:
`grasp_reference_offset_tcp_frame`, `max_slip_during_lift`,
`max_slip_during_transport`, `max_slip_during_lower`, `slip_at_release`,
and `post_release_tcp_cube_separation_m` (never called slip). The legacy
`max_cube_slip_m` field is retained unmodified so the historical 0.156 m
figure stays reproducible. **Corrected genuine grasp-phase slip: 3.3–5.4
cm** (nominal: lift 0.032 m, transport 0.045 m, lower 0.052 m, release
0.020 m — vs. the old, wrong, single 0.156 m figure). No pass/fail outcome
changed anywhere — slip was never an acceptance criterion, and `task_pass`
for all Stage A/Stage B trials is unchanged from `363aa83`.

This was a measurement/reporting fix only: `arm_kp=400.0`, `arm_kv=25.0`,
`gripper_kp=150.0`, `gripper_kd=10.0`, and every trajectory parameter are
byte-identical to the committed Phase 4B configuration (verified via diff
against `363aa83`). 10 new tests added (`tests/test_phase4c_slip_audit.py`):
3 synthetic math unit tests (pure translation, pure rotation, known 5mm
displacement), 6 tests confirming post-release isolation on the real
nominal trial, 1 confirming the fix holds on the y+0.03 variant's
different (placement-failure) code path.

### Video capture

`imageio-ffmpeg` installed into the existing project venv (user-local,
no admin/system-wide install) — worked on the first real attempt, so no
GIF fallback was needed. `artifacts/phase4b_task1_nominal.mp4`: 640x480,
29.41 fps (physics-stride-synchronized to real-time playback), 375
frames, 12.75s duration, 229,060 bytes, decode-verified (frame count and
fps confirmed by reading the file back). Fixed (non-tracking) third-person
camera; shows G1, red cube, table, and blue target pad throughout. 3 still
frames captured: grasp, mid-transport, final released-in-target. Full
detail: `reports/phase4c-task1-evidence.md`.

### Test-suite hygiene

Unchanged pattern from Phase 4A/4B: the 3 Phase 3 legacy regression
diagnostics remain untouched. Full suite this session: 104 tests, 0
unexpected failures (94 pre-existing + 10 new).

Per this file's scope, Task 2 (cameras as a data-collection feature,
dataset pipeline, language-conditioned variants, policy integration)
remains unimplemented and unauthorized.

## Notice: previous Task 1 success results under review

**Previous Task 1 success results are under review following visual detection
of collision/support inconsistencies.**

*(Superseded 2026-09-02 by the Phase 4F human acceptance decision below:
Task 1 is now accepted at the entrance-test-prototype level, with a
documented grasp-slip limitation. This notice is left in place, unedited,
as the historical record of why the review began.)*

## Phase 4D — physics-integrity investigation (diagnosis only, no fix)

Triggered by a user visual GUI inspection of the committed Task 1 pipeline
(commit `dfeec9e`) reporting: (1) the hand/fingers visibly pass through the
cube, (2) the cube visibly falls instead of being lifted. Full investigation:
`reports/phase4d-physics-integrity-audit.md`.

**Diagnosis:**

- **Defect (1): CONFIRMED and reproduced.** Root cause: the vendor G1 model's
  own decorative `right_rubber_hand` visual mesh (fixed to
  `right_wrist_yaw_link`, non-articulated, `contype="0" conaffinity="0"` by
  the vendor's own authoring) was never suppressed by this project's scene
  generator. Its local-frame extent (`[0.0415, 0.1733]` m) overlaps this
  project's real, physically-simulated finger pads (`[0.088, 0.112]` m), so
  it renders as visibly clipping through the cube on every grasp, even
  though it carries zero collision force. This is a scene-authoring/visual
  defect, not a contact-solver defect. Reproduced in
  `artifacts/phase4d_failure_reproduction.mp4` and
  `artifacts/phase4d_collision_debug_close.png`/`_hold.png`.
- **Defect (2): NOT reproduced** by direct instrumentation of current
  committed code. An isolated cube/table settling test (no robot motion,
  3s of gravity) shows correct support (settles within 0.22mm, contact
  force balances the cube's weight to 4 significant figures, never passes
  through the table). A fresh instrumented rerun of the real nominal trial
  shows genuine 0.108m height gain with real nonzero bilateral contact
  force during HOLD. Most likely explanation: the same decorative-mesh
  confusion in (1) makes the real (correctly functioning) grip look
  precarious/asymmetric to a viewer, or an intended
  `LOWER_TO_TARGET`/`OPEN` transition was misread as an uncontrolled drop.
  Not treated as a second confirmed bug.
- **Why 104/104 tests passed anyway**: every "real end-to-end" test (50 of
  76 across the Phase 3C/4A/4B/4C test files, re-checked class-by-class) is
  genuinely fresh-simulated, not cached-log-based — but none of them, in
  any category, ever inspected whether an unrelated, collision-free visual
  geom spatially overlaps the cube. That is a pure rendering property with
  zero prior test coverage, not a false-positive or a mocked result. New
  test `Phase4DDecorativeHandOverlapTest` (in
  `tests/test_phase4d_physics_integrity.py`) closes that specific gap and
  **fails on purpose** against current code, to keep the confirmed,
  unfixed defect visible in the default test run. Do not "fix" this test by
  loosening it — fix the scene instead, in a future authorized phase.

**No fix was implemented in this phase.** No historical report, log, or
commit was modified or reinterpreted with different numbers — all prior
numeric results stand as accurate descriptions of what the simulation
computed; only the *visual/interpretive* claim of "Task 1 success" is under
review pending a scene fix (recommended: hide/remove the vendor's decorative
hand meshes in the task-local scene generator, the same copy-and-edit
pattern already used for every other task-local scene change) and
revalidation. Vendor model, controller gains, thresholds, and trajectories
are all unchanged from `dfeec9e`.

Full test suite after this phase: 117 tests, 1 failure (the intentional,
documented `Phase4DDecorativeHandOverlapTest` case) — this is expected and
correct, not a regression to silently fix.

Per this file's scope, no fix, no dataset collection, and no Task 2 work
were started in Phase 4D. A revalidation/fix phase requires new, explicit
authorization.

## Notice: previous Task 1 success results are under review

**Previous Task 1 success results are under review following visual
detection of collision/support inconsistencies.** See "Phase 4D" above for
the diagnosis and "Phase 4E" below for the repair attempt and its honest,
incomplete outcome. Task 1 is NOT considered valid again until a human has
visually reviewed `artifacts/phase4e_task1_corrected.mp4` AND the
remaining quantitative grasp-stability gap documented below is closed in a
future, separately authorized phase.

## Phase 4E — gripper visual/collision repair and grasp-stability redesign

Authorized after a user frame-by-frame review of
`artifacts/phase4d_failure_reproduction.mp4` found: (1) the cube does rise
in world Z; (2) it visibly slides downward relative to the gripper
throughout HOLD -- an unstable near-drop; (3) the white vendor decorative
hand penetrates the cube; (4) the dark physical collision gripper is
visually inconsistent with the displayed hand. Full evidence, attempt log,
and honest outcome: `reports/phase4e-gripper-integrity-repair.md`.

**Section A (visual/collision correspondence): FIXED.** The vendor's
`right_rubber_hand` decorative mesh is removed from the task-local Task 1
scene only (`write_grasp_scene_4b`, via new `_build_grasp_tree` parameters
`finger_pad_half`/`apply_phase4e_gripper_visuals` -- Phase 3/3B/3C's own
scenes are unaffected, confirmed by unchanged sha256/height-gain values). A
palm backing plate and two distinguishably-colored finger pads were added.
`tests/test_phase4d_physics_integrity.py`'s previously-failing-on-purpose
`Phase4DDecorativeHandOverlapTest` now genuinely passes (updated to check
the real generated scene, not a permanently-true static fact about the
vendor STL).

**Sections B/C (grasp-stability evidence and redesign): substantially
improved, but the tightened acceptance bar is NOT met.** Evidence-first
diagnosis found two real root causes: (1) LIFT used a one-shot position-
servo step (never updated when the same defect was fixed for TRANSPORT/
LOWER in Phase 4B), producing a worst-instant bilateral safety factor of
only 0.146x (below 1x -- physically-caused slip, not a measurement
artifact); (2) `solve_ik_waypoint` has no orientation term, so wrist roll
drifts freely and shows up as up to +/-5cm of finger-pad/cube-center
vertical misalignment. 3 authorized repair attempts (visual+pad-geometry;
LIFT smoothing+gain raise; further gain/waypoint increase) reduced max
slip while grasped from as much as 5.19cm to 2.05cm and raised the worst-
instant safety factor from 0.054x to consistently >=1.0x -- genuine,
measured improvement -- but the new <=10mm max-slip-while-grasped
criterion is still not met (20.5mm, ~2x over), and the cube-center-within-
pad-vertical-overlap criterion also still fails. No 4th attempt was made;
no threshold was loosened. `GRIPPER_KP_4E=320.0`/`GRIPPER_KD_4E=20.0`,
`LIFT_DRIVE_S_4E=1.5`/`LIFT_N_WAYPOINTS_4E=30`, and
`FINGER_PAD_HALF=(0.012, 0.006, 0.030)` are now `run_pick_place.py`'s/
`write_grasp_scene_4b`'s defaults.

**Stage D gate not met, so Stage B was not run as an authorized
evaluation** (informational-only numbers are in
`logs/phase4e_gripper_integrity.json`, explicitly not to be cited as an
authorized Stage B result).

New evidence: `artifacts/phase4e_task1_corrected.mp4` (full episode, burned-
in overlay of task state/height gain/live slip/contact force) and
`artifacts/phase4e_task1_closeup.mp4` (tight side view). Neither
`artifacts/phase4d_failure_reproduction.mp4` nor
`artifacts/phase4b_task1_nominal.mp4` was overwritten.

**Task 1 success is NOT restored.** Per the authorization: human visual
approval of the new videos is required, and even then, the quantitative
max-slip-while-grasped gap must be closed in a future, separately
authorized phase before any success claim is reinstated.

## Phase 4F — orientation-constrained grasp stabilization

Authorized after human review of `phase4e_task1_closeup.mp4` did not
approve Task 1: decorative-hand fix confirmed good and the cube genuinely
lifts/places, but the cube still slides ~20.5mm downward relative to the
gripper (above the 10mm requirement), bilateral opposing finger placement
was not visually clear, and the grasp appeared offset below/away from the
gripper center. Full evidence, per-attempt log, and honest outcome:
`reports/phase4f-orientation-grasp-stabilization.md`.

**Section A: `solve_ik_waypoint_oriented()`** (new, additive, in
`controller_3c.py`) adds a null-space orientation objective to Phase 3C's
position-priority IK, aligning the wrist's local Z axis to world vertical
(the axis this project's finger pads treat as "tall"), leaving yaw about
that axis free. `ORIENT_TOL_RAD ≈ 7.0 deg`, derived the same evidence-based
way `IK_POS_TOL` was in Phase 3C. **Deliberately opt-in**: `write_grasp_scene_4b()` and `run_trial_pick_place()`'s default (`use_oriented_ik=False`)
are unchanged, byte/physics-identical to Phase 4E -- confirmed by the full
139-test pre-existing suite passing unmodified. Phase 4F's own path uses
the new `write_grasp_scene_4f()` and `use_oriented_ik=True`.

**3-attempt budget, all real, evidence-driven, none forcing a pass:**
1. Null-space orientation objective (`orient_weight=0.6`): orientation
   residual at APPROACH improved only marginally (47.4 -> 44.5 deg); slip
   did not improve.
2. Increased weighting (0.6 -> 2.0, plus a co-primary-stacked diagnostic):
   found a genuine kinematic reachability conflict -- reaching the 7 deg
   orientation tolerance at this Cartesian point requires 30-70mm of
   position error (missing the cube). Real full trial FAILED even earlier
   (at SETTLE_APPROACH, no grasp attempted). Reverted to `orient_weight=0.6`.
3. Measured finger-pad mounting correction (`FINGER_MOUNT_FIX_QUAT` in
   `gripper_scene.py`): a fixed, measured rotation of each finger body
   (not its position) about the wrist's local Y (jaw) axis, calibrated to
   the real converged nominal APPROACH configuration. Reduced the targeted
   contact-z-offset metric by ~17% (43.8mm -> 36.5mm) but did **not**
   reduce overall slip (rose slightly to 25.9mm) -- slip is evidently also
   driven by dynamic effects (impact/settling), not solely static contact
   geometry.

**Result: 7 of 11 tightened acceptance criteria pass; the grasp-quality-
critical ones (max 3D slip <=10mm, cube-center-within-pad-vertical-overlap,
physical release/placement) fail.** Deterministic across 5 reruns (bit-
identical). Stage B (3 Phase-4A-reachable variants) run informationally
only, per HANDOFF.md's own Stage-A-gate requirement -- all 3 fail the same
gate, consistently.

New evidence: `artifacts/phase4f_task1_full.mp4` (full episode, overlay of
state/position-residual/orientation-residual/slip/vertical-slip/contact-
force) and `artifacts/phase4f_bilateral_contact_view.mp4` (diagnostic view
perpendicular to the measured jaw axis, camera azimuth chosen from an
8-way sweep specifically so neither finger pad is fully hidden behind the
cube). Neither overwrites any earlier phase's artifacts.

**Task 1 success is NOT restored.** Per the authorization, this phase
stops after producing the videos; Task 1 remains under review pending
human visual approval, and the documented ~26mm slip gap most likely
requires a further, separately-authorized phase (trajectory/waypoint
redesign, not another tuning attempt within this exhausted budget).

## Phase 4F human acceptance decision — Task 1 restored as prototype, with a documented limitation

Full record: `reports/phase4f-human-acceptance-decision.md`.

Human visual review of `artifacts/phase4f_task1_full.mp4` and
`artifacts/phase4f_bilateral_contact_view.mp4` found the task execution
(approach/grasp/lift/transport/lower/release/retreat) visually and
functionally acceptable for this entrance-test prototype, and confirmed
the decorative-hand visual/collision defect (Phase 4D) remains fixed.

**Decision: Task 1 is restored as "prototype task completed with a
documented grasp-slip limitation."**

This is an acceptance-**policy** decision, separating two questions Phase
4E/4F's single <=10mm bar had collapsed together:

- Strict engineering-quality grasp (max 3D slip while grasped <=10mm,
  cube-center-within-pad-vertical-overlap): **FAIL** — measured 0.02592m,
  unchanged, not claimed to pass.
- Entrance-test prototype task completion (physically honest pick-and-
  place, no scripted assistance, judged acceptable by direct human video
  review): **PASS, with the slip limitation documented.**
- Human visual review of the Phase 4F videos: **PASS.**

No log, threshold, test assertion, or historical report (Phase 3 through
4F) was altered to reach this decision. `tests/test_phase4f_orientation_grasp.py`'s
`test_max_slip_while_grasped_still_exceeds_tightened_bar` and
`test_grasp_stability_pass_4f_is_honestly_false` continue to assert, and
pass by asserting, that the strict bar is not met — this is the same
regression-diagnostic pattern already used for Phase 3's historical
failures, not a new mechanism.

**Correction note (this project's own transparency standard, not a
historical-report edit):** `reports/phase4f-orientation-grasp-stabilization.md`
states "7 of 11 [criteria] pass." Re-deriving the count from that same
report's own criteria table (3 named failures: max-slip, pad-vertical-
overlap, release/placement) and from `logs/phase4f_orientation_grasp.json`'s
`criteria_grasp_stability_4f` dict gives 8 pass / 3 fail, not 7/11. This is
an arithmetic slip in the original write-up, not a data discrepancy — the
underlying measured numbers are unchanged and correct. Per this phase's
instruction not to alter historical reports, the original file is left as
written; `reports/phase4f-human-acceptance-decision.md` uses the corrected
count.

Per this phase's instructions: no controller retuning was performed, no
prior test/threshold/log/report/video was altered, and Task 2 (cameras as
a data-collection feature, dataset pipeline, language-conditioned variants,
policy integration) remains unimplemented and unauthorized.

## Phase 5A — onboard RGB observation smoke test

Full record: `reports/phase5a-onboard-camera.md`.

Adds one task-local onboard RGB camera (`head_cam`), rigidly mounted on
`torso_link` (the vendor G1 model has no separate head/neck body -- the
head mesh is a static geom on `torso_link`; confirmed by direct
inspection). Purely additive: `write_grasp_scene_5a()` is a new function
that re-parses `write_grasp_scene_4b()`'s own written output and adds one
`<camera>` element -- `write_grasp_scene_4b()` itself, every other scene
generator, `controller.py`, `controller_3c.py`, and `run_pick_place.py`
are all confirmed unchanged (zero diff against commit `c9352ac`). The
existing third-person evidence camera (`mujoco.MjvCamera`, constructed at
render time, no MJCF element at all) is untouched and unrelated.

**Configuration used, and why (a documented decision):** Phase 4F's
orientation-IK configuration does not reach `OPEN`/`VERIFY_TASK_SUCCESS`
in its own logged behavior (fails at `SETTLE_LOWER`) -- confirmed by
re-running it here. Since HANDOFF.md requires onboard visibility evidence
at exactly those two states, this smoke test uses Phase 4B/4C's
originally-completing configuration (`use_oriented_ik=False`) instead,
sharing every physical constant (cube, target, gripper, arm gains) with
Phase 4F's configuration. This does not retune or re-authorize either
configuration.

**Camera pose iteration (documented, not hidden):** the first candidate
position, designed in world coordinates, was written into the MJCF's
`pos` attribute without converting to the parent body's local frame (a
`<camera>` child's `pos` is local, not world) -- this placed the camera
~0.85m too high, looking down at the whole scene from well above the
robot. Fixed by explicit frame conversion. The corrected position then sat
*inside* the head mesh's own bounding volume, causing visible self-
occlusion; the final pose moves forward of the mesh's own front-face
extent (computed from the actual STL bounding box, not guessed).

**Full-episode visibility (not just the reset frame)**: verified against a
fresh nominal Task 1 trial -- red cube visible at all 11 required
checkpoints (RESET, PREGRASP, APPROACH, CLOSE, first bilateral contact,
LIFT, HOLD, TRANSPORT_ABOVE_TARGET, LOWER_TO_TARGET, OPEN,
VERIFY_TASK_SUCCESS); blue target visible at all 8 required pre-placement
checkpoints (its near-disappearance at OPEN/VERIFY_TASK_SUCCESS is the
cube physically covering the pad after a successful placement -- expected,
not a defect). Rendering confirmed read-only w.r.t. physics (bit-identical
trial outcome with/without the camera active). Camera pose confirmed
constant to solver precision (~0.19mm over 13.24s) under the current
fixed-base weld, which is soft/finite-stiffness, not bit-exact rigid --
documented rather than assumed.

**Performance**: ~18.5 fps combined sim+render throughput measured
directly; 10 Hz recommended for a future policy observation rate (~2x
headroom under the measured ceiling), with 20 Hz noted as viable only for
offline (non-real-time) dataset generation.

**Evidence**: `tasks/g1_pick_place/camera_observation.py`,
`tasks/g1_pick_place/record_onboard_camera_episode.py`,
`tests/test_phase5a_onboard_camera.py` (19 tests),
`logs/phase5a_camera_smoke.json`, `artifacts/phase5a_head_camera.mp4`
(onboard view only, 160x120, 29.41fps, 390 frames, 13.26s, decode-verified),
`artifacts/phase5a_head_camera_frames/` (11 PNGs).

Full regression this session: **178 tests, 0 unexpected failures** (159
pre-existing unchanged + 19 new). Vendor pin unchanged at
`4134cb5dc7ff1ba7f484deda48b5274b58694519`; vendor XML confirmed to not
contain the string `head_cam` anywhere. No Task 1 controller parameter,
gain, threshold, or historical report/log/video was touched. Per this
phase's instructions, the HDF5 dataset pipeline, Task 2, model training,
and any Task 1 retuning remain out of scope and unimplemented.

## Phase 5B — VLA demonstration dataset prototype (authorized 2026-09-02/03)

Full record: `reports/phase5b-data-pipeline.md`, schema reference
`data/schema.md`.

**Canonical manifest first, as required**: `data/task1_canonical_config.json`
was authored and its own content SHA-256 stamped
(`tasks/g1_pick_place/canonical_config.py`) before any collector/validator/
replay code existed. It pins the scene generator (`write_grasp_scene_5a`,
built on `write_grasp_scene_4b` — the Phase 4E-lineage visual/collision-
corrected gripper, never the Phase 4F orientation-IK path, which does not
complete placement), the non-oriented IK controller (`use_oriented_ik=false`),
arm/gripper gains, success thresholds, and the camera specification — all
read from the live code, not asserted. The camera's parent body
(`torso_link`) was reconfirmed by direct `ElementTree` parse of the
generated scene, and its true rendered resolution (160x120) was confirmed
against the actual `mujoco.Renderer` output array shape, resolving the
Phase 5A video's 160x128 ffprobe report as H.264 encoder padding, not the
real observation size. The collector, validator, and replay tool all load
this manifest and fail loudly (`ManifestMismatchError`) on any live
mismatch — exercised directly with a tampered dataset copy, which was
correctly rejected.

**Pre-collection baseline** (`logs/phase5b_baseline_check.json`): all 5
required checks passed on the first run — corrected gripper geometry
present, vendor decorative hand absent, Task 1 physically completes,
onboard RGB renders non-blank, live config hash matches manifest.

**Disclosed deviation from the original plan**: `y_plus_0.03` was
specified as the labeled placement-failure episode, based on Phase 4B/4C
history (grasp succeeds, placement fails at a 20.4mm margin). Re-measuring
it under the *current* canonical config (Phase 4E's gripper-gain increase
and LIFT/TRANSPORT/LOWER trajectory smoothing, both added after Phase 4B)
found it now **passes** deterministically (xy_err=2.07mm) — those
controller improvements closed the placement-margin gap that variant used
to expose. A sweep of nearby/alternate offsets found no remaining
placement-margin-only failure zone under the current config at all: every
tested offset either succeeds cleanly or fails at `SETTLE_APPROACH` (a
grasp-reachability failure, before any grasp is attempted). `x_plus_0.03`
(a genuine, deterministic `SETTLE_APPROACH` failure, previously documented
in Phase 4A/4B as IK-unreachable) was substituted as the collection's
failure episode instead — documented as a grasp/reachability failure, not
mislabeled as a placement failure to match the original plan. Full
measured record in `data/task1_canonical_config.json`'s
`instruction_variants` object.

**Dataset**: `data/task1_prototype.hdf5` (3 episodes: `nominal` and
`x_minus_0.03` successful, `x_plus_0.03` a labeled/excluded failure),
1,740,909 bytes, SHA-256 `d30250fac4fc0fb4dcd2bc9972dbc43a600afc73ef70fca386dd89b6c919454f`,
committed directly (small enough). Schema: `policy_observations/` (RGB,
joint positions/velocities, TCP pose, gripper state — the declared VLA
policy-input group, raw frames with no video-text overlay) and
`privileged/` (cube/target pose, contact, state-machine phase — a
*separate* top-level group, never nested under `policy_observations/`).
Transition convention `observation_t -> action_t -> physics_substeps ->
observation_t+1`, recorded at 10 Hz against 500 Hz physics (50 substeps/
transition); `len(observations) == len(actions) + 1` is a structural
invariant of the recording loop, not a runtime coincidence.

**Validator**: all required checks pass, including independently
*recomputing* each episode's success by rerunning the deterministic
simulation (not trusting the stored flag) — all three agreed.

**Replay**: action replay (through the same `CubeInitGuard` pre-lock
boundary, never overwritten again) deviates from the original recording by
up to 4.9cm TCP / 15.2° max joint on the two successful episodes — expected
and disclosed, since the 10 Hz recorded action stream is a zero-order-hold
downsample of the original 500 Hz fine-grained waypoint ramp
(`run_pick_place._drive_smooth`); the reachability-failure episode deviates
by <1mm (it never reaches a ramped segment). Observation-only visualization
replay (no physics stepped) also verified for all three episodes. Replay
correctly refuses (`ManifestMismatchError`) when pointed at a
manifest-hash-mismatched dataset copy.

Full regression this session: **214 tests, 0 unexpected failures** (178
pre-existing unchanged + 36 new). No Task 1 controller parameter, gain,
threshold, or historical report/log/video was touched (verified: manifest's
copied success thresholds cross-checked byte-for-byte against
`run_pick_place.py`'s live constants in
`tests/test_phase5b_dataset.py::TestTask1CriteriaUnchanged`). Vendor pin
unchanged. Per this phase's instructions, scaled collection, language-
conditioned Task 2, and model training remain out of scope and
unimplemented; stopped after exactly 3 episodes.

## Phase 5C — VLA action/replay fidelity fix (authorized 2026-09-03)

Full record: `reports/phase5c-replay-fidelity.md`, schema reference
`data/schema_v2.md`, raw numbers `logs/phase5c_replay_fidelity.json`.
`data/task1_prototype.hdf5`, `reports/phase5b-data-pipeline.md`, and
commit `67ccf89` are preserved unmodified as the original Phase 5B
prototype evidence — this phase is entirely additive (`_v2` files).

**Root cause of Phase 5B's 4.9cm replay error, quantified**: the arm's
commanded joint target actually changes every physics step (500Hz) during
`LIFT`/`TRANSPORT_ABOVE_TARGET`/`LOWER_TO_TARGET` (`run_pick_place._drive_smooth`
ramps linearly between IK-solved waypoints), but Phase 5B's dataset only
sampled one instantaneous end-of-interval value per 100ms transition, and
its replay held that single sample constant (zero-order hold) across the
whole interval — discarding the real intra-transition ramp entirely.
Replaying the *same* nominal episode's literal 500Hz `applied_ctrl` trace
instead drops the max TCP replay error from 4.87cm to 3.65e-8m (~1.3
million times smaller) — proving this ZOH gap was the entire error, with
no other meaningful contributor.

**Two-rate schema** (`data/task1_prototype_v2.hdf5`): unchanged 10Hz
`policy/` group (observations, high-level actions, RGB — RGB not
duplicated elsewhere) plus a new 500Hz `execution/` group (one row per
physics step: literal applied ctrl, joint/TCP/cube state, transition-index
mapping into its parent policy transition). `execution_hz=500` was chosen
because that is the actual rate the controller's set-point changes at,
confirmed by reading `_drive_smooth`'s source, not picked arbitrarily.

**Three replay modes, both honestly measured**:
- **Exact execution replay** (replays literal per-step `applied_ctrl`):
  max TCP error 3.4-4.4e-8m across all three episodes, 4-5 orders of
  magnitude tighter than the 1e-4rad/1e-3m targets. Not literally
  bit-exact (floating-point accumulation), but the achieved tolerance is
  measured and justified, not loosened.
- **Policy-action replay** (decodes only the 10Hz action stream through
  the same IK/PD primitives, re-ramping toward the recorded static goal
  every 100ms instead of holding it constant): the **maximum** TCP error
  during an episode does **not** meet the ≤10mm target (measured
  ~97-98mm for the two successful episodes) — diagnosed, not adjusted to
  pass. Root cause: `cartesian_target` stores one static per-phase goal
  repeated across every transition inside a multi-second phase (e.g. all
  ~40 transitions of `TRANSPORT_ABOVE_TARGET` share one value); a decoder
  with no waypoint-index information re-ramps toward that same distant
  goal every 100ms, producing a different (faster, more direct) path
  shape than the true multi-waypoint ramp, even though it converges to
  the same final point (final TCP error 6.3mm on both successful
  episodes, within target). Closing this would require a richer
  per-transition action (e.g. the next incremental waypoint, not the
  phase's final goal) — explicitly out of scope for this phase, which
  authorized a two-rate dataset and honest replay validation, not an
  action-representation redesign.

**Tests**: `tests/test_phase5c_replay_fidelity.py`, 17 new tests covering
all 7 required transition-alignment properties plus a deliberately
shifted-action tamper test (`np.roll()`s a recorded action array by one
transition in a copied file; `validate_dataset_v2` correctly flags it).

Full regression this session: **231 tests, 0 unexpected failures** (214
pre-existing unchanged + 17 new). Task 1 controller/gains/geometry/
success-thresholds/camera and the canonical manifest's content are
byte-identical to Phase 5B (verified via `git diff`); vendor pin
unchanged; no push. Per this phase's instructions, scaled collection and
Task 2 remain out of scope and unimplemented.

## Phase 5D — redesigned VLA policy-action representation (authorized 2026-09-03)

Full record: `reports/phase5d-policy-action-redesign.md`, schema reference
`data/schema_v3.md`, raw numbers `logs/phase5d_policy_replay.json`.
`data/task1_prototype.hdf5`, `data/task1_prototype_v2.hdf5`, their reports,
and commits `67ccf89`/`965b947` are preserved unmodified as historical
evidence — this phase is entirely additive (`_v3` files).

**Root cause of Phase 5C's remaining ~97-98mm policy-replay error**: the
stored `cartesian_target` was one static per-phase goal repeated across
every transition of a multi-second phase; this phase replaces it with
`policy/actions/{tcp_delta_position,tcp_delta_orientation,gripper_command}`
— the expert's actual commanded TCP-reference DELTA over each 100ms
interval, derived via forward kinematics of `execution/arm_joint_target`
(never from privileged cube/target state). Position delta is world frame
(measured to coincide with the robot-base frame to ~0.19mm, the pelvis
weld's independently-measured softness from Phase 5A); orientation delta
is TCP-local body frame using MuJoCo's native `mju_subQuat`/
`mju_quatIntegrate` pair, with the right/body-frame composition order
verified numerically (not assumed) against a hand-constructed
left-vs-right-composition discriminator test.

**Three attempts, in the authorized order** (full numbers in the report):
1. Single whole-interval delta, one IK solve + one linear ramp across
   100ms: 23.6mm max (down from 96.9mm, still over target). Diagnosed:
   `_drive_segment` phases (PREGRASP, RETREAT, ...) issue a fixed target
   ONCE and hold it for many subsequent transitions, so the true trial had
   far more than 100ms to converge to it — forcing the decoder to fully
   reach the same target within one interval starves the position-servo of
   settle time it actually had.
2. Ramp-speed sweep (immediate step through the full 50-step ramp): no
   single speed worked everywhere — fast ramps fixed the PREGRASP jump
   (4.7mm) but made a later RETREAT-phase jump much worse (~49.5mm), and
   vice versa. Diagnosed as a genuine information gap (one 100ms delta
   cannot describe a trajectory with a large sub-100ms reference change),
   not a tuning problem.
3. **Shipped**: fixed-size sub-action chunk, H=5 sub-deltas per 10Hz
   transition (50Hz sub-action rate), each decoded with its own one-IK-
   solve-plus-one-ramp over 10 physics steps. Measured **8.09mm** (nominal)
   and **5.99mm** (x_minus_0.03) max TCP error — both under the ≤10mm
   target. H=2 (43.1mm) still failed; H=10 (7.36mm) was measured but not
   shipped since H=5 already meets the target with margin.

**Exact execution replay unchanged from Phase 5C** (~3.4-4.4e-8m across all
three episodes — `execution/` group untouched by this phase).

**x_plus_0.03** (pre-grasp reachability failure): 12.77mm max policy-replay
error — the ≤10mm bar applies only to successful episodes per this phase's
authorization; this episode's requirement (identical failure stage/label)
is met (`SETTLE_APPROACH`, `stored_success=False` reproduced), reported
honestly rather than folded into the successful-episode claim.

**Tests**: `tests/test_phase5d_policy_actions.py`, 25 new tests covering
synthetic-delta recovery, rotation-composition/frame verification,
causality (auxiliary field reconstructed purely from its own transition's
execution data), no off-by-one shift, no-phase-goal-repetition (plus a
tamper test that injects the old bug pattern and confirms the validator
catches it), manifest/decoder-hash mismatch rejection, and the shipped
10mm regression on both successful episodes.

Full regression this session: **256 tests, 0 unexpected failures** (231
pre-existing unchanged + 25 new). Task 1 controller/gains/geometry/
success-thresholds/camera and the canonical manifest's content are
byte-identical to Phase 5B/5C (verified via `git diff`); vendor pin
unchanged; no push. Per this phase's instructions, scaled collection
remains out of scope and unimplemented.
