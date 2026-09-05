# G1 MuJoCo Entrance-Test Report — Two-Task Manipulation Prototype with VLA-Oriented Demonstration Data

## 1. Executive Summary

A two-task Unitree G1 manipulation prototype in MuJoCo with classical
expert control, plus a VLA-oriented demonstration dataset with validated
schema and replay instrumentation.

**Task 1** ("Pick up the red cube and place it in the blue target area") is
complete: a fixed-base, torso-constrained G1 with a task-local physical
parallel gripper performs the full approach → grasp → lift → transport →
lower → release → retreat sequence, with no weld, attachment, teleport, or
post-reset cube manipulation. I accepted it, on direct video review, as
"prototype task completed with a documented grasp-slip limitation" — a
strict 10mm grasp-slip engineering bar is **not** met (measured 25.92mm).

**Task 2** ("language-conditioned two-object selection") is also complete:
the same robot and gripper with a second, physically-identical green cube.
A privileged scripted expert receives `selected_object_id` from the task
specification — this is a **language-conditioned task specification with a
privileged scripted expert**, not natural-language parsing or
visual-language grounding. All 4 required configurations pass, 3
deterministic repeats each (12/12), with zero wrong-object placements.

**Dataset**: 32 episodes collected (28 task successes, 4 diagnostic).
Exact-execution replay matches the original physics to ~1e-5m; the shipped
policy-action decoder meets a 10mm replay-fidelity target on 18 of the 24
behaviour-cloning-pool episodes (22 of 29 dataset-wide), with every
excess-error case isolated to the post-release RETREAT phase.

**Tests**: 311 (288 pre-Task-2 + 23 new), 0 unexpected failures.

Task 3, model training, and model inference were never attempted; Task 2's
own scaled dataset collection and policy integration remain not started.

## 2. Contributions and Role

I drove the project at the level of problem formulation, system architecture, experiment design, and technical decision-making. I used an AI coding agent to accelerate implementation, testing, and documentation, while I remained responsible for defining the system, interpreting failures, and deciding what results were valid.

My main contributions were:

1. **Defined the manipulation scope and system boundary.** I reduced the broad humanoid/VLA task into a physically testable fixed-base G1 manipulation setup, including a task-local gripper, explicit physical-validity constraints, and reproducible evaluation criteria.

2. **Redesigned the controller based on measured failure.** After torque-PD and continuous IK remained unstable, I used tracking and actuator measurements to identify an architectural limitation and replaced the controller with bounded position servos and waypoint IK, producing the first deterministic grasp and the baseline used by later tasks.

3. **Built the interface from classical control to future VLA learning.** I designed the demonstration pipeline, separated learner-facing observations from privileged simulator state, and studied how expert trajectories should be represented for policy replay. This led to a chunked action representation that substantially reduced replay error.

4. **Extended the system to multi-object, task-conditioned manipulation.** Task 2 introduced a distractor and object-conditioned selection. The experiment exposed trajectory-level interference that was invisible from endpoint geometry alone, and the final configuration achieved 12/12 deterministic executions with zero wrong-object placements.

Overall, my contribution was not simply implementing a working demo, but building an experimental pipeline connecting **robot control, physical interaction, demonstration generation, and future VLA learning**.

## 3. Scope, Environment, and Task Design

**Host and simulator**: Intel Mac (macOS 12.7.6, x86_64). **Isaac Lab was
not used** — it has no macOS support, so it was not a candidate on this
host; the official allowed fallback, `unitree_mujoco` / MuJoCo, was used
instead. The simulator is pinned as a git submodule at commit
`4134cb5dc7ff1ba7f484deda48b5274b58694519` (merge PR #129, 2026-08-25) and
its files are byte-for-byte unmodified throughout. Versions: Python
3.12.14, `mujoco==3.3.6`, `numpy==2.5.2`, `imageio==2.37.4`,
`imageio-ffmpeg`, `h5py==3.16.0`, `matplotlib==3.11.1`.

**Task 1**: *"Pick up the red cube and place it in the blue target area."*
Single-object pick-and-place, under a fixed-base, torso-constrained
upper-body scope — pelvis and torso are rigidly welded to the world by
MuJoCo equality constraints. This is a deliberate, binding scope decision,
never described as full-body or free-standing manipulation: a
free-standing probe in Phase 2 showed 0.93m of pelvis drift in 2 seconds
under small arm torques, which is why the fixed-base MVP was chosen.

**Task 2**: language-conditioned two-object selection on the same scene
with a second, physically-identical green cube (§7).

**Task 3** and any form of model training or inference were never
implemented or attempted in any phase.

## 4. System Design

**Task-local gripper.** No pinned G1 model variant (`g1_23dof.xml`,
`g1_29dof.xml`, and their scenes) contains an actuated gripper, Inspire
hand, or Dex3 hand (Phase 2 audit). A task-local physical parallel-jaw
gripper was therefore built under `right_wrist_yaw_link`: two actuated
slide-joint fingers, collision-enabled finger pads, and a `grasp_tcp` site.
All of it lives under `tasks/g1_pick_place/`; the vendor model is never
edited. Phase 4E additionally removed the vendor's decorative
`right_rubber_hand` visual mesh from the task-local scene (it overlapped
the real pads and clipped through the cube) and added a palm plate and two
distinguishably-coloured pads.

**Controller** (Phase 3C onward, entirely classical):

- **Bounded MuJoCo `<position>` servos** per right-arm joint, retaining the
  real force limits — replacing the earlier torque-PD architecture (§8).
- **Waypoint-based, position-priority damped-least-squares IK**
  (`controller_3c.py::solve_ik_waypoint`), solved once per motion segment,
  with null-space joint-limit avoidance and a nominal-posture objective.
  Orientation is intentionally unconstrained on the data-collection path.
- The Phase 4F orientation-constrained variant
  (`solve_ik_waypoint_oriented`) exists but **failed** — it never reaches
  placement — and is explicitly not the collection controller
  (`use_oriented_ik=False` everywhere, verified in
  `data/task1_canonical_config.json`).
- **Integrator**: `implicitfast`; explicit Euler was numerically unstable
  at this timestep with stiff position-servo gains, independent of gains.
- **18-state machine**: `RESET → PREGRASP → SETTLE_PREGRASP → APPROACH →
  SETTLE_APPROACH → CLOSE → VERIFY_BILATERAL_CONTACT → LIFT → HOLD →
  TRANSPORT_ABOVE_TARGET → SETTLE_ABOVE_TARGET → LOWER_TO_TARGET →
  SETTLE_LOWER → OPEN → VERIFY_RELEASE → RETREAT → VERIFY_TASK_SUCCESS →
  DONE/FAILED`. Waypoints are always computed from the trial's own observed
  cube/target pose, never hardcoded.

**Physical integrity.** No weld, attachment, teleport, mocap manipulation,
or applied `xfrc` is ever used on the cube after a trial's first physics
step. This is enforced in code by `CubeInitGuard`, which raises if cube
`qpos`/`qvel` is written after its `lock()` call, and is exercised by a
dedicated test in every dataset-collection phase.

## 5. Results

| Result | Value |
| --- | --- |
| Nominal Task 1 | Deterministic success, 5/5 identical reruns (Phase 3C onward) |
| Supported-envelope success | 3/3 (100%) of reachable variants complete the full task |
| Original five-position coverage | 3/5 (60%) — the 2 unreachable variants reported separately, unchanged since Phase 4A |
| Visual acceptance | Phase 4F videos reviewed and accepted as "prototype completed with a documented grasp-slip limitation" |
| Strict 10mm grasp-slip target | **Not met** — 25.92mm max 3D slip while grasped |
| Corrected grasp-phase slip | 33–54mm across lift/transport/lower/release (Phase 4C audit) |
| Tests | 311, 0 unexpected failures |

**Setup variants** (Phase 4A, 5 deterministic cube positions, one shared
untuned configuration):

| Variant | Reachable (pre-run IK check) | Outcome |
| --- | --- | --- |
| nominal | yes | succeeds |
| x−0.03 | yes | succeeds |
| y+0.03 | yes | succeeds |
| x+0.03 | **no** (27.1mm residual vs. 8mm tolerance) | fails at `SETTLE_APPROACH` — pre-run filter and live trial agree |
| y−0.03 | **no** (8.43mm residual) | fails at `SETTLE_APPROACH` — same double-confirmation |

3/5 succeed: a genuine grasp-**reachability** limitation on the +x/−y
directions, not a tracking or grasp-quality failure. This envelope has held
unchanged from Phase 4A through Phase 5E. Phase 4B originally found y+0.03
missing the placement-XY margin (20.4mm vs. 15mm); Phase 4E's gripper-gain
and trajectory-smoothing changes closed that gap (re-measured at 2.07mm),
and the full sweep now reports `supported_envelope_success_rate: 1.0`.

## 6. VLA Demonstration Dataset

Built in four additive phases (5B → 5C → 5D → 5E), each preserving every
prior phase's dataset, report, and commit unmodified. Full schema and
usage terms: [`DATASET_CARD.md`](DATASET_CARD.md) and
[`../data/schema_v3.md`](../data/schema_v3.md).

**Design decisions:**

- **Two-rate recording**: a 10Hz `policy/` group (RGB `head_cam` 160×120,
  joint positions/velocities, TCP pose, gripper state) alongside a 500Hz
  `execution/` trace of the literal applied control, so replay fidelity can
  be measured against what the expert actually did.
- **Action representation**: `tcp_delta_position` / `tcp_delta_orientation`
  as `[T, 5, 3]` sub-action chunks (H=5, 50Hz effective) plus
  `gripper_command`. Every action is derived from the expert's own
  commanded joint trajectory by forward kinematics — never from privileged
  cube/target state, never leaking future observations (verified by
  dedicated causality tests).
- **Leakage boundary**: privileged state (cube/target pose, contact,
  ground-truth phase) lives in a separate top-level `privileged/` group,
  never nested under the declared policy-input group.
- **Provenance**: a canonical config manifest, hashed and verified at
  collection, validation, and replay time, is the single source of truth
  for which scene/controller/camera/thresholds are authorized; every tool
  fails loudly on mismatch. Its hash is byte-identical from Phase 5B
  through 5E, proving no silent drift across four pipeline phases.

**Contents**: 32 configurations attempted → 28 task successes, 4 diagnostic
(1 physical failure, 3 rejected pre-physics by the IK reachability filter).
Behaviour-cloning pool: exactly 24 successes (16 train / 4 val / 4 test, by
disjoint `cube_dx` bands assigned before collection); all 8
diagnostics-split episodes stay `train_eligible=False` regardless of
outcome. The HDF5 is 62,196,309 bytes (SHA-256 `accfe446…`), left untracked
as it exceeds the commit-size threshold, and regenerates deterministically.

**Replay and validation.** Three replay modes exist per episode: exact
execution (literal 500Hz control, no IK re-solve), policy-action (decodes
only the 10Hz/H=5 action stream through the real IK/PD primitives), and
observation-only visualization.

| Metric | Result |
| --- | --- |
| Exact-execution replay, 29 episodes | max 3.31e-5m, mean 1.25e-6m — ≥4 orders of magnitude inside the 1e-3m target |
| Policy-action replay, BC pool (24) | **18/24** meet the ≤10mm max-TCP-error target |
| Policy-action replay, dataset-wide (29) | 22/29 (76%) |
| Policy-action replay, Phase 5D's 3 canonical episodes | both successful ones meet it (8.09mm, 5.99mm) |

All 7 dataset-wide episodes exceeding 10mm first diverge at the
post-release RETREAT phase, which is verified never to affect cube
placement or task-success determination — placement is decided before
RETREAT begins, and the offending episodes' final placement error
(1.5–4.2mm) falls inside the same range as the passing ones' (1.1–4.1mm).

## 7. Task 2 — Language-Conditioned Two-Object Selection

Committed as `5f119ce`, independently audited 2026-09-03, merged to `main`.
Full evidence: `reports/task2-language-selection.md`.

Required reporting language: **language-conditioned task specification with
a privileged scripted expert.** `selected_object_id` and simulator object
poses are expert/evaluation metadata, not declared policy observations. No
visual recognition or learned language understanding is used anywhere:
`parse_selected_object()` is a keyword lookup over exactly the two
authorized instruction strings, used only to build labels.

`write_task2_scene()` adds one green cube — identical size, mass, and
friction to the red one — to Task 1's own unmodified scene, and
`run_trial_pick_place` gained four optional parameters (defaulting to Task
1's literal values) so the same scripted controller can act on a
caller-specified cube and track the other as read-only telemetry.

| Selected | Arrangement | task2_pass (3/3) | Wrong object placed | Distractor max disp. | Final target error |
| --- | --- | --- | --- | --- | --- |
| red | A | True | No | 0.0mm | 1.72mm |
| green | A | True | No | 1.73mm | 6.83mm |
| red | swapped | True | No | 1.73mm | 6.83mm |
| green | swapped | True | No | 0.0mm | 1.72mm |

12/12 trials pass. The pipeline has no RNG anywhere, so the 3 repeats per
configuration are bit-identical reruns, not distinct seeds. Distractor
displacement is measured as the maximum over the entire episode (not
final-minus-initial), tracked every physics step: **1.733mm maximum across
all 12 trials**, against a 10mm gate. Both cubes are initialized once
through their own `CubeInitGuard` before the first physics step.

**Engineering finding — the rejected 8cm distractor slot.** An earlier
candidate slot `(-0.08, 0.0)`, IK-reachable and apparently safe, was
rejected after a physics trial measured **48.7mm** of real distractor
displacement, peaking at RETREAT (independently reproduced in the audit at
48.72mm). Root cause: `RETREAT`'s one-shot joint-space trajectory sweeps
the arm near the second cube — Task 1 never had one nearby. The shipped
configurations use `SLOT_B_OFFSET = (-0.08, -0.10)` instead, found via a
three-way search on reachability, displacement, and camera visibility. The
controller was not tuned separately per object colour.

**Non-regression and audit**: the 4 new parameters default to Task 1's
exact prior values, and Task 1's measured nominal result is unchanged
(re-verified before and after the merge). 23 new tests; full suite at
`5f119ce`: 311 tests, 0 unexpected failures. The independent audit found no
material defects and required no corrective commit.

## 8. Failures and Debugging Process

Two controller architectures were tried before one succeeded; a visual
defect was caught by my own review after the pipeline had already passed
its test suite; an orientation-IK repair found a genuine kinematic
conflict, not a tuning gap; and three successive data phases each
diagnosed and fixed a specific, quantified replay defect before scaling.
In chronological order:

1. **Torque-PD/DLS controller — FAILED, 3 attempts** (Phase 3): uniform PD
   gains across joints with 5x differing torque limits caused tracking
   oscillation that shoved the cube out of position before the gripper
   closed. Final height gain 0.005m of a required 0.08m.
2. **Per-joint gain / torque-weighted IK — FAILED, 3 attempts** (Phase 3B):
   root cause was that uniform PD gains and a torque-agnostic IK are two
   separate, interacting problems — fixing either alone fixed neither.
3. **Bounded position-servo controller — PASSED** (Phase 3C): architecture
   replaced rather than tuned again. Second attempt succeeded outright —
   height gain 0.108m, 3.5s continuous hold, deterministic 5/5.
4. **Slip metric correction** (Phase 4C): the reported 156mm slip figure
   included post-release retreat, not just grasp-phase slip. Corrected,
   grasp-phase-only slip is 33–54mm. No pass/fail outcome changed.
5. **Decorative-hand visual defect — confirmed** (Phase 4D): a vendor
   decorative mesh, collision-free by design, overlapped the real gripper
   and clipped through the cube on every grasp. A second reported defect
   (cube "falling") was investigated and not reproduced.
6. **Gripper repair** (Phase 4E): visual defect fixed; a smoothed LIFT
   trajectory cut max slip from 51.9mm to 20.5mm — still over the ≤10mm bar.
7. **Orientation-IK experiment** (Phase 4F): found a genuine kinematic
   reachability conflict — levelling the wrist at the grasp waypoint
   requires 30–70mm of position error. 3 attempts improved a contact-offset
   metric ~17% but did not reduce overall slip (25.9mm); bar still unmet.
8. **Acceptance decision** (Phase 4F): Task 1 accepted as a prototype with
   a documented limitation; the strict bar FAILS at 25.92mm, direct video
   review of task execution PASSES.
9. **Replay zero-order-hold error** (Phase 5B/5C): the v1 dataset's 10Hz
   ZOH replay dropped the controller's real 500Hz intra-transition ramp,
   causing up to 48.7mm TCP error.
10. **Exact 500Hz replay** (Phase 5C): replaying the literal control trace
    reduced max TCP error to 3.65e-8m, confirming the ZOH gap was the
    entire error.
11. **10Hz phase-goal decoding** (Phase 5C): decoding a single static
    per-phase goal still produced ~97–98mm max error — a genuine
    action-representation limitation, not a decoder bug.
12. **H=5 action-chunk redesign — shipped** (Phase 5D): reference-relative
    TCP deltas at 50Hz cut max TCP error to 8.09mm / 5.99mm, under target.
13. **Scaled-collection generalization gap** (Phase 5E): the unchanged
    decoder applied to 32 configurations left 7/29 episodes above 10mm (up
    to 22.3mm), all diverging at RETREAT — reported as a measured gap for a
    future phase, not hidden.

Per-attempt evidence for every item is in the matching `reports/phase*.md`.

## 9. Limitations

- Strict max-3D-grasp-slip ≤10mm bar: **not met** (25.92mm). The
  pad-vertical-overlap and physical-release criteria under Phase 4F's
  tightened bar are also not met.
- x+0.03 / y−0.03 cube positions are **not reachable** by this
  controller/gripper geometry — a reachability limit, not a tracking one.
- Target-pad position is **fixed** for all 32 episodes; only cube position
  varies. The pad is fixed MJCF geometry with no offset parameter, and
  varying only the controller's internal target would have produced RGB
  frames showing the pad in the wrong place — reverted before use.
- The onboard camera is **torso-mounted near head height**, parented to
  `torso_link`, not to a head body (the G1 model has none).
- Phase 4F's orientation-constrained IK **failed** and is not the
  collection controller for any dataset phase.
- **7 of 29 scaled policy-action replays exceed the 10mm target**, all
  first diverging at the post-release RETREAT phase.
- **Model training and inference were never attempted.**
- **Task 3 was never implemented.** Task 2's own scaled dataset collection
  and policy integration were also never started (§7).
- The 5-variant sweep uses a single shared, untuned configuration by
  design — the envelope is a property of that configuration, not the best
  achievable per position.
- Replay does not reimplement the state machine's SETTLE/VERIFY gating;
  phase agreement during replay is reported from recorded metadata, not
  independently re-derived.

## 10. Time Spent

No phase tracked hands-on time while in progress. The figures below are
derived from the commit history: elapsed wall-clock time between each
phase's completion commit and the previous one. This is **calendar time
between commits, not continuous hands-on work** — three gaps (*) span what
was most likely a rest break, and every figure includes reading, thinking,
and report writing, not just typing.

| Phase | Commit | Timestamp | Elapsed |
| --- | --- | --- | --- |
| Setup, Phase 1, Phase 2 | `dd2bc9b` | 09-02 00:52 | N/A — initial commit |
| Phase 3 | `f5ce62d` | 09-02 01:13 | 20m |
| Phase 3B | `dd29718` | 09-02 01:39 | 26m |
| Phase 3C | `bf57b74` | 09-02 08:27 | 6h47m* |
| Phase 4A | `edb175a` | 09-02 11:12 | 2h44m |
| Phase 4B | `363aa83` | 09-02 12:13 | 1h01m |
| Phase 4C | `dfeec9e` | 09-02 12:57 | 44m |
| Phase 4D | `e212777` | 09-02 13:30 | 32m |
| Phase 4E | `b5fd237` | 09-02 16:16 | 2h45m |
| Phase 4F | `e6b53f4` | 09-02 20:47 | 4h31m |
| Phase 4F acceptance decision | `c9352ac` | 09-02 22:05 | 1h17m |
| Phase 5A | `03a9b51` | 09-03 00:18 | 2h13m |
| Phase 5B | `67ccf89` | 09-03 01:01 | 43m |
| Phase 5C | `965b947` | 09-03 10:36 | 9h34m* |
| Phase 5D | `621a63a` | 09-03 11:49 | 1h12m |
| Phase 5E | `d904ef9` | 09-03 12:43 | 54m |
| Phase 6 — submission prep | `35f15a5` | 09-03 14:24 | 1h40m |
| Task 2 — implementation | `5f119ce` | 09-03 16:31 | 2h07m |
| Task 2 — audit + merge | `0920882` | 09-03 17:22 | 50m |
| Phase 6 — updated for Task 2 | `7702f99` | 09-03 17:32 | 10m |
| Task 2 — scene cleanup | `537932f` | 09-03 17:58 | 26m |
| Post-submission doc/media polish | `c7632b3`…`7ba22e2` | 09-04 01:00–01:25 | 7h01m* + 25m |

Total span: 2026-09-02 00:52 to 2026-09-04 01:25, ~48.5 hours of calendar
time across 3 rest gaps. Where a phase separately logged a hands-on
estimate, it is in that phase's own report (Phase 3C: a "~4 hour" budget,
1 of 4 attempts consumed; Phase 5E: ~3 hours).

## 11. Reproduction, Documentation, and Future Work

**Reproduce**: see [`REPRODUCE.md`](REPRODUCE.md) — every command listed
there was actually executed in this environment, with real observed
runtimes recorded.

**Where the detail lives**: [`DATASET_CARD.md`](DATASET_CARD.md) (dataset
usage terms and biases), [`../data/schema_v3.md`](../data/schema_v3.md)
(HDF5 layout), `reports/` (17 per-phase audit reports — the traceable
source behind every number here), `HANDOFF.md` (chronological engineering
history and the per-phase authorization record).

**Future work:**

- Close the RETREAT-phase policy-replay gap (§8, item 13) — a larger H, or
  a position-dependent correction for large single-transition jumps.
- A v2 collection spec enabling genuine target-position variation, which
  requires an authorized Task 1 scene-geometry change.
- Close the strict ≤10mm grasp-slip bar (still 25.92mm) — needs a
  trajectory/waypoint redesign beyond the exhausted Phase 3B/3C/4E/4F
  tuning budgets.
- Scaled dataset collection and policy integration for Task 2.
- Model training and inference against the dataset.
- A Task 3 environment.
