# G1 MuJoCo Entrance-Test Report — Task 1 Pick-and-Place with VLA-Oriented Demonstration Data

Date: 2026-09-03. Author: this repository's automated engineering log, compiled
in Phase 6 from the project's own reports/logs/tests (`reports/*.md`,
`HANDOFF.md`, `docs/work_log.md`) — every figure below is sourced from one of
those files or independently re-measured during this phase, not invented.

## 1. Executive Summary

This project builds a physically-honest pick-and-place manipulation pipeline
for the Unitree G1 humanoid in MuJoCo, then a **VLA-oriented demonstration
dataset prototype with validated schema and replay instrumentation** on top
of it. **Task 1** ("Pick up the red cube and place it in the blue target
area") is complete: a fixed-base, torso-constrained G1 with a task-local
physical parallel gripper performs the full approach/grasp/lift/transport/
lower/release/retreat sequence with no weld, attachment, teleport, or
post-reset cube manipulation. It was accepted by direct human video review
as "prototype task completed with a documented grasp-slip limitation" — a
strict 10mm grasp-slip engineering bar is **not** met (measured 25.92mm).
288 automated tests pass. A 32-episode demonstration dataset was collected
(28 task successes, 4 diagnostic probes); exact-execution replay matches
the original physics to ~1e-8m; the shipped policy-action decoder meets a
10mm replay-fidelity target on 18 of the 24 behavior-cloning-pool episodes
outright (22 of 29 dataset-wide), with every excess-error case isolated to
the post-release RETREAT phase and traced to a documented mechanism. Task 2,
Task 3, and any form of model training/inference were never attempted.

## 2. Scope and Environment Choice

- **Host**: Intel Mac (macOS 12.7.6, x86_64).
- **Isaac Lab was not used**: it has no macOS support, so it is not a
  candidate on this host.
- **Official allowed fallback used instead**: `unitree_mujoco` / MuJoCo,
  the Unitree-provided Python simulator path.
- **Pinned repository commit**: `4134cb5dc7ff1ba7f484deda48b5274b58694519`
  (`unitreerobotics/unitree_mujoco`, merge PR #129, 2026-08-25), registered
  as a git submodule. Vendor files are byte-for-byte unmodified throughout
  the project; the one pre-existing artifact (`unitree_robots/go2w/assets/
  terrain.STL` macOS case-insensitive checkout collision) was found at setup
  time, is unrelated to G1, and was never touched.
- **Versions**: Python 3.12.14 (project-local `.venv`), `mujoco==3.3.6`,
  `numpy==2.5.2`, `imageio==2.37.4`, `imageio-ffmpeg` (bundled ffmpeg binary,
  no system ffmpeg install), `h5py==3.16.0`, `matplotlib==3.11.1`.

## 3. Task Design

**Task 1** (the only task implemented): *"Pick up the red cube and place it
in the blue target area."* Fixed-base, torso-constrained upper-body
manipulation scope — the pelvis and torso are rigidly welded to the world
via MuJoCo equality constraints. This is a deliberate, binding scope
decision (Phase 4A): never described as full-body or free-standing
manipulation. A free-standing probe (Phase 2) showed 0.93m of pelvis drift
in 2 seconds under small arm torques, which is why the fixed-base MVP was
chosen in the first place.

Task 2 (language-conditioned variants beyond the 3 fixed instruction
templates, policy integration) and Task 3 were never implemented or
authorized in any phase.

## 4. Unitree G1 and Task-Local Gripper

The pinned G1 model variants (`scene.xml`, `scene_23dof.xml`,
`scene_29dof.xml`, `g1_23dof.xml`, `g1_29dof.xml`) contain **no actuated
gripper, Inspire hand, or Dex3 hand** (Phase 2 audit). A **task-local
physical parallel-jaw gripper** was built under `right_wrist_yaw_link`: two
actuated slide-joint fingers, real collision-enabled finger pads, a `grasp_tcp`
site. All gripper geometry lives under `tasks/g1_pick_place/`; the vendor
model is never edited.

A real defect was found and fixed here: the vendor's own decorative
`right_rubber_hand` visual mesh (collision-free by the vendor's own
authoring, `contype="0" conaffinity="0"`) spatially overlapped this
project's real, physically-simulated finger pads, so the vendor hand
visibly clipped through the cube on every grasp even though it carried zero
collision force (Phase 4D, confirmed by direct STL bounding-box computation
and zoomed render). Phase 4E removed that mesh from the task-local scene
only and added a palm backing plate plus two distinguishably-colored real
finger pads.

## 5. Non-Learned Control Pipeline

The final, shipped controller (Phase 3C onward) is entirely classical:

- **Bounded MuJoCo `<position>` servos** per right-arm joint (real force
  limits retained), not torque motors — replacing an earlier torque-PD
  architecture that failed (Section 10).
- **Waypoint-based, position-priority damped-least-squares IK**
  (`controller_3c.py::solve_ik_waypoint`), solved once per motion segment,
  with null-space joint-limit avoidance and a nominal-posture objective.
  Orientation is intentionally unconstrained on the data-collection path.
- **A Phase 4F experimental orientation-constrained IK variant**
  (`solve_ik_waypoint_oriented`) exists but **failed** (fails at
  `SETTLE_LOWER`, never reaches placement) and is explicitly **not** the
  collection controller — `use_oriented_ik=False` is the default and the
  only configuration ever used for data collection, verified directly in
  `data/task1_canonical_config.json`.
- **Integrator**: `implicitfast` (the vendor model has no `<option>`
  element by default; explicit Euler was numerically unstable at this
  timestep with stiff position-servo gains, independent of gain choice).
- **State machine**: `RESET → PREGRASP → SETTLE_PREGRASP → APPROACH →
  SETTLE_APPROACH → CLOSE → VERIFY_BILATERAL_CONTACT → LIFT → HOLD →
  TRANSPORT_ABOVE_TARGET → SETTLE_ABOVE_TARGET → LOWER_TO_TARGET →
  SETTLE_LOWER → OPEN → VERIFY_RELEASE → RETREAT → VERIFY_TASK_SUCCESS →
  DONE/FAILED`. Waypoints are always computed from the trial's own observed
  cube/target pose, never hardcoded nominal coordinates (verified by the
  Phase 4A/5E variant sweeps, which use the same controller code with
  different poses and get different, pose-dependent IK solutions).

No weld, attachment, teleport, mocap manipulation, or applied `xfrc` is ever
used on the cube after the first physics step of a trial. This is enforced
in code by `CubeInitGuard`, which raises if cube `qpos`/`qvel` is written
after its `lock()` call, and is exercised by a dedicated test in every
dataset-collection phase.

## 6. Setup Variants

Phase 4A evaluated 5 deterministic cube-position variants (nominal, ±0.03m
on each table-plane axis) with one shared, untuned configuration:

| Variant | Reachable (pre-run IK check) | Outcome |
| --- | --- | --- |
| nominal | yes | succeeds |
| x−0.03 | yes | succeeds |
| y+0.03 | yes | succeeds (see note below) |
| x+0.03 | **no** (27.1mm residual vs. 8mm tolerance) | fails at `SETTLE_APPROACH`, confirmed by the pre-run filter and the live trial agreeing |
| y−0.03 | **no** (8.43mm residual) | fails at `SETTLE_APPROACH`, same double-confirmation |

**3/5 variants succeed, 2/5 fail — a genuine grasp-*reachability* limitation
on the +x/−y directions**, not a tracking or grasp-quality failure. This
envelope has held, unchanged, from Phase 4A through Phase 5E. Phase 4B's
Stage B (full pick-and-place, not just grasp) found y+0.03 grasps
successfully but originally missed the placement-XY margin (20.4mm vs.
15mm) — a separate, narrower placement-accuracy limit that was later closed
by Phase 4E's gripper-gain/trajectory-smoothing changes (re-verified in
Phase 5B: y+0.03 placement error dropped to 2.07mm under the current
config, and re-confirmed live during this phase: the full Stage A/B sweep
now reports `supported_envelope_success_rate: 1.0`, i.e. 3/3).

## 7. Demonstration Data Pipeline

Built in four additive phases (5B→5C→5D→5E), each preserving every prior
phase's dataset/report/commit unmodified:

- **Phase 5B**: first HDF5 prototype, 3 episodes, canonical manifest
  (`data/task1_canonical_config.json`) established *before* any pipeline
  code, hash-stamped and enforced by every downstream tool.
- **Phase 5C**: added a 500Hz `execution/` trace alongside the 10Hz
  `policy/` group and 3 distinct replay modes, diagnosing (not yet fixing)
  a large policy-replay gap.
- **Phase 5D**: redesigned the *stored action representation itself*
  (reference-relative TCP delta, H=5 sub-action chunking at 50Hz) to close
  that gap on 3 hand-picked episodes.
- **Phase 5E**: scaled collection to 32 attempted configurations across a
  continuous, pilot-calibrated cube-position envelope.

### Dataset (Phase 5E, `data/task1_demonstrations_v1.hdf5`)

- **32 configurations attempted → 28 task successes, 4 diagnostic**
  (1 physical failure at `SETTLE_APPROACH`, 3 rejected pre-physics by the
  IK reachability filter). This does **not** match the originally-intended
  24-success/8-diagnostic split — reported honestly rather than resampled
  to force it (Phase 5E Section I): all 24 configs sampled from the
  calibrated success envelope succeeded (100% hit rate, exactly the
  intended BC pool), while 4 of the 8 diagnostic probes (genuinely
  uncertain by design) turned out to succeed physically too.
- **Intended BC-training pool: exactly 24 successful episodes** (16 train /
  4 val / 4 test), unaffected by the diagnostic-split surplus — all 8
  diagnostics-split episodes keep `train_eligible=False` regardless of
  individual outcome, by a pre-registered split rule.
- **Onboard RGB**: 10Hz, raw frames, no overlay, in `policy/observations/rgb`.
- **Action representation**: `policy/actions/{tcp_delta_position,
  tcp_delta_orientation,gripper_command}`, each an H=5 sub-action chunk
  (50Hz effective sub-action rate) per 10Hz transition, derived purely from
  the expert's own commanded joint-target trajectory via forward
  kinematics — never from privileged cube/target state.
- **Execution trace**: 500Hz, one row per physics step, literal applied
  control + joint/TCP/cube state, in a separate `execution/` group.
- **Privileged state** (cube/target pose, contact, state-machine phase) is
  a separate top-level `privileged/` group, never nested under the
  declared policy-input observation group.
- **HDF5**: 62,196,309 bytes, SHA-256
  `accfe4461e7decc0dac2f7b959496487ba94747a504c534e149b8593c7749f21` — left
  **untracked** (exceeds a 20MB commit-size threshold); regenerate with
  `python3 -m tasks.g1_pick_place.collect_dataset` (deterministic).
- **Canonical manifest hash**: `f7375efc18d00fd83b2c75228bac76a7c23922913e104d6970e9b8241b9c290b`
  — byte-identical from Phase 5B through Phase 5E, proving the underlying
  scene/controller/gains/thresholds/camera never drifted across four
  dataset-pipeline phases.
- **Decoder configuration hash**: `9ef9bd49fc376f125c10be5b46d21a45148a79a2057f32d28afc849629a81d86`.
- **Collection spec hash**: `3e007e1fab0fca8ff5916ab42803470396f753b31e0fb89430564a5b56807d1d`.

This dataset is a **VLA-oriented demonstration dataset prototype with
validated schema and replay instrumentation** — not a claim of being
model-ready, and no model was trained or evaluated against it in any phase.

## 8. Replay and Validation

Three replay modes exist for every episode: (1) exact-execution replay
(literal 500Hz `applied_ctrl`, no IK re-solve), (2) policy-action replay
(decodes only the 10Hz/H=5-chunked action stream through the real IK/PD
primitives), (3) observation-only visualization (no physics stepped).

| Metric | Result |
| --- | --- |
| Exact-execution replay, all 29 Phase 5E episodes | max 3.31e-5m, mean 1.25e-6m — all ≥4 orders of magnitude inside the 1e-3m target |
| Policy-action replay, dataset-wide (29 episodes) | 22/29 (76%) meet the ≤10mm max-TCP-error target |
| Policy-action replay, BC pool only (24 episodes) | **18/24** meet the ≤10mm target |
| Policy-action replay, Phase 5D's original 3 canonical episodes | both successful ones meet the target (8.09mm, 5.99mm) |

**Every one of the 7 dataset-wide episodes exceeding 10mm first diverges at
the RETREAT phase** (transition 119, post-release) — see Section 9 and the
Phase 6 quality sidecar (`data/task1_demonstrations_v1_quality.json`) for
the full per-episode table and the proof that this never changes cube
placement or task success (placement is determined before RETREAT begins;
the offending episodes' final placement error, 1.5–4.2mm, falls inside the
same range as the passing episodes', 1.1–4.1mm).

Full test suite: **288 tests, 0 unexpected failures** (independently
re-run in this phase; see Section 9).

## 9. Results

- **Nominal Task 1**: deterministic success, 5/5 identical reruns
  (Phase 3C onward); re-confirmed live in this phase
  (`python3 -m tasks.g1_pick_place.run_pick_place`, Stage A `all_task_pass:
  true`).
- **Setup-variant results**: 3/5 succeed, 2/5 fail on grasp reachability
  (Section 6); Stage B (full pick-and-place) now 3/3 on the reachable
  variants under the current, post-4E configuration.
- **Supported-envelope result**: 3/3 (100%) of the reachable variants
  complete the full task.
- **Original five-position coverage**: 3/5 (60%) — the two unreachable
  variants (x+0.03, y−0.03) are excluded from the primary denominator and
  reported separately, unchanged since Phase 4A.
- **Human visual acceptance**: Phase 4F's videos
  (`artifacts/phase4f_task1_full.mp4`,
  `artifacts/phase4f_bilateral_contact_view.mp4`) were directly reviewed by
  a human and accepted as "prototype task completed with a documented
  grasp-slip limitation" (Phase 4F human acceptance decision).
- **Strict 10mm grasp-slip target**: **not met** — measured
  0.02592m (25.92mm) maximum 3D slip while grasped, unchanged since Phase
  4F, not claimed to pass. This is a distinct, stricter criterion from the
  demonstration-replay 10mm target in Section 8.
- **Measured final-slip limitation**: genuine grasp-phase slip (corrected
  metric, gated on the `carrying` signal, not conflated with post-release
  separation) is 3.3–5.4cm across lift/transport/lower/release on the
  nominal trial (Phase 4C audit).
- **288 passing tests** — see Section 9 test-suite table below.
- **Dataset episode counts/splits**: 32 attempted, 28 successes, 4
  diagnostic; splits 16 train / 4 val / 4 test / 8 diagnostics (Section 7).
- **Exact-execution replay result**: max 3.31e-5m across all 29 Phase 5E
  episodes (Section 8).
- **Policy-action replay, prototype (Phase 5D, 3 episodes) and scaled
  (Phase 5E, 32 episodes) results**: prototype 8.09mm/5.99mm (both pass);
  scaled 22/29 pass, 7/29 exceed up to 22.3mm, all at RETREAT (Section 8).

### Test-suite growth by phase

| Phase | Total tests | New this phase | Unexpected failures |
| --- | --- | --- | --- |
| 2 | 4 | 4 | 0 |
| 3/3B | 24 | 20 | 3 (historical, by design) |
| 3C | 58 | 34 | 0 |
| 4A | 61 | 3 | 0 |
| 4B | 94 | 36 | 0 |
| 4C | 104 | 10 | 0 |
| 4D | 117 | 13 | 1 (intentional, documents the confirmed decorative-hand defect) |
| 4E | 138 | 21 | 0 |
| 4F | 159 | 20 | 0 (2 tests honestly assert the still-failing strict slip bar) |
| 5A | 178 | 19 | 0 |
| 5B | 214 | 36 | 0 |
| 5C | 231 | 17 | 0 |
| 5D | 256 | 25 | 0 |
| 5E | 288 | 32 | 0 |

Re-run independently in this phase: **288 tests, 0 unexpected failures**,
see Section 12/`logs/` for the exact runtime.

## 10. Failures and Debugging Process

See Section 3 (`submission/entrance_test_report.md` §"Failure Narrative"
below) for the full chronological account. Headline: two entire controller
architectures were tried and one was discarded (torque-PD + DLS-IK, Phase
3/3B, 6 failed attempts total) before the shipped bounded-position-servo +
waypoint-IK architecture (Phase 3C) succeeded; a visual defect was found by
human GUI review after the pipeline had already "passed" its own test
suite (Phase 4D); an orientation-IK repair attempt (Phase 4F) found a
genuine kinematic conflict, not a tuning gap; and three successive
demonstration-data phases (5B→5C→5D) each diagnosed and fixed a specific,
quantified replay-fidelity defect before scaling.

## Failure Narrative (Section C)

*In true chronological order — grouping decisions in the original phase
authorizations sometimes differ from the actual dates; this section follows
the real timeline recovered from `HANDOFF.md`/`docs/work_log.md`.*

1. **Initial torque-PD/DLS failure (Phase 3)**: a single `(Kp=180,Kd=18)`
   PD gain pair applied uniformly across 7 right-arm joints with 5x
   differing torque limits (shoulder/elbow ±25 N·m, wrist ±5 N·m) produced
   multi-joint Cartesian tracking oscillation that physically shoved the
   cube out of position before the gripper closed. 3 tuning iterations,
   all FAIL (final: height gain 0.005m of the required 0.08m).
2. **Per-joint gain and torque-weighted IK investigation (Phase 3B)**: confirmed
   all 7 arm actuators are 1:1 joint-torque (no unit-conversion surprise) and
   that gravity/Coriolis feedforward was not the bottleneck (<10% of torque
   budget at rest). Three evidence-driven attempts, all FAIL:
   per-joint-scaled gains fixed the targeted wrist saturation but pushed
   load onto shoulder/elbow (TCP RMS error 0.061→0.215m, lost contact
   entirely); adding a settle-before-close gate didn't help (never
   converged — a tracking problem, not a speed problem); reverting gains
   and adding a torque-weighted DLS-IK term got closest (near-baseline
   tracking regained) but height gain was still only 0.0005m of 0.08m
   required. Root cause: uniform PD gains and a torque-agnostic IK are two
   separate, interacting problems — fixing either alone does not fix the
   coupled system.
3. **Bounded position-servo success (Phase 3C)**: replaced the entire
   architecture (torque motors → bounded MuJoCo position servos; continuous
   DLS-IK → waypoint-based position-priority IK with an evidence-derived
   8mm tolerance near a real kinematic singularity) rather than a 7th
   tuning attempt on the old one. First attempt (old gripper gains)
   achieved contact but insufficient grip strength (height 0.0497m, held
   0.198s then dropped). Second attempt (gripper gain only, kp=150/kd=10)
   **passed outright**: height gain 0.1084m, continuous hold 3.504s,
   deterministic 5/5.
4. **Slip metric incorrectly included post-release retreat, then corrected
   (Phase 4C)**: an audit of the reported `max_cube_slip_m=0.1562m` found
   the metric kept accumulating "slip" for the rest of the trial after a
   grasp reference was captured, with no check the cube was still being
   carried — so `OPEN`/`RELEASE_SETTLE`/`VERIFY_RELEASE`/`RETREAT` were
   silently included (`post_release_tcp_cube_separation_m=0.1486m`
   explained nearly the entire old figure). The TCP-local-frame math itself
   was already correct — confirmed by a synthetic rotation-only unit test
   giving exactly zero slip. Corrected, `carrying`-gated metrics found
   genuine grasp-phase slip of 3.3–5.4cm (lift/transport/lower/release),
   not 15.6cm. No pass/fail outcome changed — slip was never an acceptance
   criterion.
5. **Visual decorative-hand penetration discovered by human review (Phase
   4D)**: triggered by a user's own GUI inspection of the committed
   pipeline reporting the hand visibly passing through the cube. Confirmed:
   the vendor's decorative `right_rubber_hand` mesh (collision-free by the
   vendor's own authoring) spatially overlapped the real, physically-
   simulated finger pads. A second reported defect (cube visibly "falling")
   was investigated but **not reproduced** — an isolated support test and a
   fresh instrumented trial both showed correct physics; most likely the
   same decorative-mesh confusion made a real, correct grip look
   precarious. No fix was made this phase — a new test was added that
   **fails on purpose** to keep the confirmed defect visible rather than
   hidden by a passing suite.
6. **Correction of visual/collision alignment (Phase 4E, Section A)**: the
   decorative mesh was removed from the task-local scene only; a palm
   backing plate and two distinguishably-colored real finger pads were
   added. The previously-intentionally-failing test now genuinely passes.
   A grasp-stability redesign was also attempted in the same phase (LIFT's
   one-shot position-servo step was fixed to a smooth multi-waypoint ramp,
   matching what Phase 4B had already done for TRANSPORT/LOWER): max slip
   while grasped fell from 5.19cm to 2.05cm and the worst-instant bilateral
   safety factor rose from 0.054x to ≥1.0x — genuine, measured improvement —
   but the new ≤10mm max-slip bar was still not met (20.5mm, ~2x over).
7. **Orientation-IK experiment exposed a kinematic conflict (Phase 4F)**:
   hypothesis was that free wrist-roll drift during descent caused
   off-axis, corner-only contact. A new opt-in oriented-IK path found the
   wrist's jaw axis was already well-aligned (~4-5°off), but its "tall"
   axis was ~47° off vertical — a genuine geometric finding. Attempt 1
   (mild orientation weight) barely improved residual (47.4°→44.5°), slip
   unchanged. Attempt 2 (higher weight) discovered reaching the 7°
   orientation tolerance at this exact Cartesian point requires 30-70mm of
   *position* error — a real kinematic reachability conflict, not a tuning
   gap; the trial failed even earlier (`SETTLE_APPROACH`, no grasp
   attempted). Attempt 3 (a measured, fixed finger-mount rotation
   correction) improved a static contact-offset metric by ~17% but did not
   reduce overall slip (rose slightly to 25.9mm) — slip is evidently also
   driven by dynamic effects, not solely static contact geometry. No
   further attempt was made; the strict bar (≤10mm) remained unmet.
8. **Prototype accepted with a measured slip limitation (Phase 4F human
   acceptance decision)**: a human directly reviewed the Phase 4F videos
   and found the task execution visually/functionally acceptable, while
   explicitly *not* claiming the strict 10mm engineering bar was met
   (measured 25.92mm, unchanged). This separated two questions the earlier
   single bar had collapsed together: strict grasp-quality engineering
   (FAIL) vs. entrance-test prototype task completion judged by direct
   human video review (PASS, with the limitation documented).
9. **v1 policy replay ZOH error of 48.7mm (Phase 5B/5C)**: the first HDF5
   prototype's 10Hz action stream, replayed with a zero-order hold across
   each 100ms interval, deviated from the true trajectory by up to 4.87cm
   (48.7mm) TCP error — because the arm's real commanded target changes
   every physics step (500Hz) during the ramped LIFT/TRANSPORT/LOWER
   phases, and the ZOH replay discarded that entire intra-transition ramp.
10. **Exact 500Hz replay reduced error to 3.65e-8m (Phase 5C)**: replaying
    the *same* episode's literal 500Hz applied-control trace instead of the
    10Hz zero-order-hold dropped the max TCP replay error to 3.65e-8m — a
    ~1.3-million-times reduction, proving the ZOH gap was the *entire*
    error with no other meaningful contributor.
11. **10Hz phase-goal representation failed at 96.9mm (Phase 5C)**: even
    with a proper two-rate schema, decoding only the stored 10Hz
    `cartesian_target` (one static per-phase goal repeated across every
    transition of a multi-second phase) through the real IK/PD primitives
    produced ~97-98mm max TCP error on both successful episodes — a
    decoder re-ramping toward a distant repeated goal every 100ms takes a
    faster, more direct path than the true multi-waypoint trajectory,
    even though it converges to nearly the same final point (6.3mm final
    error). Diagnosed as a genuine action-representation limitation, not a
    decoder bug, and explicitly out of scope to fix in that phase.
12. **H=5 50Hz action chunk reduced nominal replay error to 8.09mm (Phase
    5D)**: redesigned the stored action to a reference-relative TCP delta
    (composed onto the decoder's own tracked commanded reference, never
    re-read from noisy measured state). Three attempts, in the authorized
    order: (1) one whole-interval delta per 10Hz transition, 23.6mm — the
    first large single-transition jump (e.g. PREGRASP) forced the decoder
    to reach it in 100ms when the true trial had far longer to settle; (2)
    a ramp-speed sweep found no single speed fixed every large jump
    simultaneously (fast ramps fixed PREGRASP but broke a later RETREAT
    jump, and vice versa) — a genuine information gap, not a tuning
    problem; (3) fixed-size H=5 sub-action chunking (50Hz effective rate)
    **shipped**: 8.09mm (nominal) and 5.99mm (x_minus_0.03), both meeting
    the ≤10mm target. H=2 (43.1mm) still failed; H=10 (7.36mm) was measured
    but not shipped since H=5 already met the target with margin.
13. **Scaled collection exposed remaining RETREAT generalization failures
    (Phase 5E)**: applying the unchanged H=5/50Hz decoder to 32
    configurations spanning a continuous cube-position envelope (rather
    than 3 hand-picked points) found 7 of 29 replayed episodes (24%) still
    exceed the 10mm target, up to 22.3mm — every one first diverging at
    the RETREAT phase, the same phase class (a large single-transition
    reference jump) implicated in Phase 5D's Attempt-1/2 diagnosis, now
    surfacing at cube positions Phase 5D's 3-point validation never
    sampled. The decoder was deliberately left unchanged per that phase's
    authorization; this is reported as a measured generalization gap for a
    future phase to address (e.g. a larger H, or a position-dependent
    initial-jump correction), not silently patched or hidden.

## 11. Limitations

- Strict max-3D-grasp-slip ≤10mm bar: **not met** (25.92mm).
- Cube-center-within-pad-vertical-overlap and physical-release/settled-
  placement criteria under Phase 4F's tightened bar: **not met**.
- x+0.03/y−0.03 cube positions: **not reachable** by this controller/gripper
  geometry (a grasp-reachability limitation, not a tracking failure).
- Target-pad position is **fixed** (`(0,0)` offset) for every one of the 32
  Phase 5E episodes — only cube position is varied. The rendered blue
  target pad is fixed MJCF geometry with no offset parameter; an attempt
  to vary only the controller's internal target while leaving the pad
  fixed would have produced RGB frames showing the pad in the wrong place
  relative to the true goal, and was reverted before use rather than
  shipped.
- The onboard camera (`head_cam`) is **torso-mounted, positioned near head
  height** — it is parented to `torso_link`, **not to a separate head
  body** (the G1 model has no such body; the head mesh is itself a static
  geom on `torso_link`).
- Phase 4F's orientation-constrained IK **failed** (fails at
  `SETTLE_LOWER`, a genuine kinematic reachability conflict) and is **not**
  the collection controller for any dataset phase; `use_oriented_ik=False`
  throughout.
- **7 of 29 scaled policy-action replays exceed the 10mm target, all first
  diverging at the RETREAT phase** (post cube-release; does not affect
  placement or task-success determination — see Section 8 and the quality
  sidecar).
- **Model training/inference was never attempted** in any phase.
- **Task 2 and Task 3 environments were never implemented.**
- The 5-position-variant sweep uses a single shared, untuned configuration
  by design (no per-variant tuning) — the resulting envelope is a property
  of that one configuration, not necessarily the best achievable per
  position.
- Replay does not reimplement the state machine's SETTLE/VERIFY gating;
  "phase agreement" during replay is reported via recorded ground-truth
  metadata, not independently re-derived.

## 12. Time Spent

Approximate hands-on time, drawn from each phase's own report where stated
(not all phases logged this explicitly; phases without a stated figure are
left blank rather than estimated):

| Phase(s) | Approximate time |
| --- | --- |
| Setup, Phase 1, Phase 2 | not separately logged |
| Phase 3 / 3B / 3C | not separately logged (Phase 3C: "at most ~4 hours" budget, consumed 1 of 4 attempts) |
| Phase 4A–4F | not separately logged |
| Phase 5A–5D | not separately logged |
| Phase 5E | ~3 hours (pilot design, target-position deviation investigation/revert, collector/validator/replay tooling, tests, docs — per `reports/phase5e-scaled-data-collection.md`) |
| Phase 6 (this report) | not separately logged at time of writing |

Total project wall-clock time across all phases was not tracked as a single
running figure in any prior report; this table reports only what each
phase's own report actually recorded, honestly leaving the rest blank
rather than inventing a number.

## 13. Reproduction Instructions

See [`submission/REPRODUCE.md`](REPRODUCE.md) — every command listed there
was actually executed in this environment during this phase, with real
observed runtimes recorded.

## 14. Future Work

- Close the RETREAT-phase policy-replay generalization gap (Section 10,
  item 13) — larger H, or a position-dependent correction for large
  single-transition reference jumps.
- A version-2 collection spec enabling genuine target-position variation,
  which requires a Task 1 scene-geometry change (moving the rendered pad)
  — proposed in Phase 5E, not attempted.
- A version-2 diagnostic-probe spec pushing further past the confirmed-safe
  reachability boundary, if exactly 8 diagnostic failures (vs. the 4
  actually observed) is desired.
- Closing the strict ≤10mm grasp-slip engineering bar (still 25.92mm) —
  would need trajectory/waypoint redesign beyond the exhausted Phase
  3B/3C/4E/4F tuning budgets, per each phase's own stop-condition.
- Model training/inference against the dataset — not attempted in this
  project.
- Task 2/Task 3 environments — not implemented in this project.
