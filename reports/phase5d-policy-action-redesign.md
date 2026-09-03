# Phase 5D — Redesigned VLA Policy-Action Representation

Date: 2026-09-03

Fixes Phase 5C's remaining policy-action-replay gap (max TCP error 96.9mm,
~98mm on the two successful episodes, against a <=10mm target) by redesigning
the stored **action semantics**, not just the decoder. `data/task1_prototype.hdf5`
(Phase 5B), `data/task1_prototype_v2.hdf5` (Phase 5C), their reports, and
commits `67ccf89`/`965b947` are preserved unmodified as historical evidence;
all work in this phase is additive (`_v3` files). Does not modify Task 1
physics, controller gains, geometry, camera, success thresholds, or setup
variants. Full field reference: [`data/schema_v3.md`](../data/schema_v3.md).
Raw numbers: [`logs/phase5d_policy_replay.json`](../logs/phase5d_policy_replay.json).

## Section A — new action semantics

At transition t, the primary action is the expert's actual commanded TCP
reference **delta** over `[t, t+1]` — not the final phase goal (Phase 5C's
bug) and not a single whole-interval delta (this phase's Attempt 1, see
below). Derived purely from `execution/arm_joint_target` (the expert's own
commanded joint-target trajectory, already captured at 500Hz in Phase 5C's
collector) via forward kinematics — never from privileged cube/target
state, never from a future policy observation.

- `tcp_delta_position` — world frame. Verified (not assumed): this
  fixed-base robot's pelvis/torso weld has ~0.19mm softness (measured
  independently in Phase 5A), so world frame and robot-base frame coincide
  to within that sub-millimeter margin.
- `tcp_delta_orientation` — TCP-local (body) frame, MuJoCo's native
  `mju_subQuat`/`mju_quatIntegrate` pair. Verified numerically (see
  `tests/test_phase5d_policy_actions.py::test_rotation_composition_order_is_body_frame_right_multiplication`):
  `mju_subQuat(qa, qb)` returns the rotation vector `r` such that
  `qa = qb ⊗ axisAngle2Quat(r)` (right/body-frame composition — the test
  explicitly rules out the left/world-frame alternative), and
  `mju_quatIntegrate` reconstructs `qa` to within quaternion double-cover
  (`±1e-8`).
- `gripper_command` — `0.0` = open, `1.0` = closed, held constant across
  the interval.
- `next_arm_joint_target` — optional debug field, not a declared VLA
  action.
- `state_machine_phase` — metadata only.

## Attempt history (Section D, max 3 attempts, in the authorized order)

### Attempt 1: single 10Hz reference-relative TCP delta

One IK solve + one linear joint-space ramp across the full 100ms interval.
**Result: nominal max TCP error 23.6mm** (down from Phase 5C's 96.9mm, but
still above the 10mm target).

**Diagnosis (first-divergence trace, not a guess)**: the first large
single-transition delta occurs at transition 0 (RESET → PREGRASP, a 5.85cm
jump — `_drive_segment` phases like PREGRASP issue a fixed joint-space
set-point ONCE and hold it for the whole segment, often many policy
transitions). The true trial's measured TCP converges to that target
gradually across the ENTIRE segment's wall-clock duration (potentially
several hundred ms), but Attempt 1's decoder forces its own commanded
reference to reach the SAME target within just one 100ms window — giving
the position-servo systematically less settle time than the true trial had
at that same wall-clock point. Measured max error at that boundary: 18.6mm.

### Attempt 2: adjust interpolation speed/shape only

Swept the ramp from an immediate step (reach target within 1 physics step,
then hold) to the full 50-step ramp:

| Ramp speed (physics steps to reach target) | nominal max TCP error |
| --- | --- |
| 1 (step) | 49.7mm |
| 5 | 49.5mm |
| 10 | 49.7mm |
| 15 | 49.5mm |
| 20 | 47.1mm |
| 25 | 44.5mm |
| 50 (full ramp, = Attempt 1) | 23.6mm |

**Diagnosis**: faster ramps improved the PREGRASP-transition error (an
immediate step measured only 4.7mm there) but made a DIFFERENT, later large
delta — a RETREAT-phase jump — much worse (49.4-49.7mm peak, vs. Attempt
1's 23.6mm). No single ramp speed/shape worked for every large
single-transition delta simultaneously. This was diagnosed as a genuine
**information gap**: a single 100ms-wide delta cannot describe a
trajectory that contains a large sub-100ms-scale reference change, no
matter how the decoder interpolates it. Not a tuning problem — moved to
Attempt 3 per the authorized hierarchy.

### Attempt 3 (shipped): fixed-size sub-action chunk

Each 10Hz transition stores `H` sub-deltas instead of one, each decoded
with the same one-IK-solve-plus-one-linear-ramp primitive applied to a
`SUBSTEPS_PER_TRANSITION / H`-physics-step sub-chunk. Swept `H`:

| H (sub-actions/transition) | sub-action rate | nominal max TCP error |
| --- | --- | --- |
| 1 (= Attempt 1) | 10Hz | 23.6mm |
| 2 | 20Hz | 43.1mm |
| **5 (shipped)** | **50Hz** | **8.09mm** |
| 10 | 100Hz | 7.36mm |

**H=5 shipped** — meets the <=10mm target with margin on both successful
episodes (measured below). H=2 still failed (43.1mm) — chunk *granularity*
matters, not just chunk *presence*: at H=2, a large sub-100ms jump can
still be forced into one 50ms sub-chunk instead of the finer 20ms
resolution H=5 provides. H=10 measured marginally better (7.36mm) but was
not selected — H=5 already meets the target with margin, and doubling
field size for a target already met is not warranted.

Full sweep data (including x_minus_0.03/x_plus_0.03 at H=5) in
`logs/phase5d_policy_replay.json`'s `attempt_history` key.

## Section E — per-episode results (H=5, shipped)

### Exact execution replay — unchanged from Phase 5C

| variant | max TCP error (m) | within tolerance (<=1mm) |
| --- | --- | --- |
| nominal | 3.65e-08 | **True** |
| x_minus_0.03 | 3.38e-08 | **True** |
| x_plus_0.03 | 4.40e-08 | **True** |

Byte-identical numbers to Phase 5C — `execution/` schema/semantics
untouched by this phase (verified directly: `tests/test_phase5d_policy_actions.py::test_exact_execution_replay_still_near_machine_precision`).

### Policy-action replay (H=5)

| variant | max TCP error (m) | final TCP error (m) | max cube error (m) | final cube error (m) | within <=10mm | success outcome |
| --- | --- | --- | --- | --- | --- | --- |
| nominal | 0.00809 | 0.00565 | 0.00675 | 0.00259 | **True** | replayed placement within target margin (2.66mm) |
| x_minus_0.03 | 0.00599 | 0.00510 | 0.00862 | 0.00383 | **True** | replayed placement within target margin (2.72mm) |
| x_plus_0.03 | 0.01277 | 0.00348 | ~0 | ~0 | N/A (failure episode, see below) | fails at APPROACH before grasp, as recorded |

**Both successful episodes meet the <=10mm max-TCP-error target.** Joint
error is reported for completeness (up to 0.24-0.34 rad max, converging to
0.015-0.025 rad final — the replay's own IK re-solve picks a different but
kinematically equivalent joint configuration than the recorded one at
times, which is expected and does not affect TCP-space fidelity).

**Accumulated delta drift** (does error grow monotonically or stay
bounded?): the per-quartile TCP error profile for nominal is
`[~0mm, 0.94mm, 2.41mm, 4.31mm, 5.65mm]` — the running-max is
non-decreasing (a peak-and-hold pattern typical of composing many small
deltas with no re-anchoring to ground truth), but the ABSOLUTE magnitude
stays well-bounded under 10mm throughout, not runaway growth. Full profile
in the log.

**x_plus_0.03 (pre-grasp reachability failure)**: max TCP error 12.77mm —
slightly above 10mm, but Section E's <=10mm requirement applies only to
**successful** episodes; for this failure episode the requirement is
identical failure stage/label, which is met (`first_divergence_phase:
"APPROACH"`, consistent with the real failure occurring at `SETTLE_APPROACH`
immediately afterward; `stored_success: False` reproduced). Reported
honestly, not silently included under the 10mm claim.

**State-machine phase agreement / physical outcome**: this replay does not
reimplement the SETTLE/VERIFY gating state machine (same disclosed
limitation carried from Phase 5C) — phase agreement is reported via the
recorded `state_machine_phase` metadata aligned to each transition, not
independently re-derived; physical success is judged via final cube-vs-target
position agreement (`target_placement_within_margin`), which is `True` for
both successful episodes.

## Section F — v3 dataset

`data/task1_prototype_v3.hdf5`: same 3 episode roles as Phase 5B/5C
(nominal, x_minus_0.03 successes; x_plus_0.03 reachability failure).
Canonical manifest hash `f7375efc18d00fd83b2c75228bac76a7c23922913e104d6970e9b8241b9c290b`
— identical to Phase 5B/5C, confirming the underlying scene/controller/
gains/thresholds/camera are unchanged. `decoder_configuration_hash`
`9ef9bd49fc376f125c10be5b46d21a45148a79a2057f32d28afc849629a81d86`.
`action_schema_version` / dataset `schema_version`: `"3.0.0"`.

Regenerate: `python3 -m tasks.g1_pick_place.record_demonstrations_v3`
(deterministic — no RNG anywhere in this pipeline).

## Section G — tests

`tests/test_phase5d_policy_actions.py`: 25 new tests covering known
synthetic TCP-delta recovery, rotation-composition-order/frame
verification (against a hand-constructed left- vs. right-composition
discriminator, not assumed), decoder chunk-size consistency and exact
sub-step-count production, causality (`next_arm_joint_target` reconstructed
purely from that transition's own `execution/arm_joint_target`), no
off-by-one action shift (transition 0's action reproduces exactly the
delta between the reset boundary and transition 0's own recorded end,
verified via independent forward kinematics), no-phase-goal-repetition
(both a pass on the real dataset and a tamper test that injects a
repeated-static-goal pattern and confirms the validator catches it),
terminal/padding behavior, canonical-manifest-hash and
decoder-configuration-hash mismatch rejection (both in replay and in the
validator), and the shipped 10mm policy-replay regression on both
successful episodes.

Full-suite regression: **256 tests, 0 unexpected failures** (231
pre-existing from Phase 5C + 25 new).

## Compliance checklist

- [x] `data/task1_prototype.hdf5`, `data/task1_prototype_v2.hdf5`, their
      reports, `data/schema.md`, `data/schema_v2.md`, and commits
      `67ccf89`/`965b947` untouched (verified: empty `git diff` against the
      pre-Phase-5D commit for every Phase 5B/5C file)
- [x] Task 1 controller/gains/geometry/success-thresholds/camera constants
      unchanged (verified: empty `git diff` on `run_pick_place.py`,
      `controller_3c.py`, `gripper_scene.py`, `camera_observation.py`)
- [x] `data/task1_canonical_config.json`'s scene/controller/gains/
      thresholds/camera content unchanged; canonical manifest hash
      identical to Phase 5B/5C
- [x] Vendor submodule untouched
- [x] Cube state written only once, pre-lock, through `CubeInitGuard`, in
      both collection and both physics-stepping replay modes
- [x] No push
- [x] Stopped after reporting both policy-action and exact-execution
      replay errors — no scaled collection, no Task 2
