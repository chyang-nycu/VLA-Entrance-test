# Phase 5C — VLA Action/Replay Fidelity

Date: 2026-09-03

Fixes the root cause of Phase 5B's ~4.9cm nominal replay error, by adding a
second, execution-rate data group and two clearly distinguished replay
modes. Does not modify Task 1 physics, gains, geometry, success
thresholds, or the onboard camera. `data/task1_prototype.hdf5`,
`reports/phase5b-data-pipeline.md`, and commit `67ccf89` are preserved
unmodified as the original Phase 5B prototype evidence; all new work in
this phase is additive (`_v2` files). Full field reference:
[`data/schema_v2.md`](../data/schema_v2.md). Raw numbers:
[`logs/phase5c_replay_fidelity.json`](../logs/phase5c_replay_fidelity.json).

## Section A — timing and action-semantics audit

| Quantity | Value |
| --- | --- |
| Physics timestep | 0.002s (500 Hz) |
| Low-level actuator/control update rate | 500 Hz — the arm's commanded joint target is linearly re-interpolated and re-applied on **every** physics step during `LIFT`/`TRANSPORT_ABOVE_TARGET`/`LOWER_TO_TARGET` (`run_pick_place._drive_smooth`, confirmed by reading the source: the ramp loop is nested inside the per-waypoint loop) |
| IK/waypoint solve rate | Once per waypoint: LIFT 30wp/1.5s = 20Hz, TRANSPORT 40wp/2.0s = 20Hz, LOWER 60wp/2.0s = 30Hz — but the *ramp between* two consecutive solved waypoints still updates every physics step |
| Policy observation/action rate | 10 Hz (unchanged from Phase 5B) |
| RGB rate | 10 Hz (unchanged) |
| Low-level updates per 10Hz policy transition | 50 |
| Ramps recomputed every physics step? | **Yes**, confirmed directly in `_drive_smooth`'s source, not assumed |

### Per-field semantic classification

| Field (v1) | Classification |
| --- | --- |
| `cartesian_target` | high-level Cartesian command — the static per-phase goal in effect for the whole 100ms block, often identical across many consecutive transitions within one multi-second phase |
| `arm_joint_position_target` | actual applied actuator command, but sampled **once**, at the single instant the 50th physics step of the block completes — an instantaneous end-of-interval sample, not a value that was ever "held" for the interval |
| `gripper_target` | software PD setpoint at that same end-of-interval instant |
| `applied_ctrl` | same instantaneous end-of-interval sample, concatenated with the gripper's resulting bounded-PD torque |

**This is the bug**: Phase 5B's replay treated `arm_joint_position_target`
(an instantaneous end-of-interval sample) as if it were "target held during
interval" and zero-order-held it across the whole 100ms block, discarding
the real 50-physics-step ramp that happened inside that block.

### Quantified: where the 4.9cm came from

Replaying the *same* nominal episode's literal 500Hz `applied_ctrl` trace
(exact execution replay, this phase) drops the max TCP replay error from
Phase 5B's **4.87cm** to **3.65 x 10⁻⁸ m** — a reduction of roughly 1.3
million times. **The entire 4.9cm error is attributable to the 10Hz
zero-order-hold discarding the intra-transition ramp.** No other source
(actuator lag, controller nonlinearity, floating-point nondeterminism)
contributes meaningfully — the residual after fixing the ZOH problem is
sub-micron.

## Section B — two-rate schema

`policy/` (10Hz, unchanged in spirit from Phase 5B) and `execution/` (new,
500Hz — one row per physics step) now coexist per episode. RGB is stored
only in `policy/observations/rgb`, never duplicated at 500Hz. Full shapes
in [`data/schema_v2.md`](../data/schema_v2.md).

`execution_hz = 500.0` was chosen (not a coarser rate) because that is the
real rate at which the controller's commanded set-point changes — verified
directly in source, not picked arbitrarily.

## Section C — three replay modes

1. **Exact execution replay** — replays `execution/applied_ctrl` literally,
   step by step, no IK re-solve.
2. **Policy-action replay** — decodes only the 10Hz `high_level_actions`
   stream through a ramp reusing the same `solve_ik_waypoint`/
   `bounded_pd_step` primitives the real controller uses, re-targeting the
   recorded static goal every 100ms instead of holding it constant (this
   is the direct fix for Phase 5B's bug).
3. **Observation-only visualization replay** — RGB/state playback, zero
   physics stepped.

## Section D — fidelity results, honestly reported

### Exact execution replay — targets met with large margin

| variant | max joint err (rad) | max TCP err (m) | max cube err (m) | within tolerance |
| --- | --- | --- | --- | --- |
| `nominal` | 7.04e-08 | 3.65e-08 | 3.59e-08 | **True** |
| `x_minus_0.03` | 4.30e-08 | 3.38e-08 | 3.47e-08 | **True** |
| `x_plus_0.03` | 7.69e-08 | 4.40e-08 | 3.35e-08 | **True** |

Targets: joint ≤1e-4 rad, TCP ≤1e-3 m, cube ≤1e-3 m. All three episodes
beat the targets by 4-5 orders of magnitude. Not literally bit-exact
(residual ~1e-8, floating-point accumulation over thousands of `mj_step`
calls on a freshly-built `MjData`/model) — this is the measured, justified
tolerance, not a loosened one.

On "identical termination state and success label": exact execution
replay does not re-invoke a separate success-detector function on the
replayed trajectory — it reproduces the *same* physical run (identical
`applied_ctrl` sequence, identical physics engine) to within ~1e-8 across
the whole episode including the final state, so the stored `success` label
(a property of that one physical trajectory) necessarily carries over
rather than being independently redetermined. This is a stronger guarantee
than "the same detector agreed twice" would be, since there was never a
second, independent trial to potentially disagree — but it means the
number reported here is "final-state trajectory agreement," not "success
detector re-run and matched," and this report says so precisely rather
than implying a re-invocation that didn't happen.

### Policy-action replay — max target NOT met; final-state target met; honest gap documented

| variant | max TCP err (m) | final TCP err (m) | final cube err (m) | within tolerance (max) |
| --- | --- | --- | --- | --- |
| `nominal` | **0.0969** | 0.00633 | 0.01134 | **False** |
| `x_minus_0.03` | **0.0983** | 0.00631 | 0.00883 | **False** |
| `x_plus_0.03` | 0.0215 | 0.00382 | ~0 | False (degenerate case, see below) |

**The ≤10mm target on maximum TCP error during the episode is not met** —
measured ~97-98mm max for the two successful episodes. This is reported as
measured, not adjusted to pass.

**Diagnosed mechanism** (traced to the specific worst transitions:
`LOWER_TO_TARGET_wp7` at 96.9mm, `TRANSPORT_ABOVE_TARGET_wp7` at 94.4mm):
`high_level_actions.cartesian_target` stores one **static per-phase goal**,
repeated identically across every 10Hz transition inside a multi-second
phase (e.g. the same value for all ~40 transitions of
`TRANSPORT_ABOVE_TARGET`). The real controller reaches that goal via a
slow 40-or-60-waypoint linear ramp over 2 seconds. A decoder that only has
the repeated static goal (no waypoint-index information — a deployed
policy wouldn't have it either) re-ramps toward that same distant point
within every single 100ms interval, producing a faster, more direct path
shape than the true trajectory. This causes large **mid-phase** position
error while still converging toward the same final point each phase
actually reaches — which is exactly why the **final** TCP/cube error at
episode end is small (6.3mm TCP, ~1cm cube) even though the max error
along the way is not.

This is a genuine structural limitation of a "static per-phase goal"
high-level action representation at 10Hz, not a decoder bug: closing it
would need either a richer per-transition action (e.g. the next
incremental waypoint, not the final phase goal) or accepting final-state-
only fidelity for now. **Redesigning the stored action semantics is out of
scope for this phase** (Phase 5C's authorization asked for a two-rate
dataset and honest replay validation, not a new action representation);
this gap is documented for a future phase to decide whether to address,
consistent with this project's standing rule to diagnose rather than force
a pass.

`x_plus_0.03`'s policy-replay numbers are near-degenerate: the episode
fails at `SETTLE_APPROACH` before any multi-second ramped phase begins, so
its 21.5mm max mostly reflects `PREGRASP`/`APPROACH` IK-decoder
differences, not the LOWER/TRANSPORT divergence mechanism above — reported
separately, not blended into the other two episodes' numbers.

## Section E — transition-alignment tests

`tests/test_phase5c_replay_fidelity.py`: 17 tests, all passing, covering
every one of the seven required properties (observation/action count
invariant; execution-row-to-transition mapping; execution timestamps
inside their transition interval; final-execution-state-vs-final-
observation agreement; no one-step action shift; consistent terminal-
transition representation; reconstructed-action-matches-recorded-target),
plus a deliberately shifted-action tamper test
(`test_manually_shifted_action_array_fails_validation`) that `np.roll()`s
`nominal`'s `cartesian_target` by one transition in a copied file and
confirms `validate_dataset_v2.validate_file` reports the mismatch as an
error — proving the check actually catches a shift, not just a
hypothetical one.

## Section F — v2 dataset

`data/task1_prototype_v2.hdf5`: 4,995,223 bytes (4.76 MiB — larger than
v1's 1.66 MiB because of the added 500Hz execution trace), SHA-256
`89dc2410bfa98631047bf3b951e94bd8feec01130fd5ce28c8e8dae45340b4a7`,
committed directly (small enough). Canonical manifest hash
`f7375efc18d00fd83b2c75228bac76a7c23922913e104d6970e9b8241b9c290b` —
identical to Phase 5B's, confirming the underlying scene/controller/gains/
thresholds/camera are unchanged. Regenerate:
`python3 -m tasks.g1_pick_place.record_demonstrations_v2` (deterministic).

| variant | success | termination | transitions | execution rows | wall time |
| --- | --- | --- | --- | --- | --- |
| `nominal` | True | DONE | 132 | 6600 | 8.84s |
| `x_minus_0.03` | True | DONE | 132 | 6600 | 8.59s |
| `x_plus_0.03` | False | SETTLE_APPROACH | 31 | 1550 | 2.22s |

**`x_plus_0.03`'s execution trace**: 1550 rows spanning `RESET` →
`PREGRASP` → `SETTLE_PREGRASP` → `APPROACH` → `SETTLE_APPROACH` — entirely
pre-grasp settling steps. There is no `LIFT`/`TRANSPORT`/`LOWER` content
because the trial genuinely never reaches `CLOSE` (the documented
`SETTLE_APPROACH` reachability failure from Phase 5B/4A/4B). The execution
trace is present and complete for every physics step the trial actually
ran; it is simply shorter because the episode itself is shorter — this is
not a truncation bug, and `validate_dataset_v2` confirms
`execution_row_count == n_transitions * 50` holds for this episode exactly
as for the other two.

Validator: `VALIDATION PASSED`, 0 errors (independently recomputes success
by rerunning the trial, same pattern as Phase 5B).

## Test suite

`tests/test_phase5c_replay_fidelity.py`: 17 new tests. Full suite:
**231 tests, 0 unexpected failures** (214 pre-existing from Phase 5B + 17
new).

## Compliance checklist

- [x] `data/task1_prototype.hdf5`, `reports/phase5b-data-pipeline.md`,
      `data/schema.md`, and commit `67ccf89` untouched (verified: empty
      `git diff` against the pre-Phase-5C commit for every Phase 5B file)
- [x] Task 1 controller/gains/geometry/success-thresholds/camera constants
      unchanged (verified: empty `git diff` on `run_pick_place.py`,
      `controller_3c.py`, `gripper_scene.py`, `camera_observation.py`)
- [x] `data/task1_canonical_config.json`'s scene/controller/gains/
      thresholds/camera content unchanged; canonical manifest hash
      identical to Phase 5B's
- [x] Vendor submodule untouched
- [x] Cube state written only once, pre-lock, through `CubeInitGuard`, in
      both collection and both physics-stepping replay modes
- [x] No push
- [x] Stopped after reporting both replay modes' errors — no scaled
      collection, no Task 2
