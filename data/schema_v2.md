# Task 1 VLA demonstration dataset — schema v2 (Phase 5C)

`data/task1_prototype_v2.hdf5` — the same three episode roles as Phase 5B
(`nominal`, `x_minus_0.03` successes; `x_plus_0.03` a reachability failure),
collected through the identical deterministic pipeline
(`tasks.g1_pick_place.run_pick_place.run_trial_pick_place`, non-oriented IK,
Phase 4E-lineage gripper scene, same gains/thresholds/camera as Phase 5B —
`data/task1_canonical_config.json` is unchanged in content; only this
dataset's own `schema_version` attribute is new).

This file does **not** replace `data/task1_prototype.hdf5`
([`data/schema.md`](schema.md)), which remains the original Phase 5B
prototype evidence, unmodified. v2 exists to fix a real fidelity problem in
v1's action representation — see
[`reports/phase5c-replay-fidelity.md`](../reports/phase5c-replay-fidelity.md)
Section A for the full diagnosis.

## Regeneration

```
source .venv/bin/activate
python3 -m tasks.g1_pick_place.record_demonstrations_v2
```

Deterministic (no RNG anywhere in this pipeline). See
`logs/phase5c_replay_fidelity.json` for the current file's size/SHA-256/
manifest hash.

## Why v2 exists: the v1 fidelity problem, in one sentence

v1 stored one action snapshot per 100 ms policy transition and Phase 5B's
own replay held that snapshot constant (zero-order hold, "ZOH") for the
whole 100 ms block; the real controller actually re-ramps its commanded
joint target on **every** physics step (500 Hz) during `LIFT`/
`TRANSPORT_ABOVE_TARGET`/`LOWER_TO_TARGET`, so ZOH-replay discarded that
entire intra-block ramp — this, and only this, is where Phase 5B's ~4.9 cm
nominal replay error came from (quantified below).

## Two-rate representation

Each episode now has two co-existing groups instead of one:

- `policy/` — unchanged rate (10 Hz) from Phase 5B: `observations`
  (including RGB) at `T+1`, `high_level_actions` at `T`. This is what an
  onboard VLA policy would actually see/emit.
- `execution/` — new: one row per physics step (500 Hz), covering exactly
  what the real low-level controller commanded and how the robot/cube
  actually moved in between two policy observations. RGB is **not**
  duplicated here — only `policy/observations/rgb` carries frames.

```
policy/observations/timestamps[k]     -- k = 0 .. n_transitions
execution/timestamps[i]                -- i = 0 .. n_transitions*50 - 1
execution/transition_index[i] = k  <=>  policy/observations[k] < t_i <= policy/observations[k+1]
```

### `policy/` group (10 Hz, unchanged in spirit from Phase 5B)

`policy/observations/` (length `n_transitions + 1`):

| Dataset | Shape | dtype | Notes |
| --- | --- | --- | --- |
| `rgb` | `(N, 120, 160, 3)` | `uint8` | RGB, HWC. Only stored here, not in `execution/`. |
| `joint_positions` | `(N, 7)` | `float32` | right-arm joints, `RIGHT_ARM_JOINTS` order |
| `joint_velocities` | `(N, 7)` | `float32` | |
| `tcp_pose` | `(N, 7)` | `float32` | `[x,y,z,qw,qx,qy,qz]` |
| `gripper_state` | `(N, 2)` | `float32` | finger joint positions |
| `timestamps` | `(N,)` | `float64` | `data.time`, seconds, strictly monotonic |

`policy/high_level_actions/` (length `n_transitions`):

| Dataset | Shape | dtype | Semantics |
| --- | --- | --- | --- |
| `cartesian_target` | `(n_transitions, 3)` | `float32` | the static per-phase Cartesian goal in effect for physics steps `[T*50+1 .. (T+1)*50]` — a **high-level command**, not a per-step interpolation endpoint. Identical across every transition inside one phase (e.g. all ~20-60 transitions of `LOWER_TO_TARGET` share the same value). |
| `gripper_command_open` | `(n_transitions,)` | `bool` | open/close command a policy would emit |

This is deliberately unchanged from Phase 5B — the fix in this phase is in
**how replay decodes it**, not in what is stored at 10 Hz (see Replay modes
below and the report's Section A/D for why a richer per-transition
Cartesian representation was considered and not required to meet this
phase's goals).

### `execution/` group (500 Hz = physics rate, new in v2)

Length `n_transitions * 50` per episode (trailing steps past the last
complete 50-step block — e.g. a mid-block early failure — are truncated,
same truncation rule Phase 5B applied to observations).

| Dataset | Shape | dtype | Semantics |
| --- | --- | --- | --- |
| `transition_index` | `(M,)` | `int32` | which policy transition (0-indexed) this row belongs to; contiguous blocks of exactly 50 |
| `timestamps` | `(M,)` | `float64` | `data.time` at this physics step |
| `arm_joint_target` | `(M, 7)` | `float32` | the linearly-ramped joint target commanded THIS physics step (== `data.ctrl` for the arm's native `<position>` actuators) |
| `gripper_target` | `(M, 2)` | `float32` | desired open/close finger position this step (PD setpoint, not the resulting torque) |
| `applied_ctrl` | `(M, 9)` | `float32` | the literal `data.ctrl` vector sent to `mujoco.mj_step` this step — `[arm(7), gripper(2)]`; gripper channels are the bounded-PD torque output, numerically different from `gripper_target` |
| `joint_positions` / `joint_velocities` | `(M, 7)` | `float32` | live arm state after this step |
| `tcp_pose` | `(M, 7)` | `float32` | `[x,y,z,qw,qx,qy,qz]` after this step |
| `cube_pos` / `cube_quat` | `(M, 3)` / `(M, 4)` | `float32` | live cube state after this step (read-only observation — never written) |
| `phase` | `(M,)` | bytes | e.g. `"LOWER_TO_TARGET_wp13"` — the fine-grained internal phase/waypoint label active this step |

`execution_hz = 500.0` (root attribute) — chosen because this is the actual
rate at which the arm's commanded set-point changes during ramped
segments (verified directly from `run_pick_place._drive_smooth`, which
computes a new linearly-interpolated `ramped_target` and calls `_step_once`
on every physics step, not once per waypoint); anything coarser would
silently reintroduce the same information loss this phase exists to fix.

### `privileged/` group (10 Hz, mirrors Phase 5B for continuity)

Simulator-only ground truth, NOT part of the declared VLA policy-input
group (`policy/observations/`): `cube_pos`, `cube_quat`, `target_pos`,
`phase` (the coarse per-transition phase label). `execution/cube_pos`/
`cube_quat` carry the same physical quantity at the finer 500 Hz rate, for
replay comparison only.

## Root attributes

`schema_version` ("2.0.0" — this dataset's own schema-format version;
unrelated to and does not change `data/task1_canonical_config.json`'s own
`schema_version` field), `mujoco_version`, `robot_embodiment`,
`unitree_mujoco_pinned_commit`, `project_git_commit`, `task_id`,
`task_instruction`, `transition_convention`, `policy_control_hz` (10.0),
`physics_hz` (500.0), `execution_hz` (500.0), `substeps_per_transition`
(50), `rgb_hz` (10.0), `terminal_convention`, `camera_params_json`,
`coordinate_conventions`, `canonical_manifest_sha256`.

## Episode attributes

`instruction`, `variant_id`, `seed`, `success`, `termination_reason`,
`failure_reason`, `train_eligible`, `transition_count`,
`execution_row_count`, `canonical_manifest_sha256`, `cube_xy_offset`,
`final_xy_target_error_m`.

## Replay modes (Section C)

1. **Exact execution replay** (`replay_demonstration_v2.replay_exact_execution`) —
   feeds `execution/applied_ctrl` directly into `data.ctrl` at every
   physics step; never re-solves IK, never re-derives a target from a
   coarser sample. Restores only the initial cube pose through
   `CubeInitGuard`'s pre-lock window; never writes cube state after.
2. **Policy-action replay** (`replay_policy_actions`) — uses ONLY the 10 Hz
   `high_level_actions` stream, decoded through a ramp reusing
   `solve_ik_waypoint`/`bounded_pd_step` (the same primitives collection
   uses), re-targeting every 100 ms instead of holding constant. See the
   report for the honest, measured result of this mode, including where it
   does and does not meet the ≤10 mm target.
3. **Observation-only visualization replay** (`visualize_episode`) — plays
   back stored RGB/state with zero physics stepped.

All three load `data/task1_canonical_config.json` and reject (raise
`canonical_config.ManifestMismatchError`) a dataset whose stored
`canonical_manifest_sha256` does not match the live manifest.

## Validator (Section D/E)

`tasks/g1_pick_place/validate_dataset_v2.py` extends every Phase 5B check
to the two-rate schema and adds the Section E transition-alignment checks:
observation/action count invariant, execution-row transition-index
contiguity, execution timestamps falling within their transition's
interval, final-execution-state-vs-final-observation agreement, and
reconstructed-action-matches-recorded-target-at-the-declared-instant (this
is also what `tests/test_phase5c_replay_fidelity.py`'s deliberately
shifted-action tamper test exercises).
