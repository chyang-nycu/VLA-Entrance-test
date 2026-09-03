# Task 1 VLA demonstration dataset — schema v3 (Phase 5D)

`data/task1_prototype_v3.hdf5` — the same three episode roles as Phase 5B/5C
(`nominal`, `x_minus_0.03` successes; `x_plus_0.03` a reachability failure),
collected through the identical deterministic pipeline. `data/task1_canonical_config.json`'s
scene/controller/gains/thresholds/camera content is byte-identical to Phase
5B/5C; only this dataset's own `schema_version`/`action_schema_version`/
`decoder_version`/`decoder_configuration_hash` attributes are new.

This file does **not** replace `data/task1_prototype.hdf5` or
`data/task1_prototype_v2.hdf5` — both remain unmodified as historical
evidence. v3 exists to fix a real fidelity problem in v2's action
representation — see
[`reports/phase5d-policy-action-redesign.md`](../reports/phase5d-policy-action-redesign.md)
for the full three-attempt diagnosis.

## Regeneration

```
source .venv/bin/activate
python3 -m tasks.g1_pick_place.record_demonstrations_v3
```

Deterministic (no RNG anywhere in this pipeline).

## Why v3 exists: the v2 fidelity problem, in one sentence

v2's 10Hz `high_level_actions.cartesian_target` stored one **static
per-phase goal** repeated across every transition inside a phase (e.g. the
same `TRANSPORT_ABOVE_TARGET_POS` for ~40 consecutive transitions); a
decoder with no waypoint-index information re-ramps toward that same
distant point every single 100ms interval instead of following the true
multi-waypoint path, producing up to 9.7cm max policy-action replay error.

## What changed: reference-relative TCP-delta actions, chunked

`policy/high_level_actions/` is replaced by `policy/actions/`, whose
primary fields are **deltas relative to the expert's own commanded
reference trajectory** at each instant, not a repeated final goal — see
[`tasks/g1_pick_place/policy_action_codec.py`](../tasks/g1_pick_place/policy_action_codec.py)'s
module docstring for the full frame verification and the three-attempt
history that led to chunking.

### `policy/actions/` (length `n_transitions`)

| Dataset | Shape | dtype | Semantics |
| --- | --- | --- | --- |
| `tcp_delta_position` | `(n_transitions, 5, 3)` | `float32` | **PRIMARY VLA action.** World frame. `[k, h]` is the expert's commanded TCP-reference translation over the `h`-th of 5 sub-chunks (50Hz, 10 physics steps each) of transition `k`'s 100ms interval. Derived via forward kinematics of `execution/arm_joint_target` only — never from measured/tracked state, never from privileged cube/target state. |
| `tcp_delta_orientation` | `(n_transitions, 5, 3)` | `float32` | **PRIMARY VLA action.** TCP-local (body) frame axis-angle rotation vector, MuJoCo `mju_subQuat` convention: `quat_next = quat_prev ⊗ axisAngle2Quat(delta)` (right/body-frame composition — verified numerically in tests, not assumed). |
| `gripper_command` | `(n_transitions,)` | `float32` | **PRIMARY VLA action.** `0.0` = open, `1.0` = closed. NOT chunked — held constant across all 5 sub-chunks of a transition. |
| `next_arm_joint_target` | `(n_transitions, 7)` | `float32` | OPTIONAL auxiliary/debug field: the commanded joint target at the END of the full 100ms transition (after all 5 sub-chunks). Not a declared VLA action. |
| `state_machine_phase` | `(n_transitions,)` | bytes | METADATA ONLY, not a required policy output. |

Declared VLA policy-input action fields (attr `declared_vla_action_fields`
on the group): `tcp_delta_position`, `tcp_delta_orientation`,
`gripper_command`.

### Why chunked (H=5, not a single 100ms delta)

Three attempts (full numbers in the report):

1. A single whole-interval TCP delta, decoded via one IK solve + one linear
   joint-space ramp across the full 100ms: fixed most of v2's error
   (9.7cm → 2.36cm max) but still missed the ≤10mm target. Root cause:
   `_drive_segment` phases (PREGRASP, RETREAT, ...) issue a fixed
   joint-space set-point ONCE and hold it for the WHOLE segment (often many
   policy transitions) — the true measured TCP converges to that target
   gradually across the entire segment's wall-clock duration, but a
   single-interval decoder must force its own commanded reference to reach
   the SAME target within just one 100ms window, giving the position-servo
   systematically less settle time than the true trial had at that same
   wall-clock point.
2. Adjusting only the ramp speed/shape (a sweep from an immediate step to
   the full 100ms ramp) never found one shape that worked for every large
   single-transition delta simultaneously — a fast ramp fixed one large
   jump (PREGRASP) but made a different, later large jump (RETREAT) worse,
   and vice versa. Diagnosed as a genuine information gap (a single
   100ms-wide delta cannot describe a trajectory containing a large
   sub-100ms-scale reference change), not a tuning problem.
3. **Shipped**: H=5 sub-action chunks (50Hz, 10 physics steps each) give the
   decoder genuine information about where within the 100ms window a large
   reference change happens, instead of guessing an interpolation shape.
   Measured: 8.09mm max (nominal), 5.99mm max (x_minus_0.03) — both under
   the 10mm target. H∈{1,2,10} were also measured; H=5 was the smallest
   chunk size that met the target with margin (H=10 measured slightly
   better at 7.36mm but doubles action-field size for a target already met;
   H=2 measured 43.1mm, still failing).

### `policy/observations/` and `execution/` — unchanged from v2

Same shapes/semantics as
[`schema_v2.md`](schema_v2.md#policy-group-10-hz-unchanged-in-spirit-from-phase-5b)'s
`policy/observations/` and `execution/` groups — this phase only redesigns
`policy/actions/` (formerly `high_level_actions/`); RGB, joint/TCP/gripper
observations, and the full 500Hz execution trace are byte-for-byte the same
kind of data as v2, re-derived from the same underlying trial.

### `privileged/` group

Same as v2, plus two new debug-only datasets:

| Dataset | Shape | dtype | Notes |
| --- | --- | --- | --- |
| `commanded_ref_tcp_pos` | `(n_transitions*5+1, 3)` | `float32` | The forward-kinematics boundary values `policy/actions/tcp_delta_position` was derived from — debug evidence, never a policy input or action. |
| `commanded_ref_tcp_quat` | `(n_transitions*5+1, 4)` | `float32` | Same, orientation. |

## Root attributes (new/changed vs. v2)

`schema_version` ("3.0.0"), `action_schema_version` ("3.0.0"),
`action_frame` (JSON: `{"position": ..., "orientation": ...}`, the exact
frame documentation from `policy_action_codec.py`), `decoder_version`
("v3-chunked-ramp-h5"), `decoder_configuration_hash` (SHA-256 over the
decoder's own config — interpolation method, frame convention, chunk
size — analogous to `canonical_manifest_sha256`; replay rejects a
mismatch). All other root attributes (`mujoco_version`, `physics_hz`,
`substeps_per_transition`, `canonical_manifest_sha256`, etc.) unchanged in
meaning from v2.

## Episode attributes

Same as v2, plus `decoder_configuration_hash` (per-episode copy of the
root attribute, for convenience).

## Replay modes (Section C)

1. **Exact execution replay** — unchanged from Phase 5C (same `execution/`
   schema and semantics); duplicated in `replay_demonstration_v3.py` rather
   than imported, so this module is self-contained.
2. **Policy-action replay** — decodes ONLY `policy/actions/{tcp_delta_position,
   tcp_delta_orientation, gripper_command}` through the chunked decoder
   (`policy_action_codec.ramp_joint_targets`, called once per sub-chunk).
   Maintains its own internal commanded-reference TCP pose, composed
   purely from recorded deltas — never re-read from measured/noisy state
   mid-episode.
3. **Observation-only visualization replay** — unchanged in spirit.

All three verify BOTH the canonical manifest hash AND this dataset's
`decoder_configuration_hash` against the live `policy_action_codec`
configuration, raising on either mismatch.

## Validator (Section G)

`tasks/g1_pick_place/validate_dataset_v3.py` extends every Phase 5C check
to the redesigned action schema and adds: known-synthetic-delta recovery,
rotation-composition-order verification, no off-by-one action shift,
no-phase-goal-repetition (the direct regression test against v2's bug),
decoder-interpolation-across-exactly-10-physics-steps-per-sub-chunk,
terminal-transition padding/mask behavior, causality/leakage checks, and
manifest/decoder-hash mismatch rejection.
