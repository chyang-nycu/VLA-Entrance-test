# Task 1 VLA demonstration dataset — schema (Phase 5B)

`data/task1_prototype.hdf5` — three episodes of the G1 fixed-base Task 1
pick-and-place ("Pick up the red cube and place it in the blue target
area."), collected through the exact same deterministic pipeline every
prior phase used (`tasks.g1_pick_place.run_pick_place.run_trial_pick_place`,
non-oriented IK, Phase 4E-lineage visual/collision-corrected gripper scene).

The single source of truth for which scene/controller/gains/camera/
thresholds are authorized for this dataset is
[`data/task1_canonical_config.json`](task1_canonical_config.json) — see
`tasks/g1_pick_place/canonical_config.py`. The collector, validator, and
replay tool all load that manifest and fail loudly on any mismatch.

## Regeneration

```
source .venv/bin/activate
python3 -m tasks.g1_pick_place.record_demonstrations
```

Deterministic (no RNG anywhere in this pipeline): regenerating produces
byte-identical numeric trajectories. Current file:

- path: `data/task1_prototype.hdf5`
- size: see `logs/phase5b_collection_summary.json` (`hdf5_size_bytes`)
- SHA-256: see `logs/phase5b_collection_summary.json` (`hdf5_sha256`)

## Transition convention (Section A)

```
observation_t -> action_t -> physics substeps -> observation_t+1
```

| Quantity | Value |
| --- | --- |
| Physics frequency | 500 Hz (`TIMESTEP = 0.002s`, `tasks/g1_pick_place/run_pick_place.py`) |
| Policy / control (recording) frequency | 10 Hz |
| Substeps per recorded transition | 50 raw `mj_step` calls |
| RGB frequency | 10 Hz (one frame per recorded observation, not per physics step) |
| Simulation timestamps | `policy_observations/timestamps`, `data.time` in seconds, strictly monotonic |

`observation_0` is captured from a read-only probe: a separate `MjModel`/
`MjData` built from the identical scene and cube offset, stepped zero
times, at the exact pre-first-action RESET state. This pipeline has no RNG
anywhere, so the probe reproduces the real trial's own t=0 state bit-for-
bit. `observation_k` (k>=1) is captured at the physics step where the k-th
50-step block completes; `action_{k-1}` is the actuator control vector
active during the *final* physics substep of that same block, recorded by
the same callback invocation that captures `observation_k`. This
construction makes `len(observations) == len(actions) + 1` a structural
invariant, not a runtime coincidence — the two arrays cannot be shifted
relative to each other (see `tests/test_phase5b_dataset.py`,
`TestTransitionConvention`).

**Terminal-transition convention**: the last recorded observation in each
episode is terminal and has no paired action. Episode metadata separately
records `success` (from the trial's own `task_pass`) and
`termination_reason` (`"DONE"` on success, else the trial's own
`failure_state`, e.g. `"SETTLE_APPROACH"`). Trailing physics steps after
the last complete 50-step block (fewer than 50) are dropped, not recorded
as a partial transition.

**Action recording caveat (disclosed, not hidden)**: during LIFT/
TRANSPORT_ABOVE_TARGET/LOWER_TO_TARGET, the real controller ramps its
joint-space target on *every* physics step (`run_pick_place._drive_smooth`)
— finer-grained than the 10 Hz recording rate. The recorded
`arm_joint_position_target`/`gripper_target` for those transitions is the
value active at the end of the 50-step block, not a trace of the
intermediate ramp. `replay_demonstration.py`'s action replay measures
the resulting (expected, non-zero) deviation from a zero-order-hold
reconstruction rather than assuming it away.

## HDF5 layout

```
/ (root attrs — see below)
└── episodes/
    ├── nominal/                (episode attrs — see below)
    │   ├── policy_observations/   <- the declared VLA policy-input group
    │   │   ├── rgb                 (T+1, 120, 160, 3) uint8, RGB, HWC
    │   │   ├── joint_positions     (T+1, 7) float32   — right-arm joints, rad
    │   │   ├── joint_velocities    (T+1, 7) float32   — right-arm joints, rad/s
    │   │   ├── tcp_pose            (T+1, 7) float32   — [x,y,z,qw,qx,qy,qz]
    │   │   ├── gripper_state       (T+1, 2) float32   — [left_finger_qpos, right_finger_qpos] (m)
    │   │   └── timestamps          (T+1,)   float64   — data.time, seconds
    │   ├── actions/
    │   │   ├── cartesian_target            (T, 3) float32 — active waypoint's target position, m
    │   │   ├── arm_joint_position_target   (T, 7) float32 — == applied arm ctrl (native <position> actuators)
    │   │   ├── gripper_target              (T, 2) float32 — software PD position target (_finger_targets)
    │   │   └── applied_ctrl                (T, 9) float32 — [arm(7), gripper(2)] actual data.ctrl applied
    │   └── privileged/             <- simulator-only ground truth, NEVER part of policy_observations
    │       ├── cube_pos            (T+1, 3) float32
    │       ├── cube_quat           (T+1, 4) float32   — (w,x,y,z)
    │       ├── cube_linvel         (T+1, 3) float32
    │       ├── cube_angvel         (T+1, 3) float32
    │       ├── target_pos          (3,)     float32   — static, target pad center + release height
    │       ├── left_contact        (T+1,)   bool
    │       ├── right_contact       (T+1,)   bool
    │       ├── bilateral_contact   (T+1,)   bool       — left_contact AND right_contact
    │       ├── contact_force_n     (T+1,)   float32   — max bilateral normal contact force this step
    │       └── phase               (T+1,)   utf-8 bytes — state-machine phase string
    ├── x_minus_0.03/  (same layout)
    └── x_plus_0.03/   (same layout — labeled FAILURE episode, see below)
```

T = `transition_count` (episode attr) = number of recorded actions;
`policy_observations`/`privileged` arrays all have `T+1` rows.

### `applied_ctrl` vs. `arm_joint_position_target` / `gripper_target`

For the arm, `arm_joint_position_target` **is** the applied ctrl (this
pipeline uses MuJoCo's native `<position>` servo actuators, whose `ctrl`
value directly *is* the position setpoint — no separate software layer).
For the gripper, `gripper_target` is the software PD position setpoint
(`tasks.g1_pick_place.run_grasp_test_3c._finger_targets`) while
`applied_ctrl`'s gripper channels are the resulting *bounded PD torque*
command (`tasks.g1_pick_place.controller.bounded_pd_step`) — these differ
numerically from `gripper_target` by construction, not by error.

## Root attributes

| Attribute | Meaning |
| --- | --- |
| `schema_version` | `"1.0.0"` |
| `mujoco_version` | e.g. `"3.3.6"` |
| `robot_embodiment` | Unitree G1, fixed-base torso-constrained upper-body manipulation baseline |
| `unitree_mujoco_pinned_commit` | vendor submodule pin |
| `project_git_commit` | project HEAD **at dataset-generation time** (before this phase's own commit — the dataset is generated, then committed) |
| `task_id` | `"g1_task1_pick_place"` |
| `task_instruction` | the natural-language Task 1 instruction |
| `transition_convention`, `policy_control_hz`, `physics_hz`, `substeps_per_transition`, `rgb_hz`, `terminal_convention` | Section A fields, see above |
| `camera_params_json` | JSON dump of the canonical manifest's `camera` object |
| `coordinate_conventions` | world frame, quaternion order, RGB channel order |
| `canonical_manifest_sha256` | hash of `data/task1_canonical_config.json` this dataset was collected under |

## Episode attributes

`instruction`, `variant_id`, `seed` (always `0` — no RNG in this pipeline;
retained as a schema field for future stochastic variants), `success`
(bool, from the real objective detector's `task_pass`), `termination_reason`,
`failure_reason`, `train_eligible` (bool, `== success` in this dataset),
`transition_count`, `canonical_manifest_sha256`, `cube_xy_offset`,
`final_xy_target_error_m` (`NaN` for the failure episode, since the cube
never reached placement to measure).

## The three episodes

| variant_id | cube_xy_offset | success | termination_reason | train_eligible |
| --- | --- | --- | --- | --- |
| `nominal` | (0.0, 0.0) | True | `DONE` | True |
| `x_minus_0.03` | (-0.03, 0.0) | True | `DONE` | True |
| `x_plus_0.03` | (0.03, 0.0) | **False** | `SETTLE_APPROACH` | **False** |

### Deviation from the original plan, disclosed

The user's Phase 5B authorization specified `y_plus_0.03` as the labeled
placement-failure episode, based on Phase 4B/4C history (grasp succeeds,
final placement margin 20.4mm > 15mm). Phase 5B **re-measured** this under
the *current* canonical config (Phase 4E's gripper gains 320/20 +
LIFT/TRANSPORT/LOWER trajectory smoothing, both added after Phase 4B) and
found `y_plus_0.03` now **passes** `task_pass=True` deterministically
(final xy_err=2.07mm) — those controller improvements closed the
placement-margin gap that variant used to expose. A sweep of nearby and
alternate offsets found no remaining placement-margin-only failure zone
under the current config: every tested offset either succeeds cleanly
(xy_err in the low single-digit mm) or fails at `SETTLE_APPROACH` (a
grasp/reachability failure, before any grasp is attempted). `x_plus_0.03`
was substituted as the Phase 5B failure episode — a genuine, deterministic,
reproducible failure under the current config, previously documented in
Phase 4A/4B history as IK-unreachable (27.1mm residual). It is a
grasp-approach failure, not a placement-margin failure, and is documented
as such rather than mislabeled to match the original plan. See
`data/task1_canonical_config.json`'s `instruction_variants.y_plus_0.03` and
`.x_plus_0.03` entries for the full measured record.

## Validation

```
python3 -m tasks.g1_pick_place.validate_dataset
```

Checks required groups/attributes, matching time dimensions, monotonic
timestamps, finite values, quaternion norms, action bounds, image
dtype/range/variance, nonempty instructions, **independently recomputed**
success (reruns the deterministic simulation for the episode's offset and
compares `task_pass` — does not trust the stored flag), failure-episode
train-split exclusion, dataset reopenability, and absence of
post-initialization cube-state discontinuities. See
`reports/phase5b-data-pipeline.md` for the full result.

## Replay

```
python3 -m tasks.g1_pick_place.replay_demonstration replay --variant nominal
python3 -m tasks.g1_pick_place.replay_demonstration visualize --variant nominal
```

`replay` restores only the recorded initial condition (through
`CubeInitGuard`'s same pre-lock boundary every other module uses — never
overwritten again), applies the recorded actions through real physics, and
reports maximum joint/TCP/cube trajectory deviation against the original
recording plus the tolerances used to judge them (see
`tasks/g1_pick_place/replay_demonstration.py`). `visualize` plays back
stored RGB/state with **no physics stepped**, for inspection. Both modes
verify the dataset's stored `canonical_manifest_sha256` against the live
manifest and raise on a mismatch.
