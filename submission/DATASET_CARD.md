# Dataset Card — Task 1 VLA-Oriented Demonstration Dataset (v1, scaled)

`data/task1_demonstrations_v1.hdf5` (Phase 5E). This is a **VLA-oriented
demonstration dataset prototype with validated schema and replay
instrumentation** — it is explicitly not described as model-ready, and no
model was trained or evaluated on it in this project.

## Intended use

- Behavior-cloning-style research on a single, narrow pick-and-place task
  (Task 1) for a fixed-base, torso-constrained Unitree G1 with a
  task-local physical parallel gripper.
- Studying VLA-style dataset schema design: two-rate (10Hz policy / 500Hz
  execution) recording, action-chunked TCP-delta representations, replay
  fidelity as a first-class, measured, honestly-reported property.
- Diagnostic study of *where* a stored action representation's replay
  fidelity degrades (see the RETREAT-phase finding, `data/
  task1_demonstrations_v1_quality.json`) — this dataset is itself useful
  evidence for that kind of investigation, independent of any model.

## Non-intended use

- **Not** validated for training a deployable manipulation policy — no
  model was trained or evaluated against it.
- **Not** representative of target-position generalization: every episode
  uses the same fixed target-pad position; do not use this dataset to
  claim or evaluate target-position generalization.
- **Not** representative of cube-orientation (yaw) variation: cube yaw is
  fixed at 0 in every episode.
- **Not** a full-body or free-standing manipulation dataset: the robot's
  pelvis and torso are rigidly welded to the world throughout.
- **Not** validated for the RETREAT-phase motion specifically under a
  policy-replay decode — 7 of 29 episodes exceed the project's own 10mm
  fidelity target there (see "Replay quality distribution" below); a
  policy trained naively on the whole trajectory should be evaluated with
  this in mind, not assumed uniformly high-fidelity end to end.

## Policy inputs (declared, `policy/observations/`)

- RGB image, `head_cam`, 10Hz, uint8, true render size 160×120 (raw, no
  text/video overlay).
- Joint positions/velocities (right arm + gripper).
- TCP pose.
- Gripper state.

## Action representation (`policy/actions/`)

- `tcp_delta_position`: world-frame TCP position delta, shape `[T, H, 3]`,
  `H=5` sub-deltas per 10Hz transition (50Hz effective sub-action rate).
  World frame is used because this fixed-base robot's pelvis/torso weld
  has been independently measured to coincide with the robot-base frame to
  within ~0.19mm (Phase 5A).
- `tcp_delta_orientation`: TCP-local (body) frame rotation vector
  (axis-angle), same `[T, H, 3]` chunking, via MuJoCo's
  `mju_subQuat`/`mju_quatIntegrate` pair (right/body-frame composition,
  numerically verified against a hand-built left-vs-right discriminator).
- `gripper_command`: `0.0`=open / `1.0`=closed, held constant per 10Hz
  transition, shape `[T]`.
- `next_arm_joint_target`: optional auxiliary/debug field, not a declared
  policy action — reconstructible purely from that transition's own
  `execution/arm_joint_target`, never from a future observation.
- `state_machine_phase`: metadata only, not an action.
- Every action field is derived from the expert's own commanded joint
  trajectory (`execution/arm_joint_target`) via forward kinematics — never
  from privileged cube/target state, and never leaking future observations
  (verified by dedicated causality tests,
  `tests/test_phase5d_policy_actions.py`).

## Privileged metadata (`privileged/`, separate top-level group)

Cube pose, target pose, contact state, ground-truth state-machine phase.
Never nested under `policy/observations/` — a downstream consumer that
respects the declared-policy-input boundary cannot accidentally train on
privileged state.

## Collection controller

Non-oriented, position-priority waypoint IK
(`controller_3c.py::solve_ik_waypoint`), bounded MuJoCo position servos,
`use_oriented_ik=False` throughout. The Phase 4F orientation-constrained
IK variant (`solve_ik_waypoint_oriented`) exists in the codebase but
**failed** (fails at `SETTLE_LOWER`, never completes placement) and was
**never** used for any episode in this dataset.

## Episode / split counts

- 32 configurations attempted → 28 task successes, 4 diagnostic (1 failed
  interaction, 3 rejected pre-physics by reachability).
- Splits: **train 16, val 4, test 4, diagnostics 8** (all fixed by seed and
  disjoint spatial band, assigned before collection).
- Behavior-cloning-eligible pool: exactly the 24 train/val/test successes
  (`train_eligible=True`); all 8 diagnostics-split episodes are
  `train_eligible=False` regardless of individual physical outcome.

## Diversity

- Cube XY position: continuous, uniform, `cube_dx ∈ [-0.035,-0.005]m`,
  `cube_dy ∈ [-0.01,0.035]m` for the 24-episode success envelope, plus 8
  fixed diagnostic probes at/beyond its boundary.
- Cube yaw: fixed at 0 (not varied — no pilot evidence yet that the shared
  controller physically grasps under yaw).
- Target position: fixed at `(0,0)` offset for every episode (see "Known
  biases / fixed target limitation" below).
- Instruction: 3 deterministic paraphrase templates ("Pick up the red cube
  and place it in the blue target area." / "Move the red cube to the blue
  target." / "Place the red block on the blue pad."), selected
  deterministically per episode seed; the physical task is identical for
  any template at a given seed.
- Random seed: 32 fixed integers, no seed reused, no ambient RNG state
  anywhere in the pipeline (fully deterministic regeneration).

## Known biases

- **Fixed target limitation**: the blue target pad's rendered position is
  identical in every episode (`target_xy_offset=(0,0)`). This was a
  deliberate, disclosed decision, not an oversight: the pad is fixed MJCF
  geometry with no offset parameter, and varying only the controller's
  internal target while leaving the rendered pad fixed would have produced
  RGB frames showing the pad in the wrong place relative to the true
  placement goal — a physically dishonest mismatch this project's
  standing rules exist to prevent. A future phase could enable real
  target-position variation via an authorized scene-geometry change.
- **Torso-mounted camera limitation**: `head_cam` is mounted on
  `torso_link`, not on a separate head/neck body (the G1 model has none —
  the head mesh is itself a static geom on `torso_link`). Document it
  exactly as "torso-mounted onboard RGB camera positioned near head
  height," never as parented to a head body.
- **x-positive workspace reachability limitation**: cube positions in the
  `+x` direction from nominal (and `-y`, in the earlier Phase 4A grid) are
  not reachable by this controller/gripper geometry — a pre-run IK
  reachability filter correctly predicts and excludes these before any
  physics trial runs, and the pilot re-confirmed this exact asymmetry.
- **Spatial coverage is one contiguous region, not the full table**: only
  a ~3cm×4.5cm continuous cube-position envelope (plus boundary probes) is
  covered, calibrated for grasp/placement success — this dataset does not
  claim coverage of the full table surface.
- **Out-of-distribution definition**: val (`cube_dx ∈ [-0.020,-0.013)`)
  and test (`cube_dx ∈ [-0.013,-0.005]`) occupy `cube_dx` bands strictly
  disjoint from train (`cube_dx ∈ [-0.035,-0.020)`) — this is the dataset's
  intentional OOD axis. `cube_dy` and instruction template are drawn from
  the same full ranges/set for all three splits; no OOD claim is made
  along those axes.

## Replay quality distribution

See [`data/task1_demonstrations_v1_quality.json`](../data/task1_demonstrations_v1_quality.json)
for the full per-episode table. Headline:

- Exact-execution replay (500Hz literal control trace): 29/29 episodes
  within 1mm (max 3.31e-5m, mean 1.25e-6m).
- Policy-action replay (10Hz/H=5-chunked decoded action stream) against a
  10mm max-TCP-error target: **18/24 BC-pool episodes pass** (22/29
  dataset-wide). All 7 episodes exceeding the target first diverge at the
  post-release RETREAT phase — verified to never affect cube placement or
  task-success determination (both occur before RETREAT begins).

## Quality masks (recommended)

Defined in the sidecar file above, as episode-ID lists:

- **`full_episode_high_fidelity`** (18/24 BC-pool, 22/29 dataset-wide):
  whole-episode max policy-replay TCP error ≤10mm, including RETREAT.
- **`task_execution_through_release`** (24/24 BC-pool, 29/29
  dataset-wide — the recommended default for standard pick-and-place
  behavior cloning): fidelity holds through cube release; empirically
  equal to "all episodes" in this dataset, since zero episodes diverge
  before release.
- **`diagnostics_only`** (8 episodes): the pre-registered diagnostics
  split, for reachability-boundary/failure-mode study, never for default
  BC training.

## Licensing / dependency notes

- This project's own code (`tasks/`, `tests/`, `data/`, `submission/`) has
  no separate LICENSE file at the time of writing (repository-wide; noted
  in Section H of the main report as a hygiene observation, not resolved
  in this phase).
- `vendor/unitree_mujoco` (the pinned simulator submodule providing the G1
  model) is BSD-3-Clause, copyright Hangzhou Yushu Technology Co., Ltd.
  ("Unitree Robotics"). Never modified by this project.
- Python dependencies (`mujoco`, `numpy`, `imageio`, `h5py`, `matplotlib`,
  `imageio-ffmpeg`) each carry their own upstream licenses (BSD/MIT/Apache
  family — see each package's own `dist-info/licenses/` in `.venv`); none
  are redistributed by this dataset.

## Privacy statement

This dataset contains only simulated robot/object state and simulated RGB
renders of a MuJoCo scene (a robot, a table, a cube, a target pad). It
contains no real-world imagery, no personal data, and no data derived from
any human subject.

## Checksum and regeneration

- File: `data/task1_demonstrations_v1.hdf5`
- Size: 62,196,309 bytes
- SHA-256: `accfe4461e7decc0dac2f7b959496487ba94747a504c534e149b8593c7749f21`
- Canonical manifest SHA-256: `f7375efc18d00fd83b2c75228bac76a7c23922913e104d6970e9b8241b9c290b`
- Decoder configuration hash: `9ef9bd49fc376f125c10be5b46d21a45148a79a2057f32d28afc849629a81d86`
- Collection spec SHA-256: `3e007e1fab0fca8ff5916ab42803470396f753b31e0fb89430564a5b56807d1d`
- Regenerate: `python3 -m tasks.g1_pick_place.collect_dataset` (deterministic,
  no ambient RNG; verified byte-reproducible for smaller sibling datasets
  in this same pipeline during this phase — see `submission/REPRODUCE.md`).
