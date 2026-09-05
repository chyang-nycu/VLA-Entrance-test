# Phase 5E — Scaled Task 1 VLA Demonstration Collection

Date: 2026-09-03

Collects a compact but genuinely varied Task 1 dataset using the Phase 5D
v3 action schema/decoder UNCHANGED and the unchanged canonical manifest.
`data/task1_prototype.hdf5`, `data/task1_prototype_v2.hdf5`,
`data/task1_prototype_v3.hdf5`, their reports/schemas, and commits
`67ccf89`/`965b947`/`621a63a` are preserved unmodified as prior-phase
evidence — every file in this phase is new/additive. Does not implement
Task 2 or train a model. Full field reference:
[`data/schema_v3.md`](../data/schema_v3.md) (action schema, unchanged).
Raw numbers: [`logs/phase5e_collection_summary.json`](../logs/phase5e_collection_summary.json),
[`logs/phase5e_validation.json`](../logs/phase5e_validation.json).

## Section A — deviation disclosure: target position not varied

**Read this first.** The authorization asked for a "target XY
distribution" alongside cube position. This was investigated and **not
implemented**, for a hard structural reason found during implementation
(full account in `data/task1_collection_spec.json`'s
`DEVIATION_DISCLOSURE_target_position_not_varied` key):

The blue target pad is fixed visual/collision geometry baked into the MJCF
scene at generation time (`tasks.g1_pick_place.gripper_scene.write_grasp_scene_4b`,
`TARGET_POS` module constant, no offset parameter). A first implementation
attempt added a `target_xy_offset` parameter to `run_trial_pick_place`
that moved only the CONTROLLER's internal transport/lower/retreat
waypoints, leaving the rendered pad fixed. This was caught before use and
fully reverted (`git checkout -- tasks/g1_pick_place/run_pick_place.py`;
verified via empty `git diff` before proceeding) because it would have
produced episodes whose RGB frames show the blue pad in one place while
the actual/true placement target silently differs elsewhere — a
physically dishonest mismatch between visual ground truth and the real
task goal. Moving the rendered pad itself would be a Task 1 **geometry**
change, which this same authorization explicitly prohibits. Given this
direct conflict between two instructions in one authorization, geometry
was left unchanged and target position is fixed at `(0.0, 0.0)` for all 32
episodes; only cube position is varied. A version-2 collection spec
authorizing a scene-geometry change (moving the rendered pad together with
the controller target) is proposed here for later authorization, not
attempted in this phase.

The pilot did explore 4 target-offset probes under the since-reverted
parameterization (`logs/phase5e_pilot_trials.json`, P8/P9/P11/P12) before
this conflict was identified; they are retained in the pilot log purely as
the evidence that led to this exclusion decision (P11: `target_xy_offset =
(0.04, 0.0)` fails at `SETTLE_LOWER` despite passing IK reachability — a
genuine physical-only failure mode), not as calibration data for anything
in the locked spec.

## Section B — pilot (kept separate from the final dataset)

12 full-physics pilot trials plus two IK-only reachability grids (`logs/phase5e_pilot_ik_grid.json`,
`logs/phase5e_pilot_trials.json`), used only to calibrate the cube-position
sampling envelope before locking `data/task1_collection_spec.json`. Key
findings (full detail in the spec's own `pilot.key_findings`):

- Position-only IK reachability (`diagnose_pick_place_reachability` →
  `all_reachable`) correctly predicts the known `x_plus_0.03`
  grasp-unreachability failure and two new pilot probes (`cube_dx=+0.005`,
  `cube_dy=+0.045`) — used as a genuine pre-run reject filter, not just a
  descriptive signal.
- `cube_dx ∈ [-0.035,-0.005]`, `cube_dy ∈ [-0.01, 0.035]` is IK-reachable
  at all 4 corners + center AND physically succeeds in full trials (5/5).
- `cube_dx=0.0` is IK-reachable only for `cube_dy ≥ 0.0` — an asymmetric
  boundary discovered empirically (not assumed), which is why the locked
  envelope stops at `cube_dx=-0.005`, not `0.0`.

## Section C — locked sampling distribution

Full machine-readable spec: [`data/task1_collection_spec.json`](../data/task1_collection_spec.json)
(SHA-256 of the file as collected against: `3e007e1fab0fca8ff5916ab42803470396f753b31e0fb89430564a5b56807d1d`).
Locked BEFORE the final run; not edited after seeing results (this
report's Section G honestly records where the actual yield differs from
the intended 24/8 split — the spec file itself was not touched to make
the numbers match).

| Axis | Distribution |
| --- | --- |
| cube_dx | uniform continuous, band depends on split (see Section D) |
| cube_dy | uniform continuous, [-0.010, 0.035] m, all splits |
| cube yaw | fixed at 0 (not enabled — no pilot evidence yet the shared controller grasps under yaw) |
| target xy | fixed at (0,0) (not enabled — see Section A) |
| initial arm pose | fixed (not enabled — no pilot evidence perturbing it is safe) |
| instruction | 3 deterministic templates, selected by the episode's own seeded RNG draw |
| seed | 32 fixed integers (1000-1015 train, 2000-2003 val, 3000-3003 test, 9000-9007 diagnostics) |

Reachability pre-filter: `diagnose_pick_place_reachability(...)['all_reachable']`
(position-only IK residual < 8mm at all 7 chained waypoints), applied
**before** any physics trial for every one of the 32 configs. A config
that fails costs 0 simulated seconds — confirmed in
`logs/phase5e_collection_summary.json` (`wall_s: 0.0, sim_time_s: 0.0` for
all 3 rejected configs).

## Section D — split design

Assigned by seed and disjoint `cube_dx` band, fixed before collection:

| Split | cube_dx band (m) | seeds | intended count |
| --- | --- | --- | --- |
| train | [-0.035, -0.020) | 1000-1015 | 16 |
| val | [-0.020, -0.013) | 2000-2003 | 4 |
| test | [-0.013, -0.005] | 3000-3003 | 4 |
| diagnostics | N/A (8 fixed hand-specified probes) | 9000-9007 | 8 |

**Out-of-distribution axis**: val's and test's `cube_dx` bands are both
disjoint from train's — no train seed's `cube_dx` falls in the val/test
range and vice versa. Verified against the actual sampled values
(`logs/phase5e_validation.json`'s `spatial_coverage_by_split`): train
`cube_dx ∈ [-0.03499, -0.02308]`, val `∈ [-0.01794, -0.01378]`, test
`∈ [-0.0125, -0.00566]` — each range strictly more positive than the
previous, with margin, confirming no overlap. `cube_dy` and
instruction template are drawn from the same full ranges/set for all three
splits — no OOD claim is made along those axes. `train_eligible` is `True`
only for train/val/test episodes that also succeeded physically; **all 8
diagnostics-split episodes have `train_eligible=False` regardless of their
individual physical outcome** (verified directly against the HDF5 — see
Section F).

## Section E — dataset schema

Identical to Phase 5D's v3 schema (`policy/observations` 10Hz,
`policy/actions/{tcp_delta_position,tcp_delta_orientation,gripper_command,next_arm_joint_target,state_machine_phase}`
chunked H=5/50Hz, `execution/` 500Hz, `privileged/` separate from the
declared policy-input group) — reused via
`tasks.g1_pick_place.record_demonstrations_v3._derive_policy_actions`
UNMODIFIED, since that function derives every v3 action field purely from
`execution/arm_joint_target` via forward kinematics, independent of
cube/target position. New Phase 5E-only per-episode metadata: `seed`,
`split`, `instruction_canonical`/`instruction_utterance`/`instruction_template_id`,
`probe_kind`, `reachability_all_reachable`/`reachability_residuals_json`,
`collection_spec_hash`. Episode groups are named `seed_<seed>` (e.g.
`episodes/seed_1000`) rather than by variant name. Full field reference:
[`data/schema_v3.md`](../data/schema_v3.md) (unchanged — only new root/episode
attrs are additive, documented above and in `data/task1_collection_spec.json`).

## Section F — quality gates, measured per episode

All 29 collected episodes (of 32 attempted; 3 rejected pre-physics have no
episode group) were checked against every Section F gate:

| Gate | Result |
| --- | --- |
| Objective Task 1 success (for the 24 train/val/test episodes) | 24/24 pass |
| Onboard RGB valid at every policy observation | 29/29 pass (0 blank/near-blank frames, `logs/phase5e_validation.json`) |
| Action bounds valid / no NaN/Inf | 29/29 pass |
| Timestamp alignment valid | 29/29 pass (`len(observations) == len(actions)+1` structural invariant, asserted in the collector) |
| Exact execution replay passes (≤1mm TCP) | 29/29 pass — max observed 3.31e-05m, mean 1.25e-06m, all ≥4 orders of magnitude inside the 1e-3m target |
| Policy-action replay ≤10mm TCP (successful episodes) | **22/29 pass, 7/29 exceed 10mm** — see honest disclosure below |
| Correct manifest/decoder/spec hashes | 29/29 pass (`canonical_manifest_sha256`, `decoder_configuration_hash`, `collection_spec_hash` all verified per-episode against the live values) |
| Failure episodes: exact failure stage labeled, pre-failure data retained, `training_eligible=false`, no fake success padding | 1/1 (seed_9006, `SETTLE_APPROACH`, 31 transitions retained, `train_eligible=False`) |

**Honest disclosure — the ≤10mm policy-replay target does not hold
uniformly across the full continuous cube-position envelope.** Phase 5D
validated H=5 chunking against exactly 3 configs (nominal, x_minus_0.03,
x_plus_0.03-direction) and met the target on both successful ones
(8.09mm, 5.99mm). Scaling to 29 episodes spanning a continuous
`cube_dx × cube_dy` box reveals **7 episodes (24%) exceed 10mm** (max
observed 22.3mm, seed_3000; full list: seed_1000, 1003, 1006, 1007, 1009,
3000, 9007 — `logs/phase5e_validation.json`'s `replay_fidelity.per_episode`).
Aggregate: min 5.95mm, median 6.58mm, mean 9.20mm, max 22.3mm. This is
reported as measured, not adjusted to pass — per this project's standing
rule to diagnose rather than force a pass, and per this phase's explicit
instruction to use the Phase 5D decoder **unchanged** (no redesign
attempted here). Likely mechanism (consistent with Phase 5D's own
diagnosis): at some cube positions the RESET→PREGRASP initial commanded
jump is large enough that even a 20ms/10-physics-step sub-chunk (H=5)
cannot track it within 10mm — the action-magnitude distribution (Section
G) shows `tcp_delta_position` sub-action magnitudes up to 0.149m, with 82
of 18,635 sub-actions (0.44%) exceeding a 5cm/sub-chunk sanity threshold.
A future phase could investigate a larger H, or a position-dependent
initial-jump correction, but neither is attempted here (out of scope,
decoder held unchanged per authorization).

## Section G — dataset-level validation

Full output: `logs/phase5e_validation.json`. **0 structural errors, `PASSED: true`.**

- **Counts**: 32 attempted, 28 successes, 1 failed_interaction, 3
  rejected_by_reachability (4 diagnostic total) — see Section I for the
  honest deviation from the intended 24/8 split.
- **No duplicate configuration hashes** (checked over all 32 attempted
  configs' `(cube_xy_offset, target_xy_offset)` pairs).
- **Split counts**: train 16, val 4, test 4, diagnostics 8 — exactly as
  locked.
- **No seed reused**: 32/32 unique.
- **No configuration leakage across splits**: `cube_dx` bands verified
  disjoint both in the locked spec and in the actual sampled values
  (Section D table above).
- **Instruction distribution**: "Place the red block on the blue pad." ×9,
  "Move the red cube to the blue target." ×13, "Pick up the red cube and
  place it in the blue target area." ×10 — all 3 templates represented,
  reasonably balanced (deterministic per-seed draw, not hand-tuned for
  balance).
- **Spatial coverage**: see Section D table (per-split cube_dx/dy min/max).
- **Action distribution and saturation**: `tcp_delta_position` magnitude
  mean 1.17mm, p99 5.25mm, max 149mm (one extreme outlier sub-action, see
  Section F); `tcp_delta_orientation` magnitude mean 0.005rad, p99
  0.024rad, max 0.69rad. 82/18,635 sub-actions (0.44%) exceed a 5cm
  saturation sanity threshold.
- **RGB statistics**: mean pixel value 134.2, mean std 56.3, 0 blank/near-blank
  frames across all 29 episodes.
- **Episode duration distribution**: min 31 transitions (the one
  interaction failure), max 132, mean 128.5, median 132.
- **Success rate over all attempted configurations**: 28/32 = **87.5%**;
  excluding pre-physics reachability rejections (which cost 0 sim time and
  arguably shouldn't count against a "physical" success rate): 28/29 =
  **96.6%**.
- **Replay fidelity distributions** (not just one episode's max): exact
  execution — min 3.39e-08m, max 3.31e-05m, mean 1.25e-06m, all 29 within
  the 1mm target; policy-action — min 5.95mm, max 22.3mm, mean 9.20mm,
  median 6.58mm, 22/29 within the 10mm target (see Section F disclosure).

Plots (`artifacts/phase5e_dataset_summary/`):
`spatial_by_split.png`, `success_failure_locations.png`,
`episode_lengths.png`, `action_magnitude.png`, `replay_tcp_error.png`.

## Section H — files

| File | Size | SHA-256 |
| --- | --- | --- |
| `data/task1_demonstrations_v1.hdf5` | 62,196,309 bytes (59.3 MiB) | `accfe4461e7decc0dac2f7b959496487ba94747a504c534e149b8593c7749f21` |
| `data/task1_collection_spec.json` | 25,773 bytes | `3e007e1fab0fca8ff5916ab42803470396f753b31e0fb89430564a5b56807d1d` |

`data/task1_demonstrations_v1.hdf5` is **left untracked** (exceeds the
20MB commit-size threshold by ~3x) — `*.hdf5` is already gitignored
project-wide and no exception was added for this file (unlike
`task1_prototype*.hdf5`, which were small enough to commit directly). Only
its checksum/size/regeneration command are recorded here and in
`logs/phase5e_collection_summary.json`. A small representative sample
(one PNG per split, via `replay_dataset_episode.py visualize`) is kept in
`artifacts/phase5e_dataset_summary/`.

Regenerate: `python3 -m tasks.g1_pick_place.collect_dataset` (deterministic
— every sampled value is derived from its locked integer seed via
`numpy.random.default_rng(seed)`, no ambient RNG state).

**Collection timing**: wall time 322.2s total collection run (297.2s spent
inside actual physics trials, the remainder is model/scene loading and
HDF5 writes); 372.7s of simulated physics time across 29 episodes.
Validation (dataset-level checks + full replay of all 29 episodes in both
modes + 5 plots): ~100s. Pilot: ~40s (12 trials) + <1s (2 IK-only grids,
144+45 combinations, no physics stepping). Approximate total hands-on time
for this phase (pilot design, the target-position deviation
investigation/revert, collector/validator/replay tooling, tests, docs):
~3 hours.

**Package/version manifest**: Python 3.12.14, mujoco 3.3.6, h5py 3.16.0,
numpy 2.5.2, matplotlib 3.11.1 (**newly installed this phase** — not used
by any prior phase; installed into the project's existing `.venv` via
`pip install matplotlib`, no `requirements.txt` exists in this project to
update, versions recorded here instead).

## Section I — attempt and stop policy: actual outcome vs. intended split

**Exactly one pilot** (12 trials + 2 IK grids, kept separate, documented
in Section B), **exactly one locked final sampling plan**
(`data/task1_collection_spec.json`, all 32 seed/config pairs fixed before
the run), **exactly one collection run** over all 32 locked configs. No
seed was resampled or silently replaced after seeing results.

**The actual yield does not match the intended 24 success / 8
failed-or-rejected split, and this is reported honestly rather than
forced:**

- All 24 configs sampled from the calibrated success envelope (train/val/test)
  succeeded — **100% hit rate** on the primary training-eligible pool,
  exactly 24/24 as intended.
- Of the 8 diagnostic probes (designed with genuinely uncertain outcomes,
  per their own `probe_kind` labels), only 4 turned out to be
  failures/rejections (3 `reachability_reject_probe` configs correctly
  rejected pre-physics, 1 `physical_failure_probe` config — cube_dy=0.04 at
  dx=0.0 — failed at `SETTLE_APPROACH`). The other 4 `physical_failure_probe`/
  `reachability_boundary_probe` configs, whose outcomes were explicitly
  NOT known in advance (see `data/task1_collection_spec.json`'s pilot
  section — these values were deliberately chosen distinct from anything
  pilot-tested), turned out to succeed physically.

**Actual totals: 32 attempted, 28 successes, 4 diagnostic (1 failed
interaction + 3 rejected by reachability).** No episode was discarded to
force either number back to 24/8 — all 28 successes (including the 4
unplanned ones from the diagnostic pool) are retained in the dataset, and
all 4 diagnostic-split episodes (successes and failure alike) keep
`train_eligible=False` by the pre-registered split rule (Section D),
consistent with the authorization's instruction to keep the diagnostics
split out of behavior-cloning training regardless of individual outcome.
This means the dataset's **behavior-cloning-eligible pool is exactly 24
episodes as intended** (16/4/4 train/val/test), while the **diagnostic
pool yielded fewer genuine failures (4) than intended (8)** because the
boundary-probing configs were more often physically robust than the pilot
anticipated.

**Proposed for later authorization (not attempted here)**: a version-2
collection spec with diagnostic probes pushed further past the confirmed-
safe boundary (e.g. `cube_dy > 0.04`, finer steps between the confirmed
reject at `cube_dx=+0.005` and the confirmed-pass envelope, or — pending
separate authorization — a scene-geometry change enabling genuine
target-position variation per Section A's disclosure) if exactly 8
diagnostic failures/rejections are wanted rather than 4.

## Section J — verification and commit

Full test suite: **288 tests, 0 unexpected failures** (256 pre-existing
unchanged + 32 new, `tests/test_phase5e_scaled_collection.py`). Task 1 physics/controller/
camera confirmed unchanged: `git diff --stat tasks/g1_pick_place/run_pick_place.py`
is empty (the target-offset parameterization investigated in Section A was
fully reverted before any other work proceeded), and
`data/task1_canonical_config.json`'s `canonical_manifest_sha256`
(`f7375efc18d00fd83b2c75228bac76a7c23922913e104d6970e9b8241b9c290b`) is
byte-identical to Phase 5B/5C/5D. `logs/g1_mujoco_smoke.json` (pre-existing
dirty file, unrelated to any phase) and `vendor/unitree_mujoco` (pre-existing
submodule pointer state) are excluded from this phase's commit, consistent
with every prior phase.

## Compliance checklist

- [x] `data/task1_prototype.hdf5`, `data/task1_prototype_v2.hdf5`,
      `data/task1_prototype_v3.hdf5`, their reports/schemas, and commits
      `67ccf89`/`965b947`/`621a63a` untouched (verified: empty `git diff`
      against the pre-Phase-5E commit for every prior-phase file)
- [x] Phase 5D v3 action schema/decoder reused UNCHANGED (verified:
      `decoder_configuration_hash` identical to Phase 5D's, `_derive_policy_actions`
      imported not reimplemented)
- [x] Task 1 controller/gains/geometry/success-thresholds/camera constants
      unchanged (verified: empty `git diff` on `run_pick_place.py`,
      `controller_3c.py`, `gripper_scene.py`, `camera_observation.py`)
- [x] Canonical manifest hash identical to Phase 5B/5C/5D
- [x] Vendor submodule untouched
- [x] Cube state written only once, pre-lock, through `CubeInitGuard`
- [x] Every attempted configuration recorded, including rejections and
      failures — none silently discarded
- [x] Sampling distribution locked before the run, not edited after
      seeing results (deviation disclosure in Section A predates the
      collection run; the numeric envelope in Section C was not touched
      after seeing Section G/I's results)
- [x] No resampling to force the intended 24/8 split — actual yield
      reported honestly (Section I)
- [x] HDF5 not committed (exceeds size threshold); checksums/scripts
      tracked instead
- [x] No push
- [x] Stopped after dataset validation and this report — no Task 2, no
      further scaling
