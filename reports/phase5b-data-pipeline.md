# Phase 5B — Task 1 VLA Data Pipeline Prototype

Date: 2026-09-03

Builds the first VLA (vision-language-action) demonstration dataset for
Task 1, using the exact pipeline every prior phase already verified — no
retuning of Task 1's controller, gains, or success thresholds. Full field
reference: [`data/schema.md`](../data/schema.md). Canonical manifest:
[`data/task1_canonical_config.json`](../data/task1_canonical_config.json).

## Canonical manifest first

Per the phase authorization, `data/task1_canonical_config.json` was
authored and stamped with its own content SHA-256
(`tasks/g1_pick_place/canonical_config.py::write_manifest_hash`) **before**
any collector/validator/replay code was written. It identifies, with
values read directly from the live code (not asserted from memory):

- scene: `write_grasp_scene_5a` (re-parses `write_grasp_scene_4b`'s own
  output — Phase 4E-lineage visual/collision-corrected gripper, vendor
  decorative hand omitted; `write_grasp_scene_4b` itself is never modified)
- controller: `solve_ik_waypoint` (non-oriented), `use_oriented_ik=false`
- arm gains 400.0/25.0, gripper gains 320.0/20.0
- success thresholds (verbatim from `run_pick_place.py`'s own constants —
  cross-checked in `tests/test_phase5b_dataset.py::TestTask1CriteriaUnchanged`)
- camera: `head_cam`, parent body `torso_link` (**verified by
  `xml.etree.ElementTree` parse of the generated scene**, not asserted),
  local pos/quat/fovy, and the true rendered resolution
- task instruction and the three collected variants

**Terminology**: the manifest and this report describe `head_cam` as a
"torso-mounted onboard RGB camera positioned near head height" — it is
parented to `torso_link`; the vendor G1 model has no separate head/neck
body.

**Camera resolution discrepancy resolved**: the manifest records
`resolution_wh: [160, 120]`, confirmed as the *true rendered array shape*
(`mujoco.Renderer(model, height=120, width=160)`, `frame.shape ==
(120, 160, 3)`, checked directly in this phase — see the pre-collection
baseline check below). The Phase 5A evidence video reports 160x128 via
`ffprobe`; that is H.264 macroblock-alignment padding introduced by the
video encoder, not the observation resolution. The HDF5 dataset stores raw
160x120 frames.

The collector, validator, and replay tool all import
`tasks/g1_pick_place/canonical_config.py` and call
`verify_environment_matches_manifest()` (collector, fail-loud at startup)
or check the dataset's own stored `canonical_manifest_sha256` against the
live manifest (validator, replay — fail-loud on mismatch). All three were
exercised against a deliberately tampered manifest/dataset hash in
`tests/test_phase5b_dataset.py` and in a manual mismatch test (see
Verification below) and confirmed to raise.

## Pre-collection canonical-baseline trial

Run once, before any episode collection, via
`tasks/g1_pick_place/phase5b_baseline_check.py`; evidence saved to
`logs/phase5b_baseline_check.json`.

| Check | Result |
| --- | --- |
| 1. Corrected Phase 4E visual gripper present | **PASS** |
| 2. Vendor decorative hand mesh absent | **PASS** |
| 3. Physical Task 1 completes (real state machine, nominal) | **PASS** (`task_pass=True`) |
| 4. Onboard RGB renders non-blank frames | **PASS** (shape `(120,160,3)`, uint8, std > 1.0) |
| 5. Live-computed config hash matches manifest | **PASS** |

All 5 checks passed on the first run; no fix was needed before collection.

## Deviation from the original plan, disclosed

The user's authorization named `y_plus_0.03` as the labeled
placement-failure episode, based on Phase 4B/4C history (grasp succeeds,
placement fails at a 20.4mm margin). **Re-measuring this under the current
canonical config** (which includes Phase 4E's gripper-gain increase
320/20 and LIFT/TRANSPORT/LOWER trajectory smoothing, both added after
Phase 4B) found `y_plus_0.03` now **passes** deterministically
(`task_pass=True`, final xy_err=2.07mm). A sweep of y/x offsets and
diagonal combinations (`0.02`–`0.05` m in each direction, see the
manifest's `instruction_variants` entries) found no remaining
placement-margin-only failure zone under the current config — every tested
offset either succeeds cleanly (xy_err in the low single-digit mm) or fails
at `SETTLE_APPROACH` (a grasp/reachability failure, before any grasp is
even attempted). The controller improvements made since Phase 4B evidently
closed the specific placement-margin gap `y_plus_0.03` used to expose.

**Episode 3 was collected as `x_plus_0.03` instead** (offset +0.03m in X) —
a genuine, deterministic, reproducible failure under the *current* config
(fails at `SETTLE_APPROACH`, "TCP did not settle within tolerance before
CLOSE"), previously documented in Phase 4A/4B history as IK-unreachable
(27.1mm residual, > 8mm tolerance). This is a grasp-approach failure, not a
placement-margin failure, and is documented as such — not mislabeled to
match the original plan. Both the stale `y_plus_0.03` measurement and the
substitution rationale are recorded in
`data/task1_canonical_config.json`'s `instruction_variants` object.

## Per-episode results

From `logs/phase5b_collection_summary.json` (regenerated by
`tasks/g1_pick_place/record_demonstrations.py`):

| variant_id | success | termination_reason | transitions | observations | physics steps | wall time (s) | render time (s) | final xy err |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `nominal` | True | `DONE` | 132 | 133 | 6622 | 9.05 | 3.75 | 1.72mm |
| `x_minus_0.03` | True | `DONE` | 132 | 133 | 6610 | 8.95 | 3.69 | 4.55mm |
| `x_plus_0.03` | **False** | `SETTLE_APPROACH` | 31 | 32 | 1551 | 2.17 | 0.91 | n/a (never reached placement) |

Collection speed: ~0.73s of wall time per second of simulated episode
(9.05s wall / ~13.2s simulated nominal episode at 500Hz physics), of which
~41% is rendering (3.75s of 9.05s) — consistent with Phase 5A's measured
~18.5fps combined sim+render throughput at this camera resolution.

`data/task1_prototype.hdf5`: **1,740,909 bytes (1.66 MiB)** — small enough
to commit directly (well under any repository size concern); committed
alongside the code in this phase's commit. SHA-256:
`d30250fac4fc0fb4dcd2bc9972dbc43a600afc73ef70fca386dd89b6c919454f`.
Regeneration command: `python3 -m tasks.g1_pick_place.record_demonstrations`
(deterministic — no RNG anywhere in this pipeline, confirmed by every prior
phase; regenerating reproduces the same file).

## Sample frames

`artifacts/phase5b_sample_frames/{variant_id}_{first,middle,final}.png` —
9 PNGs, one triplet per episode. Directly inspected: red cube and blue
target pixels both present in every sampled frame across all three
episodes (red/blue pixel-mask counts in the hundreds to low thousands per
160x120 frame; see `tests/test_phase5b_dataset.py::test_red_cube_and_blue_target_visible_in_first_frame`
for the same check made a hard test assertion). `nominal_middle.png` shows
the cube mid-transport in the gripper with the target pad visible below;
`x_plus_0.03_first/middle/final.png` show the arm reaching but not
completing the grasp approach, consistent with the recorded
`SETTLE_APPROACH` failure.

## Validator result

```
python3 -m tasks.g1_pick_place.validate_dataset
```

```json
{
  "errors": [],
  "advisories": []
}
VALIDATION PASSED
```

All required checks passed, including the two that recompute rather than
trust: (a) **independent success recomputation** — the validator reruns
`run_trial_pick_place` for each episode's stored `cube_xy_offset` under the
canonical config and compares the resulting `task_pass` to the dataset's
stored `success` attribute (all three agreed); (b) the failure episode's
`train_eligible=False` was confirmed consistent with its `success=False`.
`tests/test_phase5b_dataset.py::TestValidator` additionally exercises 5
deliberately-tampered copies (wrong success flag, wrongly-train-eligible
failure episode, missing group, non-monotonic timestamps, bad quaternion
norm) and confirms the validator catches each one.

## Replay deviation

```
python3 -m tasks.g1_pick_place.replay_demonstration replay --variant <id>
```

| variant_id | max joint error (rad) | max TCP error (m) | max cube error (m) | final cube error (m) | within tolerance |
| --- | --- | --- | --- | --- | --- |
| `nominal` | 0.1519 | 0.0487 | 0.00783 | 0.00232 | **True** |
| `x_minus_0.03` | 0.0561 | 0.0184 | 0.00655 | 0.00157 | **True** |
| `x_plus_0.03` | 0.0074 | 0.00088 | 2.5e-8 | 1.6e-8 | **True** |

Tolerances used (documented, not physics ground truth — generous relative
to workspace scale so a genuine replay bug would still be caught):
joint 0.35 rad, TCP 0.08m, cube 0.08m.

**The nonzero deviation on `nominal`/`x_minus_0.03` is expected, not a
bug**: recorded actions are a 10 Hz zero-order-hold downsample of the
original 500 Hz fine-grained waypoint-ramped control
(`run_pick_place._drive_smooth` changes its commanded joint target on
*every* physics step during LIFT/TRANSPORT/LOWER). Replaying a 10 Hz
action stream by holding each recorded target constant for the whole
50-step block cannot reproduce that fine ramp exactly. `x_plus_0.03`'s
near-zero deviation is consistent with this: that episode fails at
`SETTLE_APPROACH`, before any smooth multi-waypoint segment ever runs, so
there is no ramping to lose fidelity on.

Observation-only visualization replay
(`replay_demonstration.py visualize`) was also exercised for all three
episodes, producing a 6-frame contact sheet per episode
(`artifacts/phase5b_sample_frames/{variant_id}_visualize_contact_sheet.png`)
with zero physics stepped — confirmed by inspecting one directly: the
nominal contact sheet shows the visible progression RESET → LIFT_wp3 →
HOLD → TRANSPORT_ABOVE_TARGET_wp29 → SETTLE_LOWER → VERIFY_TASK_SUCCESS.

## Manifest-hash enforcement, exercised

A tampered copy of the dataset (root `canonical_manifest_sha256` attribute
overwritten with a bogus value) was passed to
`replay_demonstration.py replay`; it raised
`canonical_config.ManifestMismatchError` and refused to run, as designed.
`tests/test_phase5b_dataset.py::test_replay_rejects_manifest_hash_mismatch`
makes this a permanent regression test.

## Test suite

`tests/test_phase5b_dataset.py`: **36 new tests**, generating a small
temporary 2-episode HDF5 file (not depending on the full
`data/task1_prototype.hdf5` artifact being present), covering: canonical
manifest loading/hashing/mismatch-rejection (9 tests), the transition
convention invariant (4), HDF5 schema shape/dtype/content (9), the
validator including 5 tamper-detection cases (8), replay determinism and
manifest enforcement (4), and a direct cross-check that Phase 5B did not
retune Task 1's own success-threshold constants (2).

Full-suite regression: **214 tests, 0 unexpected failures** (178
pre-existing from Phase 5A + 36 new from this phase — exact match, no
other test file changed count).

## Approximate time

This phase (manifest authoring, collector/validator/replay implementation,
the y_plus_0.03 re-measurement and x_plus_0.03 substitution investigation,
36 new tests, full regression, and this documentation) took approximately
0.5–1 hour of wall-clock work.

## Compliance checklist

- [x] Vendor submodule untouched (`vendor/unitree_mujoco` pin unchanged; no
      file under `vendor/` modified)
- [x] No historical report/log/test/video from Phase 1 through Phase 5A
      altered (verified by `git diff` against the pre-Phase-5B commit)
- [x] No Task 1 success-threshold constant changed (verified by
      `tests/test_phase5b_dataset.py::TestTask1CriteriaUnchanged`, which
      cross-checks the manifest's copied values against `run_pick_place.py`'s
      live constants)
- [x] Cube state written only once, pre-lock, through `CubeInitGuard`, both
      during collection and during replay (replay reuses the identical
      guard/lock mechanism; no cube-state write appears after `guard.lock()`
      in either `record_demonstrations.py` or `replay_demonstration.py`)
- [x] Failure episode labeled `success=False` and `train_eligible=False`,
      confirmed excluded by the validator
- [x] No push
- [x] Stopped after exactly 3 episodes — no scaled collection, no Task 2
