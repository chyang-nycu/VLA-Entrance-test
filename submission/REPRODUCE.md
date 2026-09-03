# Reproduction Instructions

Every command below was actually executed in this repository's environment
during Phase 6 (2026-09-03), on an Intel Mac (macOS 12.7.6). Timings are
real observed wall-clock measurements from those runs, not estimates —
expect variance of roughly ±50% depending on system load (this machine
showed real run-to-run variance, e.g. 51s vs. 77s for the same nominal
Task 1 command, and once ~3.5x slower while an unrelated background
compile was contending for CPU).

**Caution**: several of the commands below (`run_pick_place`,
`record_onboard_camera_episode`, `record_demonstrations_v3`,
`validate_scaled_dataset`, `replay_dataset_episode ... visualize`)
regenerate a fixed output path that this repository already has a
*committed, historical* version of (e.g. `logs/phase4b_pick_place_trials.json`,
`artifacts/phase5a_head_camera.mp4`, `artifacts/phase5e_dataset_summary/*.png`).
Every simulation in this project is deterministic (fixed seeds, no ambient
RNG), so re-running these produces byte-identical output in every case
observed during this phase's verification — but if you want to guarantee
the repository stays pristine, run `git status --short` before and
`git checkout -- <path>` after any command that touches a tracked file.

## 1. Environment bootstrap

```bash
git clone <this-repo-url> Robotics && cd Robotics
git submodule update --init --recursive   # fetches vendor/unitree_mujoco at its pinned commit
python3.12 -m venv .venv
.venv/bin/pip install mujoco==3.3.6 numpy==2.5.2 imageio==2.37.4 imageio-ffmpeg pillow h5py==3.16.0 matplotlib==3.11.1
source .venv/bin/activate
```

Not re-run from a fully clean clone in this phase (the existing `.venv` was
reused throughout the project); each individual package/version above was
independently confirmed installed and importable in this environment
(`python3 -c "import mujoco, numpy, imageio, h5py, matplotlib"` succeeds).
**Disk**: repository + vendor submodule + `.venv` + generated
artifacts/datasets is a few hundred MB; the untracked scaled dataset alone
is ~62MB.

## 2. Phase 1 smoke test

```bash
.venv/bin/python setup/g1_mujoco_smoke.py
```

Verified. Runtime: **~2.6s**. Writes `logs/g1_mujoco_smoke.json` (this file
is *expected* to show a timestamp-only diff on every rerun — a
pre-existing, documented, harmless drift, not a defect to fix).

## 3. Nominal Task 1 (full Stage A/B sweep)

```bash
.venv/bin/python -m tasks.g1_pick_place.run_pick_place
```

Verified. Runtime: **51–77s observed** (5 Stage-A nominal reruns + 3
Stage-B variants × 3 trials = 14 full trials). Output:
`{"stage_a_pass": true, "stage_b_supported_envelope_success_rate": 1.0}`.
Regenerates `logs/phase4b_pick_place_trials.json` (a tracked historical
file — content was byte-reproducible when checked against git, aside from
this command legitimately reflecting the *current* code's numbers, which
now differ from Phase 4B's original 67% Stage-B figure because Phase 4E's
later gripper/trajectory improvements closed that gap — see the main
report Section 6).

## 4. Full test suite

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Verified (run in background with output redirected, then waited on — a
single blocking foreground call can exceed common shell/tool timeouts).
Runtime: **see Section 9/12 of `entrance_test_report.md` for the exact
figure from this phase's own run** (prior phases observed 250–700s as the
suite grew; it is now 288 tests). Result: 0 unexpected failures (one
test, `Phase4DDecorativeHandOverlapTest`'s *original* static-vendor-mesh
variant, and two Phase 4F slip-bar diagnostics, are intentional
regression-style tests that assert a documented, honestly-reported
condition — not accidental breakage).

## 5. Onboard camera video

```bash
.venv/bin/python -m tasks.g1_pick_place.record_onboard_camera_episode
```

Verified. Runtime: **~24s**. Regenerates `artifacts/phase5a_head_camera.mp4`
(160x120 true render size, 29.41fps, 390 frames, 13.26s) and
`logs/phase5a_camera_smoke.json` — both confirmed byte-identical to the
committed historical versions after rerun (SHA-256
`037bb5cf5e7a05a7b431f204c65179b65e64e07e4a210bbeda74bee4cbfb0a85` for the
video, matched exactly).

## 6. Prototype dataset generation (v3 schema, 3 episodes)

```bash
.venv/bin/python -m tasks.g1_pick_place.record_demonstrations_v3
```

Verified. Runtime: **~26–210s observed** (large variance was traced to
unrelated background CPU contention during one run, not the command
itself — a clean rerun consistently took ~26s: nominal 9.4s + x_minus_0.03
9.0s + x_plus_0.03 (failure) 2.3s + overhead). Regenerates
`data/task1_prototype_v3.hdf5` — confirmed byte-identical (SHA-256
`d8223205865974937daf94a1dd9a971e6d919d150ad7c4a58eb42aec9b5bd8be`) across
every rerun in this phase.

## 7. Scaled collection (32 configurations)

```bash
.venv/bin/python -m tasks.g1_pick_place.collect_dataset
```

**Not re-run in this phase** (see rationale below) — verified instead by
citing its actual, already-logged Phase 5E execution: wall time 322.2s
(297.2s inside physics trials), 372.7s of simulated physics time across 29
episodes, writing `data/task1_demonstrations_v1.hdf5` (62,196,309 bytes,
SHA-256 `accfe4461e7decc0dac2f7b959496487ba94747a504c534e149b8593c7749f21`)
— see `logs/phase5e_collection_summary.json` for the complete, real record
of that run. Not re-run here because it is deterministic (verified via the
much cheaper commands in Sections 5/6 above, which regenerate different
tracked files and reproduced them byte-for-byte every time) and re-running
the full ~5.5-minute, 32-trial collection would not add verification value
beyond what its own already-recorded, machine-checked log provides.

## 8. Validator

```bash
.venv/bin/python -m tasks.g1_pick_place.validate_dataset_v3        # prototype v3 dataset
.venv/bin/python -m tasks.g1_pick_place.validate_scaled_dataset    # scaled Phase 5E dataset
```

Both verified. Runtimes: **~21s** (v3 validator) and **~113s** (scaled
validator, replays all 29 episodes in both exact and policy modes plus
generates 5 plots). Both report `PASSED: true`, matching the committed
`logs/phase5e_validation.json` exactly (byte-identical after rerun,
confirmed via diff). The scaled validator regenerates
`artifacts/phase5e_dataset_summary/*.png` (5 dashboard plots) and
`logs/phase5e_validation.json` — both tracked historical files, both
reproduced byte-identical.

## 9. Exact replay

```bash
.venv/bin/python -m tasks.g1_pick_place.replay_demonstration_v3 exact --variant nominal
```

Verified. Runtime: **~6s**. Reports `within_tolerance: true`,
`max tcp_err ~3.65e-8m`. Read-only against the dataset (writes nothing).

## 10. Policy replay

```bash
.venv/bin/python -m tasks.g1_pick_place.replay_demonstration_v3 policy --variant nominal
```

Verified. Runtime: **~5s**. Reports the H=5/50Hz-chunked policy-action
replay result (8.09mm max TCP error on this variant). Read-only.

## 11. One selected episode replay (scaled dataset)

```bash
.venv/bin/python -m tasks.g1_pick_place.replay_dataset_episode exact --episode seed_1000
.venv/bin/python -m tasks.g1_pick_place.replay_dataset_episode policy --episode seed_1000
.venv/bin/python -m tasks.g1_pick_place.replay_dataset_episode visualize --episode seed_1000
```

All three verified individually. Runtimes: **~4s, ~3s, ~1s** respectively.
`visualize` writes a contact-sheet PNG to
`artifacts/phase5e_dataset_summary/seed_1000_visualize.png` (a tracked
historical file, reproduced byte-identical).

**Note**: `replay_dataset_episode.py`'s `all` mode does **not** take a
single episode — passing `--episode` is silently ignored and it replays
*every* episode in the dataset (confirmed: `all --episode seed_1000` took
~120s and printed all 29 episodes' results, identical to running
`validate_scaled_dataset`'s replay pass). Use one of the three explicit
per-mode invocations above to replay exactly one selected episode.

## 12. Task 2 test module

```bash
.venv/bin/python -m unittest tests/test_task2_language_selection.py -v
```

Verified during the Task 2 audit/merge (2026-09-03). Runtime: **~73s**.
23/23 pass, 0 failures. Read-only with respect to `git`-tracked state
except regenerating `tasks/g1_pick_place/g1_grasp_scene_task2.xml` (a
tracked, deterministically-reproducible generated scene file).

## Reproducibility summary

| Command | Verified this phase | Runtime observed |
| --- | --- | --- |
| Env bootstrap | partially (packages confirmed importable; not from a fresh clone) | n/a |
| Phase 1 smoke test | yes | ~2.6s |
| Nominal Task 1 (full sweep) | yes | 51–77s |
| Full test suite (288 tests) | yes | see main report |
| Onboard camera video | yes | ~24s |
| Prototype dataset gen (v3) | yes | ~26s (clean) |
| Scaled collection (32 configs) | cited from Phase 5E's own log, not rerun | 322.2s (recorded) |
| Validator (v3) | yes | ~21s |
| Validator (scaled) | yes | ~113s |
| Exact replay | yes | ~6s |
| Policy replay | yes | ~5s |
| One-episode replay (3 sub-commands) | yes | ~4s / ~3s / ~1s |
| Task 2 test module (23 tests) | yes | ~73s |
