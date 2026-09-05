# Unitree G1 Manipulation → VLA Demonstration Pipeline

## 1. Overview

A Unitree G1 manipulation prototype in MuJoCo, built up from a physical
grasping baseline to a VLA-oriented demonstration pipeline:

- **Task 1** — single-object pick-and-place (fixed-base, torso-constrained upper body).
- **Task 2** — object-conditioned two-object selection on the same scene.
- **VLA pipeline** — RGB + proprioception observations, recorded expert
  actions, HDF5 demonstrations, replay validation.
- Both tasks use a **scripted classical expert** — no learned policy or
  language grounding is claimed. Every grasp is produced by physical
  contact and actuation: no teleport, weld, or scripted state overwrite.

## 2. Results at a Glance

| Component | Result |
| --- | --- |
| Task 1 | Pick-and-place completed in the supported reachable envelope |
| Task 2 | 4 configs x 3 deterministic repeats, 12/12 pass |
| Wrong-object placements | 0 |
| Distractor displacement | <=1.73mm |
| Demonstration pipeline | RGB + proprioception + actions + replay |
| Policy-action replay | 8.09mm canonical max TCP error |
| Tests | 311, 0 unexpected failures |
| Learned policy | Not attempted |
| Isaac Lab | Not used (no macOS support) — MuJoCo implementation instead |

## 3. Demo

**Task 1 — pick-and-place** (fixed-base G1, physical gripper, no teleport):

![Task 1, third-person view](submission/videos/task1_third_person.gif)

Third-person, full episode (approach → grasp → lift → transport → lower →
release → retreat), 12.0s — the video reviewed and accepted in the
Phase 4F human acceptance decision above.

![Task 1, onboard camera](submission/videos/task1_onboard_rgb.gif)

Onboard `head_cam` view (torso-mounted, near head height), 13.3s — the
same camera stream used as a policy-facing observation in the VLA data
pipeline.

![Before/after: decorative-hand defect vs. corrected gripper](submission/videos/optional_debug_before_after.gif)

Side-by-side diagnostic: **left** — the vendor's decorative hand mesh
visibly clipping through the cube (Phase 4D); **right** — the corrected
gripper after the visual/collision fix (Phase 4E). 4.3s.

**Task 2 — object-conditioned selection** (same scene, different task
specification, different selected object; see `reports/task2-language-selection.md`):

![Task 2, red instructed](submission/videos/task2_red_instruction.gif)

"Pick up the **red** cube..." — red cube grasped and placed; green
distractor undisturbed throughout. 13.2s.

![Task 2, green instructed](submission/videos/task2_green_instruction.gif)

Same physical arrangement, "Pick up the **green** cube..." instead — green
cube grasped and placed; red distractor undisturbed throughout. 13.2s.

## 4. Key Research Findings

- **Controller redesign.** Torque-PD control with a continuous IK solver
failed: heterogeneous per-joint torque limits and coupled tracking
instability shoved the cube out of position before the gripper closed.
Switching to bounded position servos + waypoint IK produced deterministic
grasping (5/5).

- **Nearby-object interference.** An apparently safe 8cm distractor slot
still moved 48.7mm during Task 2, because the arm's RETREAT motion swept
through it. Moving the slot to the shipped position reduced disturbance to
<=1.73mm.

- **Action representation.** A naive low-frequency policy action stream
could not reproduce the expert's 500Hz control trace on replay (up to
~98mm error). Chunking actions into H=5 sub-deltas at an effective 50Hz
reduced canonical replay error to 8.09mm.

## 5. VLA Demonstration Pipeline

```text
RGB + proprioception
        |
        v
policy observations

scripted expert
        |
        v
TCP / gripper actions
        |
        v
HDF5 demonstrations
        |
        v
validation + replay
```

- Policy-facing observations and privileged evaluation metadata (e.g.
  `selected_object_id`) are kept separate.
- Observations are recorded at 10Hz.
- Actions are stored as H=5 sub-action chunks at an effective 50Hz.

Full schema and per-episode detail: `submission/DATASET_CARD.md`.

## 6. Limitations

- Fixed-base, torso-constrained G1 — not full-body or free-standing manipulation.
- MuJoCo, not Isaac Lab (no macOS support on this host).
- No learned policy or VLA model inference — scripted expert only.
- Task 1's stricter <=10mm grasp-slip engineering target is **not** met
  (measured ~25.9mm); task-level pick-and-place otherwise works.
- Task 2 uses privileged object selection (`selected_object_id` supplied
  directly), not natural-language parsing or visual-language grounding.

## 7. Reproduce

```bash
git submodule update --init --recursive
./setup/preflight_macos.sh
python3.12 -m venv .venv
.venv/bin/pip install mujoco==3.3.6 numpy imageio pillow h5py matplotlib
.venv/bin/python -m unittest discover -s tests
```

Full command list, exact versions, and recorded timings:
`submission/REPRODUCE.md`.

## 8. Repository Structure

```text
tasks/g1_pick_place/    core task scenes, gripper MJCF, controllers
submission/             final report, dataset card, videos
reports/                detailed per-phase experiment reports
tests/                  regression and diagnostic tests
vendor/unitree_mujoco/  pinned upstream simulator (git submodule, never modified)
setup/                  host bootstrap scripts
data/, logs/, artifacts/  datasets, raw evidence, and small evidence media
```

## 9. Detailed Documentation

- Full submission report: `submission/entrance_test_report.md`
  (see **Contributions and Role** for the design decisions and defect
  catches that drove each phase)
- Research slides: `slide.pdf`
- Dataset card: `submission/DATASET_CARD.md`
- Full chronological development history: `HANDOFF.md`
- Per-phase evidence and attempt logs: `reports/`
