# Phase 7B — Door Scene Construction

> **Evidence record — not reviewer reading.** Exhaustive per-phase audit
> trail, kept so every number in the submission is traceable. The
> reviewer-facing account is `README.md` and
> `submission/entrance_test_report.md`.

Date: 2026-09-05

## Scope

Build the door scene whose geometry Phase 7A licensed (`GO_HINGE`), and
gate it: scene loads; `g1_grasp_scene_4b.xml`/`_5a.xml`/`_task2.xml` are
byte-identical to before; the locked geometry is reachable, oriented, and
collision-free across its full range of motion.

Implementation: `tasks/g1_pick_place/door_open.py`. Scene:
`tasks/g1_pick_place/g1_grasp_scene_door.xml` (generated, untracked).
`gripper_scene.py`, `camera_observation.py`, and `task2_language_selection.py`
are not edited — the scene is built by re-parsing `write_grasp_scene_5a`'s
own output file, following `write_task2_scene`'s exact convention.

## Geometry selection — margin, not the maximal arc

Phase 7A's raw search (`search_largest_arc`) reported the single largest
admissible arc (90° swing, 19.8cm chord) as its top candidate — but by
construction, the largest arc that still clears an admissibility gate has
the least margin left inside it (its worst sampled point sits within
0.2–0.8° of the 7° orientation wall). `select_door_geometry()` re-scores
the same admissibility predicate for margin instead of raw chord length,
at a reduced 45–60° swing, and picked:

```
pivot_xy = [0.38, -0.16]   radius = 0.08m   phi0 = 120°   theta = 60°
chord = 8.0cm   handle_z = 0.90m
orientation margin: 4.0° (worst point: 3.0° of 7°)
sigma_min margin:   0.066 (worst point: 0.119, floor 0.053)
IK residual margin: 5.3mm (worst point: 2.7mm, tol 8mm)
```

## Finding — the well-conditioned region overlaps the arm's resting pose

The first candidate selected by margin alone (pivot `[0.2, -0.12]`) passed
every reachability/conditioning check, but building the actual scene and
running `mj_forward` at the plain `mj_resetData` pose (no control applied)
found **real, severe penetration**: `door_handle_geom` vs
`right_elbow_link` at −55mm, plus contacts against `right_shoulder_yaw_link`,
`right_wrist_roll_link`, `right_wrist_pitch_link`. Re-running the identical
check against Task 1's own scene confirmed this is not a pre-existing
condition — Task 1 has zero such contacts at reset (only the unrelated,
pre-existing ankle/table artifact documented in `phase7a-workspace-map.md`).

Cause: the region Phase 7A found well-oriented and well-conditioned when
*actively driven* sits close to where the arm's neutral, uncommanded
configuration already is (`right_wrist_yaw_link` rests at approximately
`(0.20, -0.15, 0.888)` — inside the same well-conditioned band). A door
placed there occupies space the idle arm already fills, which would cause
a large, unphysical contact-resolution transient at the very first physics
step of any trial, before the controller issues its first command.

Fix: `select_door_geometry()` now also requires the CLOSED handle position
to clear every resting-arm link's world position (measured directly, not
assumed) by at least 15cm (`REST_CLEARANCE_MIN_M`). Re-running the search
under both criteria selected the geometry above (`rest_clearance_m =
0.152`). A second, much smaller issue found the same way — the frame
post's footprint grazing the cube's corner by 0.04mm — was fixed by
reducing `FRAME_THICKNESS_M` from 15mm to 10mm.

## Gate results

| Check | Result |
| --- | --- |
| Scene loads | pass |
| `g1_grasp_scene_4b.xml` SHA-256 unchanged | pass (`afbeb007...`) |
| `g1_grasp_scene_5a.xml` SHA-256 unchanged | pass (`d07ad5d3...`) |
| `g1_grasp_scene_task2.xml` SHA-256 unchanged | pass (`a95ba543...`) |
| `door_hinge` is a real hinge joint, axis `0 0 1` | pass |
| `door_panel` has exactly 1 joint; `door_frame` has 0 | pass |
| No actuator drives `door_hinge` | pass (structural: door is passive) |
| `model.neq` unchanged (still 2: pelvis/torso welds) | pass |
| Contacts at reset, beyond the pre-existing ankle/table artifact | **0** |
| Contacts across a 13-point sweep of the full hinge range | **0** |
| `diagnose_door_reachability`: `all_reachable` | True |
| `diagnose_door_reachability`: `all_position_and_orientation_reachable` | **True**, every waypoint (PREGRASP/APPROACH/13 arc samples/RETREAT) |
| Clearance from Task 1's cube and target pad | pass (both retained in-scene, inert) |

Worst position residual across every waypoint: 7.89mm (of 8mm). Worst
orientation residual: 5.9mm — at `RETREAT`, still comfortably inside the
7° tolerance. The arc itself (13 samples) never exceeds 2.4°, roughly 20x
better than Task 1's cube-grasp orientation residual (47°).

One waypoint construction note: the first `diagnose_door_reachability`
draft placed `PREGRASP_HANDLE` 8cm straight above the closed handle
(mirroring Task 1's vertical `PREGRASP_DZ` approach), which failed
orientation (16.6° — straying into the height-sensitive band Phase 7A
measured). Changed to an in-plane standoff along the arc's own tangent (8°
before the closed angle, same radius, same height) — physically the
natural way to approach a handle the gripper will close around, and it
stays inside the same well-conditioned band the arc already occupies.

## Hinge calibration (attempt 1 of a 3-attempt budget)

- **Holds under gravity**: trivially satisfied — the hinge axis is
  vertical (`0 0 1`), so gravity exerts zero torque about it regardless of
  angle. Measured drift over 3s from 5 initial angles: 0.0 rad exactly.
- **Impulse response**: a 0.5 N·m / 0.2s torque pulse from closed drove
  the panel to the joint's range limit (60°) and it settled there
  (`qvel ≈ -1.3e-10 rad/s` after 1.5s) rather than free-swinging past it —
  the range limit and `frictionloss=0.08` together stop the door cleanly
  at full open.
- **Opening force budget**: required tangential force at the handle to
  overcome static friction, `F = frictionloss / radius = 0.08 / 0.08 =
  1.0N`. Task 1's finger force limit is 15N and Phase 4E's measured
  minimum bilateral grip force is 0.2N per pad at rest — this is not
  expected to bind, but is only confirmed once Phase 2 runs a real pull
  trial.

## What this phase does not establish

- The calibration above uses a directly applied `qfrc_applied` torque for
  measurement only, never inside `run_trial_door_open`'s own code path
  (which Phase 4's self-audit will forbid by regex, mirroring
  `run_pick_place.py`'s existing anti-cheat pattern).
- Frictionloss/damping are accepted on attempt 1; Phase 2's real pull
  trial is the first end-to-end test of whether the grip can actually
  move the door under real contact, not an applied idealized torque.
