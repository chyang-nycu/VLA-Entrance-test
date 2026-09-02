# Phase 3 Fixed-Base Grasping Baseline

Date: 2026-09-02

## Scope

- Repository: `vendor/unitree_mujoco` (pinned, unchanged), task-local additions under `tasks/g1_pick_place/`
- Required vendor commit: `4134cb5dc7ff1ba7f484deda48b5274b58694519`
- Robot: Unitree G1, official 29-actuator body/arm model (`g1_29dof.xml`), fixed pelvis via a model-level equality weld
- Task-local scene generator: `tasks/g1_pick_place/gripper_scene.py` -> `tasks/g1_pick_place/g1_grasp_scene.xml`
- Controller: `tasks/g1_pick_place/controller.py` (DLS Cartesian IK + bounded joint-space PD)
- Trial runner: `tasks/g1_pick_place/run_grasp_test.py`
- Logs: `logs/phase3_grasp_trials.json`

**Result: the nominal grasp-and-lift trial does not pass after 3 documented tuning iterations. Per the tuning budget in `HANDOFF.md`, work stopped here rather than attempting a 4th iteration or introducing a scripted grasp constraint. The 5-position-variant evaluation was not run, since it is gated on nominal success.**

## Gripper design

Task-local, symmetric parallel-jaw gripper attached under `right_wrist_yaw_link` (never modifying the vendor model):

- Two finger bodies (`left_finger`, `right_finger`), each a child of `right_wrist_yaw_link`, offset `0.10 m` along the wrist's local X axis, at `y = +/-0.075 m` (open reference).
- Each finger has one `slide` joint along local Y (`left_finger_joint`, `right_finger_joint`), driven by a dedicated `motor` actuator, `ctrlrange = [-15, 15] N`.
- Finger pad geoms: boxes, half-extents `(0.012, 0.006, 0.022) m`, friction `1.2 0.01 0.001`, `contype=conaffinity=1`, mass `0.03 kg` each.
- Nominal pad-to-pad contact point at `y = +/-0.041 m` (cube half-extent `0.035 m` + pad half-thickness `0.006 m`). Joint ranges extend `0.015 m` past that nominal contact point (`FINGER_SQUEEZE_MARGIN`) so a "closed" command keeps generating real contact force instead of stopping exactly at first touch (added in tuning iteration 3, see below).
- `grasp_tcp` site on `right_wrist_yaw_link` at local `(0.10, 0, 0)`, i.e. the same X offset as the fingers and centered in Y between them.
- Cube: box, half-extent `0.035 m`, mass `0.05 kg`, friction `1 0.01 0.001` (same convention as the Phase 2 contact probe), attached with a `freejoint` — never welded, attached, teleported, or mocap-driven.
- Table: static box, top surface at `z = 0.70 m`, footprint `0.44 x 0.44 m`, fixed at the nominal cube's (x, y).
- Pelvis: `<equality><weld body1="pelvis" .../></equality>` welds it to world at its initial pose; the floating base joint itself is untouched (a model-level constraint, not a runtime hack). Verified: after 200 steps of zero control, pelvis position changes by <2e-6 m.
- Contact excludes: `right_wrist_yaw_link`/`left_finger` and `right_wrist_yaw_link`/`right_finger`, so the rigid mounting frame does not register spurious self-contact.

Workspace placement was chosen from real forward-kinematics sampling of the vendor model (not assumed): with the right arm at `shoulder_pitch=-0.55, shoulder_roll=-0.08, elbow=1.5, wrist_pitch=-0.7`, the wrist reaches `[0.231, -0.155, 0.804]`, confirming the chosen cube position `[0.33, -0.15, 0.735]` sits inside the reachable workspace.

## Controller design

`tasks/g1_pick_place/controller.py`:

- Explicit `JointMap` (per joint: qpos address, dof/qvel address, actuator id, joint range, actuator ctrlrange), built once from joint/actuator names — no positional-ordering assumptions.
- `solve_dls_ik`: damped least-squares Cartesian IK on a caller-owned scratch `MjData` (never touches the live simulation state). Position-only task Jacobian (`mj_jacSite`), damping `0.05`, per-iteration step clipped to `0.05 rad`, up to 80 iterations. Verified to converge to <0.1 mm on realistic pregrasp/grasp waypoints near the cube, and to a few mm on the actual grasp point under real dynamics.
- `bounded_pd_step`: joint-space PD with gravity/Coriolis feedforward (`data.qfrc_bias`), an explicit per-call position-step bound, a velocity safety clamp that zeroes torque if a joint is already over-speed in the commanded direction, and a final clip to each actuator's `ctrlrange`. Raises `FloatingPointError` if any output is non-finite (actively checked, not just hoped for).

## Tuning iterations (nominal trial)

**Iteration 1** — `ARM_KP=60, ARM_KD=6`, no gravity/Coriolis feedforward, `ARM_MAX_STEP=0.02 rad`.
Result: total failure. TCP tracking error after a full 1 s "pregrasp" phase was **0.13 m** (target `z=0.835`, achieved `z≈0.743`); the arm never got close enough to the cube for the fingers to contact it. `both_pads_contact_cube=False`, `height_gain=0.008 m`.
Diagnosis: isolated single-joint step-response test showed the same gains take ~2 s to settle a 0.3 rad step — far slower than the 1 s phase budget, so a full 7-joint reach never caught up to its continuously-recomputed IK target.

**Iteration 2** — added gravity/Coriolis feedforward (`+ data.qfrc_bias[dof]`) to `bounded_pd_step`, raised gains to `ARM_KP=180, ARM_KD=18`, `ARM_MAX_STEP=0.03 rad`.
Result: partial improvement. Single-joint 0.3 rad step now settles in ~0.9 s. `both_pads_contact_cube` became `True` at least once during the sequence, and the cube was momentarily disturbed, but `height_gain=0.013 m` and `max_continuous_lifted_s=0.0` — no real lift.
Diagnosis: closer inspection showed the "closed" finger target was set to land exactly at the nominal pad-cube contact point (zero overlap), so any few-mm arm/IK misalignment left near-zero real squeeze force — the gripper touched the cube but did not grip it.

**Iteration 3** — added `FINGER_SQUEEZE_MARGIN=0.015 m` overtravel on the finger joint ranges and closed-target (so the commanded closed position presses 1.5 cm past the nominal contact point, generating real contact force regardless of small misalignment), and removed a start-position discontinuity in the trial sequencer (each phase now starts from the arm's actual current TCP position rather than an assumed prior waypoint).
Result: still fails. `both_pads_contact_cube=True`, but `height_gain=0.005 m`, `max_continuous_lifted_s=0.0`. Direct step-by-step tracing (see below) shows the cube gets nudged out of position (`[0.33,-0.15,0.735] -> [0.344,-0.129,0.735]`) **during the pregrasp/approach phases**, before the gripper ever closes, and during the close phase only the left finger pad ever registers contact (`R-contact=0` throughout) — the gripper closes on empty space next to a cube that has already been shoved aside.

## Root cause (not fixed within the 3-iteration budget)

Tracing TCP position through pregrasp/approach in iteration 3 shows several centimeters of oscillation rather than smooth convergence, e.g. during "pregrasp": `tcp_z` goes `0.888 -> 0.855 -> 0.735 -> 0.736 -> 0.754 -> 0.787` over 500 steps instead of monotonically approaching `0.835`. This oscillation sweeps the arm/gripper through the cube's actual resting location before the trajectory settles, physically shoving the cube off its nominal spot.

Hypothesis: a single `(Kp, Kd)` pair (`180, 18`) is applied uniformly across all 7 right-arm joints, but their actuator torque limits differ by 5x (shoulder/elbow: +/-25 N*m; wrist pitch/yaw: +/-5 N*m) and their effective inertia differs by roughly the same order. The same gain pair is well-matched for the large-inertia shoulder joints but likely causes the low-torque wrist joints to saturate constantly while chasing a continuously-recomputed multi-joint IK target, producing a genuinely underdamped, oscillatory closed-loop response for the coupled system — even though each joint individually (tested with a fixed single-joint target, no coupling, no continuously-moving reference) settles smoothly. The most likely fix for a future session is per-joint gain scaling (e.g. proportional to `ctrlrange` or an estimated effective inertia) rather than one global `(Kp, Kd)` pair, and/or reducing how fast the IK waypoint interpolation moves during pregrasp/approach so the coupled system has time to settle before the fingers are asked to close.

## Nominal trial result

| Criterion | Required | Measured | Pass |
| --- | --- | --- | --- |
| Both finger pads contact cube | yes | contact detected (asymmetric: left only for most of the window) | yes |
| Cube height gain | >= 0.08 m | 0.0051 m | **no** |
| Continuous lift/off-table duration | >= 2.0 s | 0.0 s | **no** |
| Controller outputs finite and bounded | yes | yes (checked every control step) | yes |
| Cube released after opening | yes | yes (trivially — it was never really held) | yes |

Overall: **FAIL** (2 of 5 criteria not met; the "pass" criteria that do hold are not meaningful on their own since no real grasp occurred).

Full trial telemetry (100-step-interval cube height/contact samples across the whole ~8.5 s sequence): `logs/phase3_grasp_trials.json`.

## 5-position-variant evaluation

Not run. `run_grasp_test.py` gates the variant sweep on nominal success, per the "no overfitting to the test set" instruction, and correctly exited with `variants: []`, `variant_success_rate: null` when the nominal trial failed.

## Automated tests

```bash
.venv/bin/python -m unittest tests/test_phase3_gripper.py -v      # 7 tests, PASS
.venv/bin/python -m unittest tests/test_phase3_controller.py -v   # 7 tests, PASS
.venv/bin/python -m unittest tests/test_phase3_grasp.py -v        # 6 tests, 3 FAIL (honest — asserts the real acceptance criteria)
```

`test_phase3_gripper.py` and `test_phase3_controller.py` verify structure and controller correctness independent of the tuning outcome, and both pass. `test_phase3_grasp.py` asserts the actual HANDOFF.md acceptance criteria end to end and currently fails on the height-gain, continuous-lift, and overall-pass checks — this is intentional: it was not weakened to hide the current tuning failure.

## Blockers before Task 2

- Fix the arm tracking oscillation (per-joint gain tuning or slower/better-damped trajectory interpolation) so the gripper reaches and stays at the intended grasp pose without disturbing the cube beforehand.
- Re-verify finger squeeze margin and force limits once tracking is stable — the current `FINGER_SQUEEZE_MARGIN`/`FINGER_FORCE_LIMIT` values were never validated under a stable approach, since the arm itself failed to deliver one.
- Only after the nominal trial passes: run the 5 deterministic position variants and report the honest success rate.
- Do not proceed to full pick-and-place transport, cameras, dataset collection, language variants, or policy integration until the fixed-base single-cube grasp-and-lift baseline passes.
