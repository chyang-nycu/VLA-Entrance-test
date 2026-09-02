# Phase 3B Controller Stabilization Budget

Date: 2026-09-02

## Scope

Separate, additionally-budgeted attempt to fix the root cause identified in
`reports/phase3-grasping-baseline.md` (Phase 3, unchanged, historical
record). Nominal grasp trial only; the 5-position-variant sweep remains
gated on nominal success and was not run.

- Scene: `tasks/g1_pick_place/g1_grasp_scene.xml` (unchanged from Phase 3).
- Controller: `tasks/g1_pick_place/controller.py` (extended, not rewritten).
- Trial harness: `tasks/g1_pick_place/run_grasp_test.py` (extended with
  `CubeInitGuard` and controller/diagnostics injection).
- Tuning driver: `tasks/g1_pick_place/phase3b_tuning.py`.
- Instrumentation + all attempt metrics: `logs/phase3b_controller_tuning.json`.

**Result: all 3 Phase 3B attempts fail the nominal acceptance criteria. Per
the tuning budget, work stops here with the quantitative analysis below.**

## Initialization boundary, made explicit and enforced

HANDOFF.md's clarification (writing cube qpos/qvel is allowed only before a
trial's first `mj_step`, never after) is now enforced by
`CubeInitGuard` in `run_grasp_test.py`: `set_initial_pose`/
`set_initial_velocity` raise `RuntimeError` once `lock()` has been called
(called immediately after the trial's first `mj_step`). `run_trial`'s own
source is scanned (`_assert_run_trial_has_no_direct_cube_state_write`, run
at import time) to catch any regression that bypasses the guard.
`tests/test_phase3_grasp.py::CubeInitGuardTest` (4 tests) exercises both the
allowed pre-lock write and the rejected post-lock write/velocity-write, plus
the source-scan invariant. All 4 pass.

## Instrumentation and baseline evidence (required before Attempt 3B-1)

**Ctrl-to-torque mapping**, resolved (not assumed) via `model.actuator_gear`
and `model.actuator_trntype` for all 7 right-arm motors: gear is exactly
`[1, 0, 0, 0, 0, 0]` and transmission type is `mjTRN_JOINT` for every one.
**`ctrl` is 1:1 joint N*m torque** for all 7 joints -- the
`torque_limit_i / 25` scaling formula in HANDOFF.md applies directly with no
unit conversion.

**Gravity/Coriolis bias fraction** of each joint's torque limit, evaluated
at the model's resting forward-kinematics pose (`data.qfrc_bias` right
after `mj_forward`, no motion): all under 10% of torque authority
(`right_wrist_pitch_joint` highest at 9.7%, `right_shoulder_yaw_joint` and
`right_wrist_yaw_joint` under 0.001%). Gravity/Coriolis feedforward is
**not** the bottleneck -- it consumes a small fraction of any joint's torque
budget at rest.

**Baseline instrumented run** (existing Phase 3 controller, uniform
`Kp=180, Kd=18`, unweighted IK), full nominal trial:

| Joint | Torque limit (N*m) | Saturation fraction | RMS tracking error (rad) | Max tracking error (rad) |
| --- | --- | --- | --- | --- |
| shoulder_pitch | 25 | 0.0% | 0.056 | 0.170 |
| shoulder_roll | 25 | 0.0% | 0.020 | 0.053 |
| shoulder_yaw | 25 | 5.6% | 0.031 | 0.116 |
| elbow | 25 | 0.0% | 0.116 | 0.398 |
| wrist_roll | 25 | **92.4%** | 0.008 | 0.033 |
| wrist_pitch | 5 | **68.2%** | 0.064 | 0.234 |
| wrist_yaw | 5 | **74.8%** | 0.016 | 0.052 |

TCP RMS error 0.061 m, max 0.115 m over the trial. This confirms the Phase 3
hypothesis quantitatively: the two joints with only +/-5 N*m authority
(wrist_pitch, wrist_yaw) and, notably, `wrist_roll` (full +/-25 N*m but
still 92.4% saturated) spend most of the trial torque-saturated under a
uniform `Kp=180` gain that only requires ~1.6 degrees of error to saturate a
+/-5 N*m joint.

## Attempt 3B-1 — per-joint gain scaling by torque authority

Per HANDOFF.md's formula: `Kp_i = 180 * (torque_limit_i / 25)`,
`Kd_i = 18 * sqrt(torque_limit_i / 25)` (preserves damping ratio while
lowering natural frequency for weaker joints). Concretely: shoulder/elbow/
wrist_roll unchanged (180/18, scale 1.0); wrist_pitch/wrist_yaw reduced to
`Kp=36, Kd=8.05` (scale 0.2).

**Result: FAIL, and clearly worse than baseline.**

| Metric | Baseline | 3B-1 |
| --- | --- | --- |
| height_gain_m | 0.0051 | 0.0008 |
| both_pads_contact_cube | true | **false** |
| cube displacement before CLOSE (m) | (not tracked) | 0.0154 |
| TCP RMS error (m) | 0.061 | **0.215** |
| TCP max error (m) | 0.115 | **0.316** |
| shoulder_yaw saturation | 5.6% | **35.4%** |
| elbow RMS tracking error (rad) | 0.116 | **0.520** |
| wrist_pitch saturation | 68.2% | 9.5% (fixed, as intended) |
| wrist_yaw saturation | 74.8% | 4.1% (fixed, as intended) |

The targeted joints' saturation dropped exactly as the formula predicts.
But global tracking got dramatically worse: `shoulder_yaw` and `elbow`'s
**own PD gains were unchanged**, yet their saturation and tracking error
rose sharply. Diagnosis: `solve_dls_ik`'s pseudoinverse has no notion of
actuator torque limits -- it is a purely kinematic least-squares solve that
freely reassigns Cartesian-closing motion to whichever joint's Jacobian
column makes that cheapest in joint-space, every control step, based on
current (not target) qpos. Lowering wrist_pitch/yaw's gains made them
respond more slowly (lower natural frequency, even at matched damping
ratio), so at any instant they lagged further behind their continuously
recomputed IK target than before; the IK then compensated by demanding
larger corrections from shoulder_yaw/elbow on the next call, which those
joints' *own* torque limits could not always satisfy either, and the whole
7-joint system's tracking degraded. Per-joint PD retuning alone cannot fix
a coupling problem that originates upstream in the IK.

## Attempt 3B-2 — trajectory smoothing + settle-before-close

Kept 3B-1's per-joint gains; replaced the fixed 1.0 s approach phase with a
settle-gated one: track the grasp waypoint, then hold and keep tracking
until TCP position error <= 8 mm and max right-arm joint speed
<= 0.15 rad/s, or up to 1.5 s of extra settle time elapses, before ever
commanding the fingers closed.

**Result: FAIL, no better than 3B-1** (in fact slightly worse on TCP
error). `height_gain_m=0.0008`, `both_pads_contact_cube=false`, TCP RMS
error 0.321 m (up from 3B-1's 0.215 m), `wrist_roll` saturation 88.8%,
`shoulder_yaw` 33.0%. The settle gate exhausted its full 1.5 s extra-time
budget every run without ever reaching the tolerance (the system does not
converge to a steady state under these gains at all within the available
window) -- confirming this is not a trajectory-speed problem but the same
IK/PD coupling instability identified in 3B-1's diagnosis. Smoothing the
*commanded* trajectory does not help when the *tracking* itself cannot
converge.

## Attempt 3B-3 — torque-weighted DLS-IK, reverted to baseline PD gains

Per-joint PD gain scaling (3B-1) is reverted to baseline uniform
`Kp=180, Kd=18` (proven in the baseline run not to overload shoulder/elbow
on its own). Instead, `solve_dls_ik` is generalized to an optional weighted
pseudoinverse `dq = W J^T (J W J^T + damping^2 I)^-1 err` (implemented in
`controller.py`; `dls_weights=None` preserves the exact original unweighted
behavior for every existing caller/test). Weights are each joint's torque
limit normalized to the strongest joint:
`[1.0, 1.0, 1.0, 1.0, 1.0, 0.2, 0.2]` (shoulder x3/elbow/wrist_roll = 1.0,
wrist_pitch/yaw = 0.2) -- directly discouraging the IK from leaning on the
low-torque wrist joints to close Cartesian error, addressing the coupling
mechanism identified in 3B-1 at its source rather than downstream in the PD
layer.

**Result: FAIL, but the closest of the three attempts and closer to
baseline behavior.**

| Metric | Baseline | 3B-3 |
| --- | --- | --- |
| height_gain_m | 0.0051 | 0.0005 |
| both_pads_contact_cube | true | true |
| TCP RMS error (m) | 0.061 | 0.064 |
| TCP max error (m) | 0.115 | 0.118 |
| wrist_roll saturation | 92.4% | 91.8% |
| wrist_pitch saturation | 68.2% | 66.3% |
| wrist_yaw saturation | 74.8% | 69.8% |

The weighting avoided 3B-1/3B-2's global-tracking collapse (TCP error stays
near baseline, both pads regain contact), but only mildly reduces
wrist-joint saturation (1-5 percentage points) and does not fix the
underlying problem: at `Kp=180`, any +/-5 N*m joint saturates at roughly 1.6
degrees of position error, which the reach/grasp trajectory requires almost
continuously regardless of how lightly the IK leans on that joint. Height
gain is actually slightly worse than baseline (0.0005 m vs 0.0051 m),
i.e. this attempt did not net-improve the grasp outcome, only the
tracking-error symptom.

## Root cause, restated with Phase 3B evidence

Phase 3's diagnosis (uniform PD gains mismatched to a 5x torque spread) is
correct but incomplete. The full picture: (1) a uniform high-Kp PD gain
saturates the low-torque wrist joints almost immediately under any real
tracking error, and (2) the DLS-IK computing joint targets has no model of
actuator torque limits at all, so it does not compensate for (1) -- it
keeps assigning wrist-heavy corrections regardless of whether the wrist can
deliver them, and when a downstream PD fix (per-joint gains) or an IK-side
fix (torque weighting) is applied in isolation, the *other* layer's
ignorance of the change either amplifies the residual error (3B-1/3B-2) or
leaves the saturation largely intact (3B-3). A real fix likely needs both
layers changed *together and re-tuned as a system* -- e.g. torque-weighted
IK combined with gains re-derived for the new IK's resulting joint-target
distribution, evaluated iteratively -- which is more than one bounded
adjustment can responsibly cover within this budget.

## Nominal trial result (all attempts)

| Attempt | both_pads_contact | height_gain_ge_0.08m | lifted_ge_2s | finite_bounded | released | Overall |
| --- | --- | --- | --- | --- | --- | --- |
| Baseline (Phase 3) | yes | no (0.0051 m) | no (0.0 s) | yes | yes | FAIL |
| 3B-1 | **no** | no (0.0008 m) | no (0.0 s) | yes | yes | FAIL |
| 3B-2 | **no** | no (0.0008 m) | no (0.0 s) | yes | yes | FAIL |
| 3B-3 | yes | no (0.0005 m) | no (0.0 s) | yes | yes | FAIL |

Full per-attempt telemetry, instrumentation, and the resolved ctrl-to-torque
mapping: `logs/phase3b_controller_tuning.json`.

## 5-position-variant evaluation

Not run. Gated on nominal success, which did not occur in any of the 3
attempts.

## Automated tests

```bash
.venv/bin/python -m unittest tests/test_phase3_gripper.py -v      # 7 tests, PASS
.venv/bin/python -m unittest tests/test_phase3_controller.py -v   # 7 tests, PASS
.venv/bin/python -m unittest tests/test_phase3_grasp.py -v        # 10 tests: 4 boundary-guard PASS, 3 nominal-acceptance FAIL (honest, unchanged from Phase 3), 3 supporting-criteria PASS
```

The 3 nominal-acceptance failures (`height_gain`, `continuous lift`,
overall `pass`) are unchanged from Phase 3's recorded failure -- Phase 3B
did not fix the grasp, and the test correctly still fails rather than being
weakened.

## Blockers before any further attempt (out of this budget)

- A real fix likely requires co-designing the IK weighting and PD gains
  together (not as two independent bounded tweaks), and probably a
  fundamentally different reference-tracking scheme for the IK target
  (e.g. rate-limiting the IK target itself, not just the PD's step-from-
  current-qpos, so the "target" the PD chases is not itself jumping every
  control cycle for a lagging joint).
- `wrist_roll` saturates at ~90%+ across every attempt despite having full
  +/-25 N*m authority (same as shoulder/elbow) -- this joint's high load
  is not explained by the torque-spread hypothesis alone and deserves its
  own targeted investigation before a future attempt.
- Per HANDOFF.md and the tuning budget: do not proceed to Task 2, and do
  not relax the acceptance criteria or introduce a scripted grasp
  constraint to manufacture a pass.
