# Phase 4C: Task 1 Evidence Hardening and Video Capture

Date: 2026-09-02

**Task 1** (verbatim): "Pick up the red cube and place it in the blue target area." This phase does not add, retune, or change any capability -- it audits and corrects one reported metric, and packages the already-committed Phase 4B result (commit `363aa83`) so it is understandable without running code.

**Platform framing (binding, unchanged): this is a fixed-base, torso-constrained upper-body manipulation baseline.** The pelvis and torso remain rigidly welded to the world; only the right arm and gripper move. This is not full-body or free-standing manipulation.

## Scope

- Baseline: Phase 4B, commit `363aa83` ("feat: complete G1 Task 1 pick and place")
- No controller, gain, or trajectory parameter was changed. `arm_kp=400.0`, `arm_kv=25.0`, `gripper_kp=150.0`, `gripper_kd=10.0` are byte-identical to the committed Phase 4B configuration (verified: `tasks/g1_pick_place/controller.py`, `controller_3c.py`, `run_grasp_test.py`, `run_grasp_test_3c.py` have zero diff against `363aa83`; `run_pick_place.py`'s `ARM_KP_4B`/`ARM_KV_4B`/`GRIPPER_KP_3C`/`GRIPPER_KD_3C` usages are unchanged).
- `reports/phase4b-task1-pick-place.md` is left completely unedited, per instruction -- it remains the historical record of the original (incorrect) slip figure. The correction lives here, in this new report, as a clearly labeled addendum.

## A. Slip-metric audit

### The reported number and why it looked wrong

Phase 4B reported `max_cube_slip_m = 0.1561555402975266` for the nominal trial (`reports/phase4b-task1-pick-place.md`, section 5/10) -- larger than the cube's own 0.07 m footprint, despite bilateral finger contact never being lost (`contact_lost_during_transport = False` for every recorded trial). That combination is not physically sensible for genuine slip inside a closed, contact-retaining grip, which is exactly what the user's audit request flagged.

### Root cause (confirmed, not guessed)

The metric's code, in `run_trial_pick_place()`'s inner `_step_once()` (`tasks/g1_pick_place/run_pick_place.py`):

```python
if grasp_offset_ref["value"] is not None:
    tcp_pos = data.site_xpos[site_id]
    tcp_rot = data.site_xmat[site_id].reshape(3, 3)
    local_offset_now = tcp_rot.T @ (cube_xyz - tcp_pos)
    slip = float(np.linalg.norm(local_offset_now - grasp_offset_ref["value"]))
    telemetry["max_cube_slip_m"] = max(telemetry["max_cube_slip_m"], slip)
```

- **Reference transform**: `grasp_offset_ref["value"]` is captured exactly once, at `VERIFY_BILATERAL_CONTACT`, as `R_tcp0^T @ (cube_pos - tcp_pos)` -- the cube's position in the TCP's own frame at the instant grasp is verified. This part is correct: it is a proper, invariant local-frame reference, not a raw world-frame snapshot.
- **Rotation handling**: `local_offset_now = R_tcp^T @ (cube_pos - tcp_pos)` is the mathematically correct way to express the cube's position in the TCP's current frame, so a pure re-orientation of the TCP with the cube still rigidly attached produces `local_offset_now == grasp_offset_ref["value"]` and therefore zero slip. This part is also correct -- confirmed by the new `SlipMathUnitTest.test_pure_tcp_rotation_with_rigid_cube_offset_produces_zero_slip` (see below).
- **Reference timestep**: once, at grasp verification. Not re-captured per step. Correct.
- **The actual bug**: the `if grasp_offset_ref["value"] is not None:` guard is `True` for the rest of the trial once set -- it never turns back off. So `_step_once()` kept updating `max_cube_slip_m` through **every remaining phase**, including `OPEN`, `RELEASE_SETTLE`, `VERIFY_RELEASE`, `RETREAT`, and `VERIFY_TASK_SUCCESS` -- i.e., long after the gripper was commanded open and the cube was no longer held. During `RETREAT`, the arm withdraws and the TCP moves away from the now-stationary cube; that growing TCP-cube separation was being computed with the exact same formula and folded into "grasp slip," even though it reflects an intentional, expected release, not a slipping grip.
- **Was TCP-to-cube offset confused with the change in that offset?** No -- the subtraction against `grasp_offset_ref["value"]` is present and correct throughout. **Was max-absolute-offset reported instead of a displacement relative to reference?** No, same reasoning -- always a difference, not a raw offset. **The defect is entirely about the time window**, not the math itself.

Directly confirmed with the real (unmocked) nominal trial after instrumenting the fix: `post_release_tcp_cube_separation_m = 0.14860671167299208` for the nominal trial -- within 2 cm of the old `max_cube_slip_m = 0.1561555402975266`, and the true maximum is reached slightly later than release itself (during `RETREAT`'s further arm withdrawal). This is the root cause, not a hypothesis: the old number was overwhelmingly post-release TCP-cube separation, mislabeled as slip.

### Corrected metrics

Implemented in `run_trial_pick_place()`, gated on the existing `carrying` signal (`"grip_only"` or `"full"`, already used elsewhere in the same function to gate contact-loss/height-safety checks -- reused, not reinvented) so slip is only ever accumulated while the gripper is commanded closed and the cube is physically grasped:

| Metric | Definition |
| --- | --- |
| `grasp_reference_offset_tcp_frame` | The reference vector itself, captured once at `VERIFY_BILATERAL_CONTACT` (unchanged capture point) |
| `max_slip_during_lift` | max slip over `LIFT`/`HOLD` |
| `max_slip_during_transport` | max slip over `TRANSPORT_ABOVE_TARGET`/`SETTLE_ABOVE_TARGET` |
| `max_slip_during_lower` | max slip over `LOWER_TO_TARGET`/`SETTLE_LOWER` |
| `slip_at_release` | the slip value at the last step of `SETTLE_LOWER`, immediately before `OPEN` is commanded |
| `post_release_tcp_cube_separation_m` | max **separation** (not slip) over `OPEN`/`RELEASE_SETTLE`/`VERIFY_RELEASE`/`RETREAT`/`VERIFY_TASK_SUCCESS` -- never labeled grasp slip anywhere in code, tests, or this report |
| `max_cube_slip_m` (legacy) | Retained, unmodified, byte-identical to the Phase 4B formula, specifically so the historical 0.156 m figure stays reproducible for this addendum -- no longer treated as the authoritative slip number |

The slip-frame math itself was factored into two pure, unit-testable functions (`tcp_local_cube_offset`, `relative_slip_m`) with identical behavior to the original inline code -- confirmed by rerunning the nominal trial before and after the refactor and observing byte-identical `height_gain_m`, `final_xy_target_error_m`, and `max_cube_slip_m` (`0.10838913826086727`, `0.014564117068399008`, `0.1561555402975266` respectively, in both cases).

### Corrected numbers (nominal and Stage B variants)

Recomputed by rerunning the exact committed Phase 4B configuration through the corrected code (`tasks/g1_pick_place/audit_slip_metric.py`, output: `logs/phase4c_slip_audit.json`) -- not by post-hoc recalculation from the Phase 4B log file, since the original log did not retain per-step data granular enough to recompute phase-scoped slip after the fact. All three variants remain fully deterministic (`nominal.deterministic = true` in the audit log, 5/5 reruns).

| Variant | Old `max_cube_slip_m` | `max_slip_during_lift` | `max_slip_during_transport` | `max_slip_during_lower` | `slip_at_release` | `post_release_tcp_cube_separation_m` |
| --- | --- | --- | --- | --- | --- | --- |
| nominal | 0.15616 | 0.03198 | 0.04489 | 0.05187 | 0.01978 | 0.14861 |
| x_minus_0.03 | 0.15563 | 0.01866 | 0.02530 | 0.03398 | 0.00796 | 0.14803 |
| y_plus_0.03 | 0.14965 | 0.03340 | 0.04667 | 0.05375 | 0.02059 | 0.14138 |

**Old definition vs. corrected definition, in one line**: the old number measured "how far the cube ended up from the TCP by the end of the trial, including well after release" (effectively a release/retreat-separation metric mislabeled as slip); the corrected numbers measure "how much the cube moved relative to the closed fingers while still being carried," which is what "grasp slip" should mean.

### Honest reading of the corrected numbers

The genuine grasp-phase slip is real and non-trivial (3.3-5.4 cm depending on phase and variant) but roughly 1/3 of the old figure and, critically, bounded and monotonically small relative to the cube's 7 cm footprint -- consistent with Phase 4B's own qualitative description ("gradual, compliant repositioning... within the still-unbroken grip") once that description is no longer conflated with post-release motion. This does not change any pass/fail outcome: all Stage A/Stage B results in `reports/phase4b-task1-pick-place.md` (task-pass counts, grasp/placement pass, contact retention, final XY error) are unaffected, since slip was never part of any acceptance criterion.

### New tests (`tests/test_phase4c_slip_audit.py`, 10 tests, all passing)

- `SlipMathUnitTest` (3 synthetic, purely mathematical tests against `tcp_local_cube_offset`/`relative_slip_m`): pure world translation of TCP+cube together -> 0.0 m slip; pure TCP rotation with a rigidly-attached cube offset -> 0.0 m slip (checked at 4 different rotation angles); a known 5 mm relative displacement in the TCP's local frame -> reports exactly 5 mm.
- `PostReleaseIsolationTest` (6 tests against the real, unmocked nominal pipeline): confirms the fix is measurement-only (`task_pass`, `height_gain_m`, `final_xy_target_error_m` unchanged from Phase 4B's committed values); confirms the legacy field still reproduces the historical 0.1561555402975266 figure; confirms `post_release_tcp_cube_separation_m` explains (is within 2 cm of) the old legacy number; confirms every corrected grasp-phase metric is both far below the old number and below the cube's own footprint; confirms `slip_at_release` (captured before `OPEN` even runs) is strictly less than the post-release separation.
- `StageBVariantsUseCorrectedMetricsTest` (1 test): confirms the `y_plus_0.03` variant -- whose trial exits through the `VERIFY_TASK_SUCCESS`-fails-placement path, a different code path through `_finalize` than the nominal DONE path -- also produces bounded, sane grasp-phase slip values.

## B. Video capture

- Dependency: `imageio-ffmpeg` installed into the project's existing venv (`.venv/bin/pip install imageio-ffmpeg`, user-local, no system-wide/admin install) -- provides a bundled, working `ffmpeg` binary (verified: native x86_64 machine, `ffmpeg version 7.1` runs directly, no Rosetta/compatibility issue).
- MuJoCo's offscreen `mujoco.Renderer` works out of the box in this environment (verified with a standalone render before building the capture script).
- Rendering approach: a purely additive, opt-in `frame_callback` parameter was added to `run_trial_pick_place()` (default `None`, zero effect on any existing caller -- confirmed by rerunning the full suite, still 104/104 green after the addition). The callback is invoked once per physics step with `(phase, model, data)` and has no path back into control or physics; it can only read state and render.
- Camera: a single fixed (non-tracking) `mujoco.MjvCamera`, framed on the midpoint between the cube's start position and the target pad, with constant `lookat`/`distance`/`azimuth`/`elevation` for the entire episode -- never re-aimed.
- Output: **`artifacts/phase4b_task1_nominal.mp4`** (filename kept from the Phase 4B spec even though produced in this phase, per instruction).

| Property | Value |
| --- | --- |
| Resolution | 640 x 480 |
| FPS | 29.41 (physics stride of 17 steps at `TIMESTEP=0.002s`, i.e. normal/real-time playback rate) |
| Frame count | 375 |
| Duration | 12.75 s |
| File size | 229,060 bytes (0.22 MB) |
| Codec | H.264 (`libx264`), `yuv420p` |
| Decode verification | Read back with `imageio.get_reader`; decoded frame count (375) matches encoded frame count exactly; `meta_fps` matches the requested 29.41 |

Content: full nominal episode, `RESET` through `DONE`, `task_pass = True`; shows G1, the red cube, the table, and the blue target pad throughout, from the fixed third-person camera above. Instruction text ("Task 1: pick up the red cube and place it in the blue target area") is recorded here in the report rather than burned into the video as an overlay (optional per instruction; not added, to avoid adding a text-rendering dependency for a non-required feature).

At 229 KB, this is far below any reasonable large-file concern and consistent with the repository's existing precedent of committing small evidence media (e.g. `artifacts/phase4a_variant_sweep_demo.gif` at 4.5 MB is already committed) -- MP4 (not GIF) is used since it worked on the first real attempt; no fallback was needed.

### Still frames (`artifacts/`)

| File | Moment |
| --- | --- |
| `phase4c_still_grasp.png` | First step of `LIFT` (bilateral grasp already verified) |
| `phase4c_still_transport.png` | Midway through `TRANSPORT_ABOVE_TARGET`'s sub-waypoints |
| `phase4c_still_released.png` | Last step of `VERIFY_TASK_SUCCESS` (cube released, resting in the target) |

All three (~192-196 KB each) show the robot, cube, table, and target pad clearly; the cube's position visibly differs between the grasp/transport stills (cube held next to the pad) and the released still (cube resting directly on the pad).

## C. Result summary

**Task 1**, "pick up the red cube and place it in the blue target area," is implemented as a fixed-base, torso-constrained upper-body manipulation baseline: the G1's pelvis and torso are rigidly welded to the world (Phase 3C's trunk-weld design, unchanged); only the right arm (bounded MuJoCo position servos, `implicitfast` integrator) and a task-local physical parallel gripper move. The controller is Phase 3C's waypoint-based damped-least-squares IK with null-space joint-limit/posture objectives, extended in Phase 4B with smoothly-ramped multi-waypoint Cartesian transport/lowering segments (`_drive_smooth`) to carry the grasped cube to a static blue target pad and release it there, verified by an eight-condition, dwell-gated, purely state-based success detector (never judged from rendering or color).

- **Nominal result**: 5/5 deterministic `task_pass = True` (unchanged from Phase 4B; height gain 0.1084 m, final XY target error 14.6 mm against a 15 mm margin).
- **Supported-envelope result** (Phase 4A's 3 grasp-reachable variants, shared unmodified config, no per-variant tuning): **2/3 (67%)** complete the full task (nominal, x-0.03); `y_plus_0.03` grasps successfully (100% contact retention, 3/3) but misses the tight 15 mm placement margin (20.4 mm actual) in all 3 trials -- a placement-accuracy limit, not a dropped grasp.
- **Original five-variant coverage**: **2/5** -- `x_plus_0.03` and `y_minus_0.03` remain excluded from every denominator in this project as pre-declared, Phase-4A-confirmed grasp-unreachable (APPROACH IK residuals 27.08 mm and 8.43 mm respectively, both over the 8 mm tolerance), never re-attempted here.
- **Success detector** (all required, `criteria_placement`, continuous for `TASK_SUCCESS_DWELL_S = 0.5 s`): cube center within 15 mm of target center; cube resting height within 1 cm of the pad's surface and touching neither finger pad; linear speed <= 0.02 m/s; angular speed <= 0.05 rad/s; all of the above true continuously (any single-step violation resets the streak -- HANDOFF.md: "Do not count success immediately when the cube first crosses the target boundary"); cube XY displacement during `RETREAT` <= 5 mm; no bilateral-contact loss or unsafe-height event during transport. `criteria_grasp` (Phase 3C's original 5, reused unchanged) must also all hold.
- **Corrected slip figures** (this phase's audit; see section A): genuine grasp-phase slip is 3.3-5.4 cm depending on phase/variant (not the previously reported 14.97-15.62 cm, which was dominated by post-release TCP-cube separation).
- **Fixed-base/torso-constrained limitation**: unchanged and restated here -- no free-standing, mobile-base, or full-body capability is implied or claimed.
- **Evidence**: [`artifacts/phase4b_task1_nominal.mp4`](../artifacts/phase4b_task1_nominal.mp4) (full episode), [`artifacts/phase4c_still_grasp.png`](../artifacts/phase4c_still_grasp.png), [`artifacts/phase4c_still_transport.png`](../artifacts/phase4c_still_transport.png), [`artifacts/phase4c_still_released.png`](../artifacts/phase4c_still_released.png); raw data in [`logs/phase4c_slip_audit.json`](../logs/phase4c_slip_audit.json) and (historical, unedited) [`logs/phase4b_pick_place_trials.json`](../logs/phase4b_pick_place_trials.json).

## D. Verification

Full regression this session: **104 tests, 0 unexpected failures** (94 pre-existing, unchanged, + 10 new in `tests/test_phase4c_slip_audit.py`). Of the 94 pre-existing, 3 remain Phase 3's legacy regression diagnostics (`tests/test_phase3_grasp.py`, converted in Phase 4A, untouched since); they continue to assert the exact historical failure values persist. No acceptance threshold was changed anywhere in this phase. No controller parameter was changed anywhere in this phase (verified via `git diff 363aa83` against every controller/scene-generator file touched).

## Time

Approximately 2 hours: slip-metric investigation and root-cause confirmation (~40 min), corrected-metric implementation and refactor-equivalence verification (~20 min), synthetic + isolation tests (~30 min), video/still capture pipeline (~25 min), documentation (~25 min).

## Limitations

- The corrected grasp-phase slip metrics (3.3-5.4 cm) are still a non-trivial fraction of the cube's 7 cm footprint -- a real characteristic of this gripper's stiffness budget under sustained transport/lowering loads, not eliminated by this audit, only correctly attributed and bounded.
- No text overlay was burned into the video (optional per instruction); the task instruction is documented here in the report and in the filename's provenance instead.
- This remains a fixed-base, torso-constrained upper-body manipulation baseline. Task 2 (cameras as a data-collection feature, dataset pipeline, language-conditioned variants, policy integration) was not implemented in this phase.
