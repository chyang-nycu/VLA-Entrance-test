# Phase 7 Summary — Task 3, Articulated Door-Opening

> **Evidence record — not reviewer reading.** Exhaustive per-phase audit
> trail, kept so every number in the submission is traceable. The
> reviewer-facing account is `README.md` and
> `submission/entrance_test_report.md` §8.

Date: 2026-09-05/06

## Motivation and result

Task 1 and Task 2 are both brief grasp-and-carry of a free rigid body.
Task 3 introduces a manipulation class this project had never touched:
**sustained contact with an articulated object under a kinematic
constraint** — a hinged cabinet door, opened by pulling a handle through
an arc while the gripper must stay closed on it the whole way. LIBERO's
articulated tasks ("open the microwave," "open the top drawer") are the
reference; its joint-position success predicate is what `criteria_door`
mirrors here.

**Headline result**: the arm reaches/exceeds the door's target open angle
(45.3° of a 45° threshold, deterministic across repeats). The disclosed
limitation: bilateral grip force touches exactly 0.0N at points during the
pull, so the stricter grip-retention and slip criteria fail —
`door_pass` is honestly `False`, the same treatment Task 1's Phase 4E gave
an analogous finding rather than hiding it.

## Phase-by-phase

| Phase | What it did | Result | Report |
| --- | --- | --- | --- |
| 0 | Measured a 1,120-point TCP workspace/conditioning map (physics-free); derived door geometry from it rather than choosing by hand | **Gate: GO_HINGE**, 2,994 admissible arcs before margin-selection | `reports/phase7a-workspace-map.md` |
| 1 | Built the passive hinged-door scene; found and fixed a 55mm resting-arm collision via a new clearance check | Scene gate passed; handle reuses Task 1's exact verified squeeze geometry | `reports/phase7b-door-scene.md` |
| 2 | Built the arc-following motion and full trial state machine; found and fixed 3 real bugs (units, wrong gripper constant, IK redundancy sensitivity) | Reaches 45.3° of 45° target | `reports/phase7c-door-motion.md` |
| 3 | Tested whether Phase 4F's orientation-IK conflict transfers to this pose | **No** — every arc waypoint meets both 8mm position and 7° orientation tolerance simultaneously | `reports/phase7c-door-motion.md` (§ Phase 3 resolved) |
| 4 | `criteria_door` (LIBERO-style joint threshold), `HingeInitGuard`, extended import-time self-audit (new `qfrc_applied` cheat surface), `data/task3_canonical_config.json` | Structural integrity confirmed | This report's phase 4 work is embedded in `door_open.py`; see §8 of `submission/entrance_test_report.md` |
| 5 | 34-test suite mirroring Task 2's template; found and fixed 3 more real bugs (live-pose reading, anti-cheat baseline, finger/palm clearance) | 34/34 pass; 345/345 project-wide | `reports/phase7d-door-tests.md` |
| 7E | Instrumented per-step Jacobian conditioning, contact force, and slip during a real pull; ran a causal isolation experiment | Force is a demonstrated partial cause; conditioning's correlation is confounded, not causal | `reports/phase7e-slip-causality.md` |

Seven real bugs were found and fixed across Phases 1, 2, and 5 — every one
by actually running physics or writing a test, never by inspection alone.
None were visible from a single nominal trial; each needed either a
different starting condition, a different geometry candidate, or a
dedicated test to surface.

## Three findings that reach beyond Task 3

**1. The project's quoted reachable envelope was a sampling artifact.**
`cube_dx ∈ [-0.035,-0.005]` (Phase 5E) was never wrong, but it was read as
a reach limit when it was actually the range a grid happened to sample.
Phase 7A's direct TCP sweep found single-waypoint reach extends to
`x=0.10` at table height — 22cm toward the robot, not 3.5cm.

**2. Task 1's own operating point is explained, not just described.**
Phase 4A's accepted/rejected boundary sits at a Jacobian smallest singular
value of ≈0.010 (accepted σ_min ∈ [0.0121, 0.0527]; rejected ∈ [0.0063,
0.0084] — a clean, non-overlapping n=5 gap). Task 1's nominal grasp point
(manipulability 0.00384) sits at the **0.2th percentile** of 976 reachable
workspace points — the task has run at essentially the single
worst-conditioned reachable point on the table the entire project.

**3. Orientation reachability is height-dependent, not architecturally
limited.** At Task 1's grasp height, no point in the measured workspace
meets a 7° wrist-orientation tolerance (median residual 37.7°, minimum
31.1°). 10cm higher, 67 of the sampled points do (median 10.0°, minimum
0.9°). Phase 4F's orientation-IK failure was a property of reaching at
table height, not of `solve_ik_waypoint_oriented` or this arm in general —
confirmed by Task 3 using it throughout with zero position-residual cost.

## The slip/force gap, consolidated

Phase 7E ran the one experiment that actually isolates a variable: gripper
force and arm kinematic conditioning are independent subsystems (force is
a gripper-PD parameter; conditioning is a property of the commanded
Cartesian arc). Re-running the identical arc at 1.5x/2x gripper gain held
conditioning fixed to 4 significant figures while cutting max slip from
22.3mm to 16.3mm and eliminating all contact-loss episodes (82.4% →
100% both-pads-contact). **Force insufficiency is a demonstrated causal
contributor.** It is not the whole story: `door_pass` is still `False` at
2x gripper force, and slip already exceeds the 10mm target at t=1.55s,
before the force decline is even visible (onset t=1.82s) — a second
contributor, most plausibly the same TCP-frame-rotation-during-motion
mechanism Task 1's Phase 4C/4E audits identified, remains unclosed.
Kinematic conditioning's own raw correlation with slip (r≈0.90) is real
but confounded with monotonic arc progress and was not supported as
causal by the one experiment run — see `reports/phase7e-slip-causality.md`
§6-7 for the full derivation, including why residualizing against time
made the confound *worse*, not better (the tell that a single trial can't
separate the two).

## Scope boundary

Per the approved plan: task + success criteria + tests + report. **No
demonstration dataset was collected for Task 3** — the existing two-rate
(10Hz/500Hz) HDF5 pipeline (Task 1's Phase 5B-5E) is untouched and was
never pointed at this task. See `HANDOFF.md`'s Task 3 entry for what would
need new authorization before that or further slip-closing work begins.
