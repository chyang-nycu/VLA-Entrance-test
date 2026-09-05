# Phase 4F — Human Acceptance Decision: Task 1 Prototype

Date: 2026-09-02

## Decision

**Task 1 status is restored as: "Prototype task completed with a documented
grasp-slip limitation."**

This is a change in project **acceptance policy** for the entrance-test
prototype, not a change to any measured data. No log, threshold, test
assertion, report, or video was altered to produce this decision.

## What was reviewed

Video reviewed: `artifacts/phase4f_task1_full.mp4` and
`artifacts/phase4f_bilateral_contact_view.mp4` (Phase 4F, commit `e6b53f4`).

## What visibly improved (human judgment, video-based)

- The decorative-hand/cube visual penetration confirmed in Phase 4D and
  fixed in Phase 4E remains fixed — no visible clipping of a non-functional
  mesh through the cube.
- The full task sequence — approach, grasp, lift, transport, lower,
  release, retreat — looks reasonable end to end: the cube is genuinely
  lifted, carried, and placed, matching the numeric record (height gain
  0.1083 m, real nonzero bilateral contact force throughout HOLD).

## Measured result (unchanged, restated here for the decision record)

From `logs/phase4f_orientation_grasp.json` (Phase 4F, attempt 3 final
configuration), unmodified:

| Metric | Value | Tightened engineering target | Result |
| --- | --- | --- | --- |
| Max 3D TCP-frame slip while grasped | 0.02592 m (~25.9 mm) | <= 0.010 m | **FAIL** |
| Cube-center-within-pad-vertical-overlap | 36.5 mm max offset | <= 30 mm | **FAIL** |
| Cube height gain | 0.1083 m | >= 0.08 m | PASS |
| Continuous off-table hold | >= 2.0 s | >= 2.0 s | PASS |
| Bilateral contact force | positive, finite throughout | required | PASS |
| Physical release / settled placement | — | required | **FAIL** (per Phase 4F report) |
| Deterministic across 5 reruns | bit-identical (0.02591759542068572 m every run) | — | PASS |

8 of the 11 Phase 4F acceptance criteria pass; the three that fail are the
grasp-quality-critical ones above (`max_slip_while_grasped_le_10mm`,
`cube_center_within_pad_vertical_overlap`, `physical_release_and_settled_placement`
— confirmed directly from `logs/phase4f_orientation_grasp.json`'s
`criteria_grasp_stability_4f` dict). Note: `reports/phase4f-orientation-grasp-stabilization.md`
and this project's README/HANDOFF state "7 of 11 pass" — that is an
arithmetic slip in the original Phase 4F write-up (11 criteria minus the
3 explicitly listed failures is 8, not 7); the underlying data in
`logs/phase4f_orientation_grasp.json` is correct and unambiguous, and is
what this decision record uses. Per this phase's own instruction not to
alter historical reports, `reports/phase4f-orientation-grasp-stabilization.md`
is left as originally written; this note exists so the discrepancy is not
silently repeated.

## Why the prototype is accepted despite missing the stricter internal target

This project is explicitly framed (see `README.md`, `HANDOFF.md`) as an
**entrance-test prototype**: a from-scratch build of a physically honest
grasp-and-place capability under a fixed-base, torso-constrained upper-body
manipulation baseline, without any faked contact, weld, teleport, or
scripted assistance. The <=10 mm max-slip criterion introduced in Phase 4E
was an internal engineering-quality bar adopted specifically to chase down
a real defect (an unstable, near-drop grasp) once human video review first
raised a concern in the Phase 4D/4E cycle. That defect — the decorative-
hand visual/collision mismatch and the outright below-N_min grip-force
instant found in Phase 4E's diagnosis — has been fixed. What remains is a
genuine but bounded kinematic limitation (documented in Phase 4F: the
position-priority waypoint IK's redundancy resolution does not fully
eliminate wrist-orientation-driven contact-band drift at this specific
grasp geometry) that produces measurable, but not task-defeating, slip.
My direct visual review of the resulting task execution — the standard
I have applied throughout for "does this look physically honest and
functionally complete" — finds the executed task acceptable for prototype
purposes.

This decision therefore separates two independent questions that Phase
4E/4F's <=10 mm bar had collapsed into one:

- **Strict engineering-quality grasp** (max slip <=10 mm, tight pad-overlap
  margin): **FAIL**. This bar is not met, is not claimed to be met, and no
  test or log has been changed to suggest otherwise.
- **Entrance-test prototype task completion** (does the robot physically
  and honestly pick up the cube and place it in the target, without any
  scripted assistance, in a way I judge functionally and
  visually acceptable): **PASS, with a documented limitation.**
- **My visual review of the Phase 4F videos**: **PASS.**

## Limitations and future improvement

- Max grasp-phase slip (~25.9 mm) is real and roughly 2.6x the internal
  10 mm engineering target. It does not currently cause task failure in
  the nominal configuration, but is not a comfortable margin, and a
  different cube mass/friction/geometry could plausibly push it further.
- The root cause is a kinematic reachability conflict, not a tuning gap:
  Phase 4F's attempt 2 showed that fully leveling the wrist at the
  nominal grasp waypoint requires 30-70 mm of TCP position error with the
  current position-priority IK formulation. Closing this gap in a future
  phase most likely requires a waypoint/approach-geometry redesign (e.g.
  a different approach direction or wrist configuration that reaches a
  level orientation without sacrificing position accuracy) rather than
  another gain or weighting adjustment within the already-exhausted
  Phase 4F attempt budget.
- The Phase 4A/4B "reachable variant" envelope (x-0.03, y+0.03) was only
  evaluated informationally under this configuration (Stage B did not run
  as an authorized evaluation, since Stage A's gate was not met) and
  should be re-evaluated if/when the slip gap is closed.
- This remains a fixed-base, torso-constrained upper-body manipulation
  baseline. No free-standing, mobile-base, or full-body capability is
  implied or claimed by this decision.

## Explicit statement: no numeric test was weakened

No threshold in `tests/test_phase4f_orientation_grasp.py` or any other
test file was changed, loosened, deleted, or marked `expectedFailure` as
part of this decision. `test_max_slip_while_grasped_still_exceeds_tightened_bar`
and `test_grasp_stability_pass_4f_is_honestly_false` continue to assert,
and continue to pass by asserting, that the strict 10 mm criterion is not
met (measured value locked at 0.02592 m, `assertAlmostEqual` to 4 places).
No historical log (`logs/phase4f_orientation_grasp.json` and earlier) was
edited. No historical report (Phase 3 through 4F) was edited. This
document adds a new, separate acceptance-policy conclusion on top of an
unchanged evidentiary record.
