# Phase 7A — TCP Workspace and Conditioning Map

> **Evidence record — not reviewer reading.** Exhaustive per-phase audit
> trail, kept so every number in the submission is traceable. The
> reviewer-facing account is `README.md` and
> `submission/entrance_test_report.md`.

Date: 2026-09-04

## Purpose

Phase 4A recorded that "a full Jacobian-conditioning map over the workspace
was not computed in this phase — out of scope." This phase computes it,
motivated by a new task (door opening, see the project plan) that needs to
place an articulated object's geometry from measured reachability rather
than choosing geometry first and testing it. The map is a reusable
artifact independent of whether the door proceeds.

## Method

Physics-free (`mujoco.mj_step` never called). A 3D grid of TCP targets —
`x ∈ [0.100, 0.425]` step 0.025, `y ∈ [-0.300, 0.075]` step 0.025,
`z ∈ [0.750, 0.950]` step 0.050 — 1,120 points total, each solved **cold**
(warm-started from the fixed `mj_resetData` pose, not from a neighbouring
grid point), using the shipped `solve_ik_waypoint`
(`controller_3c.py:50`) at its shipped tolerance (`IK_POS_TOL = 8mm`). Cold
starting makes the map order-independent and reproducible; it is a
deliberately different choice from `diagnose_pick_place_reachability`'s
warm-chained solves, which model a continuous motion instead.

At each solved configuration this also records the Yoshikawa
manipulability `sqrt(det(J Jᵀ))` and the smallest singular value of the
3×7 position Jacobian (which is what actually locates a kinematic
singularity, as opposed to a residual number that only says whether *this*
IK call happened to converge), plus the natural wrist orientation residual
via the shipped `orientation_residual_rad()`.

Implementation: `tasks/g1_pick_place/workspace_map.py`. Raw data:
`logs/phase7a_tcp_workspace_map.json` (1,120 points). Runtime: 6m13s on
this host for the full sweep — recorded, not estimated.

## Finding 1 — the quoted "3cm × 4.5cm envelope" is a sampling artifact, not a wall

The envelope repeated throughout this project's docs (`cube_dx ∈
[-0.035,-0.005]`, `cube_dy ∈ [-0.01,0.035]`) comes from
`logs/phase5e_pilot_ik_grid.json`, an 81-point grid over *cube spawn
positions* run through the full 7-waypoint pick-place chain. Re-reading
that grid directly: every one of its `dx=-0.035` column entries is
reachable — the grid simply stops sampling there. The **`+x` side is a
genuine kinematic wall** (the whole `dx=+0.005` row fails, 27.1mm residual
at `x_plus_0.03`, consistent with Phase 4A), but the `-x` side (toward the
robot) was never explored.

This sweep answers it directly: at table height (z=0.75), single-waypoint
TCP reachability spans **`x ∈ [0.10, 0.325]`, roughly 22cm** toward the
robot before the wall, not 3cm. The pick-place envelope is tight because
grasping a cube requires *several* waypoints (PREGRASP, APPROACH, LIFT,
TRANSPORT, ...) to be simultaneously reachable and mutually consistent,
not because the arm's single-point reach is that small.

## Finding 2 — Phase 4A's rejections are now explained quantitatively

| Cube position | Residual | σ_min | Manipulability | Outcome |
| --- | --- | --- | --- | --- |
| `x_minus_0.03` | 5.06mm | **0.0527** | 0.01358 | accepted, physically verified |
| nominal | 7.83mm | 0.0140 | 0.00384 | accepted (Task 1's operating point) |
| `y_plus_0.03` | 7.63mm | 0.0121 | 0.00328 | accepted |
| `y_minus_0.03` | 8.79mm | 0.0084 | 0.00231 | **rejected** by Phase 4A |
| `x_plus_0.03` | 26.95mm | 0.0063 | 0.00174 | **rejected** by Phase 4A |

The accepted/rejected boundary sits at **σ_min ≈ 0.010**. Phase 4A's
rejections were a real conditioning cliff, not noise — this sweep supplies
the retroactive quantitative account. It is also notable that **Task 1's
own nominal operating point (0.00384) sits near the first percentile of
manipulability across the whole measured workspace** (median 0.0207): the
task has been running at one of the worst-conditioned reachable points on
the table the entire time.

## Finding 3 — Phase 4F's orientation failure is a height effect, not an arm limitation

Aggregating the map by height:

| z | Reachable | Median orient. residual | Min orient. residual | Points within 7° |
| --- | --- | --- | --- | --- |
| 0.75 (Task 1's height) | 67% | 37.7° | 31.1° | **0** |
| 0.80 | 81% | 25.8° | 18.0° | 0 |
| 0.85 | 90% | 15.1° | 6.4° | 9 |
| **0.90** | 98% | **10.0°** | **0.9°** | **67** |
| 0.95 | 100% | 16.0° | 1.1° | 27 |

**At table height, no point anywhere in the swept workspace brings the
wrist within Phase 4F's 7° orientation tolerance.** That phase's
orientation-constrained IK failure (needing 30–70mm of position error to
reach 7° near the cube grasp point) was therefore not a defect in
`solve_ik_waypoint_oriented` or in this arm's kinematics in general — it
is what reaching at `z≈0.735` costs, full stop. At `z=0.90`, 67 of the
sampled points meet the same 7° bound outright, at more than 10× better
conditioning (median σ_min 0.138 vs 0.099 at table height, and vs 0.014 at
Task 1's exact operating point).

## Door-geometry derivation and gate result

Pre-registered gate (fixed before the sweep ran): proceed with a hinged
door only if an arc of at least 30° at radius at least 0.06m stays inside
a region that is simultaneously reachable (`< 8mm`), within the shipped
7° orientation tolerance, and at least as well conditioned as
`x_minus_0.03` (σ_min ≥ 0.0527 — the best-conditioned point at which
Task 1's grasp is already physically verified). Otherwise, fall back to a
drawer.

Handle height was fixed at `z = 0.90` per Finding 3 — the region where the
orientation constraint is achievable at all. A deterministic brute-force
search (`search_largest_arc`, `workspace_map.py`) over pivot positions,
radii `{0.06, 0.08, 0.10, 0.12, 0.14}m`, and start angles found:

**Result: GO_HINGE. 2,994 admissible arcs.** The largest: **90° swing,
0.14m radius, 19.8cm handle chord**, e.g. pivot `(0.16, -0.06)` or
`(0.18, -0.28)` — both hold σ_min ≥ 0.130 and orientation residual ≤ 6.8°
across the full arc. Full ranked list:
`logs/phase7a_derived_door_geometry.json`.

**Recommendation for Phase 1 (scene design), not a gate change**: the
maximal 90° candidates hug the 7° orientation boundary at their endpoints
(6.2–6.8° of 7°) because the search reports the *largest* arc that still
clears the gate — by construction, the largest passing arc has the least
margin. A reduced swing (θ≈45–60°) at the same pivot/radius sits well
inside the boundary with comfortable margin on every criterion, at the
cost of a shorter but still generous chord (e.g. r=0.12, θ=60° → 12.0cm
chord, worst σ_min 0.111, worst orientation 5.5° warm-chained — see
`data/task3_canonical_config.json` once locked). Phase 1 should pick from
the ranked list with margin as a stated design criterion, not chase the
single largest number.

## Incidental finding: a pre-existing scene artifact

Checking self-collision at several solved configurations across the
workspace surfaced a **static right-ankle/table-top penetration
(-11.1mm) present at the plain `mj_resetData` reset pose**, independent of
arm position — i.e. it is a pre-existing artifact of the shipped Task 1
scene (legs are unactuated under the pelvis/torso weld), not something
this sweep introduced. It does not affect any measured result in this or
prior phases (the legs carry no control authority and no task-relevant
contact), but is recorded here since it was directly observed, not
inferred, and future phases touching leg geometry should be aware of it.

## What this phase does not establish

- All reachability figures are **cold-started**, single-waypoint. A real
  arc trajectory is warm-chained (each waypoint solved from the previous
  one's result, as `_drive_smooth` does), which can differ slightly —
  Phase 1/3 of the door plan must re-check the locked geometry
  warm-chained via `diagnose_door_reachability` before running physics,
  exactly as `diagnose_pick_place_reachability` already does for Task 1.
- Self-collision was spot-checked at 7 points, not swept across the full
  grid; Phase 3 of the door plan's swept-collision pre-check covers this
  properly for the locked arc.
- This sweep does not simulate any door panel, hinge dynamics, or contact
  — it characterises the arm alone.
