# Phase 5A: Onboard RGB Observation Smoke Test

Date: 2026-09-03

## Scope

Adds one task-local onboard RGB camera, rigidly mounted on the G1's own
body, as a future VLA policy observation source. This phase does **not**
build the HDF5 dataset pipeline, implement Task 2, train anything, or
retune Task 1's grasp/transport/placement controller in any way. It is
purely additive: one new `<camera>` element in a new, isolated task-local
scene file.

- New module: `tasks/g1_pick_place/camera_observation.py`
- New scene generator: `write_grasp_scene_5a()` (built by re-parsing
  `write_grasp_scene_4b()`'s own output, never by modifying it)
- New runner: `tasks/g1_pick_place/record_onboard_camera_episode.py`
- Raw results: `logs/phase5a_camera_smoke.json`

## A. Camera choice and configuration used for the smoke test

**The vendor G1 model has no dedicated head/neck body or joint.** Inspecting
`vendor/unitree_mujoco/unitree_robots/g1/g1_29dof.xml` directly: the
`head_link` mesh is just a static geom hung off `torso_link` (pos
`"0.0039635 0 -0.054"`, no separate body, no head joint). The camera is
therefore attached as a child of `torso_link` — the correct and only
sensible existing attachment point — rather than inventing a new body. It
moves with the robot's own kinematics (in this project's current
fixed-base configuration, `torso_link` is welded to the world, so the
camera's world pose is rigid; the *mechanism* — reading `data.cam_xpos`/
`data.cam_xmat` each step — generalizes unchanged to a future non-fixed
torso).

The existing third-person evidence camera used throughout Phase 4C-4F
(`_make_full_camera()`/`_make_diagnostic_camera()` in the various
`record_*_episode.py` scripts) is a free-floating `mujoco.MjvCamera`
constructed at render time in Python — it has **no** `<camera>` element in
the model at all. The onboard camera added here is a genuinely different
object: a real MJCF element rigidly attached to a body. That existing
camera is untouched by this phase.

**Configuration used for the smoke-test episode (a documented decision, not
a silent substitution):** Task 1's most recently authorized configuration
is Phase 4F's orientation-constrained IK (`write_grasp_scene_4f()`,
`use_oriented_ik=True`). Re-running it here (with this phase's camera
attached) reproduces its own already-logged behavior exactly:
`failure_state: "SETTLE_LOWER"` — it does not reach `OPEN` or
`VERIFY_TASK_SUCCESS`. HANDOFF.md's Phase 5A requirement is onboard
visibility evidence at **both** of those states, which is impossible to
obtain from a run that never reaches them without altering the trial. This
smoke test therefore uses Phase 4B/4C's original configuration
(`write_grasp_scene_4b()`, `use_oriented_ik=False`, the default) — the
configuration that actually completes the full state machine
(`task_pass=True`, confirmed unregressed through every later phase's own
reruns). Both configurations share the identical cube, target, gripper,
and arm-gain constants; only Phase 4F's optional orientation secondary
objective differs, and that objective was itself an unsuccessful attempt
at reducing slip (see `reports/phase4f-orientation-grasp-stabilization.md`),
not a defining part of "Task 1". This choice retunes nothing — it only
selects which of two already-existing, already-tested configurations this
camera smoke test observes.

No depth or segmentation buffer is added; `mujoco.Renderer(...)` is used in
its default RGB mode.

## B. Camera specification

| Property | Value |
| --- | --- |
| Parent body | `torso_link` |
| Local `pos` (in `torso_link`'s frame) | `(0.11396, 0.0, 0.34300)` m |
| Local `quat` (w,x,y,z) | `(-0.44421, -0.09965, 0.19489, 0.86877)` |
| World position at reset (`data.cam_xpos`) | `(0.11, 0.0, 1.19)` m |
| World rotation at reset (`data.cam_xmat`, row-major 3x3) | `[[-0.5855, 0.7330, -0.3463], [-0.8107, -0.5294, 0.2501], [0.0, 0.4272, 0.9042]]` |
| Field of view | `fovy = 90 deg` (vertical); derived `fovx ≈ 106.26 deg` at this aspect ratio |
| Resolution | 160 x 120 (conservative, per HANDOFF.md's suggested range) |
| Near / far clipping | `model.vis.map.znear=0.01`, `zfar=50.0` — MuJoCo expresses these as **multiples of `model.stat.extent`** (`2.372` m here), giving actual clipping distances of `≈0.0237 m` (near) and `≈118.6 m` (far) |
| RGB dtype / channel order | `uint8`, HWC, RGB (`mujoco.Renderer.render()` default) |
| World/camera convention | MuJoCo convention: the camera looks down its own local **-Z** axis; local **+Y** is image-up; local **+X** is image-right |
| Intrinsics (pinhole) | `fx = fy = 60.0` px, `cx = 80.0`, `cy = 60.0` (derived: `fy = height / (2 tan(fovy/2))`; `fx=fy` under MuJoCo's square-pixel assumption; `fovx` then follows from the aspect ratio at that focal length) |

Intrinsic matrix:

```
[[60.0,  0.0, 80.0],
 [ 0.0, 60.0, 60.0],
 [ 0.0,  0.0,  1.0]]
```

**Pose derivation (documented, not guessed):** the `head_link` mesh's own
local-frame bounding box, computed directly from
`vendor/unitree_mujoco/unitree_robots/g1/meshes/head_link.STL`, is
`x ∈ [-0.066, 0.074]`, `y ∈ [-0.078, 0.078]`, `z ∈ [0.325, 0.530]`.
Composed with the vendor MJCF's own geom offset for that mesh (`pos =
"0.0039635 0 -0.054"` in `torso_link`) and `torso_link`'s own world pose at
reset under this project's fixed pelvis+torso weld
(`(-0.0039635, 0, 0.847)`, identity orientation — confirmed by direct
query), the head mesh occupies world `z ∈ [1.118, 1.323]`. The first
iteration (`cam_pos_world = (0.02, 0, 1.20)`) placed the camera **inside**
that mesh volume and produced heavy visual self-occlusion by the robot's
own head geometry (see Section C). The final pose moves the camera forward
of the mesh's own local-x extent (`x=0.11 > x_max=0.074`) so the lens looks
out from in front of the face rather than from inside the head.

## C. Visibility — iteration record

**Iteration 1** (`cam_pos_world=(0.02,0,1.20)`, looking at the cube/target
midpoint): rendered frame was dominated by the robot's own head mesh
geometry filling the bottom of frame (camera embedded inside the mesh's own
bounding volume — see `/tmp` scratch renders from this session, not
committed since they were purely diagnostic). Cube/target technically
produced nonzero color-mask pixels but the composition was poor and the
placement was not evidence-driven yet.

Root cause found: a `<camera>` child's `pos` attribute is expressed in the
**parent body's local frame**, not world — an initial attempt at
`pos="0.02 0 1.20"` was interpreted as 1.20 m *above `torso_link`'s own
origin* (which is already at world z=0.847), placing the camera at world
z≈2.05 m, looking almost straight down at the whole scene from well above
the robot's actual head height. Fixed by explicitly converting a
world-frame-designed position into the parent frame
(`cam_pos_local = cam_pos_world - torso_link_world_pos`, valid here because
`torso_link` has identity orientation at reset).

**Iteration 2** (`cam_pos_world=(0.02,0,1.20)` corrected to the right
frame): camera now at the intended world height, but still positioned
*inside* the head mesh's own local-x span (`x=0.02 < x_max=0.074`),
producing visible self-occlusion by the head geometry in the lower part of
frame.

**Iteration 3 (final)**: `cam_pos_world=(0.11, 0.0, 1.19)`, moved forward of
the head mesh's own front face, looking at
`target_world=(0.29, -0.13, 0.72)` (workspace midpoint at table height).
Verified across the **full episode**, not just the reset frame (per
HANDOFF.md's explicit instruction) — see Section D.

## D. Smoke-test results (fresh simulation, this session)

Ran a fresh nominal Task 1 episode (`write_grasp_scene_4b()`,
`use_oriented_ik=False`) and captured onboard frames at all 10 required
checkpoints plus first bilateral contact:

| Phase | Red (cube) px | Blue (target) px | Notes |
| --- | --- | --- | --- |
| RESET | 1061 | 28 | |
| PREGRASP | 1061 | 28 | |
| APPROACH | 1031 | 27 | |
| CLOSE | 1050 | 18 | |
| FIRST_BILATERAL_CONTACT | 1072 | 18 | |
| LIFT | 1076 | 18 | |
| HOLD | 1034 | 23 | |
| TRANSPORT_ABOVE_TARGET | 1020 | 23 | |
| LOWER_TO_TARGET | 1448 | 10 | |
| OPEN | 1584 | 2 | target mostly covered by the settled cube — expected |
| VERIFY_TASK_SUCCESS | 1449 | 6 | target mostly covered by the settled cube — expected |

**Every task phase required by HANDOFF.md Section C is visible**: red cube
present (nonzero masked pixels, confirmed >0 at all 11 checkpoints incl.
`RESET` and the first-contact event) and the blue target present at every
required pre-placement checkpoint. The blue pixel count drops sharply at
`OPEN`/`VERIFY_TASK_SUCCESS` — this is the **physically correct** result of
a successful placement (the cube now sits on top of the pad, occluding
most of it from this viewing angle), not a camera or detection failure;
color-mask checks are smoke-test diagnostics only, never task-success
logic, per HANDOFF.md Section D.

Color masks were calibrated against actually-sampled pixel values in this
scene (not assumed): the rendered cube color, the rendered target-pad
color, the tan table color (`(143, 105, 69)`), and MuJoCo's own default sky
color (`(92, 137, 184)` — itself a medium blue, which a naive "is it blue"
threshold would have wrongly counted as target; the final mask requires
`b>150, b-r>110, r<60` specifically to reject the sky).

Other checks, all against a fresh, freshly-simulated run (`tests/test_phase5a_onboard_camera.py`
re-simulates; it does not read the log as ground truth):

| Check | Result |
| --- | --- |
| Frame shape/dtype | `(120, 160, 3)`, `uint8` at every checkpoint |
| Finite, in `[0,255]` | true at every checkpoint |
| Frames not blank | true (`std() > 1.0` at every checkpoint) |
| Temporal variance nonzero (video-frame sequence, 390 frames) | true |
| No two *temporally adjacent* frames identical | true, 0 of 389 consecutive pairs identical (mean abs diff range 0.011-5.78) |
| Cube red-pixel region present at required phases | true, all 11 |
| Target blue-pixel region present at required pre-placement phases | true, all 8 |
| Camera pose tracks parent body | true — world position constant to **0.1876 mm** across the whole 13.24s episode (see note below) |
| Rendering does not alter physics | true — `height_gain_m`, `task_pass`, `final_xy_target_error_m` bit-identical with vs. without the camera/render calls active |
| Nominal Task 1 still completes with the unchanged controller | true — `task_pass=True`, `height_gain_m=0.11747` (consistent with Phase 4B/4C's own established baseline) |

**Note on "camera pose changes consistently with its parent body":** an
earlier draft of this check used an exact-equality (`atol=1e-9`) comparison
and initially reported the pose as *not* constant. Investigating that
before accepting the number: the pelvis+torso weld used throughout this
project (`_build_grasp_tree`'s `pelvis_fixed`/`torso_fixed` equality
constraints) is a **soft**, finite-stiffness constraint, not an
infinitely-rigid joint — so `torso_link`, and therefore the camera rigidly
mounted on it, is expected to show a tiny nonzero solver-precision
deviation, not bit-exact zero motion. The measured max deviation is
**0.1876 mm** across the entire 13.24 s episode — two-plus orders of
magnitude below any dimension relevant to this task (cube half-extent
35 mm) — confirmed consistent with (i.e. negligibly moving with) its fixed
parent body, which is the physically correct result for a rigid head mount
on this project's current fixed-base configuration.

An earlier version of this same smoke-test script also had a self-inflicted
bug: it compared the semantically-unrelated named milestone frames (e.g.
`RESET` vs. `PREGRASP`, captured at very different simulation instants
chosen for what they *show*, not for being temporally adjacent) as if they
were a consecutive-frame sequence, which produced a spurious "identical
frames" result once (`RESET` and `PREGRASP`'s first captured instant
happened to be visually indistinguishable, since nothing had moved yet
between them) and a genuine duplicate-array bug once (the reset frame was
accidentally included twice in the comparison list). Fixed by measuring
temporal-variance / consecutive-frame-identity on the actual
`FRAME_STRIDE`-sampled video sequence (389 genuinely adjacent pairs), which
is what those checks are actually meant to test.

## E. Performance

Measured on this machine, one full nominal episode (13.244 s of simulated
time, 6622 physics steps at `dt=0.002`):

| Measurement | Value |
| --- | --- |
| Simulation-only wall-clock | 4.970 s (≈2.66x realtime) |
| Simulation + onboard rendering wall-clock (render every 17th step, ≈29.4 Hz) | 20.985 s (≈0.63x realtime — rendering dominates cost) |
| Achieved combined sim+render throughput | **18.54 fps** |
| Per-frame memory (raw `uint8` array) | 57,600 bytes (160×120×3) |

Estimated storage per 15-second episode, raw `uint8` vs. a conservative
~8x JPEG-style compression estimate:

| Rate | Raw (MB) | ~JPEG est. (MB) |
| --- | --- | --- |
| 5 Hz | 4.12 | 0.52 |
| 10 Hz | 8.24 | 1.03 |
| 20 Hz | 16.48 | 2.06 |
| 30 Hz | 24.72 | 3.09 |

**Recommendation: 10 Hz** for a future policy observation rate. Reasoning
tied to the measurement above, not a guess: the measured combined
sim+render throughput is ~18.5 fps. Recording at 10 Hz leaves close to 2x
headroom under that ceiling for a real-time onboard capture loop — margin
for compression, disk I/O, and additional cameras added later (e.g. a
wrist camera) without falling behind real time. 20 Hz would consume nearly
all of the measured budget and is realistically only safe for *offline*
(non-real-time) dataset generation, where rendering can be decoupled from
live physics stepping (exactly the mode a future dataset-collection pass
would run in anyway) — so 20 Hz remains a reasonable choice for that later
context, but 10 Hz is the safer default for anything closer to real time.

## F. Evidence produced

- `tasks/g1_pick_place/camera_observation.py` — camera pose/intrinsics/extrinsics helpers, `write_grasp_scene_5a()`, smoke-test-only color masks
- `tasks/g1_pick_place/record_onboard_camera_episode.py` — the runner that produced the video/frames/log below
- `tests/test_phase5a_onboard_camera.py` — 19 tests (synthetic pose/intrinsics math, scene-isolation checks, and a real fresh-simulation trial covering every item in Section D)
- `logs/phase5a_camera_smoke.json` — full raw results
- `artifacts/phase5a_head_camera.mp4` — onboard view only (no third-person substitution), 160x120, 29.41 fps, 390 frames, 13.26 s, 76,744 bytes, decode-verified, with a burned-in `state: <PHASE>` text overlay
- `artifacts/phase5a_head_camera_frames/` — 11 individual PNGs, one per required checkpoint (`RESET`, `PREGRASP`, `APPROACH`, `CLOSE`, `FIRST_BILATERAL_CONTACT`, `LIFT`, `HOLD`, `TRANSPORT_ABOVE_TARGET`, `LOWER_TO_TARGET`, `OPEN`, `VERIFY_TASK_SUCCESS`)

## G. Validation

Full repository suite: **178 tests, 0 unexpected failures** (159 pre-existing
unchanged + 19 new). Confirmed by diff:

- Zero diff on `tasks/g1_pick_place/controller.py`, `controller_3c.py`,
  `run_pick_place.py`, `gripper_scene.py`, and every existing
  `g1_grasp_scene_*.xml` output against commit `c9352ac`.
- Zero diff on every historical report and artifact (Phase 3 through
  Phase 4F human-acceptance-decision).
- Vendor pin unchanged at `4134cb5dc7ff1ba7f484deda48b5274b58694519`;
  `vendor/unitree_mujoco/unitree_robots/g1/g1_29dof.xml` does not contain
  the string `head_cam` anywhere (confirmed by a dedicated test).
- Excluded from staging: the pre-existing `logs/g1_mujoco_smoke.json`
  timestamp diff, the vendor submodule pointer, and a stray iteration
  scratch file (`g1_grasp_scene_5a_iter.xml`, deleted — not a deliverable).

## Limitations

- The onboard camera's world pose is only "consistent with its parent
  body" in the trivial sense of this project's current fixed-base
  configuration (torso welded to the world). Its real value — moving
  correctly with an *unwelded*, articulated torso/neck — is unverified,
  since no such configuration exists in this project yet.
- The color-mask visibility checks are calibrated to this scene's specific
  cube/target/table/sky colors and lighting; they are not general-purpose
  and are explicitly documented (in code and here) as smoke-test
  diagnostics only, never task-success logic.
- This smoke test uses Phase 4B/4C's configuration, not Phase 4F's
  currently-most-recent orientation-IK configuration, for the reason given
  in Section A. A future phase adding onboard-camera evidence to Phase 4F's
  configuration specifically (once/if its slip gap is closed) would need a
  fresh capture.
- This remains a fixed-base, torso-constrained upper-body manipulation
  baseline. No capability beyond adding one observation source is implied
  or claimed by this phase.
