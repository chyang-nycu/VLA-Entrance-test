# Work Log

## 2026-09-02

- Inspected host: macOS 12.7.6, Intel `x86_64`, 16 GB RAM, 326 GiB free on workspace volume, Xcode Command Line Tools at `/Library/Developer/CommandLineTools`.
- Confirmed tools: Homebrew 6.0.21, CMake 4.4.3, Apple Git 2.37.1, Python 3.14.7, Python 3.12.14.
- Reviewed official `unitree_mujoco` README from GitHub and the cloned repository. It documents a Linux-first C++ simulator path and a Python simulator path using `unitree_sdk2_python`, `mujoco`, and `pygame`; G1 uses the `unitree_hg` message type.
- Cloned official repository into `vendor/unitree_mujoco`.
- Pinned `unitree_mujoco` commit: `4134cb5dc7ff1ba7f484deda48b5274b58694519` (`2026-08-25T22:03:03+08:00`, merge pull request #129).
- Created project-local Python virtual environment at `.venv` using Python 3.12.14.
- Attempted unpinned `pip install mujoco`; it selected `mujoco-3.12.0` source distribution and failed because `MUJOCO_PATH` was not set.
- Installed pinned Python simulator dependencies: `mujoco==3.3.6`, `numpy==2.5.2`, `imageio==2.37.4`, `pillow==12.3.0`, plus MuJoCo transitive dependencies.
- Ran `setup/g1_mujoco_smoke.py` against official `unitree_robots/g1/scene.xml`.
- Verified model load, simulation advance, reset, joint state read, and small safe actuator command changing simulated joint state.
- Attempted offscreen screenshot capture. It failed with `CGLError('invalid CoreGraphics connection')`, indicating no valid macOS graphics context in the terminal session.
- Noted macOS case-insensitive checkout warning for Go2w `terrain.STL` and `terrain.stl`; G1 smoke test was unaffected.

## 2026-09-02 Phase 2

- Inspected pinned G1 model at `4134cb5dc7ff1ba7f484deda48b5274b58694519`.
- Generated `logs/g1_actuators.json` with all 29 actuators, controlled joints, control ranges, joint ranges, and associated bodies.
- Audited G1 variants: `scene.xml`, `scene_23dof.xml`, `scene_29dof.xml`, `g1_23dof.xml`, and `g1_29dof.xml`.
- Confirmed no pinned G1 XML contains an actuated gripper, Inspire hand, or Dex3 hand. G1 hand/finger meshes exist as assets, but compiled variants do not expose actuated finger/gripper joints.
- Ran task-local contact experiment in `tasks/g1_pick_place/g1_contact_probe_scene.xml`; MuJoCo reported contact between `probe_cube_geom` and `right_wrist_yaw_link`.
- Ran free-standing stability probe; pelvis translated `0.9305553713743192` m in 2 seconds under small arm torques, supporting a fixed-base MVP.
- Added automated tests in `tests/test_phase2_g1_audit.py`; `unittest` passed 4 tests.

## 2026-09-02 Repository hygiene

- No `HANDOFF.md` existed anywhere in the workspace despite prior expectations that one did; regenerated it from actually-verified state plus the Phase 3 spec given directly by the project owner. Added top-level `README.md`.
- Added `.gitignore` (`.venv/`, `__pycache__/`, `*.pyc`, generated videos/datasets, temp MuJoCo outputs, `.DS_Store`).
- Registered `vendor/unitree_mujoco` as a git submodule (manual gitlink + `.gitmodules`, no re-clone) pinned at `4134cb5dc7ff1ba7f484deda48b5274b58694519`; existing checkout and its pre-existing `terrain.STL` case-collision artifact left untouched.
- Re-ran Phase 1 and Phase 2 tests (all pass) and made a local checkpoint commit (`dd2bc9b`), no push.

## 2026-09-02 Phase 3

- Built `tasks/g1_pick_place/gripper_scene.py`: task-local derived MJCF (ElementTree deep-copy of vendor `g1_29dof.xml`) adding a pelvis-fixing equality weld, a symmetric parallel-jaw gripper under `right_wrist_yaw_link` (two slide-joint fingers, motor actuators, collision pads, `grasp_tcp` site), a static table, and a free-jointed cube. Reachable workspace confirmed via real forward-kinematics sampling (wrist reaches `[0.231,-0.155,0.804]` at a representative arm pose).
- Built `tasks/g1_pick_place/controller.py`: DLS Cartesian IK on a scratch `MjData` (explicit joint/qpos/qvel/actuator index maps), bounded joint-space PD with gravity/Coriolis feedforward and finite-output enforcement. Verified IK converges to sub-mm/few-mm accuracy on realistic waypoints.
- Built `tasks/g1_pick_place/run_grasp_test.py`: pregrasp -> approach -> close -> lift -> hold -> lower -> open sequence, all 5 HANDOFF.md acceptance criteria checked directly against simulation telemetry; variant sweep gated on nominal success.
- Ran 3 documented tuning iterations on the nominal grasp trial (see `reports/phase3-grasping-baseline.md` for full detail): (1) baseline gains, no gravity comp — arm never reached the target, no contact; (2) added gravity/Coriolis feedforward + higher gains — contact achieved but no real grip/lift; (3) added finger overtravel/squeeze margin + trajectory-continuity fix — contact still one-sided, cube shoved out of position during approach before the gripper closes. Diagnosed root cause as multi-joint tracking oscillation from a single PD gain pair applied across joints with 5x differing torque limits; not fixed within the 3-iteration budget.
- Per `HANDOFF.md`'s tuning budget, stopped after iteration 3 rather than attempting a 4th or introducing a scripted grasp constraint. Nominal trial result: FAIL (height gain 0.005 m of required 0.08 m; 0.0 s of required 2.0 s continuous lift). 5-position-variant evaluation not run (gated on nominal success).
- Added `tests/test_phase3_gripper.py` (7 tests, pass), `tests/test_phase3_controller.py` (7 tests, pass), `tests/test_phase3_grasp.py` (6 tests, 3 fail honestly against the real acceptance criteria — not weakened). Wrote `logs/phase3_grasp_trials.json` and `reports/phase3-grasping-baseline.md`.
