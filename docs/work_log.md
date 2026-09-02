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
