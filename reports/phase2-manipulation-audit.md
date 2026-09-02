# Phase 2 Manipulation Feasibility Audit

Date: 2026-09-02

## Scope

- Repository: `vendor/unitree_mujoco`
- Required commit: `4134cb5dc7ff1ba7f484deda48b5274b58694519`
- Robot: Unitree G1
- Stock scene inspected: `vendor/unitree_mujoco/unitree_robots/g1/scene.xml`
- Direct model inspected: `vendor/unitree_mujoco/unitree_robots/g1/g1_29dof.xml`
- Generated experiments: `tasks/g1_pick_place/`
- Logs: `logs/g1_actuators.json`, `logs/g1_contacts.json`

## Decision Summary

- Selected embodiment: official G1 29-actuator body/arm model from the pinned `unitree_mujoco` repository, with task-local manipulation additions.
- Selected grasp mechanism: B, add a simple documented parallel gripper to the official right wrist in task-local XML.
- Selected base constraint: fixed-pelvis / fixed-base upper-body manipulation for the first MVP.
- Fallback C: scripted grasp constraint is not selected now. Use it only if the simple gripper cannot produce stable grasp contacts after implementation and tuning.

## Actuator Inventory

| # | Actuator | Joint | Ctrl range | Joint range | Body |
| --- | --- | --- | --- | --- | --- |
| 0 | `left_hip_pitch` | `left_hip_pitch_joint` | `[-88.0, 88.0]` | `[-2.5307, 2.8798]` | `left_hip_pitch_link` |
| 1 | `left_hip_roll` | `left_hip_roll_joint` | `[-88.0, 88.0]` | `[-0.5236, 2.9671]` | `left_hip_roll_link` |
| 2 | `left_hip_yaw` | `left_hip_yaw_joint` | `[-88.0, 88.0]` | `[-2.7576, 2.7576]` | `left_hip_yaw_link` |
| 3 | `left_knee` | `left_knee_joint` | `[-139.0, 139.0]` | `[-0.087267, 2.8798]` | `left_knee_link` |
| 4 | `left_ankle_pitch` | `left_ankle_pitch_joint` | `[-50.0, 50.0]` | `[-0.87267, 0.5236]` | `left_ankle_pitch_link` |
| 5 | `left_ankle_roll` | `left_ankle_roll_joint` | `[-50.0, 50.0]` | `[-0.2618, 0.2618]` | `left_ankle_roll_link` |
| 6 | `right_hip_pitch` | `right_hip_pitch_joint` | `[-88.0, 88.0]` | `[-2.5307, 2.8798]` | `right_hip_pitch_link` |
| 7 | `right_hip_roll` | `right_hip_roll_joint` | `[-88.0, 88.0]` | `[-2.9671, 0.5236]` | `right_hip_roll_link` |
| 8 | `right_hip_yaw` | `right_hip_yaw_joint` | `[-88.0, 88.0]` | `[-2.7576, 2.7576]` | `right_hip_yaw_link` |
| 9 | `right_knee` | `right_knee_joint` | `[-139.0, 139.0]` | `[-0.087267, 2.8798]` | `right_knee_link` |
| 10 | `right_ankle_pitch` | `right_ankle_pitch_joint` | `[-50.0, 50.0]` | `[-0.87267, 0.5236]` | `right_ankle_pitch_link` |
| 11 | `right_ankle_roll` | `right_ankle_roll_joint` | `[-50.0, 50.0]` | `[-0.2618, 0.2618]` | `right_ankle_roll_link` |
| 12 | `waist_yaw` | `waist_yaw_joint` | `[-88.0, 88.0]` | `[-2.618, 2.618]` | `waist_yaw_link` |
| 13 | `waist_roll` | `waist_roll_joint` | `[-50.0, 50.0]` | `[-0.52, 0.52]` | `waist_roll_link` |
| 14 | `waist_pitch` | `waist_pitch_joint` | `[-50.0, 50.0]` | `[-0.52, 0.52]` | `torso_link` |
| 15 | `left_shoulder_pitch` | `left_shoulder_pitch_joint` | `[-25.0, 25.0]` | `[-3.0892, 2.6704]` | `left_shoulder_pitch_link` |
| 16 | `left_shoulder_roll` | `left_shoulder_roll_joint` | `[-25.0, 25.0]` | `[-1.5882, 2.2515]` | `left_shoulder_roll_link` |
| 17 | `left_shoulder_yaw` | `left_shoulder_yaw_joint` | `[-25.0, 25.0]` | `[-2.618, 2.618]` | `left_shoulder_yaw_link` |
| 18 | `left_elbow` | `left_elbow_joint` | `[-25.0, 25.0]` | `[-1.0472, 2.0944]` | `left_elbow_link` |
| 19 | `left_wrist_roll` | `left_wrist_roll_joint` | `[-25.0, 25.0]` | `[-1.97222, 1.97222]` | `left_wrist_roll_link` |
| 20 | `left_wrist_pitch` | `left_wrist_pitch_joint` | `[-5.0, 5.0]` | `[-1.61443, 1.61443]` | `left_wrist_pitch_link` |
| 21 | `left_wrist_yaw` | `left_wrist_yaw_joint` | `[-5.0, 5.0]` | `[-1.61443, 1.61443]` | `left_wrist_yaw_link` |
| 22 | `right_shoulder_pitch` | `right_shoulder_pitch_joint` | `[-25.0, 25.0]` | `[-3.0892, 2.6704]` | `right_shoulder_pitch_link` |
| 23 | `right_shoulder_roll` | `right_shoulder_roll_joint` | `[-25.0, 25.0]` | `[-2.2515, 1.5882]` | `right_shoulder_roll_link` |
| 24 | `right_shoulder_yaw` | `right_shoulder_yaw_joint` | `[-25.0, 25.0]` | `[-2.618, 2.618]` | `right_shoulder_yaw_link` |
| 25 | `right_elbow` | `right_elbow_joint` | `[-25.0, 25.0]` | `[-1.0472, 2.0944]` | `right_elbow_link` |
| 26 | `right_wrist_roll` | `right_wrist_roll_joint` | `[-25.0, 25.0]` | `[-1.97222, 1.97222]` | `right_wrist_roll_link` |
| 27 | `right_wrist_pitch` | `right_wrist_pitch_joint` | `[-5.0, 5.0]` | `[-1.61443, 1.61443]` | `right_wrist_pitch_link` |
| 28 | `right_wrist_yaw` | `right_wrist_yaw_joint` | `[-5.0, 5.0]` | `[-1.61443, 1.61443]` | `right_wrist_yaw_link` |

## End Effectors, Hands, Sites, Contacts

Right wrist/end-effector body:

- `right_wrist_yaw_link`; upstream bodies are `right_wrist_pitch_link`, `right_wrist_roll_link`, `right_elbow_link`, and shoulder links.
- Contact-capable wrist geoms exist on `right_wrist_roll_link`, `right_wrist_pitch_link`, and `right_wrist_yaw_link`.
- The right rubber hand mesh in `g1_29dof.xml` is visual-only: `contype="0"` and `conaffinity="0"`.

Left wrist/end-effector body:

- `left_wrist_yaw_link`; upstream bodies mirror the right side.
- Contact-capable wrist geoms exist, but the left rubber hand mesh is also visual-only in the 29-DoF model.

Hand/finger finding:

- The G1 mesh directory contains palm, thumb, index, and middle finger meshes.
- The compiled stock G1 XML variants do not expose actuated finger or gripper joints.
- `g1_23dof.xml` and `scene_23dof.xml` include rubber-hand bodies named `left_wrist_roll_rubber_hand` and `right_wrist_roll_rubber_hand`, but they are not actuated dexterous hands or parallel grippers.
- No G1 XML file in this pinned repository mentions `gripper`, `Inspire`, `inspire`, `Dex3`, or `dex3`.

Existing sites:

- `imu` on `pelvis`
- `secondary_imu` on `waist_roll_link`

Neither stock site is suitable as an IK target or mocap target for wrist manipulation. A task-local probe scene adds `right_wrist_tcp_probe` on `right_wrist_yaw_link` to verify the intended TCP anchor without touching vendor files.

## G1 Variant Audit

| File | Compiles | Actuators | Bodies | Geoms | Sites | Hand bodies | Gripper/Inspire/Dex3 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `g1_23dof.xml` | yes | 29 | 31 | 62 | 2 | `left_wrist_roll_rubber_hand`, `right_wrist_roll_rubber_hand` | no |
| `g1_29dof.xml` | yes | 29 | 31 | 73 | 2 | none | no |
| `scene.xml` | yes | 29 | 31 | 179 | 2 | none | no |
| `scene_23dof.xml` | yes | 29 | 31 | 63 | 2 | `left_wrist_roll_rubber_hand`, `right_wrist_roll_rubber_hand` | no |
| `scene_29dof.xml` | yes | 29 | 31 | 74 | 2 | none | no |

## Contact Test

Experiment file: `tasks/g1_pick_place/g1_contact_probe_scene.xml`

Method:

- Created a task-local copy of the official G1 XML structure with absolute mesh paths.
- Added a static cube body named `probe_cube` at `[0.30, -0.12, 0.84]`.
- Did not attach, weld, or teleport the cube.
- Moved the right arm slowly with a fixed-pelvis kinematic sweep from neutral toward:
  - `right_shoulder_pitch_joint = -0.5`
  - `right_elbow_joint = 0.8`
- Called MuJoCo forward dynamics/collision at each sweep step and inspected `data.contact`.

Result:

- Contact detected: yes
- Start state already in contact: no
- First contact step: 26 of 100
- Contact pair: `probe_cube_geom` with right wrist collision geom on `right_wrist_yaw_link`
- Contact distance: `-7.467504631023608e-05`
- Contact position: `[0.26503718485646977, -0.15499867044003926, 0.8749969001040352]`

## Grasp Feasibility

The stock G1 model can touch objects with wrist collision geometry, but it cannot physically form a stable grasp by itself. The 29-actuator stock model has arm and wrist actuators only. It lacks opposing finger bodies, finger joints, gripper slide joints, tendon coupling, or actuators that could close around a cube.

Implementation A is not technically honest for this pinned repository because no official G1 hand-equipped XML with actuated gripper, Inspire hand, or Dex3 hand is present.

Implementation B is technically honest and preferred: add a small documented parallel gripper under `right_wrist_yaw_link` in `tasks/g1_pick_place/`, with explicit geometry, joints, actuator limits, friction, and a TCP site. This preserves the official vendor model while making grasp mechanics physical and inspectable.

Implementation C should remain a fallback only. A scripted grasp constraint would be acceptable only if clearly disclosed and only after a task-local gripper cannot achieve stable contact within the time budget.

## Base Constraint Choice

Selected first MVP: fixed-pelvis / fixed-base upper-body manipulation.

Evidence:

- Free-standing probe ran the official G1 `scene.xml` for 2 seconds with small constant arm torques:
  - `right_shoulder_pitch = 0.2`
  - `right_elbow = 0.2`
  - `left_shoulder_pitch = 0.2`
- Pelvis moved from `[0.0, 0.0, 0.793]` to `[0.7295867926421992, -0.0010534519845503123, 0.21538827796297097]`.
- Translation norm was `0.9305553713743192`.

This is not stable enough for a first manipulation MVP. Fixed-base manipulation is the honest starting point; free-standing balance/control can be added as a later phase.

## Automated Tests

Command executed:

```bash
.venv/bin/python -m unittest tests/test_phase2_g1_audit.py
```

Result:

```text
Ran 4 tests in 6.001s
OK
```

The tests verify:

- `right_wrist_yaw_link` exists in the stock model.
- Task-local `right_wrist_tcp_probe` exists in the generated probe scene.
- Expected right-arm actuators exist.
- Cube contact can be detected and does not exist at the start of the sweep.
- Vendor G1 files remain unchanged at the pinned commit.

## Blockers Before Task 1

- Implement the task-local parallel gripper XML and choose conservative geometry/friction/contact parameters.
- Add a real wrist TCP site to the task-local gripper model, not the vendor model.
- Decide whether Task 1 uses kinematic IK commands first or low-gain position/torque control.
- Add renderer/screenshot validation from a normal macOS GUI session if visual evidence is required.
- Do not proceed to scripted grasp unless the physical gripper approach fails and the limitation is documented.
