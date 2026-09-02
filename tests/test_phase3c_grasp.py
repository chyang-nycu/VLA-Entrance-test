#!/usr/bin/env python3
"""Phase 3C tests: position-servo architecture structural checks, IK/
reachability sanity, the initialization-boundary guard (independent audit
of run_grasp_test_3c.py, not just run_grasp_test.py), and the nominal
grasp-and-lift acceptance test for the new architecture.

Does not modify or re-run Phase 3/3B's own tests (tests/test_phase3_*.py) --
those remain the untouched historical record of the torque-PD architecture's
failure.
"""

from __future__ import annotations

import subprocess
import unittest

import mujoco
import numpy as np

from tasks.g1_pick_place import gripper_scene
from tasks.g1_pick_place.controller import JointMap, RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE
from tasks.g1_pick_place.controller_3c import IK_POS_TOL, solve_ik_waypoint
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard
from tasks.g1_pick_place.run_grasp_test_3c import (
    GRIPPER_KD_3C,
    GRIPPER_KP_3C,
    diagnose_reachability,
    run_trial_3c,
)

ARM_KP_3C = 400.0
ARM_KV_3C = 25.0


class Phase3CGripperSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gripper_scene.write_grasp_scene_3c(
            arm_kp=ARM_KP_3C, arm_kv=ARM_KV_3C, scene_name="g1_grasp_scene_3c.xml"
        )
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))

    def test_right_arm_actuators_are_position_type_with_force_limits(self) -> None:
        for joint_name, actuator_name, force_limit in gripper_scene.RIGHT_ARM_JOINT_ACTUATOR_PAIRS:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            self.assertGreaterEqual(aid, 0)
            self.assertEqual(int(self.model.actuator_biastype[aid]), 1)  # affine (position servo)
            self.assertTrue(bool(self.model.actuator_forcelimited[aid]))
            frange = self.model.actuator_forcerange[aid]
            self.assertAlmostEqual(float(frange[1]), force_limit)
            self.assertAlmostEqual(float(frange[0]), -force_limit)
            gainprm = self.model.actuator_gainprm[aid]
            biasprm = self.model.actuator_biasprm[aid]
            self.assertTrue(np.isfinite(gainprm[0]) and gainprm[0] > 0)
            self.assertTrue(np.isfinite(biasprm[1]) and np.isfinite(biasprm[2]))

    def test_implicitfast_integrator_set(self) -> None:
        self.assertEqual(int(self.model.opt.integrator), 3)  # implicitfast

    def test_pelvis_and_torso_both_welded(self) -> None:
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            for i in range(self.model.neq)
        ]
        self.assertIn("pelvis_fixed", names)
        self.assertIn("torso_fixed", names)

    def test_gripper_unchanged_from_phase3(self) -> None:
        left_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_finger")
        right_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_finger")
        self.assertGreaterEqual(left_id, 0)
        self.assertGreaterEqual(right_id, 0)
        self.assertEqual(int(self.model.actuator_biastype[left_id]), 0)  # still pure motor

    def test_vendor_g1_files_remain_unchanged(self) -> None:
        head = subprocess.check_output(
            ["git", "-C", str(gripper_scene.VENDOR), "rev-parse", "HEAD"], text=True
        ).strip()
        self.assertEqual(head, "4134cb5dc7ff1ba7f484deda48b5274b58694519")
        status = subprocess.check_output(
            ["git", "-C", str(gripper_scene.VENDOR), "status", "--short", "--", "unitree_robots/g1"],
            text=True,
        ).strip()
        self.assertEqual(status, "")

    def test_historical_write_grasp_scene_unaffected(self) -> None:
        # Phase 3/3B's scene generator must produce byte-identical output
        # regardless of the Phase 3C additions to gripper_scene.py.
        import hashlib

        p = gripper_scene.write_grasp_scene()
        digest = hashlib.sha256(p.read_bytes()).hexdigest()
        self.assertEqual(digest, "1b2fd577ac5cf9baa45bdbf656c19313899168c7bc14f3cc36ded91292b767a6")


class Phase3CReachabilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gripper_scene.write_grasp_scene_3c(
            arm_kp=ARM_KP_3C, arm_kv=ARM_KV_3C, scene_name="g1_grasp_scene_3c.xml"
        )
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))

    def test_all_four_waypoints_reachable_within_tolerance(self) -> None:
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        arm_map = JointMap.build(self.model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        report = diagnose_reachability(
            self.model, arm_map, site_id, data.qpos.copy(), np.array(gripper_scene.CUBE_POS)
        )
        self.assertTrue(report["all_reachable"], msg=str(report))
        for name in ("PREGRASP", "APPROACH", "CLOSED_LIFT", "HOLD"):
            self.assertLess(report[name]["residual_m"], IK_POS_TOL)

    def test_ik_solution_respects_joint_range(self) -> None:
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        arm_map = JointMap.build(self.model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        scratch = mujoco.MjData(self.model)
        nominal_q = np.zeros(len(RIGHT_ARM_JOINTS))
        target = np.array(gripper_scene.CUBE_POS) + np.array([0.0, 0.0, 0.10])
        q, resid, _ = solve_ik_waypoint(self.model, scratch, data.qpos.copy(), arm_map, site_id, target, nominal_q)
        self.assertTrue(np.all(q >= arm_map.jnt_range[:, 0] - 1e-9))
        self.assertTrue(np.all(q <= arm_map.jnt_range[:, 1] + 1e-9))
        self.assertTrue(np.all(np.isfinite(q)))


class CubeInitGuard3CBoundaryTest(unittest.TestCase):
    """Independent audit that run_grasp_test_3c.py's run_trial_3c also
    respects the initialization boundary -- CubeInitGuard is shared code
    with Phase 3/3B, but its *usage* here is a separate integration that
    must be checked on its own, not assumed correct by association."""

    def test_run_trial_3c_source_has_no_direct_cube_state_write(self) -> None:
        from tasks.g1_pick_place.run_grasp_test_3c import (
            _assert_run_trial_3c_has_no_direct_cube_state_write,
        )

        _assert_run_trial_3c_has_no_direct_cube_state_write()  # must not raise

    def test_guard_raises_after_lock_shared_class(self) -> None:
        scene_path = gripper_scene.write_grasp_scene_3c(
            arm_kp=ARM_KP_3C, arm_kv=ARM_KV_3C, scene_name="g1_grasp_scene_3c.xml"
        )
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
        guard = CubeInitGuard(data, int(model.jnt_qposadr[cube_joint_id]), int(model.jnt_dofadr[cube_joint_id]))
        guard.set_initial_pose([0.3, -0.1, 0.8])  # pre-lock: allowed
        guard.lock()
        with self.assertRaises(RuntimeError):
            guard.set_initial_pose([0.0, 0.0, 1.0])  # post-lock: must raise
        with self.assertRaises(RuntimeError):
            guard.set_initial_velocity()


class Phase3CNominalGraspTest(unittest.TestCase):
    """The new-architecture nominal acceptance test. Unlike
    tests/test_phase3_grasp.py (Phase 3/3B, torque-PD, documented FAIL),
    this is expected to PASS -- Attempt 3C-2 (position servos with
    evidence-tuned gripper gains) meets all 5 HANDOFF.md criteria."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gripper_scene.write_grasp_scene_3c(
            arm_kp=ARM_KP_3C, arm_kv=ARM_KV_3C, scene_name="g1_grasp_scene_3c.xml"
        )
        cls.result = run_trial_3c(cls.scene_path, gripper_kp=GRIPPER_KP_3C, gripper_kd=GRIPPER_KD_3C)

    def test_no_early_failure_state(self) -> None:
        self.assertIsNone(self.result["failure_state"], msg=self.result["failure_reason"])

    def test_both_pads_contact_cube(self) -> None:
        self.assertTrue(self.result["criteria"]["both_pads_contact_cube"])

    def test_height_gain_at_least_8cm(self) -> None:
        self.assertGreaterEqual(self.result["height_gain_m"], 0.08)

    def test_lifted_off_table_at_least_2s_continuous(self) -> None:
        self.assertGreaterEqual(self.result["max_continuous_lifted_s"], 2.0)

    def test_controller_outputs_finite_and_bounded(self) -> None:
        self.assertTrue(self.result["criteria"]["finite_and_bounded"])

    def test_cube_released_after_open(self) -> None:
        self.assertTrue(self.result["criteria"]["released_after_open"])

    def test_nominal_trial_overall(self) -> None:
        self.assertTrue(self.result["pass"], msg=f"criteria: {self.result['criteria']}")

    def test_deterministic_across_five_reruns(self) -> None:
        results = [
            run_trial_3c(self.scene_path, gripper_kp=GRIPPER_KP_3C, gripper_kd=GRIPPER_KD_3C)
            for _ in range(5)
        ]
        self.assertTrue(all(r["pass"] for r in results), msg=[r["criteria"] for r in results])
        heights = [r["height_gain_m"] for r in results]
        self.assertTrue(all(h == heights[0] for h in heights), msg=heights)


if __name__ == "__main__":
    unittest.main()
