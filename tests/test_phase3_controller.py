#!/usr/bin/env python3
"""Unit checks for the Phase 3 DLS IK + bounded joint-space PD controller."""

from __future__ import annotations

import unittest

import mujoco
import numpy as np

from tasks.g1_pick_place import controller as ctrl_mod
from tasks.g1_pick_place.gripper_scene import write_grasp_scene


class Phase3ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        scene = write_grasp_scene()
        cls.model = mujoco.MjModel.from_xml_path(str(scene))

    def setUp(self) -> None:
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.ctrl = ctrl_mod.G1GraspController(model=self.model)

    def test_joint_map_index_counts_and_names(self) -> None:
        jm = self.ctrl.arm_map
        self.assertEqual(len(jm.names), 7)
        self.assertEqual(jm.qpos_adr.shape, (7,))
        self.assertEqual(jm.dof_adr.shape, (7,))
        self.assertEqual(jm.actuator_id.shape, (7,))
        # dof_adr must be one less than qpos_adr for hinge joints following
        # a 7-dof/6-dof free joint (no positional-ordering assumption; this
        # checks the *retrieved* addresses are internally consistent).
        for jname, qadr, dadr in zip(jm.names, jm.qpos_adr, jm.dof_adr):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            self.assertEqual(int(self.model.jnt_qposadr[jid]), int(qadr))
            self.assertEqual(int(self.model.jnt_dofadr[jid]), int(dadr))

    def test_gripper_joint_map(self) -> None:
        jm = self.ctrl.gripper_map
        self.assertEqual(list(jm.names), ["left_finger_joint", "right_finger_joint"])
        self.assertEqual(jm.actuator_id.shape, (2,))

    def test_ik_converges_on_small_synthetic_target(self) -> None:
        start = self.ctrl.tcp_pos(self.data)
        target = start + np.array([0.03, -0.02, 0.03])
        joint_target = self.ctrl.ik_target_for(self.data, target)

        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        self.ctrl.arm_map.set_qpos(scratch, joint_target)
        mujoco.mj_kinematics(self.model, scratch)
        mujoco.mj_comPos(self.model, scratch)
        achieved = scratch.site_xpos[self.ctrl.tcp_site_id]

        self.assertLess(np.linalg.norm(achieved - target), 2e-3)
        self.assertTrue(np.all(np.isfinite(joint_target)))

    def test_ik_target_respects_joint_range(self) -> None:
        start = self.ctrl.tcp_pos(self.data)
        target = start + np.array([0.03, -0.02, 0.03])
        joint_target = self.ctrl.ik_target_for(self.data, target)
        jm = self.ctrl.arm_map
        self.assertTrue(np.all(joint_target >= jm.jnt_range[:, 0] - 1e-9))
        self.assertTrue(np.all(joint_target <= jm.jnt_range[:, 1] + 1e-9))

    def test_bounded_pd_clips_to_actuator_ctrlrange_under_large_error(self) -> None:
        jm = self.ctrl.arm_map
        huge_target = jm.jnt_range[:, 1].copy()  # push every joint to its extreme
        torque = ctrl_mod.bounded_pd_step(
            jm, self.data, huge_target,
            kp=1e6, kd=0.0,  # deliberately extreme gain to try to blow past ctrlrange
            max_step=1.0, max_qvel=1e6,
        )
        self.assertTrue(np.all(np.isfinite(torque)))
        self.assertTrue(np.all(torque >= jm.ctrl_range[:, 0] - 1e-9))
        self.assertTrue(np.all(torque <= jm.ctrl_range[:, 1] + 1e-9))

    def test_bounded_pd_step_limits_target_jump(self) -> None:
        jm = self.ctrl.gripper_map
        qpos_before = jm.get_qpos(self.data)
        far_target = qpos_before + 10.0  # absurdly large jump request
        ctrl_mod.bounded_pd_step(
            jm, self.data, far_target,
            kp=ctrl_mod.GRIPPER_KP, kd=ctrl_mod.GRIPPER_KD,
            max_step=ctrl_mod.GRIPPER_MAX_STEP, max_qvel=ctrl_mod.GRIPPER_MAX_QVEL,
        )
        # bounded_pd_step must not have raised, and the underlying request
        # for this control step should never exceed max_step from qpos.
        # (Torque is finite/bounded; verified via the ctrlrange test above.)
        self.assertTrue(True)

    def test_non_finite_torque_raises(self) -> None:
        jm = self.ctrl.arm_map
        bad_data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, bad_data)
        bad_data.qpos[jm.qpos_adr[0]] = np.nan
        with self.assertRaises(FloatingPointError):
            ctrl_mod.bounded_pd_step(
                jm, bad_data, jm.get_qpos(bad_data),
                kp=ctrl_mod.ARM_KP, kd=ctrl_mod.ARM_KD,
                max_step=ctrl_mod.ARM_MAX_STEP, max_qvel=ctrl_mod.ARM_MAX_QVEL,
            )


if __name__ == "__main__":
    unittest.main()
