#!/usr/bin/env python3
"""Phase 3 nominal grasp-and-lift acceptance test.

As of this commit the nominal trial does NOT pass (see
reports/phase3-grasping-baseline.md for the full failure analysis after 3
tuning iterations). This test asserts the actual HANDOFF.md acceptance
criteria rather than a weakened stand-in, so it currently fails honestly;
it should start passing once the tracking-oscillation root cause documented
in the report is fixed in a future session.
"""

from __future__ import annotations

import unittest

import mujoco

from tasks.g1_pick_place.gripper_scene import write_grasp_scene
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard, run_trial


class CubeInitGuardTest(unittest.TestCase):
    """Enforces HANDOFF.md's initialization boundary: cube qpos/qvel may be
    set only before a trial's first mj_step; any later attempt must raise."""

    @classmethod
    def setUpClass(cls) -> None:
        scene = write_grasp_scene()
        cls.model = mujoco.MjModel.from_xml_path(str(scene))

    def _cube_addrs(self):
        cube_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
        return int(self.model.jnt_qposadr[cube_joint_id]), int(self.model.jnt_dofadr[cube_joint_id])

    def test_pre_lock_write_succeeds(self) -> None:
        data = mujoco.MjData(self.model)
        qpos_adr, dof_adr = self._cube_addrs()
        guard = CubeInitGuard(data, qpos_adr, dof_adr)
        guard.set_initial_pose([0.4, -0.1, 0.9])
        self.assertAlmostEqual(float(data.qpos[qpos_adr]), 0.4)
        self.assertFalse(guard.locked)

    def test_post_lock_qpos_write_raises(self) -> None:
        data = mujoco.MjData(self.model)
        qpos_adr, dof_adr = self._cube_addrs()
        guard = CubeInitGuard(data, qpos_adr, dof_adr)
        guard.set_initial_pose([0.33, -0.15, 0.735])
        mujoco.mj_forward(self.model, data)
        mujoco.mj_step(self.model, data)  # the trial's first physics step
        guard.lock()
        with self.assertRaises(RuntimeError):
            guard.set_initial_pose([0.5, 0.5, 1.0])  # simulated cheat attempt

    def test_post_lock_qvel_write_raises(self) -> None:
        data = mujoco.MjData(self.model)
        qpos_adr, dof_adr = self._cube_addrs()
        guard = CubeInitGuard(data, qpos_adr, dof_adr)
        guard.set_initial_pose([0.33, -0.15, 0.735])
        mujoco.mj_forward(self.model, data)
        mujoco.mj_step(self.model, data)
        guard.lock()
        with self.assertRaises(RuntimeError):
            guard.set_initial_velocity([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_run_trial_source_has_no_direct_cube_state_write_outside_guard(self) -> None:
        import inspect

        from tasks.g1_pick_place import run_grasp_test as rgt

        src = inspect.getsource(rgt.run_trial)
        self.assertNotIn("data.qpos[cube_qpos_adr", src)
        self.assertNotIn("data.qvel[cube_dof_adr", src)
        self.assertNotIn("xfrc_applied[cube_body_id]", src)
        # run_trial must actually route cube init through the guard, not
        # bypass it entirely.
        self.assertIn("guard.set_initial_pose", src)
        self.assertIn("guard.lock()", src)


class Phase3NominalGraspTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        scene = write_grasp_scene()
        cls.result = run_trial(scene, cube_xy_offset=(0.0, 0.0))

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


if __name__ == "__main__":
    unittest.main()
