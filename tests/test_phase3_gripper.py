#!/usr/bin/env python3
"""Structural/physical checks for the Phase 3 task-local gripper MJCF."""

from __future__ import annotations

import subprocess
import unittest

import mujoco

from tasks.g1_pick_place import gripper_scene


class Phase3GripperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gripper_scene.write_grasp_scene()
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))

    def test_finger_joints_are_slide_and_symmetric(self) -> None:
        left_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_finger_joint")
        right_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_finger_joint")
        self.assertGreaterEqual(left_id, 0)
        self.assertGreaterEqual(right_id, 0)
        self.assertEqual(int(self.model.jnt_type[left_id]), int(mujoco.mjtJoint.mjJNT_SLIDE))
        self.assertEqual(int(self.model.jnt_type[right_id]), int(mujoco.mjtJoint.mjJNT_SLIDE))

        left_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_finger")
        right_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_finger")
        left_pos = self.model.body_pos[left_body]
        right_pos = self.model.body_pos[right_body]
        self.assertAlmostEqual(float(left_pos[0]), float(right_pos[0]), places=6)
        self.assertAlmostEqual(float(left_pos[1]), -float(right_pos[1]), places=6)
        self.assertAlmostEqual(float(left_pos[2]), float(right_pos[2]), places=6)

    def test_finger_pads_are_collision_enabled(self) -> None:
        for geom_name in ("left_finger_pad", "right_finger_pad"):
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            self.assertGreaterEqual(geom_id, 0)
            self.assertEqual(int(self.model.geom_contype[geom_id]), 1)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 1)

    def test_grasp_tcp_site_exists_on_wrist(self) -> None:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "grasp_tcp")
        self.assertGreaterEqual(site_id, 0)
        body_id = int(self.model.site_bodyid[site_id])
        self.assertEqual(
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id),
            gripper_scene.WRIST_BODY,
        )

    def test_pelvis_is_fixed_by_model_level_equality_weld(self) -> None:
        self.assertGreaterEqual(self.model.neq, 1)
        eq_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "pelvis_fixed")
        self.assertGreaterEqual(eq_id, 0)
        self.assertEqual(int(self.model.eq_type[eq_id]), int(mujoco.mjtEq.mjEQ_WELD))
        pelvis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, gripper_scene.PELVIS_BODY)
        self.assertEqual(int(self.model.eq_obj1id[eq_id]), pelvis_id)

    def test_pelvis_freejoint_still_present(self) -> None:
        pelvis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, gripper_scene.PELVIS_BODY)
        jadr = int(self.model.body_jntadr[pelvis_id])
        self.assertEqual(int(self.model.jnt_type[jadr]), int(mujoco.mjtJoint.mjJNT_FREE))

    def test_cube_has_freejoint_not_welded(self) -> None:
        cube_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.assertGreaterEqual(cube_id, 0)
        jadr = int(self.model.body_jntadr[cube_id])
        self.assertEqual(int(self.model.jnt_type[jadr]), int(mujoco.mjtJoint.mjJNT_FREE))
        for i in range(self.model.neq):
            if int(self.model.eq_obj1id[i]) == cube_id or int(self.model.eq_obj2id[i]) == cube_id:
                self.fail("cube body must not participate in any equality constraint")

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


if __name__ == "__main__":
    unittest.main()
