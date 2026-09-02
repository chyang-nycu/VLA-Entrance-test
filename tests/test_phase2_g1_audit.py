#!/usr/bin/env python3
"""Automated checks for the Phase 2 G1 manipulation audit."""

from __future__ import annotations

import subprocess
import unittest

import mujoco

from tasks.g1_pick_place import g1_manipulation_audit as audit


class Phase2G1AuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = mujoco.MjModel.from_xml_path(str(audit.SCENE))
        cls.site_scene = audit.write_site_probe_scene()
        cls.contact_scene = audit.write_contact_scene()

    def test_selected_wrist_body_and_task_site_exist(self) -> None:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
        self.assertGreaterEqual(body_id, 0)

        site_model = mujoco.MjModel.from_xml_path(str(self.site_scene))
        site_id = mujoco.mj_name2id(site_model, mujoco.mjtObj.mjOBJ_SITE, "right_wrist_tcp_probe")
        self.assertGreaterEqual(site_id, 0)

    def test_expected_right_arm_actuators_exist(self) -> None:
        expected = [
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_shoulder_yaw",
            "right_elbow",
            "right_wrist_roll",
            "right_wrist_pitch",
            "right_wrist_yaw",
        ]
        for actuator in expected:
            with self.subTest(actuator=actuator):
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
                self.assertGreaterEqual(actuator_id, 0)

    def test_cube_contact_can_be_detected(self) -> None:
        result = audit.run_contact_test(self.contact_scene)
        self.assertTrue(result["contact_detected"])
        self.assertFalse(result["start_had_contact"])
        self.assertEqual(result["contacts"][0]["other_body"], "right_wrist_yaw_link")

    def test_vendor_g1_files_remain_unchanged(self) -> None:
        head = subprocess.check_output(
            ["git", "-C", str(audit.VENDOR), "rev-parse", "HEAD"],
            text=True,
        ).strip()
        self.assertEqual(head, "4134cb5dc7ff1ba7f484deda48b5274b58694519")

        status = subprocess.check_output(
            ["git", "-C", str(audit.VENDOR), "status", "--short", "--", "unitree_robots/g1"],
            text=True,
        ).strip()
        self.assertEqual(status, "")


if __name__ == "__main__":
    unittest.main()
