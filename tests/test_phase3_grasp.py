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

from tasks.g1_pick_place.gripper_scene import write_grasp_scene
from tasks.g1_pick_place.run_grasp_test import run_trial


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
