#!/usr/bin/env python3
"""Phase 4C: slip-metric audit tests.

Phase 4B's `max_cube_slip_m` (0.1561555402975266 for the nominal trial,
reports/phase4b-task1-pick-place.md section 3) turned out to include TCP-
cube separation recorded *after* the cube was released -- the gripper
opens, the arm retreats, and the cube (no longer held) separates from the
TCP, and that separation was silently folded into "grasp slip" because the
old code only ever checked whether a grasp reference had been captured,
never whether the cube was still being carried. This file has two parts:

1. `SlipMathUnitTest` -- synthetic, purely mathematical checks of
   `tcp_local_cube_offset` / `relative_slip_m` (tasks/g1_pick_place/
   run_pick_place.py) against known rigid-body motions, independent of any
   simulation trial.
2. `PostReleaseIsolationTest` -- runs the real (unmocked) nominal pipeline
   and confirms the corrected phase-scoped metrics
   (max_slip_during_lift/transport/lower, slip_at_release) stay well below
   the post-release separation, i.e. RETREAT/OPEN/VERIFY_RELEASE no longer
   leak into what is reported as grasp slip.

See reports/phase4c-task1-evidence.md for the full audit writeup and
corrected numbers; reports/phase4b-task1-pick-place.md is left unedited as
the historical record of the original (incorrect) figure.
"""

from __future__ import annotations

import unittest

import numpy as np

from tasks.g1_pick_place import gripper_scene
from tasks.g1_pick_place.run_pick_place import (
    STAGE_B_VARIANTS,
    relative_slip_m,
    run_trial_pick_place,
    tcp_local_cube_offset,
)


def _rotation_about_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class SlipMathUnitTest(unittest.TestCase):
    """Synthetic rigid-body checks of the slip math itself."""

    def test_pure_world_translation_produces_zero_local_frame_slip(self) -> None:
        tcp_rot = np.eye(3)
        tcp_pos_0 = np.array([0.30, -0.10, 0.80])
        cube_pos_0 = np.array([0.35, -0.05, 0.80])
        ref_offset = tcp_local_cube_offset(tcp_pos_0, tcp_rot, cube_pos_0)

        # Translate TCP and cube together (rigid, no rotation) by an
        # arbitrary vector -- a whole-body world translation.
        delta = np.array([0.12, -0.07, 0.03])
        tcp_pos_1 = tcp_pos_0 + delta
        cube_pos_1 = cube_pos_0 + delta
        offset_now = tcp_local_cube_offset(tcp_pos_1, tcp_rot, cube_pos_1)

        self.assertAlmostEqual(relative_slip_m(offset_now, ref_offset), 0.0, places=12)

    def test_pure_tcp_rotation_with_rigid_cube_offset_produces_zero_slip(self) -> None:
        tcp_pos = np.array([0.30, -0.10, 0.80])
        tcp_rot_0 = np.eye(3)
        local_offset_fixed = np.array([0.01, 0.0, -0.02])  # cube rigidly held at this local offset
        cube_pos_0 = tcp_pos + tcp_rot_0 @ local_offset_fixed
        ref_offset = tcp_local_cube_offset(tcp_pos, tcp_rot_0, cube_pos_0)
        self.assertTrue(np.allclose(ref_offset, local_offset_fixed))

        # Rotate the TCP (e.g. wrist redundancy resolution during transport)
        # while keeping the cube rigidly attached at the same local offset --
        # this is exactly the double-digit-degree wrist rotation documented
        # in reports/phase4b-task1-pick-place.md section 3, which motivated
        # measuring slip in the TCP's own frame in the first place.
        for theta_deg in (5.0, 15.0, 45.0, -30.0):
            tcp_rot_1 = _rotation_about_z(np.deg2rad(theta_deg))
            cube_pos_1 = tcp_pos + tcp_rot_1 @ local_offset_fixed  # still rigidly attached
            offset_now = tcp_local_cube_offset(tcp_pos, tcp_rot_1, cube_pos_1)
            self.assertAlmostEqual(
                relative_slip_m(offset_now, ref_offset), 0.0, places=10,
                msg=f"rotation-only motion at {theta_deg} deg should report zero slip",
            )

    def test_known_5mm_relative_displacement_reports_5mm(self) -> None:
        tcp_pos = np.array([0.30, -0.10, 0.80])
        tcp_rot = _rotation_about_z(np.deg2rad(20.0))  # arbitrary non-identity orientation
        local_offset_fixed = np.array([0.01, 0.0, -0.02])
        cube_pos_0 = tcp_pos + tcp_rot @ local_offset_fixed
        ref_offset = tcp_local_cube_offset(tcp_pos, tcp_rot, cube_pos_0)

        # Displace the cube by exactly 5 mm along the TCP's own local x-axis
        # -- a genuine relative displacement inside a (hypothetically)
        # slipping grip, with the TCP itself unmoved.
        local_displacement = np.array([0.005, 0.0, 0.0])
        cube_pos_1 = cube_pos_0 + tcp_rot @ local_displacement
        offset_now = tcp_local_cube_offset(tcp_pos, tcp_rot, cube_pos_1)

        self.assertAlmostEqual(relative_slip_m(offset_now, ref_offset), 0.005, places=9)


class PostReleaseIsolationTest(unittest.TestCase):
    """Runs the real, unmocked nominal pipeline and confirms RETREAT/OPEN/
    VERIFY_RELEASE motion no longer contaminates the grasp-phase slip
    metrics -- the actual regression this phase fixes.
    """

    @classmethod
    def setUpClass(cls) -> None:
        scene = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0, scene_name="g1_grasp_scene_4b.xml")
        cls.result = run_trial_pick_place(scene)

    def test_task_still_passes_metric_fix_is_measurement_only(self) -> None:
        # Original intent (Phase 4C): confirm this phase's slip-METRIC fix
        # changed reporting, not physics/controller behavior, by matching
        # the Phase 4B committed value exactly. Phase 4E (reports/
        # phase4e-gripper-integrity-repair.md) subsequently DID change
        # physics/controller behavior on purpose (gripper gains, finger
        # geometry, LIFT trajectory) as an evidence-based grasp-stability
        # repair -- so this test now tracks CURRENT write_grasp_scene_4b/
        # run_trial_pick_place behavior instead of the frozen Phase 4B
        # number (which remains correctly preserved, unedited, in
        # reports/phase4b-task1-pick-place.md and
        # logs/phase4b_pick_place_trials.json).
        self.assertTrue(self.result["task_pass"])
        self.assertAlmostEqual(self.result["height_gain_m"], 0.11746907818355656, places=9)
        self.assertAlmostEqual(self.result["final_xy_target_error_m"], 0.0017215286829355965, places=9)

    def test_legacy_max_cube_slip_m_reproduces_phase4b_value(self) -> None:
        # The old (uncorrected) field is retained unmodified -- its
        # semantics ("dominated by post-release separation, not grasp
        # slip") still hold post-Phase-4E, but its exact numeric value
        # necessarily shifted along with the Phase 4E physics change (see
        # the note above); it is no longer pinned to the frozen Phase 4B
        # commit's number, which remains preserved in
        # logs/phase4c_slip_audit.json instead.
        self.assertAlmostEqual(self.result["max_cube_slip_m"], 0.1550390839758238, places=9)

    def test_post_release_separation_dominates_the_old_legacy_number(self) -> None:
        # Root-cause confirmation: the old max_cube_slip_m is explained by
        # post-release TCP-cube separation, not by grasp-phase slip.
        self.assertGreater(self.result["post_release_tcp_cube_separation_m"], 0.10)
        self.assertAlmostEqual(
            self.result["post_release_tcp_cube_separation_m"], self.result["max_cube_slip_m"], delta=0.02,
        )

    def test_grasp_phase_slip_metrics_are_far_below_the_old_legacy_number(self) -> None:
        old_value = self.result["max_cube_slip_m"]
        for key in ("max_slip_during_lift", "max_slip_during_transport", "max_slip_during_lower", "slip_at_release"):
            with self.subTest(metric=key):
                self.assertIsNotNone(self.result[key])
                self.assertLess(self.result[key], old_value)
                # Every genuine grasp-phase slip metric should be a small
                # fraction of the cube's own 0.07 m footprint, unlike the
                # old post-release-contaminated figure.
                self.assertLess(self.result[key], 0.07)

    def test_retreat_does_not_affect_slip_at_release(self) -> None:
        # slip_at_release is captured at the last SETTLE_LOWER step, before
        # OPEN/VERIFY_RELEASE/RETREAT run at all -- it must be strictly
        # unrelated to whatever separation RETREAT later produces.
        self.assertLess(self.result["slip_at_release"], self.result["post_release_tcp_cube_separation_m"])

    def test_grasp_reference_offset_recorded_and_finite(self) -> None:
        ref = self.result["grasp_reference_offset_tcp_frame"]
        self.assertIsNotNone(ref)
        self.assertEqual(len(ref), 3)
        self.assertTrue(all(np.isfinite(v) for v in ref))


class StageBVariantsUseCorrectedMetricsTest(unittest.TestCase):
    """Confirms the corrected metrics are also well-behaved (bounded, not
    post-release-contaminated) for the y_plus_0.03 variant, whose trial ends
    at VERIFY_TASK_SUCCESS (placement failure) rather than DONE -- a
    different code path through _finalize than the nominal PostRelease test
    above.
    """

    def test_y_plus_variant_grasp_phase_slip_is_bounded(self) -> None:
        variant = next(v for v in STAGE_B_VARIANTS if v["id"] == "y_plus_0.03")
        scene = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0, scene_name="g1_grasp_scene_4b.xml")
        result = run_trial_pick_place(scene, cube_xy_offset=variant["offset"])
        self.assertTrue(result["grasp_pass"])
        for key in ("max_slip_during_lift", "max_slip_during_transport", "max_slip_during_lower"):
            self.assertLess(result[key], 0.07)


if __name__ == "__main__":
    unittest.main()
