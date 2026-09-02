#!/usr/bin/env python3
"""Phase 4B: Task 1 complete pick-and-place tests.

Fixed-base, torso-constrained upper-body manipulation baseline (Phase 3C's
grasp architecture, unchanged) extended with physical transport, lowering,
release, retreat, and an objective task-success detector
(tasks/g1_pick_place/run_pick_place.py). These tests run the actual pipeline
(not a mock) and assert structural/aggregate properties plus the exact
measured nominal/variant numbers, in the same style as
tests/test_phase3c_grasp.py and tests/test_phase4a_grasp_variants.py.

Does not modify or re-run Phase 1/2/3/3B/3C/4A's own tests -- those remain
the untouched historical record.
"""

from __future__ import annotations

import subprocess
import unittest

import mujoco
import numpy as np

from tasks.g1_pick_place import gripper_scene
from tasks.g1_pick_place.controller import RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap
from tasks.g1_pick_place.controller_3c import IK_POS_TOL
from tasks.g1_pick_place.run_grasp_test import CubeInitGuard
from tasks.g1_pick_place.run_pick_place import (
    EXCLUDED_UNREACHABLE_VARIANTS,
    RETREAT_DISTURBANCE_TOL_M,
    STAGE_B_VARIANTS,
    TARGET_XY,
    TASK_SUCCESS_DWELL_S,
    _assert_run_trial_pick_place_has_no_direct_cube_state_write,
    diagnose_pick_place_reachability,
    run_trial_pick_place,
)

# Computed once for the whole module: the nominal trial (~2s) and the 3
# Stage B variants (~6s) are each run a handful of times across the test
# classes below; caching avoids re-running the full pipeline for every
# assertion while still exercising the real, unmocked code path.
_NOMINAL = None
_VARIANTS = None


def setUpModule() -> None:
    global _NOMINAL, _VARIANTS
    scene = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0, scene_name="g1_grasp_scene_4b.xml")
    _NOMINAL = run_trial_pick_place(scene)
    _VARIANTS = {
        v["id"]: run_trial_pick_place(scene, cube_xy_offset=v["offset"]) for v in STAGE_B_VARIANTS
    }


class Phase4BTargetSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0)
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))

    def test_target_pad_geom_exists_with_documented_dimensions(self) -> None:
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
        self.assertGreaterEqual(gid, 0)
        size = self.model.geom_size[gid]
        self.assertAlmostEqual(float(size[0]), gripper_scene.TARGET_HALF_XY)
        self.assertAlmostEqual(float(size[1]), gripper_scene.TARGET_HALF_XY)
        self.assertAlmostEqual(float(size[2]), gripper_scene.TARGET_HALF_Z)
        body_id = self.model.geom_bodyid[gid]
        pos = self.model.body_pos[body_id]
        self.assertAlmostEqual(float(pos[0]), gripper_scene.TARGET_POS[0])
        self.assertAlmostEqual(float(pos[1]), gripper_scene.TARGET_POS[1])

    def test_target_pad_is_visually_distinct_blue(self) -> None:
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
        mat_id = self.model.geom_matid[gid]
        self.assertGreaterEqual(mat_id, 0)
        rgba = self.model.mat_rgba[mat_id]
        # Blue channel clearly dominant over red -- distinct from the cube
        # (red, gripper_scene.py cube_mat) and the table (brown, table_mat).
        self.assertGreater(rgba[2], rgba[0])
        self.assertGreater(rgba[2], 0.5)

    def test_target_pad_has_no_joint_and_is_not_the_cube(self) -> None:
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
        body_id = self.model.geom_bodyid[gid]
        # A body with zero joints is implicitly welded to its parent
        # (the world) -- this is a static pad, not a free/movable object.
        self.assertEqual(int(self.model.body_jntnum[body_id]), 0)
        cube_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        self.assertNotEqual(gid, cube_geom_id)

    def test_no_equality_constraint_references_target_or_cube_beyond_the_pelvis_torso_weld(self) -> None:
        # Only the two known fixed-base welds (pelvis, torso) may exist;
        # nothing may weld/tie the cube or the target pad to anything.
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            for i in range(self.model.neq)
        ]
        self.assertEqual(set(names), {"pelvis_fixed", "torso_fixed"})

    def test_no_tendon_actuator_or_force_references_cube_or_target(self) -> None:
        self.assertEqual(self.model.ntendon, 0)
        for aid in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            self.assertNotIn("cube", name)
            self.assertNotIn("target", name)

    def test_target_pad_collides_with_cube_geometry(self) -> None:
        # contype/conaffinity must actually be set so the cube can physically
        # rest on the pad (placement is a real contact, not a rendering trick).
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
        self.assertEqual(int(self.model.geom_contype[gid]), 1)
        self.assertEqual(int(self.model.geom_conaffinity[gid]), 1)

    def test_grasp_scene_3c_unaffected_by_4b_additions(self) -> None:
        # write_grasp_scene_3c (Phase 3C/4A) must not gain a target pad.
        scene_3c = gripper_scene.write_grasp_scene_3c(arm_kp=400.0, arm_kv=25.0, scene_name="g1_grasp_scene_3c.xml")
        model_3c = mujoco.MjModel.from_xml_path(str(scene_3c))
        gid = mujoco.mj_name2id(model_3c, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
        self.assertEqual(gid, -1)

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


class Phase4BReachabilityTest(unittest.TestCase):
    """Target selection: every carry waypoint (TRANSPORT_ABOVE_TARGET,
    LOWER_TO_TARGET, RETREAT) plus the grasp waypoints must be reachable
    within IK_POS_TOL, evaluated before any trial is trusted."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0)
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))

    def test_all_seven_waypoints_reachable_within_tolerance(self) -> None:
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        arm_map = JointMap.build(self.model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        report = diagnose_pick_place_reachability(
            self.model, arm_map, site_id, data.qpos.copy(), np.array(gripper_scene.CUBE_POS)
        )
        self.assertTrue(report["all_reachable"])
        for name in ("PREGRASP", "APPROACH", "CLOSED_LIFT", "HOLD", "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET", "RETREAT"):
            self.assertLess(report[name]["residual_m"], IK_POS_TOL, msg=name)

    def test_target_requires_meaningful_lateral_transport(self) -> None:
        # The target must not sit directly under the nominal lift position:
        # require at least 0.10 m of lateral separation from the cube's own
        # footprint (0.07 m) -- otherwise this would not exercise transport.
        dx = TARGET_XY[0] - gripper_scene.CUBE_POS[0]
        dy = TARGET_XY[1] - gripper_scene.CUBE_POS[1]
        lateral_dist = float(np.hypot(dx, dy))
        self.assertGreaterEqual(lateral_dist, 0.10)

    def test_target_stays_within_table_bounds_with_margin(self) -> None:
        table_half = 0.22
        edge_margin = 0.05
        dx = abs(TARGET_XY[0] - gripper_scene.CUBE_POS[0])
        dy = abs(TARGET_XY[1] - gripper_scene.CUBE_POS[1])
        limit = table_half - edge_margin - gripper_scene.CUBE_HALF
        self.assertLessEqual(dx, limit)
        self.assertLessEqual(dy, limit)


class Phase4BInitBoundaryTest(unittest.TestCase):
    def test_run_trial_pick_place_source_has_no_direct_cube_state_write(self) -> None:
        _assert_run_trial_pick_place_has_no_direct_cube_state_write()

    def test_guard_raises_after_lock(self) -> None:
        model = mujoco.MjModel.from_xml_path(
            str(gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0))
        )
        data = mujoco.MjData(model)
        cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
        guard = CubeInitGuard(data, int(model.jnt_qposadr[cube_joint_id]), int(model.jnt_dofadr[cube_joint_id]))
        guard.set_initial_pose([0.3, -0.1, 0.8])  # allowed pre-lock
        guard.lock()
        with self.assertRaises(RuntimeError):
            guard.set_initial_pose([0.0, 0.0, 0.0])


class Phase4BStateMachineTest(unittest.TestCase):
    """State-machine transition ordering for a successful trial."""

    EXPECTED_ORDER = [
        "RESET", "PREGRASP", "SETTLE_PREGRASP", "APPROACH", "SETTLE_APPROACH",
        "CLOSE", "VERIFY_BILATERAL_CONTACT", "LIFT", "HOLD",
        "TRANSPORT_ABOVE_TARGET", "SETTLE_ABOVE_TARGET", "LOWER_TO_TARGET",
        "SETTLE_LOWER", "OPEN", "VERIFY_RELEASE", "RETREAT",
        "VERIFY_TASK_SUCCESS", "DONE",
    ]

    def test_nominal_trial_enters_every_state_in_order(self) -> None:
        self.assertEqual(_NOMINAL["states_entered"], self.EXPECTED_ORDER)

    def test_no_state_skipped_or_repeated(self) -> None:
        states = _NOMINAL["states_entered"]
        self.assertEqual(len(states), len(set(states)), "a state was entered more than once")


class Phase4BAbortBehaviorTest(unittest.TestCase):
    """Loss-of-grasp abort: an aggressive transport (1 waypoint -- i.e. a
    single, un-subdivided ramp -- driven in 0.02 s instead of the winning
    Stage A configuration's 2.0 s across 40 waypoints) genuinely and
    reproducibly breaks bilateral contact during TRANSPORT_ABOVE_TARGET.
    Real, unmocked physics -- this is a permanent regression check that the
    abort path fires honestly (reports FAILED, does not force a pass) when
    the underlying grasp is actually lost, in the same spirit as Stage A
    Attempt 1's real (slower but still contact-breaking) failure."""

    @classmethod
    def setUpClass(cls) -> None:
        scene = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0)
        import tasks.g1_pick_place.run_pick_place as rp
        orig = rp.TRANSPORT_N_WAYPOINTS
        rp.TRANSPORT_N_WAYPOINTS = 1
        try:
            cls.result = run_trial_pick_place(scene, transport_drive_s=0.02)
        finally:
            rp.TRANSPORT_N_WAYPOINTS = orig

    def test_trial_aborts_rather_than_forcing_a_pass(self) -> None:
        self.assertFalse(self.result["task_pass"])
        self.assertIsNotNone(self.result["failure_state"])
        self.assertIsNotNone(self.result["failure_reason"])

    def test_abort_is_attributed_to_contact_loss(self) -> None:
        self.assertTrue(self.result["contact_lost_during_transport"])

    def test_grasp_itself_still_succeeded_before_the_abort(self) -> None:
        # The grasp (PREGRASP..VERIFY_BILATERAL_CONTACT..LIFT..HOLD) is
        # unaffected by the transport failure -- confirms the abort is
        # specific to the new transport logic, not a regression in the
        # unchanged grasp-phase code reused from Phase 3C.
        self.assertTrue(self.result["criteria_grasp"]["both_pads_contact_cube"])
        self.assertTrue(self.result["criteria_grasp"]["height_gain_ge_0_08m"])


class Phase4BReleaseDetectionTest(unittest.TestCase):
    def test_cube_released_by_both_pads_after_open(self) -> None:
        self.assertTrue(_NOMINAL["criteria_grasp"]["released_after_open"])

    def test_release_checked_after_a_settle_window_not_immediately(self) -> None:
        # RELEASE_SETTLE is a distinct, non-zero drive segment between OPEN
        # and VERIFY_RELEASE (see run_pick_place.py) -- release is not
        # declared in the same instant the fingers are commanded open.
        from tasks.g1_pick_place.run_grasp_test_3c import DRIVE_S
        self.assertGreater(DRIVE_S["RELEASE_SETTLE"], 0.0)


class Phase4BSuccessDetectorBoundaryTest(unittest.TestCase):
    """Objective task-success detector: continuous-dwell requirement."""

    def test_dwell_requirement_is_nonzero(self) -> None:
        self.assertGreater(TASK_SUCCESS_DWELL_S, 0.0)

    def test_successful_trial_achieved_the_full_dwell_not_an_instant_crossing(self) -> None:
        # A pass must show a recorded dwell duration >= TASK_SUCCESS_DWELL_S,
        # not merely "conditions were true on one step".
        self.assertGreaterEqual(_NOMINAL["task_success_dwell_achieved_s"], TASK_SUCCESS_DWELL_S)

    def test_failing_variant_never_accumulated_a_full_streak(self) -> None:
        # y_plus_0.03 fails the target-XY criterion continuously (the cube
        # never enters the boundary), so its best streak must be 0, not a
        # partial credit for briefly touching the condition.
        failing = _VARIANTS["y_plus_0.03"]
        self.assertFalse(failing["task_pass"])
        self.assertEqual(failing["task_success_dwell_achieved_s"], 0.0)

    def test_all_eight_placement_conditions_present_and_boolean(self) -> None:
        expected_keys = {
            "cube_in_target_xy", "cube_supported_not_held", "cube_linear_speed_ok",
            "cube_angular_speed_ok", "held_continuously_full_dwell",
            "retreated_without_disturbing_cube", "no_transport_contact_loss",
            "no_transport_height_violation",
        }
        self.assertEqual(set(_NOMINAL["criteria_placement"].keys()), expected_keys)
        for v in _NOMINAL["criteria_placement"].values():
            self.assertIsInstance(v, bool)


class Phase4BNominalPickPlaceTest(unittest.TestCase):
    """The measured nominal Task 1 result (see reports/phase4b-task1-pick-place.md)."""

    def test_nominal_task_pass(self) -> None:
        self.assertTrue(_NOMINAL["task_pass"])
        self.assertTrue(_NOMINAL["grasp_pass"])
        self.assertTrue(_NOMINAL["placement_pass"])

    def test_nominal_height_gain_matches_phase3c_unchanged_grasp(self) -> None:
        # Grasp physics are byte-for-byte reused from Phase 3C -- this must
        # equal Phase 3C/4A's own recorded nominal height gain exactly.
        self.assertAlmostEqual(_NOMINAL["height_gain_m"], 0.10838913826086727, places=6)

    def test_nominal_final_xy_target_error_within_margin(self) -> None:
        self.assertAlmostEqual(_NOMINAL["final_xy_target_error_m"], 0.014564117068399008, places=6)
        self.assertLess(_NOMINAL["final_xy_target_error_m"], gripper_scene.TARGET_XY_SUCCESS_MARGIN_M)

    def test_nominal_retreat_did_not_disturb_cube(self) -> None:
        self.assertLessEqual(_NOMINAL["retreat_disturbance_m"], RETREAT_DISTURBANCE_TOL_M)

    def test_nominal_cube_at_rest_at_finish(self) -> None:
        self.assertLess(_NOMINAL["final_cube_linear_speed"], 1e-6)
        self.assertLess(_NOMINAL["final_cube_angular_speed"], 1e-6)


class Phase4BDeterminismTest(unittest.TestCase):
    def test_deterministic_across_five_reruns(self) -> None:
        scene = gripper_scene.write_grasp_scene_4b(arm_kp=400.0, arm_kv=25.0)
        results = [run_trial_pick_place(scene) for _ in range(5)]
        self.assertTrue(all(r["task_pass"] for r in results))
        self.assertEqual(len({r["height_gain_m"] for r in results}), 1)
        self.assertEqual(len({r["final_xy_target_error_m"] for r in results}), 1)
        self.assertEqual(len({tuple(r["states_entered"]) for r in results}), 1)


class Phase4BStageBVariantsTest(unittest.TestCase):
    """The three Phase 4A-reachable grasp variants against the fixed target,
    under the identical Stage A configuration (no per-variant tuning)."""

    def test_exactly_three_variants_defined_matching_phase4a_reachable_set(self) -> None:
        ids = {v["id"] for v in STAGE_B_VARIANTS}
        self.assertEqual(ids, {"nominal", "x_minus_0.03", "y_plus_0.03"})

    def test_excluded_variants_are_the_two_phase4a_unreachable_ones(self) -> None:
        ids = {v["id"] for v in EXCLUDED_UNREACHABLE_VARIANTS}
        self.assertEqual(ids, {"x_plus_0.03", "y_minus_0.03"})

    def test_nominal_and_x_minus_variant_complete_task_successfully(self) -> None:
        self.assertTrue(_VARIANTS["nominal"]["task_pass"])
        self.assertTrue(_VARIANTS["x_minus_0.03"]["task_pass"])

    def test_y_plus_variant_grasps_but_fails_placement(self) -> None:
        # Documented, honestly-reported limitation (see
        # reports/phase4b-task1-pick-place.md): the grasp itself succeeds
        # (reusing Phase 3C's unmodified, already-reachable grasp) but the
        # fixed target's tight XY margin is missed for this cube start.
        v = _VARIANTS["y_plus_0.03"]
        self.assertTrue(v["grasp_pass"])
        self.assertFalse(v["placement_pass"])
        self.assertFalse(v["task_pass"])

    def test_supported_envelope_meets_at_least_two_of_three(self) -> None:
        n_pass = sum(1 for r in _VARIANTS.values() if r["task_pass"])
        self.assertGreaterEqual(n_pass, 2)

    def test_no_variant_lost_bilateral_contact_during_transport(self) -> None:
        # All three variants' grasps survive transport; the y_plus_0.03
        # failure is a placement-accuracy issue, not a dropped cube.
        for vid, r in _VARIANTS.items():
            self.assertFalse(r["contact_lost_during_transport"], msg=vid)


if __name__ == "__main__":
    unittest.main()
