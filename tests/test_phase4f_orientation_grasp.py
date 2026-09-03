#!/usr/bin/env python3
"""Phase 4F: orientation-constrained grasp stabilization tests.

Covers the orientation-IK math (synthetic, isolated from simulation), the
finger-pad mounting fix's geometry, and the real, current (still-failing)
behavior of the full pipeline against the tightened acceptance bar. Several
tests here honestly assert FAILURE against the current configuration --
per HANDOFF.md's explicit instruction for this phase ("if 3 attempts
aren't enough... report the quantitative failure analysis rather than
forcing a pass"), these are not weakened or marked expectedFailure; they
pin down the exact, real, currently-measured numbers so any future change
to this pipeline is caught by a real regression, not silently drifted.
"""
from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from tasks.g1_pick_place import gripper_scene as gs
from tasks.g1_pick_place import run_pick_place as rp
from tasks.g1_pick_place.controller import JointMap, RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE
from tasks.g1_pick_place.controller_3c import (
    ORIENT_TOL_RAD,
    ORIENT_WEIGHT,
    orientation_residual_rad,
    solve_ik_waypoint,
    solve_ik_waypoint_oriented,
)


class OrientationMathUnitTest(unittest.TestCase):
    """Synthetic checks on orientation_residual_rad() -- independent of any
    real simulation trial, same pattern as Phase 4C's slip-math unit tests.
    """

    def test_identity_rotation_has_zero_residual(self) -> None:
        R = np.eye(3)
        self.assertAlmostEqual(orientation_residual_rad(R.flatten()), 0.0, places=9)

    def test_90_degree_tilt_has_pi_over_2_residual(self) -> None:
        # Local Z rotated to point along world X: 90 deg from world Z.
        R = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
        self.assertAlmostEqual(orientation_residual_rad(R.flatten()), np.pi / 2, places=6)

    def test_yaw_about_vertical_does_not_change_residual(self) -> None:
        """Rotation about the required axis itself (world/local Z, already
        aligned) must leave the residual at zero -- this is the "leave the
        aligned axis's own yaw free" design property from controller_3c.py's
        module docstring, checked directly."""
        c, s = np.cos(0.7), np.sin(0.7)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.assertAlmostEqual(orientation_residual_rad(Rz.flatten()), 0.0, places=9)


class OrientedIKTest(unittest.TestCase):
    """Real (non-synthetic) IK solves against the actual Task 1 scene."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gs.write_grasp_scene_4b(
            arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml"
        )
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))
        cls.arm_map = JointMap.build(cls.model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        cls.site_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)

    def setUp(self) -> None:
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.scratch = mujoco.MjData(self.model)
        self.nominal_q = np.zeros(len(self.arm_map.names))
        self.cube_pos = np.array(gs.CUBE_POS)

    def test_oriented_solve_returns_four_values(self) -> None:
        result = solve_ik_waypoint_oriented(
            self.model, self.scratch, self.data.qpos.copy(), self.arm_map, self.site_id,
            self.cube_pos, self.nominal_q,
        )
        self.assertEqual(len(result), 4)
        q, pos_resid, iters, orient_resid = result
        self.assertEqual(q.shape, (len(self.arm_map.names),))
        self.assertTrue(np.isfinite(pos_resid))
        self.assertTrue(np.isfinite(orient_resid))
        self.assertGreaterEqual(orient_resid, 0.0)

    def test_oriented_solve_reduces_or_matches_orientation_residual_vs_plain(self) -> None:
        """The oriented solver must never leave orientation WORSE than the
        plain, unconstrained solver at the same target -- it is a strict
        secondary objective on top of the same primary task."""
        _, _, _, orient_oriented = solve_ik_waypoint_oriented(
            self.model, self.scratch, self.data.qpos.copy(), self.arm_map, self.site_id,
            self.cube_pos, self.nominal_q,
        )
        _, _, _ = solve_ik_waypoint(
            self.model, self.scratch, self.data.qpos.copy(), self.arm_map, self.site_id,
            self.cube_pos, self.nominal_q,
        )
        scratch2 = mujoco.MjData(self.model)
        q_plain, _, _ = solve_ik_waypoint(
            self.model, scratch2, self.data.qpos.copy(), self.arm_map, self.site_id, self.cube_pos, self.nominal_q,
        )
        scratch2.qpos[:] = self.data.qpos.copy()
        self.arm_map.set_qpos(scratch2, q_plain)
        mujoco.mj_kinematics(self.model, scratch2)
        orient_plain = orientation_residual_rad(scratch2.site_xmat[self.site_id])
        self.assertLessEqual(orient_oriented, orient_plain + 1e-6)

    def test_orientation_residual_at_approach_documents_the_reachability_conflict(self) -> None:
        """Pins down the real, measured finding this phase's report is built
        on: at the nominal APPROACH waypoint, orientation residual remains
        far above ORIENT_TOL_RAD even with the orientation objective active
        -- a genuine kinematic conflict (see reports/phase4f-orientation-
        grasp-stabilization.md Attempt 1/2), not a bug. Do not lower this
        assertion to make it pass; if a future architecture change closes
        this gap, replace this test with one that checks the new, smaller
        residual instead of loosening this one in place.
        """
        _, _, _, orient_resid = solve_ik_waypoint_oriented(
            self.model, self.scratch, self.data.qpos.copy(), self.arm_map, self.site_id,
            self.cube_pos, self.nominal_q,
        )
        self.assertGreater(orient_resid, ORIENT_TOL_RAD)
        self.assertGreater(np.degrees(orient_resid), 30.0)


class PadMountFixGeometryTest(unittest.TestCase):
    """Verifies the Attempt-3 finger-pad mounting correction is present,
    scoped correctly, and does not disturb the finger's own position or the
    other (Phase 3/3B/3C) scenes.
    """

    def test_task1_scene_finger_bodies_have_mount_fix_quat(self) -> None:
        scene = gs.write_grasp_scene_4f(arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml")
        tree = ET.parse(scene)
        found = {"left_finger": False, "right_finger": False}
        for body in tree.getroot().iter("body"):
            name = body.get("name")
            if name in found:
                quat = body.get("quat")
                self.assertIsNotNone(quat, f"{name} missing the Phase 4F mount-fix quat")
                w, x, y, z = (float(v) for v in quat.split())
                self.assertAlmostEqual(w, gs.FINGER_MOUNT_FIX_QUAT[0], places=6)
                self.assertAlmostEqual(x, gs.FINGER_MOUNT_FIX_QUAT[1], places=6)
                self.assertAlmostEqual(y, gs.FINGER_MOUNT_FIX_QUAT[2], places=6)
                self.assertAlmostEqual(z, gs.FINGER_MOUNT_FIX_QUAT[3], places=6)
                pos = body.get("pos")
                self.assertIsNotNone(pos)
                found[name] = True
        self.assertTrue(all(found.values()))

    def test_finger_position_unchanged_by_mount_fix(self) -> None:
        """The mount fix must rotate the finger body's own frame only --
        its pos attribute (origin, in the wrist's frame) must be bit-
        identical to the un-fixed geometry, so the finger still brackets
        the TCP/cube target exactly as before."""
        fixed = ET.parse(
            gs.write_grasp_scene_4f(arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml")
        )
        unfixed_tree = gs._build_grasp_tree(
            extra_trunk_weld=True, finger_pad_half=gs.FINGER_PAD_HALF, apply_phase4e_gripper_visuals=True,
            apply_phase4f_pad_mount_fix=False,
        )
        fixed_pos = {}
        unfixed_pos = {}
        for body in fixed.getroot().iter("body"):
            if body.get("name") in ("left_finger", "right_finger"):
                fixed_pos[body.get("name")] = body.get("pos")
        for body in unfixed_tree.getroot().iter("body"):
            if body.get("name") in ("left_finger", "right_finger"):
                unfixed_pos[body.get("name")] = body.get("pos")
        self.assertEqual(fixed_pos, unfixed_pos)

    def test_legacy_scenes_have_no_mount_fix(self) -> None:
        scene_3c = gs.write_grasp_scene_3c(arm_kp=400.0, arm_kv=25.0, scene_name="g1_grasp_scene_3c.xml")
        tree = ET.parse(scene_3c)
        for body in tree.getroot().iter("body"):
            if body.get("name") in ("left_finger", "right_finger"):
                self.assertIsNone(body.get("quat"))

    def test_jaw_axis_unaffected_by_mount_fix(self) -> None:
        """The joint's slide axis (0 1 0, in the finger's own local frame)
        must be numerically invariant under a rotation about that same
        axis -- confirms the fix does not perturb finger open/close travel."""
        w, x, y, z = gs.FINGER_MOUNT_FIX_QUAT
        quat_wxyz = np.array([w, x, y, z])
        axis_before = np.array([0.0, 1.0, 0.0])
        rotated = np.zeros(3)
        mujoco.mju_rotVecQuat(rotated, axis_before, quat_wxyz)
        np.testing.assert_allclose(rotated, axis_before, atol=1e-9)


class Phase4FRealPipelineTest(unittest.TestCase):
    """Real, freshly-simulated end-to-end checks against the final (Attempt
    3) configuration -- both the deterministic-repeatability property and
    the honest, currently-failing acceptance numbers.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = gs.write_grasp_scene_4f(
            arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml"
        )
        cls.result = rp.run_trial_pick_place(cls.scene_path, use_oriented_ik=True)

    def test_grasp_still_forms_with_bilateral_opposing_contact(self) -> None:
        self.assertTrue(self.result["opposing_face_contact_left"])
        self.assertTrue(self.result["opposing_face_contact_right"])

    def test_cube_still_genuinely_lifted(self) -> None:
        self.assertGreaterEqual(self.result["height_gain_m"], 0.08)

    def test_normal_forces_are_positive_and_finite(self) -> None:
        self.assertTrue(self.result["criteria_grasp_stability_4f"]["normal_forces_positive_and_finite"])

    def test_max_slip_while_grasped_still_exceeds_tightened_bar(self) -> None:
        """Documents the real, current gap this phase's report reports
        honestly: the tightened <=10mm bar is not met. Do not lower this
        assertion to force a pass -- if a future phase closes the gap,
        replace this test with one asserting the new, passing value."""
        self.assertGreater(self.result["max_slip_while_grasped_m"], 0.010)
        self.assertAlmostEqual(self.result["max_slip_while_grasped_m"], 0.02592, places=4)

    def test_grasp_stability_pass_4f_is_honestly_false(self) -> None:
        self.assertFalse(self.result["grasp_stability_pass_4f"])

    def test_deterministic_across_five_reruns(self) -> None:
        results = [rp.run_trial_pick_place(self.scene_path, use_oriented_ik=True) for _ in range(4)]
        slips = [self.result["max_slip_while_grasped_m"]] + [r["max_slip_while_grasped_m"] for r in results]
        for s in slips[1:]:
            self.assertEqual(s, slips[0])
        states = [self.result["failure_state"]] + [r["failure_state"] for r in results]
        self.assertEqual(len(set(states)), 1)

    def test_no_post_init_cube_state_manipulation(self) -> None:
        # Reuses the same module-level self-audit already enforced at import
        # time; re-invoked here so a regression is caught by this file too.
        rp._assert_run_trial_pick_place_has_no_direct_cube_state_write()

    def test_vendor_unchanged(self) -> None:
        # The submodule's own working tree has shown pre-existing, unrelated
        # local dirt (the historical Go2w terrain.STL case-collision
        # artifact) since before Phase 3 -- every prior phase's
        # verification confirms this, not a regression here. What must
        # never change is the PINNED commit this repo points at.
        import subprocess

        out = subprocess.run(
            ["git", "-C", str(ROOT), "ls-tree", "HEAD", "vendor/unitree_mujoco"],
            capture_output=True, text=True,
        )
        self.assertIn("4134cb5dc7ff1ba7f484deda48b5274b58694519", out.stdout)


class Phase4FStageBInformationalTest(unittest.TestCase):
    """Stage B was run informationally only (Stage A's gate was not met),
    per HANDOFF.md Section D -- these tests confirm that framing is
    reflected honestly, not silently upgraded to a pass/fail claim."""

    def test_stage_a_gate_not_met_on_final_configuration(self) -> None:
        scene = gs.write_grasp_scene_4f(arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml")
        r = rp.run_trial_pick_place(scene, use_oriented_ik=True)
        self.assertFalse(r["grasp_stability_pass_4f"])

    def test_stage_b_variants_all_fail_the_same_gate(self) -> None:
        scene = gs.write_grasp_scene_4f(arm_kp=rp.ARM_KP_4B, arm_kv=rp.ARM_KV_4B, scene_name="g1_grasp_scene_4f.xml")
        for offset in ((-0.03, 0.0), (0.0, 0.03)):
            r = rp.run_trial_pick_place(scene, cube_xy_offset=offset, use_oriented_ik=True)
            self.assertFalse(r["grasp_stability_pass_4f"])


if __name__ == "__main__":
    unittest.main()
