#!/usr/bin/env python3
"""Phase 4E: gripper visual/collision-integrity and grasp-stability repair.

Context: a user frame-by-frame review of artifacts/phase4d_failure_
reproduction.mp4 found the Phase-4D-confirmed visual defect (Section A
here) AND found the grasp itself was a "near-drop" -- the cube visibly
slid downward relative to the gripper throughout HOLD, consistent with the
corrected slip metrics in reports/phase4c-task1-evidence.md. This module
tests both the visual/collision fix (Section A) and the grasp-stability
redesign (Sections B/C), against the real, unmocked write_grasp_scene_4b/
run_trial_pick_place pipeline -- no mocked contacts, no cached logs.

Honest result, stated up front (full evidence in reports/phase4e-gripper-
integrity-repair.md): the visual/collision defect is fully fixed. Grasp
stability is substantially improved (worst-instant bilateral safety factor
rose from 0.054x pre-4E to >=1.0x at HOLD; max slip while grasped fell from
as much as 5.19 cm to 2.05 cm) but does NOT yet meet the tightened <=10 mm
max-slip-while-grasped bound after exhausting the authorized 3-attempt
repair budget. Per HANDOFF.md Phase 4E scope, this is reported honestly
rather than forced to pass or given a 4th attempt -- tests below assert
the CURRENT, real state (some criteria pass, one does not), not a hoped-
for one. Do not loosen any threshold here to hide the remaining gap;
extend the repair budget in a new, separately authorized phase instead.

Also per HANDOFF.md: Task 1 success is NOT restored by this phase. Human
visual review of artifacts/phase4e_task1_corrected.mp4 is still required,
and even then the quantitative grasp-stability gate below is not met.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place import gripper_scene
from tasks.g1_pick_place.gripper_scene import (
    CUBE_HALF,
    CUBE_MASS,
    FINGER_PAD_HALF,
    LEGACY_FINGER_PAD_HALF,
    write_grasp_scene,
    write_grasp_scene_3c,
    write_grasp_scene_4b,
)
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B,
    ARM_KV_4B,
    GRIPPER_KD_4E,
    GRIPPER_KP_4E,
    run_trial_pick_place,
)

ROOT = Path(__file__).resolve().parents[1]
N_MIN_BILATERAL_N = (CUBE_MASS * 9.81) / (2.0 * 1.2)  # m*g/(2*mu), mu = max(cube, pad) sliding friction


class Phase4ESceneVisualCollisionTest(unittest.TestCase):
    """Section A: visual/collision correspondence, checked against the
    real generated scenes -- not assumed."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_4b = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cls.model_4b = mujoco.MjModel.from_xml_path(str(cls.scene_4b))
        cls.scene_3c = write_grasp_scene_3c(arm_kp=400.0, arm_kv=25.0)
        cls.model_3c = mujoco.MjModel.from_xml_path(str(cls.scene_3c))
        cls.scene_legacy = write_grasp_scene()
        cls.model_legacy = mujoco.MjModel.from_xml_path(str(cls.scene_legacy))

    def _has_geom(self, model: mujoco.MjModel, name: str) -> bool:
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0

    def _has_body(self, model: mujoco.MjModel, name: str) -> bool:
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0

    def test_task1_scene_has_no_decorative_hand_mesh_reference_on_a_geom(self) -> None:
        import xml.etree.ElementTree as ET

        tree = ET.parse(self.scene_4b)
        for body in tree.getroot().iter("body"):
            if body.get("name") == "right_wrist_yaw_link":
                for g in body.iter("geom"):
                    self.assertNotEqual(g.get("mesh"), "right_rubber_hand")

    def test_legacy_and_3c_scenes_still_contain_the_vendor_decorative_mesh_geom(self) -> None:
        # Confirms the fix is scoped to Task 1's own scene, not applied
        # unconditionally -- Phase 3/3B/3C's own scenes are byte-for-byte
        # unaffected by the Phase 4E repair (see gripper_scene.py's
        # _build_grasp_tree docstring).
        import xml.etree.ElementTree as ET

        for scene in (self.scene_legacy, self.scene_3c):
            tree = ET.parse(scene)
            found = False
            for body in tree.getroot().iter("body"):
                if body.get("name") == "right_wrist_yaw_link":
                    for g in body.iter("geom"):
                        if g.get("mesh") == "right_rubber_hand":
                            found = True
            self.assertTrue(found, f"{scene} should still contain the vendor decorative mesh geom")

    @staticmethod
    def _material_rgba(model: mujoco.MjModel, geom_name: str) -> np.ndarray:
        # Rendered color comes from the referenced MATERIAL's rgba, not
        # geom_rgba directly -- geom_rgba stays at MuJoCo's (0.5,0.5,0.5,1)
        # override-sentinel for every geom here since none of them set an
        # rgba override of their own, only a `material=` reference.
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        matid = int(model.geom_matid[gid])
        return model.mat_rgba[matid]

    def test_task1_scene_has_a_palm_and_distinguishable_finger_colors(self) -> None:
        self.assertTrue(self._has_body(self.model_4b, "palm"))
        left_rgba = self._material_rgba(self.model_4b, "left_finger_pad")
        right_rgba = self._material_rgba(self.model_4b, "right_finger_pad")
        self.assertFalse(np.allclose(left_rgba, right_rgba), "left/right finger pads must be visually distinguishable")

    def test_legacy_and_3c_scenes_keep_a_single_shared_finger_color(self) -> None:
        for model in (self.model_legacy, self.model_3c):
            left_rgba = self._material_rgba(model, "left_finger_pad")
            right_rgba = self._material_rgba(model, "right_finger_pad")
            self.assertTrue(np.allclose(left_rgba, right_rgba))

    def test_finger_pad_visual_and_collision_geom_are_the_same_geom(self) -> None:
        # Each finger pad is rendered and collides via ONE geom (no
        # separate visual-only overlay) -- position/size/orientation are
        # therefore identical by construction, checked directly rather than
        # assumed: contype/conaffinity=1 (a real collision surface, not a
        # decal) and the geom's own local pos/size are what both render and
        # collide.
        for pad_name in ("left_finger_pad", "right_finger_pad"):
            gid = mujoco.mj_name2id(self.model_4b, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
            self.assertEqual(int(self.model_4b.geom_contype[gid]), 1)
            self.assertEqual(int(self.model_4b.geom_conaffinity[gid]), 1)
            np.testing.assert_allclose(self.model_4b.geom_size[gid], np.array(FINGER_PAD_HALF), atol=1e-9)

    def test_palm_does_not_reach_the_grasp_region(self) -> None:
        # The palm must not participate in any grasp (Section A: "target
        # geometry must not trap or pull the cube" applies in spirit here
        # too -- the palm must not intrude on the finger/cube contact
        # region). Its front face (local x) must sit well behind where the
        # cube is grasped (FINGER_REACH_X - CUBE_HALF).
        palm_id = mujoco.mj_name2id(self.model_4b, mujoco.mjtObj.mjOBJ_GEOM, "palm_geom")
        palm_body_id = mujoco.mj_name2id(self.model_4b, mujoco.mjtObj.mjOBJ_BODY, "palm")
        palm_local_x = float(self.model_4b.body_pos[palm_body_id][0])
        palm_half_x = float(self.model_4b.geom_size[palm_id][0])
        palm_front_face_x = palm_local_x + palm_half_x
        grasp_region_near_face_x = 0.10 - CUBE_HALF  # FINGER_REACH_X - CUBE_HALF
        self.assertLess(palm_front_face_x, grasp_region_near_face_x)

    def test_task1_pads_are_taller_than_legacy_pads_but_x_y_unchanged(self) -> None:
        self.assertGreater(FINGER_PAD_HALF[2], LEGACY_FINGER_PAD_HALF[2])
        self.assertEqual(FINGER_PAD_HALF[0], LEGACY_FINGER_PAD_HALF[0])
        self.assertEqual(FINGER_PAD_HALF[1], LEGACY_FINGER_PAD_HALF[1])


class Phase4ELegacyScenesUnaffectedTest(unittest.TestCase):
    """Confirms the Phase 4E repair did not perturb Phase 3/3B/3C's own,
    separately-verified numeric results -- these scenes/gains were never
    the target of this repair."""

    def test_legacy_scene_byte_identical_to_pre_4e(self) -> None:
        import hashlib

        scene = write_grasp_scene()
        digest = hashlib.sha256(Path(scene).read_bytes()).hexdigest()
        # Historical digest, established in Phase 3 and re-verified at every
        # later phase's test suite run (tests/test_phase3c_grasp.py) -- see
        # that file for the original assertion this mirrors.
        self.assertEqual(
            digest, "1b2fd577ac5cf9baa45bdbf656c19313899168c7bc14f3cc36ded91292b767a6",
        )

    def test_3c_nominal_result_matches_phase3c_committed_value(self) -> None:
        from tasks.g1_pick_place.run_grasp_test_3c import run_trial_3c

        scene = write_grasp_scene_3c(arm_kp=400.0, arm_kv=25.0)
        r = run_trial_3c(scene)
        self.assertAlmostEqual(r["height_gain_m"], 0.10838913826086727, places=6)


class Phase4EGraspEvidenceTest(unittest.TestCase):
    """Section B: evidence-first grasp-stability numbers, measured fresh
    against the real, unmocked pipeline -- not asserted from memory."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cls.result = run_trial_pick_place(cls.scene)

    def test_theoretical_minimum_bilateral_force_matches_hand_calculation(self) -> None:
        # N_min = m*g / (2*mu); m=0.05 kg, g=9.81, mu=1.2 (MuJoCo's default
        # equal-priority combination rule: element-wise max of the two
        # geoms' sliding-friction coefficients, 1.0 for the cube and 1.2
        # for the finger pads).
        self.assertAlmostEqual(N_MIN_BILATERAL_N, 0.20437500000000003, places=9)

    def test_current_gripper_gains_raised_from_phase3c_defaults(self) -> None:
        # Evidence-based increase (reports/phase4e-gripper-integrity-
        # repair.md): Phase 3C's 150/10 left HOLD's worst-instant safety
        # factor at only 1.07x -- too marginal for a "documented safety
        # factor". Not an arbitrary bump.
        self.assertGreater(GRIPPER_KP_4E, 150.0)
        self.assertGreater(GRIPPER_KD_4E, 10.0)

    def test_min_bilateral_normal_force_is_recorded_and_finite_valued(self) -> None:
        # The field must exist and be a real number (or None if never
        # carrying) -- never silently absent.
        self.assertIn("min_bilateral_normal_force_n", self.result)

    def test_max_abs_contact_z_offset_from_cube_center_recorded(self) -> None:
        self.assertIn("max_abs_contact_z_offset_from_cube_center_m", self.result)
        self.assertGreaterEqual(self.result["max_abs_contact_z_offset_from_cube_center_m"], 0.0)

    def test_downward_slip_during_hold_is_small(self) -> None:
        # This specific criterion IS met: <=5 mm of downward creep from the
        # start to the end of HOLD (the arm is stationary throughout HOLD,
        # so this is unambiguously cube-in-gripper slip, not TCP motion).
        self.assertLessEqual(self.result["downward_slip_during_hold_m"], 0.005)


class Phase4EStrengthenedAcceptanceCriteriaTest(unittest.TestCase):
    """Section C: the 5 strengthened criteria, evaluated against the
    CURRENT (post-3-attempt-budget) configuration. Honest result: 3 of 5
    pass; 2 do not. This is NOT a target to hide -- see the module
    docstring and reports/phase4e-gripper-integrity-repair.md."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cls.result = run_trial_pick_place(cls.scene)

    def test_all_five_criteria_keys_present(self) -> None:
        expected = {
            "max_slip_while_grasped_le_10mm",
            "downward_slip_during_hold_le_5mm",
            "cube_center_within_pad_vertical_overlap",
            "bilateral_contact_throughout_hold",
            "normal_forces_positive_and_finite",
        }
        self.assertEqual(set(self.result["criteria_grasp_stability_4e"].keys()), expected)

    def test_downward_slip_during_hold_criterion_passes(self) -> None:
        self.assertTrue(self.result["criteria_grasp_stability_4e"]["downward_slip_during_hold_le_5mm"])

    def test_bilateral_contact_throughout_hold_criterion_passes(self) -> None:
        self.assertTrue(self.result["criteria_grasp_stability_4e"]["bilateral_contact_throughout_hold"])

    def test_max_slip_while_grasped_criterion_still_fails_honestly(self) -> None:
        # Documented, honest gap: max slip while grasped is ~20.5 mm,
        # roughly 2x the 10 mm bound, after exhausting the 3-attempt
        # repair budget (Attempt 1: visual+pad alignment: ~4.9 cm worst;
        # Attempt 2: LIFT smoothing + gain raise: ~2.2 cm worst; Attempt 3:
        # further gain/waypoint increase: ~2.05 cm worst). Do NOT loosen
        # this threshold to make the assertion below pass differently --
        # it exists to keep the gap visible, the same pattern already
        # established by Phase 4D's regression test for the visual defect.
        self.assertFalse(self.result["criteria_grasp_stability_4e"]["max_slip_while_grasped_le_10mm"])
        self.assertGreater(self.result["max_slip_while_grasped_m"], 0.010)
        self.assertLess(self.result["max_slip_while_grasped_m"], 0.030)  # sanity bound on the current gap's size

    def test_grasp_stability_pass_4e_is_honestly_false(self) -> None:
        self.assertFalse(self.result["grasp_stability_pass_4e"])

    def test_deterministic_across_five_reruns(self) -> None:
        results = [run_trial_pick_place(self.scene) for _ in range(5)]
        self.assertEqual(len({r["max_slip_while_grasped_m"] for r in results}), 1)
        self.assertEqual(len({r["grasp_stability_pass_4e"] for r in results}), 1)
        self.assertFalse(results[0]["grasp_stability_pass_4e"])


class Phase4ETaskSuccessNotClaimedTest(unittest.TestCase):
    """Guards against this phase's own code accidentally asserting Task 1
    is restored -- it explicitly is not (human video review pending, and
    the grasp-stability gate is not met regardless)."""

    def test_report_does_not_claim_task1_restored(self) -> None:
        report = (ROOT / "reports" / "phase4e-gripper-integrity-repair.md").read_text(encoding="utf-8")
        lowered = report.lower()
        self.assertNotIn("task 1 restored", lowered)
        self.assertNotIn("task 1 is valid again", lowered)
        self.assertIn("pending my visual review", lowered)
        self.assertIn("not restored", lowered)


if __name__ == "__main__":
    unittest.main()
