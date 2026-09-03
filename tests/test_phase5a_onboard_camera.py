#!/usr/bin/env python3
"""Phase 5A: onboard RGB observation camera tests.

Covers the camera-pose math (synthetic), the scene-generator's isolation
from every existing Phase 4B-4F scene, and a real, freshly-simulated
nominal Task 1 episode's onboard-camera visibility, physics-invariance,
and pose-consistency properties -- not cached/logged numbers (this file
re-simulates; it does not read logs/phase5a_camera_smoke.json as its
source of truth, though its assertions are consistent with that log).
"""
from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from tasks.g1_pick_place import gripper_scene as gs
from tasks.g1_pick_place.camera_observation import (
    CAM_HEIGHT,
    CAM_WIDTH,
    HEAD_CAM_NAME,
    HEAD_CAM_PARENT_BODY,
    _look_at_quat,
    blue_target_mask,
    camera_extrinsic,
    camera_intrinsics,
    red_cube_mask,
    write_grasp_scene_5a,
)
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, run_trial_pick_place


class OrientationMathUnitTest(unittest.TestCase):
    """Synthetic checks on _look_at_quat() -- independent of any simulation."""

    def test_look_at_along_negative_z_produces_identity(self) -> None:
        # Camera at origin looking straight down world -Z with world +Y up
        # should need no rotation at all relative to MuJoCo's own
        # camera-looks-down-local–Z convention.
        q = _look_at_quat(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.0]), up_ref=np.array([0.0, 1.0, 0.0]))
        self.assertAlmostEqual(abs(q[0]), 1.0, places=6)

    def test_look_at_quat_is_unit_norm(self) -> None:
        q = _look_at_quat(np.array([0.1, 0.2, 1.3]), np.array([0.3, -0.1, 0.7]))
        self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=6)

    def test_camera_intrinsics_focal_length_matches_fovy_formula(self) -> None:
        intr = camera_intrinsics(width=160, height=120, fovy_deg=90.0)
        expected_fy = 120 / (2.0 * np.tan(np.radians(90.0) / 2.0))
        self.assertAlmostEqual(intr["fy"], expected_fy, places=6)
        self.assertAlmostEqual(intr["fx"], intr["fy"], places=6)
        self.assertAlmostEqual(intr["cx"], 80.0, places=6)
        self.assertAlmostEqual(intr["cy"], 60.0, places=6)


class ColorMaskUnitTest(unittest.TestCase):
    """Synthetic checks on the smoke-test-only color masks: they must not
    fire on plausible non-target colors (sky-blue, table-tan, robot-white)."""

    def test_red_mask_fires_on_pure_red_not_on_table_tan(self) -> None:
        red = np.zeros((4, 4, 3), dtype=np.uint8)
        red[..., 0] = 200
        self.assertTrue(bool(red_cube_mask(red).all()))
        table = np.zeros((4, 4, 3), dtype=np.uint8)
        table[...] = [143, 105, 69]  # sampled table color, see report
        self.assertFalse(bool(red_cube_mask(table).any()))

    def test_blue_mask_fires_on_target_blue_not_on_sky(self) -> None:
        target = np.zeros((4, 4, 3), dtype=np.uint8)
        target[...] = [25, 89, 230]  # rgba (0.1, 0.35, 0.9) approx sRGB
        self.assertTrue(bool(blue_target_mask(target).all()))
        sky = np.zeros((4, 4, 3), dtype=np.uint8)
        sky[...] = [92, 137, 184]  # sampled sky color, see report
        self.assertFalse(bool(blue_target_mask(sky).any()))


class SceneIsolationTest(unittest.TestCase):
    """The onboard camera must be additive-only: every existing scene
    generator's own output must be untouched."""

    def test_write_grasp_scene_4b_unaffected_by_camera_module_import(self) -> None:
        p = gs.write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        tree = ET.parse(p)
        names = [c.get("name") for body in tree.getroot().iter("body") for c in body.findall("camera")]
        self.assertNotIn(HEAD_CAM_NAME, names)

    def test_5a_scene_has_exactly_one_new_camera_vs_4b(self) -> None:
        base = gs.write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cam_scene = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        base_cams = {c.get("name") for c in ET.parse(base).getroot().iter("camera")}
        cam_scene_cams = {c.get("name") for c in ET.parse(cam_scene).getroot().iter("camera")}
        self.assertEqual(cam_scene_cams - base_cams, {HEAD_CAM_NAME})

    def test_head_camera_is_child_of_torso_link_not_vendor_modified(self) -> None:
        cam_scene = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        tree = ET.parse(cam_scene)
        found_parent = None
        for body in tree.getroot().iter("body"):
            for c in body.findall("camera"):
                if c.get("name") == HEAD_CAM_NAME:
                    found_parent = body.get("name")
        self.assertEqual(found_parent, HEAD_CAM_PARENT_BODY)

    def test_vendor_g1_xml_unchanged(self) -> None:
        p = ROOT / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1" / "g1_29dof.xml"
        content = p.read_text()
        self.assertNotIn(HEAD_CAM_NAME, content)

    def test_camera_has_no_depth_or_segmentation_attributes(self) -> None:
        cam_scene = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        tree = ET.parse(cam_scene)
        cam_el = [c for c in tree.getroot().iter("camera") if c.get("name") == HEAD_CAM_NAME][0]
        # An RGB-only <camera> element carries no rendering-mode attribute
        # that would select a depth or segmentation buffer -- that choice
        # is made by the Python renderer (`mujoco.Renderer(..., depth=False)`
        # by default), not by the MJCF element itself; this test simply
        # documents that no such extra attribute was added here.
        self.assertIsNone(cam_el.get("mode"))


class OnboardCameraRealTrialTest(unittest.TestCase):
    """Runs one real, fresh nominal Task 1 episode (Phase 4B/4C's
    completing configuration -- see record_onboard_camera_episode.py's
    module docstring for why Phase 4F's config is not used here: it does
    not reach OPEN/VERIFY_TASK_SUCCESS) and checks onboard-camera
    visibility, physics-invariance, and pose-consistency directly."""

    REQUIRED_PHASES = [
        "PREGRASP", "APPROACH", "CLOSE", "LIFT", "HOLD",
        "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET", "OPEN", "VERIFY_TASK_SUCCESS",
    ]

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene))
        cls.cam_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_CAMERA, HEAD_CAM_NAME)
        cls.renderer = mujoco.Renderer(cls.model, height=CAM_HEIGHT, width=CAM_WIDTH)
        cls.cube_body_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        cls.cube_geom_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        cls.left_pad_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
        cls.right_pad_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")

        cls.named_frames: dict[str, np.ndarray] = {}
        cls.cam_positions: list[list[float]] = []
        first_contact = [False]

        def has_bilateral_contact(cb_data: mujoco.MjData) -> bool:
            left_ok = right_ok = False
            for ci in range(cb_data.ncon):
                con = cb_data.contact[ci]
                pair = (int(con.geom1), int(con.geom2))
                if cls.cube_geom_id in pair and cls.left_pad_id in pair:
                    left_ok = True
                if cls.cube_geom_id in pair and cls.right_pad_id in pair:
                    right_ok = True
            return left_ok and right_ok

        def frame_callback(phase, cb_model, cb_data):
            if not first_contact[0] and has_bilateral_contact(cb_data):
                cls.renderer.update_scene(cb_data, camera=cls.cam_id)
                cls.named_frames["FIRST_BILATERAL_CONTACT"] = cls.renderer.render().copy()
                first_contact[0] = True
            for tp in cls.REQUIRED_PHASES:
                if phase.startswith(tp) and tp not in cls.named_frames:
                    cls.renderer.update_scene(cb_data, camera=cls.cam_id)
                    cls.named_frames[tp] = cls.renderer.render().copy()
            cls.cam_positions.append(camera_extrinsic(cb_model, cb_data, cls.cam_id)["position_world"])

        data0 = mujoco.MjData(cls.model)
        mujoco.mj_resetData(cls.model, data0)
        mujoco.mj_forward(cls.model, data0)
        cls.renderer.update_scene(data0, camera=cls.cam_id)
        cls.named_frames["RESET"] = cls.renderer.render().copy()

        cls.result = run_trial_pick_place(cls.scene, frame_callback=frame_callback, use_oriented_ik=False)
        cls.result_no_render = run_trial_pick_place(cls.scene, use_oriented_ik=False)

    def test_task_completes(self) -> None:
        self.assertTrue(self.result["task_pass"])

    def test_all_required_phases_captured(self) -> None:
        expected = {"RESET", "FIRST_BILATERAL_CONTACT"} | set(self.REQUIRED_PHASES)
        self.assertEqual(expected - set(self.named_frames.keys()), set())

    def test_frame_shape_and_dtype(self) -> None:
        for frame in self.named_frames.values():
            self.assertEqual(frame.shape, (CAM_HEIGHT, CAM_WIDTH, 3))
            self.assertEqual(frame.dtype, np.uint8)

    def test_frames_finite_and_in_range(self) -> None:
        for frame in self.named_frames.values():
            self.assertTrue(np.all(np.isfinite(frame)))
            self.assertGreaterEqual(int(frame.min()), 0)
            self.assertLessEqual(int(frame.max()), 255)

    def test_frames_not_blank(self) -> None:
        for name, frame in self.named_frames.items():
            self.assertGreater(float(frame.std()), 1.0, f"{name} frame looks blank")

    def test_cube_visible_at_every_required_phase(self) -> None:
        for name in ["RESET", "PREGRASP", "APPROACH", "CLOSE", "FIRST_BILATERAL_CONTACT", "LIFT", "HOLD",
                     "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET", "OPEN", "VERIFY_TASK_SUCCESS"]:
            self.assertGreater(int(red_cube_mask(self.named_frames[name]).sum()), 0, f"cube not visible at {name}")

    def test_target_visible_before_placement(self) -> None:
        for name in ["RESET", "PREGRASP", "APPROACH", "CLOSE", "LIFT", "HOLD",
                     "TRANSPORT_ABOVE_TARGET", "LOWER_TO_TARGET"]:
            self.assertGreater(int(blue_target_mask(self.named_frames[name]).sum()), 0, f"target not visible at {name}")

    def test_rendering_does_not_alter_physics(self) -> None:
        self.assertEqual(self.result["height_gain_m"], self.result_no_render["height_gain_m"])
        self.assertEqual(self.result["task_pass"], self.result_no_render["task_pass"])
        self.assertEqual(self.result["final_xy_target_error_m"], self.result_no_render["final_xy_target_error_m"])

    def test_camera_pose_consistent_with_fixed_base_parent(self) -> None:
        # torso_link is welded to the world: the camera's world position
        # must track it, i.e. stay constant to within solver precision
        # (soft weld, not bit-exact) -- not drift or jump independently.
        positions = np.array(self.cam_positions)
        max_dev = float(np.abs(positions - positions[0]).max())
        self.assertLess(max_dev, 1e-3)


if __name__ == "__main__":
    unittest.main()
