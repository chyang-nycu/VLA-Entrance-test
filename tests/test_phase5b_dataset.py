"""Phase 5B: VLA data pipeline tests.

Generates a small temporary HDF5 dataset (2 short-episode variants) rather
than depending on the full data/task1_prototype.hdf5 artifact being present
in git, per the phase spec.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from tasks.g1_pick_place.camera_observation import CAM_HEIGHT, CAM_WIDTH, HEAD_CAM_PARENT_BODY, write_grasp_scene_5a
from tasks.g1_pick_place.canonical_config import (
    ManifestMismatchError, compute_manifest_hash, load_manifest,
    manifest_hash, verify_environment_matches_manifest,
)
from tasks.g1_pick_place.record_demonstrations import (
    ARM_KP_4B, ARM_KV_4B, GRIPPER_KD_4E, GRIPPER_KP_4E, POLICY_HZ,
    SUBSTEPS_PER_TRANSITION, TIMESTEP, _write_hdf5, collect_episode,
)
from tasks.g1_pick_place.replay_demonstration import replay_actions, visualize_episode
from tasks.g1_pick_place.validate_dataset import validate_file

ROOT = Path(__file__).resolve().parents[1]


class TestCanonicalManifest(unittest.TestCase):
    def test_manifest_loads_and_hash_matches(self) -> None:
        m = load_manifest()
        self.assertEqual(m["hash"]["value"], compute_manifest_hash(m))

    def test_manifest_hash_helper_matches_load(self) -> None:
        self.assertEqual(manifest_hash(), load_manifest()["hash"]["value"])

    def test_tampered_manifest_content_fails_hash_check(self) -> None:
        m = load_manifest()
        m["task_instruction"] = "tampered"
        self.assertNotEqual(compute_manifest_hash(m), m["hash"]["value"])

    def test_verify_environment_matches_manifest_accepts_current_config(self) -> None:
        verify_environment_matches_manifest(
            scene_generator_name="write_grasp_scene_5a",
            use_oriented_ik=False,
            arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B,
            gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
            camera_parent_body=HEAD_CAM_PARENT_BODY,
            camera_resolution_wh=(CAM_WIDTH, CAM_HEIGHT),
        )

    def test_verify_environment_rejects_wrong_use_oriented_ik(self) -> None:
        with self.assertRaises(ManifestMismatchError):
            verify_environment_matches_manifest(
                scene_generator_name="write_grasp_scene_5a",
                use_oriented_ik=True,
                arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B,
                gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
                camera_parent_body=HEAD_CAM_PARENT_BODY,
                camera_resolution_wh=(CAM_WIDTH, CAM_HEIGHT),
            )

    def test_verify_environment_rejects_wrong_gains(self) -> None:
        with self.assertRaises(ManifestMismatchError):
            verify_environment_matches_manifest(
                scene_generator_name="write_grasp_scene_5a",
                use_oriented_ik=False,
                arm_kp=999.0, arm_kv=ARM_KV_4B,
                gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
                camera_parent_body=HEAD_CAM_PARENT_BODY,
                camera_resolution_wh=(CAM_WIDTH, CAM_HEIGHT),
            )

    def test_verify_environment_rejects_wrong_camera_parent(self) -> None:
        with self.assertRaises(ManifestMismatchError):
            verify_environment_matches_manifest(
                scene_generator_name="write_grasp_scene_5a",
                use_oriented_ik=False,
                arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B,
                gripper_kp=GRIPPER_KP_4E, gripper_kd=GRIPPER_KD_4E,
                camera_parent_body="head_link",
                camera_resolution_wh=(CAM_WIDTH, CAM_HEIGHT),
            )

    def test_manifest_camera_terminology_does_not_claim_head_body(self) -> None:
        m = load_manifest()
        desc = m["camera"]["description"].lower()
        self.assertIn("torso-mounted onboard rgb camera positioned near head height", desc)
        self.assertIn("torso_link", desc)
        self.assertEqual(m["camera"]["parent_body"], "torso_link")

    def test_manifest_identifies_non_oriented_controller(self) -> None:
        m = load_manifest()
        self.assertFalse(m["controller"]["use_oriented_ik"])
        self.assertIn("solve_ik_waypoint", m["controller"]["ik_function"])
        self.assertNotIn("oriented", m["controller"]["ik_function"].split(".")[-1])

    def test_manifest_scene_lineage_is_phase4e_not_phase4f(self) -> None:
        m = load_manifest()
        self.assertIn("write_grasp_scene_4b", m["scene"]["underlying_functional_scene_function"])
        self.assertIn("g1_grasp_scene_4f.xml", m["scene"]["explicitly_excludes"])


class _TinyDatasetMixin:
    """Builds a 2-episode dataset with a tiny artificial SUBSTEPS override is
    not needed -- both real Task 1 episodes are short (nominal/failure), so
    we just collect the real nominal + the real x_plus_0.03 failure variant
    (fast: ~2s wall) directly through the real pipeline, exactly like
    record_demonstrations.py, but into a temp file instead of data/.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.hdf5_path = Path(cls.tmpdir.name) / "tiny_test_dataset.hdf5"
        scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        ep_success = collect_episode(scene_path, "nominal", (0.0, 0.0))
        ep_fail = collect_episode(scene_path, "x_plus_0.03", (0.03, 0.0))
        _write_hdf5([ep_success, ep_fail], cls.hdf5_path)
        cls.ep_success = ep_success
        cls.ep_fail = ep_fail

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmpdir.cleanup()


class TestTransitionConvention(_TinyDatasetMixin, unittest.TestCase):
    def test_observations_are_one_more_than_actions(self) -> None:
        for ep in (self.ep_success, self.ep_fail):
            self.assertEqual(ep["n_observations"], ep["n_transitions"] + 1)

    def test_hdf5_group_shapes_respect_convention(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            for variant in ("nominal", "x_plus_0.03"):
                g = f["episodes"][variant]
                n_obs = g["policy_observations"]["rgb"].shape[0]
                n_act = g["actions"]["cartesian_target"].shape[0]
                self.assertEqual(n_obs, n_act + 1)
                self.assertEqual(int(g.attrs["transition_count"]), n_act)

    def test_root_attrs_document_transition_convention(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            self.assertEqual(f.attrs["transition_convention"], "observation_t -> action_t -> physics_substeps -> observation_t+1")
            self.assertEqual(int(f.attrs["substeps_per_transition"]), SUBSTEPS_PER_TRANSITION)
            self.assertAlmostEqual(float(f.attrs["policy_control_hz"]), POLICY_HZ)
            self.assertAlmostEqual(float(f.attrs["physics_hz"]), 1.0 / TIMESTEP)

    def test_substeps_per_transition_is_consistent_with_hz(self) -> None:
        self.assertEqual(SUBSTEPS_PER_TRANSITION, round((1.0 / TIMESTEP) / POLICY_HZ))


class TestHDF5Schema(_TinyDatasetMixin, unittest.TestCase):
    def test_policy_observations_excludes_privileged_fields(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            po_keys = set(f["episodes"]["nominal"]["policy_observations"].keys())
            self.assertNotIn("cube_pos", po_keys)
            self.assertNotIn("cube_quat", po_keys)
            self.assertNotIn("bilateral_contact", po_keys)

    def test_privileged_group_is_separate_top_level_group(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            g = f["episodes"]["nominal"]
            self.assertIn("privileged", g)
            self.assertNotIn("privileged", g["policy_observations"])

    def test_rgb_is_uint8_and_correct_shape(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            rgb = f["episodes"]["nominal"]["policy_observations"]["rgb"]
            self.assertEqual(rgb.dtype, np.uint8)
            self.assertEqual(rgb.shape[1:], (CAM_HEIGHT, CAM_WIDTH, 3))

    def test_rgb_frames_are_nonblank_with_temporal_variance(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            rgb = f["episodes"]["nominal"]["policy_observations"]["rgb"][:]
        self.assertGreater(rgb.std(), 1.0)
        first, last = rgb[0].astype(int), rgb[-1].astype(int)
        self.assertGreater(np.abs(first - last).mean(), 0.5)

    def test_red_cube_and_blue_target_visible_in_first_frame(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            frame = f["episodes"]["nominal"]["policy_observations"]["rgb"][0]
        r, g, b = frame[..., 0].astype(int), frame[..., 1].astype(int), frame[..., 2].astype(int)
        red_mask = (r > 120) & (r - g > 40) & (r - b > 40)
        blue_mask = (b > 120) & (b - r > 30)
        self.assertGreater(int(red_mask.sum()), 20)
        self.assertGreater(int(blue_mask.sum()), 100)

    def test_success_episode_has_expected_attrs(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            g = f["episodes"]["nominal"]
            self.assertTrue(bool(g.attrs["success"]))
            self.assertTrue(bool(g.attrs["train_eligible"]))
            self.assertEqual(g.attrs["termination_reason"], "DONE")

    def test_failure_episode_has_expected_attrs_and_is_excluded(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            self.assertFalse(bool(g.attrs["success"]))
            self.assertFalse(bool(g.attrs["train_eligible"]))
            self.assertEqual(g.attrs["termination_reason"], "SETTLE_APPROACH")

    def test_episode_stores_manifest_hash(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            self.assertEqual(f.attrs["canonical_manifest_sha256"], manifest_hash())
            self.assertEqual(f["episodes"]["nominal"].attrs["canonical_manifest_sha256"], manifest_hash())

    def test_action_group_has_all_required_datasets(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            act = f["episodes"]["nominal"]["actions"]
            for name in ("cartesian_target", "arm_joint_position_target", "gripper_target", "applied_ctrl"):
                self.assertIn(name, act)


class TestValidator(_TinyDatasetMixin, unittest.TestCase):
    def test_valid_dataset_passes(self) -> None:
        result = validate_file(self.hdf5_path)
        self.assertEqual(result["errors"], [])

    def test_validator_recomputes_success_independently(self) -> None:
        # A tampered "success" flag on the failing episode must be caught by
        # re-running the deterministic simulation, not merely reading the flag.
        import shutil
        tampered = self.hdf5_path.parent / "tampered.hdf5"
        shutil.copy(self.hdf5_path, tampered)
        with h5py.File(tampered, "r+") as f:
            f["episodes"]["x_plus_0.03"].attrs["success"] = True
            f["episodes"]["x_plus_0.03"].attrs["train_eligible"] = True
        result = validate_file(tampered)
        self.assertTrue(any("disagrees with independently recomputed" in e for e in result["errors"]))

    def test_validator_catches_failure_episode_wrongly_marked_train_eligible(self) -> None:
        import shutil
        tampered = self.hdf5_path.parent / "tampered2.hdf5"
        shutil.copy(self.hdf5_path, tampered)
        with h5py.File(tampered, "r+") as f:
            f["episodes"]["x_plus_0.03"].attrs["train_eligible"] = True
        result = validate_file(tampered)
        self.assertTrue(any("train_eligible=True" in e for e in result["errors"]))

    def test_validator_catches_missing_group(self) -> None:
        import shutil
        tampered = self.hdf5_path.parent / "tampered3.hdf5"
        shutil.copy(self.hdf5_path, tampered)
        with h5py.File(tampered, "r+") as f:
            del f["episodes"]["nominal"]["privileged"]
        result = validate_file(tampered)
        self.assertTrue(any("missing group privileged" in e for e in result["errors"]))

    def test_validator_catches_non_monotonic_timestamps(self) -> None:
        import shutil
        tampered = self.hdf5_path.parent / "tampered4.hdf5"
        shutil.copy(self.hdf5_path, tampered)
        with h5py.File(tampered, "r+") as f:
            ts = f["episodes"]["nominal"]["policy_observations"]["timestamps"][:]
            ts[1] = ts[0]
            f["episodes"]["nominal"]["policy_observations"]["timestamps"][:] = ts
        result = validate_file(tampered)
        self.assertTrue(any("monotonic" in e for e in result["errors"]))

    def test_validator_catches_bad_quaternion_norm(self) -> None:
        import shutil
        tampered = self.hdf5_path.parent / "tampered5.hdf5"
        shutil.copy(self.hdf5_path, tampered)
        with h5py.File(tampered, "r+") as f:
            pose = f["episodes"]["nominal"]["policy_observations"]["tcp_pose"][:]
            pose[0, 3:7] = [5.0, 0.0, 0.0, 0.0]
            f["episodes"]["nominal"]["policy_observations"]["tcp_pose"][:] = pose
        result = validate_file(tampered)
        self.assertTrue(any("quaternion norms" in e for e in result["errors"]))

    def test_dataset_reopens_successfully(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            keys = list(f["episodes"].keys())
        self.assertEqual(set(keys), {"nominal", "x_plus_0.03"})
        with h5py.File(self.hdf5_path, "r") as f2:
            self.assertEqual(set(f2["episodes"].keys()), set(keys))

    def test_no_post_init_cube_jump(self) -> None:
        result = validate_file(self.hdf5_path)
        jump_errors = [e for e in result["errors"] if "jumped" in e]
        self.assertEqual(jump_errors, [])


class TestReplay(_TinyDatasetMixin, unittest.TestCase):
    def test_replay_success_episode_within_tolerance(self) -> None:
        result = replay_actions(self.hdf5_path, "nominal")
        self.assertTrue(result["within_tolerance"], result)
        self.assertGreater(result["max_cube_error_m"], -1)  # sanity: field present and numeric

    def test_replay_failure_episode_within_tolerance(self) -> None:
        result = replay_actions(self.hdf5_path, "x_plus_0.03")
        self.assertTrue(result["within_tolerance"], result)

    def test_replay_rejects_manifest_hash_mismatch(self) -> None:
        import shutil
        tampered = self.hdf5_path.parent / "tampered_manifest.hdf5"
        shutil.copy(self.hdf5_path, tampered)
        with h5py.File(tampered, "r+") as f:
            f.attrs["canonical_manifest_sha256"] = "0" * 64
        with self.assertRaises(ManifestMismatchError):
            replay_actions(tampered, "nominal")

    def test_visualize_episode_produces_image(self) -> None:
        out = Path(self.tmpdir.name) / "viz.png"
        result_path = visualize_episode(self.hdf5_path, "nominal", out_path=out)
        self.assertTrue(result_path.exists())
        self.assertGreater(result_path.stat().st_size, 0)


class TestTask1CriteriaUnchanged(unittest.TestCase):
    """Phase 5B must not retune Task 1 or its success thresholds."""

    def test_success_thresholds_in_manifest_match_run_pick_place_constants(self) -> None:
        from tasks.g1_pick_place import run_pick_place as rp
        m = load_manifest()["success_thresholds"]
        self.assertEqual(m["cube_linear_speed_tol_mps"], rp.CUBE_LINEAR_SPEED_TOL)
        self.assertEqual(m["cube_angular_speed_tol_radps"], rp.CUBE_ANGULAR_SPEED_TOL)
        self.assertEqual(m["cube_supported_height_tol_m"], rp.CUBE_SUPPORTED_HEIGHT_TOL)
        self.assertEqual(m["retreat_disturbance_tol_m"], rp.RETREAT_DISTURBANCE_TOL_M)
        self.assertEqual(m["task_success_dwell_s"], rp.TASK_SUCCESS_DWELL_S)
        self.assertEqual(m["task_success_max_wait_s"], rp.TASK_SUCCESS_MAX_WAIT_S)


if __name__ == "__main__":
    unittest.main()
