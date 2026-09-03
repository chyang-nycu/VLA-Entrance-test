"""Phase 5C: two-rate (policy/execution) replay-fidelity tests.

Generates a small temporary HDF5 dataset (real nominal success + real
x_plus_0.03 failure, same fast pattern as tests/test_phase5b_dataset.py's
_TinyDatasetMixin) rather than depending on the full
data/task1_prototype_v2.hdf5 artifact being present in git.

Covers the Section E transition-alignment properties plus a deliberately
shifted-action tamper test that must fail validate_dataset_v2.validate_file.
"""

from __future__ import annotations

import copy
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from tasks.g1_pick_place.camera_observation import write_grasp_scene_5a
from tasks.g1_pick_place.canonical_config import ManifestMismatchError, load_manifest
from tasks.g1_pick_place.record_demonstrations_v2 import (
    ARM_KP_4B, ARM_KV_4B, EXECUTION_HZ, POLICY_HZ, SUBSTEPS_PER_TRANSITION,
    TIMESTEP, _write_hdf5, collect_episode,
)
from tasks.g1_pick_place.replay_demonstration_v2 import (
    replay_exact_execution, replay_policy_actions, visualize_episode,
)
from tasks.g1_pick_place.validate_dataset_v2 import validate_file

ROOT = Path(__file__).resolve().parents[1]


class _TinyV2DatasetMixin:
    """Real nominal success + real x_plus_0.03 failure (fast: ~11s wall
    total), collected through the real two-rate pipeline into a temp file."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.hdf5_path = Path(cls.tmpdir.name) / "tiny_v2_dataset.hdf5"
        scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        ep_success = collect_episode(scene_path, "nominal", (0.0, 0.0))
        ep_fail = collect_episode(scene_path, "x_plus_0.03", (0.03, 0.0))
        _write_hdf5([ep_success, ep_fail], cls.hdf5_path)
        cls.ep_success = ep_success
        cls.ep_fail = ep_fail

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmpdir.cleanup()


class TestTimingConstants(unittest.TestCase):
    def test_substeps_per_transition_is_consistent(self) -> None:
        self.assertEqual(SUBSTEPS_PER_TRANSITION, round((1.0 / TIMESTEP) / POLICY_HZ))

    def test_execution_hz_is_physics_hz(self) -> None:
        self.assertAlmostEqual(EXECUTION_HZ, 1.0 / TIMESTEP)


class TestTransitionAlignment(_TinyV2DatasetMixin, unittest.TestCase):
    """Section E: the seven required transition-alignment properties."""

    def test_observations_length_is_policy_actions_length_plus_one(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            for ep_name in f["episodes"]:
                g = f["episodes"][ep_name]
                n_obs = g["policy"]["observations"]["rgb"].shape[0]
                n_act = g["policy"]["high_level_actions"]["cartesian_target"].shape[0]
                self.assertEqual(n_obs, n_act + 1, ep_name)

    def test_every_execution_step_maps_to_exactly_one_policy_transition(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            for ep_name in f["episodes"]:
                g = f["episodes"][ep_name]
                n_act = g["policy"]["high_level_actions"]["cartesian_target"].shape[0]
                trans_idx = g["execution"]["transition_index"][:]
                if trans_idx.size == 0:
                    continue
                self.assertTrue(np.all(trans_idx >= 0))
                self.assertTrue(np.all(trans_idx < n_act))
                expected = np.repeat(np.arange(n_act), SUBSTEPS_PER_TRANSITION)
                np.testing.assert_array_equal(trans_idx, expected, err_msg=ep_name)

    def test_execution_timestamps_fall_within_their_transition_interval(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            for ep_name in f["episodes"]:
                g = f["episodes"][ep_name]
                obs_t = g["policy"]["observations"]["timestamps"][:]
                exec_t = g["execution"]["timestamps"][:]
                n_act = g["policy"]["high_level_actions"]["cartesian_target"].shape[0]
                for k in range(n_act):
                    block = exec_t[k * SUBSTEPS_PER_TRANSITION:(k + 1) * SUBSTEPS_PER_TRANSITION]
                    if block.size == 0:
                        continue
                    self.assertGreater(block.min(), obs_t[k] - 1e-9, ep_name)
                    self.assertLessEqual(block.max(), obs_t[k + 1] + 1e-9, ep_name)

    def test_final_execution_state_agrees_with_final_observation(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            for ep_name in f["episodes"]:
                g = f["episodes"][ep_name]
                if g["execution"]["joint_positions"].shape[0] == 0:
                    continue
                final_exec_joint = g["execution"]["joint_positions"][-1]
                final_obs_joint = g["policy"]["observations"]["joint_positions"][-1]
                np.testing.assert_allclose(final_exec_joint, final_obs_joint, atol=1e-5, err_msg=ep_name)
                final_exec_tcp = g["execution"]["tcp_pose"][-1, :3]
                final_obs_tcp = g["policy"]["observations"]["tcp_pose"][-1, :3]
                np.testing.assert_allclose(final_exec_tcp, final_obs_tcp, atol=1e-5, err_msg=ep_name)

    def test_no_action_is_shifted_by_one_policy_step(self) -> None:
        """A correctly-aligned cartesian_target[k] must equal what
        _phase_target() derives from execution's own recorded phase label
        for transition k's last row -- a one-step shift would compare
        cartesian_target[k] against transition k-1 or k+1's phase instead
        and fail this check (this is also what the tamper test below
        exercises via validate_dataset_v2, using a real off-by-one shift)."""
        from tasks.g1_pick_place.gripper_scene import CUBE_POS
        from tasks.g1_pick_place.record_demonstrations_v2 import _base_phase, _phase_target
        from tasks.g1_pick_place.run_grasp_test_3c import LIFT_DZ

        with h5py.File(self.hdf5_path, "r") as f:
            for ep_name in f["episodes"]:
                g = f["episodes"][ep_name]
                n_act = g["policy"]["high_level_actions"]["cartesian_target"].shape[0]
                if n_act == 0:
                    continue
                offset = tuple(g.attrs["cube_xy_offset"])
                cube_pos = np.array([CUBE_POS[0] + offset[0], CUBE_POS[1] + offset[1], CUBE_POS[2]])
                lift_target = cube_pos + np.array([0.0, 0.0, LIFT_DZ])
                exec_phase = g["execution"]["phase"][:]
                cartesian_target = g["policy"]["high_level_actions"]["cartesian_target"][:]
                for k in range(n_act):
                    last_row = (k + 1) * SUBSTEPS_PER_TRANSITION - 1
                    phase_label = exec_phase[last_row].decode("utf-8")
                    expected = _phase_target(_base_phase(phase_label), cube_pos, lift_target)
                    np.testing.assert_allclose(expected, cartesian_target[k], atol=1e-6, err_msg=f"{ep_name} k={k}")

    def test_terminal_transition_represented_consistently(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            for ep_name in f["episodes"]:
                g = f["episodes"][ep_name]
                n_act = g["policy"]["high_level_actions"]["cartesian_target"].shape[0]
                n_exec = g["execution"]["transition_index"].shape[0]
                self.assertEqual(n_exec, n_act * SUBSTEPS_PER_TRANSITION, ep_name)
                self.assertEqual(int(g.attrs["transition_count"]), n_act, ep_name)
                self.assertEqual(int(g.attrs["execution_row_count"]), n_exec, ep_name)

    def test_reconstructed_actions_match_stored_targets_at_sampling_instant(self) -> None:
        # Same invariant as test_no_action_is_shifted_by_one_policy_step,
        # phrased as the user's Section E wording ("reconstructed 10 Hz
        # actions match stored controller targets at the declared sampling
        # instant") -- kept as a separate named test per the spec's list.
        self.test_no_action_is_shifted_by_one_policy_step()


class TestShiftedActionTamper(_TinyV2DatasetMixin, unittest.TestCase):
    def test_manually_shifted_action_array_fails_validation(self) -> None:
        import shutil
        tampered_path = Path(self.tmpdir.name) / "tampered_shifted.hdf5"
        shutil.copyfile(self.hdf5_path, tampered_path)
        with h5py.File(tampered_path, "r+") as f:
            g = f["episodes"]["nominal"]
            targets = g["policy"]["high_level_actions"]["cartesian_target"][:]
            self.assertGreater(targets.shape[0], 1)
            shifted = np.roll(targets, shift=1, axis=0)
            del g["policy"]["high_level_actions"]["cartesian_target"]
            g["policy"]["high_level_actions"].create_dataset("cartesian_target", data=shifted)

        result = validate_file(tampered_path)
        self.assertGreater(len(result["errors"]), 0)
        self.assertTrue(any("does not match" in e or "shifted" in e or "corrupted" in e for e in result["errors"]))


class TestReplayModes(_TinyV2DatasetMixin, unittest.TestCase):
    def test_exact_execution_replay_is_near_machine_precision(self) -> None:
        result = replay_exact_execution(self.hdf5_path, "nominal")
        self.assertTrue(result["within_tolerance"])
        self.assertLess(result["max_tcp_error_m"], 1e-3)
        self.assertLess(result["max_joint_error_rad"], 1e-4)

    def test_exact_execution_replay_matches_stored_success(self) -> None:
        result_success = replay_exact_execution(self.hdf5_path, "nominal")
        result_fail = replay_exact_execution(self.hdf5_path, "x_plus_0.03")
        self.assertEqual(result_success["stored_success"], True)
        self.assertEqual(result_fail["stored_success"], False)

    def test_policy_action_replay_runs_and_reports_honest_numbers(self) -> None:
        # This does NOT assert within_tolerance -- Section D requires
        # reporting the true measured number, not forcing a pass. See
        # reports/phase5c-replay-fidelity.md for the analysis of why the
        # max (not final) policy-replay TCP error currently exceeds 10mm.
        result = replay_policy_actions(self.hdf5_path, "nominal")
        self.assertIn("max_tcp_error_m", result)
        self.assertGreaterEqual(result["max_tcp_error_m"], 0.0)

    def test_visualize_episode_produces_image_without_stepping_physics(self) -> None:
        out = visualize_episode(self.hdf5_path, "nominal", out_path=Path(self.tmpdir.name) / "viz.png")
        self.assertTrue(out.exists())

    def test_replay_rejects_manifest_hash_mismatch(self) -> None:
        import shutil
        tampered_path = Path(self.tmpdir.name) / "tampered_hash.hdf5"
        shutil.copyfile(self.hdf5_path, tampered_path)
        with h5py.File(tampered_path, "r+") as f:
            f.attrs["canonical_manifest_sha256"] = "0" * 64
        with self.assertRaises(ManifestMismatchError):
            replay_exact_execution(tampered_path, "nominal")


class TestValidatorOnRealDataset(_TinyV2DatasetMixin, unittest.TestCase):
    def test_valid_tiny_dataset_passes(self) -> None:
        result = validate_file(self.hdf5_path)
        self.assertEqual(result["errors"], [])

    def test_dataset_reopens_successfully(self) -> None:
        with h5py.File(self.hdf5_path, "r") as f:
            names = list(f["episodes"].keys())
        with h5py.File(self.hdf5_path, "r") as f2:
            self.assertEqual(list(f2["episodes"].keys()), names)


if __name__ == "__main__":
    unittest.main()
