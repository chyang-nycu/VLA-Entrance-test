#!/usr/bin/env python3
"""Phase 5D tests: redesigned reference-relative, chunked VLA policy actions.

Covers: known synthetic TCP deltas, rotation composition order/frame, frame
conversions, no off-by-one action shift, no phase-goal repetition, decoder
interpolation across exactly SUB_STEPS_PER_SUBACTION physics steps, terminal
action/padding behavior, causality/leakage, manifest and decoder-hash
mismatch rejection, successful policy replay meeting the <=10mm tolerance,
and exact execution replay remaining unchanged from Phase 5C.

Structural/tamper tests build a small temporary HDF5 file (one short real
episode) rather than depending on the full data/task1_prototype_v3.hdf5
artifact being present in git. Replay-mode regression tests (exact-execution
and policy-action fidelity) use the real checked-in v3 dataset directly,
consistent with Phase 5C's precedent (tests/test_phase5c_replay_fidelity.py).
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import h5py
import mujoco
import numpy as np

from tasks.g1_pick_place import policy_action_codec as pac
from tasks.g1_pick_place import record_demonstrations_v3 as rec_v3
from tasks.g1_pick_place import replay_demonstration_v3 as rep_v3
from tasks.g1_pick_place import validate_dataset_v3 as val_v3
from tasks.g1_pick_place.camera_observation import write_grasp_scene_5a
from tasks.g1_pick_place.canonical_config import load_manifest
from tasks.g1_pick_place.controller import RIGHT_ARM_ACTUATORS, RIGHT_ARM_JOINTS, TCP_SITE, JointMap
from tasks.g1_pick_place.record_demonstrations_v2 import collect_episode
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B

ROOT = Path(__file__).resolve().parents[1]
REAL_V3_PATH = ROOT / "data" / "task1_prototype_v3.hdf5"


def _build_small_dataset(tmpdir: Path) -> Path:
    """One real, short episode (x_plus_0.03 -- the fastest to collect, ~31
    transitions) written through the actual v3 pipeline, for structural/
    tamper tests that don't need the full 3-episode artifact.
    """
    scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
    ep = collect_episode(scene_path, "x_plus_0.03", (0.03, 0.0))
    ep["policy_actions"] = rec_v3._derive_policy_actions(scene_path, ep)
    out_path = tmpdir / "tiny_v3.hdf5"
    rec_v3._write_hdf5([ep], scene_path, out_path)
    return out_path


class TestSyntheticDeltasAndRotation(unittest.TestCase):
    def test_known_synthetic_position_delta_recovered_exactly(self):
        pos_t = np.array([1.0, 2.0, 3.0])
        quat_t = np.array([1.0, 0.0, 0.0, 0.0])
        delta = np.array([0.01, -0.02, 0.005])
        pos_t1 = pos_t + delta
        dp, _ = pac.encode_delta(pos_t, quat_t, pos_t1, quat_t)
        np.testing.assert_allclose(dp, delta, atol=1e-6)
        recon_pos, _ = pac.decode_target(pos_t, quat_t, dp, np.zeros(3))
        np.testing.assert_allclose(recon_pos, pos_t1, atol=1e-6)

    def test_rotation_composition_order_is_body_frame_right_multiplication(self):
        """Verifies (does not assume) that sub_quat/quat_integrate use
        right/body-frame composition: qa = qb (x) axisAngle2Quat(r)."""
        qb = np.array([0.7, 0.1, 0.2, 0.3])
        qb /= np.linalg.norm(qb)
        axis = np.array([0.2, 0.5, -0.1])
        axis /= np.linalg.norm(axis)
        angle = 0.15
        rot = np.zeros(4)
        mujoco.mju_axisAngle2Quat(rot, axis, angle)

        def mul(q1, q2):
            r = np.zeros(4)
            mujoco.mju_mulQuat(r, q1, q2)
            return r

        qa_left = mul(rot, qb)   # world/left composition
        qa_right = mul(qb, rot)  # body/right composition

        r_left = pac.sub_quat(qa_left, qb)
        r_right = pac.sub_quat(qa_right, qb)

        # sub_quat must match the RIGHT-composed case's rotation vector
        # exactly (body-frame convention), not the left-composed one.
        np.testing.assert_allclose(r_right, axis * angle, atol=1e-9)
        self.assertGreater(np.linalg.norm(r_left - axis * angle), 1e-3)

    def test_sub_quat_quat_integrate_are_exact_inverses(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            qa = rng.normal(size=4)
            qa /= np.linalg.norm(qa)
            qb = rng.normal(size=4)
            qb /= np.linalg.norm(qb)
            r = pac.sub_quat(qa, qb)
            recon = pac.quat_integrate(qb, r, 1.0)
            # quaternion double-cover: recon == qa or -qa
            self.assertTrue(
                np.allclose(recon, qa, atol=1e-8) or np.allclose(recon, -qa, atol=1e-8)
            )

    def test_frame_documentation_matches_implementation(self):
        self.assertIn("world", pac.POSITION_FRAME)
        self.assertIn("subQuat", pac.ORIENTATION_FRAME)
        self.assertIn("quatIntegrate", pac.ORIENTATION_FRAME)


class TestDecoderChunking(unittest.TestCase):
    def test_chunk_constants_are_consistent(self):
        self.assertEqual(
            pac.SUB_ACTIONS_PER_TRANSITION * pac.SUB_STEPS_PER_SUBACTION,
            pac.SUBSTEPS_PER_TRANSITION,
        )
        self.assertEqual(pac.SUBSTEPS_PER_TRANSITION, 50)
        self.assertEqual(pac.SUB_STEPS_PER_SUBACTION, 10)

    def test_ramp_joint_targets_produces_exactly_n_substeps_entries(self):
        scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        ik_scratch = mujoco.MjData(model)
        nominal_q = np.zeros(len(arm_map.names))
        current = arm_map.get_qpos(data).copy()
        target_pos = data.site_xpos[site_id].copy() + np.array([0.01, 0.0, 0.0])
        ramp, q_target = pac.ramp_joint_targets(
            model, ik_scratch, data.qpos.copy(), arm_map, site_id, nominal_q,
            current, target_pos, pac.SUB_STEPS_PER_SUBACTION,
        )
        self.assertEqual(len(ramp), pac.SUB_STEPS_PER_SUBACTION)
        np.testing.assert_allclose(ramp[-1], q_target, atol=1e-9)

    def test_decoder_config_hash_reflects_chunk_size(self):
        h5 = pac.decoder_configuration_hash(50, 5)
        h1 = pac.decoder_configuration_hash(50, 1)
        self.assertNotEqual(h5, h1)


class TestSmallDatasetStructural(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.path = _build_small_dataset(Path(cls.tmpdir.name))

    @classmethod
    def tearDownClass(cls):
        cls.tmpdir.cleanup()

    def test_validator_passes_on_freshly_collected_dataset(self):
        result = val_v3.validate_file(self.path)
        self.assertEqual(result["errors"], [], msg=result["errors"])

    def test_action_chunk_shapes(self):
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            n = g.attrs["transition_count"]
            act = g["policy"]["actions"]
            self.assertEqual(act["tcp_delta_position"].shape, (n, 5, 3))
            self.assertEqual(act["tcp_delta_orientation"].shape, (n, 5, 3))
            self.assertEqual(act["gripper_command"].shape, (n,))
            self.assertEqual(act["next_arm_joint_target"].shape, (n, 7))

    def test_observations_length_is_actions_length_plus_one(self):
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            n_obs = g["policy"]["observations"]["rgb"].shape[0]
            n_act = g["policy"]["actions"]["gripper_command"].shape[0]
            self.assertEqual(n_obs, n_act + 1)

    def test_terminal_action_padding_not_needed_every_transition_is_full_chunk(self):
        """Every stored transition has a full H=5 chunk -- execution/ is
        truncated to whole 50-step blocks only, so no partial/padded chunk
        exists anywhere in the stored dataset (Section D padding note)."""
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            n_act = g["policy"]["actions"]["gripper_command"].shape[0]
            n_exec = g["execution"]["transition_index"].shape[0]
            self.assertEqual(n_exec, n_act * 50)
            self.assertTrue(np.all(np.isfinite(g["policy"]["actions"]["tcp_delta_position"][:])))

    def test_causality_next_arm_joint_target_derivable_from_own_interval_only(self):
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            act = g["policy"]["actions"]
            execu = g["execution"]
            n_act = act["gripper_command"].shape[0]
            expected = execu["arm_joint_target"][49::50][:n_act]
            np.testing.assert_allclose(act["next_arm_joint_target"][:], expected, atol=1e-6)

    def test_privileged_fields_absent_from_declared_policy_input(self):
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            declared = json.loads(g["policy"]["actions"].attrs["declared_vla_action_fields"])
            self.assertEqual(set(declared), {"tcp_delta_position", "tcp_delta_orientation", "gripper_command"})
            self.assertNotIn("cube_pos", declared)
            self.assertNotIn("target_pos", declared)
            # privileged/ is a structurally separate group from policy/observations/
            self.assertNotIn("cube_pos", g["policy"]["observations"])

    def test_success_label_and_termination_reason_not_derivable_from_action_fields(self):
        """The action arrays themselves carry no success/failure flag or
        future-state encoding -- only TCP deltas, orientation deltas, and a
        gripper scalar."""
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            act_keys = set(g["policy"]["actions"].keys())
            self.assertEqual(
                act_keys,
                {"tcp_delta_position", "tcp_delta_orientation", "gripper_command", "next_arm_joint_target", "state_machine_phase"},
            )

    def test_no_off_by_one_action_shift_first_action_matches_first_interval(self):
        """The recorded action for transition 0 must reproduce the true
        commanded delta between the reset boundary and transition 0's own
        end -- not transition 1's (an off-by-one shift)."""
        with h5py.File(self.path, "r") as f:
            g = f["episodes"]["x_plus_0.03"]
            obs_joint0 = g["policy"]["observations"]["joint_positions"][0]
            exec_target = g["execution"]["arm_joint_target"][:]
            dp = g["policy"]["actions"]["tcp_delta_position"][0].sum(axis=0)  # sum of 5 sub-deltas = whole-interval delta

        scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        scratch = mujoco.MjData(model)
        mujoco.mj_resetData(model, scratch)
        template = scratch.qpos.copy()
        pos0, _ = pac.forward_kinematics_tcp(model, template, arm_map, site_id, obs_joint0, scratch=scratch)
        pos_end, _ = pac.forward_kinematics_tcp(model, template, arm_map, site_id, exec_target[49], scratch=scratch)
        np.testing.assert_allclose(dp, pos_end - pos0, atol=1e-5)


class TestNoPhaseGoalRepetition(unittest.TestCase):
    def test_real_v3_dataset_does_not_repeat_a_static_goal(self):
        """Direct regression test against Phase 5C's bug: no long run of
        byte-identical deltas during a smooth (LIFT/TRANSPORT/LOWER) phase."""
        if not REAL_V3_PATH.exists():
            self.skipTest("data/task1_prototype_v3.hdf5 not present")
        result = val_v3.validate_file(REAL_V3_PATH)
        self.assertEqual(result["errors"], [], msg=result["errors"])

    def test_tampered_dataset_with_repeated_static_goal_fails_validation(self):
        with tempfile.TemporaryDirectory() as td:
            path = _build_small_dataset(Path(td))
            tampered = Path(td) / "tampered.hdf5"
            import shutil
            shutil.copy(path, tampered)
            with h5py.File(tampered, "r+") as f:
                g = f["episodes"]["x_plus_0.03"]
                act = g["policy"]["actions"]
                phases = [p.decode("utf-8") for p in act["state_machine_phase"][:]]
                # Force-relabel 6 consecutive transitions as a smooth phase
                # with an identical repeated delta, mimicking the old bug.
                if len(phases) >= 6:
                    for i in range(6):
                        phases[i] = "LIFT"
                    del act["state_machine_phase"]
                    act.create_dataset("state_machine_phase", data=np.array([p.encode("utf-8") for p in phases]))
                    dp = act["tcp_delta_position"][:]
                    dp[1:6] = dp[0]
                    del act["tcp_delta_position"]
                    act.create_dataset("tcp_delta_position", data=dp)
            result = val_v3.validate_file(tampered)
            self.assertTrue(any("static-per-phase-goal" in e or "byte-identical" in e for e in result["errors"]), result["errors"])


class TestManifestAndDecoderHashMismatch(unittest.TestCase):
    def test_replay_rejects_canonical_manifest_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            path = _build_small_dataset(Path(td))
            tampered = Path(td) / "tampered_manifest.hdf5"
            import shutil
            shutil.copy(path, tampered)
            with h5py.File(tampered, "r+") as f:
                f.attrs["canonical_manifest_sha256"] = "0" * 64
            from tasks.g1_pick_place.canonical_config import ManifestMismatchError
            with self.assertRaises(ManifestMismatchError):
                rep_v3.replay_exact_execution(tampered, "x_plus_0.03")

    def test_replay_rejects_decoder_configuration_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            path = _build_small_dataset(Path(td))
            tampered = Path(td) / "tampered_decoder.hdf5"
            import shutil
            shutil.copy(path, tampered)
            with h5py.File(tampered, "r+") as f:
                f.attrs["decoder_configuration_hash"] = "0" * 64
            with self.assertRaises(rep_v3.DecoderMismatchError):
                rep_v3.replay_policy_actions(tampered, "x_plus_0.03")

    def test_validator_flags_decoder_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            path = _build_small_dataset(Path(td))
            tampered = Path(td) / "tampered_decoder2.hdf5"
            import shutil
            shutil.copy(path, tampered)
            with h5py.File(tampered, "r+") as f:
                f.attrs["decoder_configuration_hash"] = "0" * 64
            result = val_v3.validate_file(tampered)
            self.assertTrue(any("decoder_configuration_hash" in e for e in result["errors"]), result["errors"])


class TestReplayFidelityRegression(unittest.TestCase):
    """Regression tests against the real, committed v3 dataset -- same
    precedent as Phase 5C's own replay regression tests."""

    def setUp(self):
        if not REAL_V3_PATH.exists():
            self.skipTest("data/task1_prototype_v3.hdf5 not present")

    def test_exact_execution_replay_still_near_machine_precision(self):
        result = rep_v3.replay_exact_execution(REAL_V3_PATH, "nominal")
        self.assertTrue(result["within_tolerance"])
        self.assertLess(result["max_tcp_error_m"], 1e-6)

    def test_policy_action_replay_meets_10mm_target_on_nominal(self):
        result = rep_v3.replay_policy_actions(REAL_V3_PATH, "nominal")
        self.assertTrue(result["within_tolerance"], msg=result)
        self.assertLessEqual(result["max_tcp_error_m"], 0.010)

    def test_policy_action_replay_meets_10mm_target_on_x_minus(self):
        result = rep_v3.replay_policy_actions(REAL_V3_PATH, "x_minus_0.03")
        self.assertTrue(result["within_tolerance"], msg=result)
        self.assertLessEqual(result["max_tcp_error_m"], 0.010)

    def test_failure_episode_replay_reports_but_does_not_require_10mm(self):
        """x_plus_0.03 is a pre-grasp reachability failure -- Section E
        requires identical failure stage/label where applicable, not the
        10mm bar (that only applies to successful episodes)."""
        result = rep_v3.replay_policy_actions(REAL_V3_PATH, "x_plus_0.03")
        self.assertFalse(result["stored_success"])


class TestFullSuiteDidNotRetuneTask1(unittest.TestCase):
    def test_manifest_physical_content_unchanged(self):
        manifest = load_manifest()
        self.assertEqual(manifest["controller"]["use_oriented_ik"], False)
        self.assertEqual(manifest["controller"]["arm_gains"]["kp"], ARM_KP_4B)
        self.assertEqual(manifest["controller"]["arm_gains"]["kv"], ARM_KV_4B)


if __name__ == "__main__":
    unittest.main()
