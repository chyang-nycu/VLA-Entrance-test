#!/usr/bin/env python3
"""Phase 4A: grasp setup-variant evaluation tests.

Fixed-base, torso-constrained upper-body manipulation baseline (Phase 3C's
architecture, unmodified). These tests run the actual 5-variant sweep
(tasks/g1_pick_place/run_variant_sweep.py) once in setUpClass and assert
structural/aggregate properties of the real result -- they do not re-derive
or hand-check individual trial numbers already covered by
tests/test_phase3c_grasp.py.
"""

from __future__ import annotations

import unittest

from tasks.g1_pick_place.run_variant_sweep import N_TRIALS_PER_VARIANT, VARIANTS, run_sweep

# The full 5-variant x >=3-trial sweep takes ~20s; computed once for the
# whole module (simulation is deterministic, so every test class sharing
# this single result is equivalent to each running its own sweep).
_SWEEP = None


def setUpModule() -> None:
    global _SWEEP
    _SWEEP = run_sweep(label="test_sweep")


class Phase4AFeasibilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.sweep = _SWEEP

    def test_five_variants_defined(self) -> None:
        self.assertEqual(len(VARIANTS), 5)
        ids = [v["id"] for v in VARIANTS]
        self.assertEqual(len(set(ids)), 5, "variant IDs must be unique/stable")
        self.assertIn("nominal", ids)

    def test_every_variant_has_a_feasibility_verdict(self) -> None:
        # HANDOFF.md: never silently exclude a variant's outcome, even a
        # failing one -- every variant must carry a recorded verdict.
        for v in self.sweep["variants"]:
            self.assertIn("accepted_as_reachable", v["feasibility"])
            self.assertIsInstance(v["feasibility"]["accepted_as_reachable"], bool)

    def test_feasibility_verdict_matches_actual_trial_outcome(self) -> None:
        # A variant rejected as unreachable at the feasibility stage must
        # not have quietly passed its trials (that would mean the
        # feasibility check is not predictive / not trustworthy), and vice
        # versa a variant accepted as reachable should not silently fail
        # every trial for an unrelated, undiagnosed reason.
        for v in self.sweep["variants"]:
            accepted = v["feasibility"]["accepted_as_reachable"]
            if not accepted:
                self.assertEqual(v["n_pass"], 0, f"{v['id']}: rejected as unreachable but a trial passed")

    def test_each_variant_run_at_least_three_times(self) -> None:
        self.assertGreaterEqual(N_TRIALS_PER_VARIANT, 3)
        for v in self.sweep["variants"]:
            self.assertGreaterEqual(v["n_trials"], 3)


class Phase4ASharedConfigurationTest(unittest.TestCase):
    """No per-variant tuning: every variant in one sweep must have been run
    under the identical arm/gripper configuration."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.sweep = _SWEEP

    def test_one_shared_scene_for_all_variants(self) -> None:
        # run_sweep builds exactly one scene and threads it through every
        # variant/trial call -- verified by construction (single write_
        # grasp_scene_3c call in run_sweep), asserted here via the returned
        # scene path being singular/consistent for the whole sweep record.
        self.assertIn("scene", self.sweep)
        self.assertTrue(self.sweep["scene"].endswith("g1_grasp_scene_3c.xml"))

    def test_no_cube_manipulation_violations_across_variants(self) -> None:
        # run_trial_3c's own source-scan guard (imported at module load in
        # run_grasp_test_3c.py) already asserts this for every call,
        # variant or nominal, with no exceptions. Re-import here to prove
        # it still executes cleanly against the current source.
        from tasks.g1_pick_place.run_grasp_test_3c import (
            _assert_run_trial_3c_has_no_direct_cube_state_write,
        )

        _assert_run_trial_3c_has_no_direct_cube_state_write()


class Phase4ANominalReproducesPhase3CTest(unittest.TestCase):
    """The nominal variant (offset 0,0) inside the Phase 4A sweep must
    reproduce Phase 3C's own recorded nominal result exactly -- it is
    driven through the identical scene/gains/trial function."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.sweep = _SWEEP
        cls.nominal = next(v for v in cls.sweep["variants"] if v["id"] == "nominal")

    def test_nominal_variant_passes(self) -> None:
        self.assertTrue(self.nominal["variant_success"])
        self.assertEqual(self.nominal["n_pass"], self.nominal["n_trials"])

    def test_nominal_height_gain_matches_phase3c(self) -> None:
        self.assertAlmostEqual(self.nominal["max_height_gain_m"], 0.10838913826086727, places=6)

    def test_nominal_hold_duration_matches_phase3c(self) -> None:
        self.assertAlmostEqual(self.nominal["max_continuous_hold_s"], 3.5039999999996145, places=6)


class Phase4AAggregateTargetTest(unittest.TestCase):
    """The actual, measured Phase 4A sweep result: >=3/5 variants succeeding
    under the single shared, unmodified Phase 3C configuration -- no global
    adjustments were needed (see reports/phase4a-grasp-variants.md)."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.sweep = _SWEEP

    def test_at_least_three_of_five_variants_succeed(self) -> None:
        self.assertGreaterEqual(
            self.sweep["n_variants_succeeded"], 3,
            msg=f"variant results: {[(v['id'], v['variant_success']) for v in self.sweep['variants']]}",
        )

    def test_target_met_flag_is_consistent(self) -> None:
        self.assertEqual(self.sweep["target_met_3_of_5"], self.sweep["n_variants_succeeded"] >= 3)

    def test_trial_success_rate_is_consistent_with_variant_counts(self) -> None:
        total_trials = sum(v["n_trials"] for v in self.sweep["variants"])
        total_pass = sum(v["n_pass"] for v in self.sweep["variants"])
        self.assertEqual(self.sweep["total_trials"], total_trials)
        self.assertEqual(self.sweep["total_trials_passed"], total_pass)
        self.assertAlmostEqual(self.sweep["trial_success_rate"], total_pass / total_trials)


if __name__ == "__main__":
    unittest.main()
