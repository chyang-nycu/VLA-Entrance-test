#!/usr/bin/env python3
"""Phase 5E: per-episode replay for the scaled dataset
(data/task1_demonstrations_v1.hdf5).

Thin wrapper around Phase 5D's UNCHANGED replay_demonstration_v3 functions
(replay_exact_execution, replay_policy_actions) -- those functions already
operate generically on any episode group name and any cube_xy_offset stored
in the episode's own attrs, so no replay LOGIC is duplicated here; this
module only adapts argument plumbing (episode ids are "seed_<seed>" here,
not "nominal"/"x_minus_0.03"/etc.) and adds a batch mode that replays every
episode in the dataset (used by validate_scaled_dataset.py for the
dataset-level replay-fidelity distribution Section G requires).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py

from tasks.g1_pick_place.replay_demonstration_v3 import (
    replay_exact_execution, replay_policy_actions, visualize_episode,
)

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATASET = ROOT / "data" / "task1_demonstrations_v1.hdf5"


def list_episode_ids(hdf5_path: Path) -> list[str]:
    with h5py.File(hdf5_path, "r") as f:
        return sorted(f["episodes"].keys(), key=lambda s: int(s.split("_")[1]))


def replay_all(hdf5_path: Path) -> list[dict]:
    """Runs BOTH exact and policy replay for every episode in the dataset.
    Used by validate_scaled_dataset.py -- Section G requires 'replay
    fidelity distributions, not only maxima from one episode.'
    """
    results = []
    for ep_id in list_episode_ids(hdf5_path):
        with h5py.File(hdf5_path, "r") as f:
            g = f["episodes"][ep_id]
            seed = int(g.attrs["seed"])
            split = str(g.attrs["split"])
            success = bool(g.attrs["success"])
        exact = replay_exact_execution(hdf5_path, ep_id)
        row = {
            "episode_id": ep_id, "seed": seed, "split": split, "stored_success": success,
            "exact_max_tcp_error_m": exact["max_tcp_error_m"],
            "exact_within_tolerance": exact["within_tolerance"],
        }
        # Policy-action replay is only meaningful for episodes with at
        # least one full 10Hz transition (a config rejected pre-physics has
        # no episode group at all; a physical failure in the very first
        # SETTLE phase can still have >=1 transition and is replayed too).
        policy = replay_policy_actions(hdf5_path, ep_id)
        row["policy_max_tcp_error_m"] = policy["max_tcp_error_m"]
        row["policy_final_tcp_error_m"] = policy["final_tcp_error_m"]
        row["policy_within_10mm"] = policy["within_tolerance"]
        row["policy_first_divergence_transition"] = policy["first_divergence_transition"]
        row["policy_first_divergence_phase"] = policy["first_divergence_phase"]
        results.append(row)
        print(f"[{ep_id}] exact_max={exact['max_tcp_error_m']:.3e}m policy_max={policy['max_tcp_error_m']:.4f}m "
              f"policy_within_10mm={policy['within_tolerance']}")
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["exact", "policy", "visualize", "all"])
    parser.add_argument("--dataset", default=str(DEFAULT_DATASET))
    parser.add_argument("--episode", help="episode group id, e.g. seed_1000 (required for exact/policy/visualize)")
    args = parser.parse_args()

    dataset = Path(args.dataset)
    if args.mode == "all":
        results = replay_all(dataset)
        print(json.dumps(results, indent=2))
        return 0

    if not args.episode:
        parser.error("--episode is required for mode " + args.mode)

    if args.mode == "exact":
        result = replay_exact_execution(dataset, args.episode)
    elif args.mode == "policy":
        result = replay_policy_actions(dataset, args.episode)
    else:
        out = visualize_episode(dataset, args.episode, out_path=ROOT / "artifacts" / "phase5e_dataset_summary" / f"{args.episode}_visualize.png")
        result = {"visualization_path": str(out)}
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
