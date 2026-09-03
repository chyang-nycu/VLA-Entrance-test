#!/usr/bin/env python3
"""Phase 5E: dataset-level validation for data/task1_demonstrations_v1.hdf5.

Checks (Section G):
  - exactly 24 successes and 8 diagnostic failures/rejections (reported
    honestly if not exact -- see Section I, no silent resampling);
  - no duplicate configuration hashes;
  - correct split counts;
  - no seed reused unintentionally;
  - no configuration leakage across splits (disjoint cube_dx bands);
  - instruction distribution;
  - spatial coverage;
  - action distribution and saturation;
  - RGB statistics;
  - episode duration distribution;
  - success rate over all attempted configurations;
  - replay fidelity distributions (not only maxima from one episode).

Also renders the 5 required plots into artifacts/phase5e_dataset_summary/.
"""

from __future__ import annotations

import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import h5py
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from tasks.g1_pick_place.replay_dataset_episode import list_episode_ids, replay_all

ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "data"
LOG_DIR = ROOT / "logs"
ARTIFACT_DIR = ROOT / "artifacts" / "phase5e_dataset_summary"

HDF5_PATH = DATA_DIR / "task1_demonstrations_v1.hdf5"
SPEC_PATH = DATA_DIR / "task1_collection_spec.json"

SPLIT_BANDS = {
    "train": (-0.035, -0.020),
    "val": (-0.020, -0.013),
    "test": (-0.013, -0.005),
}


def _config_hash(cube_xy_offset, target_xy_offset) -> str:
    payload = json.dumps({"cube_xy_offset": list(cube_xy_offset), "target_xy_offset": list(target_xy_offset)}, sort_keys=True)
    return hashlib.sha256(payload.encode()).hexdigest()


def validate(hdf5_path: Path = HDF5_PATH, spec_path: Path = SPEC_PATH) -> dict:
    errors: list[str] = []
    warnings: list[str] = []

    with h5py.File(hdf5_path, "r") as f:
        attempted = json.loads(f.attrs["attempted_configs_json"])
        canonical_manifest_sha256 = f.attrs["canonical_manifest_sha256"]
        decoder_configuration_hash = f.attrs["decoder_configuration_hash"]
        collection_spec_sha256 = f.attrs["collection_spec_sha256"]

    spec_bytes = spec_path.read_bytes()
    live_spec_hash = hashlib.sha256(spec_bytes).hexdigest()
    if collection_spec_sha256 != live_spec_hash:
        errors.append(
            f"dataset's collection_spec_sha256 ({collection_spec_sha256}) does not match the "
            f"live data/task1_collection_spec.json hash ({live_spec_hash})"
        )

    from tasks.g1_pick_place.canonical_config import manifest_hash
    if canonical_manifest_sha256 != manifest_hash():
        errors.append("dataset's canonical_manifest_sha256 does not match the live canonical manifest")

    from tasks.g1_pick_place.policy_action_codec import decoder_configuration_hash as live_decoder_hash
    from tasks.g1_pick_place.record_demonstrations_v2 import SUBSTEPS_PER_TRANSITION
    if decoder_configuration_hash != live_decoder_hash(SUBSTEPS_PER_TRANSITION):
        errors.append("dataset's decoder_configuration_hash does not match the live policy_action_codec decoder")

    # --- counts ---
    n_success = sum(1 for r in attempted if r["outcome"] == "success")
    n_failed_interaction = sum(1 for r in attempted if r["outcome"] == "failed_interaction")
    n_rejected = sum(1 for r in attempted if r["outcome"] == "rejected_by_reachability")
    n_diagnostic = n_failed_interaction + n_rejected
    if n_success != 24 or n_diagnostic != 8:
        warnings.append(
            f"actual yield ({n_success} success / {n_diagnostic} diagnostic: "
            f"{n_failed_interaction} failed_interaction + {n_rejected} rejected) "
            "does not match the intended 24/8 split -- reported honestly per Section I, not resampled."
        )

    # --- seeds / duplicate configs ---
    seeds = [r["seed"] for r in attempted]
    if len(seeds) != len(set(seeds)):
        errors.append("duplicate seed detected in attempted_configs")

    config_hashes = [_config_hash(r["cube_xy_offset"], r["target_xy_offset"]) for r in attempted]
    if len(config_hashes) != len(set(config_hashes)):
        dupes = [h for h, c in Counter(config_hashes).items() if c > 1]
        errors.append(f"duplicate configuration hash(es) detected: {dupes}")

    # --- split counts ---
    split_counts = Counter(r["split"] for r in attempted)
    expected_split_counts = {"train": 16, "val": 4, "test": 4, "diagnostics": 8}
    for split, expected in expected_split_counts.items():
        actual = split_counts.get(split, 0)
        if actual != expected:
            warnings.append(f"split '{split}' has {actual} configs, expected {expected}")

    # --- configuration leakage across splits (disjoint cube_dx bands) ---
    leakage = []
    for r in attempted:
        if r["split"] not in SPLIT_BANDS:
            continue
        lo, hi = SPLIT_BANDS[r["split"]]
        dx = r["cube_xy_offset"][0]
        if not (lo - 1e-9 <= dx <= hi + 1e-9):
            leakage.append({"seed": r["seed"], "split": r["split"], "cube_dx": dx, "expected_band": [lo, hi]})
    if leakage:
        errors.append(f"cube_dx spatial-band leakage detected: {leakage}")
    else:
        # explicit disjointness check between bands themselves
        bands_sorted = sorted(SPLIT_BANDS.items(), key=lambda kv: kv[1][0])
        for (s1, (lo1, hi1)), (s2, (lo2, hi2)) in zip(bands_sorted, bands_sorted[1:]):
            if hi1 > lo2:
                errors.append(f"declared bands for '{s1}' and '{s2}' overlap: {(lo1, hi1)} vs {(lo2, hi2)}")

    # --- instruction distribution ---
    instruction_counts = Counter(r["instruction_utterance"] for r in attempted)

    # --- spatial coverage ---
    spatial_by_split = defaultdict(list)
    for r in attempted:
        spatial_by_split[r["split"]].append(r["cube_xy_offset"])
    spatial_coverage = {
        split: {
            "n": len(vals),
            "cube_dx_min": float(min(v[0] for v in vals)) if vals else None,
            "cube_dx_max": float(max(v[0] for v in vals)) if vals else None,
            "cube_dy_min": float(min(v[1] for v in vals)) if vals else None,
            "cube_dy_max": float(max(v[1] for v in vals)) if vals else None,
        }
        for split, vals in spatial_by_split.items()
    }

    # --- episode duration distribution, RGB stats, action stats (over
    # episodes actually written to the HDF5 -- i.e. everything except
    # rejected_by_reachability, which has no episode group) ---
    ep_ids = list_episode_ids(hdf5_path)
    durations = []
    rgb_means = []
    rgb_stds = []
    blank_frame_count = 0
    action_mag_pos = []
    action_mag_rot = []
    saturated_count = 0
    SATURATION_THRESHOLD_M = 0.05  # 5cm per-sub-chunk delta would be extreme for a 20ms/10-physics-step window

    with h5py.File(hdf5_path, "r") as f:
        for ep_id in ep_ids:
            g = f["episodes"][ep_id]
            durations.append(int(g.attrs["transition_count"]))
            rgb = g["policy"]["observations"]["rgb"][:]
            rgb_means.append(float(rgb.mean()))
            rgb_stds.append(float(rgb.std()))
            if float(rgb.std()) < 1.0:
                blank_frame_count += 1
            dp = g["policy"]["actions"]["tcp_delta_position"][:]
            dr = g["policy"]["actions"]["tcp_delta_orientation"][:]
            mag_p = np.linalg.norm(dp, axis=-1)
            mag_r = np.linalg.norm(dr, axis=-1)
            action_mag_pos.append(mag_p.flatten())
            action_mag_rot.append(mag_r.flatten())
            saturated_count += int(np.sum(mag_p.flatten() > SATURATION_THRESHOLD_M))
            if not np.all(np.isfinite(dp)) or not np.all(np.isfinite(dr)):
                errors.append(f"{ep_id}: non-finite value in policy actions")

    action_mag_pos_all = np.concatenate(action_mag_pos) if action_mag_pos else np.array([])
    action_mag_rot_all = np.concatenate(action_mag_rot) if action_mag_rot else np.array([])

    # --- replay fidelity distributions (Section G: "not only maxima from
    # one episode") ---
    replay_rows = replay_all(hdf5_path)
    exact_max_errs = [r["exact_max_tcp_error_m"] for r in replay_rows]
    policy_max_errs = [r["policy_max_tcp_error_m"] for r in replay_rows]
    policy_within_10mm = [r["policy_within_10mm"] for r in replay_rows]
    n_policy_within_10mm = sum(1 for v in policy_within_10mm if v)

    # --- success rate over ALL attempted configurations ---
    success_rate_all_attempted = n_success / len(attempted) if attempted else 0.0
    success_rate_excl_rejections = (
        n_success / (n_success + n_failed_interaction) if (n_success + n_failed_interaction) else 0.0
    )

    # --- plots ---
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    _plot_spatial_by_split(attempted)
    _plot_success_failure_locations(attempted)
    _plot_episode_lengths(durations)
    _plot_action_magnitude(action_mag_pos_all)
    _plot_replay_tcp_error(policy_max_errs, exact_max_errs)

    result = {
        "hdf5_path": str(hdf5_path.relative_to(ROOT)),
        "n_attempted": len(attempted),
        "n_success": n_success,
        "n_failed_interaction": n_failed_interaction,
        "n_rejected_by_reachability": n_rejected,
        "n_diagnostic_total": n_diagnostic,
        "split_counts": dict(split_counts),
        "instruction_distribution": dict(instruction_counts),
        "spatial_coverage_by_split": spatial_coverage,
        "episode_duration_transitions": {
            "min": int(min(durations)) if durations else None,
            "max": int(max(durations)) if durations else None,
            "mean": float(np.mean(durations)) if durations else None,
            "median": float(np.median(durations)) if durations else None,
        },
        "rgb_statistics": {
            "mean_pixel_value_avg": float(np.mean(rgb_means)) if rgb_means else None,
            "std_pixel_value_avg": float(np.mean(rgb_stds)) if rgb_stds else None,
            "blank_or_near_blank_frames_episodes": blank_frame_count,
        },
        "action_distribution": {
            "tcp_delta_position_magnitude_m": {
                "min": float(action_mag_pos_all.min()) if action_mag_pos_all.size else None,
                "max": float(action_mag_pos_all.max()) if action_mag_pos_all.size else None,
                "mean": float(action_mag_pos_all.mean()) if action_mag_pos_all.size else None,
                "p99": float(np.percentile(action_mag_pos_all, 99)) if action_mag_pos_all.size else None,
            },
            "tcp_delta_orientation_magnitude_rad": {
                "min": float(action_mag_rot_all.min()) if action_mag_rot_all.size else None,
                "max": float(action_mag_rot_all.max()) if action_mag_rot_all.size else None,
                "mean": float(action_mag_rot_all.mean()) if action_mag_rot_all.size else None,
                "p99": float(np.percentile(action_mag_rot_all, 99)) if action_mag_rot_all.size else None,
            },
            "saturation_threshold_m": SATURATION_THRESHOLD_M,
            "n_sub_actions_exceeding_saturation_threshold": saturated_count,
        },
        "success_rate_over_all_attempted_configs": success_rate_all_attempted,
        "success_rate_excluding_reachability_rejections": success_rate_excl_rejections,
        "replay_fidelity": {
            "exact_execution_max_tcp_error_m": {
                "min": float(min(exact_max_errs)) if exact_max_errs else None,
                "max": float(max(exact_max_errs)) if exact_max_errs else None,
                "mean": float(np.mean(exact_max_errs)) if exact_max_errs else None,
            },
            "policy_action_max_tcp_error_m": {
                "min": float(min(policy_max_errs)) if policy_max_errs else None,
                "max": float(max(policy_max_errs)) if policy_max_errs else None,
                "mean": float(np.mean(policy_max_errs)) if policy_max_errs else None,
                "median": float(np.median(policy_max_errs)) if policy_max_errs else None,
            },
            "n_episodes_replayed": len(replay_rows),
            "n_episodes_policy_replay_within_10mm": n_policy_within_10mm,
            "per_episode": replay_rows,
        },
        "errors": errors,
        "warnings": warnings,
        "PASSED": len(errors) == 0,
    }
    return result


def _plot_spatial_by_split(attempted):
    fig, ax = plt.subplots(figsize=(6, 5))
    colors = {"train": "#4C72B0", "val": "#DD8452", "test": "#55A868", "diagnostics": "#8172B2"}
    for split, color in colors.items():
        pts = [r["cube_xy_offset"] for r in attempted if r["split"] == split]
        if not pts:
            continue
        xs, ys = zip(*pts)
        ax.scatter(xs, ys, label=split, color=color, s=40, alpha=0.8)
    ax.set_xlabel("cube_dx (m)")
    ax.set_ylabel("cube_dy (m)")
    ax.set_title("Phase 5E: cube initial position by split\n(target position fixed -- not varied, see spec deviation disclosure)")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ARTIFACT_DIR / "spatial_by_split.png", dpi=130)
    plt.close(fig)


def _plot_success_failure_locations(attempted):
    fig, ax = plt.subplots(figsize=(6, 5))
    colors = {"success": "#2ca02c", "failed_interaction": "#d62728", "rejected_by_reachability": "#7f7f7f"}
    markers = {"success": "o", "failed_interaction": "x", "rejected_by_reachability": "s"}
    for outcome, color in colors.items():
        pts = [r["cube_xy_offset"] for r in attempted if r["outcome"] == outcome]
        if not pts:
            continue
        xs, ys = zip(*pts)
        ax.scatter(xs, ys, label=outcome, color=color, marker=markers[outcome], s=50, alpha=0.85)
    ax.set_xlabel("cube_dx (m)")
    ax.set_ylabel("cube_dy (m)")
    ax.set_title("Phase 5E: outcome by cube initial position")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ARTIFACT_DIR / "success_failure_locations.png", dpi=130)
    plt.close(fig)


def _plot_episode_lengths(durations):
    fig, ax = plt.subplots(figsize=(6, 4))
    if durations:
        ax.hist(durations, bins=min(12, len(set(durations)) or 1), color="#4C72B0", edgecolor="white")
    ax.set_xlabel("episode length (10Hz transitions)")
    ax.set_ylabel("count")
    ax.set_title("Phase 5E: episode length distribution")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ARTIFACT_DIR / "episode_lengths.png", dpi=130)
    plt.close(fig)


def _plot_action_magnitude(action_mag_pos_all):
    fig, ax = plt.subplots(figsize=(6, 4))
    if action_mag_pos_all.size:
        ax.hist(action_mag_pos_all, bins=40, color="#DD8452", edgecolor="white")
    ax.set_xlabel("|tcp_delta_position| per sub-action (m)")
    ax.set_ylabel("count")
    ax.set_title("Phase 5E: action magnitude distribution")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ARTIFACT_DIR / "action_magnitude.png", dpi=130)
    plt.close(fig)


def _plot_replay_tcp_error(policy_max_errs, exact_max_errs):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].hist(policy_max_errs, bins=15, color="#55A868", edgecolor="white")
    axes[0].axvline(0.010, color="red", linestyle="--", label="10mm target")
    axes[0].set_xlabel("max TCP error, policy-action replay (m)")
    axes[0].set_ylabel("count")
    axes[0].legend()
    axes[0].set_title("Policy-action replay error distribution")
    axes[1].hist(np.log10(np.clip(exact_max_errs, 1e-12, None)), bins=15, color="#4C72B0", edgecolor="white")
    axes[1].set_xlabel("log10(max TCP error), exact execution replay (m)")
    axes[1].set_title("Exact execution replay error distribution")
    for a in axes:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ARTIFACT_DIR / "replay_tcp_error.png", dpi=130)
    plt.close(fig)


def main() -> int:
    result = validate(HDF5_PATH, SPEC_PATH)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase5e_validation.json").write_text(json.dumps(result, indent=2) + "\n")
    summary = {k: v for k, v in result.items() if k not in ("replay_fidelity",)}
    print(json.dumps(summary, indent=2))
    print("PASSED" if result["PASSED"] else "FAILED (see errors above)")
    return 0 if result["PASSED"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
