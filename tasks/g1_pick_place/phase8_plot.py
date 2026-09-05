"""Phase 8: render the comprehensive slip-onset diagnostic plot.

Eight small multiples (incompatible units preclude a shared y-axis, per
the dataviz skill's "one axis" rule), sharing a time axis, with the
event-timeline markers overlaid as vertical lines so temporal precedence
is visible directly rather than asserted.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
INK = "#1f2937"
EVENT_COLORS = {
    "slip > 5mm": "#f59e0b",
    "slip > 10mm": "#ef4444",
    "first major contact-force decline (<90% plateau, sustained)": "#8b5cf6",
    "first exact-zero force": "#dc2626",
}


def main() -> int:
    d = json.loads((ROOT / "logs" / "phase8_baseline_full.json").read_text())
    ev = json.loads((ROOT / "logs" / "phase8_event_timeline.json").read_text())
    log = [l for l in d["log"] if l["phase"].startswith("PULL_ARC")]
    t = np.array([l["t"] for l in log]); t0 = t[0]; t = t - t0

    series = {
        "Hinge angle (deg)": np.array([l["hinge_deg"] for l in log]),
        "TCP error vs. commanded (mm)": np.array([l["tcp_err_mm"] for l in log]),
        "Joint tracking error, max (deg)": np.array([l["joint_err_max_deg"] for l in log]),
        "Wrist yaw (deg)": np.array([l["yaw_deg"] for l in log]),
        "Jacobian condition number": np.array([l["condition_number"] for l in log]),
        "Min. bilateral contact force (N)": np.array([l["min_bilateral_normal_n"] for l in log]),
        "Friction margin, N_avail-N_req (N)": np.array([l["friction_margin_n"] for l in log]),
        "Grasp slip (mm)": np.array([l["slip_mm"] for l in log]),
    }

    fig, axes = plt.subplots(len(series), 1, figsize=(9, 16), sharex=True)
    for ax, (label, vals) in zip(axes, series.items()):
        ax.plot(t, vals, color=INK, lw=1.3)
        ax.set_ylabel(label, fontsize=7.5)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.tick_params(labelsize=7.5)
        ax.grid(axis="y", color="#9ca3af", alpha=0.3, lw=0.5)
        for name, color in EVENT_COLORS.items():
            tv = ev.get(name)
            if tv is not None:
                ax.axvline(tv, color=color, lw=1, ls="--", alpha=0.7)

    axes[4].set_yscale("log")
    axes[6].axhline(0, color="#6b7280", lw=1)
    axes[-1].axhline(10.0, color="#ef4444", lw=1, ls=":", alpha=0.6)
    axes[-1].set_xlabel("Time since PULL_ARC start (s)", fontsize=9)

    handles = [plt.Line2D([0], [0], color=c, lw=1.5, ls="--", label=n) for n, c in EVENT_COLORS.items()]
    fig.legend(handles=handles, loc="lower center", ncol=1, fontsize=7, bbox_to_anchor=(0.78, 0.93))
    fig.suptitle("Task 3 slip-onset diagnostic — baseline trial (arm_kp=600, gripper_kp=320)", fontsize=10, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    out = ROOT / "artifacts" / "phase8_slip_onset_diagnostic.png"
    fig.savefig(out, dpi=150)
    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
