"""Phase 7E: render the door-pull diagnostic time series.

Six variables, incompatible units and scales (deg, mm, dimensionless,
dimensionless, N, mm) -- per the dataviz skill's "one axis" rule, these are
small multiples sharing a time axis, never overlaid on one y-scale. Each
panel is a single series (ink-colored line, no legend needed); the
zero-bilateral-force window is shaded consistently across every panel so
temporal correlation is visible directly, not asserted.
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
SHADE = "#ef4444"
SHADE_ALPHA = 0.10
REF = "#9ca3af"


def main() -> int:
    d = json.loads((ROOT / "logs" / "phase7e_pull_diagnostics.json").read_text())
    log = [l for l in d["log"] if l["phase"].startswith("PULL_ARC")]
    t = np.array([l["t"] for l in log])
    t0 = t[0]
    t = t - t0
    hinge = np.array([l["hinge_deg"] for l in log])
    err = np.array([l["tcp_err_mm"] for l in log])
    cond = np.array([l["condition_number"] for l in log])
    smin = np.array([l["sigma_min"] for l in log])
    force = np.array([l["min_bilateral_force_n"] for l in log])
    orient = np.array([l["orientation_residual_deg"] for l in log])
    slip = np.array([l["slip_mm"] for l in log])

    zero_force = force <= 1e-9

    fig, axes = plt.subplots(6, 1, figsize=(9, 13), sharex=True)
    panels = [
        (axes[0], hinge, "Door hinge angle (deg)"),
        (axes[1], err, "TCP position error vs. commanded target (mm)"),
        (axes[2], cond, "Jacobian condition number (log scale)"),
        (axes[3], smin, "Jacobian min. singular value"),
        (axes[4], force, "Min. bilateral contact force (N)"),
        (axes[5], slip, "Grasp slip (mm)"),
    ]
    for ax, series, label in panels:
        # Shade every contiguous zero-force run once, across all panels.
        in_run = False
        for i in range(len(t)):
            if zero_force[i] and not in_run:
                start = t[i]
                in_run = True
            if in_run and (i == len(t) - 1 or not zero_force[i + 1]):
                ax.axvspan(start, t[i], color=SHADE, alpha=SHADE_ALPHA, lw=0)
                in_run = False
        ax.plot(t, series, color=INK, lw=1.4)
        ax.set_ylabel(label, fontsize=8)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.tick_params(labelsize=8)
        ax.grid(axis="y", color=REF, alpha=0.3, lw=0.5)

    axes[2].set_yscale("log")
    axes[5].axhline(10.0, color=SHADE, lw=1, ls="--", alpha=0.6)
    axes[5].annotate("10mm target", (t[-1], 10.0), fontsize=7, color=SHADE, ha="right", va="bottom")
    axes[-1].set_xlabel("Time since PULL_ARC start (s)", fontsize=9)
    fig.suptitle(
        "Task 3 door pull — nominal trial (shaded: zero bilateral contact force)",
        fontsize=10, y=0.995,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    out = ROOT / "artifacts" / "phase7e_pull_diagnostics.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
