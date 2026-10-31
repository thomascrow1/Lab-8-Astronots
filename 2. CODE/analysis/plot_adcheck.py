"""
plot_adcheck.py
===========================================================
Plot an attitude determination check (command 15) from the KestrelSAT SD card.

Usage:
  python plot_adcheck.py ADCheck_data001.csv [--show]

Outputs <file>.png: measured vs expected sun direction for the 8 light spots,
and the error at each spot against the +/-15 deg pass band. Prints the results.
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

TOL_DEG = 15.0  # same as AD_TOL_DEG in main.cpp ad_check()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    with open(args.csv, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        print(f"{r['step']:>10}  expected {r['expected(deg)']:>7}  measured {r['measured(deg)']:>7}  "
              f"error {r['error(deg)']:>6}  {r['result']}")

    spots = [r for r in rows if r["step"].startswith("SPOT_")]
    if not spots:
        print("No light-spot steps in this file.")
        return
    exp = np.array([float(r["expected(deg)"]) for r in spots])
    meas = np.array([float(r["measured(deg)"]) for r in spots])
    err = np.array([float(r["error(deg)"]) for r in spots])
    passed = np.array([r["result"] == "PASS" for r in spots])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    fig.suptitle(f"Attitude determination check: {os.path.basename(args.csv)}")

    # Unwrap measured next to expected so 359 deg plots next to 0 deg
    meas_plot = exp + err
    ax1.plot([0, 360], [0, 360], color="#8a8984", linewidth=1, linestyle="--", label="perfect")
    ax1.fill_between([0, 360], [-TOL_DEG, 360 - TOL_DEG], [TOL_DEG, 360 + TOL_DEG], color="#2a78d6", alpha=0.12,
                     label=f"±{TOL_DEG:.0f}° pass band")
    ax1.scatter(exp[passed], meas_plot[passed], s=64, color="#2a78d6", marker="o", label="PASS", zorder=3)
    ax1.scatter(exp[~passed], meas_plot[~passed], s=80, color="#e34948", marker="X", label="FAIL", zorder=3)
    for e, m, raw in zip(exp, meas_plot, meas):
        ax1.annotate(f"{raw:.0f}°", (e, m), xytext=(6, -10), textcoords="offset points", fontsize=8, color="#52514e")
    ax1.set_xticks(range(0, 361, 45))
    ax1.set_yticks(range(0, 361, 45))
    ax1.set_xlabel("Expected sun direction (deg)  [0=+X, 90=+Y, 180=-X, 270=-Y]")
    ax1.set_ylabel("Measured sun direction (deg)")
    ax1.set_title("Measured vs expected", loc="left")
    ax1.legend(frameon=False, fontsize=9, loc="upper left")

    ax2.axhspan(-TOL_DEG, TOL_DEG, color="#2a78d6", alpha=0.12)
    ax2.axhline(0, color="#8a8984", linewidth=1)
    ax2.bar(exp, err, width=20, color=np.where(passed, "#2a78d6", "#e34948"))
    for e, v in zip(exp, err):
        ax2.annotate(f"{v:+.1f}", (e, v), xytext=(0, 4 if v >= 0 else -12), textcoords="offset points",
                     ha="center", fontsize=8)
    ax2.set_xticks(range(0, 316, 45))
    ax2.set_xlabel("Expected sun direction (deg)")
    ax2.set_ylabel("Error (deg)")
    ax2.set_title(f"Pointing knowledge error (pass band ±{TOL_DEG:.0f}°)", loc="left")

    for ax in (ax1, ax2):
        ax.grid(True, color="#e6e6e3", linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    out = os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(out, dpi=150)
    print(f"Saved {out}  ({passed.sum()}/{len(passed)} spots PASS)")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
