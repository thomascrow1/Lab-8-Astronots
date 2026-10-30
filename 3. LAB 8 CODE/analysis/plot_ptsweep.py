"""
plot_ptsweep.py
===========================================================
Plot a phototransistor resistor sweep (command 16) and pick the best resistor.

Usage:
  python plot_ptsweep.py PTSweep_data001.csv
  python plot_ptsweep.py PTSweep_data001.csv PTSweep_data002.csv --show   # combine sessions
  python plot_ptsweep.py PTSweep_data001.csv --sat-limit 3800

Outputs <first file>.png and prints a table per resistor and channel:
  bright : counts with the light straight at that face
  dark   : counts with the light off
  span   : bright - dark  (the signal the sun sensor uses; bigger is better)
  noise  : standard deviation of the bright reading
  I_photo: estimated phototransistor current = V / R  (should stay about the same
           for every R until the output saturates near 3.3 V)

"Best" resistor = the biggest span on the WORST channel, with no channel above
--sat-limit counts (near the 4095 ADC limit the reading stops growing with light).
"""

import argparse
import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

ADC_MAX = 4095.0
V_REF = 3.3  # approximate: the ESP32 ADC is not perfectly linear near the top
CHANNELS = ["+X", "+Y", "-X", "-Y"]
COLUMNS = ["plusX", "plusY", "minusX", "minusY"]
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # validated categorical palette
MARKERS = ["o", "s", "^", "D"]


def load(paths):
    """Return {R: {"dark": [4 means], "bright": [4 means], "bright_std": [4 stds]}}.
    If the same R/condition appears more than once, the last row wins."""
    data = defaultdict(lambda: {"dark": [np.nan] * 4, "bright": [np.nan] * 4, "bright_std": [np.nan] * 4})
    for path in paths:
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                r = float(row["resistor(ohm)"])
                cond = row["condition"].strip()
                means = [float(row[f"mean_{c}(count)"]) for c in COLUMNS]
                stds = [float(row[f"std_{c}(count)"]) for c in COLUMNS]
                if cond == "DARK":
                    data[r]["dark"] = means
                elif cond.startswith("LIGHT_"):
                    i = CHANNELS.index(cond[len("LIGHT_"):])
                    data[r]["bright"][i] = means[i]
                    data[r]["bright_std"][i] = stds[i]
    if not data:
        raise ValueError("no data rows found")
    return dict(sorted(data.items()))


def fmt_ohm(r):
    if r >= 1e6:
        return f"{r / 1e6:g}M"
    if r >= 1e3:
        return f"{r / 1e3:g}k"
    return f"{r:g}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="PTSweep_dataNNN.csv file(s) from the SD card")
    ap.add_argument("--sat-limit", type=float, default=3900.0,
                    help="bright counts above this are treated as saturated (default 3900)")
    ap.add_argument("--show", action="store_true", help="show the figure window")
    args = ap.parse_args()

    data = load(args.csv)
    R = np.array(list(data.keys()))
    dark = np.array([d["dark"] for d in data.values()])          # shape (nR, 4)
    bright = np.array([d["bright"] for d in data.values()])
    noise = np.array([d["bright_std"] for d in data.values()])
    span = bright - dark
    i_photo_uA = (bright / ADC_MAX * V_REF) / R[:, None] * 1e6

    # ---------- Table ----------
    print(f"{'R':>8} {'ch':>3} {'dark':>7} {'bright':>7} {'span':>7} {'noise':>6} {'span/noise':>10} {'I_photo(uA)':>11}")
    for k, r in enumerate(R):
        for i, ch in enumerate(CHANNELS):
            if np.isnan(bright[k, i]):
                continue
            flag = "  SATURATED" if bright[k, i] > args.sat_limit else ""
            snr = span[k, i] / noise[k, i] if noise[k, i] > 0 else np.inf
            print(f"{fmt_ohm(r):>8} {ch:>3} {dark[k, i]:7.0f} {bright[k, i]:7.0f} {span[k, i]:7.0f} "
                  f"{noise[k, i]:6.1f} {snr:10.0f} {i_photo_uA[k, i]:11.1f}{flag}")

    # ---------- Pick the best resistor ----------
    best_k = None
    for k in range(len(R)):
        lit = ~np.isnan(bright[k])
        if not lit.any() or np.any(bright[k, lit] > args.sat_limit):
            continue
        worst_span = np.nanmin(span[k])
        if best_k is None or worst_span > np.nanmin(span[best_k]):
            best_k = k
    print()
    if best_k is None:
        print(f"No resistor kept every channel below {args.sat_limit:.0f} counts. Try smaller resistors "
              "or move the light farther away.")
    else:
        print(f"BEST: {fmt_ohm(R[best_k])} ohm  (worst-channel span {np.nanmin(span[best_k]):.0f} counts, "
              f"no channel above {args.sat_limit:.0f})")
        for i, ch in enumerate(CHANNELS):
            ok = [k for k in range(len(R)) if not np.isnan(bright[k, i]) and bright[k, i] <= args.sat_limit]
            if ok:
                kb = max(ok, key=lambda k: span[k, i])
                print(f"  best for {ch} alone: {fmt_ohm(R[kb])} (span {span[kb, i]:.0f})")
    print("Re-run command 10 (calibrate) after installing the chosen resistor.")

    # ---------- Plot ----------
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), constrained_layout=True)
    fig.suptitle("Phototransistor resistor sweep: " + ", ".join(os.path.basename(p) for p in args.csv))
    log_x = R.max() / R.min() > 5

    def style(ax, ylabel, title):
        if log_x:
            ax.set_xscale("log")
        ax.set_xticks(R)
        ax.set_xticklabels([fmt_ohm(r) for r in R], rotation=45 if len(R) > 6 else 0)
        ax.minorticks_off()
        ax.set_xlabel("Resistor (ohm)")
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc="left")
        ax.grid(True, color="#e6e6e3", linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    # 1) Brightness magnitude vs R
    ax = axes[0]
    ax.axhline(ADC_MAX, color="#8a8984", linewidth=1, linestyle=":")
    ax.axhline(args.sat_limit, color="#8a8984", linewidth=1, linestyle="--")
    ax.text(R[0], args.sat_limit, " saturation limit", va="top", fontsize=8, color="#52514e")
    for i, ch in enumerate(CHANNELS):
        ax.errorbar(R, bright[:, i], yerr=noise[:, i], color=COLORS[i], marker=MARKERS[i], markersize=8,
                    linewidth=2, capsize=3, label=f"{ch} light on")
        ax.plot(R, dark[:, i], color=COLORS[i], marker=MARKERS[i], markersize=6, linewidth=1.2,
                linestyle="--", markerfacecolor="white", label=f"{ch} dark")
    ax.set_ylim(0, ADC_MAX * 1.05)
    style(ax, "ADC counts (0-4095)", "Brightness recorded vs resistor\n(solid = light on, dashed = light off)")
    ax.legend(fontsize=8, ncol=2, frameon=True, framealpha=0.9, edgecolor="none", loc="center right")
    sec = ax.secondary_yaxis("right", functions=(lambda c: c / ADC_MAX * V_REF, lambda v: v / V_REF * ADC_MAX))
    sec.set_ylabel("Voltage (V, approx.)")

    # 2) Signal span vs R
    ax = axes[1]
    for i, ch in enumerate(CHANNELS):
        ax.plot(R, span[:, i], color=COLORS[i], marker=MARKERS[i], markersize=8, linewidth=2, label=ch)
        sat = bright[:, i] > args.sat_limit
        ax.scatter(R[sat], span[sat, i], s=160, facecolors="none", edgecolors="#e34948", linewidths=1.5, zorder=5)
    if best_k is not None:
        ax.axvline(R[best_k], color="#0b0b0b", linewidth=1, linestyle="--")
        ax.annotate(f"best: {fmt_ohm(R[best_k])}", (R[best_k], ax.get_ylim()[1]), xytext=(4, -12),
                    textcoords="offset points", fontsize=9)
    style(ax, "Bright - dark (counts)", "Usable signal span\n(red ring = saturated)")
    ax.legend(fontsize=8, frameon=False, loc="lower right")

    # 3) Photocurrent vs R
    ax = axes[2]
    for i, ch in enumerate(CHANNELS):
        ax.plot(R, i_photo_uA[:, i], color=COLORS[i], marker=MARKERS[i], markersize=8, linewidth=2, label=ch)
    style(ax, "Photocurrent I = V/R (uA)", "Estimated photocurrent\n(flat until saturation; drop = saturated)")
    ax.legend(fontsize=8, frameon=False)
    if np.nanmax(i_photo_uA) / max(np.nanmin(i_photo_uA), 1e-9) > 20:
        ax.set_yscale("log")

    out = os.path.splitext(args.csv[0])[0] + ".png"
    fig.savefig(out, dpi=150)
    print(f"Saved {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
