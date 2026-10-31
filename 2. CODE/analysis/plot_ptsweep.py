"""
plot_ptsweep.py
===========================================================
Plot a phototransistor resistor sweep (command 16) and pick the best resistor.

Usage:
  python plot_ptsweep.py PTSweep_data001.csv
  python plot_ptsweep.py PTSweep_data001.csv PTSweep_data002.csv --show   # combine sessions
  python plot_ptsweep.py PTSweep_data001.csv --sat-limit 3800

Outputs <first file>.png (3 detail panels), <first file>_best.png (one summary chart
showing why the best resistor wins), and prints a table per resistor and channel:
  bright : counts with the light straight at that face
  dark   : counts with the light off
  span   : bright - dark  (the signal the sun sensor uses; bigger is better)
  noise  : standard deviation of the bright reading
  I_photo: estimated phototransistor current = V / R  (should stay about the same
           for every R until the output saturates near 3.3 V)

"Best" resistor = the biggest span on the WORST channel, with every lit channel inside
the ESP32 ADC's accurate range (150-2450 mV at 11 dB attenuation = ~190-3040 counts,
per the Espressif ADC docs). Above that the reading bends over and stops tracking the
light; at 4095 it is fully saturated (--sat-limit marks saturation on the detail plots).
"""

import argparse
import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

ADC_MAX = 4095.0
V_REF = 3.3  # approximate: the ESP32 ADC is not perfectly linear near the top
ADC_ACCURATE_MIN = 190.0   # ~150 mV: below this the ESP32 ADC reads 0
ADC_ACCURATE_MAX = 3040.0  # ~2450 mV: above this the ESP32 ADC is nonlinear, then saturates
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


def summary_figure(R, bright, best_k):
    """One chart: light-on reading of each face for every resistor, against the ADC's accurate
    range. The best resistor is the one with the most signal while every face stays in range."""
    x = np.arange(len(R))
    fig, ax = plt.subplots(figsize=(11, 6.2), constrained_layout=True)

    # Zones of the ADC (background)
    ax.axhspan(ADC_ACCURATE_MIN, ADC_ACCURATE_MAX, color="#1baf7a", alpha=0.10, linewidth=0)
    ax.axhspan(ADC_ACCURATE_MAX, ADC_MAX, color="#eda100", alpha=0.12, linewidth=0)
    ax.axhline(ADC_MAX, color="#e34948", linewidth=1.5)
    ax.text(-0.55, ADC_MAX + 40, "SATURATED (4095): brighter light reads the same", ha="left", va="bottom",
            fontsize=9, color="#a32d2c")
    ax.text(-0.55, ADC_ACCURATE_MAX + 80, "Nonlinear: reading no longer tracks the light", ha="left", va="bottom",
            fontsize=9, color="#7a5300")
    ax.text(-0.55, ADC_ACCURATE_MAX - 80, "ADC accurate range", ha="left", va="top",
            fontsize=9, color="#0f6b4a")

    # Highlight the best resistor
    if best_k is not None:
        ax.fill_between([best_k - 0.42, best_k + 0.42], -650, ADC_MAX, color="#2a78d6", alpha=0.10, linewidth=0)

    # Each face as a marker, side by side
    lo = np.nanmin(bright, axis=1)
    hi = np.nanmax(bright, axis=1)
    offsets = np.array([-0.18, -0.06, 0.06, 0.18])
    for i, ch in enumerate(CHANNELS):
        ax.scatter(x + offsets[i], bright[:, i], s=70, color=COLORS[i], marker=MARKERS[i],
                   edgecolors="white", linewidths=1.2, zorder=3, label=f"{ch} face")

    # Verdict under each resistor
    for k in range(len(R)):
        if np.nanmax(bright[k]) >= ADC_MAX - 10:
            verdict, color = "saturated", "#a32d2c"
        elif np.nanmax(bright[k]) > ADC_ACCURATE_MAX:
            verdict, color = "partly\nnonlinear", "#7a5300"
        elif k == best_k:
            verdict, color = "BEST", "#1c5cab"
        else:
            verdict, color = "weaker\nsignal", "#52514e"
        ax.text(k, -330, verdict, ha="center", va="top", fontsize=10, color=color,
                fontweight="bold" if k == best_k else "normal")

    if best_k is not None:
        ax.annotate(f"{fmt_ohm(R[best_k])}: the most signal with all 4 faces\n"
                    f"inside the accurate range ({lo[best_k]:.0f}-{hi[best_k]:.0f} counts)",
                    xy=(best_k + 0.15, hi[best_k] + 80), xytext=(best_k - 1.9, ADC_ACCURATE_MAX + 560),
                    fontsize=10, color="#0b0b0b",
                    arrowprops=dict(arrowstyle="->", color="#0b0b0b", linewidth=1))

    ax.set_xticks(x)
    ax.set_xticklabels([f"{fmt_ohm(r)} Ω" for r in R], fontsize=11)
    ax.set_xlim(-0.6, len(R) - 0.4)
    ax.set_ylim(-700, ADC_MAX + 350)
    ax.set_yticks(range(0, 4001, 500))
    ax.set_ylabel("Reading with the light on the face (ADC counts)")
    ax.set_xlabel("Phototransistor resistor")
    ax.set_title("Why this resistor: biggest sun sensor signal that stays in the ADC's accurate range",
                 loc="left", fontsize=13)
    ax.grid(True, axis="y", color="#e6e6e3", linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_visible(False)
    ax.tick_params(axis="x", length=0)
    ax.legend(frameon=False, ncol=2, loc="center right", fontsize=10, title="Light on the...", title_fontsize=10)
    return fig


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
        if not lit.any() or np.any(bright[k, lit] > ADC_ACCURATE_MAX):
            continue
        worst_span = np.nanmin(span[k])
        if best_k is None or worst_span > np.nanmin(span[best_k]):
            best_k = k
    print()
    if best_k is None:
        print(f"No resistor kept every channel below {ADC_ACCURATE_MAX:.0f} counts. Try smaller resistors "
              "or move the light farther away.")
    else:
        print(f"BEST: {fmt_ohm(R[best_k])} ohm  (worst-channel span {np.nanmin(span[best_k]):.0f} counts, "
              f"every channel inside the ADC's accurate range, <= {ADC_ACCURATE_MAX:.0f})")
        for i, ch in enumerate(CHANNELS):
            ok = [k for k in range(len(R)) if not np.isnan(bright[k, i]) and bright[k, i] <= ADC_ACCURATE_MAX]
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

    out_best = os.path.splitext(args.csv[0])[0] + "_best.png"
    summary_figure(R, bright, best_k).savefig(out_best, dpi=150)
    print(f"Saved {out_best}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
