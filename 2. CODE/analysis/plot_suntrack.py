"""
plot_suntrack.py
===========================================================
Plot a sun tracker run (command 13) from the KestrelSAT SD card and check it
against the Lab 8 requirements.

Usage:
  python plot_suntrack.py SunTrack_data001.csv
  python plot_suntrack.py SunTrack_data001.csv --flip-encoder --show
  python plot_suntrack.py data/*.csv               # one figure per file

Outputs <file>.png next to each CSV and prints a requirement summary:
  R1: time to get within ±10° of the sun (after control starts) and stay there
  R2: longest continuous time within ±10°
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

BAND_DEG = 10.0
MODE_COLORS = {"SPINUP": "gold", "HOLD": "tab:blue", "ACQUIRE": "tab:purple", "TRACK": "white", "NOSUN": "tab:red"}


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} has no data rows")

    def col(name):
        return np.array([float(r[name]) for r in rows])

    return {
        "t": col("mcu time(ms)") / 1000.0,
        "mode": np.array([r["mode"] for r in rows]),
        "raw": np.vstack([col("sun_plusX(count)"), col("sun_plusY(count)"),
                          col("sun_minusX(count)"), col("sun_minusY(count)")]),
        "mag": col("sun_mag"),
        "sun_dir": col("sun_direction(deg)"),
        "err": col("error(deg)"),
        "rate": col("body_rate(deg/s)"),
        "w_cmd": col("w_RW_cmd(RPM)"),
        "w_meas": col("w_RW_meas(RPM)"),
        "tau": col("torque_cmd(N*m)"),
        "sat": col("saturated").astype(bool),
        # newer firmware: error the controller used (gyro heading error after a scan), may be missing
        "err_ctrl": col("err_ctrl(deg)") if "err_ctrl(deg)" in rows[0] else None,
    }


def longest_run(mask, t):
    """Longest continuous time (s) where mask is True."""
    best, start = 0.0, None
    for i, m in enumerate(mask):
        if m and start is None:
            start = t[i]
        if (not m or i == len(mask) - 1) and start is not None:
            best = max(best, t[i] - start)
            start = None
    return best


def summarize(d):
    track = d["mode"] == "TRACK"
    if not track.any():
        return {"settle_s": None, "within_pct": 0.0, "longest_s": 0.0, "rms_deg": None, "sat_pct": 0.0}
    # control starts at the first ACQUIRE (scan) or TRACK sample, so R1 includes the scan time
    t_control = d["t"][np.isin(d["mode"], ["ACQUIRE", "TRACK"])][0]
    t, err = d["t"][track], np.abs(d["err"][track])
    outside = np.where(err > BAND_DEG)[0]
    if len(outside) == 0:
        settle = 0.0
    elif outside[-1] == len(err) - 1:
        settle = None
    else:
        settle = t[outside[-1] + 1] - t_control
    settled = t >= (t_control + settle) if settle is not None else np.zeros_like(t, bool)
    return {
        "settle_s": settle,
        "within_pct": 100.0 * np.mean(err <= BAND_DEG),
        "longest_s": longest_run(err <= BAND_DEG, t),
        "rms_deg": float(np.sqrt(np.mean(err[settled] ** 2))) if settled.any() else None,
        "sat_pct": 100.0 * np.mean(d["sat"][track]),
    }


def shade_modes(ax, d):
    modes, t = d["mode"], d["t"]
    start = 0
    for i in range(1, len(modes) + 1):
        if i == len(modes) or modes[i] != modes[start]:
            color = MODE_COLORS.get(modes[start], "lightgray")
            if color != "white":
                ax.axvspan(t[start], t[min(i, len(t) - 1)], color=color, alpha=0.12, lw=0)
            start = i


def plot(d, title, out_path, show):
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
    t = d["t"]

    ax[0].plot(t, d["err"], lw=1.2, label="pointing error (sun sensor)")
    if d["err_ctrl"] is not None:
        ax[0].plot(t, d["err_ctrl"], lw=1, color="k", alpha=0.7, label="error used by controller")
    ax[0].axhspan(-BAND_DEG, BAND_DEG, color="tab:green", alpha=0.15, label=f"±{BAND_DEG:.0f}° requirement")
    ax[0].set_ylabel("error (deg)")
    ax[0].legend(loc="upper right")

    ax[1].plot(t, d["rate"], color="tab:orange", lw=1)
    ax[1].set_ylabel("body rate (deg/s)")

    ax[2].plot(t, d["w_cmd"], label="commanded")
    ax[2].plot(t, d["w_meas"], label="measured (encoder)", alpha=0.7)
    sat = d["sat"]
    if sat.any():
        ax[2].plot(t[sat], d["w_cmd"][sat], "rx", ms=4, label="saturated")
    ax[2].set_ylabel("wheel speed (RPM)")
    ax[2].legend(loc="upper right")

    for i, name in enumerate(["+X", "+Y", "-X", "-Y"]):
        ax[3].plot(t, d["raw"][i], lw=1, label=name)
    ax[3].set_ylabel("sun sensor (ADC count)")
    ax[3].set_xlabel("time (s)")
    ax[3].legend(loc="upper right", ncol=4)

    for a in ax:
        shade_modes(a, d)
        a.grid(alpha=0.3)
    fig.suptitle(title + "\nshading: yellow = spin-up, blue = hold/release, purple = scan, red = no sun")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="SunTrack_dataNNN.csv file(s) from the SD card")
    parser.add_argument("--flip-encoder", action="store_true",
                        help="negate measured wheel speed (if command 12 showed the encoder counts backwards)")
    parser.add_argument("--show", action="store_true", help="open plot windows")
    args = parser.parse_args()

    for path in args.csv:
        d = load(path)
        if args.flip_encoder:
            d["w_meas"] = -d["w_meas"]
        s = summarize(d)
        out = os.path.splitext(path)[0] + ".png"
        plot(d, os.path.basename(path), out, args.show)

        settle = "never" if s["settle_s"] is None else f"{s['settle_s']:.1f} s"
        rms = "n/a" if s["rms_deg"] is None else f"{s['rms_deg']:.1f}°"
        print(f"{os.path.basename(path)}")
        print(f"  R1 time to settle within ±{BAND_DEG:.0f}°: {settle}")
        print(f"  R2 longest time within ±{BAND_DEG:.0f}°:    {s['longest_s']:.1f} s")
        print(f"  time within ±{BAND_DEG:.0f}° while tracking: {s['within_pct']:.1f}%")
        print(f"  RMS error after settling:          {rms}")
        print(f"  wheel saturated:                   {s['sat_pct']:.1f}% of tracking time")
        print(f"  final mode:                        {d['mode'][-1]}")
        print(f"  plot: {out}")


if __name__ == "__main__":
    main()
