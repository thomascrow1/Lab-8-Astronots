"""
plot_fansweep.py
===========================================================
Plot a fan torque sweep (old command 18, removed 2 Nov 2026) from the KestrelSAT SD card.

Usage:
  python plot_fansweep.py FanSweep_data001.csv
  python plot_fansweep.py FanSweep_data001.csv --show

Recomputes each step's torque from the raw log the same way the firmware does:
  torque = I_BODY_Z * (body accel) + I_WHEEL * (wheel accel)
over the window from FIT_START_S to the end of each ON step. Body accel = least-squares slope of the
gyro rate; wheel accel = average wheel speed in the 2nd half minus the 1st half (from encoder counts),
divided by the time between the halves' centres.
Positive torque = counter-clockwise (+Z, seen from above).

Outputs <file>.png next to the CSV (torque vs duty for each pair, with the 5 V theory band from
fan_thrust_model.py, plus the heading / rate / accel time history) and prints the torque table.
"""

import argparse
import csv
import math
import os

import matplotlib.pyplot as plt
import numpy as np

from fan_thrust_model import T_MAX, pair_torque

I_BODY_Z = 0.014        # definitions.h
I_WHEEL = 0.0035
FIT_START_S = 1.5       # FAN_SWEEP_FIT_START_MS
RPM_TO_RADS = 2 * math.pi / 60
COUNTS_PER_REV = 64 * 10  # CPR * 10, as in measure_wheel_rpm()
PIN_COLORS = {27: "#2a78d6", 4: "#eb6834"}


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} has no data rows")

    def col(name):
        return np.array([float(r[name]) for r in rows])

    return {
        "t": col("mcu time(ms)") / 1000.0,
        "step": col("step").astype(int),
        "pin": col("pin").astype(int),
        "duty": col("duty(%)"),
        "phase": np.array([r["phase"] for r in rows]),
        "heading": col("heading(deg)"),
        "rate": col("body_rate(deg/s)"),
        "accel": col("body_accel(deg/s^2)"),
        "w_meas": col("w_RW_meas(RPM)"),
        "enc": col("enc_count"),
    }


def step_torques(d):
    """One row per ON step: pin, duty, body accel (deg/s^2), wheel accel (RPM/s), torque (N*m), turned (deg)."""
    results = []
    for s in np.unique(d["step"][d["phase"] == "ON"]):
        # The firmware logs each step's last ON row with the NEXT step number and pin 0; skip those rows.
        on = (d["step"] == s) & (d["phase"] == "ON") & (d["pin"] != 0)
        if not on.any():
            continue
        t = d["t"][on] - d["t"][on][0]
        fit = t >= FIT_START_S
        if fit.sum() < 3:
            continue
        body = np.polyfit(t[fit], d["rate"][on][fit], 1)[0]
        tf, cf = t[fit], d["enc"][on][fit]
        mid = np.searchsorted(tf, (tf[0] + tf[-1]) / 2)
        rpm = lambda i, j: (cf[j] - cf[i]) / COUNTS_PER_REV / ((tf[j] - tf[i]) / 60)
        wheel = (rpm(mid, -1) - rpm(0, mid)) / ((tf[-1] - tf[0]) / 2)
        tau = I_BODY_Z * math.radians(body) + I_WHEEL * wheel * RPM_TO_RADS
        turned = d["heading"][on][-1] - d["heading"][on][0]
        results.append((int(d["pin"][on][0]), float(d["duty"][on][0]), body, wheel, tau, turned))
    return results


def style(ax, title):
    ax.set_title(title, loc="left", fontsize=11)
    ax.grid(True, color="0.9", linewidth=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def plot(path, d, results, show=False):
    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.4])
    ax_tau = fig.add_subplot(gs[:, 0])
    axs = [fig.add_subplot(gs[i, 1]) for i in range(3)]

    # Torque vs duty: |torque| so both pairs share one axis; the legend says which way each pair twists.
    duty = np.linspace(0, 100, 201)
    ax_tau.plot(duty, pair_torque(duty / 100, T_MAX) * 1e3, color="0.6", linewidth=1.5,
                label="theory (ducted fan, 5 V)")
    for pin in (27, 4):
        rows = [r for r in results if r[0] == pin]
        if not rows:
            continue
        x = [r[1] for r in rows]
        y = [abs(r[4]) * 1e3 for r in rows]
        direction = "CCW" if np.sign(np.median([r[4] for r in rows if r[1] > 0] or [0])) >= 0 else "CW"
        ax_tau.plot(x, y, color=PIN_COLORS[pin], linewidth=2, marker="o", markersize=5,
                    label=f"GPIO {pin} pair ({direction})")
    ax_tau.set_xlabel("PWM duty (%)")
    ax_tau.set_ylabel("|Z torque| (mN·m)")
    ax_tau.set_xlim(0, 100)
    ax_tau.legend(frameon=False, fontsize=8, loc="upper left")
    style(ax_tau, "Measured disturbance torque")

    # Time history with ON steps shaded in each pair's colour.
    for ax, key, label in zip(axs, ("heading", "rate", "accel"),
                              ("Heading (deg)", "Body rate (deg/s)", "Body accel (deg/s², smoothed)")):
        ax.plot(d["t"], d[key], color="0.2", linewidth=1)
        for s in np.unique(d["step"][d["phase"] == "ON"]):
            on = (d["step"] == s) & (d["phase"] == "ON") & (d["pin"] != 0)
            if not on.any():
                continue
            ax.axvspan(d["t"][on][0], d["t"][on][-1], color=PIN_COLORS.get(int(d["pin"][on][0]), "0.8"),
                       alpha=0.15, linewidth=0)
        style(ax, label)
    axs[-1].set_xlabel("Time (s)  — shaded: fans on (blue GPIO 27, orange GPIO 4)")

    fig.suptitle(os.path.basename(path), x=0.01, ha="left")
    fig.tight_layout()
    out = os.path.splitext(path)[0] + ".png"
    fig.savefig(out, dpi=150)
    print(f"Saved {out}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    for path in args.files:
        d = load(path)
        results = step_torques(d)
        print(f"{path}:")
        print("  pin  duty(%)  body accel(deg/s^2)  wheel accel(RPM/s)  torque(mN*m)  turned(deg)")
        for pin, duty, body, wheel, tau, turned in results:
            print(f"  {pin:3d}  {duty:6.0f}  {body:19.3f}  {wheel:18.2f}  {tau * 1e3:+12.3f}  {turned:+10.1f}")
        plot(path, d, results, args.show)


if __name__ == "__main__":
    main()
