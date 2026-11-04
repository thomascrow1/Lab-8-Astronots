"""
final_figures.py
===========================================================
Graphs and numbers for the final presentation that no other script makes.

Usage (from the project root):
  python "2. THEORY/final_figures.py"

Outputs (in 6. FINAL/figures/):
  fig_pdr_vs_final.png   pointing error vs time: PDR run 052 (old controller) next to 30 Oct run 015
  fig_sim_vs_hw.png      simulation vs hardware for the same start (30 Oct run 015)
  run_summary.csv        R1 (pointing accuracy) and settle numbers for the full-length fans-off tracking runs

Pointing error after the PDR runs comes from the gyro: heading - sun heading found by the scan.
The sun-sensor error column reads ~0 whenever only -X sees the light, so it can't show the
true accuracy (see AI_LOG, 30 Oct). The gyro error is only as good as the scan's sun heading.
"""

import csv
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sim"))
import sun_tracker_sim as sim  # noqa: E402

BAND_DEG = 10.0
DATA = "4. DATA"
OUT = os.path.join("6. FINAL", "figures")
PDR_RUN = os.path.join(DATA, "1. Before PDR (Oct)", "csv", "SunTrack_data052.csv")
FINAL_RUN = os.path.join(DATA, "2. After PDR", "2026-10-30", "csv", "SunTrack_data015.csv")
SUMMARY_RUNS = [
    ("30 Oct", "SunTrack_data014.csv"), ("30 Oct", "SunTrack_data015.csv"),
    ("2 Nov", "SunTrack_data016.csv"), ("2 Nov", "SunTrack_data017.csv"),
]  # fan hold runs 019/020 are scored by fan_hold_report.py
DAY_FOLDER = {"30 Oct": "2026-10-30", "2 Nov": "2026-11-02"}


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    d = {"t": np.array([float(r["mcu time(ms)"]) for r in rows]) / 1000.0,
         "mode": np.array([r["mode"] for r in rows]),
         "err_sensor": np.array([float(r["error(deg)"]) for r in rows])}
    if "heading(deg)" in rows[0]:
        d["err_gyro"] = wrap180([float(r["heading(deg)"]) - float(r["sun_heading(deg)"]) for r in rows])
    # control starts at the first scan (ACQUIRE) sample, or TRACK on the old firmware
    d["t0"] = d["t"][np.isin(d["mode"], ["ACQUIRE", "TRACK"])][0]
    return d


def longest_run(mask, t):
    best, start = 0.0, None
    for i, m in enumerate(mask):
        if m and start is None:
            start = t[i]
        if (not m or i == len(mask) - 1) and start is not None:
            best = max(best, t[i] - start)
            start = None
    return best


def score(d):
    """R1 (pointing accuracy) and settle numbers from the gyro error while tracking."""
    use = d["mode"] == "TRACK"
    t, err = d["t"][use], np.abs(d["err_gyro"][use])
    inside = err <= BAND_DEG
    # settled = first time it enters the band and then stays there for 10 s
    settle = None
    for k in np.where(inside)[0]:
        j = k
        while j < len(err) and inside[j]:
            j += 1
        if t[min(j, len(t) - 1)] - t[k] >= 10.0:
            settle = t[k] - d["t0"]
            break
    after = err[t >= d["t0"] + settle] if settle is not None else err
    return {"settle_s": settle, "longest_s": longest_run(inside, t), "track_s": t[-1] - t[0],
            "in10_pct": 100.0 * np.mean(after <= 10.0), "in5_pct": 100.0 * np.mean(after <= 5.0),
            "rms_deg": float(np.sqrt(np.mean(after ** 2)))}


def scan_end(d):
    """Seconds after control start when the first scan finished (first TRACK sample)."""
    return d["t"][d["mode"] == "TRACK"][0] - d["t0"]


def fig_pdr_vs_final():
    pdr, fin = load(PDR_RUN), load(FINAL_RUN)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.axvspan(0, scan_end(fin), color="tab:purple", alpha=0.1, label="final: 360° scan to find the light")
    ax.axhspan(-BAND_DEG, BAND_DEG, color="tab:green", alpha=0.15, label="±10° requirement")
    ax.plot(pdr["t"] - pdr["t0"], wrap180(pdr["err_sensor"]), color="tab:red", lw=1,
            label="PDR: run 052 (23 Oct, PD on sun sensor only)")
    ax.plot(fin["t"] - fin["t0"], fin["err_gyro"], color="tab:blue", lw=1.5,
            label="Final: 30 Oct run 015 (scan + gyro hold + lobing)")
    ax.set_xlim(0, 100)
    ax.set_ylim(-180, 180)
    ax.set_yticks(range(-180, 181, 45))
    ax.set_xlabel("time since control start (s)")
    ax.set_ylabel("pointing error (deg)")
    ax.set_title("Pointing error before and after the PDR fixes (fans off)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_pdr_vs_final.png"), dpi=150)
    plt.close(fig)


def fig_sim_vs_hw():
    hw = load(FINAL_RUN)
    start_err = hw["err_gyro"][hw["mode"] == "TRACK"][0]
    # same start as run 015, hardware-like sensors and gyro drift ("drift" scenario)
    log = sim.simulate(sun_start_deg=180.0 + start_err, half_power_deg=33.0, adc_floor=0.08,
                       sun_noise=0.03, gyro_drift_dps=0.25, duration_s=120.0)
    t_sim = log["t"] - sim.TRACK_CONTROL_MS / 1000.0
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.axvspan(0, scan_end(hw), color="tab:purple", alpha=0.1, label="360° scan")
    ax.axhspan(-BAND_DEG, BAND_DEG, color="tab:green", alpha=0.15, label="±10° requirement")
    ax.plot(t_sim[t_sim >= 0], log["true_err"][t_sim >= 0], color="tab:orange", lw=1.5,
            label="Simulation (sun_tracker_sim.py, 0.25°/s gyro drift)")
    ax.plot(hw["t"] - hw["t0"], hw["err_gyro"], color="tab:blue", lw=1.5,
            label="Hardware: 30 Oct run 015 (gyro heading − sun heading)")
    ax.set_xlim(0, 100)
    ax.set_ylim(-30, 30)
    ax.set_xlabel("time since control start (s)")
    ax.set_ylabel("pointing error (deg)")
    ax.set_title("Predicted vs measured pointing (y-axis clipped at ±30°)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_sim_vs_hw.png"), dpi=150)
    plt.close(fig)
    print(f"sim settle {sim.settling_time(log)} s, hardware start error {start_err:.0f}°")


def run_summary():
    with open(os.path.join(OUT, "run_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["day", "run", "settle_s", "longest_in10_s", "track_s", "in10_pct", "in5_pct", "rms_deg"])
        for day, name in SUMMARY_RUNS:
            s = score(load(os.path.join(DATA, "2. After PDR", DAY_FOLDER[day], "csv", name)))
            w.writerow([day, name, f"{s['settle_s']:.1f}", f"{s['longest_s']:.0f}", f"{s['track_s']:.0f}",
                        f"{s['in10_pct']:.0f}", f"{s['in5_pct']:.0f}", f"{s['rms_deg']:.1f}"])
            print(day, name, {k: round(v, 1) for k, v in s.items()})


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_pdr_vs_final()
    fig_sim_vs_hw()
    run_summary()
