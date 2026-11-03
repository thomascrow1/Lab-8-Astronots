"""
fan_hold_report.py
===========================================================
Graphs for a day of disturbance-fan testing: command 8 fan hold runs, fans-off command 4
baselines, and old command 18 fan sweeps (FanSweep_data001). Writes everything into one folder.

Usage:
  python "2. THEORY/fan_hold_report.py" "4. DATA/2. After PDR/2026-11-02/csv/"*.csv --out "4. DATA/2. After PDR/2026-11-02/graphs"

Each SunTrack CSV is sorted automatically:
  - fan hold run : a fan column goes above 0 (command 8; before 2 Nov 2026 also old command 13 with command 17 duties)
  - baseline     : fans never on
FanSweep CSVs are shown as a 5 V rail-sag check (see AI_LOG, 2 Nov: the sweep can't measure torque).

Outputs (in --out):
  01_fan_hold_pointing.png   pointing error vs time since the fans turned on, CW vs CCW, ±10° band
  02_fan_hold_wheel.png      wheel speed vs time since fans on, ±400 RPM limit, fitted slope -> torque
  03_fan_hold_timeline.png   how long pointing held / when the wheel saturated, per run
  04_baseline_wheel.png      fans-off runs: wheel drift while tracking (background torque)
  05_fansweep_rail_sag.png   fan sweep: wheel speed drop when the fans switch on, per duty
  runs/<file>.png            plot_suntrack.py overview of every SunTrack run
  summary.csv, summary.md    the numbers behind the graphs

Torque from the wheel: with the body held on target, all external torque goes into the wheel, so
torque = I_WHEEL * (slope of measured wheel speed), fitted from 2 s after the fans turn on until the
wheel first saturates. Fan torque alone = (CCW torque - CW torque) / 2 when both directions were run;
the background torque (string, air) = their average.
"""

import argparse
import csv
import math
import os

import matplotlib.pyplot as plt
import numpy as np

import plot_suntrack

I_WHEEL = 0.0035          # definitions.h
W_LIMIT = 400.0           # WHEEL_MAX_RPM
BAND_DEG = 10.0           # FAN_HOLD_BAND_DEG
FIT_SKIP_S = 2.0          # skip the fan start (kick + 5 V sag transient)
BASELINE_SKIP_S = 10.0    # skip the slew back to the sun after each scan
MIN_SAG_RPM = 60.0        # sweep rail-sag check: only steps where the wheel was at least this fast
RPM_TO_RADS = 2 * math.pi / 60
COLORS = {"CW": "#2a78d6", "CCW": "#eb6834", "both": "#1baf7a", "off": "0.35"}
BASE_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]


# --------------------------------------------------------------------------------------------- load
def load_suntrack(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))

    def col(name):
        return np.array([float(r[name]) for r in rows])

    fan_cols = [k for k in rows[0] if k.startswith("fan_")]
    d = {
        "name": os.path.splitext(os.path.basename(path))[0],
        "path": path,
        "t": col("mcu time(ms)") / 1000.0,
        "mode": np.array([r["mode"] for r in rows]),
        "heading": col("heading(deg)"),
        "sun_heading": col("sun_heading(deg)"),
        "w_cmd": col("w_RW_cmd(RPM)"),
        "w_meas": col("w_RW_meas(RPM)"),
        "sat": col("saturated") > 0,
        "fan_cw": col(fan_cols[0]) if fan_cols else None,   # GPIO 27 pair (clockwise); old name fan_upper
        "fan_ccw": col(fan_cols[1]) if fan_cols else None,  # GPIO 4 pair (counter-clockwise); old name fan_lower
    }
    # Pointing error the requirement is judged on: gyro heading vs sun heading, only while tracking.
    err = (d["sun_heading"] - d["heading"] + 180.0) % 360.0 - 180.0
    err[d["mode"] != "TRACK"] = np.nan
    d["err"] = err
    return d


def fan_on_mask(d):
    if d["fan_cw"] is None:
        return np.zeros_like(d["t"], bool)
    return (d["fan_cw"] > 0) | (d["fan_ccw"] > 0)


def direction(d):
    on = fan_on_mask(d)
    cw, ccw = d["fan_cw"][on].max(), d["fan_ccw"][on].max()
    if cw > 0 and ccw > 0:
        return "both", max(cw, ccw)
    return ("CW", cw) if cw > 0 else ("CCW", ccw)


# ------------------------------------------------------------------------------------------ analyse
def analyse_hold(d):
    t, on = d["t"], fan_on_mask(d)
    t_on = t[on][0]
    after = t >= t_on
    sat_after = d["sat"] & after
    t_sat = t[sat_after][0] if sat_after.any() else None
    out = after & ~(np.abs(d["err"]) <= BAND_DEG)     # NaN (not tracking) counts as outside
    t_lost = t[out][0] if out.any() else None
    end = t_sat if t_sat is not None else t[-1]
    fit = after & (t >= t_on + FIT_SKIP_S) & (t < end)
    slope = np.polyfit(t[fit], d["w_meas"][fit], 1) if fit.sum() > 20 else None
    held = after & (t < end) & (d["mode"] == "TRACK")
    dirn, duty = direction(d)
    return {
        "run": d["name"], "type": "fan hold", "direction": dirn, "duty_pct": duty,
        "t_fans_on_s": t_on,
        "wheel_at_fans_on_rpm": d["w_meas"][on][0],
        "time_to_saturate_s": None if t_sat is None else t_sat - t_on,
        "time_to_lose_pointing_s": None if t_lost is None else t_lost - t_on,
        "max_err_before_sat_deg": float(np.nanmax(np.abs(d["err"][held]))) if held.any() else None,
        "wheel_slope_rpm_s": None if slope is None else slope[0],
        "torque_mNm": None if slope is None else I_WHEEL * slope[0] * RPM_TO_RADS * 1e3,
        "_fit": slope, "_t_on": t_on, "_t_sat": t_sat, "_t_lost": t_lost,
    }


def analyse_baseline(d):
    t = d["t"]
    track = d["mode"] == "TRACK"
    # tracking samples at least BASELINE_SKIP_S after the most recent scan ended
    last_acq = -np.inf
    keep = np.zeros_like(track)
    for i in range(len(t)):
        if d["mode"][i] == "ACQUIRE":
            last_acq = t[i]
        keep[i] = track[i] and t[i] - last_acq >= BASELINE_SKIP_S and not d["sat"][i]
    slope = np.polyfit(t[keep], d["w_meas"][keep], 1) if keep.sum() > 50 else None
    return {
        "run": d["name"], "type": "baseline (fans off)", "direction": "off", "duty_pct": 0,
        "wheel_slope_rpm_s": None if slope is None else slope[0],
        "torque_mNm": None if slope is None else I_WHEEL * slope[0] * RPM_TO_RADS * 1e3,
        "pct_in_band_tracking": 100.0 * np.nanmean(np.abs(d["err"][track]) <= BAND_DEG) if track.any() else None,
        "_fit": slope, "_keep": keep,
    }


# --------------------------------------------------------------------------------------------- plot
def style(ax, title):
    ax.set_title(title, loc="left", fontsize=11)
    ax.grid(True, color="0.9", linewidth=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def plot_pointing(holds, out):
    fig, ax = plt.subplots(figsize=(10, 5))
    sat_order = sorted(range(len(holds)), key=lambda k: holds[k][1]["_t_sat"] or 0)
    ax.axhspan(-BAND_DEG, BAND_DEG, color="#1baf7a", alpha=0.12, linewidth=0, label=f"±{BAND_DEG:.0f}° requirement")
    for k, (d, r) in enumerate(holds):
        c = COLORS[r["direction"]]
        tt = d["t"] - r["_t_on"]
        ax.plot(tt, d["err"], color=c, linewidth=1.5, label=f"{r['direction']} {r['duty_pct']:.0f}% ({r['run']})")
        if r["_t_sat"] is not None:
            x = r["_t_sat"] - r["_t_on"]
            ax.axvline(x, color=c, linestyle="--", linewidth=1)
            first = sat_order.index(k) % 2 == 0   # stagger: earlier saturation labelled to the left, later to the right
            ax.text(x, 37 - 7 * sat_order.index(k), f"{r['direction']} wheel saturates {x:.1f} s",
                    color=c, fontsize=8, va="top", ha="right" if first else "left")
        if r["_t_lost"] is not None:
            ax.plot(r["_t_lost"] - r["_t_on"], np.sign(d["err"][d["t"] >= r["_t_lost"]][0] or 1) * BAND_DEG,
                    "o", color=c, markersize=8, markeredgecolor="white")
    ax.axvline(0, color="0.3", linewidth=1)
    ax.text(0, -36, " fans on", fontsize=8, color="0.3")
    ax.set_ylim(-40, 40)
    ax.set_xlim(left=-25)
    ax.set_xlabel("time since fans turned on (s)")
    ax.set_ylabel("pointing error, gyro heading − sun heading (deg)")
    ax.legend(frameon=False, fontsize=8, loc="lower left")
    style(ax, "Pointing under 100% fan disturbance (dots: pointing first leaves the band; y-axis clipped at ±40°)")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_wheel(holds, out):
    fig, ax = plt.subplots(figsize=(10, 5))
    for lim in (-W_LIMIT, W_LIMIT):
        ax.axhline(lim, color="#d03b3b", linewidth=1, linestyle=":")
    ax.text(-24, W_LIMIT + 10, "wheel limit +400 RPM", color="#d03b3b", fontsize=8)
    ax.text(-24, -W_LIMIT - 30, "wheel limit −400 RPM", color="#d03b3b", fontsize=8)
    for d, r in holds:
        c = COLORS[r["direction"]]
        tt = d["t"] - r["_t_on"]
        ax.plot(tt, d["w_meas"], color=c, linewidth=1.2, label=f"{r['direction']} measured ({r['run']})")
        ax.plot(tt, d["w_cmd"], color=c, linewidth=0.8, alpha=0.45, label=f"{r['direction']} commanded")
        if r["_fit"] is not None:
            end = (r["_t_sat"] if r["_t_sat"] is not None else d["t"][-1]) - r["_t_on"]
            x = np.array([FIT_SKIP_S, end])
            ax.plot(x, np.polyval(r["_fit"], x + r["_t_on"]), color="0.1", linewidth=1.2, linestyle="--")
            # label beside the middle of the fit, on the side away from the zero line
            xm = (FIT_SKIP_S + end) / 2
            ym = np.polyval(r["_fit"], xm + r["_t_on"])
            ax.text(xm - 6, ym + (80 if r["wheel_slope_rpm_s"] > 0 else -80),
                    f"{r['wheel_slope_rpm_s']:+.1f} RPM/s = {abs(r['torque_mNm']):.2f} mN·m", fontsize=9,
                    color=c, ha="right", va="center")
    ax.axvline(0, color="0.3", linewidth=1)
    ax.set_ylim(-W_LIMIT - 60, W_LIMIT + 60)
    ax.set_xlim(left=-25)
    ax.set_xlabel("time since fans turned on (s)")
    ax.set_ylabel("reaction wheel speed (RPM)")
    ax.legend(frameon=False, fontsize=8, loc="upper left", bbox_to_anchor=(0.0, 0.93))
    style(ax, "Reaction wheel absorbing the fan torque (dashed: least-squares fit, torque = I_wheel × slope)")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_timeline(results, out):
    holds = [r for r in results if r["type"] == "fan hold"]
    fig, ax = plt.subplots(figsize=(10, 1.4 + 0.9 * len(holds)))
    for i, r in enumerate(holds):
        c = COLORS[r["direction"]]
        sat, lost = r["time_to_saturate_s"], r["time_to_lose_pointing_s"]
        held = lost if lost is not None else (sat or 0)
        ax.barh(i, held, color=c, height=0.5)
        ax.text(held, i, f"  pointing held {held:.1f} s", va="center", fontsize=9)
        if sat is not None:
            ax.plot([sat, sat], [i - 0.35, i + 0.35], color="0.1", linewidth=2)
            ax.text(sat, i - 0.42, f"saturated {sat:.1f} s", ha="right", va="top", fontsize=8, color="0.25")
    ax.set_yticks(range(len(holds)))
    ax.set_yticklabels([f"{r['direction']} {r['duty_pct']:.0f}%\n{r['run']}" for r in holds], fontsize=9)
    ax.set_xlabel("time since fans turned on (s)")
    ax.set_xlim(0, max((r["time_to_lose_pointing_s"] or r["time_to_saturate_s"] or 0) for r in holds) * 1.35)
    ax.invert_yaxis()
    style(ax, f"How long the ±{BAND_DEG:.0f}° requirement held with the fans on (black tick: wheel saturates)")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_baselines(bases, out):
    fig, ax = plt.subplots(figsize=(10, 5))
    for k, (d, r) in enumerate(bases):
        c = BASE_COLORS[k % len(BASE_COLORS)]
        ax.plot(d["t"], d["w_meas"], color=c, linewidth=1, alpha=0.85,
                label=f"{r['run']}" + ("" if r["_fit"] is None else
                                       f"  ({r['wheel_slope_rpm_s']:+.2f} RPM/s = {r['torque_mNm']:+.2f} mN·m)"))
        if r["_fit"] is not None:
            tk = d["t"][r["_keep"]]
            ax.plot(tk, np.polyval(r["_fit"], tk), color=c, linewidth=1.5, linestyle="--")
    for lim in (-W_LIMIT, W_LIMIT):
        ax.axhline(lim, color="#d03b3b", linewidth=1, linestyle=":")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("reaction wheel speed (RPM)")
    ax.legend(frameon=False, fontsize=8, loc="upper right")
    style(ax, f"Fans off: wheel drift while tracking (dashed fit, from {BASELINE_SKIP_S:.0f} s after each scan) = background torque")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_sweep_sag(path, out):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    step = np.array([int(r["step"]) for r in rows])
    phase = np.array([r["phase"] for r in rows])
    pin = np.array([int(r["pin"]) for r in rows])
    duty = np.array([float(r["duty(%)"]) for r in rows])
    w = np.array([float(r["w_RW_meas(RPM)"]) for r in rows])
    pts = {27: [], 4: []}
    for s in np.unique(step):
        before = (step == s) & (phase == "SETTLE")
        on = (step == s) & (phase == "ON") & (pin != 0)  # pin 0 + ON = last row of the previous step (firmware quirk)
        if before.sum() < 10 or on.sum() < 20:
            continue
        w_before = np.mean(w[before][-10:])                     # last 0.5 s before the fans start
        if abs(w_before) < MIN_SAG_RPM:
            continue  # wheel too slow: a 1-2 RPM encoder step would read as a big % change
        w_on = np.mean(w[on][10:30])                            # 0.5-1.5 s after
        drop = abs(w_on) - abs(w_before)                        # negative = wheel slowed
        pts[int(pin[on][0])].append((duty[on][0], 100.0 * drop / max(abs(w_before), 1.0)))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for p, label, c in ((27, "GPIO 27 pair (CW)", COLORS["CW"]), (4, "GPIO 4 pair (CCW)", COLORS["CCW"])):
        if pts[p]:
            x, y = zip(*sorted(pts[p]))
            ax.plot(x, y, "o-", color=c, linewidth=2, markersize=5, label=label)
    ax.axhline(0, color="0.3", linewidth=1)
    ax.set_xlabel("fan PWM duty (%)")
    ax.set_ylabel("change in wheel speed when fans start (%)")
    ax.set_xlim(0, 100)
    ax.legend(frameon=False, fontsize=8)
    style(ax, f"{os.path.basename(path)}: wheel slows when the fans start (shared 5 V rail sags)\n"
              f"Steps with the wheel under {MIN_SAG_RPM:.0f} RPM left out. Sweep torque numbers are not valid.")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------------------------- main
def fmt(v, nd=2):
    return "" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def write_summary(results, out_dir):
    keys = ["run", "type", "direction", "duty_pct", "time_to_saturate_s", "time_to_lose_pointing_s",
            "max_err_before_sat_deg", "wheel_at_fans_on_rpm", "wheel_slope_rpm_s", "torque_mNm",
            "pct_in_band_tracking"]
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in results:
            w.writerow([fmt(r.get(k)) for k in keys])

    holds = [r for r in results if r["type"] == "fan hold"]
    lines = ["# Disturbance fan tests", "",
             "| Run | Direction | Pointing held (s) | Wheel saturated (s) | Worst error before saturation (°) | Wheel slope (RPM/s) | Torque (mN·m) |",
             "|---|---|---|---|---|---|---|"]
    for r in holds:
        lines.append(f"| {r['run']} | {r['direction']} {r['duty_pct']:.0f}% | {fmt(r['time_to_lose_pointing_s'], 1)} | "
                     f"{fmt(r['time_to_saturate_s'], 1)} | {fmt(r['max_err_before_sat_deg'], 1)} | "
                     f"{fmt(r['wheel_slope_rpm_s'])} | {fmt(r['torque_mNm'])} |")
    cw = [r["torque_mNm"] for r in holds if r["direction"] == "CW" and r["torque_mNm"] is not None]
    ccw = [r["torque_mNm"] for r in holds if r["direction"] == "CCW" and r["torque_mNm"] is not None]
    bases = [r for r in results if r["type"].startswith("baseline") and r["torque_mNm"] is not None]
    lines += ["", "Torques: + = counter-clockwise seen from above. The measured torque is fan + background "
                  "(string twist, air)."]
    if cw and ccw:
        fan = (np.mean(ccw) - np.mean(cw)) / 2
        bg = (np.mean(ccw) + np.mean(cw)) / 2
        lines += ["", f"- **Estimate A (CW and CCW runs only, assumes both pairs equally strong):** fan torque "
                      f"{fan:.2f} mN·m per pair, background {bg:+.2f} mN·m."]
    if bases:
        b = float(np.mean([r["torque_mNm"] for r in bases]))
        est = ", ".join(f"{r['direction']} {abs(r['torque_mNm'] - b):.2f} mN·m" for r in holds if r["torque_mNm"] is not None)
        lines += [f"- **Estimate B (subtract the fans-off background {b:+.2f} mN·m):** {est}."]
        lines += ["", "Fans-off baselines (wheel drift while tracking):", ""]
        lines += [f"- {r['run']}: {r['wheel_slope_rpm_s']:+.2f} RPM/s = {r['torque_mNm']:+.2f} mN·m" for r in bases]
    if cw and ccw and bases:
        lines += ["", "A and B disagree on the background's sign, so the fan torque per pair is somewhere in the "
                      "range they span. Repeat runs in each direction would narrow it."]
    lines += ["", "Torque = I_wheel × slope of measured wheel speed, from 2 s after the fans start until the wheel "
                  "first saturates. Pointing error = gyro heading − sun heading (the sun-sensor error reads ~0 "
                  "whenever only −X sees the light). Graphs made by `2. THEORY/fan_hold_report.py`."]
    with open(os.path.join(out_dir, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--out", required=True, help="folder to write the graphs into")
    args = parser.parse_args()
    os.makedirs(os.path.join(args.out, "runs"), exist_ok=True)

    holds, bases, results, sweeps = [], [], [], []
    for path in sorted(args.files):
        name = os.path.basename(path)
        if name.startswith("FanSweep"):
            sweeps.append(path)
        elif name.startswith("SunTrack"):
            d = load_suntrack(path)
            if fan_on_mask(d).any():
                r = analyse_hold(d)
                holds.append((d, r))
            else:
                r = analyse_baseline(d)
                bases.append((d, r))
            results.append(r)
            ov = plot_suntrack.load(path)
            plot_suntrack.plot(ov, name, os.path.join(args.out, "runs", os.path.splitext(name)[0] + ".png"), False)

    if holds:
        plot_pointing(holds, os.path.join(args.out, "01_fan_hold_pointing.png"))
        plot_wheel(holds, os.path.join(args.out, "02_fan_hold_wheel.png"))
        plot_timeline(results, os.path.join(args.out, "03_fan_hold_timeline.png"))
    if bases:
        plot_baselines(bases, os.path.join(args.out, "04_baseline_wheel.png"))
    for p in sweeps:
        plot_sweep_sag(p, os.path.join(args.out, "05_fansweep_rail_sag.png"))
    write_summary(results, args.out)
    print(f"\nGraphs written to {args.out}")


if __name__ == "__main__":
    main()
