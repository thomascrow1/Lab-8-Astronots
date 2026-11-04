"""
requirement_graphs.py
===========================================================
Six graphs for the final requirements: hardware (command 10 CSVs) next to the model (sun_tracker_sim.py).
Every graph has the same axes: x = time (s), y = where the solar array points with the Sun at 0 deg
(+ = counter-clockwise from above), with the ±10° R1 band shaded.

  Graph 1  R1 + R2  turn 180° away, timed slew back (R2: within 10 s), then hold (R1: within ±10°)
                    hardware: command 10 -> 1                    (R1R2_dataNNN.csv)
                    t = 0 at the start of the slew back
  Graph 2  R3       fans at 50% and 100%: how long until the wheel saturates and pointing is lost
                    hardware: command 10 -> "2 ccw 50" and "2 ccw 100"  (R3_dataNNN.csv)
                    t = 0 when the fans turn on
  Graph 3  S2       momentum dumps: how long pointing holds while the thrusters unload the wheel
                    hardware: command 10 -> 3                    (S2_dataNNN.csv)
                    t = 0 when the disturbance fans turn on

Usage (from the project root):
  python "2. THEORY/requirement_graphs.py" path/to/csv/*.csv --out "6. FINAL/requirements"
  python "2. THEORY/requirement_graphs.py" --out "6. FINAL/requirements"      # model only (no hardware yet)

Files are sorted by name: R1R2_* -> graph 1, R3_* -> graph 2, S2_* -> graph 3. Older SunTrack_* files
(command 8 / 11) also work: fans on -> graph 2, fans on with dumping -> graph 3.

Outputs (in --out):
  requirements_6graphs.png        all six: rows = graphs 1-3, left = hardware, right = model
  graph1_hardware.png ... graph3_model.png    each graph on its own
  graph1_hardware.csv ... graph3_model.csv    the plotted data, ready for Excel:
                                  run, t(s), array_angle(deg), phase, wheel(RPM), fan_dist(%), fan_thruster(%), saturated
  summary.md, summary.csv         the requirement numbers, hardware vs model

The hardware angle is the gyro heading minus the sun heading the scan found (the same number command 10
judges); the model plots the same quantity from the simulated gyro.
"""

import argparse
import csv
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim"))
import sun_tracker_sim as sim  # noqa: E402

BAND = sim.FAN_HOLD_BAND_DEG
R2_LIMIT_S = sim.DEMO_R2_LIMIT_MS / 1000.0
R2_CONFIRM_S = sim.DEMO_R2_CONFIRM_MS / 1000.0
R3_JUDGE_S = sim.DEMO_R3_FAN_MS / 1000.0
PRE_FANS_S = 15.0          # graphs 2 and 3 start this long before the fans turn on
GRAPH23_YLIM = 45.0        # graphs 2 and 3: y axis ±45° (pointing lost runs off the top/bottom)
SAT_HOLD_S = 2.0           # "saturated" = at the wheel limit this long in a row (firmware warns after 2 s);
                           #   near the limit the wheel touches it for a fraction of a second many times first

# Reference palette (dataviz skill, validated): categorical slots in fixed order.
BLUE, ORANGE, AQUA, YELLOW, MAGENTA = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"
INK, INK2, BAND_FILL, GRID = "#0b0b0b", "#52514e", "#dcdcd6", "#ebebe6"
# One color per fan setting, the same in the hardware and model panels.
FAN_COLORS = {"CCW 100%": ORANGE, "CCW 50%": BLUE, "CW 100%": AQUA, "CW 50%": MAGENTA}


def unwrap_deg(a):
    """Unwrap an angle trace (NaN gaps kept) so a spin after saturation runs off the chart
    instead of jumping between +180 and -180."""
    a = np.array(a, dtype=float)
    ok = ~np.isnan(a)
    if ok.sum() > 1:
        u = np.degrees(np.unwrap(np.radians(a[ok])))
        u -= 360.0 * np.round(u[0] / 360.0 - a[ok][0] / 360.0)
        a[ok] = u
    return a

# Model runs (same start for all: sun 90° from the array, hardware-like narrow sensors, ADC dead zone).
MODEL_BASE = dict(sun_start_deg=90.0, half_power_deg=33.0, adc_floor=0.08, sun_noise=0.02)
MODEL_RUNS = {
    "graph1": [("Model: R1/R2", dict(demo_req=1, duration_s=200.0))],
    "graph2": [("Model: CCW 50%", dict(demo_req=2, fan_dir=1, fan_pct=50.0, duration_s=1200.0)),
               ("Model: CCW 100%", dict(demo_req=2, fan_dir=1, fan_pct=100.0, duration_s=1200.0))],
    "graph3": [("Model: CCW 70% + thrusters", dict(demo_req=3, fan_dir=1, fan_pct=70.0, desat=True,
                                                   duration_s=600.0))],
}
TITLES = {
    "graph1": "Graph 1 · R1 + R2: slew back from 180° within 10 s, then hold ±10°",
    "graph2": "Graph 2 · R3: hold ±10° against the fans until the wheel saturates",
    "graph3": "Graph 3 · S2: hold ±10° while the thrusters dump wheel momentum",
}


def wrap180(a):
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------- load
def load_hardware(path):
    """One SD-card CSV -> dict of arrays in the same form as the model (t in s)."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    def col(name, default=0.0):
        return np.array([float(r[name]) if r.get(name, "") not in ("", None) else default for r in rows])

    mode = np.array([r["mode"] for r in rows])
    fan_cols = [k for k in rows[0] if k.startswith("fan_")]
    fan27, fan4 = (col(fan_cols[0]), col(fan_cols[1])) if len(fan_cols) == 2 else (np.zeros(len(rows)),) * 2
    dumping = col("dumping") > 0 if "dumping" in rows[0] else np.zeros(len(rows), bool)
    if "phase" in rows[0]:
        phase = np.array([r["phase"] for r in rows])
        angle = col("array_angle(deg)", np.nan)
    else:  # older SunTrack file (before 3 Nov): rebuild phase and angle from the other columns
        angle = wrap180(col("heading(deg)") - col("sun_heading(deg)"))
        angle[mode != "TRACK"] = np.nan
        phase = mode.copy()
        fans = (fan27 > 0) | (fan4 > 0)
        phase[(mode == "TRACK") & fans] = "FANS"
        phase[(mode == "TRACK") & dumping] = "DUMP"
    name = os.path.splitext(os.path.basename(path))[0]
    return {"name": name, "t": col("mcu time(ms)") / 1000.0, "phase": phase, "array_angle": angle,
            "w_meas": col("w_RW_meas(RPM)"), "w_cmd": col("w_RW_cmd(RPM)"), "sat": col("saturated") > 0,
            "fan27": fan27, "fan4": fan4, "dumping": dumping}


def classify(d):
    """Which graph a hardware file belongs to (or None)."""
    n = d["name"].upper()
    if n.startswith("R1R2"):
        return "graph1"
    if n.startswith("R3"):
        return "graph2"
    if n.startswith("S2"):
        return "graph3"
    fans = (d["fan27"] > 0) | (d["fan4"] > 0)
    if fans.any():
        return "graph3" if d["dumping"].any() else "graph2"
    return None


def fan_label(d):
    """'CCW 100%' from the fan columns (duty while the fans were on, skipping the 100% start kick)."""
    on = np.isin(d["phase"], ["FANS", "DUMP"])
    dist_ccw = np.median(d["fan4"][on & ~d["dumping"]]) if on.any() else 0.0
    dist_cw = np.median(d["fan27"][on & ~d["dumping"]]) if on.any() else 0.0
    return f"CCW {dist_ccw:.0f}%" if dist_ccw >= dist_cw else f"CW {dist_cw:.0f}%"


def from_model(label, kw):
    log = sim.simulate(**MODEL_BASE, **kw)
    fan_dir = kw.get("fan_dir", 0)
    dist, thr = log["fan_dist"], log["fan_thr"]
    # put the duties on the same pins as the hardware log (CCW = GPIO 4, CW = GPIO 27)
    fan4 = dist if fan_dir > 0 else thr
    fan27 = thr if fan_dir > 0 else dist
    if fan_dir == 0:
        fan4 = fan27 = np.zeros_like(dist)
    return {"name": label, "t": log["t"], "phase": log["phase"], "array_angle": log["array_angle"],
            "w_meas": log["w_meas"], "w_cmd": log["w_cmd"], "sat": log["sat"].astype(bool),
            "fan27": fan27, "fan4": fan4, "dumping": log["dumping"].astype(bool), "events": log["events"]}


# ------------------------------------------------------------------------------------------- score
def first_t(d, mask):
    idx = np.where(mask)[0]
    return d["t"][idx[0]] if len(idx) else None


def first_long_run(t, mask, min_s):
    """Start of the first stretch where mask stays True for at least min_s (None if never)."""
    start = None
    for i in range(len(t)):
        if mask[i] and start is None:
            start = i
        if start is not None and (not mask[i] or i == len(t) - 1):
            end = i if mask[i] else i - 1
            if t[end] - t[start] >= min_s - 1e-6:
                return t[start]
            start = None
    return None


def time_zero(d, graph):
    """Graph 1: start of the slew back. Graphs 2/3: fans on."""
    if graph == "graph1":
        return first_t(d, d["phase"] == "SLEW")
    return first_t(d, np.isin(d["phase"], ["FANS", "DUMP"]))


def score(d, graph):
    t0 = time_zero(d, graph)
    if t0 is None:
        return {"note": "requirement start never reached"}
    t, a = d["t"] - t0, d["array_angle"]
    inside = np.abs(a) <= BAND  # NaN (scan) counts as outside
    s = {}
    if graph == "graph1":
        s["start_angle_deg"] = float(a[np.argmin(np.abs(t))])
        # R2 reach = last entry into the band that is then held for 5 s (same as command 10)
        reach = None
        after = np.where(t >= 0)[0]
        for k in after:
            if inside[k] and (k == 0 or not inside[k - 1] or reach is None):
                j = k
                while j < len(t) and inside[j]:
                    j += 1
                if t[min(j, len(t) - 1)] - t[k] >= R2_CONFIRM_S:
                    reach = t[k]
                    break
        s["r2_reach_s"] = reach
        s["r2"] = "PASS" if reach is not None and reach <= R2_LIMIT_S else "FAIL"
        # R1 hold: from the R2 arrival to the end of the run (command 10 times R1 from the arrival too)
        hold = (t >= reach) & np.isin(d["phase"], ["SLEW", "POINT"]) if reach is not None else np.zeros(len(t), bool)
        if hold.any():
            ah = np.abs(a[hold])
            s.update(r1_hold_s=float(t[hold][-1] - t[hold][0]), r1_in10_pct=100 * np.mean(ah <= BAND),
                     r1_in5_pct=100 * np.mean(ah <= 5.0), r1_max_deg=float(np.nanmax(ah)),
                     r1_rms_deg=float(np.sqrt(np.nanmean(ah ** 2))))
            s["r1"] = "PASS" if s["r1_in10_pct"] == 100 and s["r1_hold_s"] >= sim.DEMO_R1_HOLD_MS / 1000 - 0.2 else "FAIL"
        else:
            s["r1"] = "not reached"
    else:
        s["fans"] = fan_label(d)
        on = t >= 0
        lost = first_t({"t": t}, on & ~inside)
        s["pointing_held_s"] = lost
        s["first_at_limit_s"] = first_t({"t": t}, on & d["sat"])
        s["saturated_s"] = first_long_run(t, on & d["sat"], SAT_HOLD_S)
        s["run_after_fans_s"] = float(t[-1])
        held_until = lost if lost is not None else t[-1]
        s["max_err_while_held_deg"] = float(np.nanmax(np.abs(a[on & (t < held_until)]))) if (on & (t < held_until)).any() else None
        if graph == "graph2":
            s["r3"] = "PASS" if lost is None or lost >= R3_JUDGE_S else "FAIL"
        else:
            # finished dumps = DUMP stretches that ended before the run did
            dmp = d["dumping"].astype(int)
            s["dumps"] = int(np.sum(np.diff(dmp) == -1))
            first_dump_end = first_t({"t": t}, np.concatenate([[False], np.diff(dmp) == -1]))
            s["s2"] = ("PASS" if s["dumps"] >= 1 and (lost is None or lost > first_dump_end) else "FAIL")
    return s


# ------------------------------------------------------------------------------------------- plot
def style_axis(ax, graph, side):
    ax.axhspan(-BAND, BAND, color=BAND_FILL, alpha=0.6, lw=0, zorder=0)
    ax.axhline(0, color=INK2, lw=0.8, zorder=1)
    ax.text(1.005, 0, "Sun", transform=ax.get_yaxis_transform(), va="center", ha="left",
            fontsize=8, color=INK2, clip_on=False)
    ax.axvline(0, color=INK2, lw=0.8, ls=":", zorder=1)
    if graph == "graph1":
        ax.set_ylim(-190, 190)
        ax.set_yticks(range(-180, 181, 45))
        ax.axvline(R2_LIMIT_S, color=INK2, lw=1, ls="--", zorder=1)
        ax.text(R2_LIMIT_S, 185, " R2 limit 10 s", fontsize=8, color=INK2, va="top")
        ax.set_xlabel("time since the slew back started (s)")
    else:
        ax.set_ylim(-GRAPH23_YLIM, GRAPH23_YLIM)
        ax.set_yticks(range(-45, 46, 15))
        ax.set_xlabel("time since the fans turned on (s)")
    ax.set_ylabel("solar array angle from Sun (deg)")
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.set_title(side, fontsize=10, color=INK, loc="left")


def draw(ax, graph, runs, side):
    """runs: list of (label, data dict). Returns the rows for the graph's CSV."""
    style_axis(ax, graph, side)
    rows = []
    if not runs:
        ax.text(0.5, 0.75, "no hardware run yet\n" + {"graph1": "command 10 → 1",
                                                       "graph2": "command 10 → 2 ccw 50 and 2 ccw 100",
                                                       "graph3": "command 10 → 3"}[graph],
                transform=ax.transAxes, ha="center", va="center", color=INK2, fontsize=10)
        ax.set_xlim(-5 if graph == "graph1" else -PRE_FANS_S, 60)
        return rows
    colors = [BLUE, ORANGE, AQUA, YELLOW, MAGENTA]
    x_max = 0.0
    for i, (label, d) in enumerate(runs):
        t0 = time_zero(d, graph)
        if t0 is None:
            continue
        c = colors[i % len(colors)]
        if graph != "graph1":
            c = FAN_COLORS.get(fan_label(d), c)
        t = d["t"] - t0
        start = (first_t(d, d["phase"] == "AWAY") - t0 - 1.0) if graph == "graph1" else -PRE_FANS_S
        keep = t >= start
        t, a = t[keep], unwrap_deg(d["array_angle"][keep])
        x_max = max(x_max, t[-1])
        if graph == "graph3":  # shade each momentum dump
            dmp = d["dumping"][keep].astype(int)
            starts = t[np.where(np.diff(dmp) == 1)[0] + 1]
            ends = t[np.where(np.diff(dmp) == -1)[0] + 1]
            if dmp[0]:
                starts = np.r_[t[0], starts]
            if len(ends) < len(starts):
                ends = np.r_[ends, t[-1]]
            for k, (s0, s1) in enumerate(zip(starts, ends)):
                ax.axvspan(s0, s1, color=YELLOW, alpha=0.18, lw=0, zorder=0,
                           label="thrusters firing (dump)" if (k == 0 and i == 0) else None)
        sc = score(d, graph)
        if graph == "graph2" and sc.get("saturated_s") is not None:
            ax.axvline(sc["saturated_s"], color=c, lw=1, ls="--", zorder=2)
            ax.text(sc["saturated_s"], GRAPH23_YLIM * (0.92 - 0.12 * i), f"wheel saturated {sc['saturated_s']:.0f} s ",
                    color=INK, fontsize=8, va="top", ha="right")
        if graph == "graph3" and sc.get("pointing_held_s") is not None:
            ax.axvline(sc["pointing_held_s"], color=c, lw=1, ls="--", zorder=2)
            ax.text(sc["pointing_held_s"], -GRAPH23_YLIM * 0.85, f"pointing lost {sc['pointing_held_s']:.0f} s ",
                    color=INK, fontsize=8, ha="right")
        if graph == "graph1" and sc.get("r2_reach_s") is not None:
            y_reach = float(np.interp(sc["r2_reach_s"], t, np.nan_to_num(a)))  # on the plotted (unwrapped) trace
            up = 1 if y_reach >= 0 else -1
            ax.plot(sc["r2_reach_s"], y_reach, "o", ms=8, color=c, mec="white", mew=2, zorder=4)
            ax.annotate(f"reached ±10° at {sc['r2_reach_s']:.1f} s", (sc["r2_reach_s"], y_reach),
                        xytext=(14, 40 * up), textcoords="offset points", fontsize=8, color=INK,
                        arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
        ax.plot(t, a, color=c, lw=2, label=label, zorder=3)
        for k in np.where(keep)[0]:
            rows.append([label, f"{d['t'][k] - t0:.3f}", "" if math.isnan(d["array_angle"][k]) else f"{d['array_angle'][k]:.2f}",
                         d["phase"][k], f"{d['w_meas'][k]:.1f}",
                         f"{max(d['fan4'][k], d['fan27'][k]) if not d['dumping'][k] else min(d['fan4'][k], d['fan27'][k]):.0f}",
                         f"{max(d['fan4'][k], d['fan27'][k]) if d['dumping'][k] else 0:.0f}", int(d["sat"][k])])
    ax.set_xlim(left=(ax.get_xlim()[0] if graph == "graph1" else -PRE_FANS_S), right=x_max * 1.02 + 1)
    if graph == "graph1":
        ax.set_xlim(left=min(ax.get_xlim()[0], -5))
    ax.legend(loc="lower right", fontsize=8, frameon=False)
    return rows


# ------------------------------------------------------------------------------------------- slide
# Graph 3 on its own, made for a projector: big text, thick lines, and two stacked panels that tell
# the story (top = pointing, bottom = wheel speed). Same data as the small panel in requirements_6graphs.
SLIDE_FONT = 15
DUMP_FILL = "#f5c542"      # stronger yellow than the 6-panel graph so it survives a washed-out projector
SLIDE_BAND = "#b9d7b0"     # green-gray band = "good" zone


def dump_spans(t, dumping):
    dmp = dumping.astype(int)
    starts = list(t[np.where(np.diff(dmp) == 1)[0] + 1])
    ends = list(t[np.where(np.diff(dmp) == -1)[0] + 1])
    if dmp[0]:
        starts.insert(0, t[0])
    if len(ends) < len(starts):
        ends.append(t[-1])
    return list(zip(starts, ends))


def draw_graph3_slide(label, d, side, out_path, compare=None):
    """label, d: the run with thrusters. compare: optional (label, d) of the same fans without thrusters."""
    plt.rcParams.update({"font.size": SLIDE_FONT, "axes.labelsize": SLIDE_FONT, "xtick.labelsize": SLIDE_FONT - 1,
                         "ytick.labelsize": SLIDE_FONT - 1, "axes.edgecolor": INK, "xtick.color": INK,
                         "ytick.color": INK, "axes.labelcolor": INK})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8.5), sharex=True, gridspec_kw={"height_ratios": [3, 2]})

    t0 = time_zero(d, "graph3")
    t = d["t"] - t0
    keep = t >= -PRE_FANS_S
    t, a = t[keep], unwrap_deg(d["array_angle"][keep])
    # wheel: the dumps trigger on the COMMANDED speed, so plot that (2 s average hides the ±3° aim dither)
    n_avg = max(1, int(round(2.0 / max(np.median(np.diff(t)), 1e-3))))
    box = np.ones(n_avg)
    w = np.convolve(d["w_cmd"][keep], box, mode="same") / np.convolve(np.ones(keep.sum()), box, mode="same")
    spans = dump_spans(t, d["dumping"][keep])
    sc = score(d, "graph3")

    for ax in (ax1, ax2):
        for k, (s0, s1) in enumerate(spans):
            ax.axvspan(s0, s1, color=DUMP_FILL, alpha=0.45, lw=0, zorder=0)
        ax.axvline(0, color=INK, lw=1.5, ls=":", zorder=1)
        ax.grid(axis="y", color=GRID, lw=1)
        ax.set_axisbelow(True)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_linewidth(1.5)

    # ---- top: pointing
    ax1.axhspan(-BAND, BAND, color=SLIDE_BAND, alpha=0.7, lw=0, zorder=0)
    ax1.axhline(0, color=INK, lw=1.2, zorder=1)
    ax1.text(1.005, BAND, f"+{BAND:.0f}°", transform=ax1.get_yaxis_transform(), va="center", fontsize=SLIDE_FONT - 2,
             color=INK, clip_on=False)
    ax1.text(1.005, -BAND, f"−{BAND:.0f}°", transform=ax1.get_yaxis_transform(), va="center", fontsize=SLIDE_FONT - 2,
             color=INK, clip_on=False)
    ax1.text(1.005, 0, "Sun", transform=ax1.get_yaxis_transform(), va="center", fontsize=SLIDE_FONT - 1,
             fontweight="bold", color=INK, clip_on=False)
    ylim = 40.0
    if compare is not None:
        cl, cd = compare
        ct0 = time_zero(cd, "graph3")
        ct = cd["t"] - ct0
        ck = ct >= -PRE_FANS_S
        ca = unwrap_deg(cd["array_angle"][ck])
        ax1.plot(ct[ck], ca, color=ORANGE, lw=2.5, zorder=2, label=cl)
        csc = score(cd, "graph2")
        if csc.get("pointing_held_s") is not None:
            tl = csc["pointing_held_s"]
            y_lost = np.sign(np.interp(tl, ct[ck], np.nan_to_num(ca))) * BAND
            ax1.plot(tl, y_lost, "X", ms=16, color=ORANGE, mec="white", mew=2, zorder=5)
            ax1.annotate(f"no thrusters:\nwheel saturates,\nSun lost at {tl:.0f} s", (tl, y_lost),
                         xytext=(tl + 25, -ylim * 0.62), fontsize=SLIDE_FONT - 1, color=ORANGE, fontweight="bold",
                         va="center", arrowprops=dict(arrowstyle="-", color=ORANGE, lw=2))
    ax1.plot(t, a, color=BLUE, lw=2.2, zorder=3, label=label)
    ax1.set_ylim(-ylim, ylim)
    ax1.set_yticks([-30, -10, 0, 10, 30])
    ax1.set_ylabel("array angle\nfrom Sun (deg)")
    held = sc.get("pointing_held_s")
    msg = (f"With thrusters: held the whole {sc['run_after_fans_s']:.0f} s run, worst error {sc['max_err_while_held_deg']:.1f}°"
           if held is None else f"With thrusters: pointing lost at {held:.0f} s")
    ax1.text(0.99, 0.95, msg, transform=ax1.transAxes, ha="right", va="top", fontsize=SLIDE_FONT, fontweight="bold",
             color=BLUE, bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="none", alpha=0.9))

    # ---- bottom: wheel speed (why the dumps happen)
    wl = sim.WHEEL_MAX_RPM
    sgn = 1 if np.nanmedian(w[t > 0]) >= 0 else -1      # only show the side the wheel actually spins
    ax2.axhspan(sgn * wl, sgn * wl * 1.3, color=ORANGE, alpha=0.18, lw=0, zorder=0)
    ax2.axhline(sgn * wl, color=ORANGE, lw=2, zorder=1)
    ax2.text(1.005, sgn * wl, "wheel limit", transform=ax2.get_yaxis_transform(), va="center", fontsize=SLIDE_FONT - 2,
             color=ORANGE, fontweight="bold", clip_on=False)
    for lvl, txt in ((sim.DESAT_START_RPM, "dump\nstarts"), (sim.DESAT_STOP_RPM, "dump\nstops")):
        ax2.axhline(sgn * lvl, color=INK2, lw=1.2, ls="--", zorder=1)
        ax2.text(1.005, sgn * lvl, txt, transform=ax2.get_yaxis_transform(), va="center", fontsize=SLIDE_FONT - 3,
                 color=INK2, clip_on=False)
    ax2.plot(t, w, color=INK, lw=2.5, zorder=3)
    ax2.set_ylim(*sorted((-sgn * 60, sgn * wl * 1.12)))
    ax2.set_yticks([sgn * v for v in (0, 100, 200, 300, 400)])
    ax2.set_ylabel("wheel speed cmd\n(RPM, 2 s avg)")
    ax2.set_xlabel("time since the disturbance fans turned on (s)")
    ax2.set_xlim(-PRE_FANS_S, t[-1] * 1.01)
    if spans:
        s0, s1 = spans[0]
        ax2.annotate("fans spin the\nwheel up", (s0 * 0.6, sgn * 0.6 * sim.DESAT_START_RPM),
                     xytext=(s0 * 0.08, sgn * wl * 0.85), fontsize=SLIDE_FONT - 2, color=INK, ha="left", va="center",
                     arrowprops=dict(arrowstyle="->", color=INK, lw=1.5))
        ax2.text((s0 + s1) / 2, sgn * 20, "thrusters\nslow it", va="bottom", fontsize=SLIDE_FONT - 2, color="#8a5a00",
                 fontweight="bold", ha="center")

    fig.suptitle("S2: thrusters dump wheel momentum, so the array stays on the Sun", fontsize=SLIDE_FONT + 4,
                 fontweight="bold", x=0.01, ha="left", color=INK)
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=BLUE, lw=3, label="with thrusters")]
    if compare is not None:
        handles.append(Line2D([], [], color=ORANGE, lw=3, label="no thrusters (same fans)"))
    handles += [Patch(color=DUMP_FILL, alpha=0.6, label="thrusters firing"),
                Patch(color=SLIDE_BAND, alpha=0.9, label=f"±{BAND:.0f}° requirement")]
    fig.text(0.01, 0.935, side, fontsize=SLIDE_FONT - 2, color=INK2, va="center")
    fig.legend(handles=handles, loc="upper right", ncol=len(handles), frameon=False, fontsize=SLIDE_FONT - 2,
               bbox_to_anchor=(0.99, 0.955))
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    plt.rcdefaults()


# ------------------------------------------------------------------------------------------- main
def fmt(v, unit=""):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.1f}{unit}"
    return f"{v}{unit}"


def write_summary(out, results):
    lines = ["# Requirement results", "",
             "Pointing = solar array angle from the Sun (gyro heading − scan sun heading). ±10° band = R1.", ""]
    flat = []
    for graph, side, label, s in results:
        flat.append({"graph": graph, "source": side, "run": label, **{k: v for k, v in s.items()}})
    lines += ["## Graph 1 · R1 + R2", "", "| Source | Run | Start angle | R2 reach (limit 10 s) | R2 | R1 hold | in ±10° | in ±5° | max | RMS | R1 |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in flat:
        if r["graph"] == "graph1":
            lines.append(f"| {r['source']} | {r['run']} | {fmt(r.get('start_angle_deg'), '°')} | {fmt(r.get('r2_reach_s'), ' s')} | "
                         f"{r.get('r2', '—')} | {fmt(r.get('r1_hold_s'), ' s')} | {fmt(r.get('r1_in10_pct'), '%')} | "
                         f"{fmt(r.get('r1_in5_pct'), '%')} | {fmt(r.get('r1_max_deg'), '°')} | {fmt(r.get('r1_rms_deg'), '°')} | {r.get('r1', '—')} |")
    lines += ["", "## Graph 2 · R3 (fans)", "", "| Source | Run | Fans | Pointing held | Wheel first at limit | Wheel saturated (≥2 s) | Max error while held | R3 (30 s) |",
              "|---|---|---|---|---|---|---|---|"]
    for r in flat:
        if r["graph"] == "graph2":
            held = fmt(r.get("pointing_held_s"), " s") if r.get("pointing_held_s") is not None else f"never lost ({fmt(r.get('run_after_fans_s'), ' s')} run)"
            lines.append(f"| {r['source']} | {r['run']} | {r.get('fans', '—')} | {held} | {fmt(r.get('first_at_limit_s'), ' s')} | {fmt(r.get('saturated_s'), ' s')} | "
                         f"{fmt(r.get('max_err_while_held_deg'), '°')} | {r.get('r3', '—')} |")
    lines += ["", "## Graph 3 · S2 (momentum dump)", "", "| Source | Run | Disturbance | Dumps finished | Pointing held | Max error while held | S2 |",
              "|---|---|---|---|---|---|---|"]
    for r in flat:
        if r["graph"] == "graph3":
            held = fmt(r.get("pointing_held_s"), " s") if r.get("pointing_held_s") is not None else f"never lost ({fmt(r.get('run_after_fans_s'), ' s')} run)"
            lines.append(f"| {r['source']} | {r['run']} | {r.get('fans', '—')} | {r.get('dumps', '—')} | {held} | "
                         f"{fmt(r.get('max_err_while_held_deg'), '°')} | {r.get('s2', '—')} |")
    lines += ["", "Times are from the requirement's start: the slew back (graph 1) or fans on (graphs 2, 3).",
              f"Model fan torque: {sim.FAN_TAU_100_NM * 1e3:.2f} mN·m per pair at 100% (2 Nov hold tests), ~speed² below that."]
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    keys = sorted({k for r in flat for k in r} - {"graph", "source", "run"})
    with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["graph", "source", "run"] + keys)
        for r in flat:
            w.writerow([r["graph"], r["source"], r["run"]] + ["" if r.get(k) is None else r.get(k) for k in keys])
    print("\n".join(lines))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="*", help="command 10 CSVs (R1R2_*, R3_*, S2_*), or older command 8/11 SunTrack_* files")
    p.add_argument("--out", default=os.path.join("6. FINAL", "requirements"))
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)

    hardware = {g: [] for g in TITLES}
    for path in sorted(args.csv):
        d = load_hardware(path)
        g = classify(d) if d else None
        if g is None:
            print(f"skipped {path} (not a command 10 / fan run)")
            continue
        label = d["name"] if g == "graph1" else f"{d['name']} ({fan_label(d)})"
        hardware[g].append((label, d))
    model = {g: [(lbl, from_model(lbl, kw)) for lbl, kw in runs] for g, runs in MODEL_RUNS.items()}

    header = ["run", "t(s)", "array_angle(deg)", "phase", "wheel(RPM)", "fan_dist(%)", "fan_thruster(%)", "saturated"]
    fig, axes = plt.subplots(3, 2, figsize=(15, 14))
    results = []
    for row, g in enumerate(TITLES):
        for col, (side, runs) in enumerate([("Hardware (KestrelSAT)", hardware[g]), ("Model (sun_tracker_sim.py)", model[g])]):
            tag = "hardware" if col == 0 else "model"
            rows = draw(axes[row, col], g, runs, side)
            # the same graph on its own
            f1, a1 = plt.subplots(figsize=(10, 5))
            draw(a1, g, runs, side)
            f1.suptitle(TITLES[g], fontsize=12, x=0.01, ha="left")
            f1.tight_layout()
            f1.savefig(os.path.join(args.out, f"{g}_{tag}.png"), dpi=150)
            plt.close(f1)
            if g == "graph3" and runs:
                # projector version replaces the plain one; the model gets a "no thrusters" run to compare
                compare = None
                if tag == "model":
                    kw = dict(MODEL_RUNS["graph3"][0][1], desat=False, demo_req=2)
                    compare = ("no thrusters", from_model("no thrusters", kw))
                draw_graph3_slide(*runs[0], side, os.path.join(args.out, f"{g}_{tag}.png"), compare)
            with open(os.path.join(args.out, f"{g}_{tag}.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(rows)
            results += [(g, tag, lbl, score(d, g)) for lbl, d in runs]
        # share the x range across the row so hardware and model line up
        lo = min(axes[row, 0].get_xlim()[0], axes[row, 1].get_xlim()[0])
        hi = max(axes[row, 0].get_xlim()[1], axes[row, 1].get_xlim()[1])
        for a in axes[row]:
            a.set_xlim(lo, hi)
        axes[row, 0].annotate(TITLES[g], (0, 1.13), xycoords="axes fraction", fontsize=12, fontweight="bold", color=INK)
    fig.tight_layout(h_pad=4)
    fig.savefig(os.path.join(args.out, "requirements_6graphs.png"), dpi=150)
    plt.close(fig)
    write_summary(args.out, results)
    print(f"\nGraphs, CSVs and summary saved to {args.out}")


if __name__ == "__main__":
    main()
