"""
fan_thrust_model.py
===========================================================
Theoretical thrust / disturbance-torque curves for the four LD3007MS disturbance fans,
and how long the reaction wheel can absorb that torque before it saturates.
Optionally measures the real disturbance torque from a command 8 run.

Usage:
  python fan_thrust_model.py                              # table + fan_thrust_model.png
  python fan_thrust_model.py --csv SunTrack_data060.csv   # also: measured torque from that run
  python fan_thrust_model.py --csv SunTrack_data019.csv --baseline SunTrack_data016.csv SunTrack_data017.csv
                                                          # ...minus the fans-off background

Model (see the bottom of this docstring for where each number comes from):
  speed     N(d) = N_MAX * (d - d0) / (1 - d0)       for duty d > d0 (stall below d0)
  airflow   Q = Q_MAX * N / N_MAX                     (fan law: Q ~ N)
  thrust    T = rho * Q^2 / A                         (momentum flux of the jet, jet speed = Q / A)
            -> T ~ N^2 (fan law), so T(d) = T_MAX * ((d - d0) / (1 - d0))^2
  torque    tau_z = 2 * r * T                         per diagonal pair (+Y and -Y fans push opposite ways)
  wheel     H_max = I_WHEEL * w_max                   momentum the wheel can absorb from 0 RPM
            t_sat = H_max / tau_z                     how long pointing can be held

Numbers:
  The LD3007MS has no published RPM/airflow. N_MAX, Q_MAX, DP_MAX are from the closest datasheet
  match with the same rating (CUI/Same Sky CFM-3007CF-085-264: 5 V, 0.2 A, 30x30x7 mm).
  A is the swept annulus of a 30 mm fan (27 mm rotor, 13 mm hub). d0 is a guess; measure it on the
  bench (lowest duty that still spins after the kick).
  Thrust uses the ducted-fan model (the jet leaves at the rotor's axial speed, Q / A), which fits a
  framed fan. Open-rotor momentum theory would give 2x this, but the 2 Nov hold tests (SunTrack_data019,
  020: 2.04 and 2.78 mN*m per pair at 100% after removing the fans-off background from 016-018, fans on
  5 V, mean 2.41) match the ducted model at Colorado Springs air density. The static pressure gives a hard ceiling DP_MAX * A that no free-air thrust can exceed.
"""

import argparse
import csv
import math

import matplotlib.pyplot as plt
import numpy as np

# Fan (proxy datasheet: CFM-3007CF-085-264)
N_MAX_RPM = 8500.0
Q_MAX_CFM = 2.81
DP_MAX_INH2O = 0.12
D_ROTOR_M = 0.027
D_HUB_M = 0.013
D0 = 0.20                 # duty below which the fan stalls (GUESS - measure it)

# KestrelSAT (definitions.h)
R_ARM_M = 0.32            # fans at y = +/-32 cm
Z_OFF_M = 0.02            # fans 2 cm above / below centre
I_WHEEL = 0.0035          # kg*m^2
W_MAX_RPM = 400.0         # WHEEL_MAX_RPM (also -400)
KP = 0.126                # DEFAULT_KP, N*m/rad

RHO = 0.98                # air density (kg/m^3) at Colorado Springs (~2200 m); sea level is 1.20
CFM_TO_M3S = 4.719e-4
INH2O_TO_PA = 249.09
RPM_TO_RADS = 2 * math.pi / 60

A = math.pi / 4 * (D_ROTOR_M ** 2 - D_HUB_M ** 2)
Q_MAX = Q_MAX_CFM * CFM_TO_M3S
T_MAX = RHO * Q_MAX ** 2 / A              # ducted: jet speed = Q / A
MEASURED_MNM = (2.04, 2.78)               # 2 Nov hold tests at 100%, background removed: CW (019), CCW (020), mN*m per pair
T_CEILING = DP_MAX_INH2O * INH2O_TO_PA * A
H_MAX = I_WHEEL * W_MAX_RPM * RPM_TO_RADS


def speed_fraction(duty):
    """N / N_MAX for duty 0..1 (0 below the stall duty d0)."""
    return np.clip((np.asarray(duty, dtype=float) - D0) / (1 - D0), 0.0, 1.0)


def thrust(duty, t_max):
    """Thrust of one fan (N)."""
    return t_max * speed_fraction(duty) ** 2


def pair_torque(duty, t_max):
    """Z torque of one pair (+Y fan and -Y fan pushing opposite ways), N*m."""
    return 2 * R_ARM_M * thrust(duty, t_max)


def sat_time(tau):
    """Seconds until the wheel goes from 0 to W_MAX_RPM under a steady torque tau (inf if tau = 0)."""
    tau = np.asarray(tau, dtype=float)
    with np.errstate(divide="ignore"):
        return np.where(tau > 0, H_MAX / np.where(tau > 0, tau, 1), np.inf)


def print_table():
    print(f"Swept area A = {A * 1e4:.2f} cm^2, Q_max = {Q_MAX * 1e3:.2f} L/s, jet speed = {Q_MAX / A:.2f} m/s")
    print(f"Thrust per fan at 100%: {T_MAX * 1e3:.1f} mN ({T_MAX / 9.81e-3:.2f} gram-force);"
          f" ceiling DP*A = {T_CEILING * 1e3:.1f} mN")
    print(f"Torque per pair at 100%: {pair_torque(1.0, T_MAX) * 1e3:.2f} mN*m"
          f" (measured {MEASURED_MNM[0]:.2f} CW, {MEASURED_MNM[1]:.2f} CCW)")
    print(f"Wheel momentum 0 -> {W_MAX_RPM:.0f} RPM: H_max = {H_MAX:.4f} N*m*s")
    print()
    print(" duty  speed  thrust/fan  Z torque/pair  wheel accel  time to saturate  PD offset")
    print("  (%)  (RPM)     (mN)         (mN*m)       (RPM/s)          (s)           (deg)")
    for pct in (100, 90, 80, 70, 60, 50, 40, 30):
        d = pct / 100
        n = N_MAX_RPM * speed_fraction(d)
        T, tau = thrust(d, T_MAX), pair_torque(d, T_MAX)
        accel = tau / I_WHEEL / RPM_TO_RADS
        print(f"  {pct:3d}  {n:5.0f}     {T * 1e3:4.1f}         {tau * 1e3:4.2f}         {accel:5.2f}"
              f"          {float(sat_time(tau)):6.0f}          {math.degrees(tau / KP):3.1f}")
    print()
    print("Torque assumes each GPIO pair is a couple about Z (+Y fan and -Y fan push opposite ways).")
    print("If both fans of a pair push the SAME way, their Z torques cancel and the pair only makes a")
    print(f"tilt torque 2*{Z_OFF_M}*T about Y ({2 * Z_OFF_M / (2 * R_ARM_M):.0%} of the numbers above).")
    print("PD offset = steady pointing error the PD needs to cancel the torque (tau / Kp), on top of the 1 deg deadband.")


def plot(path="fan_thrust_model.png"):
    blue = "#2a78d6"
    duty = np.linspace(0, 1, 201)
    pct = duty * 100
    tau = pair_torque(duty, T_MAX)

    fig, axs = plt.subplots(2, 2, figsize=(11, 7.5))
    panels = [
        (axs[0, 0], N_MAX_RPM * speed_fraction(duty), "Fan speed (RPM)"),
        (axs[0, 1], thrust(duty, T_MAX) * 1e3, "Thrust per fan (mN)"),
        (axs[1, 0], tau * 1e3, "Z torque per pair (mN·m)"),
        (axs[1, 1], sat_time(tau), "Time until wheel saturates (s)"),
    ]
    for ax, y, title in panels:
        ax.plot(pct, y, color=blue, linewidth=2, label="ducted-fan model")
        ax.set_title(title, loc="left", fontsize=11)
        ax.set_xlabel("PWM duty (%)")
        ax.set_xlim(0, 100)
        ax.grid(True, color="0.9", linewidth=0.8)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.axvline(D0 * 100, color="0.6", linewidth=1, linestyle=":")
    axs[0, 0].text(D0 * 100 + 1, N_MAX_RPM * 0.9, f"stall duty d0 = {D0:.0%} (guess)", fontsize=8, color="0.4")
    axs[1, 0].plot([100, 100], MEASURED_MNM, "o", color="#d6762a", label="measured (2 Nov hold tests)")
    axs[1, 0].legend(frameon=False, fontsize=8, loc="upper left")
    axs[1, 1].set_yscale("log")
    axs[1, 1].set_ylim(10, 1e4)
    fig.suptitle("LD3007MS disturbance fans: theoretical curves (fan laws, proxy datasheet)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved {path}")


BASELINE_SKIP_S = 10.0      # fans-off runs: skip the slew back to the sun after each scan (as fan_hold_report.py)


def load_run(path, need_fans=True):
    """Columns needed from a SunTrack CSV (time in s). Runs from before the fans existed read as fans off."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    has_fans = "fan_gpio27(%)" in rows[0]
    if need_fans and not has_fans:
        raise SystemExit(f"{path} has no fan columns (made before the fans were added).")
    return {
        "t": np.array([float(r["mcu time(ms)"]) for r in rows]) / 1000,
        "w": np.array([float(r["w_RW_meas(RPM)"]) for r in rows]),
        "mode": np.array([r["mode"] for r in rows]),
        "fan": np.array([max(float(r["fan_gpio27(%)"]), float(r["fan_gpio4(%)"])) if has_fans else 0.0
                         for r in rows]),
        "sat": np.array([int(r["saturated"]) for r in rows]),
    }


def wheel_torque(d, mask, label):
    """Torque the wheel absorbs over the masked samples: I_wheel * (slope of measured wheel speed), N*m."""
    if mask.sum() < 20:
        print(f"  {label}: not enough TRACK data ({mask.sum()} points)")
        return None
    t = d["t"][mask]
    rpm_s = np.polyfit(t, d["w"][mask], 1)[0]
    tau = I_WHEEL * rpm_s * RPM_TO_RADS
    print(f"  {label}: wheel {rpm_s:+.2f} RPM/s over {t[-1] - t[0]:.0f} s -> torque {tau * 1e3:+.3f} mN*m")
    return tau


def baseline_torque(path):
    """Background torque (string twist etc.) from a separate fans-off command 4 run. Uses tracking
    samples at least BASELINE_SKIP_S after the most recent scan, so the slew back to the sun is skipped."""
    d = load_run(path, need_fans=False)
    if (d["fan"] > 0).any():
        print(f"  {path}: fans were on in this run; not used as a baseline")
        return None
    keep = np.zeros(len(d["t"]), dtype=bool)
    last_scan = -np.inf
    for i in range(len(d["t"])):
        if d["mode"][i] == "ACQUIRE":
            last_scan = d["t"][i]
        keep[i] = d["mode"][i] == "TRACK" and d["t"][i] - last_scan >= BASELINE_SKIP_S and not d["sat"][i]
    return wheel_torque(d, keep, f"baseline {path}")


def measure(path, baseline_paths):
    """Disturbance torque from a command 8 run: tau = I_wheel * (slope of measured wheel speed) with the
    fans on, minus the background torque from separate fans-off runs. The fans-off part of a command 8
    run is not used: it is the settling right after the scan, not steady tracking."""
    d = load_run(path)
    on = (d["mode"] == "TRACK") & (d["fan"] > 0) & (d["sat"] == 0)
    if on.any():
        on &= d["t"] > d["t"][on][0] + 2.0          # skip the kick and the first transient
    print(f"Measured from {path}:")
    with_fans = wheel_torque(d, on, "fans on")
    if with_fans is None:
        return
    theory = f"(theory at this duty: {pair_torque(d['fan'].max() / 100, T_MAX) * 1e3:.2f} mN*m per pair)"
    bases = [b for b in (baseline_torque(p) for p in baseline_paths) if b is not None]
    if bases:
        base = float(np.mean(bases))
        print(f"  -> fan disturbance torque = {(with_fans - base) * 1e3:+.3f} mN*m after removing the"
              f" {base * 1e3:+.3f} mN*m background (mean of {len(bases)} fans-off run(s)) {theory}")
    else:
        print(f"  -> fan disturbance torque = {with_fans * 1e3:+.3f} mN*m, no background removed"
              f" (add --baseline with fans-off command 4 runs) {theory}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", help="command 8 SunTrack CSV run with the fans on")
    parser.add_argument("--baseline", nargs="*", default=[],
                        help="fans-off command 4 SunTrack CSV(s) for the background torque")
    parser.add_argument("--out", default="fan_thrust_model.png")
    args = parser.parse_args()
    print_table()
    plot(args.out)
    if args.csv:
        print()
        measure(args.csv, args.baseline)


if __name__ == "__main__":
    main()
