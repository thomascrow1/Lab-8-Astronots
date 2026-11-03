"""
fan_thrust_model.py
===========================================================
Theoretical thrust / disturbance-torque curves for the four LD3007MS disturbance fans
(command 17), and how long the reaction wheel can absorb that torque before it saturates.
Optionally measures the real disturbance torque from a command 13 run.

Usage:
  python fan_thrust_model.py                              # table + fan_thrust_model.png
  python fan_thrust_model.py --csv SunTrack_data060.csv   # also: measured torque from that run

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
  A is the swept annulus of a 30 mm fan (27 mm rotor, 13 mm hub). d0 is a guess; measure it with
  command 17 (lowest duty that still spins after the kick).
  T_MAX is bracketed: LOW = jet leaves at the rotor's axial speed (ducted fan, the usual case for a
  frame fan); HIGH = open-rotor momentum theory (jet speeds up to 2x after the rotor). The static
  pressure gives a hard ceiling DP_MAX * A that no free-air thrust can exceed.
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

RHO = 1.20                # air density (kg/m^3); Colorado Springs (~2200 m) is ~0.98, which lowers T by ~18%
CFM_TO_M3S = 4.719e-4
INH2O_TO_PA = 249.09
RPM_TO_RADS = 2 * math.pi / 60

A = math.pi / 4 * (D_ROTOR_M ** 2 - D_HUB_M ** 2)
Q_MAX = Q_MAX_CFM * CFM_TO_M3S
T_MAX_LOW = RHO * Q_MAX ** 2 / A          # ducted: jet speed = Q / A
T_MAX_HIGH = 2 * T_MAX_LOW                # open rotor: far-wake speed = 2 x rotor speed
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
    print(f"Thrust per fan at 100%: {T_MAX_LOW * 1e3:.1f} (ducted) to {T_MAX_HIGH * 1e3:.1f} mN (open rotor);"
          f" ceiling DP*A = {T_CEILING * 1e3:.1f} mN  ({T_MAX_LOW / 9.81e-3:.2f}-{T_MAX_HIGH / 9.81e-3:.2f} gram-force)")
    print(f"Wheel momentum 0 -> {W_MAX_RPM:.0f} RPM: H_max = {H_MAX:.4f} N*m*s")
    print()
    print(" duty  speed   thrust/fan    Z torque/pair      wheel accel     time to saturate   PD offset")
    print("  (%)  (RPM)      (mN)          (mN*m)            (RPM/s)              (s)            (deg)")
    for pct in (100, 90, 80, 70, 60, 50, 40, 30):
        d = pct / 100
        n = N_MAX_RPM * speed_fraction(d)
        lo, hi = thrust(d, T_MAX_LOW), thrust(d, T_MAX_HIGH)
        tlo, thi = pair_torque(d, T_MAX_LOW), pair_torque(d, T_MAX_HIGH)
        alo, ahi = (tlo / I_WHEEL / RPM_TO_RADS, thi / I_WHEEL / RPM_TO_RADS)
        slo, shi = sat_time(thi), sat_time(tlo)
        elo, ehi = math.degrees(tlo / KP), math.degrees(thi / KP)
        print(f"  {pct:3d}  {n:5.0f}   {lo * 1e3:4.1f} - {hi * 1e3:4.1f}   {tlo * 1e3:4.2f} - {thi * 1e3:4.2f}"
              f"       {alo:5.2f} - {ahi:5.2f}      {slo:6.0f} - {shi:6.0f}       {elo:3.1f} - {ehi:3.1f}")
    print()
    print("Torque assumes each GPIO pair is a couple about Z (+Y fan and -Y fan push opposite ways).")
    print("If both fans of a pair push the SAME way, their Z torques cancel and the pair only makes a")
    print(f"tilt torque 2*{Z_OFF_M}*T about Y ({2 * Z_OFF_M / (2 * R_ARM_M):.0%} of the numbers above).")
    print("PD offset = steady pointing error the PD needs to cancel the torque (tau / Kp), on top of the 1 deg deadband.")


def plot(path="fan_thrust_model.png"):
    blue = "#2a78d6"
    duty = np.linspace(0, 1, 201)
    pct = duty * 100
    t_lo, t_hi = pair_torque(duty, T_MAX_LOW), pair_torque(duty, T_MAX_HIGH)
    t_mid = (t_lo + t_hi) / 2

    fig, axs = plt.subplots(2, 2, figsize=(11, 7.5))
    panels = [
        (axs[0, 0], N_MAX_RPM * speed_fraction(duty), None, None, "Fan speed (RPM)"),
        (axs[0, 1], thrust(duty, (T_MAX_LOW + T_MAX_HIGH) / 2) * 1e3,
         thrust(duty, T_MAX_LOW) * 1e3, thrust(duty, T_MAX_HIGH) * 1e3, "Thrust per fan (mN)"),
        (axs[1, 0], t_mid * 1e3, t_lo * 1e3, t_hi * 1e3, "Z torque per pair (mN·m)"),
        (axs[1, 1], sat_time(t_mid), sat_time(t_hi), sat_time(t_lo), "Time until wheel saturates (s)"),
    ]
    for ax, mid, lo, hi, title in panels:
        if lo is not None:
            ax.fill_between(pct, lo, hi, color=blue, alpha=0.18, linewidth=0, label="ducted ↔ open-rotor bound")
        ax.plot(pct, mid, color=blue, linewidth=2)
        ax.set_title(title, loc="left", fontsize=11)
        ax.set_xlabel("PWM duty (%)")
        ax.set_xlim(0, 100)
        ax.grid(True, color="0.9", linewidth=0.8)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.axvline(D0 * 100, color="0.6", linewidth=1, linestyle=":")
    axs[0, 0].text(D0 * 100 + 1, N_MAX_RPM * 0.9, f"stall duty d0 = {D0:.0%} (guess)", fontsize=8, color="0.4")
    axs[0, 1].legend(frameon=False, fontsize=8, loc="upper left")
    axs[1, 1].set_yscale("log")
    axs[1, 1].set_ylim(10, 1e4)
    fig.suptitle("LD3007MS disturbance fans: theoretical curves (fan laws, proxy datasheet)", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved {path}")


def measure(path):
    """Disturbance torque from a command 13 run: tau = I_wheel * (slope of measured wheel speed)."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if "fan_gpio27(%)" not in rows[0]:
        raise SystemExit(f"{path} has no fan columns (made before command 17 existed).")
    t = np.array([float(r["mcu time(ms)"]) for r in rows]) / 1000
    w = np.array([float(r["w_RW_meas(RPM)"]) for r in rows])
    mode = np.array([r["mode"] for r in rows])
    fan = np.array([max(float(r["fan_gpio27(%)"]), float(r["fan_gpio4(%)"])) for r in rows])
    sat = np.array([int(r["saturated"]) for r in rows])

    def slope(mask, label):
        if mask.sum() < 20:
            print(f"  {label}: not enough TRACK data ({mask.sum()} points)")
            return None
        rpm_s = np.polyfit(t[mask], w[mask], 1)[0]
        tau = I_WHEEL * rpm_s * RPM_TO_RADS
        print(f"  {label}: wheel {rpm_s:+.2f} RPM/s over {t[mask][-1] - t[mask][0]:.0f} s -> torque {tau * 1e3:+.3f} mN*m")
        return tau

    track = mode == "TRACK"
    on = track & (fan > 0) & (sat == 0)
    if on.any():
        on &= t > t[on][0] + 2.0          # skip the kick and the first transient
    print(f"Measured from {path}:")
    base = slope(track & (fan == 0), "fans off (string twist etc.)")
    with_fans = slope(on, "fans on")
    if base is not None and with_fans is not None:
        print(f"  -> fan disturbance torque = {(with_fans - base) * 1e3:+.3f} mN*m"
              f" (theory at this duty: {pair_torque(fan.max() / 100, T_MAX_LOW) * 1e3:.2f}-"
              f"{pair_torque(fan.max() / 100, T_MAX_HIGH) * 1e3:.2f} mN*m per pair)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", help="command 13 SunTrack CSV run with the fans on")
    parser.add_argument("--out", default="fan_thrust_model.png")
    args = parser.parse_args()
    print_table()
    plot(args.out)
    if args.csv:
        print()
        measure(args.csv)


if __name__ == "__main__":
    main()
