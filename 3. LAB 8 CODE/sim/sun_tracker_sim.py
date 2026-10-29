"""
sun_tracker_sim.py
===========================================================
Single-axis (Z) simulation of the KestrelSAT sun tracker.

Models:
  - KestrelSAT body + reaction wheel (conservation of angular momentum)
  - Wheel motor as a first-order lag to the commanded speed (open-loop PWM)
  - 4 phototransistors with a 50 deg half-power angle (Lab 6)
  - Gyro and sun sensor noise, optional string torsion
  - Optional opposing fan banks, momentum supervisor, and thrust rise/coast-down

The controller is a line-by-line copy of KestrelSAT_SunTracker/src/sun_tracker.cpp
and fan_desaturation.cpp; timing/limits come from include/definitions.h.
If you change one, change the other. Fan thrust is unspecified by default.

Usage:
  python sun_tracker_sim.py                    # run all scenarios, save plots to sim/output/
  python sun_tracker_sim.py --kp 0.005 --kd 0.012
  python sun_tracker_sim.py --scenario step90 --show
  python -m unittest discover -s sim -p 'test_*.py' -v  # from 3. LAB 8 CODE/
"""

import argparse
from dataclasses import dataclass
import math
import os

import numpy as np

# ---------------------------------------------------------------- definitions.h mirror
MOTOR_VOLTAGE = 5.05
RPM_PER_PWM = 1000.0 * MOTOR_VOLTAGE / 12.0
SOLAR_ARRAY_AXIS_DEG = 180.0
I_BODY_Z = 0.014
I_WHEEL = 0.0035
DEFAULT_KP = 0.126
DEFAULT_KD = 0.1092
DEFAULT_MAX_RATE_DPS = 90.0
DEFAULT_BIAS_PWM = 0.0
POINTING_DEADBAND_DEG = 3.0
WHEEL_MIN_RPM = -400.0
WHEEL_MAX_RPM = 400.0
WHEEL_MAX_ACCEL_RPM_S = 400.0
WHEEL_STALL_RPM = 50.0
MOTOR_TAU_S = 1.1
SUN_LOST_THRESHOLD = 0.15
SUN_LOST_TIME_MS = 500
TRACK_SPINUP_MS = 3000
TRACK_RELEASE_MS = 8000
TRACK_CONTROL_MS = 10000
DT_CONTROL = 0.050  # interval_testPoint (s)
FAN_DESATURATION_ENABLED = False
FAN_TARGET_RPM = 0.0
FAN_START_RPM = 200.0
FAN_STOP_RPM = 100.0
FAN_COMMAND_START_RPM = 300.0
FAN_UNLOAD_DURING_SLEWS = True
FAN_START_ERROR_DEG = 5.0
FAN_STOP_ERROR_DEG = 10.0
FAN_MAX_BODY_RATE_DPS = 3.0
FAN_CALIBRATION_IN_RPM_S = True
FAN_POSITIVE_EQUIV_RPM_S = 0.0
FAN_NEGATIVE_EQUIV_RPM_S = 0.0
FAN_POSITIVE_TORQUE_NM = 0.0
FAN_NEGATIVE_TORQUE_NM = 0.0
FAN_TORQUE_TAU_S = 0.5
FAN_MAX_ON_S = 60.0
FAN_MAX_CONTROL_DT_S = 0.2

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)
SENSOR_AXES_DEG = [0.0, 90.0, 180.0, 270.0]  # +X, +Y, -X, -Y


# ---------------------------------------------------------------- sun_tracker.cpp mirror
def wrap180(angle_deg):
    a = math.fmod(angle_deg + 180.0, 360.0)
    if a < 0:
        a += 360.0
    return a - 180.0


def compute_sun(norm):
    """norm: calibrated brightness [+X, +Y, -X, -Y] (calibration already applied)."""
    norm = [min(n, 1.2) for n in norm]  # same cap as sun_tracker.cpp
    magnitude = sum(norm)
    sun_x = norm[0] - norm[2]
    sun_y = norm[1] - norm[3]
    direction = math.degrees(math.atan2(sun_y, sun_x))
    if direction < 0:
        direction += 360.0
    return direction, magnitude


def tracker_step(error_deg, body_rate_dps, dt_s, kp, kd, deadband_deg, w_cmd_rpm,
                 max_rate_dps=DEFAULT_MAX_RATE_DPS, external_torque_nm=0.0,
                 I_wheel=I_WHEEL):
    error_rad = math.radians(error_deg)
    rate_rad_s = math.radians(body_rate_dps)
    if abs(error_deg) < deadband_deg:
        error_rad = 0.0
    if kd > 0:
        max_rate = math.radians(max_rate_dps)
        rate_des = max(-max_rate, min(max_rate, (kp / kd) * error_rad))
        tau = kd * (rate_des - rate_rad_s)
    else:
        tau = kp * error_rad

    accel_rpm_s = ((external_torque_nm - tau) / I_wheel) * RPM_PER_RAD_S
    accel_rpm_s = max(-WHEEL_MAX_ACCEL_RPM_S, min(WHEEL_MAX_ACCEL_RPM_S, accel_rpm_s))
    w_cmd_rpm += accel_rpm_s * dt_s

    saturated = False
    if w_cmd_rpm > WHEEL_MAX_RPM:
        w_cmd_rpm, saturated = WHEEL_MAX_RPM, True
    if w_cmd_rpm < WHEEL_MIN_RPM:
        w_cmd_rpm, saturated = WHEEL_MIN_RPM, True
    return tau, w_cmd_rpm, saturated


def motor_lag_comp(w_cmd_rpm, w_prev_cmd_rpm, dt_s, motor_tau_s, max_rpm):
    w = w_cmd_rpm
    if dt_s > 0:
        w += motor_tau_s * (w_cmd_rpm - w_prev_cmd_rpm) / dt_s
    return max(-max_rpm, min(max_rpm, w))


# ---------------------------------------------------------------- fan_desaturation.cpp mirror
def fan_equivalent_torque_nm(wheel_accel_rpm_s, I_wheel):
    """Model-equivalent torque; inertia cancels in the wheel feedforward path."""
    return I_wheel * wheel_accel_rpm_s / 9.54929659


@dataclass
class FanDesaturationConfig:
    enabled: bool = FAN_DESATURATION_ENABLED
    target_rpm: float = FAN_TARGET_RPM
    start_rpm: float = FAN_START_RPM
    stop_rpm: float = FAN_STOP_RPM
    start_error_deg: float = FAN_START_ERROR_DEG
    stop_error_deg: float = FAN_STOP_ERROR_DEG
    max_body_rate_dps: float = FAN_MAX_BODY_RATE_DPS
    positive_torque_nm: float = (fan_equivalent_torque_nm(FAN_POSITIVE_EQUIV_RPM_S, I_WHEEL)
                                if FAN_CALIBRATION_IN_RPM_S else FAN_POSITIVE_TORQUE_NM)
    negative_torque_nm: float = (fan_equivalent_torque_nm(FAN_NEGATIVE_EQUIV_RPM_S, I_WHEEL)
                                if FAN_CALIBRATION_IN_RPM_S else FAN_NEGATIVE_TORQUE_NM)
    torque_time_constant_s: float = FAN_TORQUE_TAU_S
    max_on_s: float = FAN_MAX_ON_S
    max_dt_s: float = FAN_MAX_CONTROL_DT_S
    command_start_rpm: float = FAN_COMMAND_START_RPM
    unload_during_slews: bool = FAN_UNLOAD_DURING_SLEWS


@dataclass
class FanDesaturationState:
    bank: int = 0
    on_time_s: float = 0.0
    positive_torque_nm: float = 0.0
    negative_torque_nm: float = 0.0
    fault: bool = False


def fan_config_valid(c):
    return (math.isfinite(c.target_rpm) and math.isfinite(c.start_rpm) and
            math.isfinite(c.stop_rpm) and c.stop_rpm > 0 and c.start_rpm > c.stop_rpm and
            math.isfinite(c.start_error_deg) and c.start_error_deg >= 0 and
            math.isfinite(c.stop_error_deg) and c.stop_error_deg >= c.start_error_deg and
            math.isfinite(c.max_body_rate_dps) and c.max_body_rate_dps > 0 and
            math.isfinite(c.positive_torque_nm) and c.positive_torque_nm > 0 and
            math.isfinite(c.negative_torque_nm) and c.negative_torque_nm > 0 and
            math.isfinite(c.torque_time_constant_s) and c.torque_time_constant_s >= 0 and
            math.isfinite(c.max_on_s) and c.max_on_s > 0 and
            math.isfinite(c.max_dt_s) and c.max_dt_s > 0 and
            math.isfinite(c.command_start_rpm) and c.command_start_rpm > c.stop_rpm)


def fan_desaturation_step(wheel_rpm, error_deg, body_rate_dps, dt_s, permit, c, s,
                          wheel_command_rpm):
    """Return (bank command, estimated signed body torque, fault), matching C++."""
    if not c.enabled:
        s.bank, s.on_time_s, s.positive_torque_nm, s.negative_torque_nm, s.fault = 0, 0., 0., 0., False
        return 0, 0.0, False
    if (not fan_config_valid(c) or not math.isfinite(dt_s) or dt_s < 0 or dt_s > c.max_dt_s or
            (permit and (not math.isfinite(wheel_rpm) or not math.isfinite(error_deg) or
                         not math.isfinite(body_rate_dps) or dt_s == 0))):
        if fan_config_valid(c) and math.isfinite(dt_s) and dt_s > c.max_dt_s:
            # The previous output stayed energized during this known loop gap.
            alpha = -math.expm1(-dt_s / c.torque_time_constant_s) if c.torque_time_constant_s > 0 else 1.0
            positive_target = c.positive_torque_nm if s.bank > 0 else 0.0
            negative_target = c.negative_torque_nm if s.bank < 0 else 0.0
            s.positive_torque_nm += alpha * (positive_target - s.positive_torque_nm)
            s.negative_torque_nm += alpha * (negative_target - s.negative_torque_nm)
        s.bank, s.fault = 0, True
        # Shut off electrically but keep the current thrust estimate: spinning fans
        # still coast after a sensor/timing fault. Decay on later valid steps.
        return 0, s.positive_torque_nm - s.negative_torque_nm, True

    delta_rpm = wheel_rpm - c.target_rpm
    slew = c.unload_during_slews and abs(error_deg) > c.stop_error_deg
    pointing_ok = slew or (abs(error_deg) <= c.stop_error_deg and
                           abs(body_rate_dps) <= c.max_body_rate_dps)
    command_delta = wheel_command_rpm - c.target_rpm
    command_min_speed = 0.5 * (c.start_rpm + c.stop_rpm)
    positive_command_trigger = (math.isfinite(command_delta) and
                                command_delta >= c.command_start_rpm and delta_rpm >= command_min_speed)
    negative_command_trigger = (math.isfinite(command_delta) and
                                command_delta <= -c.command_start_rpm and delta_rpm <= -command_min_speed)
    if not permit or s.fault or not pointing_ok:
        s.bank = 0
    elif s.bank != 0:
        if ((s.bank < 0 and delta_rpm <= c.stop_rpm) or
                (s.bank > 0 and delta_rpm >= -c.stop_rpm)):
            s.bank = 0
    elif slew or abs(error_deg) <= c.start_error_deg:
        if delta_rpm >= c.start_rpm or positive_command_trigger:
            s.bank = -1
        elif delta_rpm <= -c.start_rpm or negative_command_trigger:
            s.bank = 1

    if s.bank == 0:
        s.on_time_s = 0.0
    else:
        s.on_time_s += dt_s
        if s.on_time_s >= c.max_on_s:
            s.bank, s.fault = 0, True

    alpha = -math.expm1(-dt_s / c.torque_time_constant_s) if c.torque_time_constant_s > 0 else 1.0
    positive_target = c.positive_torque_nm if s.bank > 0 else 0.0
    negative_target = c.negative_torque_nm if s.bank < 0 else 0.0
    s.positive_torque_nm += alpha * (positive_target - s.positive_torque_nm)
    s.negative_torque_nm += alpha * (negative_target - s.negative_torque_nm)
    return s.bank, s.positive_torque_nm - s.negative_torque_nm, s.fault


# ---------------------------------------------------------------- plant models
def phototransistor_response(angle_off_axis_deg, half_power_deg=50.0):
    """cos^p response with 50% output at the half-power angle (BPT-NPG3C1: 50 deg)."""
    p = math.log(0.5) / math.log(math.cos(math.radians(half_power_deg)))
    c = math.cos(math.radians(angle_off_axis_deg))
    return c ** p if c > 0 else 0.0


def sun_sensor(sun_body_deg, rng, noise, intensity=1.0):
    norm = []
    for axis in SENSOR_AXES_DEG:
        n = intensity * phototransistor_response(wrap180(sun_body_deg - axis))
        n += rng.normal(0.0, noise)
        norm.append(max(0.0, n))
    return norm


def simulate(kp=DEFAULT_KP, kd=DEFAULT_KD, bias_pwm=DEFAULT_BIAS_PWM,
             sun_start_deg=90.0, sun_rate_dps=0.0, duration_s=60.0,
             motor_tau_s=MOTOR_TAU_S, string_k=0.0, gyro_noise_dps=0.1, sun_noise=0.01,
             sun_off_at_s=None, seed=1, fan_config=None, disturbance_torque_nm=0.0,
             fan_torque_scale=1.0, fan_actual_tau_s=None):
    """
    sun_start_deg : sun azimuth in the body frame at release (0 = +X; array is at 180)
    sun_rate_dps  : light source moving around KestrelSAT (deg/s, inertial)
    motor_tau_s   : wheel speed time constant (estimate from Lab 7 Test A data)
    string_k      : string torsion stiffness (N*m/rad), 0 = ignore
    sun_off_at_s  : turn the light off at this time to test sun-lost behaviour
    fan_config   : optional explicitly enabled/calibrated bank configuration;
                   default has fans OFF and no assumed thrust
    disturbance_torque_nm : constant +Z environmental torque after release
    fan_torque_scale/fan_actual_tau_s : plant-only calibration error, for robustness checks
    """
    rng = np.random.default_rng(seed)
    dt = 0.001
    n_sub = int(round(DT_CONTROL / dt))

    theta = 0.0      # body angle, inertial (deg, +Z)
    omega_b = 0.0    # body rate (rad/s)
    w_wheel = 0.0    # wheel speed (RPM, +Z)
    w_cmd = 0.0
    tau = 0.0
    saturated = False
    sun_lost_since = None
    bias_rpm = bias_pwm * RPM_PER_PWM
    released = False
    w_prev_cmd = 0.0
    fans = FanDesaturationConfig() if fan_config is None else fan_config
    fan_state = FanDesaturationState()
    actual_positive_torque = actual_negative_torque = 0.0
    actual_fan_tau = fans.torque_time_constant_s if fan_actual_tau_s is None else fan_actual_tau_s

    log = {k: [] for k in ["t", "mode", "sun_dir", "err", "rate", "w_cmd", "w_meas", "tau", "sat",
                            "fan_bank", "fan_torque", "fan_fault"]}
    t = 0.0
    step = 0
    while t < duration_s:
        t_ms = t * 1000.0
        sun_inertial = sun_start_deg + sun_rate_dps * max(0.0, t - TRACK_CONTROL_MS / 1000.0)
        intensity = 0.0 if (sun_off_at_s is not None and t >= sun_off_at_s) else 1.0

        # ---- sense (body frame sun azimuth = inertial sun azimuth - body angle)
        norm = sun_sensor(sun_inertial - theta, rng, sun_noise, intensity)
        sun_dir, mag = compute_sun(norm)
        err = wrap180(sun_dir - SOLAR_ARRAY_AXIS_DEG)
        rate_meas = math.degrees(omega_b) + rng.normal(0.0, gyro_noise_dps)
        fan_bank, fan_torque, fan_fault = fan_desaturation_step(
            w_wheel, err, rate_meas, DT_CONTROL,
            t_ms >= TRACK_CONTROL_MS and mag >= SUN_LOST_THRESHOLD, fans, fan_state, w_cmd)

        # ---- decide & control (same structure as sun_track())
        saturated = False
        tau = 0.0
        if t_ms < TRACK_SPINUP_MS:
            mode = "SPINUP"
            w_cmd = bias_rpm * t_ms / TRACK_SPINUP_MS
        elif t_ms < TRACK_CONTROL_MS:
            mode = "HOLD"
            w_cmd = bias_rpm
        elif mag < SUN_LOST_THRESHOLD:
            mode = "NOSUN"
            if sun_lost_since is None:
                sun_lost_since = t_ms
            if t_ms - sun_lost_since > SUN_LOST_TIME_MS:
                mode = "STOPPED"
                w_cmd = 0.0
            elif fan_torque != 0:
                # No pointing information is available during this grace period.
                # Counter only the residual fan thrust, requesting zero body torque.
                tau, w_cmd, saturated = tracker_step(0, 0, DT_CONTROL, kp, kd,
                                                     POINTING_DEADBAND_DEG, w_cmd,
                                                     external_torque_nm=fan_torque)
        else:
            mode = "TRACK"
            sun_lost_since = None
            tau, w_cmd, saturated = tracker_step(err, rate_meas, DT_CONTROL, kp, kd,
                                                 POINTING_DEADBAND_DEG, w_cmd,
                                                 external_torque_nm=fan_torque)

        for key, val in zip(log.keys(), [t, mode, sun_dir, err, math.degrees(omega_b), w_cmd, w_wheel,
                                        tau, saturated, fan_bank, fan_torque, fan_fault]):
            log[key].append(val)
        if mode == "STOPPED":
            break

        # ---- motor lag compensation (same as sun_track())
        w_motor = motor_lag_comp(w_cmd, w_prev_cmd, DT_CONTROL, MOTOR_TAU_S, RPM_PER_PWM)
        # Same stall-zone handling as set_wheel_rpm() in the firmware.
        if 1.0 < abs(w_motor) < WHEEL_STALL_RPM:
            w_motor = math.copysign(WHEEL_STALL_RPM, w_motor)
        w_prev_cmd = w_cmd

        # ---- plant (1 ms steps until next control update)
        released = t_ms >= TRACK_RELEASE_MS
        for _ in range(n_sub):
            # Independent fan plant: thrust rises/decays between control samples.
            positive_target = fans.positive_torque_nm * fan_torque_scale if fan_bank > 0 else 0.0
            negative_target = fans.negative_torque_nm * fan_torque_scale if fan_bank < 0 else 0.0
            alpha = -math.expm1(-dt / actual_fan_tau) if actual_fan_tau > 0 else 1.0
            actual_positive_torque += alpha * (positive_target - actual_positive_torque)
            actual_negative_torque += alpha * (negative_target - actual_negative_torque)
            w_dot = (w_motor - w_wheel) / motor_tau_s          # RPM/s
            w_wheel += w_dot * dt
            torque_on_body = -I_WHEEL * (w_dot / RPM_PER_RAD_S) - string_k * math.radians(theta)
            torque_on_body += actual_positive_torque - actual_negative_torque + disturbance_torque_nm
            if released:
                omega_b += torque_on_body / I_BODY_Z * dt
                theta += math.degrees(omega_b) * dt
            # while held, a teammate absorbs the reaction torque
        t += DT_CONTROL
        step += 1

    return {k: np.array(v) for k, v in log.items()}


def settling_time(log, band_deg=10.0):
    """Time after control start at which |err| enters the band and stays there."""
    track = log["t"] >= TRACK_CONTROL_MS / 1000.0
    t, err = log["t"][track], np.abs(log["err"][track])
    outside = np.where(err > band_deg)[0]
    if len(outside) == 0:
        return 0.0
    if outside[-1] == len(err) - 1:
        return None  # never settled
    return t[outside[-1] + 1] - TRACK_CONTROL_MS / 1000.0


SCENARIOS = {
    "step90":   dict(sun_start_deg=90.0, description="R1: sun 90 deg from array"),
    "step170":  dict(sun_start_deg=10.0, description="Sun nearly behind the array (170 deg)"),
    "hold":     dict(sun_start_deg=175.0, duration_s=80.0, description="R2: hold on target 60 s"),
    "moving":   dict(sun_start_deg=150.0, sun_rate_dps=5.0, description="Stretch: light moving 5 deg/s"),
    "string":   dict(sun_start_deg=90.0, string_k=0.0005, description="R1 with string torsion"),
    "sunlost":  dict(sun_start_deg=120.0, sun_off_at_s=30.0, description="Light turned off at 30 s"),
}


def plot(log, title, path=None, show=False):
    import matplotlib.pyplot as plt  # plotting is optional for headless controller tests
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    t = log["t"]
    ax[0].plot(t, log["err"], label="pointing error")
    ax[0].axhspan(-10, 10, color="tab:green", alpha=0.15, label="±10° requirement")
    ax[0].set_ylabel("error (deg)")
    ax[1].plot(t, log["rate"], color="tab:orange")
    ax[1].set_ylabel("body rate (deg/s)")
    ax[2].plot(t, log["w_cmd"], label="commanded")
    ax[2].plot(t, log["w_meas"], label="actual", alpha=0.8)
    ax[2].axhline(WHEEL_MAX_RPM, ls="--", color="gray", lw=0.8)
    ax[2].axhline(WHEEL_MIN_RPM, ls="--", color="gray", lw=0.8)
    ax[2].set_ylabel("wheel speed (RPM)")
    ax[2].set_xlabel("time (s)")
    for a in ax:
        a.axvline(TRACK_RELEASE_MS / 1000, color="tab:blue", ls=":", lw=1)
        a.axvline(TRACK_CONTROL_MS / 1000, color="k", ls=":", lw=1)
        a.grid(alpha=0.3)
    ax[0].legend(loc="upper right")
    ax[2].legend(loc="upper right")
    fig.suptitle(title)
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--bias", type=float, default=DEFAULT_BIAS_PWM)
    parser.add_argument("--motor-tau", type=float, default=MOTOR_TAU_S, help="wheel time constant (s)")
    parser.add_argument("--scenario", choices=list(SCENARIOS), help="run one scenario (default: all)")
    parser.add_argument("--show", action="store_true", help="open plot windows")
    args = parser.parse_args()

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(out_dir, exist_ok=True)

    names = [args.scenario] if args.scenario else list(SCENARIOS)
    print(f"Kp={args.kp}  Kd={args.kd}  bias={args.bias}  motor_tau={args.motor_tau}s")
    for name in names:
        cfg = dict(SCENARIOS[name])
        desc = cfg.pop("description")
        log = simulate(kp=args.kp, kd=args.kd, bias_pwm=args.bias, motor_tau_s=args.motor_tau, **cfg)
        ts = settling_time(log)
        track = log["mode"] == "TRACK"
        within = np.mean(np.abs(log["err"][track]) <= 10.0) * 100 if track.any() else 0.0
        sat = np.mean(log["sat"][track]) * 100 if track.any() else 0.0
        ts_txt = "never" if ts is None else f"{ts:5.1f} s"
        print(f"{name:8s} | settle(±10°): {ts_txt:>8s} | time within ±10°: {within:5.1f}% | "
              f"wheel saturated: {sat:4.1f}% | final mode: {log['mode'][-1]:7s} | {desc}")
        plot(log, f"{name}: {desc}\nKp={args.kp}, Kd={args.kd}",
             os.path.join(out_dir, f"sim_{name}.png"), args.show)
    print(f"Plots saved to {out_dir}")


if __name__ == "__main__":
    main()
