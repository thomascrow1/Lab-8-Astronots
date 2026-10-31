"""
sun_tracker_sim.py
===========================================================
Single-axis (Z) simulation of the KestrelSAT sun tracker.

Models:
  - KestrelSAT body + reaction wheel (conservation of angular momentum)
  - Wheel motor as a first-order lag to the commanded speed (open-loop PWM)
  - 4 phototransistors with a 50 deg half-power angle (Lab 6)
  - Gyro and sun sensor noise, optional string torsion

The controller is a line-by-line copy of KestrelSAT_SunTracker/src/sun_tracker.cpp
and the timing/limits come from include/definitions.h. If you change one, change the other.

Usage:
  python sun_tracker_sim.py                    # run all scenarios, save plots to sim/output/
  python sun_tracker_sim.py --kp 0.005 --kd 0.012
  python sun_tracker_sim.py --scenario step90 --show
"""

import argparse
import math
import os

import matplotlib.pyplot as plt
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
LATCH_ENTER_DEG = 150.0
LATCH_EXIT_DEG = 120.0
LATCH_EXIT_TIME_MS = 300
SCAN_RATE_DPS = 45.0
SCAN_TURN_DEG = 420.0
SCAN_TIMEOUT_MS = 30000
SCAN_PEAK_FRACTION = 0.8
REACQ_FRACTION = 0.5
REACQ_TIME_MS = 1000
REACQ_ON_TARGET_DEG = 10.0
SCAN_MAX_SAMPLES = 600
TRACK_DURATION_MS = 120000
WHEEL_MIN_RPM = -400.0
WHEEL_MAX_RPM = 400.0
WHEEL_MAX_ACCEL_RPM_S = 400.0
MOTOR_TAU_S = 1.1
SUN_LOST_THRESHOLD = 0.15
SUN_LOST_TIME_MS = 500
TRACK_SPINUP_MS = 3000
TRACK_RELEASE_MS = 8000
TRACK_CONTROL_MS = 10000
DT_CONTROL = 0.050  # interval_testPoint (s)

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


def latch_far_error(error_deg, dt_s, latch, enter_deg=LATCH_ENTER_DEG, exit_deg=LATCH_EXIT_DEG,
                    exit_time_s=LATCH_EXIT_TIME_MS / 1000.0):
    """180 deg direction latch, same as sun_tracker.cpp. latch = {"dir": 0, "below_s": 0.0}."""
    if latch["dir"] == 0:
        if abs(error_deg) > enter_deg:
            latch["dir"] = 1 if error_deg >= 0 else -1
            latch["below_s"] = 0.0
        return error_deg
    if abs(error_deg) < exit_deg:
        latch["below_s"] += dt_s
        if latch["below_s"] >= exit_time_s:
            latch["dir"] = 0
        return error_deg
    latch["below_s"] = 0.0
    if latch["dir"] > 0 and error_deg < 0:
        return error_deg + 360.0
    if latch["dir"] < 0 and error_deg > 0:
        return error_deg - 360.0
    return error_deg


def tracker_step(error_deg, body_rate_dps, dt_s, kp, kd, deadband_deg, w_cmd_rpm,
                 max_rate_dps=DEFAULT_MAX_RATE_DPS):
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

    w_cmd_rpm, saturated = wheel_from_torque(tau, dt_s, w_cmd_rpm)
    return tau, w_cmd_rpm, saturated


def wheel_from_torque(tau, dt_s, w_cmd_rpm):
    accel_rpm_s = (-tau / I_WHEEL) * RPM_PER_RAD_S
    accel_rpm_s = max(-WHEEL_MAX_ACCEL_RPM_S, min(WHEEL_MAX_ACCEL_RPM_S, accel_rpm_s))
    w_cmd_rpm += accel_rpm_s * dt_s
    saturated = False
    if w_cmd_rpm > WHEEL_MAX_RPM:
        w_cmd_rpm, saturated = WHEEL_MAX_RPM, True
    if w_cmd_rpm < WHEEL_MIN_RPM:
        w_cmd_rpm, saturated = WHEEL_MIN_RPM, True
    return w_cmd_rpm, saturated


def rate_step(rate_des_dps, body_rate_dps, dt_s, kd, w_cmd_rpm):
    """Same as rate_step() in sun_tracker.cpp: turn the body at a set rate (scan)."""
    tau = kd * math.radians(rate_des_dps - body_rate_dps)
    w_cmd_rpm, saturated = wheel_from_torque(tau, dt_s, w_cmd_rpm)
    return tau, w_cmd_rpm, saturated


def scan_start(heading_deg):
    return {"start": heading_deg, "h": [], "b": []}


def scan_add(scan, heading_deg, brightness, turn_deg=SCAN_TURN_DEG):
    if len(scan["h"]) < SCAN_MAX_SAMPLES:
        scan["h"].append(heading_deg)
        scan["b"].append(brightness)
    return abs(heading_deg - scan["start"]) >= turn_deg or len(scan["h"]) >= SCAN_MAX_SAMPLES


def scan_result(scan, peak_fraction=SCAN_PEAK_FRACTION, min_bright=SUN_LOST_THRESHOLD):
    """Same as scan_result() in sun_tracker.cpp. Returns (found, sun_heading_deg, peak)."""
    b, h = scan["b"], scan["h"]
    if not b:
        return False, 0.0, 0.0
    edge = max(0.0, (abs(h[-1] - scan["start"]) - 360.0) / 2.0)  # search the middle 360 deg only
    idx = [i for i in range(len(b)) if abs(h[i] - scan["start"]) >= edge and abs(h[-1] - h[i]) >= edge]
    if not idx:
        return False, 0.0, 0.0
    i_max = max(idx, key=lambda i: (b[i], -i))  # first maximum, like the C++ loop
    peak = b[i_max]
    if peak < min_bright:
        return False, 0.0, peak
    thr = peak_fraction * peak
    i0 = i1 = i_max
    while i0 > 0 and b[i0 - 1] >= thr:
        i0 -= 1
    while i1 < len(b) - 1 and b[i1 + 1] >= thr:
        i1 += 1
    w = sum(b[i0:i1 + 1])
    return True, sum(bi * hi for bi, hi in zip(b[i0:i1 + 1], h[i0:i1 + 1])) / w, peak


def motor_lag_comp(w_cmd_rpm, w_prev_cmd_rpm, dt_s, motor_tau_s, max_rpm):
    w = w_cmd_rpm
    if dt_s > 0:
        w += motor_tau_s * (w_cmd_rpm - w_prev_cmd_rpm) / dt_s
    return max(-max_rpm, min(max_rpm, w))


# ---------------------------------------------------------------- plant models
def phototransistor_response(angle_off_axis_deg, half_power_deg=50.0):
    """cos^p response with 50% output at the half-power angle (BPT-NPG3C1: 50 deg)."""
    p = math.log(0.5) / math.log(math.cos(math.radians(half_power_deg)))
    c = math.cos(math.radians(angle_off_axis_deg))
    return c ** p if c > 0 else 0.0


def sun_sensor(sun_body_deg, rng, noise, intensity=1.0, half_power_deg=50.0, mx_spike_prob=0.0,
               adc_floor=0.0):
    """mx_spike_prob: chance per sample that -X reads a false bright value (motor noise seen in
    SunTrack_data008 once the wheel was running).
    adc_floor: calibrated readings below this read 0 (the ESP32 ADC reads 0 below ~150 mV), which
    leaves only one channel lit near a face (SunTrack_data011)."""
    norm = []
    for axis in SENSOR_AXES_DEG:
        n = intensity * phototransistor_response(wrap180(sun_body_deg - axis), half_power_deg)
        n += rng.normal(0.0, noise)
        norm.append(n if n >= adc_floor else 0.0)
    if mx_spike_prob > 0 and rng.random() < mx_spike_prob:
        norm[2] = rng.uniform(0.2, 1.2)
    return norm


def simulate(kp=DEFAULT_KP, kd=DEFAULT_KD, bias_pwm=DEFAULT_BIAS_PWM,
             sun_start_deg=90.0, sun_rate_dps=0.0, duration_s=60.0,
             motor_tau_s=MOTOR_TAU_S, string_k=0.0, gyro_noise_dps=0.1, sun_noise=0.01,
             sun_off_at_s=None, half_power_deg=50.0, mx_spike_prob=0.0, use_latch=True,
             adc_floor=0.0, gyro_bias_dps=0.0, use_scan=True, seed=1):
    """
    sun_start_deg : sun azimuth in the body frame at release (0 = +X; array is at 180)
    sun_rate_dps  : light source moving around KestrelSAT (deg/s, inertial)
    motor_tau_s   : wheel speed time constant (estimate from Lab 7 Test A data)
    string_k      : string torsion stiffness (N*m/rad), 0 = ignore
    sun_off_at_s  : turn the light off at this time to test sun-lost behaviour
    half_power_deg: phototransistor half-power angle (~33 deg measured in SunTrack_data008)
    mx_spike_prob : chance per sample of a false -X reading while the wheel runs
    adc_floor     : calibrated readings below this read 0 (ESP32 ADC dead zone)
    gyro_bias_dps : constant gyro bias (removed by the bias measured while held)
    use_scan      : True = scan-and-hold (current firmware), False = old sun-sensor PD
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
    latch = {"dir": 0, "below_s": 0.0}
    bias_sum, bias_n, gyro_bias, heading = 0.0, 0, 0.0, 0.0
    control_started, acquiring = False, False
    sun_heading, sun_peak, scan, scan_t0, dim_since = 0.0, 0.0, None, 0.0, None

    log = {k: [] for k in ["t", "mode", "sun_dir", "err", "rate", "w_cmd", "w_meas", "tau", "sat", "true_err"]}
    t = 0.0
    step = 0
    while t < duration_s:
        t_ms = t * 1000.0
        sun_inertial = sun_start_deg + sun_rate_dps * max(0.0, t - TRACK_CONTROL_MS / 1000.0)
        intensity = 0.0 if (sun_off_at_s is not None and t >= sun_off_at_s) else 1.0

        # ---- sense (body frame sun azimuth = inertial sun azimuth - body angle)
        spikes = mx_spike_prob if t_ms >= TRACK_CONTROL_MS else 0.0
        norm = sun_sensor(sun_inertial - theta, rng, sun_noise, intensity, half_power_deg, spikes, adc_floor)
        sun_dir, mag = compute_sun(norm)
        err = wrap180(sun_dir - SOLAR_ARRAY_AXIS_DEG)
        true_err = wrap180(sun_inertial - theta - SOLAR_ARRAY_AXIS_DEG)
        rate_meas = math.degrees(omega_b) + gyro_bias_dps + rng.normal(0.0, gyro_noise_dps)
        if control_started:
            heading += (rate_meas - gyro_bias) * DT_CONTROL

        # ---- decide & control (same structure as sun_track())
        saturated = False
        tau = 0.0
        if t_ms < TRACK_SPINUP_MS:
            mode = "SPINUP"
            w_cmd = bias_rpm * t_ms / TRACK_SPINUP_MS
        elif t_ms < TRACK_CONTROL_MS:
            mode = "HOLD"
            w_cmd = bias_rpm
            if TRACK_SPINUP_MS <= t_ms < TRACK_RELEASE_MS:
                bias_sum += rate_meas
                bias_n += 1
        elif use_scan:
            # ---- scan-and-hold (same as sun_track() in main.cpp)
            if not control_started:
                control_started = True
                gyro_bias = bias_sum / bias_n if bias_n else 0.0
                heading = 0.0
                acquiring, scan, scan_t0 = True, scan_start(0.0), t_ms
            if acquiring:
                mode = "ACQUIRE"
                tau, w_cmd, saturated = rate_step(SCAN_RATE_DPS, rate_meas, DT_CONTROL, kd, w_cmd)
                done = scan_add(scan, heading, norm[2])
                if done or t_ms - scan_t0 > SCAN_TIMEOUT_MS:
                    found, sun_heading, sun_peak = scan_result(scan)
                    if not found:
                        mode, w_cmd = "STOPPED", 0.0
                    else:
                        acquiring, dim_since = False, None
                        latch = {"dir": 0, "below_s": 0.0}
            else:
                mode = "TRACK"
                err_gyro = wrap180(sun_heading - heading)
                err_ctrl = latch_far_error(err_gyro, DT_CONTROL, latch) if use_latch else err_gyro
                tau, w_cmd, saturated = tracker_step(err_ctrl, rate_meas, DT_CONTROL, kp, kd,
                                                     POINTING_DEADBAND_DEG, w_cmd)
                if abs(err_gyro) < REACQ_ON_TARGET_DEG and norm[2] < REACQ_FRACTION * sun_peak:
                    dim_since = t_ms if dim_since is None else dim_since
                    if t_ms - dim_since > REACQ_TIME_MS:
                        acquiring, scan, scan_t0 = True, scan_start(heading), t_ms
                else:
                    dim_since = None
        elif mag < SUN_LOST_THRESHOLD:
            mode = "NOSUN"
            if sun_lost_since is None:
                sun_lost_since = t_ms
            if t_ms - sun_lost_since > SUN_LOST_TIME_MS:
                mode = "STOPPED"
                w_cmd = 0.0
        else:
            mode = "TRACK"
            sun_lost_since = None
            err_ctrl = latch_far_error(err, DT_CONTROL, latch) if use_latch else err
            tau, w_cmd, saturated = tracker_step(err_ctrl, rate_meas, DT_CONTROL, kp, kd,
                                                 POINTING_DEADBAND_DEG, w_cmd)

        for key, val in zip(log.keys(), [t, mode, sun_dir, err, math.degrees(omega_b), w_cmd, w_wheel, tau,
                                         saturated, true_err]):
            log[key].append(val)
        if mode == "STOPPED":
            break

        # ---- motor lag compensation (same as sun_track())
        w_motor = motor_lag_comp(w_cmd, w_prev_cmd, DT_CONTROL, MOTOR_TAU_S, RPM_PER_PWM)
        w_prev_cmd = w_cmd

        # ---- plant (1 ms steps until next control update)
        released = t_ms >= TRACK_RELEASE_MS
        for _ in range(n_sub):
            w_dot = (w_motor - w_wheel) / motor_tau_s          # RPM/s
            w_wheel += w_dot * dt
            torque_on_body = -I_WHEEL * (w_dot / RPM_PER_RAD_S) - string_k * math.radians(theta)
            if released:
                omega_b += torque_on_body / I_BODY_Z * dt
                theta += math.degrees(omega_b) * dt
            # while held, a teammate absorbs the reaction torque
        t += DT_CONTROL
        step += 1

    return {k: np.array(v) for k, v in log.items()}


def settling_time(log, band_deg=10.0):
    """Time after control start at which the TRUE pointing error enters the band and stays there."""
    track = log["t"] >= TRACK_CONTROL_MS / 1000.0
    t, err = log["t"][track], np.abs(log["true_err"][track])
    outside = np.where(err > band_deg)[0]
    if len(outside) == 0:
        return 0.0
    if outside[-1] == len(err) - 1:
        return None  # never settled
    return t[outside[-1] + 1] - TRACK_CONTROL_MS / 1000.0


SCENARIOS = {
    "step90":   dict(sun_start_deg=90.0, description="R1: sun 90 deg from array"),
    "step170":  dict(sun_start_deg=10.0, description="Sun nearly behind the array (170 deg)"),
    "step180":  dict(sun_start_deg=0.0, description="Sun exactly behind the array (180 deg)"),
    "step180hw": dict(sun_start_deg=0.0, half_power_deg=33.0, sun_noise=0.02, mx_spike_prob=0.06,
                      description="180 deg + narrow sensors + -X motor noise (like SunTrack_data008)"),
    "hwoffset": dict(sun_start_deg=150.0, string_k=0.0005, half_power_deg=33.0, adc_floor=0.08,
                     gyro_bias_dps=0.05, description="Only -X lit near target + string twist (SunTrack_data011)"),
    "hold":     dict(sun_start_deg=175.0, duration_s=80.0, description="R2: hold on target 60 s"),
    "moving":   dict(sun_start_deg=150.0, sun_rate_dps=5.0, description="Stretch: light moving 5 deg/s"),
    "string":   dict(sun_start_deg=90.0, string_k=0.0005, description="R1 with string torsion"),
    "sunlost":  dict(sun_start_deg=120.0, sun_off_at_s=30.0, description="Light turned off at 30 s"),
}


def plot(log, title, path=None, show=False):
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    t = log["t"]
    ax[0].plot(t, log["err"], label="sun sensor error", alpha=0.6)
    ax[0].plot(t, log["true_err"], label="true error", color="k", lw=1)
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
        within = np.mean(np.abs(log["true_err"][track]) <= 10.0) * 100 if track.any() else 0.0
        sat = np.mean(log["sat"][track]) * 100 if track.any() else 0.0
        ts_txt = "never" if ts is None else f"{ts:5.1f} s"
        print(f"{name:8s} | settle(±10°): {ts_txt:>8s} | time within ±10°: {within:5.1f}% | "
              f"wheel saturated: {sat:4.1f}% | final mode: {log['mode'][-1]:7s} | {desc}")
        plot(log, f"{name}: {desc}\nKp={args.kp}, Kd={args.kd}",
             os.path.join(out_dir, f"sim_{name}.png"), args.show)
    print(f"Plots saved to {out_dir}")


if __name__ == "__main__":
    main()
