/* sun_tracker.cpp
* ===========================================================
* Description: Hardware-free math for the Lab 8 sun tracker.
*              See include/sun_tracker.h for an overview.
* =========================================================== */

#include <math.h>
#include "sun_tracker.h"

static const float DEG_PER_RAD = 57.2957795f;
static const float RPM_PER_RAD_S = 9.54929659f; // 60 / (2*pi)

/**
 * @brief Wrap an angle to the range [-180, 180) deg.
 */
float wrap180(float angle_deg) {
  float a = fmodf(angle_deg + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

/**
 * @brief Calibrate the 4 sun sensor channels and compute sun direction.
 *
 * Each channel is scaled so 0 = dark and 1 = light pointed straight at it.
 * This removes differences between the four phototransistors (and room
 * light) that would otherwise bias the atan2() direction.
 * Direction uses the same method as Lab 6: atan2(+Y - -Y, +X - -X).
 */
SunReading compute_sun(const float raw[4], const SunCalibration &cal) {
  SunReading r;
  r.magnitude = 0.0f;
  for (int i = 0; i < 4; i++) {
    float span = cal.bright[i] - cal.dark[i];
    if (span < 1.0f) span = 1.0f;  // avoid divide by zero on a bad calibration
    float n = (raw[i] - cal.dark[i]) / span;
    if (n < 0.0f) n = 0.0f;        // darker than calibration = no light
    if (n > 1.2f) n = 1.2f;        // brighter than calibration: cap it so one badly calibrated channel can't dominate
    r.norm[i] = n;
    r.magnitude += n;
  }
  r.sun_X = r.norm[SUN_PX] - r.norm[SUN_MX];
  r.sun_Y = r.norm[SUN_PY] - r.norm[SUN_MY];
  r.direction_deg = atan2f(r.sun_Y, r.sun_X) * DEG_PER_RAD;
  if (r.direction_deg < 0) r.direction_deg += 360.0f;  // range 0-360 like Lab 6
  return r;
}

/**
 * @brief Pointing error between the sun and the solar array axis.
 *
 * Positive error means the sun is counter-clockwise (+Z) from the array,
 * so KestrelSAT must rotate +Z to fix it.
 */
float pointing_error_deg(float sun_direction_deg, float array_axis_deg) {
  return wrap180(sun_direction_deg - array_axis_deg);
}

void lobe_reset(LobeState &lobe) {
  lobe.side = 1;
  lobe.t_side_s = 0.0f;
  lobe.sum = 0.0f;
  lobe.n = 0;
  lobe.b_plus = 0.0f;
  lobe.last_step_deg = 0.0f;
}

/**
 * @brief Sequential lobing step (see sun_tracker.h).
 *
 * Example: aiming +4 deg reads 1.10, aiming -4 deg reads 0.90 -> the sun is on the + side,
 * so the sun heading moves a few degrees +.
 */
float lobe_step(LobeState &lobe, float brightness, bool settled, float dt_s,
                float amp_deg, float dwell_s, float avg_s, float gain, float max_step_deg,
                float half_power_deg, float &sun_heading_deg) {
  lobe.t_side_s += dt_s;
  if (lobe.t_side_s >= dwell_s - avg_s && settled) {
    lobe.sum += brightness;
    lobe.n++;
  }

  // Switch sides once the dwell is over and we have enough settled samples (otherwise keep waiting).
  if (lobe.t_side_s >= dwell_s && lobe.n >= 5) {
    float avg = lobe.sum / lobe.n;
    if (avg < 0.01f) avg = 0.01f;  // keep the log below finite
    if (lobe.side > 0) {
      lobe.b_plus = avg;
    } else {
      // Completed a +/- pair: estimate the sun's offset from the sun heading.
      float p = logf(0.5f) / logf(cosf(half_power_deg / DEG_PER_RAD));
      float amp_rad = amp_deg / DEG_PER_RAD;
      float offset_deg = (logf(lobe.b_plus) - logf(avg)) / (2.0f * p * amp_rad) * DEG_PER_RAD;
      float step = gain * offset_deg;
      if (step >  max_step_deg) step =  max_step_deg;
      if (step < -max_step_deg) step = -max_step_deg;
      sun_heading_deg += step;
      lobe.last_step_deg = step;
    }
    lobe.side = -lobe.side;
    lobe.t_side_s = 0.0f;
    lobe.sum = 0.0f;
    lobe.n = 0;
  }
  return lobe.side * amp_deg;
}

/**
 * @brief 180 deg direction latch (see sun_tracker.h).
 *
 * Example: latched +1, sensor noise makes the error read -179 -> returns +181, so the
 * controller keeps turning the same way instead of reversing.
 */
float latch_far_error(float error_deg, float dt_s, float enter_deg, float exit_deg,
                      float exit_time_s, FarErrorLatch &latch) {
  if (latch.dir == 0) {
    if (fabsf(error_deg) > enter_deg) {
      latch.dir = (error_deg >= 0.0f) ? 1 : -1;  // whichever way it reads first
      latch.below_s = 0.0f;
    }
    return error_deg;
  }

  if (fabsf(error_deg) < exit_deg) {
    // Getting close: use the normal error. Release only if it stays close (not a 1-sample spike).
    latch.below_s += dt_s;
    if (latch.below_s >= exit_time_s) latch.dir = 0;
    return error_deg;
  }

  latch.below_s = 0.0f;
  if (latch.dir > 0 && error_deg < 0.0f) return error_deg + 360.0f;
  if (latch.dir < 0 && error_deg > 0.0f) return error_deg - 360.0f;
  return error_deg;
}

/**
 * @brief Turn a desired body torque into a wheel speed command (shared by tracker_step and rate_step).
 */
static void wheel_from_torque(float tau, float dt_s, const WheelLimits &limits,
                              float I_wheel, float &w_cmd_rpm, bool &saturated) {
  float accel_rpm_s = (-tau / I_wheel) * RPM_PER_RAD_S;
  if (accel_rpm_s >  limits.max_accel_rpm_s) accel_rpm_s =  limits.max_accel_rpm_s;
  if (accel_rpm_s < -limits.max_accel_rpm_s) accel_rpm_s = -limits.max_accel_rpm_s;

  w_cmd_rpm += accel_rpm_s * dt_s;

  saturated = false;
  if (w_cmd_rpm > limits.max_rpm) { w_cmd_rpm = limits.max_rpm; saturated = true; }
  if (w_cmd_rpm < limits.min_rpm) { w_cmd_rpm = limits.min_rpm; saturated = true; }
}

/**
 * @brief One step of the PD sun-pointing controller.
 *
 * 1. Desired body torque:  tau = Kp * error - Kd * body_rate
 *    (A fixed sun moves clockwise in the body frame when KestrelSAT turns +Z,
 *     so error shrinks when body_rate has the same sign as error.)
 * 2. Reaction torque: the body feels -I_wheel * wheel_accel, so
 *    wheel_accel = -tau / I_wheel.
 * 3. Limit wheel acceleration, integrate to a wheel speed command, and keep the
 *    speed between min_rpm and max_rpm (the wheel may spin either way).
 */
float tracker_step(float error_deg, float body_rate_dps, float dt_s,
                   const TrackerGains &gains, const WheelLimits &limits,
                   float I_wheel, float &w_cmd_rpm, bool &saturated) {
  float error_rad = error_deg / DEG_PER_RAD;
  float rate_rad_s = body_rate_dps / DEG_PER_RAD;

  if (fabsf(error_deg) < gains.deadband_deg) error_rad = 0.0f;  // close enough: just damp

  // Same as PD (tau = Kp*error - Kd*rate), written as "turn at a desired rate":
  // desired rate = (Kp/Kd)*error, capped at max_rate so big slews are smooth and steady.
  float tau;
  if (gains.Kd > 0.0f) {
    float rate_des = (gains.Kp / gains.Kd) * error_rad;
    float max_rate = gains.max_rate_dps / DEG_PER_RAD;
    if (rate_des >  max_rate) rate_des =  max_rate;
    if (rate_des < -max_rate) rate_des = -max_rate;
    tau = gains.Kd * (rate_des - rate_rad_s);
  } else {
    tau = gains.Kp * error_rad;
  }

  wheel_from_torque(tau, dt_s, limits, I_wheel, w_cmd_rpm, saturated);
  return tau;
}

/**
 * @brief Turn the body at a set rate (deg/s) with the reaction wheel. Used for the sun scan.
 */
float rate_step(float rate_des_dps, float body_rate_dps, float dt_s,
                const TrackerGains &gains, const WheelLimits &limits,
                float I_wheel, float &w_cmd_rpm, bool &saturated) {
  float tau = gains.Kd * (rate_des_dps - body_rate_dps) / DEG_PER_RAD;
  wheel_from_torque(tau, dt_s, limits, I_wheel, w_cmd_rpm, saturated);
  return tau;
}

void scan_start(SunScan &scan, float heading_deg) {
  scan.start_heading_deg = heading_deg;
  scan.n = 0;
}

bool scan_add(SunScan &scan, float heading_deg, float brightness, float turn_deg) {
  if (scan.n < SCAN_MAX_SAMPLES) {
    scan.heading_deg[scan.n] = heading_deg;
    scan.bright[scan.n] = brightness;
    scan.n++;
  }
  return fabsf(heading_deg - scan.start_heading_deg) >= turn_deg || scan.n >= SCAN_MAX_SAMPLES;
}

bool scan_result(const SunScan &scan, float peak_fraction, float min_bright,
                 float &sun_heading_deg, float &peak) {
  if (scan.n == 0) return false;
  // Look for the peak only in the middle 360 deg of the scan, so the bright stretch around it
  // is never cut off by the start or end of the scan (that cost up to ~12 deg in the sim).
  float start = scan.start_heading_deg;
  float last = scan.heading_deg[scan.n - 1];
  float edge = (fabsf(last - start) - 360.0f) / 2.0f;
  if (edge < 0.0f) edge = 0.0f;  // scan timed out before a full turn: use everything
  int i_max = -1;
  for (int i = 0; i < scan.n; i++) {
    if (fabsf(scan.heading_deg[i] - start) < edge || fabsf(last - scan.heading_deg[i]) < edge) continue;
    if (i_max < 0 || scan.bright[i] > scan.bright[i_max]) i_max = i;
  }
  if (i_max < 0) return false;
  peak = scan.bright[i_max];
  if (peak < min_bright) return false;

  // Walk out from the peak while still bright, then take the brightness-weighted centre.
  float threshold = peak_fraction * peak;
  int i0 = i_max, i1 = i_max;
  while (i0 > 0 && scan.bright[i0 - 1] >= threshold) i0--;
  while (i1 < scan.n - 1 && scan.bright[i1 + 1] >= threshold) i1++;
  float sum_w = 0.0f, sum_wh = 0.0f;
  for (int i = i0; i <= i1; i++) {
    sum_w += scan.bright[i];
    sum_wh += scan.bright[i] * scan.heading_deg[i];
  }
  sun_heading_deg = sum_wh / sum_w;
  return true;
}

/**
 * @brief Motor lag compensation.
 *
 * The wheel only reaches a new speed about MOTOR_TAU_S after it is commanded, so the
 * controller always acts on a wheel that is behind. Commanding ahead by
 * motor_tau * (how fast the command is changing) makes the real wheel follow the
 * command almost without lag (the inverse of a first-order lag).
 */
float motor_lag_comp(float w_cmd_rpm, float w_prev_cmd_rpm, float dt_s,
                     float motor_tau_s, float max_rpm) {
  float w = w_cmd_rpm;
  if (dt_s > 0.0f) w += motor_tau_s * (w_cmd_rpm - w_prev_cmd_rpm) / dt_s;
  if (w > max_rpm)  w = max_rpm;
  if (w < -max_rpm) w = -max_rpm;
  return w;
}
