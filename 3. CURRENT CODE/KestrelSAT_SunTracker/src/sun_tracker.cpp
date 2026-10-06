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

/**
 * @brief One step of the PD sun-pointing controller.
 *
 * 1. Desired body torque:  tau = Kp * error - Kd * body_rate
 *    (A fixed sun moves clockwise in the body frame when KestrelSAT turns +Z,
 *     so error shrinks when body_rate has the same sign as error.)
 * 2. Reaction torque: the body feels -I_wheel * wheel_accel, so
 *    wheel_accel = -tau / I_wheel.
 * 3. Limit wheel acceleration, integrate to a wheel speed command, and keep the
 *    speed between min_rpm and max_rpm (the wheel never crosses 0 RPM).
 */
float tracker_step(float error_deg, float body_rate_dps, float dt_s,
                   const TrackerGains &gains, const WheelLimits &limits,
                   float I_wheel, float &w_cmd_rpm, bool &saturated) {
  float error_rad = error_deg / DEG_PER_RAD;
  float rate_rad_s = body_rate_dps / DEG_PER_RAD;

  float p_term = gains.Kp * error_rad;
  if (fabsf(error_deg) < gains.deadband_deg) p_term = 0.0f;  // close enough: just damp
  float tau = p_term - gains.Kd * rate_rad_s;

  float accel_rpm_s = (-tau / I_wheel) * RPM_PER_RAD_S;
  if (accel_rpm_s >  limits.max_accel_rpm_s) accel_rpm_s =  limits.max_accel_rpm_s;
  if (accel_rpm_s < -limits.max_accel_rpm_s) accel_rpm_s = -limits.max_accel_rpm_s;

  w_cmd_rpm += accel_rpm_s * dt_s;

  saturated = false;
  if (w_cmd_rpm > limits.max_rpm) { w_cmd_rpm = limits.max_rpm; saturated = true; }
  if (w_cmd_rpm < limits.min_rpm) { w_cmd_rpm = limits.min_rpm; saturated = true; }

  return tau;
}
