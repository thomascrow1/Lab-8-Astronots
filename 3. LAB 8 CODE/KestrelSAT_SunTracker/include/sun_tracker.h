#pragma once

/* sun_tracker.h
* ===========================================================
* Description: Hardware-free math for the Lab 8 sun tracker.
*   - Sun sensor calibration and sun direction
*   - Pointing error
*   - PD controller that turns pointing error + body rate into a
*     reaction wheel speed command
* Nothing in this file touches pins, so the same math can be
* checked against the Python simulation (sim/sun_tracker_sim.py).
* =========================================================== */

// Channel order used everywhere: +X, +Y, -X, -Y
enum SunChannel { SUN_PX = 0, SUN_PY = 1, SUN_MX = 2, SUN_MY = 3 };

struct SunCalibration {
  float dark[4];    // raw ADC count with no light on the channel
  float bright[4];  // raw ADC count with the light pointed straight at the channel
};

struct SunReading {
  float norm[4];        // calibrated channel brightness (0 = dark, 1 = full light)
  float sun_X;          // +X minus -X brightness
  float sun_Y;          // +Y minus -Y brightness
  float magnitude;      // sum of calibrated channels (used to detect "no sun")
  float direction_deg;  // sun azimuth in body frame, 0-360 deg (0 = +X)
};

struct TrackerGains {
  float Kp;             // N*m per rad of pointing error
  float Kd;             // N*m per rad/s of body rate
  float deadband_deg;   // inside this error, Kp term is turned off
  float max_rate_dps;   // slew rate cap (deg/s): the controller never asks to turn faster than this
};

struct WheelLimits {
  float min_rpm;
  float max_rpm;
  float max_accel_rpm_s;
};

// Wrap an angle to the range [-180, 180) deg.
float wrap180(float angle_deg);

// Apply calibration to the 4 raw ADC readings and find the sun direction.
SunReading compute_sun(const float raw[4], const SunCalibration &cal);

// Pointing error (deg): positive when the sun is counter-clockwise (+Z) of the array.
float pointing_error_deg(float sun_direction_deg, float array_axis_deg);

// One PD controller step.
//   error_deg     : pointing error from pointing_error_deg()
//   body_rate_dps : body rotation rate about +Z (deg/s)
//   dt_s          : time since last step (s)
//   w_cmd_rpm     : current wheel speed command (RPM), updated in place
//   saturated     : set true if the wheel hit a speed limit this step
//   external_torque_nm : estimated fan body torque about +Z (default: no fans)
// Returns the desired body torque (N*m) for logging.
float tracker_step(float error_deg, float body_rate_dps, float dt_s,
                   const TrackerGains &gains, const WheelLimits &limits,
                   float I_wheel, float &w_cmd_rpm, bool &saturated,
                   float external_torque_nm = 0.0f);

// Wheel speed to send to the motor so the (laggy) wheel follows w_cmd_rpm.
//   max_rpm : wheel speed at 100% PWM (RPM_PER_PWM)
float motor_lag_comp(float w_cmd_rpm, float w_prev_cmd_rpm, float dt_s,
                     float motor_tau_s, float max_rpm);
