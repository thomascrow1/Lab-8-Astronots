#pragma once

// Each bank switches TWO matched fans together. Bank +1 produces +Z body
// torque; bank -1 produces -Z body torque. RPM must use physical +Z sign.
// No Arduino dependencies: this supervisor can be exercised off the satellite.
struct FanDesaturationConfig {
  bool enabled;
  float target_rpm;
  float start_rpm;             // distance from target that starts unloading
  float stop_rpm;              // smaller distance that stops unloading
  float start_error_deg;       // start only after pointing has settled
  float stop_error_deg;        // stop if pointing leaves this band
  float max_body_rate_dps;
  float positive_torque_nm;    // +Z bank: measured or RPM/s-equivalent torque, magnitude
  float negative_torque_nm;    // -Z bank: measured or RPM/s-equivalent torque, magnitude
  float torque_time_constant_s;// measured thrust rise/decay time, first-order approximation
  float max_on_s;              // continuous bank runtime before a latched fault
  float max_dt_s;              // largest permitted control interval
  float command_start_rpm;     // anticipatory trigger if command nears a limit
  bool unload_during_slews;    // relax pointing/rate gates outside the stop band
};

struct FanDesaturationState {
  int bank = 0;
  float on_time_s = 0.0f;
  float positive_torque_nm = 0.0f;
  float negative_torque_nm = 0.0f;
  bool fault = false;          // reset only at the start of a new test
};

struct FanDesaturationOutput {
  int bank;                   // electrical command: -1, 0 (both off), +1
  float torque_nm;            // signed model torque, including fan coast-down
  bool fault;
};

bool fan_config_valid(const FanDesaturationConfig &config);

// Convert an empirical full-ON bank effect (RPM/s) for the existing controller.
// Use the SAME configured wheel inertia in tracker_step: its division by inertia
// cancels this multiplication. The result is model-equivalent, not measured torque.
float fan_equivalent_torque_nm(float wheel_accel_rpm_s, float I_wheel);

// permit is false during spinup/hold, sun loss, or invalid hardware feedback.
// Positive wheel momentum selects NEGATIVE fan torque. The pointing controller
// must also receive torque_nm so wheel deceleration cancels that body torque.
// A near-limit command can trigger below start_rpm, provided measured momentum
// has the same sign and is at least halfway between stop_rpm and start_rpm.
FanDesaturationOutput fan_desaturation_step(
    float wheel_rpm, float error_deg, float body_rate_dps, float dt_s,
    bool permit, const FanDesaturationConfig &config,
    FanDesaturationState &state, float wheel_command_rpm);
