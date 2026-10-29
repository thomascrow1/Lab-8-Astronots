#include <math.h>
#include "fan_desaturation.h"

float fan_equivalent_torque_nm(float wheel_accel_rpm_s, float I_wheel) {
  return I_wheel * wheel_accel_rpm_s / 9.54929659f;
}

bool fan_config_valid(const FanDesaturationConfig &c) {
  return isfinite(c.target_rpm) && isfinite(c.start_rpm) &&
      isfinite(c.stop_rpm) && c.stop_rpm > 0 && c.start_rpm > c.stop_rpm &&
      isfinite(c.start_error_deg) && c.start_error_deg >= 0 &&
      isfinite(c.stop_error_deg) && c.stop_error_deg >= c.start_error_deg &&
      isfinite(c.max_body_rate_dps) && c.max_body_rate_dps > 0 &&
      isfinite(c.positive_torque_nm) && c.positive_torque_nm > 0 &&
      isfinite(c.negative_torque_nm) && c.negative_torque_nm > 0 &&
      isfinite(c.torque_time_constant_s) && c.torque_time_constant_s >= 0 &&
      isfinite(c.max_on_s) && c.max_on_s > 0 &&
      isfinite(c.max_dt_s) && c.max_dt_s > 0 &&
      isfinite(c.command_start_rpm) && c.command_start_rpm > c.stop_rpm;
}

FanDesaturationOutput fan_desaturation_step(
    float wheel_rpm, float error_deg, float body_rate_dps, float dt_s,
    bool permit, const FanDesaturationConfig &c, FanDesaturationState &s,
    float wheel_command_rpm) {
  if (!c.enabled) {
    s = FanDesaturationState{};
    return {0, 0.0f, false};
  }
  if (!fan_config_valid(c) || !isfinite(dt_s) || dt_s < 0 || dt_s > c.max_dt_s ||
      (permit && (!isfinite(wheel_rpm) || !isfinite(error_deg) ||
                  !isfinite(body_rate_dps) || dt_s == 0))) {
    if (fan_config_valid(c) && isfinite(dt_s) && dt_s > c.max_dt_s) {
      // The PREVIOUS electrical command remained active throughout the delay.
      // Account for thrust that built up (or decayed) before we can switch off.
      float alpha = (c.torque_time_constant_s > 0)
          ? -expm1f(-dt_s / c.torque_time_constant_s) : 1.0f;
      float positive_target = s.bank > 0 ? c.positive_torque_nm : 0.0f;
      float negative_target = s.bank < 0 ? c.negative_torque_nm : 0.0f;
      s.positive_torque_nm += alpha * (positive_target - s.positive_torque_nm);
      s.negative_torque_nm += alpha * (negative_target - s.negative_torque_nm);
    }
    s.bank = 0;
    s.fault = true;
    // The fans were only just switched off. A timing/feedback fault does not
    // remove their existing thrust; keep the current estimate, then decay it on
    // subsequent valid steps while the electrical fault stays latched.
    return {0, s.positive_torque_nm - s.negative_torque_nm, true};
  }

  float delta_rpm = wheel_rpm - c.target_rpm;
  bool slew = c.unload_during_slews && fabsf(error_deg) > c.stop_error_deg;
  bool pointing_ok = slew || (fabsf(error_deg) <= c.stop_error_deg &&
                             fabsf(body_rate_dps) <= c.max_body_rate_dps);
  float command_delta = wheel_command_rpm - c.target_rpm;
  float command_min_speed = 0.5f * (c.start_rpm + c.stop_rpm);
  // Command saturation can occur below the nominal speed. Anticipate it only
  // when real, same-direction wheel momentum exists outside the stop band.
  bool positive_command_trigger = isfinite(command_delta) &&
      command_delta >= c.command_start_rpm && delta_rpm >= command_min_speed;
  bool negative_command_trigger = isfinite(command_delta) &&
      command_delta <= -c.command_start_rpm && delta_rpm <= -command_min_speed;
  if (!permit || s.fault || !pointing_ok) {
    s.bank = 0;
  } else if (s.bank != 0) {
    // Also stop if the wheel crosses the target. Do not reverse banks here.
    if ((s.bank < 0 && delta_rpm <= c.stop_rpm) ||
        (s.bank > 0 && delta_rpm >= -c.stop_rpm)) s.bank = 0;
  } else if (slew || fabsf(error_deg) <= c.start_error_deg) {
    if (delta_rpm >= c.start_rpm || positive_command_trigger) s.bank = -1;
    else if (delta_rpm <= -c.start_rpm || negative_command_trigger) s.bank = 1;
  }

  if (s.bank == 0) s.on_time_s = 0.0f;
  else {
    s.on_time_s += dt_s;
    if (s.on_time_s >= c.max_on_s) {
      s.bank = 0;
      s.fault = true;
    }
  }

  // Electrical OFF does not mean thrust vanishes instantly. Continue estimating
  // the residual torque so the wheel can counter it while the fans coast down.
  float alpha = (c.torque_time_constant_s > 0)
      ? -expm1f(-dt_s / c.torque_time_constant_s) : 1.0f;
  float positive_target = (s.bank > 0) ? c.positive_torque_nm : 0.0f;
  float negative_target = (s.bank < 0) ? c.negative_torque_nm : 0.0f;
  s.positive_torque_nm += alpha * (positive_target - s.positive_torque_nm);
  s.negative_torque_nm += alpha * (negative_target - s.negative_torque_nm);
  return {s.bank, s.positive_torque_nm - s.negative_torque_nm, s.fault};
}
