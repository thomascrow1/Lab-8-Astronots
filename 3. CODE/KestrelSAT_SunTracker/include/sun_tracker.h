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

struct FarErrorLatch {
  int dir;              // 0 = not latched, +1 / -1 = committed turn direction
  float below_s;        // time |error| has been back below the exit threshold (s)
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

// 180 deg direction latch. Near 180 deg the sign of the error is set by sensor noise, so PD
// pushes one way, then the other, and goes nowhere. Once |error| > enter_deg this commits to
// one turn direction and, if noise flips the sign, returns the error measured the long way
// round (e.g. -179 -> +181). Releases after |error| < exit_deg for exit_time_s.
// Start each test with latch = {0, 0}.
float latch_far_error(float error_deg, float dt_s, float enter_deg, float exit_deg,
                      float exit_time_s, FarErrorLatch &latch);

// One PD controller step.
//   error_deg     : pointing error from pointing_error_deg()
//   body_rate_dps : body rotation rate about +Z (deg/s)
//   dt_s          : time since last step (s)
//   w_cmd_rpm     : current wheel speed command (RPM), updated in place
//   saturated     : set true if the wheel hit a speed limit this step
// Returns the desired body torque (N*m) for logging.
float tracker_step(float error_deg, float body_rate_dps, float dt_s,
                   const TrackerGains &gains, const WheelLimits &limits,
                   float I_wheel, float &w_cmd_rpm, bool &saturated);

// Rate hold: same wheel math as tracker_step, but turns the body at a set rate (used for the scan).
// Returns the desired body torque (N*m) for logging.
float rate_step(float rate_des_dps, float body_rate_dps, float dt_s,
                const TrackerGains &gains, const WheelLimits &limits,
                float I_wheel, float &w_cmd_rpm, bool &saturated);

// ---- Sun acquisition scan ----
// KestrelSAT turns one full circle while recording gyro heading and -X brightness. The sun's
// heading is the centre of the brightest stretch. This works even though the -X sensor only
// reports "sun is somewhere in my cone": brightness still peaks when -X points straight at it.
const int SCAN_MAX_SAMPLES = 600;  // 30 s at 20 Hz

struct SunScan {
  float start_heading_deg;
  int n;
  float heading_deg[SCAN_MAX_SAMPLES];
  float bright[SCAN_MAX_SAMPLES];
};

// Start a new scan at the current heading.
void scan_start(SunScan &scan, float heading_deg);

// Add one sample. Returns true once the scan has turned turn_deg (or the buffer is full).
bool scan_add(SunScan &scan, float heading_deg, float brightness, float turn_deg);

// Find the sun: the brightness-weighted centre of the samples around the brightest one that
// are >= peak_fraction * the peak (the peak is searched for in the middle 360 deg of the scan).
// Returns false if the peak is below min_bright (no sun seen).
bool scan_result(const SunScan &scan, float peak_fraction, float min_bright,
                 float &sun_heading_deg, float &peak);

// ---- Sequential lobing (fine pointing while tracking) ----
// Aim +amp, then -amp, from the sun heading. Average -X brightness at the end of each side
// (only samples taken while settled). After each +/- pair, estimate how far the sun is from the
// sun heading and move the sun heading part of the way there.
// For a cos^p sensor: ln(b+) - ln(b-) = 2 * p * amp * offset (radians), p from the half-power angle.
struct LobeState {
  int side;          // +1 or -1: which side we are aiming at
  float t_side_s;    // time spent on this side
  float sum;         // brightness sum over the averaging window
  int n;             // samples in that sum
  float b_plus;      // average brightness on the + side (last cycle)
  float last_step_deg;  // most recent correction (for logging)
};

void lobe_reset(LobeState &lobe);

// One control step. brightness: uncapped -X brightness; settled: within LOBE_SETTLED_* of the aim point.
// Updates sun_heading_deg after each +/- pair. Returns the aim offset (deg) to add to the sun heading.
float lobe_step(LobeState &lobe, float brightness, bool settled, float dt_s,
                float amp_deg, float dwell_s, float avg_s, float gain, float max_step_deg,
                float half_power_deg, float &sun_heading_deg);

// Wheel speed to send to the motor so the (laggy) wheel follows w_cmd_rpm.
//   max_rpm : wheel speed at 100% PWM (RPM_PER_PWM)
float motor_lag_comp(float w_cmd_rpm, float w_prev_cmd_rpm, float dt_s,
                     float motor_tau_s, float max_rpm);
