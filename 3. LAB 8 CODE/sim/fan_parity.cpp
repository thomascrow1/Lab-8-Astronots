// Optional host executable used by test_fan_desaturation.py. Synthetic torque.
#include <iomanip>
#include <iostream>
#include "fan_desaturation.h"
#include "sun_tracker.h"

int main() {
  // Exercise the RPM/s calibration helper while retaining the synthetic 2mN*m
  // bank strength used by the Python parity case.
  const float bank_torque = fan_equivalent_torque_nm(0.002f * 9.54929659f / 0.0035f,
                                                     0.0035f);
  FanDesaturationConfig config = {true, 0, 200, 100, 5, 10, 3,
                                  bank_torque, bank_torque, 0.5f, 60, 0.2f, 300, true};
  FanDesaturationState state;
  TrackerGains gains = {0.126f, 0.1092f, 3, 90};
  WheelLimits limits = {-400, 400, 400};
  float rpm, error, rate, dt, wheel = 200;
  int permit;
  std::cout << std::setprecision(9);
  while (std::cin >> rpm >> error >> rate >> dt >> permit) {
    FanDesaturationOutput fans = fan_desaturation_step(rpm, error, rate, dt,
                                                      permit != 0, config, state, wheel);
    bool saturated;
    float tau = tracker_step(error, rate, dt, gains, limits, 0.0035f, wheel,
                              saturated, fans.torque_nm);
    std::cout << fans.bank << ' ' << fans.torque_nm << ' ' << fans.fault << ' '
              << tau << ' ' << wheel << ' ' << saturated << '\n';
  }
}
