# Disturbance fan tests

| Run | Direction | Pointing held (s) | Wheel saturated (s) | Worst error before saturation (°) | Wheel slope (RPM/s) | Torque (mN·m) |
|---|---|---|---|---|---|---|
| SunTrack_data019 | CW 100% | 55.1 | 51.5 | 7.7 | -6.27 | -2.30 |
| SunTrack_data020 | CCW 100% | 52.7 | 49.5 | 3.7 | 6.86 | 2.51 |

Torques: + = counter-clockwise seen from above. The measured torque is fan + background (string twist, air).

- **Estimate A (CW and CCW runs only, assumes both pairs equally strong):** fan torque 2.41 mN·m per pair, background +0.11 mN·m.
- **Estimate B (subtract the fans-off background -0.27 mN·m):** CW 2.03 mN·m, CCW 2.78 mN·m.

Fans-off baselines (wheel drift while tracking):

- SunTrack_data016: -0.70 RPM/s = -0.26 mN·m
- SunTrack_data017: -0.94 RPM/s = -0.34 mN·m
- SunTrack_data018: -0.55 RPM/s = -0.20 mN·m

A and B disagree on the background's sign, so the fan torque per pair is somewhere in the range they span. Repeat runs in each direction would narrow it.

Torque = I_wheel × slope of measured wheel speed, from 2 s after the fans start until the wheel first saturates. Pointing error = gyro heading − sun heading (the sun-sensor error reads ~0 whenever only −X sees the light). Graphs made by `2. CODE/analysis/fan_hold_report.py`.
