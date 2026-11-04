# KestrelSAT Capstone: Sun Tracker (Astro 331 Lab 8)

## About us
- Cadet team, newer to Arduino and C++. Explain each change in plain terms before coding.
- Prefer simple code over clever code. Match the style of the existing Lab 6/7 code.

## Project
- Point the solar array (body −X, 180°) at a light using the Lab 6 sun sensor, the ICM-20948 gyro, and the Lab 7 reaction wheel.
- Folders: `2. THEORY/` Python analysis + sim, `3. CODE/` firmware + GCS, `4. DATA/` SD card data and graphs. This file, `README.md`, `AI_LOG.md` and `requirements.txt` are at the project root.
- Firmware: `3. CODE/KestrelSAT_SunTracker/` (PlatformIO env `esp32thing_plus`).
- Controller math: `src/sun_tracker.cpp`. It is mirrored in `2. THEORY/sim/sun_tracker_sim.py`, so **change both together**.
- Settings: `include/definitions.h`. The constants at the top of the sim must match it.

## Hardware: SparkFun Thing Plus ESP32
- XBee: HardwareSerial(2), 9600 baud. ICM-20948 IMU and MAX17048 gauge on I2C.
- Sun sensor: A0 = +X, A1 = +Y, A2 = −X, A3 = −Y (3.3 V, 3.3 kΩ).
- TB9051FTG motor driver PWM1 = 33, PWM2 = 15. Encoder A = 14, B = 32. Motor supply 5 V.
- Disturbance fans (4× LD3007MS on the 5 V rail, switched by XY-MOS modules, GPIO straight to TRIG): wired in diagonal pairs. GPIO 27 (`cw` in command 8) = clockwise from above, GPIO 4 (`ccw`) = counter-clockwise (LEDC channels 0 and 2). In command 11 one pair is the disturbance and the other acts as "thrusters". They share the 5 V rail with the wheel motor, so watch for rail sag.
- Pin map: `include/definitions.h`.

## Conventions
- Commands are dispatched in `process_main_menu()`, numbered 0–8 in order (renumbered 2 Nov 2026; old numbers are listed in README.md), then 10 (requirement demo; 9 is unused) and 11 (momentum dump). Add new commands at the end.
- Telemetry format is `Name:value,Name:value`. Mirror every XBee message to Serial (`print_both()`).
- Sun direction: 0° = +X, counter-clockwise (+Z) positive. Error = wrap180(sun_dir − array_axis).
- Wheel speed is positive when spinning about +Z. Convert to PWM with `wheel_pwm_sign`.
- Every SD log has a header row. Flush every 10 points.

## Rules
- Never change pins without asking.
- Add libraries only via `lib_deps`.
- No blocking waits in `loop()`. Test loops must exit on 'X' and stop the wheel.
- Any code path that ends a test must call `driver.setOutput(0)`.
- You can't test on hardware: list what we must verify on KestrelSAT.
- Log each session in `AI_LOG.md`.
