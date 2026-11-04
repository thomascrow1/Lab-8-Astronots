# KestrelSAT_SunTracker: what every function does

This is a guide to the firmware in `KestrelSAT_SunTracker/`. It covers every function, what it does,
and which other functions use it. Settings (pins, gains, timings) live in `include/definitions.h`, so
this guide names the setting instead of repeating its value.

## How the files fit together

| File | Job |
|---|---|
| `src/main.cpp` | Everything that touches hardware: startup, the command menu, sensors, the wheel, the fans, SD logging, and each test. |
| `src/sun_tracker.cpp` + `include/sun_tracker.h` | The controller math only (no pins). The Python simulation `2. THEORY/sim/sun_tracker_sim.py` copies this math line by line, so **change both together**. |
| `src/sd_funcitons.cpp` + `include/sd_functions.h` | Start the SD card and make numbered data files. |
| `include/definitions.h` | Pins and every tunable number. |

Sign conventions used everywhere: sun direction 0° = +X face, angles grow counter-clockwise seen from
above (+Z). The solar array is the −X face (180°). Sun sensor channels are always in the order
**+X, +Y, −X, −Y**.

## Ground station commands

| Cmd | Function it runs | What it's for |
|---|---|---|
| 0 | (inline) `driver.setOutput(0)` + `set_fans(0, 0)` | Emergency stop: wheel and fans off. |
| 1 | (inline) | Print the menu. |
| 2 | `send_battery_telemetry()` | Battery voltage, charge %, charge rate. |
| 3 | `sun_calibrate()` | Calibrate the sun sensor. |
| 4 | `sun_track(0, 0)` | Sun tracker, wheel only (requirements 1 and 2). |
| 5 | `sun_set_params()` | Change gains and settings without reflashing. |
| 6 | `ad_check()` | Guided sun sensor + gyro check. |
| 7 | `pt_sweep()` | Phototransistor resistor sweep. |
| 8 | `fan_hold_test()` | Sun tracker + disturbance fans (requirement 3). |
| 10 | `req_demo()` | Final presentation demo: shows R1/R2 (one run), R3 or S2 with a live PASS/FAIL. |
| 11 | `momentum_dump_test()` | Sun tracker + disturbance fans + "thruster" fans that unload the wheel (stretch S2). |

---

## `main.cpp`: startup and the command loop

### `setup()`
Runs once at power-up. In order: turns the fans **off first** (so they never spin while waiting),
starts USB Serial and waits for the Serial monitor, starts I2C, the RGB LED (red), the XBee, the SD
card (`sd_init`), the battery gauge, the IMU (stops here forever if the IMU is missing), sets the ADC
to 12 bits for the sun sensor, enables the motor driver at 0 and zeroes the encoder. LED turns green
when done.

### `loop()`
Runs forever after `setup()`. Every 10 ms it calls `process_main_menu()` to check for a command.
Every 3 s it sends a heartbeat (`*` on XBee, a reminder on USB) and sets the LED green.

### `get_command_from_ground_station()`
Reads one line from the XBee (checked first) or USB, trims spaces and returns it as a number.
Returns 0 if nothing is waiting and −1 for an empty line. Prints where the command came from.
Used by: `process_main_menu()`.

### `process_main_menu()`
If a command is waiting, reads it and runs the matching function from the table above. Commands 0
and 1 are handled right here. Anything else prints a caution and is ignored.

### `send_battery_telemetry()`
Reads the MAX17048 gauge and sends one line to both USB and XBee:
`BAT,V=3.912V, SOC=87.5%, CR=-4.160%/hr` (charge rate: positive = charging). Used for the
power-positive stretch requirement.

---

## `main.cpp`: helpers used by the tests

### `print_both(msg)`
Prints a line to USB Serial **and** the XBee. Almost every message in the firmware goes through this.

### `read_sun_raw(raw[4])`
Reads all four phototransistors. The lamp flickers at 120 Hz, so it takes `SUN_READS_PER_SAMPLE`
reads per channel spread evenly over one flicker period (`SUN_READ_SPAN_US`, about 8.3 ms) and
averages them. Fills `raw` with ADC counts (0–4095) in the order +X, +Y, −X, −Y.
Used by: every test that reads the sun sensor.

### `read_body_rate_dps()`
Reads the IMU and returns the body rotation rate about Z in deg/s, multiplied by `gyro_sign` so +
means counter-clockwise from above. Also stores the raw gyro reading and the magnetometer X/Y in
globals (`gyro_Z`, `mag_X`, `mag_Y`) for logging.

### `measure_wheel_rpm()`
Returns the reaction wheel speed in RPM from how many encoder counts went by since the last call
(`CPR` × 10 counts per wheel turn). Remembers the last count and time between calls, so call it once
at the start of a test to reset it.

### `set_wheel_rpm(w_rpm)`
Turns a wheel speed command (RPM) into a motor PWM (−1 to +1) and sends it to the driver. The wheel
stalls at low speed, so any command between 1 RPM and `WHEEL_STALL_RPM` is bumped up to
`WHEEL_STALL_RPM`. Uses `wheel_pwm_sign` and `RPM_PER_PWM` to convert.

### `wait_for_line()`
Waits (blocking) until the user sends a line from USB or XBee, returns it trimmed, and clears any
extra characters. Used by every prompt ("send any key", "kp 0.004", "cw 60"...).

### `abort_requested()`
Checks, without waiting, whether the user sent `X`. The test loops call it every pass so `X` stops a
test immediately.

### `sample_sun_stats(n, mean[4], sd[4])`
Takes `n` sun sensor readings and returns each channel's average and standard deviation (noise).
Used by: `ad_check()` and `pt_sweep()`.

### `set_fans(upper_pct, lower_pct)`
Sets the two disturbance fan pairs to a duty from 0 to 100% right away (negative = 0). "upper" is the
GPIO 27 pair (twists clockwise from above), "lower" is the GPIO 4 pair (counter-clockwise). Stores
the current duties in `fan_upper_pct` / `fan_lower_pct` so they get logged. Flips the signal if
`FAN_PWM_INVERTED` is set (only if an inverting transistor is ever added).

---

## `main.cpp`: the commands

### `sun_calibrate()` (command 3)
Teaches the firmware what "dark" and "full light" look like for each phototransistor:
1. Light off → averages 100 readings as the dark count for all four channels.
2. Light pointed straight at +X, then +Y, −X, −Y → the bright count for that channel.

If any channel changed by less than 300 counts, it refuses to apply the new calibration (a tiny
range would wreck the sun direction) and keeps the old one. Otherwise it applies it and prints a
`SunCalibration sun_cal = ...` line to paste into `main.cpp` so it survives a reboot. `X` cancels.

### `sun_track(fan_u_pct, fan_l_pct, demo_req, desat)` (command 4; 8, 10 and 11 call it too)
The main test. Creates `SunTrack_dataNNN.csv`, waits for a key, then runs a 20 Hz loop
(`interval_testPoint` = 50 ms) through these modes:

| Mode | When | What happens | LED |
|---|---|---|---|
| SPINUP | 0 to `TRACK_SPINUP_MS` | Wheel ramps to the bias speed. Hold KestrelSAT still. | yellow |
| HOLD | to `TRACK_CONTROL_MS` | Wheel at bias. The gyro bias is measured while you hold it. At `TRACK_RELEASE_MS` the LED turns blue: let go. | yellow → blue |
| ACQUIRE | after that | Turns one full circle at `SCAN_RATE_DPS` (using `rate_step`) and records −X brightness against gyro heading. `scan_result` picks the sun's heading. No sun found → test ends. | purple |
| TRACK | after the scan | Holds the sun heading with the gyro and the PD controller (`tracker_step`). Sequential lobing (`lobe_step`) aims a few degrees each side to keep correcting gyro drift. If −X stays dim while pointed at the sun, it scans again. | green within 10°, orange outside |

Every pass it also: compensates for motor lag (`motor_lag_comp`), sends the wheel command
(`set_wheel_rpm`), warns if the wheel stays saturated, writes one CSV row (25 columns; the last, `dumping`, is 1 while command 11 thrusters fire), prints
telemetry (`Name:value` pairs, every point on USB, every 10th on XBee; the GCS Sun Map reads these),
and flushes the SD file every 10 points.

**Fans** (only when a duty above 0 is passed in, i.e. command 8): once pointing has stayed within
`FAN_HOLD_BAND_DEG` for `FAN_HOLD_TIME_MS`, the chosen fan pair turns on (100% for `FAN_KICK_MS` to
start it, then the requested duty). It reports when pointing first leaves the band and when the wheel
saturates, and ends the test once the wheel has been saturated for `FAN_SAT_STOP_MS`.

**The test ends** on `X`, no sun found in a scan, wheel saturated with the fans on, or after
`track_duration_ms` (command 4) / `FAN_HOLD_DURATION_MS` (command 8). Every ending goes through the
`end_test` helper inside the function: wheel off, fans off, file closed, message, and (command 8) a
summary of time to lose pointing and % of time inside the band.

### `sun_print_params()`
Prints the current tracker settings (kp, kd, deadband, rate cap, bias, array axis, wheel and gyro
signs, test length) and the sun calibration. Used by `sun_set_params()`.

### `sun_set_params()` (command 5)
Lets you change settings over the radio until reboot. Send `<name> <value>` lines, then `done`:
`kp`, `kd`, `db` (deadband, deg), `rate` (max slew, deg/s), `bias` (wheel bias PWM, ±0.9), `axis`
(array direction, deg), `wsign` / `gsign` (±1 wheel and gyro signs), `dur` (command 4 length, 30–3600 s).
Copy good values into `definitions.h` to keep them.

### `ad_check()` (command 6)
A guided PASS/FAIL checkout of the sensors with the wheel off. Logs `ADCheck_dataNNN.csv`. Send any
key to go, `s` to skip a step, `X` to cancel.
1. **Light off:** each channel should read low and steady, and no "sun" should be seen. Flags
   full-scale (shorted / missing resistor) and noisy (loose wire) channels.
2. **Light on 8 spots** (each face and corner, every 45°): measured sun direction must be within 15°.
   Flags saturated channels and, on a face, a different channel being brighter (swapped wires).
3. **Hold still 3 s:** gyro bias under 2 deg/s and noise under 1 deg/s.
4. **Turn KestrelSAT ~90° by hand** with the light fixed: the gyro's angle must match the sun sensor's
   within 15°. If they turn opposite ways, `gyro_sign` is wrong.

Prints a pass/fail count at the end.

### `pt_sweep()` (command 7)
Compares phototransistor resistor values. For each resistor you install: send its value (`4700`,
`10k`, `1M`), then take readings with the light off and on each face (200 readings each). Each row
(mean and noise of all four channels) is printed and logged to `PTSweep_dataNNN.csv`, plus a summary
of each channel's dark-to-bright span. Send `done` to finish; plot with `2. THEORY/plot_ptsweep.py`.

### `fan_hold_test()` (command 8)
Asks for the fan direction (`cw` = GPIO 27 pair, `ccw` = GPIO 4 pair, seen from above) and an
optional duty (default 100%), e.g. `ccw` or `cw 60`. Then runs `sun_track()` with that pair, so the
fans switch on once KestrelSAT has held the sun for 10 s.

### `req_demo()` (command 10)
Asks for `1`, `2` or `3` and runs `sun_track(..., demo_req)`. With `demo_req` set, `sun_track()` works
as usual, but once tracking starts it judges that requirement every sample and ends the test with a
PASS or FAIL line (progress every 5 s):
- **1 (R2 then R1)**: when the scan finds the sun, `sun_track()` sets `demo_offset = 180`, so the controller
  turns the array away before it ever tracks (lobing is paused, since −X sees nothing). After 3 s there it
  sets `demo_offset = 0` and starts a timer. R2 PASS if it enters ±10° within 10 s and stays in for 5 s; an
  overshoot out of the band restarts the reach time. Then it keeps holding: R1 PASS after 30 s in a row inside
  ±10° (timed from the R2 arrival; leaving the band restarts it). The last line gives both results and the %
  of the hold within ±5°.
- **2 (R3)**: same fan start as command 8. PASS after 30 s of fans on without leaving ±10°; FAIL the moment it leaves.
- **3 (S2)**: runs the command 11 momentum dump (`desat = true`). PASS once one dump has finished, pointing has
  never left ±10° since the fans turned on, and 10 s more have passed. FAIL the moment pointing leaves the band.
Settings: `DEMO_*` and `DESAT_*` in `definitions.h`. 1 and 2 give up (FAIL) after 3 min, 3 after 20 min.
`parse_fan_choice()` reads the optional fan words (`cw`, `ccw 60`) for 2, 3 and command 11.

### `momentum_dump_test()` (command 11)
Asks for the *disturbance* pair and duty (default 70%) and runs `sun_track(..., 0, true)`. The other pair
is the "thruster". After 10 s on target the disturbance fans turn on and the wheel spins up to fight them.
When the wheel command reaches `DESAT_START_RPM` (300), the thrusters fire at 100% against the
disturbance, so the controller has to slow the wheel to keep pointing. That is the momentum dump. Below
`DESAT_STOP_RPM` (80) the thrusters stop and the cycle repeats. `[DESAT]` lines mark each dump; the end
summary gives the number of dumps, their average length and the longest time inside ±10°. A dump longer
than `DESAT_MAX_MS` ends the test (the thrusters aren't unloading the wheel). The disturbance must be
below 100%, because two equal fans at 100% cancel.

---

## `sun_tracker.cpp`: the controller math

None of these touch hardware. They're called from `sun_track()` (and the calibration and check
commands) and copied into the Python simulation.

### `wrap180(angle_deg)`
Wraps any angle into −180 to +180°, so "350°" becomes "−10°". Used for every angle difference.

### `compute_sun(raw[4], cal)`
Turns four raw ADC counts into a sun reading. Each channel is scaled so 0 = its dark count and 1 =
its bright count (capped at 1.2 so one bad channel can't dominate). Then, like Lab 6:
`direction = atan2(+Y − −Y, +X − −X)`, in 0–360°. Also returns `magnitude` (sum of the four scaled
channels; below `SUN_LOST_THRESHOLD` means no sun) and each scaled channel (`norm`).

### `pointing_error_deg(sun_direction_deg, array_axis_deg)`
`wrap180(sun − array)`. Positive means the sun is counter-clockwise of the solar array, so KestrelSAT
must turn +Z.

### `latch_far_error(error, dt, enter, exit, exit_time, latch)`
Fixes the "sun exactly behind the array" problem. Near 180° the sign of the error flips with sensor
noise, so the controller would push left, then right, and go nowhere. Once |error| goes above
`LATCH_ENTER_DEG` it commits to one direction and, if noise flips the sign, returns the error the
long way round (e.g. −179° becomes +181°). It lets go once |error| has stayed under `LATCH_EXIT_DEG`
for `LATCH_EXIT_TIME_MS`.

### `tracker_step(error, body_rate, dt, gains, limits, I_wheel, w_cmd_rpm, saturated)`
One step of the PD controller. Works out the body torque wanted, written as "turn toward the target
at a desired rate":
- desired rate = (Kp / Kd) × error, capped at `max_rate_dps`
- torque = Kd × (desired rate − body rate)

Inside the deadband the error is treated as 0 (just damp the motion). The torque is then turned into
a wheel speed command (see `wheel_from_torque`). Returns the torque for logging, updates `w_cmd_rpm`,
and sets `saturated` if the wheel hit a speed limit.

### `rate_step(rate_des, body_rate, dt, ...)`
Same as `tracker_step`, but holds a set body rotation rate instead of a heading. Used to turn the full
circle during the ACQUIRE scan.

### `wheel_from_torque(...)` (internal to `sun_tracker.cpp`)
Newton's third law: to push the body with torque τ, the wheel must speed up the other way,
`wheel accel = −τ / I_wheel`. Limits the acceleration to `WHEEL_MAX_ACCEL_RPM_S`, adds it to the
wheel speed command, and clamps the speed between `WHEEL_MIN_RPM` and `WHEEL_MAX_RPM` (that clamp is
"saturated").

### `scan_start(scan, heading)` / `scan_add(scan, heading, brightness, turn_deg)` / `scan_result(...)`
The sun search. `scan_start` begins a new scan at the current heading. `scan_add` stores one
(heading, −X brightness) sample and returns true once KestrelSAT has turned `SCAN_TURN_DEG` (or the
600-sample buffer is full). `scan_result` finds the brightest sample in the middle 360° of the scan,
takes all the neighboring samples at least `SCAN_PEAK_FRACTION` of that peak, and returns their
brightness-weighted average heading as the sun heading. Returns false if the peak is too dim (no sun).

### `lobe_reset(lobe)` / `lobe_step(lobe, brightness, settled, dt, ...)`
Sequential lobing: fine pointing while tracking. The gyro heading slowly drifts, so the tracker aims
`LOBE_AMP_DEG` to one side for `LOBE_DWELL_MS`, then the other side. It averages −X brightness at the
end of each side (only while settled on the aim point). After each +/− pair it estimates how far
off-centre the sun is from the brightness difference, and moves the sun heading part of the way
there (`LOBE_GAIN`, at most `LOBE_MAX_STEP_DEG`). Returns the current aim offset. `lobe_reset` starts
it fresh after each scan.

### `motor_lag_comp(w_cmd, w_prev_cmd, dt, motor_tau, max_rpm)`
The real wheel takes about `MOTOR_TAU_S` to reach a new speed. This commands slightly ahead
(`w + τ × rate of change of the command`) so the actual wheel keeps up with the controller. Clamped to
the wheel's top speed.

---

## `sd_funcitons.cpp`: SD card

### `sd_init(csPin)`
Starts the SD card at 25 MHz. If there's no card it stops and prints an error every 2 s, because
nothing can be logged without it.

### `sd_createDataFile(&dataFile, preamble)`
Makes the next unused numbered file, e.g. `SunTrack_data001.csv`, then `002`, and so on up to 999,
and opens it for writing. The preamble (`SunTrack`, `ADCheck`, `PTSweep`) is cut to 12 characters.
Returns false if the card is full of numbered files or the file can't be opened. The numbering
restarts on a fresh card, which is why two different `SunTrack_data001` files exist in `4. DATA/`.
