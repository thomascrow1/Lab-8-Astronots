# GenAI Use Log (Lab 8, GenAI Level 4)

## 2026-10-29: XY-MOS photos and calibration using relative wheel response

**Tool:** Codex desktop with independent calibration review and simulation subagents.
**Request:** Review additional wiring photos and determine whether relative
thrust can replace measured thrust data. User also asked what TRIG/PWM means.

**Photo observations:** Modules visibly say XY-MOS with VIN+/VIN-, OUT+/OUT-,
GND and TRIG/PWM labels. Power/load wires are visible; exact GPIO control,
common-ground paths and breadboard contact mapping remain unverified. Updated
the guide to these actual terminal labels instead of the earlier Adafruit example.
Explained trigger HIGH/LOW, common ground, and full-ON/OFF rather than assuming
two-wire brushless fan thrust varies linearly with PWM. No pin assignments changed.

**Changes:** Added preferred RPM/s calibration settings, all zero by default,
and `fan_equivalent_torque_nm()`. Calibrate each bank from measured physical
wheel RPM slope ON minus comparable OFF baselines while freely hanging under
stable closed-loop pointing. Relative bank ratios alone need one measured scale.
The converted equivalent torque uses the same configured wheel inertia as the
tracker; inertia cancels from fan feedforward. Model torque is not measured Nm.
Documented comparable body acceleration/disturbance assumptions and exclusion of
transients/saturation. A physical thrust sensor and accurate wheel inertia are
not required for this method. It does not automate calibration pulses.

**Verified:** 22 Python control/simulation tests pass; one optional host C++
parity check is skipped because no host compiler is installed. The new physical
plant test checks both signs with real wheel inertia different from configured
inertia. PlatformIO compilation succeeds. Fan actuation stays disabled and
response settings remain zero; nothing was uploaded or operated on hardware.

**Still required:** Trace and verify TRIG/PWM and common ground, exact controller
GPIO labels, regulated fan supply, airflow torque signs, encoder sign, and the
recorded reverse-drive/timing issues. Obtain per-bank ON/OFF response data and
rise/decay before enabling automatic unloading.

## 2026-10-29: Fan-assisted wheel unloading (hardware confirmation pending)

**Tool:** Codex desktop with independent control review and simulation/test subagents.
**Request:** Use four LD3007MS 5 V fans to unload reaction-wheel momentum while
continuing sun pointing. The user confirmed fan centers Y = +/-0.32 m (radius),
Z = +/-0.02 m, fans face X; top fans share one two-wire bank and bottom fans
share another. Unloading is allowed during slews; on-target requirement is +/-10 degrees.

**Changes:** Added hardware-free `fan_desaturation.h/.cpp`, integrated into
command 13, and mirrored it in the Python simulation. Positive wheel momentum
selects negative fan torque; the wheel integrates `(fan_torque - pointing_torque)/I_wheel`.
Supervisor includes measured/command speed hysteresis, near-target gates,
slew unloading, timeout, feedback/timing faults and modeled fan coast-down.
Command 12 also estimates encoder sign. SD/telemetry include fan state/torque/fault.
Proposed new driver-input pins: top GPIO27, bottom GPIO4 (FREE connection).
No existing pin was changed. `FAN_WIRING_CONFIRMED=false`, fan enable=false,
fan torque=0, and torque/encoder signs=0 prevent unconfirmed fan actuation.
Added `FAN_DESATURATION.md` with geometry, wiring through two 3.3 V-compatible
MOSFET drivers, calibration, assumptions and recorded-data findings.

**Data review:** Existing command limit is +/-400 RPM, not a verified motor
maximum. Older runs 045-046 show roughly +/-350 measured RPM at full command;
recent 050-052 show near-zero measured RPM during sustained reverse commands.
Run 052 has seven roughly 1.02 s control gaps. These need hardware/timing checkout.
Initial settings are actual 200 RPM start /100 stop, or command >=300 with
same-sign measured >=150; these are provisional, not measured fan performance.

**Verified:** PlatformIO `esp32thing_plus` build succeeds (RAM 24,120 bytes,
flash 399,549 bytes). Used short temporary library/build directories to avoid
Windows dependency filename-length errors. Headless unittest: 21 passed,
one optional host C++/Python runtime comparison skipped because no host C++
compiler is installed; the actual ESP32 C++ firmware compiles. Tests cover both
directions, command hysteresis, geometry, slew unloading, motor lag/stall,
thrust mismatch, coast-down, faults, timing, sun loss and torque limits.
Synthetic mismatched fan models unload both signs from +/-250 to about +/-104
RPM; this is not an LD3007MS thrust measurement or hardware accuracy claim.

**Still required:** Close-up board/fan/wiring photos; airflow and torque sign
verification; confirm parallel fan wiring, driver, 5 V supply and current;
measure installed torque and rise/decay; check reverse wheel drive and control
gaps; upload only after configuration and verify both directions, pointing,
abort, sun loss and runtime/timing faults on the satellite. Nothing was uploaded.

One entry per agent session. Record what the agent made, what we changed, and how we verified it.

## 2026-10-06 · Sun tracker plan + initial implementation
**Tool:** Claude Code (desktop app), model Claude Opus 5.5
**Driver:** _[name]_  **Reviewers:** _[names]_

**Prompts:** Asked for a plan to build a sun tracker using the Lab 6 phototransistors and the Lab 7 reaction wheel. The agent read the Lab 6/7/8 slides and code, proposed a plan, and asked four design questions. Our answers:
- Solar array faces **−X (180°)**
- **PD controller with wheel bias**
- Deliverables: firmware, Python sim, plotting script, docs
- If the sun is lost: **stop the wheel and report**

**Agent made:**
- `KestrelSAT_SunTracker/`: copy of Lab 7 code plus commands 10–14 (calibrate, stream, sign check, track, parameters), `sun_tracker.h/.cpp`, and settings in `definitions.h`
- `sim/sun_tracker_sim.py`: 1-axis simulation using the same controller
- `analysis/plot_suntrack.py`: plots SD logs and scores them against R1/R2
- `README.md`, `CLAUDE.md`, this log

**Agent verified (no hardware):** compiled `sun_tracker.cpp` natively with no warnings, and checked that its outputs match the Python controller exactly. Ran all six sim scenarios. Ran the plot script on a simulated log.
**Not verified:** a full PlatformIO build of `main.cpp` (espressif32 platform not installed on this machine), and anything on KestrelSAT.

**We must verify on hardware:**
- [ ] PlatformIO build + upload succeeds
- [ ] Command 11 shows +X ≈ 0°, +Y ≈ 90°, −X ≈ 180°, −Y ≈ 270° after calibration (10)
- [ ] Command 12 reports consistent gyro and wheel signs. Copy them into definitions.h.
- [ ] Command 13 turns the −X face toward the light, not away from it
- [ ] Measure the string twist period and motor time constant, and rerun the sim with them

**We changed:** _[fill in]_
**Issues found:** _[fill in]_


## 2026-10-13: GCS Sun Map + reaction wheel debugging

- Wheel not spinning: the log showed `wMeas` = 0 and encoder counts = 0 for every motor command, and the motor code matched Lab 7. Diagnosed as wiring. Team fixed the wiring and the wheel now spins.
- Added `4. GCS MACOS/sun_map.py`, a "Lab 8 > Sun Map..." window: a live polar map of sun sensor brightness around 360°, with the brightest spot, firmware sDir, and a selectable update rate. `gcs_mac.py` installs it. The compiled GCS (`kestrelsat/*.pyc`) is unchanged.
- Rebuilt `.venv` (it was stale after the folder moved) and `dist/KestrelSAT GCS.app`.

**Agent verified (no hardware):** interpolation and peak math; the window opens from the real GCS menu, receives lines through the GCS, and closes cleanly; layout checked from a rendered export; `sun_map` is packed into the rebuilt app.
**We must verify on hardware:**
- [ ] Open the rebuilt app, choose Lab 8 > Sun Map, run option 11, and check that the brightest spot follows the lamp

## 2026-10-13: Tracker turned away from the light (wheel sign) + faster gains

**Found in SunTrack_data020:** each time the wheel actually sped up (`wMeas` going from 0 to -99 RPM at t=54.7-57 s, err ≈ -88°), KestrelSAT sped up +Z (gZ +7 to +25 deg/s), `sDir` dropped 92° → 340°, and the error grew. The controller was pushing the wrong way. `wheel_pwm_sign = -1` came from a command 12 run while the wheel was disconnected (encoder counts = 0), so that result was not real. The same data showed the wheel stalls below ~45 RPM and only restarts at ~50-70 RPM, so the 40 RPM floor was inside the stall zone.

**Changed:**
- `definitions.h`: `DEFAULT_WHEEL_PWM_SIGN` -1 → 1, `WHEEL_MIN_RPM` 40 → 80, `DEFAULT_BIAS_PWM` 0.5 → 0.6 (~250 RPM, middle of 80-400), gains wn 0.5 → 1.0 rad/s (`DEFAULT_KP` 0.0035 → 0.014, `DEFAULT_KD` 0.0098 → 0.0196). The sim constants were changed to match.
- `main.cpp` command 12: if the encoder barely moves (< 20 counts), print a caution and leave the signs unchanged.

**Agent verified (no hardware):** PlatformIO build succeeds. In the sim, new vs old gains: step90 settles in 2.1 s vs 4.1 s; string torsion (k=0.0005) settles in 2.1 s vs never; moving light (5 deg/s) settles in 1.2 s vs never. Strong string (k=0.002) fails with both: the wheel saturates.
**We must verify on hardware:**
- [ ] Upload, then run 12 hanging and still: expect `wheel_pwm_sign = 1` and non-zero encoder counts
- [ ] Run 13: after 10 s, err should move toward 0 and stay inside ±10°
- [ ] If it overshoots and swings, raise kd (option 14); if the wheel keeps saturating, untwist the string so it rests with -X toward the light

## 2026-10-13: Tracker ignoring the light = bad sun calibration

**Found:** The calibration was applied with +Y dark 533 / bright 537 (a 4-count range). That made +Y read ~1000x full brightness, so sDir was stuck at 90.0 for all of SunTrack_data025 and the wheel ran up to 400 RPM chasing a fake error. Earlier streams had n values in the thousands, from a calibration where the light didn't move between steps.
**Changed:** `main.cpp` command 10: any channel with bright < dark + 300 counts is reported, and the whole calibration is NOT applied (the previous one is kept).
**Verified:** PlatformIO build succeeds.
**We must verify on hardware:**
- [ ] Recalibrate (10) with the light close and straight at each face; paste the line into main.cpp
- [ ] With 11 and the light off, n values are ~0; with the light on a face, that face's value is ~1

## 2026-10-13: ~30° steady pointing offset + wheel saturating one way

**Found in SunTrack_data026:**
- The calibration was still bad. With the light off, -Y read n = 7.7: room light on -Y (~3900 counts) was far above its calibrated "bright" of 519. +Y also only had a 356-count range.
- From t = 43-62 s, err held at about -30° while wCmd ramped steadily 83 -> 400 RPM (~18 RPM/s). That is a constant torque (string twist): PD control holds where Kp*err balances it, and the wheel must keep accelerating to supply it until it saturates. When err passed ±180°, the controller did reverse (wCmd 398 -> 80), then saturated at the low end.
**Changed:** `compute_sun()` caps each calibrated channel at 1.2 (mirrored in the sim's `compute_sun`).
**Verified:** PlatformIO build succeeds; all sim scenarios unchanged (step90 2.1 s, string 2.1 s, moving 1.2 s).

## 2026-10-13: Sensor data during tracking + Sun Map shows the satellite

**Changed:**
- `main.cpp` command 13: each telemetry line also carries command 11's fields (`mag`, `n+X`, `n+Y`, `n-X`, `n-Y`), so the Sun Map works during tracking.
- `4. GCS MACOS/sun_map.py`: the ring is now in the room frame; KestrelSAT is drawn as a rotating square (heading from integrated gZ, with the bias learned while the sun direction is steady, plus a Reset heading button); the -X array line turns green within ±10° and an arc shows the error. App rebuilt.
**Verified:** PlatformIO build succeeds. With synthetic data (fixed lamp, satellite spinning 20°/s, 0.5°/s gyro bias): bias learned 0.49, heading 101.2° vs 100.0° true; layout checked from a rendered export. The end-to-end GCS menu test could not run this time (Qt would not load in the agent's shell); install() is unchanged from the earlier verified version.
**We must verify on hardware:** run 13 with the Sun Map open; the square should turn with KestrelSAT and the SUN marker should stay put while the lamp is still.

## 2026-10-13: Real sun calibration in main.cpp

- The ±Y phototransistors were fixed, and the new command 10 result is now the default `sun_cal` (all channel ranges are 3175-3822 counts; ±Y peaked at 4095).
- PlatformIO build succeeds. Upload, then check with 11.

## 2026-10-13: Wheel spins both ways + fast slew (target 500 ms)

**Measured:** wheel speed time constant ≈ 1.1 s (fit to the SunTrack_data026 spin-up). With that lag in the sim, the old gains took ~10 s to settle 90°, and higher gains went unstable.
**Changed:**
- Wheel range is now -400..+400 RPM (crosses 0) with bias 0. Commands smaller than 50 RPM jump to ±50 to get past the stall zone.
- Motor lag compensation: `motor_lag_comp()` in sun_tracker.cpp commands w + tau·dw/dt (mirrored in the sim).
- wn 4 rad/s: Kp 0.224, Kd 0.0784. Wheel accel limit 50 -> 400 RPM/s (above the Lab 7 prelab value).
**Verified (no hardware):** PlatformIO build succeeds; the C++ tracker_step/motor_lag_comp match the Python sim. Sim with 1.1 s motor lag: 30° in 0.40 s, 90° in 0.70 s, 170° in 0.90 s, no saturation; stays at 0.5-0.7 s for 90° if the real lag is 0.7-1.6 s or with a cable spring. 500 ms is met for slews up to ~45°.
**Must verify on hardware:** step response with 13; if it overshoots or oscillates, use 14: `kp 0.126` `kd 0.0588` (wn 3).

## 2026-10-13: Overdamped, smooth slews (no overshoot)

**Changed:**
- Gains: wn 3 rad/s, zeta 1.3 (overdamped): Kp 0.126, Kd 0.1092.
- tracker_step now writes PD as "turn at a desired rate": rate_des = (Kp/Kd)·err, capped at `max_rate_dps` (default 90°/s), tau = Kd·(rate_des - rate). This is identical to PD below the cap. The cap can be changed with command 14 `rate`. Mirrored in the sim.
**Verified (no hardware):** PlatformIO build succeeds; the C++ matches the Python sim. Sim with 1.1 s motor lag, settle to 5° / overshoot: 30° 1.4 s / 0.5°, 90° 2.2 s / 0.4°, 170° 3.2 s / 0.4°; cable spring 2.5 s / 0°; motor lag 0.7-1.6 s 1.9-2.2 s / ≤0.5° (overshoot is at sensor-noise level).
