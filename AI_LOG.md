# GenAI Use Log (Lab 8, GenAI Level 4)

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

## 2026-10-30: Post-rewire checkout (command 15) + phototransistor resistor sweep (command 16)

**Changed:**
- `main.cpp` command 15 `ad_check()`: guided attitude determination check, wheel off. Light off (flags full-scale, noisy, or room light seen as sun), light at 8 spots every 45° (PASS if within ±15°, flags saturated channels and swapped pins), gyro still for 3 s (bias/noise), then a hand turn of ~90° comparing gyro angle to sun sensor angle (catches a flipped gyro_sign). Logs `ADCheck_dataNNN.csv`.
- `main.cpp` command 16 `pt_sweep()`: for each resistor value typed in, records mean/std of all 4 channels with the light off and on each face. Logs `PTSweep_dataNNN.csv` and prints each row so the data can be copied if the SD card fails.
- New `analysis/plot_ptsweep.py` (brightness, span, photocurrent vs R; best R = biggest worst-channel span with nothing above 3900 counts) and `analysis/plot_adcheck.py`.
- Note: a git merge put back the old main.cpp; the new code was restored from the "main 2.cpp" copy in commit 5401082.
**Agent verified (no hardware):** PlatformIO build succeeds. Both plot scripts run on made-up data and pick the expected answer.
**We must verify on hardware:**
- [ ] Run 10 (recalibrate after the rewire), then 15; all 8 spots and the hand turn should PASS
- [ ] Run 16 with each resistor at a fixed light distance; plot it, install the best R, then rerun 10 and paste the new sun_cal line

## 2026-10-30: 180° start never turned toward the light (SunTrack_data008) → direction latch

**Found in SunTrack_data008:**
- Started with the light on +X (error ±180°). The error stayed at |e| > 170° for 96% of tracking, and its sign flipped 487 times in 1036 samples (±Y noise of a few counts decided it). The torque command reversed 509 times and averaged −0.004 N·m, so the wheel chattered and KestrelSAT only wobbled −48°..+23°. In the first 0.9 s, the error held at +179° and the body did turn +20° the right way before a flip reversed it.
- −X read 0 with the wheel off and jumped to 500–4095 counts (61 samples) once the wheel ran, while +X was lit: electrical noise, not light. Each spike made the error read ~0°.
- ±Y stayed under ~200 counts even after a 50° turn off +X, so the sensor is nearly blind between faces.
- `sun_cal` in main.cpp is still the 13 Oct (10 kΩ) calibration; the run used a calibration done at runtime.
**Changed:**
- `sun_tracker.cpp/.h`: `latch_far_error()`. Once |error| > 150° it commits to a turn direction; if noise flips the sign it reports the error the long way round (−179 → +181). It releases after |error| < 120° for 300 ms, so a short sensor spike can't release it. `LATCH_*` settings are in `definitions.h`.
- `main.cpp` `sun_track()`: the controller uses the latched error. SD log adds `err_ctrl(deg)` and `latch`; telemetry adds `latch`.
- `sim/sun_tracker_sim.py`: same latch (`use_latch`), plus a sensor model option (`half_power_deg`, `mx_spike_prob`) and scenarios `step180`, `step180hw`.
**Agent verified (no hardware):** PlatformIO build succeeds. The C++ and Python latch give identical output on the SunTrack_data008 error sequence; there, sign flips while far from target drop from 487 to 2 and the latch never changes direction. This replay is open-loop (it can't show the real satellite's response). Other sim scenarios are unchanged (step90 1.4 s, step170 2.4 s, hold 100%).
**We must verify on hardware:**
- [ ] Run 13 starting with the light on +X (180°): `latch` should go to ±1 and stay, and KestrelSAT should turn one way until it faces the light
- [ ] Check −X noise: send 5 (50%), then 11 with the light off; −X should stay near 0
- [ ] Run 10 and paste the new sun_cal line into main.cpp (3.3 kΩ calibration)

## 2026-10-30: Sun sensor read rotation backwards (SunTrack_data009) → ±Y pins swapped

**Found in SunTrack_data009 (array started 90° from the light):** the controller asked for −Z, and the gyro (−65°) and the video both show a clockwise turn. But the light moved from the +Y channel to the +X channel (sun direction 89° → 0°), when a −Z body turn should move a fixed light toward −X (180°). SunTrack_data008 shows the same reversal. With that sign flipped, the true target is unstable and "array facing away" becomes the resting point, which matches both runs. The team confirmed ±X were right and asked for ±Y to be swapped in software.
**Changed:**
- `definitions.h`: `SUN_SENSOR_PLUS_Y_PIN` A1 → A3, `SUN_SENSOR_MINUS_Y_PIN` A3 → A1.
- `main.cpp`: swapped the ±Y entries of the stored `sun_cal` to follow the pins (still the stale 13 Oct values).
- `CLAUDE.md`: pin map updated.
**Agent verified:** PlatformIO build succeeds.
**We must verify on hardware:**
- [ ] Command 11: light on +Y face → sDir ≈ 90°; on −Y → ≈ 270°
- [ ] Run 10, paste the new sun_cal line
- [ ] Command 15: step 4 (hand turn) should PASS with the gyro and sun sensor agreeing
- [ ] Command 13 from 90° and from 180°
- **Follow-up (command 11 + 10 on hardware, 30 Oct):** with the swap, spinning KestrelSAT +Z (gZ +72..+154) moved the light −Y → −X → +Y (sDir 270 → 180 → 90): the sun sensor and gyro now agree. Calibration (10) failed on ±Y because the light was aimed at the faces as labeled; the face labeled +Y is body −Y. Aim "+Y" steps at the face labeled −Y, or relabel the ±Y faces. Also, only one channel was ever lit at a time (sDir only 90/180/270), so there are blind gaps between faces.

## 2026-10-30: ±Y pin swap reverted

**Found (hardware, after the swap):**
- Command 15 step 4 (slow hand turn, strong signals at both ends): gyro −90.1°, but the sun sensor said the body turned +89.4° (sDir 181 → 92). They still disagree, so the swap made it wrong.
- SunTrack_data010 (swap + latch): latch −1, body turned −Z as commanded (gZ −44, −66 °/s), but the light moved +X → −Y; a −Z turn should move it toward +Y.
- The earlier "follow-up" reading of command 11 was a mistake: the gyro adds up to only ~35° there while the light jumped 180°, so the light moved too. It was not a valid rotation test.
- SunTrack_data009 still looks backwards with the original pins. It can't be reconciled with step 4 and 010 using the same wiring; a sensor wire may have been re-seated in between. Re-check with 15 step 4 using a lamp on a stand.
- All four corner steps of command 15 failed (mag 0.07–0.20, at most one channel lit): the frame shades the face-mounted sensors. In 010 the sat swung into a gap, the sun was "lost" for 0.5 s, and the tracker stopped.
- The latch worked as intended in 010 (held −1, one turn direction, no chatter).
**Changed:**
- `definitions.h`: `SUN_SENSOR_PLUS_Y_PIN` back to A1, `SUN_SENSOR_MINUS_Y_PIN` back to A3.
- `main.cpp`: `sun_cal` is the 30 Oct command 10 result (3.3 kΩ) with its ±Y entries re-ordered for these pins: `{{0.0, 34.6, 0.0, 0.0}, {2086.7, 1680.1, 1777.8, 2771.7}}`.
- `CLAUDE.md`: pin map back to A1 = +Y, A3 = −Y; resistor 3.3 kΩ.
**Agent verified:** PlatformIO build succeeds.
**We must verify on hardware:**
- [ ] Upload, run 11: light on +Y face → sDir ≈ 90° (no recalibration needed)
- [ ] Command 15 with the lamp on a stand ('s' to skip to step 4): step 4 should PASS
- [ ] Command 13 from 90°

## 2026-10-30: Scan-and-hold tracking (fixes the ~20° steady offset in SunTrack_data011)

**Found in SunTrack_data011 (first run that tracked):** after the slew, only −X saw the light (n−X 0.7–1.2, others ≤ 0.02), so sun direction read 181.0 ± 0.2° the whole hold while the brightness drifted. The controller saw ~1° error (inside the 3° deadband), and the string left KestrelSAT ~20° off.
**Changed (team's design):**
- `sun_tracker.cpp/.h`: `rate_step()` (turn at a set rate; shares `wheel_from_torque()` with `tracker_step`), and the scan: `scan_start()`, `scan_add()`, `scan_result()`. The sun heading is the brightness-weighted centre of the −X samples ≥ 80% of the peak, searched in the middle 360° of a 420° turn.
- `main.cpp` `sun_track()`: gyro bias measured while held; after release: ACQUIRE (one 420° turn at 45°/s, LED purple) → TRACK (PD on `wrap180(sun_heading − heading)`, with the latch). Rescan if −X < 50% of the scan peak for 1 s while within 10° of the sun heading. Test stops if a scan finds no sun. The old "sun lost" stop and the "no sun at start" abort are removed. SD log adds `heading(deg)`, `sun_heading(deg)`; telemetry adds `hdg`, `sunHdg`, `acq`.
- `definitions.h`: `SCAN_*`, `REACQ_*` settings.
- `sim/sun_tracker_sim.py`: same logic (`use_scan`), plus the ADC floor (`adc_floor`), a gyro bias, true error logging, and scenario `hwoffset`.
- `analysis/plot_suntrack.py`: ACQUIRE shading, plots `err_ctrl`, and R1 counts from the scan start.
**Agent verified (no hardware):** PlatformIO build succeeds. C++ and Python scan agree to 0.0001° on 300 random scans; the scan finds the sun within 0.65° on average (worst 2.75°) at the hardware's 33° half-power angle with noise and the ADC floor. Sim, true error: `hwoffset` 27° (old) → ~2° (new); all fixed-light scenarios settle in 11–12 s; `moving` (5°/s) now fails (keeps rescanning); `sunlost` stops after the rescan finds nothing. With string twist the wheel ramps toward −400 RPM during a long hold (saturates after ~1 min in `hwoffset`).
**We must verify on hardware:**
- [ ] Command 13: LED purple, KestrelSAT turns one full circle, prints "Sun found at heading …", then turns back and holds; check by eye/video that −X faces the lamp
- [ ] Watch the wheel speed during a long hold (string twist)
- [ ] Block the lamp for >1 s: it should print "Rescanning" and spin again

## 2026-10-30: Sequential lobing so tracking corrects drift without rescanning

**Found in SunTrack_data014:** the scan worked (sun heading 186°) and the gyro heading held at 546–550° for 100 s, but −X fell from 1.20 to ~0.55: the array actually drifted ~20–30° because the gyro bias (−0.272°/s) was measured by hand. Also, `n-X` is capped at 1.2 in `compute_sun()`, which flattened the top of the brightness peak.
**Changed:**
- `sun_tracker.cpp/.h`: `LobeState`, `lobe_reset()`, `lobe_step()`. While tracking, aim +4° then −4° (2.5 s each), average −X brightness over the last 1 s of each side (only while within 3.5° and < 5°/s), and move the sun heading by 0.5 × the estimated offset (max 5° per cycle). Offset estimate for a cos^p sensor: ln(b+) − ln(b−) = 2·p·amp·offset.
- `main.cpp`: the scan and lobing use uncapped −X brightness (`mx_bright`); the PD aims at sun heading + lobe offset; rescan only below 30% of the peak. SD log adds `aim_offset(deg)`, `mx_bright`; telemetry adds `aim`, `bX`.
- `definitions.h`: `LOBE_*`; `REACQ_FRACTION` 0.5 → 0.3. Sim mirrored, plus `gyro_drift_dps` and scenario `drift`.
**Agent verified (no hardware):** PlatformIO build succeeds; the C++ and Python lobe_step match exactly on 4000 random samples with a 1/16 s step (with 0.05 s steps, float vs double rounding can move one boundary sample into the next averaging window). Sim true error with a 0.25°/s gyro error: max 24° → 8° (100% within ±10°); other fixed-light scenarios stay within 1–5°. A 5°/s moving light still fails (lobing follows ≤ ~1°/s).
**We must verify on hardware:**
- [ ] Run 13 for 2 min: `aim` alternates ±4, `sunHdg` changes slowly, and `bX` stays near the scan peak (no slow fade like 014)
- [ ] If it wobbles too much, try a smaller `LOBE_AMP_DEG` (3) or a longer `LOBE_DWELL_MS`

## 2026-10-30: Toward ±5° pointing (after SunTrack_data015)

**Found in SunTrack_data015 (best run yet):** two things limited the fine pointing.
1. Lamp flicker: −X raw jumped ±300 counts sample to sample in an alternating pattern (e.g. 2677, 1932, 2747, 1864), the same with the wheel stopped (sd 316) as running (sd 281), so it is not motor noise. The 5 ADC reads per sample were back-to-back (~0.25 ms), so each caught a random point of the 120 Hz flicker. Lobe estimates were noisy and `sunHdg` wandered 167.7°–180.1°.
2. Deadband ate the lobe: in every lobe the heading stopped ~3.5° short of the ±4° aim point (inside the 3° deadband), so the real swing was ~±1° and the brightness difference ~¼ of what `lobe_step` assumes.
Also: the `error(deg)` column reads ~0° the whole run (only −X lit), so the true accuracy has to come from the video.
**Changed:**
- `main.cpp` `read_sun_raw()`: 16 reads per channel spread evenly over 8.333 ms (one flicker period) with `micros()`; ~8.3 ms per call, inside the 20 ms loop. Affects every command that reads the sun sensors (they get slower by ~8 ms per sample).
- `definitions.h`: `SUN_READS_PER_SAMPLE` 16, `SUN_READ_SPAN_US` 8333; `POINTING_DEADBAND_DEG` 3 → 1; `LOBE_AMP_DEG` 4 → 3; `LOBE_DWELL_MS` 2500 → 2000; `LOBE_GAIN` 0.5 → 0.7.
- `sim/sun_tracker_sim.py`: same constants; `simulate()` takes `deadband_deg`.
**Agent verified (no hardware):** PlatformIO build succeeds. Sim, true error after 30–40 s, 6 seeds of `drift` (0.25°/s gyro error, sensor noise 0.03): within ±5° 46% → 93–95%, max 8.3° → ~6–7.6°. With hardware-level noise (0.13, i.e. if the flicker fix doesn't work) the new tuning still beats the old one (68% vs 47% within ±5°). Fixed-light scenarios still settle in 11–13 s (step180hw 17.6 s, same as before); `hold` max 3.4°.
**We must verify on hardware:**
- [ ] Command 11 with the lamp on −X: the −X count should now change by tens, not hundreds, between lines
- [ ] Command 13: CSV `mcu time(ms)` should still step by 20 ms (the loop isn't overrunning)
- [ ] Command 13 for 2 min with video: within ±5°? `aim` alternates ±3 and the heading actually reaches each aim point (`err_ctrl` swings through 0)
- [ ] If it chatters near the sun (wheel stall at ~45–50 RPM), raise `db` to 1.5 at runtime (command 14)

## 2026-11-02: Disturbance fans (command 17) and thrust/torque model

**Goal:** four LD3007MS fans (y = ±32 cm; upper pair on GPIO 27 faces −X, lower pair on GPIO 4 faces +X) add a known Z disturbance torque so we can measure how long the reaction wheel holds pointing.
**Changed:**
- `definitions.h`: `FAN_*` settings (pins 27/4, LEDC channels 0/2, 25 Hz PWM, 500 ms start kick, fans on 10 s after the sun is found, stop 10 s after saturation).
- `main.cpp`: `set_fans()`, `start_fans()`, command 17 `fan_control()` (`u`, `l`, `both`, `off`, `freq`). Command 13 turns the fans on after the first scan + 10 s, logs `fan_upper(%)`, `fan_lower(%)`, telemetry `fanU`/`fanL`, prints `[RESULT] Wheel saturated N s after the fans turned on`, and ends 10 s after saturation. Every command 13 exit and command 0 also stop the fans. Command 14 adds `dur` (command 13 length, s).
- `analysis/fan_thrust_model.py`: fan-law thrust/torque/saturation-time curves (proxy datasheet CFM-3007CF-085-264; no LD3007MS data published) and `--csv` to measure the real torque from the wheel speed slope (fans on minus fans off).
**Agent verified (no hardware):** PlatformIO build succeeds; GPIO 4/27 are not used by the sun sensor (A0–A3 = 26, 25, 34, 39). The `--csv` torque estimate recovers a synthetic 5 RPM/s slope exactly (1.833 mN·m).
**Theory (per pair, 100%):** 3.1–6.1 mN·m if the pair is a couple about Z → wheel saturates in ~24–48 s from 0 RPM; ~95–190 s at 60%.
**We must verify on hardware:**
- [ ] Which way each fan blows: the +Y and −Y fans of one pair must push opposite ways, or their Z torques cancel
- [ ] Command 17: lowest duty that keeps the fans spinning (set `D0` in the model); try `freq 20000` if 25 Hz is rough
- [ ] XY-MOS modules stay cool at 3.3 V gate drive (0.4 A per module)
- [ ] 5 V supply holds up with the wheel + 0.8 A of fans
- [ ] Command 13 with `dur 300` and fans at 0 (baseline), then e.g. 60%, 80%, 100%

## 2026-11-02: Fans didn't spin: 3.3 V can't fully turn on the XY-MOS

**Found (hardware):** command 17 at `both 100`: XY-MOS LEDs lit, fans spin when wired straight to the 5 V rail, but only 2.2 V across OUT+/OUT−. The D4184 MOSFETs are only partly on with a 3.3 V trigger.
**Changed (team chose to keep the XY-MOS):** a 2N2222 in front of each trigger (GPIO → 2.2 kΩ → base, emitter → GND, collector → TRIG, 220 Ω TRIG → 5 V). This flips the signal, so:
- `definitions.h`: `FAN_PWM_INVERTED 1`.
- `main.cpp` `set_fans()`: inverts the duty when `FAN_PWM_INVERTED`; fan PWM setup moved to the very top of `setup()` (before the Serial wait) so the fans stop as soon as the board boots.
**Agent verified (no hardware):** PlatformIO build succeeds; `ledcWrite` treats 1023 as a true 100% (pin held high), so "fans off" is a steady high.
**We must verify on hardware:**
- [ ] `both 100`: OUT+ to OUT− ≥ 4.7 V (if lower, use a smaller pull-up, e.g. 150 Ω)
- [ ] `off` / `done`: fans stop and OUT ≈ 0 V
- [ ] Fans run at full speed during upload/reset until `setup()` starts (expected with this circuit)

## 2026-11-02: Back to direct GPIO → XY-MOS (no transistor or resistors available)

**Changed:** `FAN_PWM_INVERTED` 1 → 0 (the 2N2222 stage was never built; with 1 the fans were backwards and only half-on at "off", which looked like "sometimes works"). Fan pins set to the strongest drive (`GPIO_DRIVE_CAP_3`) so the trigger LED doesn't pull the gate voltage below 3.3 V.
**Agent verified (no hardware):** PlatformIO build succeeds.
**We must verify on hardware:**
- [ ] `both 100`: measure OUT+ to OUT− again (was 2.2 V) and TRIG to GND (should be ≈ 3.3 V)
- [ ] Do the fans start by themselves at `both 100`, or need a flick of the blade?
- [ ] Use the wheel-slope torque from `fan_thrust_model.py --csv`, not the 5 V theory table

## 2026-11-02: Diagonal fan pairs + command 18 fan torque sweep

**Found (hardware):** each pin drove two fans blowing the same way at +Y and −Y, so their Z torques cancelled at any duty. The team rewired the fans diagonally: GPIO 27 (`u`) now twists KestrelSAT clockwise seen from above, GPIO 4 (`l`) counter-clockwise. With the weak 3.3 V drive, fans at 100% sometimes cut out; 50% ran reliably.
**Changed:**
- `main.cpp`: command 18 `fan_sweep()`: gyro bias → for each duty 0–100% (10% steps), GPIO 27 pair then GPIO 4 pair. SETTLE (fans off, `rate_step(0)` stops the body with the wheel) → ON 4 s (wheel at constant PWM). Torque = I_BODY_Z·(least-squares gyro-rate slope) + I_WHEEL·(wheel accel from encoder counts, 2nd half vs 1st half of the window), fitted from 1.5 s to 4 s. `[RESULT]` line per step + table; logs `FanSweep_dataNNN.csv` (heading, rate, smoothed accel, wheel cmd/meas, encoder count).
- Pair labels: prints and CSV/telemetry renamed to GPIO 27 / GPIO 4 (`fan_gpio27(%)`, `fan_gpio4(%)`, `fan27`, `fan4`); `definitions.h` `FAN_SWEEP_*`.
- `analysis/plot_fansweep.py` (new): recomputes the torques from the raw log, plots |torque| vs duty per pair over the 5 V theory band, plus heading/rate/accel with the ON steps shaded. `fan_thrust_model.py` reads the renamed columns.
**Agent verified (no hardware):** PlatformIO build succeeds. On a synthetic sweep (gyro noise 0.3 °/s, encoder-quantised wheel with a random drift each step, 0.5 s start kick) the torque is recovered within 0.05 mN·m worst / 0.015 mean of a 1 mN·m signal. The first version fitted the 50 ms wheel RPM readings and was off by up to 0.5 mN·m, so it was switched to encoder counts.
**We must verify on hardware:**
- [ ] Command 18: GPIO 27 steps read negative (CW), GPIO 4 positive (CCW). If both are backwards, `gsign` is wrong
- [ ] SETTLE stops the body in < 20 s between steps (no "[CAUTION] Body still turning")
- [ ] Wheel speed stays well inside ±400 RPM over the sweep
- [ ] At 100% the fans may cut out (stall protection); a sudden drop in torque at the top duties means that happened
- [ ] String twist: large steps turn up to ~40°, so the top-duty torques may read a little low

## 2026-11-02: FanSweep_data001 (first command 18 run): not usable, fix deferred

**Found:**
- Fans sag the shared 5 V rail, so the open-loop wheel slows when they switch on (−4 RPM at 20% up to −22 RPM at 100%) and speeds back up when they stop. That swaps momentum with the body: every step, both pairs, kicked KestrelSAT clockwise (turned −1° to −22°, rate jumps then stays flat). Total heading drift −247° ≈ the sum of those kicks.
- Real fan torque (weak 3.3 V XY-MOS drive) looks like ~0.1–0.3 mN·m, about the same as the per-step noise (gyro jitter ±2–3 °/s, not the 0.3 °/s assumed in the synthetic test).
- "Body still turning" every step: the 1 °/s settle threshold is below the gyro jitter.
- Steady ~0.2 mN·m clockwise background torque with fans off (wheel drifted −30 → −315 RPM over 9 min).
- `I_BODY_Z` 0.014 predates the ±32 cm fan arms; the kick sizes suggest the real value is larger.
**Decision (team):** fix command 18 later, and run it with the reaction wheel OFF (removes the rail-sag kick). Needs: a way to settle between steps without the wheel (wait for the string), a heading-based fit (less noisy than rate), a fans-off baseline per step, and a re-measured I_BODY_Z.
**Changed:** nothing.

## 2026-11-02: Command 19 fan hold test

**Goal:** measure how long KestrelSAT keeps the ±10° pointing requirement, and when the wheel saturates, with one fan pair on (CW or CCW).
**Changed:**
- `main.cpp`: `sun_track(hold_trigger, fan_u_pct, fan_l_pct)`: command 13 calls it with the command 17 duties and its old timing (unchanged behaviour). Command 19 `fan_hold_test()` asks `cw`/`ccw` [+ duty, default 100%] and calls it with `hold_trigger = true`: the fans turn on once |gyro heading − sun heading| ≤ 10° for 10 s straight. Prints `[RESULT]` when pointing first leaves ±10° and when the wheel saturates, and a summary on every exit (time to lose pointing, % of fan-on time inside the band). Runs up to 20 min (`FAN_HOLD_DURATION_MS`), ends 10 s after saturation (as 13).
- `definitions.h`: `FAN_HOLD_BAND_DEG` 10, `FAN_HOLD_TIME_MS` 10000, `FAN_HOLD_DURATION_MS` 1200000.
**Agent verified (no hardware):** PlatformIO build succeeds.
**We must verify on hardware:**
- [ ] Command 19 `ccw`: fans start only after the LED has been green ~10 s; `[INFO] Disturbance fans ON` printed
- [ ] Expect a small clockwise kick when the fans start (5 V rail sag slows the wheel, see FanSweep_data001); the tracker should pull it back
- [ ] "Saturated" is the commanded wheel speed hitting ±400 RPM; with the rail sag the real wheel (wMeas) runs ~20 RPM slower, so check wMeas too
- [ ] Repeat with `cw`

## 2026-11-02: Graphs for the 2 Nov runs (fan hold results)

**Data:** `3. DATA/2. After PDR/Data for 02NOV2026`: SunTrack_data016–018 (command 13, fans off), 019 (command 19 CW 100%), 020 (command 19 CCW 100%), FanSweep_data001.
**Found:**
- 019 CW: pointing 100% within ±10° (worst 7.7°) until the wheel saturated 51.5 s after fans on; pointing lost at 55.1 s. Wheel −6.27 RPM/s = 2.30 mN·m.
- 020 CCW: 100% within ±10° (worst 3.7°), saturated 49.5 s, pointing lost 52.7 s. Wheel +6.86 RPM/s = 2.51 mN·m.
- Fans-off baselines: −0.20 to −0.34 mN·m (CW) background. Averaging CW/CCW instead gives +0.11 mN·m, so the per-pair fan torque is 2.0–2.8 mN·m depending on which background is right (repeat runs needed).
- The fans started with the wheel near 0 RPM, so only half the wheel's range (0 → ±400) was used.
- The command 18 sweep is >10× off (it suggested ~0.2 mN·m), confirming it measured the rail-sag kick, not the fans.
- Command 18 log quirk: each step's last ON row has the next step number and pin 0 (step counter advances before logging). The analysis scripts now skip those rows; firmware left as is (command 18 is being redesigned).
**Changed:** `analysis/fan_hold_report.py` (new): pointing / wheel / timeline graphs aligned to fan-on, fans-off baselines, sweep rail-sag check, per-run `plot_suntrack` overviews, `summary.csv` + `summary.md`. `plot_fansweep.py` skips the pin-0 ON rows.
**Output:** `3. DATA/2. After PDR/Graphs/02NOV2026/`.

## 2026-11-02: Firmware cleanup (removed unused commands)

**Asked:** cut the firmware down to what the requirements need. Team chose to remove commands 2, 3, 5, 6, 7, 8, 9, 11, 12, 98, 99.
**Changed:**
- `main.cpp`: removed `get_sat_rssi()` (2), LED toggle (3), `manual_set_RW_speed()` (5), `lab6_run_test()` (6), `lab7_run_test_A/B()` + `set_speed_test_B()` (7, 8), `stream_RWspeed()` (9), `sun_stream()` (11), `sun_sign_check()` (12), SD list/print (98, 99), and the unused Lab 6 globals (`sun_X`, `sun_Y`, `sun_direction`, `S_mag`, `n_sun_sensor_reads`). Menu now prints with `print_both()`. File header updated. Remaining commands keep their numbers: 0, 1, 4, 10, 13–19.
- `sd_funcitons.cpp` / `sd_functions.h`: removed `sd_listFiles()`, `sd_printFileMenu()`, `sd_printFile()` and an old commented-out function.
- main.cpp 2698 → ~1700 lines. Compiles (`pio run -e esp32thing_plus`).
**Note:** with 98/99 gone, data comes off by pulling the SD card. With 12 gone, signs are set only in definitions.h or with command 14. Sun Map still works from command 13 telemetry.
**Verify on KestrelSAT:**
- [ ] `1` prints the new menu over XBee and USB
- [ ] `0`, `4`, `10`, `13`, `19` behave as before

## 2026-11-02: Commands renumbered 0–10

**Asked:** put the remaining commands in order 0–10.
**Changed:** old → new: 0 → 0, 1 → 1, 4 → 2, 10 → 3, 13 → 4, 14 → 5, 15 → 6, 16 → 7, 17 → 8, 18 → 9, 19 → 10. Updated the `switch`, menu, and every "command N" comment in `main.cpp` and `definitions.h`, the analysis script docstrings, `README.md` (new command table, old → new list, test procedure without 11/12/98/99), and `CLAUDE.md`. Log entries above this one use the old numbers. Compiles.
**Verify on KestrelSAT:**
- [ ] `1` prints 0–10 in order; each number runs the command named in the menu
- [ ] GCS buttons/macros (if any) send the new numbers

## 2026-11-02: Fans out of the sun tracker (command 4)

**Asked:** command 4 should only use the reaction wheel; fans shouldn't run there.
**Changed:**
- `main.cpp`: `sun_track(fan_u_pct, fan_l_pct)` lost its `hold_trigger` argument. Command 4 calls it with 0, 0 (no fans, `track_duration_ms` length). Command 10 passes a duty, so the fans turn on after 10 s inside ±10° (same as before). The "fans on 10 s after the scan" path is gone.
- Command 8 is now a bench check only: duties start at 0 every time and are not kept for other commands. Removed the `fan_test_upper_pct` / `fan_test_lower_pct` globals.
- `definitions.h`: removed `FAN_START_DELAY_MS`. README, `fan_hold_report.py` and `fan_thrust_model.py` notes updated. Compiles.
**Verify on KestrelSAT:**
- [ ] Command 8 → `u 60` → `done`, then command 4: fans never turn on
- [ ] Command 10 `ccw`: fans still turn on after ~10 s green

## 2026-11-02: Command 8 (fan bench check) removed

**Asked:** remove command 8.
**Changed:** `main.cpp`: removed `fan_control()` and `start_fans()` (only command 8 used them) and the `fan_freq_hz` global (`setup()` now uses `FAN_PWM_FREQ_HZ` directly; the runtime `freq` option is gone). `set_fans()` stays for the sweep and hold test. Renumbered to keep 0–9 in order: 9 (fan torque sweep) → 8, 10 (fan hold test) → 9. Comments, analysis docstrings, README and CLAUDE.md updated. Compiles.
**Verify on KestrelSAT:**
- [ ] `1` prints 0–9; `9 ccw` runs the fan hold test and the fans still start after ~10 s green

## 2026-11-02: Fan torque sweep (command 8, old 18) removed

**Asked:** remove command 8 (fan torque sweep).
**Changed:** `main.cpp`: removed `fan_sweep()` and its `fit_slope()` helper. Fan hold test 9 → 8, so commands run 0–8. `definitions.h`: removed the `FAN_SWEEP_*` settings. `plot_fansweep.py` and `fan_hold_report.py` kept for the existing FanSweep_data001 (docstrings say the command is gone). README, CLAUDE.md updated. main.cpp is now 1358 lines. Compiles.
**Note:** the fan torque now has to come from the hold test (wheel speed slope after the fans turn on, `fan_thrust_model.py --csv`). A wheel-off sweep can be written fresh later if needed.
**Verify on KestrelSAT:**
- [ ] `1` prints 0–8; `8 ccw` runs the fan hold test

## 2026-11-02: Correction: fans were missing their 5 V supply, not half-driven

**Found (team):** the fans had not been connected to the 5 V rail. That, not the 3.3 V XY-MOS trigger, caused the 2.2 V across the fans in the "Fans didn't spin" entry above. They were connected before SunTrack_data019/020, so those runs are at full 5 V. The "weak 3.3 V XY-MOS drive" explanations in the entries above are wrong.
**Check:** the ducted-fan model at Colorado Springs air density (ρ = 0.98) predicts 2.51 mN·m per pair at 100%; measured 2.30 (019, CW) and 2.51 (020, CCW). Open-rotor theory (2×) is too high.
**Changed:**
- `CLAUDE.md` hardware line, `definitions.h` `FAN_PWM_INVERTED` comment, `main.cpp` pin-drive comment: no longer say the XY-MOS is only partly on.
- `analysis/fan_thrust_model.py`: ducted model only, ρ = 0.98, measured 019/020 points shown on the torque plot and printed next to the theory.
**Known issue (not changed):** `fan_thrust_model.py --csv` takes its fans-off baseline from the same run, which in a command 8 hold test is the post-scan settling (+4.5 to +5.0 mN·m), so its "fan disturbance torque" line is wrong for hold tests. Use the "fans on" line, or baselines from separate fans-off runs (016–018: −0.20 to −0.34 mN·m).
**Verify on KestrelSAT:**
- [ ] Fan OUT+ to OUT− ≈ 5 V at 100%

## 2026-11-02: fan_thrust_model.py baseline fixed

**Asked:** fix the fans-off baseline in `fan_thrust_model.py --csv`.
**Problem:** it took the background torque from the fans-off part of the same run. In a command 8 hold test that is the settling right after the scan (+4.5 to +5.0 mN·m), so run 019 came out −6.8 mN·m instead of about −2 mN·m.
**Changed (`2. THEORY/fan_thrust_model.py`):** the background now comes only from separate fans-off command 4 runs, passed with `--baseline` (same method as `fan_hold_report.py`: tracking samples ≥ 10 s after each scan, not saturated; runs without fan columns are read as fans off). Without `--baseline` it prints the fans-on torque and says no background was removed. Measured points on the plot updated to the background-corrected values.
**Result** (`--baseline` 016 017 018, mean background −0.267 mN·m): 019 CW −2.04 mN·m, 020 CCW +2.78 mN·m, mean 2.41 vs ducted theory 2.51. The 018 baseline uses only 8 s of tracking.

## 2026-11-02: Docs updated for the new folder layout

**Asked:** update the README and CLAUDE.md paths after the team reorganized the project (`2. THEORY/` Python, `3. CODE/` firmware + GCS, `4. DATA/` data; README, CLAUDE.md, AI_LOG.md, requirements.txt at the root).
**Changed:** `README.md` folder table rewritten (adds the fan scripts and the top-level folders), run commands use `"2. THEORY/..."` from the project root. `CLAUDE.md` gets a folders line and the new firmware and sim paths. `fan_hold_report.py` summary text names its new path. Entries above this one use the old `2. CODE/analysis/` and `sim/` paths.
**Also fixed:** `plot_fansweep.py` still imported `T_MAX_LOW`/`T_MAX_HIGH`, removed in the ducted-only change; it now plots the single ducted theory curve.
**Checked:** every `2. THEORY` script runs from the root (`--help`; sim `step90`; `plot_suntrack` on 019; `plot_fansweep` on a copy of FanSweep_data001). Firmware builds from `3. CODE/KestrelSAT_SunTracker`.

## 2026-11-02: Data folder reorganized

**Asked:** organize `4. DATA`.
**Changed:** Lab 6/7 data → `0. Labs 6-7/` (the after-PDR `Lab7_testA_data001.csv` renamed `Lab7_testA_data001 (after PDR).csv`, it clashed). Before-PDR runs → `1. Before PDR (Oct)/csv/` and `plots/` (empty 041/043 plot folders removed). After-PDR runs by test day: `2. After PDR/2026-10-30/` and `2026-11-02/`, each with `csv/` and `graphs/`. New `4. DATA/README.md` indexes the runs and warns that `SunTrack_data001–015` exist twice (numbering restarted after the PDR). No file names changed except that one; `SunTrack_data017_all 2.pdf` kept (differs from `_all.pdf`). Checked: all 745 files present with identical MD5s. Updated the paths in `README.md`, `fan_hold_report.py` usage, and the `analyze_suntrack.m` header. Entries above use the old `3. DATA/...` paths.

## 2026-11-02: main.cpp shortened (no behaviour change)

**Asked:** reduce the code (tier 1: same behaviour, less code).
**Changed (`main.cpp`, 1357 → 1215 lines):**
- `sun_track()`: the four "end the test" blocks (wheel off, fans off, close file, message, command 8 summary) are one `end_test()` helper. Messages unchanged.
- SD log row built as one `String` (same 24 columns, same decimals) instead of 48 `print` calls.
- `send_battery_telemetry()`: one `print_both()` line. USB now prints the same `BAT,V=...` line as XBee (was `[BAT] V=...`).
- `get_command_from_ground_station()`: reads the line once from whichever port has data, instead of two copies.
- Removed: the commented-out IMU sample-rate setup, the out-of-date `process_main_menu()` doc, and extra blank lines. File header shortened to purpose + command table.
**Kept on purpose:** `SPI_*` pins (pin map; ask before changing), `MOTOR_VOLTAGE` (used by `RPM_PER_PWM`), `SUN_LOST_TIME_MS` and `I_BODY_Z` (mirrored in the sim). Compiles; flash +0.6 KB from the String-built log row.
**Verify on KestrelSAT:**
- [ ] Command 4: CSV header and rows look the same as before (24 columns), 'X' stops the wheel and closes the file
- [ ] Command 2: one `BAT,V=...` line on both USB and XBee

## 2026-11-02: Firmware function guide

**Asked:** a .md in `3. CODE` explaining each function in KestrelSAT_SunTracker.
**Changed:** new `3. CODE/FUNCTIONS.md` (every function in `main.cpp`, `sun_tracker.cpp`, `sd_funcitons.cpp`, plus the command table and sun_track mode table), linked from `README.md`. While reading, fixed three messages the renumbering missed in `main.cpp`: calibration said "Run 10 again" (now 3), AD check said "14 -> gsign" (now 5), resistor sweep pointed at `analysis/plot_ptsweep.py` (now `2. THEORY/`). Compiles.

## 2026-11-02: Command 10 requirement demo

**Asked:** a command for the final presentation demo: send 10, then 1, 2 or 3 to show each requirement working.
**Requirements used:** the ones in `6. FINAL/Astro-nots Lab 8 Final Presentation.pptx` slide 5 (not the older README draft): R1 hold within 10°, R2 from 180° reach the sun within 10 s and stay within 10°, R3 hold within 10° with the disturbance fans on.
**Changed:**
- `main.cpp`: new `req_demo()` (command 10; 9 left unused because the team asked for 10). `sun_track()` takes an optional `demo_req` (default 0, so commands 4 and 8 are unchanged). With it set, it prints `[DEMO]` progress every 5 s and ends with PASS/FAIL. For R2 a `demo_offset` of 180° turns the array away (lobing paused), then 0° starts the timed slew back. Every demo exit goes through `end_test()` (wheel and fans off, file closed).
- `definitions.h`: `DEMO_*` settings (R1 30 s hold, R2 5 s settle / 3 s away / 10 s limit / 5 s confirm, R3 30 s with fans, 3 min give-up).
- README, `3. CODE/FUNCTIONS.md`, CLAUDE.md: command 10 added. Compiles.
**Note:** R3 in the presentation says 2 N·m; the fans make about 2.4 mN·m (`fan_thrust_model.py`).
**Verify on KestrelSAT:**
- [ ] `1` lists 10; `10` then `1`: progress lines, R1 PASS after ~30 s green; 'X' stops the wheel
- [ ] `10` then `2`: turns ~180° away and holds, then slews back; reach time printed; latch picks one direction (no back-and-forth)
- [ ] `10` then `3` (and `3 cw`): fans start after 10 s green, PASS/FAIL after 30 s, fans and wheel stop at the end
- [ ] After R2's slew back, no unexpected rescan (−X bright again when pointed at the sun)

## 2026-11-02: Final presentation draft

**Asked:** what the final presentation should contain, then a first draft of the slides.
**Used:** Lab 8 guidance (18–20 min, plots, live demo), PDR feedback (keep and compare pointing error plots; quantify settling time, error and wheel saturation; saturation → resistor → data; show code), and the 30 Oct / 2 Nov data.
**Changed:**
- `2. THEORY/final_figures.py` (new): `6. FINAL/figures/fig_pdr_vs_final.png` (PDR run 052 vs 30 Oct run 015), `fig_sim_vs_hw.png` (sim with run 015's start vs hardware), `run_summary.csv` (settle time, longest hold, % in ±10°/±5°, RMS for runs 014–017). Error = gyro heading − scanned sun heading, because the sun-sensor error reads ~0 when only −X is lit.
- `6. FINAL/Astro-nots Lab 8 Final Presentation DRAFT.pptx` (new, 19 slides; the original deck is unchanged). Kept the title, overview, application, requirements (status column rewritten) and the PDR run 052 slide; removed the guidance slide and the other PDR-only slides. Added system overview, control architecture, code walkthrough, what changed since PDR, resistor sweep, sim vs hardware, PDR vs final, results table, two disturbance slides, limitations, live demo (command 10), lessons/AI use, questions. Speaker notes on every slide.
**Results used:** runs 014/015 settle 14.5–15.1 s from control start (10 s is the scan), 100% within ±10° after settling, RMS 2.4–2.6°, ~95 s continuous hold. Runs 016/017 hold 52–54 s before a rescan. Fans: 2.0–2.8 mN·m, pointing held 52–55 s. Sim with the same start settles in 12.5 s.
**Not checked:** the slides were not looked at as images (PowerPoint's PDF export hung while the screen was locked). The file passes the package validator.
**Team must fill in or decide (marked [TEAM] in red on the slides):**
- [ ] One set of requirements: the deck (PDR) set or the README set
- [ ] "2 Nm" in the disturbance requirement (measured torque is ~2.4 mN·m)
- [ ] R2 with command 10 → 2 (slew time without the scan); battery stretch (command 2 during command 4) or drop it
- [ ] Demo backup video, GCS screenshot, repo link, one example of fixing AI output
- [ ] Open the deck in PowerPoint and check every slide for overlap/overflow

## 2026-11-03: Final requirements

**Asked:** how long the array takes to meet R2, then update the docs to the team's final requirements (R1 ±10° pointing, R2 180° to the sun in 10 s, R3 2 N·m disturbance, stretch: battery charge, wheel desaturation).
**Found:** no command 10 → 2 run on the SD card yet. After the scan, runs 014–020 reached ±10° in 1.6–2.0 s from 93–125° error.
**Changed:**
- `README.md`: "Draft requirements" replaced by "Requirements (final)" with a status column from the data (R1 pass, R2 pass from the post-scan slews, R3 fail as written but met at 2.0–2.8 mN·m, both stretch goals not tested). Sim table and notes no longer cite the old R1/R2/S1.
- `2. THEORY/plot_suntrack.py`, `final_figures.py`, `sim/sun_tracker_sim.py`: labels only. R1 is now pointing accuracy; settle time has no R number; `step180` is the R2 scenario. Firmware already matched the final set (command 10), so it is unchanged.
**Not checked:** `plot_suntrack.py` not run (matplotlib not installed on this Mac); all three scripts compile.
**Team must decide:**
- [ ] R1 pass line in the deck says ±5°; the data is 95–98% within ±5°, 100% within ±10°
- [ ] Run command 10 → `2` once for a true 180° slew time

## 2026-11-03: Command 11 momentum dump, command 10 reworked

**Asked:** a new command that shows momentum dumping: find the sun, face it, hold, let a fan pair saturate the wheel, then use the other pair as "thrusters" to desaturate it, to show how long pointing can be held. Also rework command 10 to match the final requirements.
**Changed:**
- `main.cpp`: new command 11 `momentum_dump_test()`. `sun_track()` takes an optional `desat` flag. With it, after the disturbance pair turns on (same 10 s on target as command 8), the other pair fires at 100% when |wheel command| ≥ 300 RPM and stops at ≤ 80 RPM; repeats until 'X' (max 20 min). `[DESAT]` lines per dump; the summary adds the dump count, the average dump length and the longest stretch inside ±10° (that longest stretch is now in the command 8 summary too). A dump over 3 min ends the test. New log column `dumping`, telemetry `dump:`.
- Command 10: added `4` (S1: average MAX17048 charge rate over 60 s while on the sun > 0 → PASS) and `5` (S2: command 11; PASS after one finished dump with pointing never leaving ±10° since the fans turned on, + 10 s). R1's PASS line also gives the % within ±5°. R3 text mentions 2.4 mN·m vs 2 N·m. New `parse_fan_choice()` for "cw 60"-style words.
- `definitions.h`: `DESAT_*` and `DEMO_S1_*` / `DEMO_S2_*`.
- README (commands 10/11, S1/S2 verification), `3. CODE/FUNCTIONS.md`, CLAUDE.md. Sim unchanged: no controller math changed (fan switching only). Compiles, no warnings.
**Why the disturbance defaults to 70%:** both pairs are the same fans, so 100% vs 100% cancels and the wheel never unloads. Fan model (thrust ~ duty²): 70% ≈ 0.9 mN·m, so ~2 min to reach 300 RPM and ~1 min to dump with the thrusters.
**Verify on KestrelSAT:**
- [ ] `1` lists 10 and 11. `11` then `ccw`: disturbance on after 10 s green, wheel climbs, `[DESAT] Dump 1` at 300 RPM with the GPIO 27 pair on, wheel falls to 80 RPM, pointing stays inside ±10°
- [ ] The thrusters really slow the wheel (if the wheel keeps climbing, the pairs or duty are wrong; lower the disturbance duty)
- [ ] 5 V rail with the wheel plus all four fans on (rail sag, brown-out)
- [ ] 'X' during a dump stops the wheel and all four fans
- [ ] `10` then `4`: charge-rate lines every 5 s, PASS/FAIL at 60 s (does the gauge's rate update that fast?)
- [ ] `10` then `5`: PASS after the first dump; how long the whole run takes

## 2026-11-03: Command 10 trimmed to R1/R2, R3, S2

**Asked:** drop S1 (team is not going for it); combine R1 and R2 into one demo; after the scan, turn 180° away before tracking.
**Changed:**
- `main.cpp`: command 10 is now `1` = R2 + R1 in one run, `2` = R3, `3` = S2. For `1`, when the scan finds the sun the array is sent 180° away right away (no settling on the sun first), sits there 3 s, then the timed slew back (R2), then a 30 s hold within ±10° (R1). One final line gives both results and the % within ±5°. S1 battery demo removed.
- `definitions.h`: removed `DEMO_R2_SETTLE_MS` and `DEMO_S1_*`.
- README (command 10 row, requirement table: S1 row removed, verification refs renumbered), `3. CODE/FUNCTIONS.md`. Compiles, no warnings.
**Verify on KestrelSAT:**
- [ ] `10` then `1`: after the purple scan the array turns 180° away (−X dark), holds ~3 s, slews back with one direction (latch), R2 time printed, then R1 PASS after 30 s
- [ ] No rescan while turned away or right after the slew back
- [ ] `10` then `2` / `3`: fan options read as `2 cw`, `3 ccw 60`
