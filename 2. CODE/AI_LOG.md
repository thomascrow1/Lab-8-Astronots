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
