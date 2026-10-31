# KestrelSAT Sun Tracker (Astro 331 Lab 8)

Closed-loop sun pointing about KestrelSAT's Z axis. The Lab 6 four-phototransistor sun sensor
measures where the sun is. The ICM-20948 gyro measures body rate. The Lab 7 reaction wheel turns
the satellite until the **solar array (−X face, 180°)** points at the light.

```
 sun sensor ──► sun_direction ──► error = wrap(sun_dir − 180°) ─┐
                                                                 ├─► PD: τ = Kp·e − Kd·ω ─► wheel accel = −τ / I_w
 gyro Z ─────────────────────────────────► body rate ω ─────────┘                          │
                                                                                             ▼
                                                    wheel speed cmd (bias ≈ 210 RPM, 40–400 RPM) ─► PWM ─► TB9051FTG
```

## Folder layout

| Path | What it is |
|---|---|
| `KestrelSAT_SunTracker/` | PlatformIO firmware (Lab 7 code + Lab 8 commands 10–14). Open **this folder** in VS Code / PlatformIO. |
| `KestrelSAT_SunTracker/src/sun_tracker.cpp` | Hardware-free controller math (sun direction, error, PD step). |
| `KestrelSAT_SunTracker/include/definitions.h` | Pins plus all sun tracker settings (gains, limits, timing, signs). |
| `sim/sun_tracker_sim.py` | 1-axis body + wheel + sun sensor simulation. Uses the same controller as the firmware. |
| `analysis/plot_suntrack.py` | Plots a `SunTrack_dataNNN.csv` from the SD card and scores it against R1/R2. |
| `analysis/plot_adcheck.py` | Plots an `ADCheck_dataNNN.csv` (command 15): measured vs expected sun direction. |
| `analysis/plot_ptsweep.py` | Plots `PTSweep_dataNNN.csv` (command 16): brightness, signal span and photocurrent vs resistor; recommends one. |
| `CLAUDE.md` | Project rules for coding agents. |
| `AI_LOG.md` | GenAI use log required by the Lab 8 guidance. |

## Draft requirements

| ID | Requirement | Verification |
|---|---|---|
| R1 | Starting from a sun error of ≤ 90°, KestrelSAT shall point the solar array normal within ±10° of the light source within 15 s of control start. | Test: command 13, `plot_suntrack.py` "R1 time to settle" |
| R2 | KestrelSAT shall hold the solar array within ±10° of a fixed light source for ≥ 60 s. | Test: command 13, "R2 longest time within ±10°" |
| R3 | KestrelSAT shall log pointing error, body rate, and wheel speed to SD at 20 Hz and send them over XBee at ≥ 2 Hz. | Demonstration: GCS plot + SD file |
| S1 (stretch) | KestrelSAT shall keep the array within ±10° of a light moving at 5°/s. | Test: command 13 with a moving light |

## Ground commands (new in Lab 8)

| Cmd | Function |
|---|---|
| 10 | **Calibrate sun sensor.** Captures dark counts, then bright counts for each channel. Prints a line to paste into `main.cpp`. |
| 11 | **Stream sun sensor.** Shows sun direction, error, magnitude, gyro and calibrated channels (wheel off). Send `X` to stop. |
| 12 | **Sign check.** Pulses the wheel +30% for 1 s, then sets `gyro_sign` and `wheel_pwm_sign` from how the body turned. |
| 13 | **Run sun tracker.** Spin-up (hold the satellite) → LED **blue** = release → closed-loop tracking. Logs `SunTrack_dataNNN.csv`. Send `X` to stop. |
| 14 | **Print/set parameters.** Send e.g. `kp 0.004`, `kd 0.01`, `bias 0.5`, `axis 180`, `wsign -1`, `gsign 1`, then `done`. |
| 15 | **Attitude determination check** (wheel off). Light off → 8 light spots every 45° → gyro still → turn KestrelSAT ~90° by hand. PASS/FAIL per step (±15°). Logs `ADCheck_dataNNN.csv`; plot with `python analysis/plot_adcheck.py`. Run 10 first if the sun sensor wiring changed. |
| 16 | **Phototransistor resistor sweep.** Send a resistor value (`4700`, `10k`), then light off + light on each face. Repeat per resistor, then `done`. Logs `PTSweep_dataNNN.csv`; plot with `python analysis/plot_ptsweep.py`, which picks the resistor with the most signal and no saturation. Keep the light at the same distance every time. |
| 0 | Stop the reaction wheel (unchanged). |

**RGB LED during command 13:** yellow = hold still, blue = let go, orange = slewing, green = within ±10°, red = sun lost (wheel stopped).

## Hardware test procedure

1. **Build and upload** `KestrelSAT_SunTracker` with PlatformIO. The LED should turn green.
2. **Sun sensor checkout (USB, on the bench).** Run `10` and follow the prompts. Then run `11` and shine the light on each face. Expect +X ≈ 0°, +Y ≈ 90°, −X ≈ 180° (err ≈ 0), −Y ≈ 270°. Paste the printed `sun_cal` line into `main.cpp` and re-upload.
3. **Sign check (hanging, XBee).** Place the light about 45° off the −X face and let KestrelSAT settle. Run `12`. Copy the reported signs into `DEFAULT_GYRO_SIGN` / `DEFAULT_WHEEL_PWM_SIGN` in `definitions.h`.
4. **Tracking runs (hanging, XBee).** Start with the light about 90° from the −X face. Run `13`, hold KestrelSAT until the LED turns blue, then release smoothly. Repeat with 180°, and with a light moving slowly.
5. **Analyze.** Pull the CSVs off the SD card (command `98`/`99`, or a card reader) and run:
   ```
   python analysis/plot_suntrack.py SunTrack_data001.csv
   ```
6. **Tune** with command `14` (no reflash). Copy good gains into `definitions.h`.

If the satellite turns *away* from the light, the wheel sign is wrong. Send `14` → `wsign` with the opposite value, or rerun `12`.

## Simulation

```
pip install -r requirements.txt
python sim/sun_tracker_sim.py                 # all scenarios → sim/output/*.png
python sim/sun_tracker_sim.py --kp 0.005 --kd 0.012 --scenario step90 --show
```

Results with default gains (Kp = 0.0035, Kd = 0.0098, motor τ = 0.4 s):

| Scenario | Settle (±10°) | Notes |
|---|---|---|
| step90 | ~4 s | meets R1 |
| step170 | ~5 s | sun nearly behind the array |
| hold | 0 s, 100% in band | meets R2 |
| moving (5°/s) | never | PD lags a moving target by about (Kd/Kp)·rate ≈ 14°. S1 needs a change, for example damping on the error rate instead of the body rate, or an integral term. |
| string torsion | never | String twist acts like a spring, which gives a steady offset of about 13° with k = 0.0005 N·m/rad. Measure the string's twist period on hardware: k = I·(2π/T)². |
| sunlost | stops at 30.5 s | wheel stopped and reported, as designed |

## Known limitations and things to measure

- **Sequential lobing (fine pointing while tracking, 30 Oct).** After the scan, the gyro alone drifted ~20–30° in 100 s (SunTrack_data014, bias measured by hand). Now, while tracking, KestrelSAT aims 4° to one side for 2.5 s, then 4° to the other, compares −X brightness, and nudges the sun heading toward the brighter side (`LOBE_*` in `definitions.h`). It keeps seeking the brightest pointing without a rescan; rescan only below 30% of the peak. Sim: a 0.25°/s gyro error gives max 8° (was 24°). It can follow drift up to ~1°/s, so a 5°/s moving light still fails.

- **Scan-and-hold (command 13, 30 Oct).** With the sensors mid-face, only −X sees the light near the target, so the sun direction reads 181° anywhere in its cone (SunTrack_data011 rested ~20° off). Command 13 now turns one full circle (420°, ~9 s at 45°/s, LED purple), takes the heading where −X is brightest as the sun, and then holds that heading with the gyro. If −X drops below half its scan peak for 1 s while pointed there, it scans again. If a scan sees no sun, the test stops. Settings are `SCAN_*` and `REACQ_*` in `definitions.h`. Sim: settles in 11–12 s (R1 ≤ 15 s) and holds within ~2°; a moving light (S1) now fails because each move triggers a rescan.

- **Starting with the sun exactly behind the array (180°).** Turning left and turning right are equally short, so sensor noise picked the sign of the error each sample and the push reversed ~10 times/s (SunTrack_data008). A direction latch (`latch_far_error()`, `LATCH_*` in `definitions.h`) now commits to one direction once |error| > 150° and releases below 120°. The SD log has `err_ctrl(deg)` (error after the latch) and `latch` (0, +1, −1).
- **−X sensor noise with the wheel running.** In SunTrack_data008, −X jumped to 500–4095 counts while +X was lit, only once the wheel ran. Check the −X wiring and ground, keep sensor wires away from the motor leads, or add 0.1 µF from A2 to GND.

- **Signs are unverified on hardware.** Lab 6/7 code hints at sign flips (`w_RW_cmd = −pwm…`). Command 12 exists for this.
- **Wheel speed is open-loop PWM.** The outer PD loop corrects for it. The motor time constant in the sim (0.4 s) should be measured from Lab 7 Test A data.
- **Stopping the wheel when the sun is lost** dumps the wheel's momentum into the body, so KestrelSAT will start spinning. This was the chosen behavior. A gentler option is to ramp the wheel down.
- **The prelab MOIs** (I_body = 0.014, I_wheel = 0.0035 kg·m²) probably don't match the hanging satellite. Estimate the real I_body from Lab 7 Test B, as the lab report asks.
