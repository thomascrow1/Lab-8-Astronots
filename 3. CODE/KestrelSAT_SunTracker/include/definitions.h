/*---------------------------------------------------------------------------------------------*/
// Configuration:
/*---------------------------------------------------------------------------------------------*/
#define SD_CS_PIN 5  // Chip select pin for the microSD card on Thing Plus
// #define SD_CS_PIN 33  // Chip select pin for the microSD card on Feather Adalogger
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23

#define XBEE_RX 16  // ESP32 RX <- XBee DOUT
#define XBEE_TX 17  // ESP32 TX -> XBee DIN

#define SUN_SENSOR_PLUS_X_PIN   A0
#define SUN_SENSOR_MINUS_X_PIN  A2
#define SUN_SENSOR_PLUS_Y_PIN   A1   // briefly swapped with A3 on 30 Oct, reverted: command 6 step 4 showed the swap was wrong
#define SUN_SENSOR_MINUS_Y_PIN  A3

#define MOTOR_VOLTAGE           5.05
#define CPR                     64
#define MOTOR_PWM_1_PIN         33
#define MOTOR_PWM_2_PIN         15
#define ENCODER_PIN_A           14
#define ENCODER_PIN_B           32



/*---------------------------------------------------------------------------------------------*/
// Lab 8 Sun Tracker Configuration:
/*---------------------------------------------------------------------------------------------*/
// Pointing target: body-frame azimuth (deg) of the solar array normal.
// Sun sensor convention: +X = 0, +Y = 90, -X = 180, -Y = 270.
#define SOLAR_ARRAY_AXIS_DEG    180.0   // Solar array faces -X

// Physical properties (Lab 7 prelab Table 1 & 2):
#define I_BODY_Z                0.014   // KestrelSAT MOI about Z (kg*m^2)
#define I_WHEEL                 0.0035  // Reaction wheel MOI (kg*m^2)
#define RPM_PER_PWM             (1000.0 * MOTOR_VOLTAGE / 12.0) // wheel RPM at 100% PWM (same as Lab 7 w_RW_cmd)

// Default controller gains (tunable at runtime with command 5):
//   Kp = I_BODY_Z * wn^2, Kd = 2 * zeta * wn * I_BODY_Z  with wn = 3.0 rad/s, zeta = 1.3 (OVERDAMPED: no overshoot)
//   Sim (measured 1.1 s motor lag): settles to 5 deg in 30 deg ~1.4 s, 90 deg ~2.2 s, 170 deg ~3.2 s, 0 deg overshoot
#define DEFAULT_KP              0.126   // N*m per rad of pointing error
#define DEFAULT_KD              0.1092  // N*m per rad/s of body rate
#define DEFAULT_MAX_RATE_DPS    90.0    // slew rate cap: big turns run at <= this speed, then ease in
#define DEFAULT_BIAS_PWM        0.0     // wheel starts at 0 RPM: full -400..+400 RPM range available both ways
#define POINTING_DEADBAND_DEG   1.0     // inside this error, only rate damping is applied (was 3.0: it ate most of the lobe, SunTrack_data015)

// 180 deg direction latch (SunTrack_data008: at 180 deg, sensor noise flipped the error sign every
// other sample, so the push reversed ~10 times/s and KestrelSAT never turned toward the light):
#define LATCH_ENTER_DEG         150.0   // |error| above this: commit to one turn direction
#define LATCH_EXIT_DEG          120.0   // release once |error| stays below this...
#define LATCH_EXIT_TIME_MS      300     // ...for this long (so a 1-2 sample sensor spike can't release it)

// Scan-and-hold (30 Oct, SunTrack_data011: with only -X lit the sun direction reads 181 deg anywhere in its
// cone, so KestrelSAT rested ~20 deg off). Spin one full turn, find the heading where -X is brightest,
// then hold that heading with the gyro. Rescan if -X gets dim while pointed there.
#define SCAN_RATE_DPS           45.0    // scan turn rate (one turn ~8 s)
#define SCAN_TURN_DEG           420.0   // full turn + 30 deg each side, so the peak's bright stretch is never cut off
#define SCAN_TIMEOUT_MS         30000   // give up on a scan after this long (wheel can't turn the body?)
#define SCAN_PEAK_FRACTION      0.8     // sun heading = centre of the samples >= 80% of the peak
#define REACQ_FRACTION          0.3     // rescan (safety net) if -X brightness < 30% of the scan peak...
#define REACQ_TIME_MS           1000    // ...for this long...
#define REACQ_ON_TARGET_DEG     10.0    // ...while the gyro says we are pointed at the sun

// Sequential lobing (30 Oct, SunTrack_data014: gyro heading held, but -X dimmed 1.20 -> 0.55 over 100 s
// because the gyro bias measured by hand was off). While tracking, aim LOBE_AMP_DEG to one side, then the
// other; whichever side is brighter is where the sun is, so nudge the sun heading that way. Keeps drift
// (gyro bias, slowly moving light) corrected without a full rescan.
#define LOBE_AMP_DEG            3.0     // aim this far each side of the sun heading
#define LOBE_DWELL_MS           2000    // time on each side
#define LOBE_AVG_MS             1000    // brightness is averaged over the last part of each side (after settling)
#define LOBE_SETTLED_DEG        3.5     // only average while within this of the aim point...
#define LOBE_SETTLED_DPS        5.0     // ...and turning slower than this
#define LOBE_GAIN               0.7     // fraction of the estimated offset corrected each cycle
#define LOBE_MAX_STEP_DEG       5.0     // largest correction per cycle
#define LOBE_HALF_POWER_DEG     33.0    // -X sensor half-power angle (estimated from SunTrack_data008)

// Wheel limits:
#define WHEEL_MIN_RPM           -400.0  // wheel may spin both ways (crosses 0 RPM) for twice the range
#define WHEEL_MAX_RPM           400.0   // ~100% PWM on 5V supply
#define WHEEL_STALL_RPM         50.0    // measured 13 Oct: wheel stalls below ~45 RPM; smaller commands jump to +/- this
#define WHEEL_MAX_ACCEL_RPM_S   400.0   // was 50 (Lab 7 prelab limit); raised for fast slews
#define MOTOR_TAU_S             1.1     // measured 13 Oct (SunTrack_data026 spin-up): wheel speed time constant

// Sign conventions (can be changed at runtime with command 5):
#define DEFAULT_WHEEL_PWM_SIGN  1       // measured 13 Oct (SunTrack_data020): -1 drove KestrelSAT away from the light
#define DEFAULT_GYRO_SIGN       1       // +1 if gyrZ() is positive for +Z (counter-clockwise from above) rotation

// Sun sensor sampling: the lamp flickers at 120 Hz (8.33 ms), which made back-to-back reads jump
// +/-300 counts sample to sample (SunTrack_data015, same with the wheel off). Spreading the reads evenly
// over one full flicker period averages the flicker out.
#define SUN_READS_PER_SAMPLE    16      // ADC reads per channel per sample
#define SUN_READ_SPAN_US        8333    // spread them over one 120 Hz flicker period

// Sun detection:
#define SUN_LOST_THRESHOLD      0.15    // sum of calibrated channels below this = no sun
#define SUN_LOST_TIME_MS        500     // sun must be missing this long before tracker stops

// Tracker test timing (ms from start of command 4):
#define TRACK_SPINUP_MS         3000    // ramp wheel 0 -> bias (hold KestrelSAT still)
#define TRACK_RELEASE_MS        8000    // RGB LED turns blue: release KestrelSAT
#define TRACK_CONTROL_MS        10000   // closed-loop control begins
#define TRACK_DURATION_MS       120000  // test ends automatically after this (or send 'X'); change at runtime with 'dur' (command 5)



/*---------------------------------------------------------------------------------------------*/
// Disturbance Fans (Pi-Fan LD3007MS, 5 V 0.2 A, 2-wire brushless) - command 8:
/*---------------------------------------------------------------------------------------------*/
// Two fans per pin, each pair switched by an XY-MOS (D4184) module. Fans sit at y = +/-32 cm, 2 cm above
// or below centre. Wired DIAGONALLY (2 Nov) so each pin twists KestrelSAT about Z: one fan at +Y and one at -Y,
// blowing opposite ways. (Same-direction pairs cancel about Z at any duty.) The "UPPER"/"LOWER" names are
// only the old labels; in command 8 'cw' = GPIO 27 pair, 'ccw' = GPIO 4 pair.
#define FAN_UPPER_PIN           27      // diagonal pair 'u': twists KestrelSAT CLOCKWISE seen from above (-Z)
#define FAN_LOWER_PIN           4       // diagonal pair 'l': twists KestrelSAT COUNTER-CLOCKWISE from above (+Z)
#define FAN_UPPER_LEDC_CH       0       // LEDC channels 0 and 2 (timers 0 and 1); analogWrite() for the
#define FAN_LOWER_LEDC_CH       2       //   motor driver takes channels 15 and 14 (timer 3), so no clash
#define FAN_PWM_BITS            10      // duty resolution (0-1023)
#define FAN_PWM_INVERTED        0       // 0: GPIO straight to the XY-MOS trigger (what we have). 1: only if an inverting
                                        //   NPN stage is ever added in front of the trigger (GPIO high = fans OFF).
                                        //   The 2.2 V once seen across the fans at 100% was their 5 V supply not being connected,
                                        //   not the 3.3 V trigger; with 5 V connected the fans get the full rail.
#define FAN_PWM_FREQ_HZ         25      // low-frequency PWM suits 2-wire fans; try 20000 if
                                        //   speed control is poor (the XY-MOS is rated 0-20 kHz)
#define FAN_KICK_MS             500     // run at 100% this long when starting from rest, so low duties still spin up
#define FAN_SAT_STOP_MS         10000   // command 8: with fans on, end the test after the wheel is saturated this long

// Command 8 fan hold test: command 4, but the fans turn on once KestrelSAT has been inside the pointing band
// for FAN_HOLD_TIME_MS. Reports how long pointing stays inside the band and when the wheel saturates.
#define FAN_HOLD_BAND_DEG       10.0    // pointing requirement (deg, gyro heading vs sun heading)
#define FAN_HOLD_TIME_MS        10000   // fans turn on after this long inside the band
#define FAN_HOLD_DURATION_MS    1200000 // command 8 runs up to 20 min (the weak fans may take minutes to saturate)

// Command 10 requirement demo (final presentation): command 4, judged live against one requirement,
// prints PASS/FAIL and ends. Uses the same +/-FAN_HOLD_BAND_DEG band (gyro heading vs sun heading).
#define DEMO_DURATION_MS        180000  // R1/R2: give up (FAIL) after this (R3 and S2 use FAN_HOLD_DURATION_MS, 20 min)
#define DEMO_PRINT_MS           5000    // progress line this often
#define DEMO_R1_HOLD_MS         30000   // R1: after the R2 slew, PASS after this long inside the band in a row
#define DEMO_R2_AWAY_MS         3000    // R1/R2: after the scan, held 180 deg from the sun this long before the timed slew back
#define DEMO_R2_LIMIT_MS        10000   // R2: must reach the band within this
#define DEMO_R2_CONFIRM_MS      5000    // R2: ...and then stay inside it this long
#define DEMO_R2_TIMEOUT_MS      30000   // R2: FAIL if not settled this long after the slew starts
#define DEMO_R3_FAN_MS          30000   // R3: PASS after the fans have been on this long without leaving the band
                                        //   (then R3 and S2 keep running until pointing is lost, for the graphs)
#define DEMO_LOST_STOP_MS       10000   // R3 / S2: end the run once pointing has been outside the band this long
#define DEMO_R3_DEFAULT_DUTY    100.0   // R3 fan duty (%) if none is given
#define DEMO_S2_CONFIRM_MS      10000   // S2: after the first momentum dump ends, stay in the band this long -> PASS



/*---------------------------------------------------------------------------------------------*/
// Command 11 Momentum Dump (and command 10 -> 3): pretend thrusters unload the reaction wheel
/*---------------------------------------------------------------------------------------------*/
// Command 4 with one fan pair as a constant disturbance; the wheel spins up to fight it. When the wheel
// gets near saturation, the OTHER pair fires as a "thruster" against the disturbance, so the controller
// spins the wheel back down while it keeps pointing. Thrusters stop once the wheel is slow again; repeat.
// Both pairs are the same fans: a 100% disturbance and a 100% thruster cancel, and the wheel never unloads.
// So the disturbance runs below 100% (fan thrust ~ duty^2: 70% gives ~40% of full thrust, ~0.9 mN*m).
#define DESAT_DIST_DEFAULT_PCT  70.0    // disturbance pair duty (%) if none is given
#define DESAT_THRUSTER_PCT      100.0   // "thruster" pair duty (%) while dumping
#define DESAT_START_RPM         300.0   // fire the thrusters when |wheel command| reaches this (75% of 400 RPM)
#define DESAT_STOP_RPM          80.0    // stop them once |wheel command| is back below this (above the 50 RPM stall)
#define DESAT_MAX_MS            180000  // a dump longer than this is not working (wrong pair, or too weak): end test

