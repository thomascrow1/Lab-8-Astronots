# Fan-assisted reaction-wheel unloading (draft hardware configuration)

The existing command **13** now includes a fan unloading supervisor. The wheel
continues controlling pointing while a pair of fans removes angular momentum.
**Fan actuation is disabled by default.** Proposed pins are gated off, airflow
and encoder signs are unconfirmed, and fan response settings remain zero until calibrated. This is a configurable
implementation, not a claim that the pictured hardware has been commissioned.

## Confirmed geometry and remaining assumptions

- This is a hanging laboratory test in air, with pointing and wheel rotation
  about body **Z**. These cooling fans cannot provide aerodynamic thrust in vacuum.
- The user confirmed 32 cm is the **radius**: fan centers are Y = +/-0.32 m.
  Opposing thrust lines are **64 cm** apart.
- There is one fan at Z = +2 cm and one at Z = -2 cm on **each** Y face, X = 0.
- Fans can be physically oriented for opposing tangential thrust along X.
- The user confirmed the top two fans share one pair of wires, and the bottom
  two share the other pair. Each bank needs one independent driver. Confirm that
  the two fans within each bank blow in opposing X directions with equal thrust.
  Blade rotation alone is insufficient to establish airflow direction.
- The existing +/-400 RPM wheel limits and inertia estimates are retained.
  Provisional unload settings: start at measured |RPM| >=200, stop at <=100,
  target zero. Also start if command reaches >=300 RPM and measured speed is
  >=150 RPM in the same physical direction. Near target, start within 5 degrees
  and rate <=3 deg/s; continue within 10 degrees. The user permits unloading
  during slews: outside 10 degrees the pointing/rate gates are relaxed.
  The 0.5 s fan thrust time constant and 60 s continuous-run limit are provisional.

The photograph does not establish fan orientation, control wiring, or an axis
definition. No manufacturer thrust curve or verified electrical datasheet for
this exact fan has been established. Model number, airflow rating, and electrical
power do not determine installed static thrust.

## Mounting and torque

For force **on the satellite**, `tau_Z = x*F_Y - y*F_X`. A fan at X = 0 blowing
straight outward along Y creates **zero Z torque**. Its 2 cm Z offset creates
roll torque instead. A Z offset does not provide a yaw lever arm for radial flow.

One suitable arrangement, subject to mechanical confirmation:

| Bank | Fan center (X,Y,Z), meters | Force on satellite | Air blown toward |
|---|---|---|---|
| +Z | (0, +0.32, +0.02) | -X | +X |
| +Z | (0, -0.32, +0.02) | +X | -X |
| -Z | (0, +0.32, -0.02) | +X | -X |
| -Z | (0, -0.32, -0.02) | -X | +X |

This table is **one possible airflow arrangement**, not a determination of the
supplied fans' directions. If the top bank makes -Z torque, change
`FAN_TOP_TORQUE_SIGN`; no control-input rewiring is necessary.

Equal thrusts in each same-height pair cancel net force and pitch torque while
adding yaw torque. A diagonal pair across opposite heights adds pitch torque.
Fan mismatch, blocked intakes, and interaction with the chassis can spoil that
cancellation; this one-axis controller does not correct roll/pitch.

With each fan's installed thrust equal to F newtons, the bank torque magnitude
is **0.64 F N*m** for the confirmed radius. Use the actual perpendicular distance
between thrust lines, not just the chassis width.

## What the code does

`include/fan_desaturation.h` and `src/fan_desaturation.cpp` contain hardware-free
bank selection, speed hysteresis, pointing/rate gates, timeout, and a first-order
thrust estimate. The estimate continues through fan coast-down. At most one
bank is electrically commanded on. The fans use full-voltage ON/OFF control;
the code makes no assumption that these brushless fans accept supply PWM or
reverse when their electrical polarity changes.

A positive measured wheel speed selects the **negative-Z** torque bank. Negative
speed selects the positive-Z bank. Actual momentum direction always comes from
the encoder. A near-limit command can trigger earlier when sufficient measured
momentum exists in the same direction. Command **12** also calculates
`fan_encoder_sign` from its wheel pulse.
Verify that manually identified +Z wheel rotation reads positive before use.
The older `w_RW_meas(RPM)` field retains its raw encoder sign for compatibility;
the new `w_RW_physical(RPM)` is signed physically and is `nan` until calibrated.

The wheel command uses:

```
desired_body_torque = existing pointing PD controller
wheel_acceleration = (estimated_fan_torque - desired_body_torque) / I_wheel
```

Thus the wheel's reaction counters the fan body torque while wheel momentum
decreases. The combined request is then limited by the existing wheel
acceleration and speed limits. Actual motor torque, lag compensation, and the
existing 50 RPM stall jump can still cause pointing motion. Fan torque is a
calibrated model quantity, not a thrust measurement during the run. With RPM/s
calibration, it is an **equivalent torque based on configured inertia**, not a
measurement of actual physical torque.

Fans stay off during spinup/hold and switch off when the sun is lost,
near-target pointing/rate gates fail, feedback fails, the run ends, or `X` is received.
A timeout or control interval longer than 0.2 s latches fan unloading off for
that test. A new command 13 resets the latch. Electrical OFF does not stop the
fan's mechanical coasting immediately. Existing end-of-test wheel-stop behavior
is retained and can transfer wheel momentum back to the satellite.

Unloading is allowed during slews, with the pointing/rate gates relaxed outside
10 degrees. Within 10 degrees, fans pause if body rate exceeds 3 deg/s; a new
unloading cycle starts within 5 degrees. This is intended to protect the
on-target +/-10 degree requirement while allowing slews to unload. It is not a
hard accuracy guarantee: torque calibration and actuator limits still apply.
A continuously
frozen encoder returning plausible zero cannot be detected from count data
alone; independent encoder and direction checks remain necessary.

The SD log appends `fan_bank`, `fan_torque_est(N*m)`, `fan_fault`, and
`w_RW_physical(RPM)`. Telemetry adds `fan`, `fanTau`, and `fanFault`.
The torque field remains named for compatibility; in RPM/s mode it reports the
equivalent model torque described above.

## Calibration without a thrust sensor

Absolute thrust data in newtons is **not required**. The preferred configuration
now accepts each full-ON bank's effect on wheel speed, in **RPM/s**. A relative
top/bottom ratio alone does not establish this scale. Measure each bank separately:

1. Use a fixed light source and a freely hanging satellite under closed-loop
   pointing. Do not hold it by hand: the hand would absorb the external torque.
   Keep wheel speed away from the stall zone and speed/command limits. Resolve
   the previously observed reverse-drive and timing problems first.
2. Record a settled fan-OFF window, a one-bank full-ON window, and a second OFF
   window after the fans coast down and pointing settles. Use a controlled
   manual or commissioning-test switch of one bank; automatic unloading remains
   disabled until calibrated. Record the actual switch times. This firmware does
   not currently automate that calibration pulse sequence.
3. Exclude fan startup, motor lag, coast-down, and body settling from the fit.
   Fit measured **signed physical wheel RPM** against seconds in each window.
   Compare OFF baselines before and after; they should be similar. If the body
   accelerates, the pointing changes materially, or the OFF slopes disagree,
   repeat with comparable conditions. A bare RPM difference is not enough;
   divide by elapsed time or fit a slope.
4. Compute the signed bank effect:

   ```
   bank_effect_RPM_s = slope_ON - average(slope_OFF_before, slope_OFF_after)
   ```

   This assumes body angular acceleration is near zero/comparable and external
   torque (especially the suspension string) stays comparable. Use the same
   supply voltage and installed geometry as the actual experiment.
5. A +Z bank should give a positive slope difference; a -Z bank should give a
   negative difference. Put the **magnitudes** into `FAN_POSITIVE_EQUIV_RPM_S`
   and `FAN_NEGATIVE_EQUIV_RPM_S`, with `FAN_CALIBRATION_IN_RPM_S=true`.
   Determine the top/bottom torque mapping separately; do not guess it from fan
   blade rotation. Verify the response repeats in each relevant wheel direction.

For example, if an OFF slope is +1 RPM/s and a bank's ON slope is -4 RPM/s,
the bank effect is -5 RPM/s: enter **5** for the negative torque bank. This is
an illustrative calculation, not a measurement of the supplied fans.

The existing torque-based controller uses the conversion:

```
equivalent_fan_torque = configured_I_wheel * bank_effect_RPM_s / 9.54929659
```

The pointing controller divides by the **same** configured wheel inertia and
converts back to RPM/s, so the inertia cancels from this fan feedforward term.
An accurate wheel inertia is therefore unnecessary for this calibration method.
The existing pointing gains, physical performance predictions, and any actual
Nm thrust estimate still depend on the physical dynamics. Fan spin-up/coast-down
must also be characterized from the response; 0.5 s remains provisional.

Alternatively set `FAN_CALIBRATION_IN_RPM_S=false` and enter independently
measured bank torque magnitudes in the existing `FAN_*_TORQUE_NM` settings.

## Wiring and calibration before enabling

The supplied close-ups identify the existing switches as **XY-MOS** modules.
Their boards are labeled `VIN+`, `VIN-`, `OUT+`, `OUT-`, `GND`, and `TRIG/PWM`.
Use one module per fan bank. The two power transistors on one board form one
switching channel, not two independently controlled fan outputs. The photos
show power/load wires, but do not let us verify completed GPIO-to-TRIG wiring,
the complete common-ground path, or each breadboard contact.

**What TRIG/PWM means:** this is the small **control-signal input**, not fan
power. TRIG means trigger. The software sends HIGH (about 3.3 V on this ESP32)
to switch that bank ON, and LOW (0 V) to switch it OFF. PWM means pulse-width
modulation: rapid ON/OFF pulses can control average power. This implementation
uses plain ON/OFF, so no PWM frequency or fan speed setting is needed. Switching
capability on the module does not prove the two-wire fans tolerate supply PWM.
The signal `GND` is the shared 0 V reference. Use the small pads explicitly
labeled `GND` and `TRIG/PWM`; connect them with a sound soldered wire/header.

Proposed connections for the **ESP32 Thing Plus C pin map in this project**:

| Connection | Top fan pair | Bottom fan pair |
|---|---|---|
| XY-MOS `TRIG/PWM` | **GPIO27** | **GPIO4** |
| XY-MOS `VIN+` | Regulated +5 V | Regulated +5 V |
| XY-MOS `VIN-` | Fan-supply negative | Fan-supply negative |
| XY-MOS `OUT+` | Both top fans' positive leads | Both bottom fans' positive leads |
| XY-MOS `OUT-` | Both top fans' negative leads | Both bottom fans' negative leads |
| XY-MOS signal `GND` | ESP32 GND and supply common ground | Same common ground |

The photographed modules appear to be the common dual-MOSFET trigger-switch
family; [a seller's specification](https://circuit.rocks/products/switch-drive-high-power-mosfet-trigger-module)
lists 3.3 V logic triggering and 5 V load-supply operation for that family.
This is not proof of the exact unbranded boards' ratings. Verify reliable OFF
at 0 V and ON at 3.3 V with your fan load; do not infer their protection features
or current rating from the earlier Adafruit example. XY-MOS is a switch, not a
5 V regulator. If the power source is a battery pack, regulate and verify 5 V.

Connect the two fans of each bank **in parallel**, so each receives 5 V. Keep
their factory supply polarity. Each pair's negative leads go to its driver's
switched output, not directly to common ground. Add a 10 kOhm resistor from each
driver `TRIG/PWM` to GND for an explicit default-OFF input. A GPIO supplies only the
control signal. Use a regulated 5 V fan supply sized from the actual current
labels and startup measurement; do not use the ESP32's 3V3 output or an
unregulated battery as the 5 V fan supply.

The [SparkFun pinout](https://docs.sparkfun.com/SparkFun_Thing_Plus_ESP32_WROOM_C/board_files/graphical_datasheet.pdf)
shows GPIO27 and a GPIO4 `FREE` connection. **GPIO4 is not the header label
`04`**: that label is GPIO14, already used by the encoder. Confirm the actual
board and accessible pad/header from a close-up before connecting. These proposed
GPIOs are in `FAN_TOP_BANK_PIN` and `FAN_BOTTOM_BANK_PIN`; the code does not touch
them while `FAN_WIRING_CONFIRMED` is false.

1. Confirm the controller board. The selected PlatformIO board ID is
   `sparkfun_esp32s2_thing_plus_c`, but its installed board definition actually
   specifies **ESP32 / SparkFun ESP32 Thing Plus C**. The lab folder also contains
   Pico reference material. Confirm the physical board before assigning GPIOs.
   No board or existing pin assignment was changed.
2. Confirm the fan current label, driver choice, 5 V supply, and the proposed
   GPIO connections from close-up photographs. The fans have two wires each;
   no tachometer or separate PWM input is assumed. Verify startup current and
   the default-OFF behavior with the fans mounted securely.
3. Confirm force directions and equal-height pairing with the wheel stopped.
   Positive and negative bank labels describe satellite torque, not fan rotation
   or the airflow arrow. Identify the physical +Z convention.
4. Calibrate each bank in RPM/s as described above; no thrust sensor is needed.
   Independently measured torque is an optional alternative. Characterize rise
   and decay too. If they differ substantially, extend the first-order model
   before relying on its compensation during switching.
5. In `include/definitions.h`, confirm `FAN_TOP_BANK_PIN` / `FAN_BOTTOM_BANK_PIN`
   and set `FAN_WIRING_CONFIRMED` true. Set `FAN_TOP_TORQUE_SIGN` to +1 if the
   top bank makes +Z torque, or -1 if it makes -Z; the bottom must make the
   opposite torque. Confirm the driver's `FAN_ACTIVE_HIGH`, verified
   `FAN_ENCODER_SIGN`, calibrated positive/negative RPM/s magnitudes (or the
   selected torque mode), measured `FAN_TORQUE_TAU_S`, and agreed limits.
   Set `FAN_DESATURATION_ENABLED` true only after these are confirmed. The code
   checks output capability, duplicate/conflicting pins, basic configuration,
   and reserves half the configured wheel acceleration capacity for pointing.
   Those checks do not establish the real motor's available torque or verify
   every board-specific reserved pin.
6. Build and upload, verify startup leaves drivers off, run commands 10-12 as
   needed, then test command 13 in both wheel directions. Check `fan_ready=1`
   with command 14. Verify the bank produces the intended torque, measured wheel
   speed decreases, pointing stays acceptable, and `X`, sun loss, timeout and
   normal completion remove fan power. Monitor the new SD fields.

The fans must exceed the sustained disturbance torque to unload against it.
For an illustrative **synthetic** bank torque of 0.001 N*m and the retained
I_wheel = 0.0035 kg*m^2, reducing 200 to 100 RPM takes about **37 s**, ignoring
disturbances and transients. Smaller torque can require several minutes and
exceed the provisional timeout or existing 120 s tracker duration. This example
is not a specification for LD3007MS fans.

## What the recorded wheel data establishes

The firmware command limit is +/-400 RPM; that is not a measured physical speed
limit. Older runs 045-046 show full commands producing roughly +/-350 measured
RPM. The latest
[run 052](<../5. DATA/File archive/Lab 8 Data/SunTrack_data052.csv>) commands
|RPM| >=399 for about 56.75 s. A sustained +400 plateau has a steady median
226.9 measured RPM, while a sustained -400 plateau measures essentially zero.
Runs 050-051 also show near-zero reverse rotation. Check reverse drive, wiring,
power and encoder feedback before fan commissioning; an encoder sign flip does
not explain this change from earlier bidirectional operation.

This is why the provisional trigger uses both actual speed and a near-limit
command. It refuses a command-only start against a stationary wheel. The latest
run also contains seven roughly 1.02 s control gaps. The 0.2 s timing guard
will disable fan unloading in a run with those gaps. Diagnose their source
(sensor reads, storage, or communication) before relying on sustained unloading.
The guard runs when control execution resumes; it cannot interrupt a blocked
hardware operation. Independent watchdog hardware is needed for a strict
maximum fan-ON time under a hung processor.

## Software checks

Run the headless checks from `3. LAB 8 CODE`:

```
python -m unittest discover -s sim -p "test_fan*.py" -v
```

Compile firmware from `KestrelSAT_SunTracker` with `pio run`. Simulation and
compilation cannot confirm fan authority, wiring, sign, or real pointing accuracy.
The current PlatformIO build passes; 22 headless tests pass. One optional host
C++/Python runtime comparison is skipped because this PC has no host C++ compiler.

The external-torque requirement for momentum unloading is described in
[NASA's Guidance, Navigation, and Control chapter](https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/).
The mounting cross products, bank arrangement, and control integration above
are derived for this proposed geometry.
