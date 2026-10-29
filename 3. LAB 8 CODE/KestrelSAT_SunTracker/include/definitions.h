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
#define SUN_SENSOR_PLUS_Y_PIN   A1
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

// Default controller gains (tunable at runtime with command 14):
//   Kp = I_BODY_Z * wn^2, Kd = 2 * zeta * wn * I_BODY_Z  with wn = 3.0 rad/s, zeta = 1.3 (OVERDAMPED: no overshoot)
//   Sim (measured 1.1 s motor lag): settles to 5 deg in 30 deg ~1.4 s, 90 deg ~2.2 s, 170 deg ~3.2 s, 0 deg overshoot
#define DEFAULT_KP              0.126   // N*m per rad of pointing error
#define DEFAULT_KD              0.1092  // N*m per rad/s of body rate
#define DEFAULT_MAX_RATE_DPS    90.0    // slew rate cap: big turns run at <= this speed, then ease in
#define DEFAULT_BIAS_PWM        0.0     // wheel starts at 0 RPM: full -400..+400 RPM range available both ways
#define POINTING_DEADBAND_DEG   3.0     // inside this error, only rate damping is applied

// Wheel limits:
#define WHEEL_MIN_RPM           -400.0  // wheel may spin both ways (crosses 0 RPM) for twice the range
#define WHEEL_MAX_RPM           400.0   // ~100% PWM on 5V supply
#define WHEEL_STALL_RPM         50.0    // measured 13 Oct: wheel stalls below ~45 RPM; smaller commands jump to +/- this
#define WHEEL_MAX_ACCEL_RPM_S   400.0   // was 50 (Lab 7 prelab limit); raised for fast slews
#define MOTOR_TAU_S             1.1     // measured 13 Oct (SunTrack_data026 spin-up): wheel speed time constant

// Sign conventions (verify with command 12, can be changed at runtime with command 14):
#define DEFAULT_WHEEL_PWM_SIGN  1       // measured 13 Oct (SunTrack_data020): -1 drove KestrelSAT away from the light
#define DEFAULT_GYRO_SIGN       1       // +1 if gyrZ() is positive for +Z (counter-clockwise from above) rotation

// Sun detection:
#define SUN_LOST_THRESHOLD      0.15    // sum of calibrated channels below this = no sun
#define SUN_LOST_TIME_MS        500     // sun must be missing this long before tracker stops

// Tracker test timing (ms from start of command 13):
#define TRACK_SPINUP_MS         3000    // ramp wheel 0 -> bias (hold KestrelSAT still)
#define TRACK_RELEASE_MS        8000    // RGB LED turns blue: release KestrelSAT
#define TRACK_CONTROL_MS        10000   // closed-loop control begins
#define TRACK_DURATION_MS       120000  // test ends automatically after this (or send 'X')

/*---------------------------------------------------------------------------------------------*/
// Fan momentum unloading -- INCOMPLETE HARDWARE CONFIGURATION, OFF by default.
// Proposed driver-input pins for the current ESP32 firmware, pending board photo.
// See the Lab 8 folder's FAN_DESATURATION.md before enabling.
// Each bank is two fans across opposite Y faces at the SAME Z height, with
// opposite tangential thrusts. Both fans of a bank switch together through a
// suitably rated external driver. These are digital enable signals, not fan power.
#define FAN_DESATURATION_ENABLED false
#define FAN_WIRING_CONFIRMED     false // no fan GPIOs are touched until confirmed
#define FAN_TOP_BANK_PIN         27  // proposed GPIO -> top-pair DRIVER input
#define FAN_BOTTOM_BANK_PIN      4   // proposed GPIO -> bottom-pair DRIVER input (FREE pad)
#define FAN_TOP_TORQUE_SIGN      0   // +1/-1 from airflow test; bottom must be opposite
#define FAN_POSITIVE_BANK_PIN    ((FAN_TOP_TORQUE_SIGN > 0) ? FAN_TOP_BANK_PIN : FAN_BOTTOM_BANK_PIN)
#define FAN_NEGATIVE_BANK_PIN    ((FAN_TOP_TORQUE_SIGN > 0) ? FAN_BOTTOM_BANK_PIN : FAN_TOP_BANK_PIN)
#define FAN_ACTIVE_HIGH          true
#define FAN_ENCODER_SIGN         0    // +1/-1 converts RAW encoder RPM to physical +Z
// Preferred calibration: difference between fan-ON and fan-OFF measured wheel
// RPM slopes while holding the same target. Use positive magnitudes here;
// positive/negative refer to physical torque direction, NOT top/bottom wiring.
// This avoids needing a thrust sensor or an accurate wheel inertia for feedforward.
#define FAN_CALIBRATION_IN_RPM_S true
#define FAN_POSITIVE_EQUIV_RPM_S  0.0  // +Z bank: ON slope minus OFF baseline, RPM/s
#define FAN_NEGATIVE_EQUIV_RPM_S  0.0  // magnitude of negative-Z bank's slope difference
// Optional alternative for an independently measured torque (select false above).
#define FAN_POSITIVE_TORQUE_NM    0.0
#define FAN_NEGATIVE_TORQUE_NM    0.0
#define FAN_TORQUE_TAU_S          0.5  // provisional rise/decay model; measure before enabling
#define FAN_TARGET_RPM            0.0
#define FAN_START_RPM             200.0 // actual wheel-speed trigger, provisional from recorded data
#define FAN_STOP_RPM              100.0
#define FAN_COMMAND_START_RPM     300.0 // also trigger at command >=300 with same-sign measured >=150
#define FAN_UNLOAD_DURING_SLEWS   true  // user permits unloading outside the +/-10 deg pointing band
#define FAN_START_ERROR_DEG       5.0
#define FAN_STOP_ERROR_DEG        10.0
#define FAN_MAX_BODY_RATE_DPS     3.0
#define FAN_MAX_ON_S              60.0 // latch fans off for this test if continuous run exceeds this
#define FAN_MAX_CONTROL_DT_S      0.2
