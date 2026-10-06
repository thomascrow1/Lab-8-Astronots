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
//   Kp = I_BODY_Z * wn^2, Kd = 2 * zeta * wn * I_BODY_Z  with wn = 0.5 rad/s, zeta = 0.7
#define DEFAULT_KP              0.0035  // N*m per rad of pointing error
#define DEFAULT_KD              0.0098  // N*m per rad/s of body rate
#define DEFAULT_BIAS_PWM        0.5     // wheel bias so it never crosses zero speed (Lab 7 Test B used 0.6)
#define POINTING_DEADBAND_DEG   3.0     // inside this error, only rate damping is applied

// Wheel limits:
#define WHEEL_MIN_RPM           40.0    // stay above this to avoid motor stiction near 0 RPM
#define WHEEL_MAX_RPM           400.0   // ~100% PWM on 5V supply
#define WHEEL_MAX_ACCEL_RPM_S   50.0    // Lab 7 prelab wheel angular acceleration limit

// Sign conventions (verify with command 12, can be changed at runtime with command 14):
#define DEFAULT_WHEEL_PWM_SIGN  -1      // Lab 7 code: w_RW_cmd = -speed_pwm * RPM_PER_PWM
#define DEFAULT_GYRO_SIGN       1       // +1 if gyrZ() is positive for +Z (counter-clockwise from above) rotation

// Sun detection:
#define SUN_LOST_THRESHOLD      0.15    // sum of calibrated channels below this = no sun
#define SUN_LOST_TIME_MS        500     // sun must be missing this long before tracker stops

// Tracker test timing (ms from start of command 13):
#define TRACK_SPINUP_MS         3000    // ramp wheel 0 -> bias (hold KestrelSAT still)
#define TRACK_RELEASE_MS        8000    // RGB LED turns blue: release KestrelSAT
#define TRACK_CONTROL_MS        10000   // closed-loop control begins
#define TRACK_DURATION_MS       120000  // test ends automatically after this (or send 'X')
