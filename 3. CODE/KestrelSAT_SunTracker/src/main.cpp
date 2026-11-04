/**
 * @file main.cpp
 * @brief KestrelSAT firmware for the Astro 331 Lab 8 sun tracker.
 *
 * Points the solar array (body -X) at a light using the sun sensor, the ICM-20948 gyro and the
 * reaction wheel, runs the disturbance fan test, reports battery state, and logs every test to SD.
 * Commands come from the ground station over XBee (9600 baud) or USB Serial (115200 baud).
 * Pins and settings: definitions.h. Controller math: sun_tracker.cpp (mirrored in the Python sim).
 *
 * @author Lt Col Wyatt Harris (Lab 6/7 base), Astro-nots (Lab 8)
 *
 * @section commands Ground Station Commands
 * | Cmd | Function |
 * |-----|----------|
 * | 0   | Stop reaction wheel and fans |
 * | 1   | Display options menu |
 * | 2   | Get battery telemetry (V, SOC, charge rate) |
 * | 3   | Lab 8: Calibrate sun sensor |
 * | 4   | Lab 8: Run sun tracker (closed-loop, logs to SD) |
 * | 5   | Lab 8: Print / set sun tracker parameters |
 * | 6   | Lab 8: Attitude determination check (sun sensor + gyro, PASS/FAIL) |
 * | 7   | Lab 8: Phototransistor resistor sweep (logs to SD) |
 * | 8   | Lab 8: Fan hold test (sun tracker + fans once on target; time to lose pointing / saturate) |
 * | 10  | Lab 8: Requirement demo (then 1-3: shows R1/R2, R3 or S2 with a live PASS/FAIL) |
 * | 11  | Lab 8: Momentum dump (sun tracker + disturbance fans + "thruster" fans that unload the wheel) |
 */

/*---------------------------------------------------------------------------------------------*/
// Library includes:
/*---------------------------------------------------------------------------------------------*/
#include <Arduino.h>                                      // Main Arduino library
#include <Wire.h>                                         // libray for I2C communication
#include <SPI.h>                                          // SPI communication library
#include <SdFat.h>                                        // SD Card library
#include "definitions.h"                                  // Project definitions (this directory)
#include "sd_functions.h"                                 //SD helper functions (this directory)
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // MAX17048 fuel gauge
#include <ICM_20948.h>                                    // Sparkfun IMU library
#include <TB9051FTGMotorCarrier.h>                        // Pololu Motor Carrier Library
#include "driver/gpio.h"                                  // gpio_set_drive_capability() for the fan pins
#include <ESP32Encoder.h>                                 // Motor encoder library to measure wheel speed
#include "sun_tracker.h"                                  // Lab 8 sun tracker math (this directory)

/*---------------------------------------------------------------------------------------------*/
// Globals:
/*---------------------------------------------------------------------------------------------*/
// Objects:
FsFile dataFile;   // data file object
HardwareSerial Xbee(2); // Serial object for communication with XBee 
SFE_MAX1704X lipo; // SparkFun Thing Plus ESP32-WROOM onboard fuel gauge (I2C addr 0x36)
ICM_20948_I2C imu_sensor; // IMU object
// Motor Variables/Object
constexpr uint8_t pwm1Pin{MOTOR_PWM_1_PIN}; // PWM1
constexpr uint8_t pwm2Pin{MOTOR_PWM_2_PIN}; // PWM2
TB9051FTGMotorCarrier driver{ pwm1Pin, pwm2Pin };// Instantiate TB9051FTGMotorCarrier
ESP32Encoder enc;

// Variables:
uint32_t timeLastCheckForCommand; // time of next Xbee check
uint32_t interval_CheckForCommand = 10; // time interval between Xbee/Serial checks (ms)
uint32_t timeLastHeartBeat; // time of last heartbeat (ms)
uint32_t interval_heartBeat = 3000; // interval between heartbeat (ms)

float sun_plusX, sun_minusX, sun_plusY, sun_minusY;

uint32_t timeNext_testPoint; // time of next test point (ms)
uint32_t interval_testPoint = 50; // time interval between test points (ms)

float gyro_Z = 0.0;
float mag_X = 0.0;
float mag_Y = 0.0;

// Lab 8 Sun Tracker:
SunCalibration sun_cal = {{0.0, 34.6, 0.0, 0.0}, {2086.7, 1680.1, 1777.8, 2771.7}}; // 30 Oct, 3.3k resistors (command 3, +/-Y entries re-ordered for the A1=+Y / A3=-Y pins)
TrackerGains tracker_gains = {DEFAULT_KP, DEFAULT_KD, POINTING_DEADBAND_DEG, DEFAULT_MAX_RATE_DPS};
WheelLimits wheel_limits = {WHEEL_MIN_RPM, WHEEL_MAX_RPM, WHEEL_MAX_ACCEL_RPM_S};
float array_axis_deg = SOLAR_ARRAY_AXIS_DEG;
float bias_pwm = DEFAULT_BIAS_PWM;
int wheel_pwm_sign = DEFAULT_WHEEL_PWM_SIGN;
int gyro_sign = DEFAULT_GYRO_SIGN;
uint32_t track_duration_ms = TRACK_DURATION_MS; // command 4 length, set with 'dur' in command 5
// Disturbance fans (command 8):
float fan_upper_pct = 0.0;       // duty the GPIO 27 pair is running at right now
float fan_lower_pct = 0.0;       // duty the GPIO 4 pair is running at right now

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see defintiions after loop()):
/*---------------------------------------------------------------------------------------------*/
int get_command_from_ground_station();
void process_main_menu();
void send_battery_telemetry();
void print_both(const String &msg);
// Lab 8 Sun Tracker:
void read_sun_raw(float raw[4]);
float read_body_rate_dps();
float measure_wheel_rpm();
void set_wheel_rpm(float w_rpm);
String wait_for_line();
bool abort_requested();
void sun_calibrate();
void sun_track(float fan_u_pct, float fan_l_pct, int demo_req = 0, bool desat = false);
void fan_hold_test();
void req_demo();
void momentum_dump_test();
bool parse_fan_choice(String words, float default_duty, String &dir, float &duty);
void sun_set_params();
void sun_print_params();
void sample_sun_stats(int n, float mean[4], float sd[4]);
void ad_check();
void pt_sweep();
void set_fans(float upper_pct, float lower_pct);

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Disturbance fans off FIRST, before waiting for the Serial monitor.
  ledcSetup(FAN_UPPER_LEDC_CH, FAN_PWM_FREQ_HZ, FAN_PWM_BITS);
  ledcSetup(FAN_LOWER_LEDC_CH, FAN_PWM_FREQ_HZ, FAN_PWM_BITS);
  ledcAttachPin(FAN_UPPER_PIN, FAN_UPPER_LEDC_CH);
  ledcAttachPin(FAN_LOWER_PIN, FAN_LOWER_LEDC_CH);
  // Strongest pin drive (~40 mA): the XY-MOS trigger LED loads the pin.
  gpio_set_drive_capability((gpio_num_t)FAN_UPPER_PIN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)FAN_LOWER_PIN, GPIO_DRIVE_CAP_3);
  set_fans(0, 0);

  Serial.begin(115200); // Begin Serial communication with computer
  while (!Serial) {delay(10);} // Wait for user to open Serial monitor before proceeding
  Serial.println("[INFO] Hello World!");

  Wire.begin(); // Initialize I2C communication

  // Initialize built-in RGB LED (WS2812) and STAT LED
  // #define RGB_BUILTIN  2
  pinMode(RGB_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 255, 0, 0); // Default to red (R=255, G=0, B=0)

  //----------------------------------------------
  // Initialize Serial link with XBee
  //----------------------------------------------
  Xbee.begin(9600,SERIAL_8N1, XBEE_RX, XBEE_TX);  // Begin MCU <> XBee communication
  Xbee.setTimeout(20);
  Xbee.println("[INFO] KestrelSAT online \npress 1 for options");
  // ----------------------------------------------

  //----------------------------------------------
  // Initialize SD Card
  //----------------------------------------------
  sd_init(SD_CS_PIN);
  //----------------------------------------------

  //----------------------------------------------
  // Initialize MAX17048 fuel gauge
  //----------------------------------------------
  if (!lipo.begin(Wire)) // Uses I2C address 0x36) 
  {
    Serial.println("[WARN] MAX17048 not detected on I2C (0x36). Battery telemetry (cmd 2) will be unavailable.");
  } else {
    lipo.quickStart();    // Improves initial SOC accuracy after boot. Returns 0 on success.
    Serial.println("[INFO] MAX17048 online.");
  }
  //----------------------------------------------

  //----------------------------------------------
  // Initialize ICM20948
  //----------------------------------------------
  if (imu_sensor.begin(Wire, 1) != ICM_20948_Stat_Ok) {
    Serial.println("[CAUTION] IMU not found.");
    while (1);
  } else{
    Serial.println("[INFO] IMU Initialized.");
  }
  //----------------------------------------------

  //----------------------------------------------
  // Initialize Sun Sensor
  //----------------------------------------------
  analogReadResolution(12);

  //----------------------------------------------
  // Initialize Reaction Wheel
  //----------------------------------------------
  driver.enable(); // TB9051FTG Motor Driver
  driver.setOutput(0);
  enc.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B); // Motor Encoder
  enc.clearCount();
  //----------------------------------------------

  Serial.println("[INFO] SETUP COMPLETE.");
  Xbee.println("[INFO] SETUP COMPLETE.SEND '1' FOR OPTIONS.");
  neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //----------------------------------------------
  // Check for data from ground station
  // ----------------------------------------------
  if (timeLastCheckForCommand + interval_CheckForCommand < millis()) { // periodic Xbee send
    timeLastCheckForCommand = millis();
    process_main_menu();
  }

  if (timeLastHeartBeat + interval_heartBeat < millis()) { // periodic heartbeat to indicate program is alive
    timeLastHeartBeat = millis();
    Serial.println("[INFO] Send '1' for Options");
    Xbee.println("*");
    neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCITON DEFINITIONS:
/////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------------------------*/
// Get Command from Ground Station:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Read one command line from XBee (checked first) or USB Serial and return it as an integer.
 *        Returns 0 if nothing is waiting and -1 for an empty line. Blocks until the line arrives.
 */
int get_command_from_ground_station() {
  if (!Xbee.available() && !Serial.available()) return 0;
  bool from_xbee = Xbee.available();
  String line = from_xbee ? Xbee.readStringUntil('\n') : Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return -1;
  Serial.println(String("Received from ") + (from_xbee ? "XBee: " : "Serial: ") + line);
  return line.toInt();
}

/*---------------------------------------------------------------------------------------------*/
// Process Main Menu:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Run the command received from the ground station (see the table at the top of this file).
 */
void process_main_menu() {
  if (!Xbee.available() && !Serial.available()) return;

  int received_int = get_command_from_ground_station();

  switch (received_int) {
    case 0:
      driver.setOutput(0);
      set_fans(0, 0);
      print_both("Motor and fans stopped.");
      break;
    
    case 1:
      print_both("0 - Stop reaction wheel and fans");
      print_both("1 - Print Options Menu");
      print_both("2 - Get Battery State (V, SOC, dSOC/dt)");
      print_both("3 - Lab 8: Calibrate Sun Sensor");
      print_both("4 - Lab 8: Run Sun Tracker");
      print_both("5 - Lab 8: Print/Set Tracker Parameters");
      print_both("6 - Lab 8: Attitude Determination Check");
      print_both("7 - Lab 8: Phototransistor Resistor Sweep");
      print_both("8 - Lab 8: Fan Hold Test (CW/CCW)");
      print_both("10 - Lab 8: Requirement Demo (R1/R2, R3, S2)");
      print_both("11 - Lab 8: Momentum Dump (fans = disturbance + thrusters)");
      break;

    case 2:
      send_battery_telemetry();
      break;

    case 3:
      sun_calibrate();
      break;

    case 4:
      sun_track(0.0, 0.0); // fans off
      break;

    case 5:
      sun_set_params();
      break;

    case 6:
      ad_check();
      break;

    case 7:
      pt_sweep();
      break;

    case 8:
      fan_hold_test();
      break;

    case 10:
      req_demo();
      break;

    case 11:
      momentum_dump_test();
      break;

    default:
      print_both("[CAUTION] Invalid input from ground station, ignoring.");
      break;
  }
  return;
}//end function process_main_menu()

/*---------------------------------------------------------------------------------------------*/
// Get Battery Information:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Send MAX17048 battery telemetry over the XBee link.
 *
 * Reads:
 *  - Battery voltage (V)
 *  - State of Charge (SOC, %)
 *  - Change / discharge rate (dSOC/dt, %/hr; negative = discharging)
 *
 * Output format (single line):
 *   BAT,V=<volts>,SOC=<percent>,CR=<percent_per_hr>
 */
void send_battery_telemetry() {
  // Charge rate is %/hr: positive = charging, negative = discharging.
  print_both(String("BAT,V=") + String(lipo.getVoltage(), 3) + "V, SOC=" + String(lipo.getSOC(), 1) +
             "%, CR=" + String(lipo.getChangeRate(), 3) + "%/hr");
} 

/////////////////////////////////////////////////////////////////////////////////////////////////
// LAB 8: SUN TRACKER
/////////////////////////////////////////////////////////////////////////////////////////////////
// Goal: point the solar array (body -X, SOLAR_ARRAY_AXIS_DEG) at the sun using the Lab 6 sun
// sensor for pointing error, the gyro for rate damping, and the Lab 7 reaction wheel for torque.
// Recommended order on hardware: 3 (calibrate) -> 6 (check) -> 4 (track).
// Controller math lives in sun_tracker.cpp so it can be compared to the Python simulation.

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Helpers:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Print a line to both USB Serial and XBee.
 */
void print_both(const String &msg) {
  Serial.println(msg);
  Xbee.println(msg);
}

/**
 * @brief Read all four sun sensor channels (averaged), order +X, +Y, -X, -Y.
 *        The SUN_READS_PER_SAMPLE reads are spread evenly over SUN_READ_SPAN_US (one lamp flicker
 *        period) so the 120 Hz flicker averages out. Takes ~8.3 ms; fits inside the 20 ms control loop.
 */
void read_sun_raw(float raw[4]) {
  sun_plusX = 0.0;
  sun_minusX = 0.0;
  sun_plusY = 0.0;
  sun_minusY = 0.0;
  const uint32_t spacing_us = SUN_READ_SPAN_US / SUN_READS_PER_SAMPLE;
  uint32_t t_start = micros();
  for (int i = 0; i < SUN_READS_PER_SAMPLE; i++) {
    while (micros() - t_start < i * spacing_us) {}  // wait for this read's slot in the flicker period
    sun_plusX += analogRead(SUN_SENSOR_PLUS_X_PIN);
    sun_minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
    sun_plusY += analogRead(SUN_SENSOR_PLUS_Y_PIN);
    sun_minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
  }
  sun_plusX /= SUN_READS_PER_SAMPLE;
  sun_minusX /= SUN_READS_PER_SAMPLE;
  sun_plusY /= SUN_READS_PER_SAMPLE;
  sun_minusY /= SUN_READS_PER_SAMPLE;

  raw[SUN_PX] = sun_plusX;
  raw[SUN_PY] = sun_plusY;
  raw[SUN_MX] = sun_minusX;
  raw[SUN_MY] = sun_minusY;
}

/**
 * @brief Read the IMU and return body rate about +Z (deg/s) using gyro_sign.
 *        Also updates gyro_Z, mag_X, mag_Y globals (gyro_Z is the raw reading).
 */
float read_body_rate_dps() {
  imu_sensor.getAGMT();
  gyro_Z = imu_sensor.gyrZ();
  mag_X = imu_sensor.magX();
  mag_Y = imu_sensor.magY();
  return gyro_sign * gyro_Z;
}

/**
 * @brief Measured wheel speed from the encoder (RPM), same math as Lab 7.
 *        Sign is the raw encoder sign.
 */
float measure_wheel_rpm() {
  static int64_t lastCount = 0;
  static uint32_t timeLastEncMeas = 0;
  uint32_t timeNow = millis();
  int64_t c = enc.getCount();
  float dt_s = (timeNow - timeLastEncMeas) / 1000.0f;
  float w_rpm = 0.0;
  if (dt_s > 0) {
    float rev = (float)(c - lastCount) / ((float)CPR * 10.0);
    w_rpm = (rev / dt_s) * 60.0f;
  }
  lastCount = c;
  timeLastEncMeas = timeNow;
  return w_rpm;
}

/**
 * @brief Command the wheel to a speed (RPM, positive = wheel spinning about +Z).
 *        Open loop: PWM is proportional to speed, like Lab 7 (RPM_PER_PWM).
 */
void set_wheel_rpm(float w_rpm) {
  // The wheel stalls below ~WHEEL_STALL_RPM, so jump small commands past the stall zone.
  if (fabs(w_rpm) > 1.0 && fabs(w_rpm) < WHEEL_STALL_RPM) w_rpm = (w_rpm > 0) ? WHEEL_STALL_RPM : -WHEEL_STALL_RPM;
  float pwm = wheel_pwm_sign * w_rpm / RPM_PER_PWM;
  if (pwm > 1.0) pwm = 1.0;
  if (pwm < -1.0) pwm = -1.0;
  driver.setOutput(pwm);
}

/**
 * @brief Wait (blocking) for one line of text from USB Serial or XBee.
 * @return the trimmed line
 */
String wait_for_line() {
  while (!Serial.available() && !Xbee.available()) {} // wait for user input
  delay(100); // small delay to ensure serial buffer is fully received
  String line;
  if (Xbee.available()) {
    line = Xbee.readStringUntil('\n');
  } else {
    line = Serial.readStringUntil('\n');
  }
  line.trim();
  while (Serial.available()) Serial.read(); // clear serial buffer
  while (Xbee.available()) Xbee.read();     // clear Xbee buffer
  return line;
}

/**
 * @brief Non-blocking check for an 'X' abort from USB Serial or XBee.
 * @return true if the user sent 'X' or 'x'
 */
bool abort_requested() {
  while (Serial.available() > 0 || Xbee.available() > 0) {
    char c = (Serial.available() > 0) ? Serial.read() : Xbee.read();
    if (c == 'X' || c == 'x') return true;
  }
  return false;
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 3: Calibrate Sun Sensor
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Measure dark and bright ADC counts for each phototransistor.
 *
 * Steps (user sends any key to advance, 'X' to cancel):
 *  1. Light source off (room as it will be during the test) -> dark counts for all channels
 *  2. Light pointed straight at +X, +Y, -X, -Y in turn       -> bright count for that channel
 *
 * Results are applied immediately and printed as a line to paste into main.cpp
 * so they survive a reboot.
 */
void sun_calibrate() {
  const int n_samples = 100;
  const char *names[4] = {"+X", "+Y", "-X", "-Y"};
  SunCalibration new_cal;
  float raw[4];
  bool cal_ok = true;

  print_both("[INFO] SUN SENSOR CALIBRATION. Send 'X' at any prompt to cancel.");
  print_both("[INFO] Step 1: turn OFF the light source (room lights as they will be during the test), then send any key.");
  if (wait_for_line().equalsIgnoreCase("x")) { print_both("[INFO] Calibration canceled."); return; }

  float sum[4] = {0, 0, 0, 0};
  for (int k = 0; k < n_samples; k++) {
    read_sun_raw(raw);
    for (int i = 0; i < 4; i++) sum[i] += raw[i];
    delay(5);
  }
  for (int i = 0; i < 4; i++) new_cal.dark[i] = sum[i] / n_samples;

  for (int i = 0; i < 4; i++) {
    print_both(String("[INFO] Step ") + (i + 2) + ": turn ON the light and point it straight at the " + names[i] + " phototransistor, then send any key.");
    if (wait_for_line().equalsIgnoreCase("x")) { print_both("[INFO] Calibration canceled."); return; }
    float s = 0;
    for (int k = 0; k < n_samples; k++) {
      read_sun_raw(raw);
      s += raw[i];
      delay(5);
    }
    new_cal.bright[i] = s / n_samples;
    if (new_cal.bright[i] < new_cal.dark[i] + 300) {
      print_both(String("[CAUTION] ") + names[i] + " barely changed with light on it (dark " + String(new_cal.dark[i], 0) +
                 ", bright " + String(new_cal.bright[i], 0) + "). Aim the light straight at it, closer.");
      cal_ok = false;
    }
  }

  // A tiny dark-to-bright range blows that channel up to huge values and wrecks the sun direction,
  // so keep the old calibration instead of applying a bad one.
  if (!cal_ok) {
    print_both("[CAUTION] Calibration NOT applied (previous calibration kept). Run 3 again.");
    return;
  }

  sun_cal = new_cal;
  print_both("[INFO] Calibration applied. To keep it after a reboot, paste this line over the sun_cal line in main.cpp:");
  String line = "SunCalibration sun_cal = {{";
  for (int i = 0; i < 4; i++) { line += String(sun_cal.dark[i], 1); line += (i < 3) ? ", " : "}, {"; }
  for (int i = 0; i < 4; i++) { line += String(sun_cal.bright[i], 1); line += (i < 3) ? ", " : "}};"; }
  print_both(line);
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 4: Run Sun Tracker
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Closed-loop sun tracking with the reaction wheel. Logs every test point to SD.
 *
 * Timeline (ms after start, see definitions.h):
 *  - 0 to TRACK_SPINUP_MS      : SPINUP - ramp wheel to bias speed (hold KestrelSAT still), LED yellow
 *  - to TRACK_CONTROL_MS       : HOLD   - wheel at bias; LED turns BLUE at TRACK_RELEASE_MS = let go
 *  - after TRACK_CONTROL_MS    : ACQUIRE - turn one full circle at SCAN_RATE_DPS, find the heading where
 *                                          -X is brightest (LED purple)
 *                                TRACK   - hold that heading with the gyro (PD), LED green within 10 deg,
 *                                          orange outside. Sequential lobing (aim +/-LOBE_AMP_DEG, compare
 *                                          -X brightness) keeps nudging the sun heading to correct drift.
 *                                          If -X gets very dim while pointed there -> ACQUIRE again
 *  - Disturbance fans (command 8 only, when a duty > 0 is passed in): on once pointing has stayed
 *    within FAN_HOLD_BAND_DEG for FAN_HOLD_TIME_MS; reports when pointing first leaves the band.
 *    With fans on, the test ends once the wheel has been saturated for FAN_SAT_STOP_MS.
 *  - Momentum dump (command 11, desat = true): same fan start, but when |wheel command| reaches
 *    DESAT_START_RPM the OTHER fan pair fires as a "thruster" at DESAT_THRUSTER_PCT against the disturbance.
 *    The controller then spins the wheel down while it keeps pointing; the thrusters stop below DESAT_STOP_RPM.
 *  - no sun found in a scan    : wheel and fans stopped, warning sent, test ends
 *  - track_duration_ms or 'X'  : wheel and fans stopped, test ends ('dur' in command 5 sets the length;
 *                                command 8 runs up to FAN_HOLD_DURATION_MS)
 * The gyro bias is measured while KestrelSAT is held (SPINUP/HOLD) and removed from the heading.
 * @param fan_u_pct     GPIO 27 pair duty once the fans turn on (0-100); command 4 passes 0 (no fans)
 * @param fan_l_pct     GPIO 4 pair duty once the fans turn on (0-100); command 4 passes 0 (no fans)
 * @param demo_req      command 10 only: requirement to judge live (1-3; see req_demo()). 0 = normal run
 * @param desat         command 11 (and 10 -> 3): the fan pair NOT passed in is the thruster that unloads the wheel
 */
static SunScan sun_scan;  // ~5 KB, kept off the stack
void sun_track(float fan_u_pct, float fan_l_pct, int demo_req, bool desat) {
  char file_name[40];
  if (sd_createDataFile(&dataFile, "SunTrack")) {
    dataFile.println("mcu time(ms),mode,sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),"
                     "sun_mag,sun_direction(deg),error(deg),body_rate(deg/s),mag_X(uT),mag_Y(uT),"
                     "torque_cmd(N*m),w_RW_cmd(RPM),w_RW_meas(RPM),saturated,err_ctrl(deg),latch,"
                     "heading(deg),sun_heading(deg),aim_offset(deg),mx_bright,fan_gpio27(%),fan_gpio4(%),dumping");
    dataFile.flush();
    dataFile.getName(file_name, sizeof(file_name));
    print_both(String("[INFO] Data file created successfully: ") + file_name);
  } else {
    print_both("[ERROR] Failed to create data file. Aborting test.");
    return;
  }

  float raw[4];
  read_sun_raw(raw);
  if (compute_sun(raw, sun_cal).magnitude < SUN_LOST_THRESHOLD) {
    print_both("[CAUTION] No sun seen right now (OK if it is in a blind spot; the scan will look for it).");
  }

  print_both("[INFO] Ready to run sun tracker. HOLD KestrelSAT still and send any key to begin.");
  print_both("[INFO] Release KestrelSAT when the RGB LED turns BLUE. Send 'X' to stop.");
  if (wait_for_line().equalsIgnoreCase("x")) { dataFile.close(); print_both("[INFO] Test canceled."); return; }

  const float bias_rpm = bias_pwm * RPM_PER_PWM;
  float w_cmd_rpm = 0.0;
  float w_prev_cmd_rpm = 0.0;   // for motor lag compensation
  float tau = 0.0;
  FarErrorLatch latch = {0, 0.0};  // 180 deg direction latch
  float err_ctrl = 0.0;            // error the controller used (after the latch)
  float gyro_bias = 0.0;           // deg/s, measured while held
  float bias_sum = 0.0;
  int bias_n = 0;
  float heading = 0.0;             // deg, integrated gyro since control start (+Z)
  bool acquiring = false;
  bool control_started = false;
  float sun_heading = 0.0;         // heading where -X was brightest
  float sun_peak = 0.0;            // that brightness (calibrated -X)
  uint32_t scan_t0 = 0;
  uint32_t dim_since = 0;
  LobeState lobe;                  // sequential lobing (fine pointing)
  lobe_reset(lobe);
  float aim_offset = 0.0;          // lobing aim offset from the sun heading (deg)
  bool saturated = false;
  uint32_t saturated_since = 0;
  uint32_t last_saturation_warning = 0;
  bool release_announced = false;
  int test_point_count = 0;
  bool fans_wanted = fan_u_pct > 0 || fan_l_pct > 0;
  // Command 10 -> 3 (S2) needs time for the wheel to spin up and dump, so it gets the long fan test length.
  const uint32_t duration_ms = (demo_req && !desat) ? DEMO_DURATION_MS : (fans_wanted ? FAN_HOLD_DURATION_MS : track_duration_ms);
  const char *demo_names[4] = {"", "R1/R2", "R3", "S2"};
  const char *demo_name = (demo_req >= 1 && demo_req <= 3) ? demo_names[demo_req] : "";
  bool fans_on = false;
  uint32_t fan_kick_until = 0;     // millis() when the 100% start-up kick ends
  uint32_t fan_on_t = 0;           // test time (ms) the fans turned on
  bool fan_sat_reported = false;
  float err_point = 999.0;         // pointing error (deg) the requirement is judged on: gyro heading vs sun heading
  uint32_t in_band_since = 0;      // command 8: millis() pointing entered the band (0 = outside)
  uint32_t fan_band_lost_t = 0;    // test time (ms) pointing first left the band after the fans turned on
  uint32_t fan_band_ms = 0;        // total time inside the band since the fans turned on
  uint32_t fan_run_start = 0;      // test time (ms) the current stretch inside the band began (0 = outside)
  uint32_t fan_longest_ms = 0;     // longest stretch inside the band since the fans turned on
  // Momentum dump (command 11 and 10 -> 3, desat = true): the pair not used as the disturbance is the thruster.
  const float thr_u_pct = (desat && fan_u_pct <= 0) ? DESAT_THRUSTER_PCT : 0.0;  // GPIO 27 thruster duty
  const float thr_l_pct = (desat && fan_l_pct <= 0) ? DESAT_THRUSTER_PCT : 0.0;  // GPIO 4 thruster duty
  bool dumping = false;            // thrusters firing right now
  uint32_t dump_start = 0;         // millis() this dump started
  int dump_count = 0;              // dumps finished
  uint32_t dump_total_ms = 0;      // time spent dumping (finished dumps)
  uint32_t last_dump_end = 0;      // millis() the last dump finished
  // Command 10 (requirement demo):
  float demo_offset = 0.0;         // R1/R2: aim the array this far from the sun heading (180 = turned away)
  float err_target = 999.0;        // error to where the array is aimed now (= err_point unless R2 turned away)
  int demo_step = 0;               // R1/R2: 1 turn 180 deg away (after the scan), 2 timed slew back, 3 R1 hold
  uint32_t demo_in_since = 0;      // millis() err_target entered the band (0 = outside)
  float demo_max_err = 0.0;        // largest |err_target| since entering the band
  uint32_t demo_next_print = 0;    // millis() of the next progress line
  uint32_t demo_slew_t = 0;        // R2: millis() the timed slew back started
  bool demo_reached = false;       // R2: inside the band since the slew started
  uint32_t demo_reach_ms = 0;      // R2: time from slew start to entering the band
  uint32_t demo_n = 0, demo_n5 = 0; // R1: samples in the hold, and how many of them were within +/-5 deg
  bool demo_r2_pass = false;       // R2: reached the sun within the limit
  set_fans(0, 0);
  if (desat) {
    print_both(String("[INFO] Disturbance fans (") + (fan_u_pct > 0 ? "GPIO 27" : "GPIO 4") + ", " +
               String(fan_u_pct > 0 ? fan_u_pct : fan_l_pct, 0) + "%) turn on after " + (FAN_HOLD_TIME_MS / 1000) +
               " s within +/-" + String(FAN_HOLD_BAND_DEG, 0) + " deg. Thrusters (" + (thr_u_pct > 0 ? "GPIO 27" : "GPIO 4") +
               ", " + String(DESAT_THRUSTER_PCT, 0) + "%) fire at " + String(DESAT_START_RPM, 0) + " RPM and stop at " +
               String(DESAT_STOP_RPM, 0) + " RPM. Runs up to " + (duration_ms / 60000) + " min or until 'X'.");
  } else if (fans_wanted) {
    print_both(String("[INFO] Disturbance fans will turn on after ") + (FAN_HOLD_TIME_MS / 1000) + " s within +/-" +
               String(FAN_HOLD_BAND_DEG, 0) + " deg (GPIO 27 " + String(fan_u_pct, 0) + "%, GPIO 4 " +
               String(fan_l_pct, 0) + "%). Runs until the wheel saturates (max " + (duration_ms / 60000) + " min).");
  }
  // Command 8 summary, printed on every way the test ends.
  auto print_hold_summary = [&](uint32_t t_end) {
    if (!fans_wanted) return;
    if (!fans_on) { print_both("[RESULT] Fans never turned on (pointing never held the band long enough)."); return; }
    float on_s = (t_end - fan_on_t) / 1000.0;
    uint32_t longest_ms = fan_longest_ms;
    if (fan_run_start != 0 && t_end - fan_run_start > longest_ms) longest_ms = t_end - fan_run_start;
    if (desat) {
      print_both(String("[RESULT] Momentum dumps finished: ") + dump_count +
                 (dump_count > 0 ? String(" (average ") + String(dump_total_ms / 1000.0 / dump_count, 1) + " s each)" : String("")) +
                 (dumping ? ", one more was still running" : "") + ". Longest time inside +/-" +
                 String(FAN_HOLD_BAND_DEG, 0) + " deg with the fans on: " + String(longest_ms / 1000.0, 1) + " s.");
    }
    print_both(String("[RESULT] Fans on for ") + String(on_s, 1) + " s. Pointing " +
               (fan_band_lost_t ? String("first left +/-") + String(FAN_HOLD_BAND_DEG, 0) + " deg after " +
                                  String((fan_band_lost_t - fan_on_t) / 1000.0, 1) + " s"
                                : String("never left the band")) +
               "; inside the band " + String(on_s > 0 ? 100.0 * fan_band_ms / (on_s * 1000.0) : 0.0, 0) +
               "% of the time. Wheel " + (fan_sat_reported ? "saturated (see above)." : "did not saturate."));
  };

  // Every way the test ends: wheel and fans off, file closed, message, command 8 summary.
  auto end_test = [&](const String &msg, uint32_t t_end) {
    driver.setOutput(0);
    set_fans(0, 0);
    dataFile.close();
    print_both(msg + " File " + file_name + " closed.");
    print_hold_summary(t_end);
  };

  uint32_t t0 = millis();
  uint32_t tLastControl = t0;
  timeNext_testPoint = t0;
  measure_wheel_rpm(); // reset encoder timing
  neopixelWrite(RGB_BUILTIN, 255, 255, 0); // yellow: hold still

  while (true) {
    if (abort_requested()) {
      end_test("[CAUTION] Test stopped by user. Wheel and fans stopped.", millis() - t0);
      return;
    }

    uint32_t now = millis();
    if (now < timeNext_testPoint) continue;
    timeNext_testPoint += interval_testPoint;
    test_point_count++;
    uint32_t t = now - t0;
    float dt_s = (now - tLastControl) / 1000.0f;
    tLastControl = now;

    // Sense:
    read_sun_raw(raw);
    SunReading sun = compute_sun(raw, sun_cal);
    float err = pointing_error_deg(sun.direction_deg, array_axis_deg);
    float rate = read_body_rate_dps();
    float w_meas = measure_wheel_rpm();
    if (control_started) heading += (rate - gyro_bias) * dt_s;
    // -X brightness WITHOUT the 1.2 cap in compute_sun(), so the top of the brightness peak isn't flattened
    float mx_bright = (raw[SUN_MX] - sun_cal.dark[SUN_MX]) / (sun_cal.bright[SUN_MX] - sun_cal.dark[SUN_MX]);
    if (mx_bright < 0.0) mx_bright = 0.0;

    // Decide & control:
    const char *mode;
    saturated = false;
    tau = 0.0;
    if (t < TRACK_SPINUP_MS) {
      mode = "SPINUP";
      w_cmd_rpm = bias_rpm * t / TRACK_SPINUP_MS;
    } else if (t < TRACK_CONTROL_MS) {
      mode = "HOLD";
      w_cmd_rpm = bias_rpm;
      if (t >= TRACK_SPINUP_MS && t < TRACK_RELEASE_MS) { bias_sum += rate; bias_n++; } // held still
      if (t >= TRACK_RELEASE_MS && !release_announced) {
        release_announced = true;
        neopixelWrite(RGB_BUILTIN, 0, 0, 255); // blue: release
        print_both("[INFO] RELEASE KestrelSAT now.");
      }
    } else {
      if (!control_started) {
        control_started = true;
        gyro_bias = (bias_n > 0) ? bias_sum / bias_n : 0.0;
        heading = 0.0;
        acquiring = true;
        scan_start(sun_scan, heading);
        scan_t0 = now;
        print_both(String("[INFO] Gyro bias ") + String(gyro_bias, 3) + " deg/s. Scanning for the sun (one full turn).");
      }

      if (acquiring) {
        mode = "ACQUIRE";
        err_point = 999.0;  // scanning: not pointed at anything
        err_target = 999.0;
        neopixelWrite(RGB_BUILTIN, 128, 0, 128); // purple: scanning
        tau = rate_step(SCAN_RATE_DPS, rate, dt_s, tracker_gains, wheel_limits, I_WHEEL, w_cmd_rpm, saturated);
        bool done = scan_add(sun_scan, heading, mx_bright, SCAN_TURN_DEG);
        if (done || now - scan_t0 > SCAN_TIMEOUT_MS) {
          if (!scan_result(sun_scan, SCAN_PEAK_FRACTION, SUN_LOST_THRESHOLD, sun_heading, sun_peak)) {
            neopixelWrite(RGB_BUILTIN, 255, 0, 0); // red
            end_test(String("[WARN] No sun found in the scan (brightest -X ") + String(sun_peak, 2) +
                     "). Wheel stopped.", t);
            return;
          }
          if (!done) print_both("[CAUTION] Scan timed out before a full turn; using what was seen.");
          acquiring = false;
          dim_since = 0;
          latch = {0, 0.0};
          lobe_reset(lobe);
          aim_offset = 0.0;
          print_both(String("[INFO] Sun found at heading ") + String(sun_heading, 1) + " deg (peak -X " +
                     String(sun_peak, 2) + "). Tracking with the gyro.");
          // Command 10 R1/R2: before tracking, send the array 180 deg AWAY from the sun it just found.
          if (demo_req == 1 && demo_step == 0) {
            demo_offset = 180.0;
            demo_step = 1;
            print_both("[DEMO] R2: sun found. Turning the solar array 180 deg AWAY from the sun before tracking.");
          }
        }
      } else {
        mode = "TRACK";
        float err_gyro = wrap180(sun_heading - heading);           // how far the array is from the sun heading
        err_point = err_gyro;
        err_target = wrap180(sun_heading + demo_offset - heading);
        if (demo_offset == 0.0) {
          float err_aim = wrap180(sun_heading + aim_offset - heading); // how far from this lobe's aim point
          bool settled = fabs(err_aim) < LOBE_SETTLED_DEG && fabs(rate) < LOBE_SETTLED_DPS;
          aim_offset = lobe_step(lobe, mx_bright, settled, dt_s, LOBE_AMP_DEG, LOBE_DWELL_MS / 1000.0,
                                 LOBE_AVG_MS / 1000.0, LOBE_GAIN, LOBE_MAX_STEP_DEG, LOBE_HALF_POWER_DEG, sun_heading);
        } else {
          // Command 10 R1/R2 has the array turned away: -X sees no light, so lobing would only spoil the sun heading.
          aim_offset = 0.0;
          lobe_reset(lobe);
        }
        float err_aim = wrap180(sun_heading + demo_offset + aim_offset - heading);
        err_ctrl = latch_far_error(err_aim, dt_s, LATCH_ENTER_DEG, LATCH_EXIT_DEG, LATCH_EXIT_TIME_MS / 1000.0, latch);
        tau = tracker_step(err_ctrl, rate, dt_s, tracker_gains, wheel_limits, I_WHEEL, w_cmd_rpm, saturated);
        if (fabs(err_gyro) <= 10.0) neopixelWrite(RGB_BUILTIN, 0, 255, 0);   // green: on target
        else                        neopixelWrite(RGB_BUILTIN, 255, 165, 0); // orange: slewing

        // Pointed at the sun's heading but -X is dim: the sun moved or the heading drifted -> rescan.
        if (fabs(err_gyro) < REACQ_ON_TARGET_DEG && mx_bright < REACQ_FRACTION * sun_peak) {
          if (dim_since == 0) dim_since = now;
          if (now - dim_since > REACQ_TIME_MS) {
            acquiring = true;
            scan_start(sun_scan, heading);
            scan_t0 = now;
            print_both(String("[INFO] -X dimmed to ") + String(mx_bright, 2) + " at t=" + t + " ms. Rescanning.");
          }
        } else {
          dim_since = 0;
        }
      }
    }
    set_wheel_rpm(motor_lag_comp(w_cmd_rpm, w_prev_cmd_rpm, dt_s, MOTOR_TAU_S, RPM_PER_PWM));
    w_prev_cmd_rpm = w_cmd_rpm;

    // Warn (once per 5 s) if the wheel stays at a speed limit for more than 2 s:
    if (saturated) {
      if (saturated_since == 0) saturated_since = now;
      if (now - saturated_since > 2000 && now - last_saturation_warning > 5000) {
        print_both("[WARN] Reaction wheel saturated: cannot produce more torque in that direction.");
        last_saturation_warning = now;
      }
    } else {
      saturated_since = 0;
    }

    // Disturbance fans (command 8): on once pointing has stayed inside the band for FAN_HOLD_TIME_MS.
    bool in_band = fabs(err_point) <= FAN_HOLD_BAND_DEG;
    if (in_band) { if (in_band_since == 0) in_band_since = now; }
    else in_band_since = 0;
    if (fans_wanted && !fans_on && in_band_since != 0 && now - in_band_since >= FAN_HOLD_TIME_MS) {
      fans_on = true;
      fan_on_t = t;
      set_fans(fan_u_pct > 0 ? 100 : 0, fan_l_pct > 0 ? 100 : 0); // kick so low duties start
      fan_kick_until = now + FAN_KICK_MS;
      print_both(String("[INFO] Disturbance fans ON at t=") + t + " ms (GPIO 27 " + String(fan_u_pct, 0) +
                 "%, GPIO 4 " + String(fan_l_pct, 0) + "%), wMeas " + String(w_meas, 0) + " RPM.");
    }
    if (fan_kick_until != 0 && now >= fan_kick_until) {
      fan_kick_until = 0;
      set_fans(fan_u_pct, fan_l_pct);
    }
    // Momentum dump (command 11): fire the thrusters when the wheel nears saturation, stop once it has slowed.
    // Uses the wheel COMMAND, the same number the saturation check uses. Not during the 100% start-up kick.
    if (desat && fans_on && fan_kick_until == 0) {
      if (!dumping && fabs(w_cmd_rpm) >= DESAT_START_RPM) {
        dumping = true;
        dump_start = now;
        set_fans(fan_u_pct + thr_u_pct, fan_l_pct + thr_l_pct);
        print_both(String("[DESAT] Dump ") + (dump_count + 1) + ": wheel at " + String(w_cmd_rpm, 0) +
                   " RPM. THRUSTERS ON at t=" + t + " ms.");
      } else if (dumping && fabs(w_cmd_rpm) <= DESAT_STOP_RPM) {
        dumping = false;
        dump_count++;
        dump_total_ms += now - dump_start;
        last_dump_end = now;
        set_fans(fan_u_pct, fan_l_pct);
        print_both(String("[DESAT] Dump ") + dump_count + " done in " + String((now - dump_start) / 1000.0, 1) +
                   " s: wheel back to " + String(w_cmd_rpm, 0) + " RPM. Thrusters off. Pointing error " +
                   String(err_point, 1) + " deg.");
      } else if (dumping && now - dump_start > DESAT_MAX_MS) {
        end_test(String("[WARN] Thrusters on for ") + (DESAT_MAX_MS / 1000) + " s and the wheel did not slow below " +
                 String(DESAT_STOP_RPM, 0) + " RPM (now " + String(w_cmd_rpm, 0) +
                 "). Disturbance too strong for the thrusters? Try a lower duty. Wheel and fans stopped.", t);
        return;
      }
    }

    // How long pointing stays inside the band with the fans on.
    if (fans_on) {
      if (in_band) {
        fan_band_ms += interval_testPoint;
        if (fan_run_start == 0) fan_run_start = t;
      } else if (fan_run_start != 0) {
        if (t - fan_run_start > fan_longest_ms) fan_longest_ms = t - fan_run_start;
        fan_run_start = 0;
      }
      if (!in_band && fan_band_lost_t == 0) {
        fan_band_lost_t = t;
        print_both(String("[RESULT] Pointing left +/-") + String(FAN_HOLD_BAND_DEG, 0) + " deg " +
                   String((t - fan_on_t) / 1000.0, 1) + " s after the fans turned on (error " +
                   String(err_point > 900 ? 999.0 : err_point, 1) + " deg, wMeas " + String(w_meas, 0) + " RPM).");
      }
    }
    // With the fans on, a saturated wheel means pointing can't be held any more: report it, then end the test.
    if (fans_on && saturated && !fan_sat_reported) {
      fan_sat_reported = true;
      print_both(String("[RESULT] Wheel saturated ") + String((t - fan_on_t) / 1000.0, 1) +
                 " s after the fans turned on (wMeas " + String(w_meas, 0) + " RPM).");
    }
    if (fans_on && saturated_since != 0 && now - saturated_since > FAN_SAT_STOP_MS) {
      end_test(String("[INFO] Wheel saturated for ") + (FAN_SAT_STOP_MS / 1000) +
               " s with the fans on. Test ended. Wheel and fans stopped.", t);
      return;
    }

    // Command 10 (requirement demo): judge one requirement live, print PASS/FAIL, then end the test.
    if (demo_req != 0 && control_started && !acquiring) {
      bool in_target = fabs(err_target) <= FAN_HOLD_BAND_DEG;
      if (in_target) {
        if (demo_in_since == 0) { demo_in_since = now; demo_max_err = 0.0; demo_n = 0; demo_n5 = 0; }
        if (fabs(err_target) > demo_max_err) demo_max_err = fabs(err_target);
        demo_n++;
        if (fabs(err_target) <= 5.0) demo_n5++;
      } else {
        if (demo_in_since != 0 && demo_req == 1 && demo_step == 3)
          print_both(String("[DEMO] R1: left +/-10 deg (error ") + String(err_target, 1) + " deg). Hold timer restarted.");
        demo_in_since = 0;
      }
      uint32_t held_ms = in_target ? now - demo_in_since : 0;
      bool print_now = now >= demo_next_print;
      if (print_now) demo_next_print = now + DEMO_PRINT_MS;

      if (demo_req == 1) {
        // R2 then R1. When the scan found the sun, the array was sent 180 deg AWAY from it (demo_step 1).
        if (demo_step == 1) {
          // Step 1: get to 180 deg from the sun and sit there DEMO_R2_AWAY_MS.
          if (print_now) print_both(String("[DEMO] R2: turning 180 deg away from the sun (") + String(err_target, 1) +
                                    " deg from that aim point).");
          if (held_ms >= DEMO_R2_AWAY_MS) {
            print_both(String("[DEMO] R2: array is 180 deg from the sun. Timed slew back starts NOW (limit ") +
                       (DEMO_R2_LIMIT_MS / 1000) + " s).");
            demo_offset = 0.0;
            demo_step = 2;
            demo_slew_t = now;
            demo_in_since = 0;
          }
        } else if (demo_step == 2) {
          // Step 2 (R2): time the slew back to the sun.
          if (in_target && !demo_reached) {
            demo_reached = true;
            demo_reach_ms = now - demo_slew_t;
            print_both(String("[DEMO] R2: reached +/-10 deg ") + String(demo_reach_ms / 1000.0, 1) + " s after the slew started.");
          } else if (!in_target && demo_reached) {
            demo_reached = false;  // overshot back out: the reach time counts from the last entry
            print_both(String("[DEMO] R2: overshot out of +/-10 deg (error ") + String(err_target, 1) + " deg).");
          }
          if (held_ms >= DEMO_R2_CONFIRM_MS) {
            demo_r2_pass = demo_reach_ms <= DEMO_R2_LIMIT_MS;
            print_both(String("[DEMO] R2 ") + (demo_r2_pass ? "PASS" : "FAIL") + ": reached the sun from 180 deg in " +
                       String(demo_reach_ms / 1000.0, 1) + " s (limit " + (DEMO_R2_LIMIT_MS / 1000) +
                       " s) and stayed within +/-10 deg. Now holding for R1.");
            demo_step = 3;
          } else if (now - demo_slew_t > DEMO_R2_TIMEOUT_MS) {
            end_test(String("[DEMO] R2 FAIL: did not settle within +/-10 deg of the sun ") + (DEMO_R2_TIMEOUT_MS / 1000) +
                     " s after the slew started. Wheel stopped.", t);
            return;
          }
        } else if (demo_step == 3) {
          // Step 3 (R1): keep the array within +/-10 deg of the sun. The timer runs from the R2 arrival.
          if (print_now) print_both(String("[DEMO] R1: within +/-10 deg for ") + String(held_ms / 1000.0, 0) + " of " +
                                    (DEMO_R1_HOLD_MS / 1000) + " s (error now " + String(err_target, 1) + " deg).");
          if (held_ms >= DEMO_R1_HOLD_MS) {
            end_test(String("[DEMO] R2 ") + (demo_r2_pass ? "PASS" : "FAIL") + " (180 deg slew in " +
                     String(demo_reach_ms / 1000.0, 1) + " s, limit " + (DEMO_R2_LIMIT_MS / 1000) + " s). R1 PASS: held within +/-10 deg for " +
                     (DEMO_R1_HOLD_MS / 1000) + " s (max error " + String(demo_max_err, 1) + " deg, within +/-5 deg " +
                     String(demo_n > 0 ? 100.0 * demo_n5 / demo_n : 0.0, 0) + "% of the time). Wheel stopped.", t);
            return;
          }
        }
      } else if (demo_req == 2) {
        // R3: keep the pointing requirement with the disturbance fans on.
        if (!fans_on) {
          if (print_now) print_both(String("[DEMO] R3: fans turn on after ") + (FAN_HOLD_TIME_MS / 1000) +
                                    " s within +/-10 deg (within for " + String(held_ms / 1000.0, 0) + " s).");
        } else if (fan_band_lost_t != 0) {
          end_test(String("[DEMO] R3 FAIL: pointing left +/-10 deg ") + String((fan_band_lost_t - fan_on_t) / 1000.0, 1) +
                   " s after the fans turned on. Wheel and fans stopped.", t);
          return;
        } else {
          if (print_now) print_both(String("[DEMO] R3: fans on for ") + String((t - fan_on_t) / 1000.0, 0) + " of " +
                                    (DEMO_R3_FAN_MS / 1000) + " s, error " + String(err_target, 1) + " deg, wMeas " +
                                    String(w_meas, 0) + " RPM.");
          if (t - fan_on_t >= DEMO_R3_FAN_MS) {
            end_test(String("[DEMO] R3 PASS: held within +/-10 deg for ") + (DEMO_R3_FAN_MS / 1000) +
                     " s with the fans pushing (max error " + String(demo_max_err, 1) + " deg). Wheel and fans stopped.", t);
            return;
          }
        }
      } else if (demo_req == 3) {
        // S2: unload the wheel with the thrusters while keeping the pointing requirement.
        if (!fans_on) {
          if (print_now) print_both(String("[DEMO] S2: disturbance fans turn on after ") + (FAN_HOLD_TIME_MS / 1000) +
                                    " s within +/-10 deg (within for " + String(held_ms / 1000.0, 0) + " s).");
        } else if (fan_band_lost_t != 0) {
          end_test(String("[DEMO] S2 FAIL: pointing left +/-10 deg ") + String((fan_band_lost_t - fan_on_t) / 1000.0, 1) +
                   " s after the disturbance fans turned on (" + dump_count + " dumps finished). Wheel and fans stopped.", t);
          return;
        } else if (dump_count >= 1 && now - last_dump_end >= DEMO_S2_CONFIRM_MS) {
          end_test(String("[DEMO] S2 PASS: the thrusters unloaded the wheel in ") + String(dump_total_ms / 1000.0, 1) +
                   " s and pointing stayed within +/-10 deg the whole time (" + String((t - fan_on_t) / 1000.0, 0) +
                   " s with the fans on, max error " + String(demo_max_err, 1) + " deg). Wheel and fans stopped.", t);
          return;
        } else if (print_now) {
          print_both(String("[DEMO] S2: fans on ") + String((t - fan_on_t) / 1000.0, 0) + " s, wheel " + String(w_cmd_rpm, 0) +
                     " RPM" + (dumping ? " (THRUSTERS ON, dumping)" : String(" (dump at ") + String(DESAT_START_RPM, 0) + ")") +
                     ", error " + String(err_target, 1) + " deg.");
        }
      }
    }

    // Log to SD (same columns as the header row):
    dataFile.println(String(t) + "," + mode + "," + String(raw[SUN_PX]) + "," + String(raw[SUN_PY]) + "," +
                     String(raw[SUN_MX]) + "," + String(raw[SUN_MY]) + "," + String(sun.magnitude, 3) + "," +
                     String(sun.direction_deg, 2) + "," + String(err, 2) + "," + String(rate, 3) + "," +
                     String(mag_X, 2) + "," + String(mag_Y, 2) + "," + String(tau, 6) + "," +
                     String(w_cmd_rpm, 1) + "," + String(w_meas, 1) + "," + (saturated ? "1" : "0") + "," +
                     String(err_ctrl, 2) + "," + String(latch.dir) + "," + String(heading, 2) + "," +
                     String(sun_heading, 2) + "," + String(aim_offset, 1) + "," + String(mx_bright, 3) + "," +
                     String(fan_upper_pct, 0) + "," + String(fan_lower_pct, 0) + "," + (dumping ? "1" : "0"));

    // Telemetry (Name:value for GCS plotter): every point on USB, 2 Hz on XBee
    String s = "t:" + String(t) + ",err:" + String(err, 1) + ",latch:" + String(latch.dir) + ",hdg:" + String(heading, 1) +
               ",sunHdg:" + String(sun_heading, 1) + ",aim:" + String(aim_offset, 0) +
               ",bX:" + String(mx_bright, 2) + ",acq:" + String(acquiring ? 1 : 0) + ",gZ:" + String(rate, 2) +
               ",wCmd:" + String(w_cmd_rpm, 0) + ",wMeas:" + String(w_meas, 0) +
               ",sDir:" + String(sun.direction_deg, 1) +
               // sun sensor fields for the GCS Sun Map
               ",mag:" + String(sun.magnitude, 2) +
               ",n+X:" + String(sun.norm[SUN_PX], 2) + ",n+Y:" + String(sun.norm[SUN_PY], 2) +
               ",n-X:" + String(sun.norm[SUN_MX], 2) + ",n-Y:" + String(sun.norm[SUN_MY], 2) +
               ",fan27:" + String(fan_upper_pct, 0) + ",fan4:" + String(fan_lower_pct, 0) + ",dump:" + String(dumping ? 1 : 0);
    Serial.println(s);
    if (test_point_count % 10 == 0) {
      dataFile.flush(); // save file every 10 test points
      Xbee.println(s);
    }

    // End test if complete:
    if (t > duration_ms) {
      if (demo_req) end_test(String("[DEMO] ") + demo_name + " FAIL: ran out of time (" + (duration_ms / 1000) +
                             " s) before the requirement was shown. Wheel and fans stopped.", t);
      else          end_test("[INFO] Sun tracker test complete. Wheel and fans stopped.", t);
      return;
    }
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 5: Print / Set Tracker Parameters
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Print all sun tracker parameters to USB Serial and XBee.
 */
void sun_print_params() {
  print_both(String("kp=") + String(tracker_gains.Kp, 5) + " kd=" + String(tracker_gains.Kd, 5) +
             " db=" + String(tracker_gains.deadband_deg, 1) + " rate=" + String(tracker_gains.max_rate_dps, 0) +
             " bias=" + String(bias_pwm, 2) +
             " axis=" + String(array_axis_deg, 1) + " wsign=" + wheel_pwm_sign + " gsign=" + gyro_sign +
             " dur=" + (track_duration_ms / 1000));
  String cal = "cal dark/bright (+X,+Y,-X,-Y): ";
  for (int i = 0; i < 4; i++) cal += String(sun_cal.dark[i], 0) + "/" + String(sun_cal.bright[i], 0) + " ";
  print_both(cal);
}

/**
 * @brief Change sun tracker parameters over XBee/USB without reflashing.
 *        Send lines like "kp 0.004". Send "done" (or 'X') to finish.
 *        Changes last until reboot; copy good values into definitions.h.
 */
void sun_set_params() {
  sun_print_params();
  print_both("[INFO] Send '<name> <value>' (names: kp kd db rate bias axis wsign gsign dur). Send 'done' to finish.");
  while (true) {
    String line = wait_for_line();
    line.toLowerCase();
    if (line == "done" || line == "x" || line.length() == 0) break;

    int space = line.indexOf(' ');
    if (space < 0) { print_both("[CAUTION] Format is '<name> <value>', e.g. 'kp 0.004'."); continue; }
    String name = line.substring(0, space);
    float value = line.substring(space + 1).toFloat();

    if      (name == "kp")    tracker_gains.Kp = value;
    else if (name == "kd")    tracker_gains.Kd = value;
    else if (name == "db")    tracker_gains.deadband_deg = value;
    else if (name == "rate")  tracker_gains.max_rate_dps = value;
    else if (name == "bias")  bias_pwm = constrain(value, -0.9, 0.9);
    else if (name == "axis")  array_axis_deg = value;
    else if (name == "wsign") wheel_pwm_sign = (value < 0) ? -1 : 1;
    else if (name == "gsign") gyro_sign = (value < 0) ? -1 : 1;
    else if (name == "dur")   track_duration_ms = (uint32_t)(constrain(value, 30, 3600) * 1000); // seconds
    else { print_both("[CAUTION] Unknown parameter: " + name); continue; }
    sun_print_params();
  }
  print_both("[INFO] Parameters set.");
}

/*---------------------------------------------------------------------------------------------*/
// Shared helper for commands 6 and 7
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Take n sun sensor readings and return the mean and standard deviation of each
 *        channel (order +X, +Y, -X, -Y, raw ADC counts).
 */
void sample_sun_stats(int n, float mean[4], float sd[4]) {
  float sum[4] = {0, 0, 0, 0};
  float sumsq[4] = {0, 0, 0, 0};
  float raw[4];
  for (int k = 0; k < n; k++) {
    read_sun_raw(raw);
    for (int i = 0; i < 4; i++) {
      sum[i] += raw[i];
      sumsq[i] += raw[i] * raw[i];
    }
    delay(5);
  }
  for (int i = 0; i < 4; i++) {
    mean[i] = sum[i] / n;
    float var = sumsq[i] / n - mean[i] * mean[i];
    sd[i] = (var > 0) ? sqrt(var) : 0.0;
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 6: Attitude Determination Check
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Guided checkout of the sun sensor and gyro (wheel off). Prints PASS/FAIL for each step
 *        and logs the sun sensor steps to ADCheck_dataNNN.csv.
 *
 * Steps (user sends any key to advance, 's' to skip a step, 'X' to cancel):
 *  1. Light off      -> every channel should read low and steady, no "sun" seen
 *  2. Light on 8 spots (4 faces + 4 corners, every 45 deg) -> sun direction within AD_TOL_DEG
 *  3. KestrelSAT still for 3 s -> gyro bias and noise
 *  4. Light fixed, user turns KestrelSAT ~90 deg by hand -> gyro angle must match sun sensor angle
 *
 * Run command 3 (calibrate) first if the sun sensor wiring changed.
 */
void ad_check() {
  const float AD_TOL_DEG = 15.0;   // sun direction must be within this of expected
  const int n_samples = 50;
  const char *spot_names[8] = {"+X face", "+X/+Y corner", "+Y face", "+Y/-X corner",
                               "-X face (solar array)", "-X/-Y corner", "-Y face", "-Y/+X corner"};
  const char *ch_names[4] = {"+X", "+Y", "-X", "-Y"};
  float mean[4], sd[4];
  int n_pass = 0, n_fail = 0;
  char file_name[40];

  driver.setOutput(0);
  if (sd_createDataFile(&dataFile, "ADCheck")) {
    dataFile.println("step,expected(deg),measured(deg),error(deg),sun_mag,"
                     "sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),result");
    dataFile.flush();
    dataFile.getName(file_name, sizeof(file_name));
    print_both(String("[INFO] Data file created successfully: ") + file_name);
  } else {
    print_both("[CAUTION] Failed to create data file. Running the check without logging.");
    file_name[0] = '\0';
  }

  print_both("[INFO] ATTITUDE DETERMINATION CHECK (wheel off). Any key = go, 's' = skip step, 'X' = cancel.");
  print_both("[INFO] If you changed the sun sensor wiring, run 3 (calibrate) first.");

  // ---------- Step 1: light off ----------
  print_both("[INFO] Step 1: turn OFF the light source, then send any key.");
  String line = wait_for_line();
  if (line.equalsIgnoreCase("x")) { if (file_name[0]) dataFile.close(); driver.setOutput(0); print_both("[INFO] Check canceled."); return; }
  if (!line.equalsIgnoreCase("s")) {
    sample_sun_stats(n_samples, mean, sd);
    SunReading sun = compute_sun(mean, sun_cal);
    bool ok = true;
    for (int i = 0; i < 4; i++) {
      String msg = String("  ") + ch_names[i] + ": " + String(mean[i], 0) + " counts (noise " + String(sd[i], 1) + ")";
      if (mean[i] > 4000) { msg += "  <-- reads FULL SCALE in the dark: output shorted to 3.3V or resistor missing?"; ok = false; }
      if (sd[i] > 50)     { msg += "  <-- very noisy: loose wire, floating pin, or missing ground?"; ok = false; }
      print_both(msg);
    }
    if (sun.magnitude >= SUN_LOST_THRESHOLD) {
      print_both(String("  Sun magnitude ") + String(sun.magnitude, 2) + " with the light off: room light looks like sun. Recalibrate (3) in these room conditions.");
      ok = false;
    }
    print_both(String("[RESULT] Step 1 dark check: ") + (ok ? "PASS" : "FAIL"));
    ok ? n_pass++ : n_fail++;
    if (file_name[0]) {
      dataFile.println(String("DARK,,,,") + String(sun.magnitude, 3) + "," + String(mean[0], 1) + "," + String(mean[1], 1) +
                       "," + String(mean[2], 1) + "," + String(mean[3], 1) + "," + (ok ? "PASS" : "FAIL"));
      dataFile.flush();
    }
  }

  // ---------- Step 2: light on 8 spots ----------
  for (int k = 0; k < 8; k++) {
    float expected = k * 45.0;
    print_both(String("[INFO] Step 2.") + (k + 1) + ": light ON, pointed at the " + spot_names[k] +
               " (expect " + String(expected, 0) + " deg). Send any key ('s' to skip).");
    line = wait_for_line();
    if (line.equalsIgnoreCase("x")) { if (file_name[0]) dataFile.close(); driver.setOutput(0); print_both("[INFO] Check canceled."); return; }
    if (line.equalsIgnoreCase("s")) continue;

    sample_sun_stats(n_samples, mean, sd);
    SunReading sun = compute_sun(mean, sun_cal);
    float err = wrap180(sun.direction_deg - expected);
    bool ok = (fabs(err) <= AD_TOL_DEG) && (sun.magnitude >= SUN_LOST_THRESHOLD);

    print_both(String("  sDir:") + String(sun.direction_deg, 1) + " err:" + String(err, 1) + " mag:" + String(sun.magnitude, 2) +
               "  raw(+X,+Y,-X,-Y): " + String(mean[0], 0) + "," + String(mean[1], 0) + "," + String(mean[2], 0) + "," + String(mean[3], 0) +
               "  n: " + String(sun.norm[0], 2) + "," + String(sun.norm[1], 2) + "," + String(sun.norm[2], 2) + "," + String(sun.norm[3], 2));
    if (sun.magnitude < SUN_LOST_THRESHOLD) print_both("  No sun seen: is the light on / close enough?");
    for (int i = 0; i < 4; i++) {
      if (mean[i] > 4090) print_both(String("  ") + ch_names[i] + " is SATURATED (4095): try a smaller resistor or move the light back.");
    }
    if (k % 2 == 0) { // on a face: that face should be the brightest channel
      int face_ch[4] = {SUN_PX, SUN_PY, SUN_MX, SUN_MY};
      int ch = face_ch[k / 2];
      for (int i = 0; i < 4; i++) {
        if (i != ch && sun.norm[i] > sun.norm[ch]) {
          print_both(String("  ") + ch_names[i] + " is brighter than " + ch_names[ch] + ": wires swapped between pins?");
          ok = false;
        }
      }
    }
    print_both(String("[RESULT] Step 2.") + (k + 1) + " " + spot_names[k] + ": " + (ok ? "PASS" : "FAIL") +
               " (|err| " + String(fabs(err), 1) + " deg, limit " + String(AD_TOL_DEG, 0) + ")");
    ok ? n_pass++ : n_fail++;
    if (file_name[0]) {
      dataFile.println(String("SPOT_") + String(expected, 0) + "," + String(expected, 1) + "," + String(sun.direction_deg, 1) + "," +
                       String(err, 1) + "," + String(sun.magnitude, 3) + "," + String(mean[0], 1) + "," + String(mean[1], 1) + "," +
                       String(mean[2], 1) + "," + String(mean[3], 1) + "," + (ok ? "PASS" : "FAIL"));
      dataFile.flush();
    }
  }

  // ---------- Step 3: gyro bias ----------
  float gyro_bias = 0.0;
  print_both("[INFO] Step 3: hold KestrelSAT completely STILL (or set it on the table), then send any key. Takes 3 s.");
  line = wait_for_line();
  if (line.equalsIgnoreCase("x")) { if (file_name[0]) dataFile.close(); driver.setOutput(0); print_both("[INFO] Check canceled."); return; }
  if (!line.equalsIgnoreCase("s")) {
    float sum = 0, sumsq = 0;
    int n = 0;
    uint32_t t_end = millis() + 3000;
    while (millis() < t_end) {
      float r = read_body_rate_dps();
      sum += r;
      sumsq += r * r;
      n++;
      delay(20);
    }
    gyro_bias = sum / n;
    float var = sumsq / n - gyro_bias * gyro_bias;
    float noise = (var > 0) ? sqrt(var) : 0.0;
    bool ok = (fabs(gyro_bias) < 2.0) && (noise < 1.0) && (noise > 0.0);
    print_both(String("  gyro Z bias: ") + String(gyro_bias, 3) + " deg/s, noise: " + String(noise, 3) + " deg/s (" + n + " samples)");
    if (noise == 0.0) print_both("  Gyro reading never changed: IMU frozen or not updating?");
    print_both(String("[RESULT] Step 3 gyro still: ") + (ok ? "PASS" : "FAIL") + " (limits: |bias| < 2, noise < 1 deg/s)");
    ok ? n_pass++ : n_fail++;
  }

  // ---------- Step 4: hand rotation, gyro vs sun sensor ----------
  print_both("[INFO] Step 4: light ON and FIXED, pointed at the -X face. Send any key, then slowly turn KestrelSAT by hand");
  print_both("[INFO]   about 90 deg counter-clockwise seen from above (light should end on the +Y face), then send any key.");
  line = wait_for_line();
  if (line.equalsIgnoreCase("x")) { if (file_name[0]) dataFile.close(); driver.setOutput(0); print_both("[INFO] Check canceled."); return; }
  if (!line.equalsIgnoreCase("s")) {
    float raw[4];
    read_sun_raw(raw);
    SunReading sun0 = compute_sun(raw, sun_cal);
    float gyro_angle = 0.0;
    uint32_t t_last = millis();
    uint32_t t_print = t_last;
    print_both("[INFO] Turn now. Send any key when done.");
    while (!Serial.available() && !Xbee.available()) {
      uint32_t now = millis();
      if (now - t_last < 20) continue;
      gyro_angle += (read_body_rate_dps() - gyro_bias) * (now - t_last) / 1000.0;
      t_last = now;
      if (now - t_print > 500) { t_print = now; print_both(String("  gyro angle: ") + String(gyro_angle, 1) + " deg"); }
    }
    wait_for_line(); // clear the key
    read_sun_raw(raw);
    SunReading sun1 = compute_sun(raw, sun_cal);
    // A fixed light moves the OPPOSITE way in the body frame when the body turns.
    float sun_turn = -wrap180(sun1.direction_deg - sun0.direction_deg);
    float diff = gyro_angle - sun_turn;
    bool ok = (fabs(diff) <= AD_TOL_DEG) && (fabs(gyro_angle) > 30.0);
    print_both(String("  body turn from gyro: ") + String(gyro_angle, 1) + " deg, from sun sensor: " + String(sun_turn, 1) +
               " deg (sDir " + String(sun0.direction_deg, 1) + " -> " + String(sun1.direction_deg, 1) + ")");
    if (fabs(gyro_angle) <= 30.0) print_both("  Gyro saw less than 30 deg of turn: turn further, or gyro not working.");
    else if (gyro_angle * sun_turn < 0) print_both("  Gyro and sun sensor turned OPPOSITE ways: gyro_sign is flipped (5 -> gsign) or sun sensor pins swapped.");
    print_both(String("[RESULT] Step 4 gyro vs sun sensor: ") + (ok ? "PASS" : "FAIL") + " (difference " + String(diff, 1) +
               " deg, limit " + String(AD_TOL_DEG, 0) + ")");
    ok ? n_pass++ : n_fail++;
    if (file_name[0]) {
      dataFile.println(String("ROTATION,") + String(gyro_angle, 1) + "," + String(sun_turn, 1) + "," + String(diff, 1) + "," +
                       String(sun1.magnitude, 3) + "," + String(raw[0], 1) + "," + String(raw[1], 1) + "," + String(raw[2], 1) + "," +
                       String(raw[3], 1) + "," + (ok ? "PASS" : "FAIL"));
    }
  }

  driver.setOutput(0);
  if (file_name[0]) dataFile.close();
  print_both(String("[RESULT] ATTITUDE DETERMINATION CHECK: ") + n_pass + " passed, " + n_fail + " failed." +
             (file_name[0] ? String(" File ") + file_name + " closed." : String("")));
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 7: Phototransistor Resistor Sweep
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Record sun sensor brightness for different phototransistor resistor values.
 *        Logs to PTSweep_dataNNN.csv; plot with 2. THEORY/plot_ptsweep.py.
 *
 * For each resistor value (the user swaps the resistors by hand):
 *  1. User sends the resistor value in ohms (e.g. "4700", "10k", "1M")
 *  2. Light off               -> DARK row (all 4 channels)
 *  3. Light on each face      -> LIGHT_+X, LIGHT_+Y, LIGHT_-X, LIGHT_-Y rows (all 4 channels)
 * Send 'done' at the resistor prompt to finish. 's' skips a step, 'X' cancels.
 *
 * Keep the light at the SAME distance and angle for every resistor so only R changes.
 */
void pt_sweep() {
  const int n_samples = 200;
  const char *ch_names[4] = {"+X", "+Y", "-X", "-Y"};
  const char *conditions[5] = {"DARK", "LIGHT_+X", "LIGHT_+Y", "LIGHT_-X", "LIGHT_-Y"};
  const char *header = "resistor(ohm),condition,mean_plusX(count),mean_plusY(count),mean_minusX(count),mean_minusY(count),"
                       "std_plusX(count),std_plusY(count),std_minusX(count),std_minusY(count)";
  float mean[4], sd[4];
  char file_name[40];

  driver.setOutput(0);
  if (sd_createDataFile(&dataFile, "PTSweep")) {
    dataFile.println(header);
    dataFile.flush();
    dataFile.getName(file_name, sizeof(file_name));
    print_both(String("[INFO] Data file created successfully: ") + file_name);
  } else {
    print_both("[CAUTION] Failed to create data file. Rows will only be printed; copy them from the Serial monitor.");
    file_name[0] = '\0';
  }

  print_both("[INFO] PHOTOTRANSISTOR RESISTOR SWEEP. Keep the light at the SAME distance/angle for every resistor.");
  print_both(header);

  while (true) {
    print_both("[INFO] Install the resistor(s), then send the value in ohms (e.g. 4700, 10k, 1M). Send 'done' to finish.");
    String line = wait_for_line();
    line.toLowerCase();
    if (line == "done" || line == "x") break;
    float r_ohm = line.toFloat();
    if (line.endsWith("k")) r_ohm *= 1000.0;
    if (line.endsWith("m")) r_ohm *= 1000000.0;
    if (r_ohm <= 0) { print_both("[CAUTION] Could not read a resistor value. Example: 10k"); continue; }

    float dark[4] = {NAN, NAN, NAN, NAN};
    float bright[4] = {NAN, NAN, NAN, NAN};
    float bright_sd[4] = {NAN, NAN, NAN, NAN};
    bool canceled = false;
    for (int c = 0; c < 5; c++) {
      if (c == 0) print_both(String("[INFO] R = ") + String(r_ohm, 0) + " ohm. Turn the light OFF, then send any key ('s' to skip).");
      else        print_both(String("[INFO] Light ON, pointed straight at the ") + ch_names[c - 1] + " face, then send any key ('s' to skip).");
      String key = wait_for_line();
      if (key.equalsIgnoreCase("x")) { canceled = true; break; }
      if (key.equalsIgnoreCase("s")) continue;

      sample_sun_stats(n_samples, mean, sd);
      String row = String(r_ohm, 0) + "," + conditions[c];
      for (int i = 0; i < 4; i++) row += "," + String(mean[i], 1);
      for (int i = 0; i < 4; i++) row += "," + String(sd[i], 2);
      print_both(row);
      if (file_name[0]) { dataFile.println(row); dataFile.flush(); }

      if (c == 0) for (int i = 0; i < 4; i++) dark[i] = mean[i];
      else { bright[c - 1] = mean[c - 1]; bright_sd[c - 1] = sd[c - 1]; }
    }
    if (canceled) break;

    // Quick summary for this resistor: brightness span of each channel
    for (int i = 0; i < 4; i++) {
      if (isnan(bright[i])) continue;
      String s = String("  ") + ch_names[i] + ": bright " + String(bright[i], 0);
      if (!isnan(dark[i])) s += " dark " + String(dark[i], 0) + " span " + String(bright[i] - dark[i], 0);
      s += " noise " + String(bright_sd[i], 1);
      if (bright[i] > 4000) s += "  SATURATED: R too big (or light too close)";
      print_both(s);
    }
  }

  driver.setOutput(0);
  if (file_name[0]) {
    dataFile.close();
    print_both(String("[INFO] Sweep finished. File ") + file_name + " closed. Plot it with: python \"2. THEORY/plot_ptsweep.py\" " + file_name);
  } else {
    print_both("[INFO] Sweep finished.");
  }
  print_both("[INFO] After installing the resistor you pick, re-run 3 (calibrate) and paste the new sun_cal line.");
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Disturbance Fans (used by command 8)
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Set the PWM duty of the two fan pairs right now (0-100%). Fans can't run backwards,
 *        so negative values are treated as 0. Updates fan_upper_pct / fan_lower_pct.
 *        With FAN_PWM_INVERTED the GPIO duty is flipped, because the transistor in front of each
 *        XY-MOS trigger turns the fans on when the GPIO is low.
 */
void set_fans(float upper_pct, float lower_pct) {
  fan_upper_pct = constrain(upper_pct, 0.0, 100.0);
  fan_lower_pct = constrain(lower_pct, 0.0, 100.0);
  const float full = (1 << FAN_PWM_BITS) - 1;  // ledcWrite treats this as 100% (pin stays high)
  uint32_t upper = (uint32_t)(fan_upper_pct / 100.0 * full + 0.5);
  uint32_t lower = (uint32_t)(fan_lower_pct / 100.0 * full + 0.5);
  if (FAN_PWM_INVERTED) {  // transistor flips the signal: GPIO high = fans off
    upper = (uint32_t)full - upper;
    lower = (uint32_t)full - lower;
  }
  ledcWrite(FAN_UPPER_LEDC_CH, upper);
  ledcWrite(FAN_LOWER_LEDC_CH, lower);
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 8: Fan Hold Test
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Sun tracker (command 4) with a disturbance: once pointing has stayed within FAN_HOLD_BAND_DEG
 *        for FAN_HOLD_TIME_MS, one fan pair turns on and the reaction wheel has to fight it.
 *        Asks for the direction ('cw' = GPIO 27 pair, 'ccw' = GPIO 4 pair, seen from above) and an
 *        optional duty (default 100%), e.g. "ccw" or "cw 60". Prints when pointing first leaves the band
 *        and when the wheel saturates; logs to SunTrack_dataNNN.csv like command 4.
 */
void fan_hold_test() {
  print_both("[INFO] Fan hold test. Send the fan direction seen from above: 'cw' or 'ccw' (optional duty, e.g. 'cw 60'; default 100%). 'X' cancels.");
  String line = wait_for_line();
  line.toLowerCase();
  if (line == "x" || line.length() == 0) { print_both("[INFO] Test canceled."); return; }
  int space = line.indexOf(' ');
  String dir = (space < 0) ? line : line.substring(0, space);
  float duty = (space < 0) ? 100.0 : constrain(line.substring(space + 1).toFloat(), 0.0, 100.0);
  if (dir != "cw" && dir != "ccw") { print_both("[CAUTION] Send 'cw' or 'ccw'. Test canceled."); return; }
  if (duty <= 0) { print_both("[CAUTION] Duty must be above 0. Test canceled."); return; }

  print_both(String("[INFO] ") + (dir == "cw" ? "Clockwise (GPIO 27 pair)" : "Counter-clockwise (GPIO 4 pair)") +
             " at " + String(duty, 0) + "%.");
  if (dir == "cw") sun_track(duty, 0.0);
  else             sun_track(0.0, duty);
}

/*---------------------------------------------------------------------------------------------*/
// Fan choice helper (commands 10 and 11)
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Read a fan pair and duty from words like "ccw", "cw 60" (same words as command 8).
 * @param words         the text after the command (may be empty: keeps the defaults)
 * @param default_duty  duty (%) if none is given
 * @param dir           in: default direction; out: "cw" (GPIO 27 pair) or "ccw" (GPIO 4 pair)
 * @param duty          out: duty (0-100%)
 * @return false (and prints why) if the words don't make sense
 */
bool parse_fan_choice(String words, float default_duty, String &dir, float &duty) {
  words.trim();
  duty = default_duty;
  if (words.length() > 0) {
    int space = words.indexOf(' ');
    dir = (space < 0) ? words : words.substring(0, space);
    if (space > 0) duty = constrain(words.substring(space + 1).toFloat(), 0.0, 100.0);
  }
  if (dir != "cw" && dir != "ccw") { print_both("[CAUTION] Fan direction must be 'cw' or 'ccw'."); return false; }
  if (duty <= 0) { print_both("[CAUTION] Duty must be above 0."); return false; }
  return true;
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 10: Requirement Demo (final presentation)
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Show the final requirements working, live, with a PASS/FAIL at the end. Runs the sun tracker
 *        (command 4: hold, release on BLUE, scan) and then judges the option picked:
 *  - 1: R2 + R1 (one run). As soon as the scan finds the sun, the array turns 180 deg AWAY from it and sits
 *       there DEMO_R2_AWAY_MS. Then the timed slew back: R2 PASS if it reaches +/-10 deg within DEMO_R2_LIMIT_MS
 *       and stays there DEMO_R2_CONFIRM_MS. It keeps holding: R1 PASS once the array has stayed within
 *       +/-10 deg for DEMO_R1_HOLD_MS (also prints how much of that time was within +/-5 deg).
 *  - 2: R3 disturbance. Fans turn on after FAN_HOLD_TIME_MS on target; PASS if pointing stays within
 *       +/-10 deg for DEMO_R3_FAN_MS with them on. Send "2" (CCW pair, 100%) or e.g. "2 cw", "2 ccw 60".
 *  - 3: S2 desaturation (stretch). Command 11 (disturbance + thrusters); PASS once one momentum dump has
 *       finished and pointing has stayed within +/-10 deg the whole time since the fans turned on, plus
 *       DEMO_S2_CONFIRM_MS. Send "3" (CCW disturbance, 70%) or e.g. "3 cw", "3 ccw 60".
 *  Logs to SunTrack_dataNNN.csv like command 4. 'X' stops at any time.
 */
void req_demo() {
  print_both("[INFO] Requirement demo. Send 1 (R1/R2: turn 180 deg away, slew back in 10 s, hold within 10 deg), "
             "2 (R3 hold against the fans; '2 cw' / '2 ccw 60', default ccw 100%), "
             "or 3 (S2 desaturate with thrusters; '3 cw' / '3 ccw 60', default ccw 70%). 'X' cancels.");
  String line = wait_for_line();
  line.toLowerCase();
  if (line == "x" || line.length() == 0) { print_both("[INFO] Demo canceled."); return; }
  int req = line.toInt();
  int space = line.indexOf(' ');
  String rest = (space > 0) ? line.substring(space + 1) : String("");

  if (req == 1) {
    print_both(String("[DEMO] R2: from 180 deg off, the array shall reach the Sun vector within ") + (DEMO_R2_LIMIT_MS / 1000) +
               " s. R1: then stay within +/-10 deg (PASS after " + (DEMO_R1_HOLD_MS / 1000) +
               " s in a row). After the scan KestrelSAT turns itself 180 deg away first.");
    sun_track(0.0, 0.0, 1);
  } else if (req == 2) {
    String dir = "ccw";
    float duty;
    if (!parse_fan_choice(rest, DEMO_R3_DEFAULT_DUTY, dir, duty)) { print_both("[INFO] Demo canceled."); return; }
    print_both(String("[DEMO] R3: the array shall stay within +/-10 deg with the ") +
               (dir == "cw" ? "clockwise (GPIO 27)" : "counter-clockwise (GPIO 4)") + " fans at " + String(duty, 0) +
               "% (about 2.4 mN*m at 100%; the 2 N*m in the requirement is not possible at this scale). PASS after " +
               (DEMO_R3_FAN_MS / 1000) + " s with the fans on.");
    if (dir == "cw") sun_track(duty, 0.0, 2);
    else             sun_track(0.0, duty, 2);
  } else if (req == 3) {
    String dir = "ccw";
    float duty;
    if (!parse_fan_choice(rest, DESAT_DIST_DEFAULT_PCT, dir, duty)) { print_both("[INFO] Demo canceled."); return; }
    print_both(String("[DEMO] S2: the thrusters shall unload the reaction wheel while the array stays within +/-10 deg. ") +
               "Disturbance: " + (dir == "cw" ? "clockwise (GPIO 27)" : "counter-clockwise (GPIO 4)") + " at " + String(duty, 0) +
               "%; the other pair is the thruster. PASS after one dump plus " + (DEMO_S2_CONFIRM_MS / 1000) + " s.");
    if (dir == "cw") sun_track(duty, 0.0, 3, true);
    else             sun_track(0.0, duty, 3, true);
  } else {
    print_both("[CAUTION] Send 1, 2 or 3. Demo canceled.");
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 11: Momentum Dump
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Show the reaction wheel being desaturated by "thrusters" while pointing is held.
 *        Runs the sun tracker (command 4: hold, release on BLUE, scan, face the sun). After FAN_HOLD_TIME_MS
 *        on target, one fan pair turns on as a constant disturbance and the wheel spins up to fight it.
 *        When |wheel command| reaches DESAT_START_RPM, the OTHER pair fires at DESAT_THRUSTER_PCT against the
 *        disturbance, so the controller spins the wheel back down; the thrusters stop below DESAT_STOP_RPM.
 *        This repeats until 'X' (or FAN_HOLD_DURATION_MS). The summary says how many dumps finished and
 *        how long pointing stayed within +/-10 deg (compare with command 8, where the wheel saturates).
 *        Asks for the DISTURBANCE pair and duty, e.g. "ccw" or "cw 60" (default DESAT_DIST_DEFAULT_PCT).
 *        Keep the disturbance below 100%: two equal fans at 100% cancel and the wheel never unloads.
 *        Logs to SunTrack_dataNNN.csv like command 4 (the "dumping" column is 1 while the thrusters fire).
 */
void momentum_dump_test() {
  print_both(String("[INFO] Momentum dump. Send the DISTURBANCE fan direction seen from above: 'cw' or 'ccw' ") +
             "(optional duty, e.g. 'cw 60'; default " + String(DESAT_DIST_DEFAULT_PCT, 0) +
             "%). The other pair is the thruster. 'X' cancels.");
  String line = wait_for_line();
  line.toLowerCase();
  if (line == "x" || line.length() == 0) { print_both("[INFO] Test canceled."); return; }
  String dir = "ccw";
  float duty;
  if (!parse_fan_choice(line, DESAT_DIST_DEFAULT_PCT, dir, duty)) { print_both("[INFO] Test canceled."); return; }
  if (duty >= DESAT_THRUSTER_PCT) {
    print_both("[CAUTION] Disturbance at 100% cancels the 100% thrusters, so the wheel will not unload. Try 'ccw 70'.");
  }
  print_both(String("[INFO] Disturbance: ") + (dir == "cw" ? "clockwise (GPIO 27 pair)" : "counter-clockwise (GPIO 4 pair)") +
             " at " + String(duty, 0) + "%. Thrusters: " + (dir == "cw" ? "GPIO 4 pair" : "GPIO 27 pair") + ".");
  if (dir == "cw") sun_track(duty, 0.0, 0, true);
  else             sun_track(0.0, duty, 0, true);
}
