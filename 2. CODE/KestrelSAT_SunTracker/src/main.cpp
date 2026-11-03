/**
 * @file main.cpp
 * @brief Main control program for KestrelSAT attitude determination and control system
 * 
 * @details This is firmware for the USAFA Astro 331 KestrelSAT, implementing:
 * - Lab 6 Attitude Determination capabilities using IMU (gyroscope, magnetometer) and sun sensors
 * - Lab 7 Attitude Control via reaction wheel motor with encoder feedback
 * - Ground station command interface via XBee radio and USB Serial
 * - Battery telemetry monitoring via MAX17048 fuel gauge
 * - Data logging to SD card for Lab 6 and Lab 7 experiments
 * 
 * @author Lt Col Wyatt Harris
 * @date Spring 2026
 * @version 2.0
 * 
 * @section hardware Hardware Requirements
 * - ESP32 microcontroller (SparkFun Thing Plus ESP32-WROOM)
 * - ICM-20948 9-DoF IMU (via I2C at 0x36)
 * - MAX17048 Fuel Gauge (via I2C at 0x36)
 * - Pololu TB9051FTG Motor Driver
 * - DC motor with quadrature encoder
 * - Sun sensor (4-channel analog inputs)
 * - XBee radio module (UART at 9600 baud)
 * - SD card module (SPI)
 * - WS2812 RGB LED and digital status LED
 * 
 * @section dependencies External Libraries
 * - greiman/SdFat@^2.3.1 - SD card file system
 * - sparkfun/SparkFun MAX1704x Fuel Gauge@^1.0.3 - Battery monitoring
 * - sparkfun/SparkFun 9DoF IMU (ICM 20948)@^1.3.2 - IMU sensor interface
 * - meelonusk/TB9051FTGMotorCarrier@^1.0.2 - Motor driver control
 * - madhephaestus/ESP32Encoder@^0.12.0 - Quadrature encoder reading
 * 
 * @section commands Ground Station Commands
 * | Cmd | Function |
 * |-----|----------|
 * | 0   | Stop reaction wheel |
 * | 1   | Display options menu |
 * | 2   | Query XBee RSSI |
 * | 3   | Toggle status LED |
 * | 4   | Get battery telemetry (V, SOC, charge rate) |
 * | 5   | Manually set reaction wheel throttle (-100 to 100%) |
 * | 6   | Run Lab 6 test (Attitude Determination) |
 * | 7   | Run Lab 7 Test A (tabletop static, RW torque measurement) |
 * | 8   | Run Lab 7 Test B (dynamic test with prescribed wheel speed) |
 * | 9   | Stream reaction wheel speed |
 * | 10  | Lab 8: Calibrate sun sensor |
 * | 11  | Lab 8: Stream sun sensor (direction, pointing error) |
 * | 12  | Lab 8: Sign check (wheel pulse vs gyro & sun sensor) |
 * | 13  | Lab 8: Run sun tracker (closed-loop, logs to SD) |
 * | 14  | Lab 8: Print / set sun tracker parameters |
 * | 15  | Lab 8: Attitude determination check (sun sensor + gyro, PASS/FAIL) |
 * | 16  | Lab 8: Phototransistor resistor sweep (logs to SD) |
 * | 17  | Lab 8: Disturbance fans (set speeds for command 13, bench test) |
 * | 18  | Lab 8: Fan torque sweep (0-100% each pair, measures torque, logs to SD) |
 * | 19  | Lab 8: Fan hold test (sun tracker + fans once on target; time to lose pointing / saturate) |
 * | 98  | SD Card: List files (USB Serial only) |
 * | 99  | SD Card: Print file menu (USB Serial only) |
 * 
 * @section testing Test Modes
 * - **Lab 6**: Collects IMU (gyro Z, mag X/Y) and sun sensor data at 50ms intervals
 * - **Lab 7A**: 15-second tabletop test with 100% motor step input from 3-10 seconds
 * - **Lab 7B**: 50-second dynamic test with prescribed motor speed profile (hold, ramp up/down cycles)
 * 
 * @section datalog Data Logging Format
 * All test data is logged to SD card in CSV format with:
 * - MCU timestamp (ms)
 * - Gyroscope Z-axis (deg/s)
 * - Magnetometer X, Y axes (µT)
 * - Sun direction (degrees)
 * - Sun sensor raw values (4 channels)
 * - Reaction wheel commanded and measured speed (RPM)
 * 
 * @section communication Communication
 * - **USB Serial**: 115200 baud for USB debugging/control
 * - **XBee UART**: 9600 baud for wireless ground station link
 * - **I2C**: Sensor communication at standard rate
 * - **SPI**: SD card access
 * 
 * @note Heartbeat signal sent every 2 seconds to indicate program status
 * @note Ground station commands checked every 10ms
 * @note RGB LED provides visual status: Red (startup), Green (idle), Orange (Lab 6), Cyan (Lab 7A), Magenta (Lab 7B)
 * 
 * @warning All test loops use blocking serial reads; may delay program execution if waiting for input
 * @warning Reaction wheel tests should be conducted with appropriate safety precautions
 * 
 * @see definitions.h for hardware pin assignments and constants
 * @see sd_functions.h for SD card utility functions
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
float sun_X = 0.0, sun_Y = 0.0, sun_direction = 0.0;
float S_mag;
int16_t n_sun_sensor_reads = 5; // number of readings to average for sun sensor test point

uint32_t timeNext_testPoint; // time of next test point (ms)
uint32_t interval_testPoint = 50; // time interval between test points (ms)

float gyro_Z = 0.0;
float mag_X = 0.0;
float mag_Y = 0.0;

// Lab 8 Sun Tracker:
SunCalibration sun_cal = {{0.0, 34.6, 0.0, 0.0}, {2086.7, 1680.1, 1777.8, 2771.7}}; // 30 Oct, 3.3k resistors (command 10, +/-Y entries re-ordered for the A1=+Y / A3=-Y pins)
TrackerGains tracker_gains = {DEFAULT_KP, DEFAULT_KD, POINTING_DEADBAND_DEG, DEFAULT_MAX_RATE_DPS};
WheelLimits wheel_limits = {WHEEL_MIN_RPM, WHEEL_MAX_RPM, WHEEL_MAX_ACCEL_RPM_S};
float array_axis_deg = SOLAR_ARRAY_AXIS_DEG;
float bias_pwm = DEFAULT_BIAS_PWM;
int wheel_pwm_sign = DEFAULT_WHEEL_PWM_SIGN;
int gyro_sign = DEFAULT_GYRO_SIGN;
uint32_t track_duration_ms = TRACK_DURATION_MS; // command 13 length, set with 'dur' in command 14
// Disturbance fans (command 17):
float fan_test_upper_pct = 0.0;  // duty command 13 turns the GPIO 27 pair on to
float fan_test_lower_pct = 0.0;  // duty command 13 turns the GPIO 4 pair on to
float fan_upper_pct = 0.0;       // duty the GPIO 27 pair is running at right now
float fan_lower_pct = 0.0;       // duty the GPIO 4 pair is running at right now
uint32_t fan_freq_hz = FAN_PWM_FREQ_HZ;

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see defintiions after loop()):
/*---------------------------------------------------------------------------------------------*/
int get_command_from_ground_station();
void process_main_menu();
void get_sat_rssi();
void send_battery_telemetry();
void print_both(const String &msg);
void manual_set_RW_speed();
void lab6_run_test();
void lab7_run_test_A();
void lab7_run_test_B();
float set_speed_test_B(uint32_t t0);
void stream_RWspeed();
// Lab 8 Sun Tracker:
void read_sun_raw(float raw[4]);
float read_body_rate_dps();
float measure_wheel_rpm();
void set_wheel_rpm(float w_rpm);
String wait_for_line();
bool abort_requested();
void sun_calibrate();
void sun_stream();
void sun_sign_check();
void sun_track(bool hold_trigger, float fan_u_pct, float fan_l_pct);
void fan_hold_test();
void sun_set_params();
void sun_print_params();
void sample_sun_stats(int n, float mean[4], float sd[4]);
void ad_check();
void pt_sweep();
void set_fans(float upper_pct, float lower_pct);
void start_fans(float upper_pct, float lower_pct);
void fan_control();
void fan_sweep();

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Disturbance fans off FIRST, before waiting for the Serial monitor.
  ledcSetup(FAN_UPPER_LEDC_CH, fan_freq_hz, FAN_PWM_BITS);
  ledcSetup(FAN_LOWER_LEDC_CH, fan_freq_hz, FAN_PWM_BITS);
  ledcAttachPin(FAN_UPPER_PIN, FAN_UPPER_LEDC_CH);
  ledcAttachPin(FAN_LOWER_PIN, FAN_LOWER_LEDC_CH);
  // Strongest pin drive (~40 mA): the XY-MOS trigger LED loads the pin, and every bit of gate voltage
  // helps a MOSFET that 3.3 V only half-opens.
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
    Serial.println("[WARN] MAX17048 not detected on I2C (0x36). Battery telemetry (cmd 4) will be unavailable.");
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
    // // 1. Set to Continuous Mode for consistent sampling
    // imu_sensor.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    // // 2. Configure Sample Rate Divider (formula: 1125 / (1 + divider) Hz)
    // ICM_20948_smplrt_t mySmplrt;
    // mySmplrt.a = 1; // Accel divider 1: 1125 / (1+1) = ~562.5 Hz
    // mySmplrt.g = 1; // Gyro divider 1: 1125 / (1+1) = ~562.5 Hz
    // imu_sensor.setSampleRate(ICM_20948_Internal_Acc, mySmplrt);
    // imu_sensor.setSampleRate(ICM_20948_Internal_Gyr, mySmplrt);    
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
 * @brief Gets command received from the ground station (USB or XBee).
 *
 * Reads a command string from USB or XBee and parses the command into 
 * an integer.
 *
 * @note Command strings are trimmed of leading/trailing whitespace before
 *       parsing. Commands are expected to be integers at the start of the
 *       string. Unrecognized commands are ignored.
 * 
 * @returns int of command received
 *
 * @warning This function uses blocking serial reads (readStringUntil())
 *          and may delay program execution if no command is received.
 *
 */
int get_command_from_ground_station()
{
  int received_int = 0; // default to no command
  if (Xbee.available())
  {
    String received_string = Xbee.readStringUntil('\n');
    received_string.trim();
    if (received_string.length() == 0) return -1;
    Serial.print("Received from XBee: ");
    Serial.println(received_string);
    received_int = received_string.toInt();
  }else if (Serial.available())
  {
    String received_string = Serial.readStringUntil('\n');
    received_string.trim();
    if (received_string.length() == 0) return -1;
    Serial.print("Received from Serial: ");
    Serial.println(received_string);
    received_int = received_string.toInt();
  }
  return received_int;
}

/*---------------------------------------------------------------------------------------------*/
// Process Main Menu:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Process and execute commands received from the ground station.
 *
 * Reads a command integer from the ground station (USB Serial or XBee),
 * parses it, and executes the corresponding action. Supported commands:
 *
 * - 0: Stop reaction wheel motor
 * - 1: Print options menu
 * - 2: Get satellite RSSI
 * - 3: Toggle LED
 * - 4: Get battery telemetry (voltage, SOC, charge rate)
 * - 5: Manually set reaction wheel speed (throttle %)
 * - 6: Run Lab 6 - Attitude Determination test
 * - 7: Run Lab 7 - Test A (tabletop static test)
 * - 8: Run Lab 7 - Test B
 * - 9: Stream reaction wheel speed
 * - 10-19: Lab 8 sun tracker commands (see sun tracker section below)
 * - 98: SD Card - List files (USB Serial only)
 * - 99: SD Card - Print file menu (USB Serial only)
 *
 * @note Command strings are trimmed of leading/trailing whitespace before
 *       parsing. Commands are expected to be integers. Unrecognized commands
 *       are ignored with a caution message.
 *
 * @return void
 *
 */
void process_main_menu() {
  if (!Xbee.available() && !Serial.available()) return;

  int received_int = get_command_from_ground_station();

  switch (received_int) {
    case 0:
      driver.setOutput(0);
      set_fans(0, 0);
      Xbee.println("Motor and fans stopped.");
      Serial.println("Motor and fans stopped.");
      break;
    
    case 1:
      Xbee.print("0 - Stop reation wheel and fans\n");
      Xbee.print("1 - Print Options Menu\n");
      Xbee.print("2 - Get RSSI\n");
      Xbee.print("3 - Toggle LED\n");
      Xbee.print("4 - Get Battery State (V, SOC, dSOC/dt) \n");
      Xbee.print("5 - Set Motor Throttle Percent (-100...100)\n");
      Xbee.print("6 - Lab 6: Run Test\n");
      Xbee.print("7 - Lab 7: Run Test A\n");
      Xbee.print("8 - Lab 7: Run Test B\n");
      Xbee.print("9 - Stream RW speed\n");
      Xbee.print("10 - Lab 8: Calibrate Sun Sensor\n");
      Xbee.print("11 - Lab 8: Stream Sun Sensor\n");
      Xbee.print("12 - Lab 8: Sign Check (wheel pulse)\n");
      Xbee.print("13 - Lab 8: Run Sun Tracker\n");
      Xbee.print("14 - Lab 8: Print/Set Tracker Parameters\n");
      Xbee.print("15 - Lab 8: Attitude Determination Check\n");
      Xbee.print("16 - Lab 8: Phototransistor Resistor Sweep\n");
      Xbee.print("17 - Lab 8: Disturbance Fans\n");
      Xbee.print("18 - Lab 8: Fan Torque Sweep\n");
      Xbee.print("19 - Lab 8: Fan Hold Test (CW/CCW)\n");
      Xbee.print("98 - SD Card: List Files (USB SERIAL ONLY)\n");
      Xbee.print("99 - SD Card: Print File Menu (USB SERIAL ONLY)\n");
      
      Serial.print("0 - Stop reation wheel and fans\n");
      Serial.print("1 - Print Options Menu\n");
      Serial.print("2 - Get RSSI\n");
      Serial.print("3 - Toggle LED\n");
      Serial.print("4 - Get Battery State (V, SOC, dSOC/dt) \n");
      Serial.print("5 - Set Motor Throttle Percent (-100...100)\n");
      Serial.print("6 - Lab 6: Run Test\n");
      Serial.print("7 - Lab 7: Run Test A\n");
      Serial.print("8 - Lab 7: Run Test B\n");
      Serial.print("9 - Stream RW speed\n");
      Serial.print("10 - Lab 8: Calibrate Sun Sensor\n");
      Serial.print("11 - Lab 8: Stream Sun Sensor\n");
      Serial.print("12 - Lab 8: Sign Check (wheel pulse)\n");
      Serial.print("13 - Lab 8: Run Sun Tracker\n");
      Serial.print("14 - Lab 8: Print/Set Tracker Parameters\n");
      Serial.print("15 - Lab 8: Attitude Determination Check\n");
      Serial.print("16 - Lab 8: Phototransistor Resistor Sweep\n");
      Serial.print("17 - Lab 8: Disturbance Fans\n");
      Serial.print("18 - Lab 8: Fan Torque Sweep\n");
      Serial.print("19 - Lab 8: Fan Hold Test (CW/CCW)\n");
      Serial.print("98 - SD Card: List Files (USB SERIAL ONLY)\n");
      Serial.print("99 - SD Card: Print File Menu (USB SERIAL ONLY)\n");
      break;

    case 2:
      get_sat_rssi();
      break;

    case 3:
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      Serial.print("LED is now ");
      Serial.println(digitalRead(LED_BUILTIN) ? "LED ON" : "LED OFF");
      Xbee.println(digitalRead(LED_BUILTIN) ? " LED ON" : " LED OFF");
      break;

    case 4:
      send_battery_telemetry();
      break;

    case 5:
      manual_set_RW_speed();
      break;

    case 6:
      lab6_run_test();
      break;

    case 7:
      lab7_run_test_A();
      break;

    case 8:
      lab7_run_test_B();
      break;

    case 9:
      stream_RWspeed();
      break;

    case 10:
      sun_calibrate();
      break;

    case 11:
      sun_stream();
      break;

    case 12:
      sun_sign_check();
      break;

    case 13:
      sun_track(false, fan_test_upper_pct, fan_test_lower_pct);
      break;

    case 14:
      sun_set_params();
      break;

    case 15:
      ad_check();
      break;

    case 16:
      pt_sweep();
      break;

    case 17:
      fan_control();
      break;

    case 18:
      fan_sweep();
      break;

    case 19:
      fan_hold_test();
      break;

    case 98:
      sd_listFiles("/", 0);
      break;

    case 99:
      sd_printFileMenu();
      break;

    default:
      Xbee.println("[CAUTION] Invalid input from ground station, ignoring.");
      Serial.println("[CAUTION] invalid input from ground station, ignoring.");
      break;
  }
  return;
}//end function process_main_menu()

/*---------------------------------------------------------------------------------------------*/
// Get XBee RSSI:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Query the connected XBee radio for received signal strength (RSSI).
 *
 * This function places the XBee module into command mode, issues the ATDB
 * command to request the RSSI of the last received packet, parses the hex
 * response into a decimal value, and prints the result over both Serial and
 * Xbee in dBm.
 *
 * @note The function uses blocking delays and loops; it will not return until
 *       the XBee responds correctly with "OK\r".
 *
 * @warning Assumes that Xbee is connected to an XBee radio and already
 *          initialized at the correct baud rate.
 *
 * @post After the function runs, the XBee is returned to data mode using the
 *       ATCN command.
 *
 * @see Digi XBee AT Command Reference for details on ATDB and ATCN commands.
 */
void get_sat_rssi() {
  Xbee.println(" standby for RSSI");
  
  // put the radio in command mode:
  bool not_done = true;
  String ok_response = "OK\r";   //The﻿response we expect.
  String response = String("");  //Create an empty string
  Serial.println("Starting get_sat_rssi()");
  
  // Read the text of the response into the response variable
  while (not_done) {  // As long as we did not get a response from the XBee
    response = String("");
    delay(1100);
    Xbee.print("+++");  // Put the XBee 3 into 'Command Mode'
    // Serial.print("+++");   // Put the XBee 3 into 'Command Mode'

    // delay(1100);  // Wait for the XBee to finish
    while (response.length() < ok_response.length()) {
      if (Xbee.available() > 0) {

        response += (char)Xbee.read();  // Read a single character at a time
      }
    }
    not_done = !response.equals(ok_response);  // Set the not_done flag to the opposite of the result of equality check
  }
  // Serial.println(response);
      

  // If we got the right response, configure the radio and return true.   
  Xbee.print("ATDB\r");  // destination high and destination low addresses set to 0 means all messages will only go
  delay(100);               // Wait for the XBee
  response = String("");
  while (Xbee.available() > 0) {
    response += (char)Xbee.read();  //Read a single character at a time
    
  }
  Serial.println(response);
  Xbee.print("ATCN\r");  // Switch back to data mode

  String response2 = response; 
  uint32_t dec_response = strtoul(response2.c_str(), NULL, 16);

  Xbee.print("RSSI: -");
  Xbee.print(dec_response);
  Xbee.println(" dBm");
  Serial.println("sat rssi sent");
} // end function get_rssi()

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

  const float v = lipo.getVoltage();
  const float soc = lipo.getSOC();
  const float crate = lipo.getChangeRate(); // %/hr (positive=charging, negative=discharging)

  // Ground-station friendly, parseable response
  Xbee.print("BAT,V=");
  Xbee.print(v, 3);
  Xbee.print("V, SOC=");
  Xbee.print(soc, 1);
  Xbee.print("%, CR=");
  Xbee.print(crate, 3);
  Xbee.println("%/hr");

  // Also mirror to USB serial for debugging
  Serial.print("[BAT] V=");
  Serial.print(v, 3);
  Serial.print(" V, SOC=");
  Serial.print(soc, 1);
  Serial.print(" %, CR=");
  Serial.print(crate, 3);
  Serial.println(" %/hr");
} 


/*---------------------------------------------------------------------------------------------*/
// Manually Set Reaction Wheel Speed:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Prompts user to manually set the reaction wheel motor speed
 * 
 * Requests a throttle percentage from the user (-100 to 100) via Serial or XBee,
 * validates the input, and applies it to the motor driver.
 * 
 * @note Input values outside the range [-100, 100] are clamped to the limits.
 * 
 * @return none
 */
void manual_set_RW_speed(){
  Serial.println("Enter RW Motor Throttle Percent (-100 to 100):");
  Xbee.println("Enter RW Motor Throttle Percent (-100 to 100):");
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  
  int rw_speed_int = get_command_from_ground_station();
  if (rw_speed_int>100) rw_speed_int = 100;
  if (rw_speed_int<-100) rw_speed_int = -100;
  float rw_speed = float(rw_speed_int) / 100.0;
  
  Serial.println("Setting motor speed to: ");
  Serial.println(rw_speed);
  Xbee.println("Setting motor speed to: ");
  Xbee.println(rw_speed);

  delay(500);

  driver.setOutput(rw_speed);

  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer
}

/*---------------------------------------------------------------------------------------------*/
// Run Lab 6 Test:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Runs the test for Lab 6 - Attitude Determination
 * 
 * Collects IMU (gyroscope, magnetometer) and sun sensor data at regular intervals
 * and logs all readings to an SD card file. Test continues until user sends 'X'.
 * 
 * @note Data is written to a CSV file with the following columns:
 *       mcu time (ms), gyro Z (dps), mag X (uT), mag Y (uT), sun direction (deg),
 *       sun_plusX, sun_plusY, sun_minusX, sun_minusY
 * 
 * @return none
 */
void lab6_run_test() {

  char file_name[40];
  if(sd_createDataFile(&dataFile, "Lab6_test")){
    // write header row:
    dataFile.println("mcu time (ms),gyro Z (dps),mag X (uT),mag Y (uT),sun direction (deg),sun_plusX,sun_plusY,sun_minusX,sun_minusY");
    dataFile.flush();
    dataFile.getName(file_name, sizeof(file_name));
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 6 test, send any key to begin (send 'X' to stop test)...");
  Xbee.println("[INFO] Ready to start Lab 6 test, send any key to begin (send 'X' to stop test)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer
  
  timeNext_testPoint = millis();
  int test_point_count = 0;
  neopixelWrite(RGB_BUILTIN, 255, 165, 0); // Set to orange (R=255, G=165, B=0)
  while(true){

    if(Serial.available() > 0 || Xbee.available() > 0) { // Check for user input from USB or XBee
      char c = (Serial.available() > 0) ? Serial.read() : Xbee.read();
      if(c == 'X' || c == 'x') { // If user sent 'X', stop the test
        Serial.print("[INFO] Test Complete. File ");
        Serial.print(file_name);
        Xbee.print("[INFO] Test Complete. File: ");
        Xbee.print(file_name);
        if(dataFile.close()) {
          Serial.println(" closed.");
          Xbee.println(" closed.");
        } else {
          Serial.println(" failed to close.");
          Xbee.println(" failed to close.");
        }
        return;
      } else {
        Serial.println("[CAUTION] Invalid Input, continuing test...");
        Xbee.println("[CAUTION] Invalid Input, continuing test...");
      }
    }

    uint32_t timeNow = millis();
    if(timeNow > timeNext_testPoint){ // Collect Test Point loop
      test_point_count++;
      timeNext_testPoint += interval_testPoint; // Update time for next Test Point
      
      // Collect IMU Test Point:
      imu_sensor.getAGMT();
      gyro_Z = imu_sensor.gyrZ(); 
      mag_X = imu_sensor.magX();
      mag_Y = imu_sensor.magY();

      // Collect Sun Sensor Test Point:
      // Average readings for each analog channel
      sun_plusX = 0.0;
      sun_minusX = 0.0;
      sun_plusY = 0.0;
      sun_minusY = 0.0;
      for (int i = 0; i < n_sun_sensor_reads; i++) {
        sun_plusX += analogRead(SUN_SENSOR_PLUS_X_PIN);
        sun_minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
        sun_plusY += analogRead(SUN_SENSOR_PLUS_Y_PIN);
        sun_minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
      }
      sun_plusX /= n_sun_sensor_reads;
      sun_minusX /= n_sun_sensor_reads;
      sun_plusY /= n_sun_sensor_reads;
      sun_minusY /= n_sun_sensor_reads;

      // ////////////* find sun direction *////////////////////////////////////////
      // // // uncomment sun_plusX & sun_plusY lines to calculate sun direction
      // // // (highlight them, CTRL-/)
      S_mag = sun_plusX + sun_minusX + sun_plusY + sun_minusY;
      // sun_X = ;
      // sun_Y = ;
      sun_direction = (atan2(sun_Y, sun_X) * RAD_TO_DEG);
      if (sun_direction < 0) {
        sun_direction += 360; // Adjust to range 0-360
      }
      /////////////////////////////////////////////////////////////////////////////

      // Print data to .csv file:
      dataFile.print(timeNow);
      dataFile.print(",");
      dataFile.print(gyro_Z,4);
      dataFile.print(",");
      dataFile.print(mag_X,4);
      dataFile.print(",");
      dataFile.print(mag_Y,4);
      dataFile.print(",");
      dataFile.print(sun_direction,4);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.println(sun_minusY);

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += timeNow;
      test_point_string += ",gZ:";
      test_point_string += gyro_Z;
      test_point_string += ",mX:";
      test_point_string += mag_X;
      test_point_string += ",mY:";
      test_point_string += mag_Y;
      test_point_string += ",sDir:";
      test_point_string += sun_direction;
      test_point_string += ",s+X:";
      test_point_string += sun_plusX;
      test_point_string += ",s+Y:";
      test_point_string += sun_plusY;
      test_point_string += ",s-X:";
      test_point_string += sun_minusX;
      test_point_string += ",s-Y:";
      test_point_string += sun_minusY;
      test_point_string += "\n";

      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      if(test_point_count % 10 == 0){ 
        dataFile.flush(); // save file every 10 test points
        Xbee.print("tp:");
        Xbee.println(test_point_count);
        // Xbee.print(test_point_string);
      }
    }
  }  
}

/*---------------------------------------------------------------------------------------------*/
// Lab 7: Run Test A
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Runs Test A - Tabletop static test. Initiates a step input at 100% speed 3 seconds into test. 
 *         Used to determine reaction wheel torque. Test duration is 15 seconds.
 * @details
 *   - 0-3s: Motor off (speed_pwm = 0)
 *   - 3-10s: Motor at full speed (speed_pwm = 1.0)
 *   - 10-15s: Motor off (speed_pwm = 0)
 * @return void
 */
void lab7_run_test_A() {
    
  if(sd_createDataFile(&dataFile, "Lab7_testA")){
    // write header row:
    dataFile.println("mcu time(ms),gyro_Z(deg/s),mag_X(uT),mag_Y(uT),sun_direction(deg),sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),w_RW_cmd(RPM),w_RW_meas(RPM)");
    dataFile.flush();
    char file_name[40];
    dataFile.getName(file_name, sizeof(file_name));
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 7 test A, send any key to begin (wait for test to complete or send 'X' to abort)...");
  Xbee.println("[INFO] Ready to start Lab 7 test A, send any key to begin (wait for test to complete or send 'X' to abort)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer

  timeNext_testPoint = millis();
  bool newUserInput = false;
  float speed_pwm = 0.0;
  uint32_t t0 = millis();
  neopixelWrite(RGB_BUILTIN, 0, 255, 255); // Set to cyan (R=0, G=255, B=255)
  while (true) { //test loop
    
    // Check for User Input:
    char c;
    if (Serial.available() > 0) {  // Check for user input from USB
      c = Serial.read();
      newUserInput = true;
    }
    if (Xbee.available() > 0) {  // Check for user input from USB
      c = Xbee.read();
      newUserInput = true;
    }
    if(newUserInput){
      newUserInput = false;
      switch (c) {
        case 'X':
        case 'x':
          dataFile.close();
          Serial.print("[CAUTION] Test Canceled Early. File closed.");
          Xbee.print("[CAUTION] Test Canceled Early. File closed.");
          driver.setOutput(0);
          return;
        default:
          Serial.println("[CAUTION] Invalid Input, continuing test...");
          Xbee.println("[CAUTION] Invalid Input, continuing test...");
          break;
      }
    }

    // Set RW Motor Speed:
    if ((millis() - t0) > 3000 && (millis() - t0) < 10000){
      speed_pwm = 1.0;
      driver.setOutput(speed_pwm);
    } else if (millis() - t0 > 10000) {
      speed_pwm = 0.0;
      driver.setOutput(speed_pwm);
    }

    // Record test point:
    if (millis() > timeNext_testPoint) {          // Collect Test Point loop
      timeNext_testPoint += interval_testPoint;  // Update time for next Test Point
      uint32_t time = millis() - t0;

      // Get commanded reaction wheel speed:
      float w_RW_cmd = -speed_pwm * 1000.0 * MOTOR_VOLTAGE / 12.0;
      // Get measured reaction wheel speed:
      static int64_t lastCount = 0;
      static uint32_t timeLastEncMeas = 0;
      uint32_t timeNow = millis();
      int64_t c = enc.getCount();
      int64_t dc = c - lastCount;
      float dt_s = (timeNow - timeLastEncMeas) / 1000.0f;
      float rev = (float)dc / ((float)CPR * 10.0);
      float w_RW_meas = (rev / dt_s) * 60.0f;
      lastCount = c;
      timeLastEncMeas = timeNow;

      // Print data to .csv file:
      dataFile.print(time);
      dataFile.print(",");
      dataFile.print(gyro_Z);
      dataFile.print(",");
      dataFile.print(mag_X);
      dataFile.print(",");
      dataFile.print(mag_Y);
      dataFile.print(",");
      dataFile.print(sun_direction);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.print(sun_minusY);
      dataFile.print(",");
      dataFile.print(w_RW_cmd);
      dataFile.print(",");
      dataFile.println(w_RW_meas);

      dataFile.flush();  // save file

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += time;
      test_point_string += ",w_RW_cmd:";
      test_point_string += w_RW_cmd;
      test_point_string += ",w_RW_meas:";
      test_point_string += w_RW_meas;

      test_point_string += "\n";
      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      // Xbee.print(test_point_string);

      // Serial.println(millis() - time + t0);
    }

    // End test if complete:
    if (millis() - t0 > 15000){
      dataFile.close();
      Serial.print("[INFO] Test A Complete. File closed.");
      Xbee.print("[INFO] Test A Complete. File closed.");
      return;
    }
  }
}  // end function lab7_run_test_A()

/*---------------------------------------------------------------------------------------------*/
// Lab 7: Run Test B
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Runs Lab 7 Test B - Reaction wheel control and sensor data collection test.
 * 
 * This function performs a comprehensive test of the spacecraft's attitude determination
 * and control system by collecting sensor data while commanding a reaction wheel motor.
 * The test runs for 40 seconds and logs all measurements to an SD card file.
 * The reaction wheel is commanded with a speed profile defined in set_speed_test_B().
 * 
 * @return void
 * 
 * @note
 * - Test duration is fixed at 40 seconds
 * - Test points are collected at intervals defined by interval_testPoint
 * - XBee data transmission is decimated (every 10th test point)
 * - Motor command speed is calculated as: -speed_pwm * 1000 * MOTOR_VOLTAGE / 12
 * - Sun direction is calculated from 4-quadrant sun sensor using atan2 function
 * - Data is flushed to SD card every 10 test points for data safety
 * 
 * @see set_speed_test_B()
 * @see sd_createDataFile()
 * @see IMU sensor getAGMT(), gyrZ(), magX(), magY() methods
 * @see Encoder getCount() method
 */
void lab7_run_test_B()
{
  if(sd_createDataFile(&dataFile, "Lab7_testB")){
    // write header row:
    dataFile.println("mcu time(ms),gyro_Z(deg/s),mag_X(uT),mag_Y(uT),sun_direction(deg),sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),w_RW_cmd(RPM),w_RW_meas(RPM)");
    dataFile.flush();
    char file_name[40];
    dataFile.getName(file_name, sizeof(file_name));
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 7 test B, send any key to begin (wait for test to complete or send 'X' to abort)...");
  Xbee.println("[INFO] Ready to start Lab 7 test B, send any key to begin (wait for test to complete or send 'X' to abort)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer

  neopixelWrite(RGB_BUILTIN, 255, 0, 255); // Set to magenta (R=255, G=0, B=255)
  static uint32_t t0;
  t0 = millis();
  timeNext_testPoint = millis();
  while (true) { // test loop
    
    // Check for User Input:
    char c;
    bool newUserInput = false;
    if (Serial.available() > 0) {  // Check for user input from USB
      c = Serial.read();
      newUserInput = true;
    }
    if (Xbee.available() > 0) {  // Check for user input from USB
      c = Xbee.read();
      newUserInput = true;
    }
    if(newUserInput){
      newUserInput = false;
      switch (c) {
        case 'X':
        case 'x':
          dataFile.close();
          Serial.print("[CAUTION] Test Canceled Early. File closed.");
          Xbee.print("[CAUTION] Test Canceled Early. File closed.");
          driver.setOutput(0);
          return;
        default:
          Serial.println("[CAUTION] Invalid Input, continuing test...");
          Xbee.println("[CAUTION] Invalid Input, continuing test...");
          break;
      }
    }

    

    // Record test point:
    int test_point_count = 0;
    if (millis() > timeNext_testPoint) {         // Collect Test Point loop
      timeNext_testPoint += interval_testPoint;  // Update time for next Test Point
      test_point_count++;
      uint32_t time = millis() - t0;

      // Set RW Motor Speed:
      float speed_pwm = set_speed_test_B(t0); 

      // Collect IMU Test Point:
      imu_sensor.getAGMT();
      gyro_Z = imu_sensor.gyrZ(); 
      mag_X = imu_sensor.magX();
      mag_Y = imu_sensor.magY();

      // Collect Sun Sensor Test Point:
      // Average readings for each analog channel
      sun_plusX = 0.0;
      sun_minusX = 0.0;
      sun_plusY = 0.0;
      sun_minusY = 0.0;
      for (int i = 0; i < n_sun_sensor_reads; i++) {
        sun_plusX += analogRead(SUN_SENSOR_PLUS_X_PIN);
        sun_minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
        sun_plusY += analogRead(SUN_SENSOR_PLUS_Y_PIN);
        sun_minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
      }
      sun_plusX /= n_sun_sensor_reads;
      sun_minusX /= n_sun_sensor_reads;
      sun_plusY /= n_sun_sensor_reads;
      sun_minusY /= n_sun_sensor_reads;

      // ////////////* find sun direction *////////////////////////////////////////
      // // // uncomment sun_plusX & sun_plusY lines to calculate sun direction
      // // // (highlight them, CTRL-/)
      S_mag = sun_plusX + sun_minusX + sun_plusY + sun_minusY;
      sun_X = (sun_plusX - sun_minusX) / S_mag;
      sun_Y = (sun_plusY - sun_minusY) / S_mag;
      sun_direction = (atan2(sun_Y, sun_X) * RAD_TO_DEG);
      if (sun_direction < 0) {
        sun_direction += 360; // Adjust to range 0-360
      }
      /////////////////////////////////////////////////////////////////////////////

      // Get commanded reaction wheel speed:
      float w_RW_cmd = -speed_pwm * 1000.0 * MOTOR_VOLTAGE / 12.0;
      // Get measured reaction wheel speed:
      static int64_t lastCount = 0;
      static uint32_t timeLastEncMeas = 0;
      uint32_t timeNow = millis();
      int64_t c = enc.getCount();
      int64_t dc = c - lastCount;
      float dt_s = (timeNow - timeLastEncMeas) / 1000.0f;
      float rev = (float)dc / ((float)CPR * 10.0);
      float w_RW_meas = (rev / dt_s) * 60.0f;
      lastCount = c;
      timeLastEncMeas = timeNow;

      // Print data to .csv file:
      dataFile.print(time);
      dataFile.print(",");
      dataFile.print(gyro_Z);
      dataFile.print(",");
      dataFile.print(mag_X);
      dataFile.print(",");
      dataFile.print(mag_Y);
      dataFile.print(",");
      dataFile.print(sun_direction);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.print(sun_minusY);
      dataFile.print(",");
      dataFile.print(w_RW_cmd);
      dataFile.print(",");
      dataFile.println(w_RW_meas);

      dataFile.flush();  // save file

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += time;
      test_point_string += ",gyro_Z:";
      test_point_string += gyro_Z;
      test_point_string += ",mag_X:";
      test_point_string += mag_X;
      test_point_string += ",mag_Y:";
      test_point_string += mag_Y;
      test_point_string += ",sun_direction:";
      test_point_string += sun_direction;
      test_point_string += ",sun_plusX:";
      test_point_string += sun_plusX;
      test_point_string += ",sun_plusY:";
      test_point_string += sun_plusY;
      test_point_string += ",sun_minusX:";
      test_point_string += sun_minusX;
      test_point_string += ",sun_minusY:";
      test_point_string += sun_minusY;
      test_point_string += ",w_RW_cmd:";
      test_point_string += w_RW_cmd;
      test_point_string += ",w_RW_meas:";
      test_point_string += w_RW_meas;

      test_point_string += "\n";
      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      if(test_point_count % 10 == 0){ 
        dataFile.flush(); // save file every 10 test points
        Xbee.print("tp:");
        Xbee.println(test_point_count);
        // Xbee.print(test_point_string);
      }
      // Serial.println(millis() - time + t0);
    }

    // End test if complete:
    if (millis() - t0 > 40e3){
      dataFile.close();
      Serial.print("[INFO] Test B Complete. File closed.");
      Xbee.print("[INFO] Test B Complete. File closed.");
      return;
    }
  }
} // end function lab7_run_test_B()

/*---------------------------------------------------------------------------------------------*/
// Set Wheel Speed:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Sets the reaction wheel motor speed according to a prescribed test profile for Lab 7 Test B.
 * 
 * Implements a dynamic speed profile over 50 seconds:
 *  - 0-10s: Hold at base speed (0.6)
 *  - 10-15s: Hold at base speed (0.6)
 *  - 15-17.5s: Ramp up to base + ramp speed
 *  - 17.5-20s: Ramp back down to base speed
 *  - 20-25s: Hold at base speed
 *  - 25-27.5s: Ramp down to 0
 *  - 27.5-30s: Ramp back up to base speed
 *  - 30-35s: Hold at base speed
 *  - 35s+: Turn off wheel
 * 
 * @return (float) commanded PWM motor speed (0.0 to 1.0)
 */
float set_speed_test_B(uint32_t t0) {
  static uint32_t elapsed; 
  static uint32_t t;
  static float base_speed = 0.6;
  static float ramp_speed = 0.4;
  float motor_PWM_cmd = 0.0;

  t = millis() - t0;

  if (t < 10e3) {  // hold still at half speed (10 sec)
    motor_PWM_cmd = base_speed;
    neopixelWrite(RGB_BUILTIN, 255, 255, 0); // Set to yellow (R=255, G=255, B=0)
  }

  else if (t < 15e3) {  // hold still at half speed (5 sec)
    motor_PWM_cmd = base_speed;
    neopixelWrite(RGB_BUILTIN, 0, 0, 255); // Set to blue (R=0, G=0, B=255)
  }

  else if (t < 17.5e3) {  // ramp up (2.5 sec)
    elapsed = t - 15e3;
    motor_PWM_cmd = base_speed + ramp_speed * elapsed / 2.5e3;
  }

  else if (t < 20e3) {  // ramp back down to half speed (2.5 sec)
    elapsed = t - 17.5e3;
    motor_PWM_cmd = base_speed + ramp_speed - (ramp_speed * elapsed / 2.5e3);
  }

  else if (t < 25e3) {  // hold new position (5 sec)
    motor_PWM_cmd = base_speed;
  }

  else if (t < 27.5e3) {  // ramp down (2.5 sec)
    elapsed = t - 25e3;
    motor_PWM_cmd = base_speed - (ramp_speed * elapsed / 2.5e3);

  } else if (t < 30e3) {  // ramp back up to half speed  (5 sec)
    elapsed = t - 27.5e3;
    motor_PWM_cmd = base_speed - ramp_speed + (ramp_speed * elapsed / 2.5e3);
  }

  else if (t < 35e3) {  // hold new position (5 sec)
    motor_PWM_cmd = base_speed;
    neopixelWrite(RGB_BUILTIN, 255, 0, 255); // Set to magenta (R=255, G=0, B=255)

  }

  else if (t > 35e3) {  // turn off wheel
    motor_PWM_cmd = 0;
    driver.setOutput(0);
    neopixelWrite(RGB_BUILTIN, 255, 255, 255); // Set to white (R=255, G=255, B=255)

  }

  driver.setOutput(motor_PWM_cmd);
  Serial.println(motor_PWM_cmd);

  return motor_PWM_cmd;
}  // end set_speed()


void stream_RWspeed()
{
  Serial.println("Ready to stream RW Motor speed, send any key to start. Send 'X' to stop.");
  Xbee.println("Ready to stream RW Motor speed, send any key to start. Send 'X' to stop.");
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received

  while(true){
    // Check for User Input:
    char c;
    bool newUserInput = false;
    if (Serial.available() > 0) {  // Check for user input from USB
      c = Serial.read();
      newUserInput = true;
    }
    if (Xbee.available() > 0) {  // Check for user input from USB
      c = Xbee.read();
      newUserInput = true;
    }
    if(newUserInput){
      newUserInput = false;
      switch (c) {
        case 'X':
        case 'x':
          return;
      }
    }

    static uint32_t timeLastEncMeas = millis();
    static int64_t lastCount = 0;
    #define ENC_SAMPLE_MS 50
    
    uint32_t now = millis();
    if (now - timeLastEncMeas >= ENC_SAMPLE_MS) {
      int64_t c = enc.getCount();
      int64_t dc = c - lastCount;

      float dt_s = (now - timeLastEncMeas) / 1000.0f;

      // If you use full-quad (x4), make sure CPR reflects *counts per rev after decoding*
      float rev = (float)dc / ((float)CPR * 10.0);
      float rpm = (rev / dt_s) * 60.0f;

      Serial.printf("count:%lld,dc:%lld,rpm:%.2f\n", (long long)c, (long long)dc, rpm);
      Xbee.printf("count:%lld,dc:%lld,rpm:%.2f\n", (long long)c, (long long)dc, rpm);

      lastCount = c;
      timeLastEncMeas = now;
    }
  }
} //end stream_RWspeed()



/////////////////////////////////////////////////////////////////////////////////////////////////
// LAB 8: SUN TRACKER
/////////////////////////////////////////////////////////////////////////////////////////////////
// Goal: point the solar array (body -X, SOLAR_ARRAY_AXIS_DEG) at the sun using the Lab 6 sun
// sensor for pointing error, the gyro for rate damping, and the Lab 7 reaction wheel for torque.
// Recommended order on hardware: 10 (calibrate) -> 11 (check) -> 12 (sign check) -> 13 (track).
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
 *        Also updates the sun_plusX/... globals used by the older tests.
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
 *        Sign is the raw encoder sign (see command 12 output).
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
// Lab 8 Command 10: Calibrate Sun Sensor
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
    print_both("[CAUTION] Calibration NOT applied (previous calibration kept). Run 10 again.");
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
// Lab 8 Command 11: Stream Sun Sensor
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Stream calibrated sun direction and pointing error (wheel off) until 'X'.
 *        Use this to check calibration: light on +X ~0 deg, +Y ~90, -X ~180, -Y ~270,
 *        and err ~0 when the light is on the solar array (-X) face.
 */
void sun_stream() {
  print_both("[INFO] Streaming sun sensor. Send 'X' to stop.");
  uint32_t timeNext = millis();
  int count = 0;
  float raw[4];
  while (true) {
    if (abort_requested()) { print_both("[INFO] Stream stopped."); return; }
    if (millis() < timeNext) continue;
    timeNext += 100;
    count++;

    read_sun_raw(raw);
    SunReading sun = compute_sun(raw, sun_cal);
    float err = pointing_error_deg(sun.direction_deg, array_axis_deg);
    float rate = read_body_rate_dps();

    String s = "sDir:" + String(sun.direction_deg, 1) + ",err:" + String(err, 1) +
               ",mag:" + String(sun.magnitude, 2) + ",gZ:" + String(rate, 2) +
               ",n+X:" + String(sun.norm[SUN_PX], 2) + ",n+Y:" + String(sun.norm[SUN_PY], 2) +
               ",n-X:" + String(sun.norm[SUN_MX], 2) + ",n-Y:" + String(sun.norm[SUN_MY], 2);
    if (sun.magnitude < SUN_LOST_THRESHOLD) s += ",NO_SUN";
    Serial.println(s);
    if (count % 5 == 0) Xbee.println(s); // 2 Hz over XBee
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 12: Sign Check
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Pulse the wheel and compare how the body turned (from the sun sensor) with the gyro
 *        and with the PWM sign. Sets gyro_sign and wheel_pwm_sign for this session.
 *
 * Setup: KestrelSAT hanging and still, light on roughly between two sensors (e.g. 45 deg off
 * the array) so the sun direction changes smoothly, wheel stopped.
 *
 * Physics: spinning the wheel up in one direction turns the body the other way. A fixed sun
 * moves clockwise in the body frame when the body turns +Z (counter-clockwise from above).
 */
void sun_sign_check() {
  const float pulse_pwm = 0.3;
  const uint32_t pulse_ms = 1000;
  float raw[4];

  print_both("[INFO] SIGN CHECK. Hang KestrelSAT, let it settle, light on. Wheel will pulse +30% for 1 s.");
  print_both("[INFO] Send any key to start ('X' to cancel).");
  if (wait_for_line().equalsIgnoreCase("x")) { print_both("[INFO] Sign check canceled."); return; }

  read_sun_raw(raw);
  SunReading sun0 = compute_sun(raw, sun_cal);
  if (sun0.magnitude < SUN_LOST_THRESHOLD) {
    print_both("[CAUTION] No sun detected. Turn on the light / recalibrate (10) and try again.");
    return;
  }
  int64_t count0 = enc.getCount();

  float gyro_angle = 0.0;   // integral of RAW gyro (deg), before gyro_sign
  uint32_t t0 = millis();
  uint32_t tLast = t0;
  driver.setOutput(pulse_pwm);
  while (millis() - t0 < pulse_ms) {
    if (abort_requested()) { driver.setOutput(0); print_both("[INFO] Sign check canceled."); return; }
    uint32_t now = millis();
    if (now - tLast >= 20) {
      read_body_rate_dps();
      gyro_angle += gyro_Z * (now - tLast) / 1000.0;
      tLast = now;
    }
  }
  read_sun_raw(raw);
  SunReading sun1 = compute_sun(raw, sun_cal);
  int64_t dcount = enc.getCount() - count0;

  // Ramp the wheel down gently so KestrelSAT is not flung around.
  for (int k = 10; k >= 0; k--) { driver.setOutput(pulse_pwm * k / 10.0); delay(200); }

  float body_turn_from_sun = -wrap180(sun1.direction_deg - sun0.direction_deg); // deg, +Z positive
  print_both(String("[RESULT] body turn from sun sensor: ") + String(body_turn_from_sun, 1) +
             " deg, raw gyro integral: " + String(gyro_angle, 1) +
             " deg, encoder counts for +PWM: " + String((long)dcount));

  // If the wheel never spun, any body motion came from the string or a bump, not the wheel.
  if (llabs(dcount) < 20) {
    print_both("[CAUTION] Wheel did not spin (encoder counts ~0). Check motor wiring/power. Signs not changed.");
    return;
  }

  float body_turn;  // best estimate of the body turn, +Z positive
  if (fabs(body_turn_from_sun) >= 3.0) {
    gyro_sign = (gyro_angle * body_turn_from_sun > 0) ? 1 : -1;
    body_turn = body_turn_from_sun;
    print_both(String("[RESULT] gyro_sign = ") + gyro_sign);
    if (fabs(fabs(gyro_angle) - fabs(body_turn_from_sun)) > 0.5 * fabs(body_turn_from_sun)) {
      print_both("[CAUTION] Gyro and sun sensor disagree on HOW MUCH the body turned. Check sun calibration.");
    }
  } else {
    body_turn = gyro_sign * gyro_angle;
    print_both("[CAUTION] Sun direction barely changed; keeping current gyro_sign and using the gyro only.");
  }

  if (fabs(body_turn) < 1.0) {
    print_both("[CAUTION] Body barely moved. Is KestrelSAT hanging freely? Signs not changed.");
    return;
  }
  // +PWM turned body +Z  -> wheel spun -Z for +PWM -> wheel_pwm_sign = -1
  wheel_pwm_sign = (body_turn > 0) ? -1 : 1;
  print_both(String("[RESULT] wheel_pwm_sign = ") + wheel_pwm_sign);
  print_both("[INFO] Signs applied for this session. Update DEFAULT_GYRO_SIGN / DEFAULT_WHEEL_PWM_SIGN in definitions.h to keep them.");
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 13: Run Sun Tracker
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
 *  - Disturbance fans (only if a duty > 0 is passed in):
 *      command 13 (hold_trigger false): on FAN_START_DELAY_MS after the first scan finds the sun
 *      command 19 (hold_trigger true) : on once pointing has stayed within FAN_HOLD_BAND_DEG for
 *                                       FAN_HOLD_TIME_MS; reports when pointing first leaves the band
 *    With fans on, the test ends once the wheel has been saturated for FAN_SAT_STOP_MS.
 *  - no sun found in a scan    : wheel and fans stopped, warning sent, test ends
 *  - track_duration_ms or 'X'  : wheel and fans stopped, test ends ('dur' in command 14 sets the length;
 *                                command 19 runs up to FAN_HOLD_DURATION_MS)
 * The gyro bias is measured while KestrelSAT is held (SPINUP/HOLD) and removed from the heading.
 * @param hold_trigger  false = command 13 fan timing, true = command 19 (fans on after holding the band)
 * @param fan_u_pct     GPIO 27 pair duty once the fans turn on (0-100)
 * @param fan_l_pct     GPIO 4 pair duty once the fans turn on (0-100)
 */
static SunScan sun_scan;  // ~5 KB, kept off the stack
void sun_track(bool hold_trigger, float fan_u_pct, float fan_l_pct) {
  char file_name[40];
  if (sd_createDataFile(&dataFile, "SunTrack")) {
    dataFile.println("mcu time(ms),mode,sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),"
                     "sun_mag,sun_direction(deg),error(deg),body_rate(deg/s),mag_X(uT),mag_Y(uT),"
                     "torque_cmd(N*m),w_RW_cmd(RPM),w_RW_meas(RPM),saturated,err_ctrl(deg),latch,"
                     "heading(deg),sun_heading(deg),aim_offset(deg),mx_bright,fan_gpio27(%),fan_gpio4(%)");
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
  const uint32_t duration_ms = hold_trigger ? FAN_HOLD_DURATION_MS : track_duration_ms;
  bool fans_wanted = fan_u_pct > 0 || fan_l_pct > 0;
  bool fans_on = false;
  uint32_t fan_start_at = 0;       // millis() when the fans turn on (0 = not scheduled yet)
  uint32_t fan_kick_until = 0;     // millis() when the 100% start-up kick ends
  uint32_t fan_on_t = 0;           // test time (ms) the fans turned on
  bool fan_sat_reported = false;
  float err_point = 999.0;         // pointing error (deg) the requirement is judged on: gyro heading vs sun heading
  uint32_t in_band_since = 0;      // command 19: millis() pointing entered the band (0 = outside)
  uint32_t fan_band_lost_t = 0;    // test time (ms) pointing first left the band after the fans turned on
  uint32_t fan_band_ms = 0;        // total time inside the band since the fans turned on
  set_fans(0, 0);
  if (fans_wanted && !hold_trigger) {
    print_both(String("[INFO] Disturbance fans will turn on ") + (FAN_START_DELAY_MS / 1000) +
               " s after the sun is found (GPIO 27 " + String(fan_u_pct, 0) + "%, GPIO 4 " +
               String(fan_l_pct, 0) + "%). Test length " + (duration_ms / 1000) + " s.");
  } else if (fans_wanted) {
    print_both(String("[INFO] Disturbance fans will turn on after ") + (FAN_HOLD_TIME_MS / 1000) + " s within +/-" +
               String(FAN_HOLD_BAND_DEG, 0) + " deg (GPIO 27 " + String(fan_u_pct, 0) + "%, GPIO 4 " +
               String(fan_l_pct, 0) + "%). Runs until the wheel saturates (max " + (duration_ms / 60000) + " min).");
  }
  // Command 19 summary, printed on every way the test ends.
  auto print_hold_summary = [&](uint32_t t_end) {
    if (!hold_trigger) return;
    if (!fans_on) { print_both("[RESULT] Fans never turned on (pointing never held the band long enough)."); return; }
    float on_s = (t_end - fan_on_t) / 1000.0;
    print_both(String("[RESULT] Fans on for ") + String(on_s, 1) + " s. Pointing " +
               (fan_band_lost_t ? String("first left +/-") + String(FAN_HOLD_BAND_DEG, 0) + " deg after " +
                                  String((fan_band_lost_t - fan_on_t) / 1000.0, 1) + " s"
                                : String("never left the band")) +
               "; inside the band " + String(on_s > 0 ? 100.0 * fan_band_ms / (on_s * 1000.0) : 0.0, 0) +
               "% of the time. Wheel " + (fan_sat_reported ? "saturated (see above)." : "did not saturate."));
  };

  uint32_t t0 = millis();
  uint32_t tLastControl = t0;
  timeNext_testPoint = t0;
  measure_wheel_rpm(); // reset encoder timing
  neopixelWrite(RGB_BUILTIN, 255, 255, 0); // yellow: hold still

  while (true) {
    if (abort_requested()) {
      driver.setOutput(0);
      set_fans(0, 0);
      dataFile.close();
      print_both(String("[CAUTION] Test stopped by user. Wheel and fans stopped. File ") + file_name + " closed.");
      print_hold_summary(millis() - t0);
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
        neopixelWrite(RGB_BUILTIN, 128, 0, 128); // purple: scanning
        tau = rate_step(SCAN_RATE_DPS, rate, dt_s, tracker_gains, wheel_limits, I_WHEEL, w_cmd_rpm, saturated);
        bool done = scan_add(sun_scan, heading, mx_bright, SCAN_TURN_DEG);
        if (done || now - scan_t0 > SCAN_TIMEOUT_MS) {
          if (!scan_result(sun_scan, SCAN_PEAK_FRACTION, SUN_LOST_THRESHOLD, sun_heading, sun_peak)) {
            driver.setOutput(0);
            set_fans(0, 0);
            dataFile.close();
            neopixelWrite(RGB_BUILTIN, 255, 0, 0); // red
            print_both(String("[WARN] No sun found in the scan (brightest -X ") + String(sun_peak, 2) +
                       "). Wheel stopped. File " + file_name + " closed.");
            print_hold_summary(t);
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
        }
      } else {
        mode = "TRACK";
        float err_gyro = wrap180(sun_heading - heading);           // how far the array is from the sun heading
        err_point = err_gyro;
        float err_aim = wrap180(sun_heading + aim_offset - heading); // how far from this lobe's aim point
        bool settled = fabs(err_aim) < LOBE_SETTLED_DEG && fabs(rate) < LOBE_SETTLED_DPS;
        aim_offset = lobe_step(lobe, mx_bright, settled, dt_s, LOBE_AMP_DEG, LOBE_DWELL_MS / 1000.0,
                               LOBE_AVG_MS / 1000.0, LOBE_GAIN, LOBE_MAX_STEP_DEG, LOBE_HALF_POWER_DEG, sun_heading);
        err_aim = wrap180(sun_heading + aim_offset - heading);
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

    // Disturbance fans. Command 13: on FAN_START_DELAY_MS after the first scan has found the sun.
    // Command 19: on once pointing has stayed inside the band for FAN_HOLD_TIME_MS.
    bool in_band = fabs(err_point) <= FAN_HOLD_BAND_DEG;
    if (in_band) { if (in_band_since == 0) in_band_since = now; }
    else in_band_since = 0;
    if (fans_wanted && fan_start_at == 0) {
      if (!hold_trigger && control_started && !acquiring) fan_start_at = now + FAN_START_DELAY_MS;
      if (hold_trigger && in_band_since != 0 && now - in_band_since >= FAN_HOLD_TIME_MS) fan_start_at = now;
    }
    if (fans_wanted && !fans_on && fan_start_at != 0 && now >= fan_start_at) {
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
    // Command 19: how long pointing stays inside the band with the fans on.
    if (hold_trigger && fans_on) {
      if (in_band) fan_band_ms += interval_testPoint;
      else if (fan_band_lost_t == 0) {
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
      driver.setOutput(0);
      set_fans(0, 0);
      dataFile.close();
      print_both(String("[INFO] Wheel saturated for ") + (FAN_SAT_STOP_MS / 1000) +
                 " s with the fans on. Test ended. Wheel and fans stopped. File " + file_name + " closed.");
      print_hold_summary(t);
      return;
    }

    // Log to SD:
    dataFile.print(t);                            dataFile.print(",");
    dataFile.print(mode);                         dataFile.print(",");
    dataFile.print(raw[SUN_PX]);                  dataFile.print(",");
    dataFile.print(raw[SUN_PY]);                  dataFile.print(",");
    dataFile.print(raw[SUN_MX]);                  dataFile.print(",");
    dataFile.print(raw[SUN_MY]);                  dataFile.print(",");
    dataFile.print(sun.magnitude, 3);             dataFile.print(",");
    dataFile.print(sun.direction_deg, 2);         dataFile.print(",");
    dataFile.print(err, 2);                       dataFile.print(",");
    dataFile.print(rate, 3);                      dataFile.print(",");
    dataFile.print(mag_X, 2);                     dataFile.print(",");
    dataFile.print(mag_Y, 2);                     dataFile.print(",");
    dataFile.print(tau, 6);                       dataFile.print(",");
    dataFile.print(w_cmd_rpm, 1);                 dataFile.print(",");
    dataFile.print(w_meas, 1);                    dataFile.print(",");
    dataFile.print(saturated ? 1 : 0);           dataFile.print(",");
    dataFile.print(err_ctrl, 2);                  dataFile.print(",");
    dataFile.print(latch.dir);                    dataFile.print(",");
    dataFile.print(heading, 2);                   dataFile.print(",");
    dataFile.print(sun_heading, 2);               dataFile.print(",");
    dataFile.print(aim_offset, 1);                dataFile.print(",");
    dataFile.print(mx_bright, 3);                 dataFile.print(",");
    dataFile.print(fan_upper_pct, 0);             dataFile.print(",");
    dataFile.println(fan_lower_pct, 0);

    // Telemetry (Name:value for GCS plotter): every point on USB, 2 Hz on XBee
    String s = "t:" + String(t) + ",err:" + String(err, 1) + ",latch:" + String(latch.dir) + ",hdg:" + String(heading, 1) +
               ",sunHdg:" + String(sun_heading, 1) + ",aim:" + String(aim_offset, 0) +
               ",bX:" + String(mx_bright, 2) + ",acq:" + String(acquiring ? 1 : 0) + ",gZ:" + String(rate, 2) +
               ",wCmd:" + String(w_cmd_rpm, 0) + ",wMeas:" + String(w_meas, 0) +
               ",sDir:" + String(sun.direction_deg, 1) +
               // same sun sensor fields as command 11, so the GCS Sun Map works during tracking
               ",mag:" + String(sun.magnitude, 2) +
               ",n+X:" + String(sun.norm[SUN_PX], 2) + ",n+Y:" + String(sun.norm[SUN_PY], 2) +
               ",n-X:" + String(sun.norm[SUN_MX], 2) + ",n-Y:" + String(sun.norm[SUN_MY], 2) +
               ",fan27:" + String(fan_upper_pct, 0) + ",fan4:" + String(fan_lower_pct, 0);
    Serial.println(s);
    if (test_point_count % 10 == 0) {
      dataFile.flush(); // save file every 10 test points
      Xbee.println(s);
    }

    // End test if complete:
    if (t > duration_ms) {
      driver.setOutput(0);
      set_fans(0, 0);
      dataFile.close();
      print_both(String("[INFO] Sun tracker test complete. Wheel and fans stopped. File ") + file_name + " closed.");
      print_hold_summary(t);
      return;
    }
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 14: Print / Set Tracker Parameters
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
// Shared helper for commands 15 and 16
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
// Lab 8 Command 15: Attitude Determination Check
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
 * Run command 10 (calibrate) first if the sun sensor wiring changed.
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
  print_both("[INFO] If you changed the sun sensor wiring, run 10 (calibrate) first.");

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
      print_both(String("  Sun magnitude ") + String(sun.magnitude, 2) + " with the light off: room light looks like sun. Recalibrate (10) in these room conditions.");
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
    else if (gyro_angle * sun_turn < 0) print_both("  Gyro and sun sensor turned OPPOSITE ways: gyro_sign is flipped (14 -> gsign) or sun sensor pins swapped.");
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
// Lab 8 Command 16: Phototransistor Resistor Sweep
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Record sun sensor brightness for different phototransistor resistor values.
 *        Logs to PTSweep_dataNNN.csv; plot with analysis/plot_ptsweep.py.
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
    print_both(String("[INFO] Sweep finished. File ") + file_name + " closed. Plot it with: python analysis/plot_ptsweep.py " + file_name);
  } else {
    print_both("[INFO] Sweep finished.");
  }
  print_both("[INFO] After installing the resistor you pick, re-run 10 (calibrate) and paste the new sun_cal line.");
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 17: Disturbance Fans
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

/**
 * @brief Spin the fans at the given duties, starting any pair that is at rest with a short
 *        100% kick so low duties still spin up. Blocks for FAN_KICK_MS (only used in command 17).
 */
void start_fans(float upper_pct, float lower_pct) {
  bool kick_upper = fan_upper_pct == 0 && upper_pct > 0;
  bool kick_lower = fan_lower_pct == 0 && lower_pct > 0;
  if (kick_upper || kick_lower) {
    set_fans(kick_upper ? 100 : fan_upper_pct, kick_lower ? 100 : fan_lower_pct);
    delay(FAN_KICK_MS);
  }
  set_fans(upper_pct, lower_pct);
}

/**
 * @brief Set the fan duties that command 13 will use, and spin the fans now so you can check them.
 *        Send lines like "u 60" (GPIO 27 pair), "l 60" (GPIO 4 pair), "both 60",
 *        "off" (both 0), "freq 25" (PWM frequency, Hz). Send "done" (or 'X') to finish.
 *        The fans stop when you finish; the duties are kept until reboot.
 */
void fan_control() {
  print_both("[INFO] Disturbance fans: 'u' = GPIO 27 pair (clockwise), 'l' = GPIO 4 pair (counter-clockwise), duty 0-100%.");
  print_both("[INFO] Send 'u <pct>', 'l <pct>', 'both <pct>', 'off', 'freq <Hz>', or 'done'.");
  print_both("[INFO] Fans spin now so you can check them. Command 13 uses the same duties (0 = no fans).");
  print_both(String("fan gpio27=") + String(fan_test_upper_pct, 0) + "% gpio4=" + String(fan_test_lower_pct, 0) +
             "% freq=" + fan_freq_hz + " Hz");
  start_fans(fan_test_upper_pct, fan_test_lower_pct);
  while (true) {
    String line = wait_for_line();
    line.toLowerCase();
    if (line == "done" || line == "x" || line.length() == 0) break;

    if (line == "off") {
      fan_test_upper_pct = 0;
      fan_test_lower_pct = 0;
    } else {
      int space = line.indexOf(' ');
      if (space < 0) { print_both("[CAUTION] Format is '<name> <value>', e.g. 'u 60'."); continue; }
      String name = line.substring(0, space);
      float value = line.substring(space + 1).toFloat();

      if      (name == "u")    fan_test_upper_pct = constrain(value, 0.0, 100.0);
      else if (name == "l")    fan_test_lower_pct = constrain(value, 0.0, 100.0);
      else if (name == "both") { fan_test_upper_pct = constrain(value, 0.0, 100.0); fan_test_lower_pct = fan_test_upper_pct; }
      else if (name == "freq") {
        uint32_t f = (uint32_t)constrain(value, 10, 20000); // XY-MOS module is rated 0-20 kHz
        if (ledcChangeFrequency(FAN_UPPER_LEDC_CH, f, FAN_PWM_BITS) == 0 ||
            ledcChangeFrequency(FAN_LOWER_LEDC_CH, f, FAN_PWM_BITS) == 0) {
          print_both("[CAUTION] That frequency can't be set.");
          continue;
        }
        fan_freq_hz = f;
      }
      else { print_both("[CAUTION] Unknown setting: " + name); continue; }
    }
    start_fans(fan_test_upper_pct, fan_test_lower_pct);
    print_both(String("fan gpio27=") + String(fan_test_upper_pct, 0) + "% gpio4=" + String(fan_test_lower_pct, 0) +
               "% freq=" + fan_freq_hz + " Hz");
  }
  set_fans(0, 0);
  print_both("[INFO] Fans off. Command 13 will use these duties.");
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 18: Fan Torque Sweep
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Least-squares slope of y vs t from running sums (n points). Returns 0 if there are too few points.
 */
float fit_slope(int n, double st, double sy, double stt, double sty) {
  double den = n * stt - st * st;
  if (n < 3 || den == 0) return 0.0;
  return (float)((n * sty - st * sy) / den);
}

/**
 * @brief Measure the Z torque each fan pair makes at each duty, with KestrelSAT hanging free.
 *
 * Order: gyro bias (hang still, wheel off) -> for each duty 0, 10, ... 100%: GPIO 27 pair, then GPIO 4 pair.
 * Each step:
 *  - SETTLE : fans off, the wheel damps the body rate to ~0 (rate_step with 0 deg/s)
 *  - ON     : fans on at the duty for FAN_SWEEP_ON_MS, wheel held at a constant speed so it adds
 *             (almost) no torque. Over the last part of the step (from FAN_SWEEP_FIT_START_MS):
 *             body accel = least-squares slope of the gyro rate; wheel accel = change in average wheel
 *             speed between the two halves, from raw encoder counts (the 50 ms RPM readings are too noisy).
 *             torque = I_BODY_Z * body accel + I_WHEEL * wheel accel.
 * Positive torque = counter-clockwise (+Z, seen from above). Prints one [RESULT] line per step and a table
 * at the end; logs every point to FanSweep_dataNNN.csv. Send 'X' to stop.
 * @note The string twist pushes back as KestrelSAT turns, so big torques read a little low.
 */
void fan_sweep() {
  const int n_levels = 100 / FAN_SWEEP_STEP_PCT + 1;
  const int n_steps = 2 * n_levels;
  float tau_result[2][n_levels];  // [0] = GPIO 27 pair, [1] = GPIO 4 pair (N*m)
  char file_name[40];

  if (sd_createDataFile(&dataFile, "FanSweep")) {
    dataFile.println("mcu time(ms),step,pin,duty(%),phase,heading(deg),body_rate(deg/s),body_accel(deg/s^2),"
                     "w_RW_cmd(RPM),w_RW_meas(RPM),enc_count");
    dataFile.flush();
    dataFile.getName(file_name, sizeof(file_name));
    print_both(String("[INFO] Data file created successfully: ") + file_name);
  } else {
    print_both("[ERROR] Failed to create data file. Aborting sweep.");
    return;
  }

  print_both(String("[INFO] Fan torque sweep: ") + n_steps + " steps (0-100% in " + FAN_SWEEP_STEP_PCT +
             "% steps, GPIO 27 pair then GPIO 4 pair). About " + (n_steps * 8 / 60 + 1) + " min.");
  print_both("[INFO] Let KestrelSAT hang still, then send any key. Don't touch it during the sweep. Send 'X' to stop.");
  if (wait_for_line().equalsIgnoreCase("x")) { dataFile.close(); print_both("[INFO] Sweep canceled."); return; }

  enum { BIAS, SETTLE, ON } phase = BIAS;
  float gyro_bias = 0.0;
  float bias_sum = 0.0;
  int bias_n = 0;
  float heading = 0.0;
  float prev_rate = 0.0;
  float accel_f = 0.0;            // smoothed body accel (deg/s^2), for the log/telemetry only
  float w_cmd_rpm = 0.0;
  float w_prev_cmd_rpm = 0.0;
  bool saturated = false;
  int step = 0;
  int pin = FAN_UPPER_PIN;
  float duty = 0.0;
  uint32_t phase_t0 = 0;
  uint32_t still_since = 0;
  uint32_t kick_until = 0;
  float heading_on0 = 0.0;
  // least-squares sums over the fit window (t in s from the start of the step)
  int fit_n = 0;
  double st = 0, stt = 0, sr = 0, str_ = 0;
  // encoder count at the start, middle and end of the fit window (for the wheel accel)
  const uint32_t fit_mid_ms = FAN_SWEEP_FIT_START_MS + (FAN_SWEEP_ON_MS - FAN_SWEEP_FIT_START_MS) / 2;
  int64_t enc_c[3] = {0, 0, 0};
  uint32_t enc_t[3] = {0, 0, 0};
  int enc_marks = 0;
  float rate = 0.0;
  int test_point_count = 0;

  set_fans(0, 0);
  driver.setOutput(0);
  uint32_t t0 = millis();
  uint32_t tLast = t0;
  timeNext_testPoint = t0;
  measure_wheel_rpm(); // reset encoder timing
  neopixelWrite(RGB_BUILTIN, 255, 255, 0); // yellow: measuring gyro bias, keep still

  while (true) {
    if (abort_requested()) {
      driver.setOutput(0);
      set_fans(0, 0);
      dataFile.close();
      print_both(String("[CAUTION] Sweep stopped by user. Wheel and fans stopped. File ") + file_name + " closed.");
      return;
    }

    uint32_t now = millis();
    if (now < timeNext_testPoint) continue;
    timeNext_testPoint += interval_testPoint;
    test_point_count++;
    uint32_t t = now - t0;
    float dt_s = (now - tLast) / 1000.0f;
    tLast = now;

    // Sense:
    rate = read_body_rate_dps() - gyro_bias;
    float w_meas = measure_wheel_rpm();
    if (phase != BIAS) heading += rate * dt_s;
    if (dt_s > 0) accel_f += ((rate - prev_rate) / dt_s - accel_f) * dt_s / (FAN_SWEEP_ACCEL_TAU_S + dt_s);
    prev_rate = rate;

    const char *phase_name = "BIAS";
    if (phase == BIAS) {
      bias_sum += rate;
      bias_n++;
      if (t >= FAN_SWEEP_BIAS_MS) {
        gyro_bias = (bias_n > 0) ? bias_sum / bias_n : 0.0;
        print_both(String("[INFO] Gyro bias ") + String(gyro_bias, 3) + " deg/s.");
        phase = SETTLE;
        phase_t0 = now;
        still_since = 0;
        accel_f = 0.0;
        prev_rate = 0.0;
      }
    } else if (phase == SETTLE) {
      phase_name = "SETTLE";
      neopixelWrite(RGB_BUILTIN, 0, 0, 255); // blue: wheel stopping the body
      rate_step(0.0, rate, dt_s, tracker_gains, wheel_limits, I_WHEEL, w_cmd_rpm, saturated);
      set_wheel_rpm(motor_lag_comp(w_cmd_rpm, w_prev_cmd_rpm, dt_s, MOTOR_TAU_S, RPM_PER_PWM));
      w_prev_cmd_rpm = w_cmd_rpm;

      if (fabs(rate) < FAN_SWEEP_STILL_DPS) { if (still_since == 0) still_since = now; }
      else still_since = 0;
      bool still = still_since != 0 && now - still_since >= FAN_SWEEP_STILL_MS;
      bool timed_out = now - phase_t0 >= FAN_SWEEP_SETTLE_MAX_MS;
      if (still || timed_out) {
        if (timed_out && !still) print_both(String("[CAUTION] Body still turning at ") + String(rate, 1) + " deg/s; starting the step anyway.");
        if (saturated) print_both("[CAUTION] Wheel is at its speed limit; this step's numbers may be off.");
        // Start the step: hold the wheel at its current speed, fans on (kick from rest so low duties start).
        phase = ON;
        phase_t0 = now;
        pin = (step % 2 == 0) ? FAN_UPPER_PIN : FAN_LOWER_PIN;
        duty = (step / 2) * FAN_SWEEP_STEP_PCT;
        set_wheel_rpm(w_cmd_rpm);
        w_prev_cmd_rpm = w_cmd_rpm;
        float start = (duty > 0) ? 100.0 : 0.0;
        if (pin == FAN_UPPER_PIN) set_fans(start, 0); else set_fans(0, start);
        kick_until = (duty > 0) ? now + FAN_KICK_MS : 0;
        heading_on0 = heading;
        fit_n = 0; st = stt = sr = str_ = 0;
        enc_marks = 0;
        print_both(String("[INFO] Step ") + (step + 1) + "/" + n_steps + ": GPIO " + pin + " pair at " +
                   String(duty, 0) + "%.");
      }
    } else { // ON
      phase_name = "ON";
      neopixelWrite(RGB_BUILTIN, 0, 255, 0); // green: fans on, measuring
      if (kick_until != 0 && now >= kick_until) {
        kick_until = 0;
        if (pin == FAN_UPPER_PIN) set_fans(duty, 0); else set_fans(0, duty);
      }
      uint32_t t_step = now - phase_t0;
      if (t_step >= FAN_SWEEP_FIT_START_MS) {
        double ts = t_step / 1000.0;
        fit_n++; st += ts; stt += ts * ts;
        sr += rate;   str_ += ts * rate;
      }
      // mark the encoder count at the start, middle and end of the fit window
      if ((enc_marks == 0 && t_step >= FAN_SWEEP_FIT_START_MS) || (enc_marks == 1 && t_step >= fit_mid_ms) ||
          (enc_marks == 2 && t_step >= FAN_SWEEP_ON_MS)) {
        enc_c[enc_marks] = enc.getCount();
        enc_t[enc_marks] = now;
        enc_marks++;
      }
      if (t_step >= FAN_SWEEP_ON_MS) {
        set_fans(0, 0);
        float body_accel = fit_slope(fit_n, st, sr, stt, str_);   // deg/s^2
        // wheel accel (RPM/s): average speed in the 2nd half minus the 1st half, over the time between their centres
        float wheel_accel = 0.0;
        if (enc_marks == 3 && enc_t[1] > enc_t[0] && enc_t[2] > enc_t[1]) {
          float w1 = (float)(enc_c[1] - enc_c[0]) / ((float)CPR * 10.0) / ((enc_t[1] - enc_t[0]) / 60000.0f);
          float w2 = (float)(enc_c[2] - enc_c[1]) / ((float)CPR * 10.0) / ((enc_t[2] - enc_t[1]) / 60000.0f);
          wheel_accel = (w2 - w1) / ((enc_t[2] - enc_t[0]) / 2000.0f);
        }
        float tau = I_BODY_Z * body_accel / 57.2957795 + I_WHEEL * wheel_accel * 2.0 * PI / 60.0;
        tau_result[pin == FAN_UPPER_PIN ? 0 : 1][step / 2] = tau;
        print_both(String("[RESULT] GPIO ") + pin + " pair " + String(duty, 0) + "%: torque " +
                   String(tau * 1000.0, 3) + " mN*m (" + (tau >= 0 ? "CCW" : "CW") + ") | body accel " +
                   String(body_accel, 2) + " deg/s^2, rate " + String(rate, 1) + " deg/s, turned " +
                   String(heading - heading_on0, 1) + " deg | wheel " + String(wheel_accel, 2) + " RPM/s");
        step++;
        if (step >= n_steps) break;
        phase = SETTLE;
        phase_t0 = now;
        still_since = 0;
      }
    }

    // Log to SD:
    dataFile.print(t);                       dataFile.print(",");
    dataFile.print(step + 1);                dataFile.print(",");
    dataFile.print(phase == ON ? pin : 0);   dataFile.print(",");
    dataFile.print(phase == ON ? duty : 0, 0); dataFile.print(",");
    dataFile.print(phase_name);              dataFile.print(",");
    dataFile.print(heading, 2);              dataFile.print(",");
    dataFile.print(rate, 3);                 dataFile.print(",");
    dataFile.print(accel_f, 3);              dataFile.print(",");
    dataFile.print(w_cmd_rpm, 1);            dataFile.print(",");
    dataFile.print(w_meas, 1);               dataFile.print(",");
    dataFile.println((long)enc.getCount());

    // Telemetry (Name:value for GCS plotter): every point on USB, 2 Hz on XBee
    String s = "t:" + String(t) + ",step:" + String(step + 1) + ",pin:" + String(phase == ON ? pin : 0) +
               ",duty:" + String(phase == ON ? duty : 0, 0) + ",hdg:" + String(heading, 1) +
               ",gZ:" + String(rate, 2) + ",aZ:" + String(accel_f, 2) +
               ",wCmd:" + String(w_cmd_rpm, 0) + ",wMeas:" + String(w_meas, 0);
    Serial.println(s);
    if (test_point_count % 10 == 0) {
      dataFile.flush(); // save file every 10 test points
      Xbee.println(s);
    }
  }

  driver.setOutput(0);
  set_fans(0, 0);
  dataFile.close();
  neopixelWrite(RGB_BUILTIN, 0, 255, 0);
  print_both("[RESULT] Fan torque (mN*m, + = CCW / +Z from above):");
  print_both("duty(%),GPIO27_pair,GPIO4_pair");
  for (int i = 0; i < n_levels; i++) {
    print_both(String(i * FAN_SWEEP_STEP_PCT) + "," + String(tau_result[0][i] * 1000.0, 3) + "," +
               String(tau_result[1][i] * 1000.0, 3));
  }
  print_both(String("[INFO] Sweep complete. Wheel and fans stopped. File ") + file_name +
             " closed. Plot it with analysis/plot_fansweep.py.");
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Command 19: Fan Hold Test
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Sun tracker (command 13) with a disturbance: once pointing has stayed within FAN_HOLD_BAND_DEG
 *        for FAN_HOLD_TIME_MS, one fan pair turns on and the reaction wheel has to fight it.
 *        Asks for the direction ('cw' = GPIO 27 pair, 'ccw' = GPIO 4 pair, seen from above) and an
 *        optional duty (default 100%), e.g. "ccw" or "cw 60". Prints when pointing first leaves the band
 *        and when the wheel saturates; logs to SunTrack_dataNNN.csv like command 13.
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
  if (dir == "cw") sun_track(true, duty, 0.0);
  else             sun_track(true, 0.0, duty);
}
