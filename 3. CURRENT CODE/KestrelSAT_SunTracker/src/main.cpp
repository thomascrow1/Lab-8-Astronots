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
SunCalibration sun_cal = {{0.0, 0.0, 0.0, 0.0}, {4095.0, 4095.0, 4095.0, 4095.0}}; // replace with command 10 results
TrackerGains tracker_gains = {DEFAULT_KP, DEFAULT_KD, POINTING_DEADBAND_DEG};
WheelLimits wheel_limits = {WHEEL_MIN_RPM, WHEEL_MAX_RPM, WHEEL_MAX_ACCEL_RPM_S};
float array_axis_deg = SOLAR_ARRAY_AXIS_DEG;
float bias_pwm = DEFAULT_BIAS_PWM;
int wheel_pwm_sign = DEFAULT_WHEEL_PWM_SIGN;
int gyro_sign = DEFAULT_GYRO_SIGN;

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
void sun_track();
void sun_set_params();
void sun_print_params();

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
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
 * - 10-14: Lab 8 sun tracker commands (see sun tracker section below)
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
      Xbee.println("Motor Stopped.");
      Serial.println("Motor Stopped.");
      break;
    
    case 1:
      Xbee.print("0 - Stop reation wheel\n");
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
      Xbee.print("98 - SD Card: List Files (USB SERIAL ONLY)\n");
      Xbee.print("99 - SD Card: Print File Menu (USB SERIAL ONLY)\n");
      
      Serial.print("0 - Stop reation wheel\n");
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
      sun_track();
      break;

    case 14:
      sun_set_params();
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
 *        Also updates the sun_plusX/... globals used by the older tests.
 */
void read_sun_raw(float raw[4]) {
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
    if (new_cal.bright[i] < new_cal.dark[i] + 50) {
      print_both(String("[CAUTION] ") + names[i] + " barely changed with light on it. Check wiring/aim and recalibrate.");
    }
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
 *  - to TRACK_DURATION_MS      : TRACK  - PD control, LED green within 10 deg, orange outside
 *  - sun lost > SUN_LOST_TIME_MS: wheel stopped, warning sent, test ends
 *  - 'X' at any time           : wheel stopped, test ends
 */
void sun_track() {
  char file_name[40];
  if (sd_createDataFile(&dataFile, "SunTrack")) {
    dataFile.println("mcu time(ms),mode,sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),"
                     "sun_mag,sun_direction(deg),error(deg),body_rate(deg/s),mag_X(uT),mag_Y(uT),"
                     "torque_cmd(N*m),w_RW_cmd(RPM),w_RW_meas(RPM),saturated");
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
    print_both("[CAUTION] No sun detected. Turn on the light / recalibrate (10) and try again.");
    dataFile.close();
    return;
  }

  print_both("[INFO] Ready to run sun tracker. HOLD KestrelSAT still and send any key to begin.");
  print_both("[INFO] Release KestrelSAT when the RGB LED turns BLUE. Send 'X' to stop.");
  if (wait_for_line().equalsIgnoreCase("x")) { dataFile.close(); print_both("[INFO] Test canceled."); return; }

  const float bias_rpm = bias_pwm * RPM_PER_PWM;
  float w_cmd_rpm = 0.0;
  float tau = 0.0;
  bool saturated = false;
  uint32_t saturated_since = 0;
  uint32_t last_saturation_warning = 0;
  uint32_t sun_lost_since = 0;
  bool release_announced = false;
  int test_point_count = 0;

  uint32_t t0 = millis();
  uint32_t tLastControl = t0;
  timeNext_testPoint = t0;
  measure_wheel_rpm(); // reset encoder timing
  neopixelWrite(RGB_BUILTIN, 255, 255, 0); // yellow: hold still

  while (true) {
    if (abort_requested()) {
      driver.setOutput(0);
      dataFile.close();
      print_both(String("[CAUTION] Test stopped by user. Wheel stopped. File ") + file_name + " closed.");
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
    bool sun_visible = sun.magnitude >= SUN_LOST_THRESHOLD;

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
      if (t >= TRACK_RELEASE_MS && !release_announced) {
        release_announced = true;
        neopixelWrite(RGB_BUILTIN, 0, 0, 255); // blue: release
        print_both("[INFO] RELEASE KestrelSAT now.");
      }
    } else if (!sun_visible) {
      mode = "NOSUN"; // keep last wheel command while waiting to see if the sun comes back
      if (sun_lost_since == 0) sun_lost_since = now;
      if (now - sun_lost_since > SUN_LOST_TIME_MS) {
        driver.setOutput(0);
        dataFile.close();
        neopixelWrite(RGB_BUILTIN, 255, 0, 0); // red
        print_both(String("[WARN] SUN LOST at t=") + t + " ms. Wheel stopped. File " + file_name + " closed.");
        return;
      }
    } else {
      mode = "TRACK";
      sun_lost_since = 0;
      tau = tracker_step(err, rate, dt_s, tracker_gains, wheel_limits, I_WHEEL, w_cmd_rpm, saturated);
      if (fabs(err) <= 10.0) neopixelWrite(RGB_BUILTIN, 0, 255, 0);   // green: on target
      else                   neopixelWrite(RGB_BUILTIN, 255, 165, 0); // orange: slewing
    }
    set_wheel_rpm(w_cmd_rpm);

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
    dataFile.println(saturated ? 1 : 0);

    // Telemetry (Name:value for GCS plotter): every point on USB, 2 Hz on XBee
    String s = "t:" + String(t) + ",err:" + String(err, 1) + ",gZ:" + String(rate, 2) +
               ",wCmd:" + String(w_cmd_rpm, 0) + ",wMeas:" + String(w_meas, 0) +
               ",sDir:" + String(sun.direction_deg, 1);
    Serial.println(s);
    if (test_point_count % 10 == 0) {
      dataFile.flush(); // save file every 10 test points
      Xbee.println(s);
    }

    // End test if complete:
    if (t > TRACK_DURATION_MS) {
      driver.setOutput(0);
      dataFile.close();
      print_both(String("[INFO] Sun tracker test complete. Wheel stopped. File ") + file_name + " closed.");
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
             " db=" + String(tracker_gains.deadband_deg, 1) + " bias=" + String(bias_pwm, 2) +
             " axis=" + String(array_axis_deg, 1) + " wsign=" + wheel_pwm_sign + " gsign=" + gyro_sign);
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
  print_both("[INFO] Send '<name> <value>' (names: kp kd db bias axis wsign gsign). Send 'done' to finish.");
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
    else if (name == "bias")  bias_pwm = constrain(value, 0.1, 0.9);
    else if (name == "axis")  array_axis_deg = value;
    else if (name == "wsign") wheel_pwm_sign = (value < 0) ? -1 : 1;
    else if (name == "gsign") gyro_sign = (value < 0) ? -1 : 1;
    else { print_both("[CAUTION] Unknown parameter: " + name); continue; }
    sun_print_params();
  }
  print_both("[INFO] Parameters set.");
}
