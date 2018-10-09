#ifndef mpu_h
#define mpu_h

#include "Arduino.h"

/* This code is based on Joop Brokkings excellent work:
  http://www.brokking.net/imu.html
  https://www.youtube.com/watch?v=4BoIE8YQwM8
  https://www.youtube.com/watch?v=j-kE0AMEWy4

  I (TheDIYGuy999) have modified it to fit my needs

  -->> Note:
  - The adapter will not work, if  no MPU-6050 sensor is wired up!!
  - The measurements are taken with 125Hz (8ms) refresh rate. Reason: processing all the code requires up to
    7ms loop time with 8MHz MCU clock.
*/

//
// =======================================================================================================
// GLOBAL VARIABLES
// =======================================================================================================
//

// 6050 variables
int gyro_x, gyro_y, gyro_z;
long acc_x_raw, acc_y_raw, acc_z_raw;
int temperature;
long gyro_z_cal;
float yaw_rate = 0;
float yaw_angle = 0;
long acc_z = 0;
long acc_z_diff = 0;
long acc_z_old;

// configuration variables (you may have to change them)
const int calibrationPasses = 125; // 125 is useful, so the gyro calibration takes 1s @ 125Hz refresh rate
int movement_threshold = 80; // (80) Vehicle movement is detected as soon as the z acceleration exceeds this value

//
// =======================================================================================================
// PROCESS MPU 6050 DATA SUBFUNCTION
// =======================================================================================================
//

void processMpu6050Data() {

  // Subtract the offset calibration value from the raw gyro_z value
  gyro_z -= gyro_z_cal;

  //Gyro angle calculations
  //0.0000611 = 1 / (125Hz / 131) in theory, adjust until result is absolutely accurate!
  yaw_rate = (gyro_z * 0.0000646); // Yaw rate in degrees per second
  yaw_angle = yaw_angle + yaw_rate;

  // Measure acceleration (for movement detection)
  acc_z = acc_z_raw;
  acc_z_diff = abs(acc_z_old - acc_z);
  acc_z_old = acc_z;

  // Reset yaw_angle (heading direction), if vehicle is not moving during 500ms!
  static unsigned long lastMovement;
  if (acc_z_diff > movement_threshold) lastMovement = millis();
  //if (millis() - lastMovement >= 500) yaw_angle = 0;
}

//
// =======================================================================================================
// READ MPU 6050 RAW DATA FUNCTION
// =======================================================================================================
//

// Sub function allows setup to call it without delay
void readMpu6050Raw() {
  Wire.beginTransmission(0x68);                                        // Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    // Send the requested starting register
  Wire.endTransmission();                                              // End the transmission
  Wire.requestFrom(0x68, 14);                                          // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       // Wait until all the bytes are received
  acc_x_raw = Wire.read() << 8 | Wire.read();                          // Add the low and high byte to the acc_x variable
  acc_y_raw = Wire.read() << 8 | Wire.read();                          // Add the low and high byte to the acc_y variable
  acc_z_raw = Wire.read() << 8 | Wire.read();                          // Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        // Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             // Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             // Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             // Add the low and high byte to the gyro_z variable
}

// Main function
void readMpu6050Data() {
  static unsigned long lastReading;
  if (micros() - lastReading >= 8000) {                                // Read the data every 8000us (equals 125Hz)
    lastReading = micros();

    readMpu6050Raw();                                                  // Read RAW data

    processMpu6050Data();                                              // Process the MPU 6050 data
  }
}

//
// =======================================================================================================
// MPU 6050 SETUP
// =======================================================================================================
//

void setupMpu6050() {

  Wire.begin();                                                        // Start I2C as master
  // Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        // Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    // Send the requested starting register
  Wire.write(0x00);                                                    // Set the requested starting register
  Wire.endTransmission();                                              // End the transmission
  // Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        // Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    // Send the requested starting register
  Wire.write(0x10);                                                    // Set the requested starting register
  Wire.endTransmission();                                              // End the transmission
  // Configure the gyro (250° per second full scale)
  Wire.beginTransmission(0x68);                                        // Start communicating with the MPU-6050
  Wire.write(0x00);   //0x00 = FS_SEL0 = Full scale range +/-250°/s    // Send the requested starting register
  Wire.write(0x18);                                                    // Set the requested starting register
  Wire.endTransmission();                                              // End the transmission

  // Calibrate the gyro (the vehicle must stay steady during this time!)
  int cal_int = 0;
  while (cal_int < calibrationPasses) {                                // Run the calibrating code X times
    static unsigned long lastGyroCal;
    if (micros() - lastGyroCal >= 8000) {                              // Read the data every 8000us (equals 125Hz)
#ifdef DEBUG
      if (cal_int % (calibrationPasses / 32) == 0)Serial.print(".");   // Print a dot every X readings
#endif
      readMpu6050Raw();                                                // Read the raw acc and gyro data from the MPU-6050
      gyro_z_cal += gyro_z;                                            // Add the gyro z-axis offset to the gyro_z_cal variable
      lastGyroCal = micros();
      cal_int ++;
    }
  }
  gyro_z_cal /= calibrationPasses;                                      // Divide the gyro_z_cal variable by X to get the avarage offset
}

#endif
