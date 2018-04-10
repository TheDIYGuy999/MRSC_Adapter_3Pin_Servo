/* Standalone MRSC "Micro Rc Stability Control" converter for 3 pin RC steering servo
    MPU: Atmega 328P 3.3V, 8MHz
    Board: Pro Mini
    MPU-6050 board: GY-521

   Pins:
   - Steering input 4
   - Throttle input 5 (wired in parallel with your ESC)
   - MRSC Gain potentiometer input A1
   - MRSC direction inversion switch A2
   - Steering output A0
   - MPU-6050 SDA A4
   - MPU-6050 SCL A5
*/

const float codeVersion = 0.2; // Software revision

//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

#include <Wire.h> // I2C library (for the MPU-6050 gyro /accelerometer)
#include <Servo.h> // Servo library

#include "mpu.h" // MPU-6050 handling (.h file in sketch folder)

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

volatile uint8_t prev; // remembers state of input bits from previous interrupt
volatile uint32_t risingEdge[2]; // time of last rising edge for each channel
volatile uint32_t uSec[2]; // the latest measured pulse width for each channel

// Create Servo objects
Servo servoSteering;

// Servo limits (initial value = center position = 90°)
// Limits are adjusted during the first steering operation!
// Always move the steering wheel inside its entire range after switching on the car
byte limSteeringL = 90, limSteeringR = 90; // usually 45 - 135° (90° is the servo center position)

// MRSC gain
byte mrscGain = 80; // This MRSC gain applies, if you don't have an adjustment pot and don't call readInputs()

// Switch states
boolean mpuInversed = false;

// Pin definition (don't change servo imputs, interrupt routine is hardcoded)
#define INPUT_STEERING 4
#define INPUT_THROTTLE 5

#define OUTPUT_STEERING A0

#define GAIN_POT A1
#define INVERSE_MPU_DIRECTION A2

//
// =======================================================================================================
// SERVO INPUT PIN CHANGE INTERRUPT ROUTINE
// =======================================================================================================
// Based on: http://ceptimus.co.uk/?p=66

ISR(PCINT2_vect) { // one or more of pins 4~5 have changed state
  uint32_t now = micros();
  uint8_t curr = PIND; // current state of the 2 input bits
  uint8_t changed = curr ^ prev; // bitwise XOR (= bit has changed)
  uint8_t channel = 0;
  for (uint8_t mask = B00010000; mask <= B00100000 ; mask <<= 1) { // do it with bit 4 and 5 masked
    if (changed & mask) { // this pin has changed state
      if (curr & mask) { // Rising edge, so remember time
        risingEdge[channel] = now;
      }
      else { // Falling edge, so store pulse width
        uSec[channel] = now - risingEdge[channel];
      }
    }
    channel++;
  }
  prev = curr;
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

  // Configure inputs
  pinMode(INVERSE_MPU_DIRECTION, INPUT_PULLUP);
  pinMode(GAIN_POT, INPUT);

  // Activate servo signal input pullup resistors
  pinMode(INPUT_STEERING, INPUT_PULLUP);
  pinMode(INPUT_THROTTLE, INPUT_PULLUP);

  // Interrupt settings. See: http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
  PCMSK2 |= B00110000; // PinChangeMaskRegister: set the mask to allow pins 4-5 to generate interrupts (see page 94)
  PCICR |= B00000100;  // PinChangeInterruptControlRegister: enable interupt for port D (Interrupt Enable 2, see page 92)

  // Servo pins
  servoSteering.attach(OUTPUT_STEERING);
  servoSteering.write((limSteeringL + limSteeringR) / 2); // Servo to center position

  // Initialize servo pulses to center position
  uSec[0] = 1500; // Steering
  uSec[1] = 1500; // Throttle

  // MPU 6050 accelerometer / gyro setup
  setupMpu6050();
}

//
// =======================================================================================================
// DETECT STEERING RANGE
// =======================================================================================================
//

void detectSteeringRange() {
  int servoAngle = map(uSec[0], 1000, 2000, 45, 135); // The range usually is 45 to 135° (+/- 45°)
if (servoAngle > 20 && servoAngle < limSteeringL) limSteeringL = servoAngle;
if (servoAngle < 180 && servoAngle > limSteeringR) limSteeringR = servoAngle;
}

//
// =======================================================================================================
// READ INPUTS
// =======================================================================================================
//

void readInputs() {
  mrscGain = map(analogRead(GAIN_POT), 0, 255, 0, 100);
  mpuInversed = !digitalRead(INVERSE_MPU_DIRECTION);
}

//
// =======================================================================================================
// SIGNAL VALIDITY CHECK
// =======================================================================================================
//

void checkValidity() {
  for (int i = 0; i <= 1; i++) {
    if (uSec[i] < 800 || uSec[i] > 2200) uSec[i] = 1500; // go to neutral, if an invalid servo signal arrives
  }
}

//
// =======================================================================================================
// MRSC (MICRO RC STABILITY CONTROL) CALCULATIONS
// =======================================================================================================
// For cars with stability control (steering overlay depending on gyro yaw rate)

void mrsc() {

  int steeringAngle;

  // Read sensor data
  readMpu6050Data();

  // Compute steering compensation overlay
  int turnRateSetPoint = map(uSec[0], 1000, 2000, -50, 50);  // turnRateSetPoint = steering angle (1000 to 2000us) = -50 to 50
  int steering = abs(turnRateSetPoint);
  int gain = map(steering, 0, 50, mrscGain, (mrscGain / 10)); // more MRSC gain around center position!
  int turnRateMeasured = yaw_rate * 50;//abs(map(uSec[1], 1000, 2000, -50, 50)); // degrees/s * speed (not speed dependent for big cars)
  if (mpuInversed) {
    steeringAngle = turnRateSetPoint + (turnRateMeasured * gain / 100);  // Compensation depending on the pot value
  }
  else {
    steeringAngle = turnRateSetPoint - (turnRateMeasured * gain / 100);
  }

  steeringAngle = constrain (steeringAngle, -50, 50); // range = -50 to 50

  // Control steering servo
  servoSteering.write(map(steeringAngle, -50, 50, limSteeringL, limSteeringR) ); // 45 - 135°
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {
  //readInputs(); // Read pots and switches
  checkValidity(); // Signal valid?
  detectSteeringRange(); // Detect the steering input signal range
  mrsc(); // Do stability control calculations
}
