# This is the Arduino MRSC (Micro Rc Stability Control) for standard 3 pin RC servos!
## Features:
- For RC cars with standard 3pin servo, ESC and receiver
- Provides about the same functionality as the Traxxas TSM stability management system
- The car tries to follow the direction, which is requested from the steering wheel. Deviations are compensated with countersteering
- I also have an Adruino RC system with fully integrated MRSC: https://github.com/TheDIYGuy999/Micro_RC_Receiver
- Programmable with Arduino IDE
- Input for standard 1000 - 2000us servo pulses
- The throttle signal is passed thru and used as a speed reference for the MRSC calculations
- The steering signal is read and used as the steering angle reference
- A new steering signal is then computed in accordance with the reading from the gyro sensor and sent to the servo
- Uses an 8MHz, 3.3V Pro Mini
- MPU-6050 gyro / accelerometer wired to the I2C pins
- Gain potentiometer
- Input for gyro direction inversion (if the MPU-6050 is mounted upside down)

## Changelog:

New in V 0.1
- Initial commit
- Works fine on the bench
- Not yet tested in a car

## Usage

See pictures

First test
![](https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo/blob/master/Test_rig.jpg)

(c) 2018 TheDIYGuy999
