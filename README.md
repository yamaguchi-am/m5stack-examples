# m5stack-examples
Yet another M5Stack components usage example

## imu: IMU example using embedded MPU6886 sensor
Similar to M5Stack's example code named "IMU".

### Currently Supported Boards
- M5StackFire
- M5StickCPlus

### Background
The AHRS(pose estimation) implementation integrated in the M5Stack's base library doesn't support these functionalities:
1. arbitrary sampling frequency
2. gyro offset removal

This example instead uses [Arduino's Madgwick library](https://github.com/arduino-libraries/MadgwickAHRS) for getting orientation of a M5Stack device based on accelerometer and gyroscope readings.
The example also adds gyro calibration function to capture and adjust the offset of all gyro readings. It is done upon startup, and when the Button A is pressed.
If the "Yaw" value keeps drifting, try to fixate the device and press the Button A.

## servo: R/C servo signal
Generates a PWM signal for R/C servo control using the LED control peripheral.
### Supported Boards
- M5StickCPlus
