// A simple example of IMU using M5StickCPlus / M5StackFire.
// Install Madgwick library from Arduino's Library Manager.
// https://github.com/arduino-libraries/MadgwickAHRS

#ifdef ARDUINO_M5Stick_C_PLUS

#include <M5StickCPlus.h>

#elif defined(ARDUINO_M5STACK_FIRE)

#define M5STACK_MPU6886
#include <M5Stack.h>

#else
#error("selected board is not yet supported")
#endif

#include "imu.h"

const float kSamplingFrequency = 100.;
const float kIntervalMicros = 1e6 / kSamplingFrequency;

M5StackIMUManager imu;
long last_time_micros;

void setup() {
  M5.begin();
#ifdef ARDUINO_M5Stick_C_PLUS
  M5.Lcd.setRotation(3);
#endif
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(80, 15);
  M5.Lcd.println("IMU TEST");
  M5.Lcd.setCursor(30, 30);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(30, 70);
  M5.Lcd.println("  Pitch   Roll    Yaw");
  imu.Init();
  last_time_micros = micros();
}

void loop() {
  float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
  M5.IMU.getGyroData(&gyro_x, &gyro_y, &gyro_z);
  M5.IMU.getAccelData(&acc_x, &acc_y, &acc_z);
  imu.Update(kSamplingFrequency, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
  float pitch, roll, yaw;
  imu.GetAHRSData(&pitch, &roll, &yaw);
  float temp;
  M5.IMU.getTempData(&temp);

  M5.Lcd.setCursor(30, 40);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyro_x, gyro_y, gyro_z);
  M5.Lcd.setCursor(170, 40);
  M5.Lcd.print("deg/s");
  M5.Lcd.setCursor(30, 50);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", acc_x, acc_y, acc_z);
  M5.Lcd.setCursor(170, 50);
  M5.Lcd.print("G");
  M5.Lcd.setCursor(30, 80);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

  M5.Lcd.setCursor(30, 95);
  M5.Lcd.printf("Temperature : %.2f C", temp);

  M5.Lcd.setCursor(30, 110);
  if (imu.Ready()) {
    M5.Lcd.printf("Press BtnA to recalibrate");
  } else {
    M5.Lcd.printf("     calibrating...      ");
  }

  M5.update();
  if (M5.BtnA.wasPressed()) {
    imu.Init();
  }

  long next_time_micros = last_time_micros + kIntervalMicros;
  while (micros() < next_time_micros) {
    delay(1);
  }
  last_time_micros = next_time_micros;
}
