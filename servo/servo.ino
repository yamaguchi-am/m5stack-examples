// An example program to output PWM signal for R/C servo.
// Connect the signal pin of an R/C servo to G26.
// The shaft of the servo will rotate back and forth periodically.
#include <M5StickCPlus.h>

const int kLEDCChannel = 1;
const int kServoPin = 26;

// The interval of the PWM signal.
const int kPWMIntervalUsec = 20000;  // [us] = in microseconds
const uint8_t kPWMBitNum = 16;

void SetupPWM(int channel, int pin) {
  ledcSetup(channel, 1e6 / kPWMIntervalUsec, kPWMBitNum);
  ledcAttachPin(kServoPin, channel);
}

void SetPulseWidth(int channel, int microseconds) {
  int v = (1L << kPWMBitNum) * microseconds / ((long)kPWMIntervalUsec);
  ledcWrite(channel, v);
}

void setup(){
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 15);
  M5.Lcd.println("R/C servo PWM example");
  SetupPWM(kLEDCChannel, kServoPin);
}

void loop() {
  const int kCenter = 1500;
  const int kAmplitude = 700;
  const float kFrequency = 0.2;
  int w = kCenter + kAmplitude * sin(1e-3 * millis() * kFrequency * 2 * M_PI);
  SetPulseWidth(kLEDCChannel, w);

  M5.Lcd.setCursor(30, 30);
  M5.Lcd.printf("pulse width = %5d [us]", w);
}
