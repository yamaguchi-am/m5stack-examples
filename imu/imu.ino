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

#include <MadgwickAHRS.h>

const size_t kCalibDataSize = 100;
const float kSamplingFrequency = 100.;
const float kIntervalMicros = 1e6 / kSamplingFrequency;

struct Gyro {
  float x;
  float y;
  float z;
};

class GyroCalibrationData {
 public:
  void Add(float x, float y, float z);
  const Gyro &GetOffset() const { return offset_; };
  bool Ready() const { return ready_; }
  void Clear() {
    ready_ = 0;
    count_ = 0;
  }

 private:
  float x_[kCalibDataSize];
  float y_[kCalibDataSize];
  float z_[kCalibDataSize];
  int count_;
  bool ready_;
  Gyro offset_;
};

float Average(float *data, size_t n) {
  float sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += data[i];
  }
  return sum / n;
}

void GyroCalibrationData::Add(float x, float y, float z) {
  if (count_ < kCalibDataSize) {
    x_[count_] = x;
    y_[count_] = y;
    z_[count_] = z;
    count_++;
    if (count_ == kCalibDataSize) {
      // TODO: Evaluate the variance to make sure the unit was kept stable
      // during calibration.
      offset_.x = Average(x_, kCalibDataSize);
      offset_.y = Average(y_, kCalibDataSize);
      offset_.z = Average(z_, kCalibDataSize);
      ready_ = true;
    }
  }
}

class M5StackIMUManager {
 public:
  void Init();
  // Whether the calibration has been finished.
  bool Ready() const { return calib_data_.Ready(); }
  void StartCalibration();
  void Update(float sample_frequency, float gyro_x, float gyro_y, float gyro_z,
              float acc_x, float acc_y, float acc_z);
  void GetAHRSData(float *pitch, float *roll, float *yaw);

 private:
  Madgwick filter_;
  GyroCalibrationData calib_data_;
};

void M5StackIMUManager::StartCalibration() { calib_data_.Clear(); };

void M5StackIMUManager::Update(float sampling_frequency, float gyro_x,
                               float gyro_y, float gyro_z, float acc_x,
                               float acc_y, float acc_z) {
  if (!calib_data_.Ready()) {
    calib_data_.Add(gyro_x, gyro_y, gyro_z);
  }

  const auto &o = calib_data_.GetOffset();
  gyro_x = gyro_x - o.x;
  gyro_y = gyro_y - o.y;
  gyro_z = gyro_z - o.z;
  // Madgwick.begin() just sets the sampling frequency used in updateIMU().
  // Therefore we can call it multiple times.
  filter_.begin(sampling_frequency);
  filter_.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
}

void M5StackIMUManager::GetAHRSData(float *pitch, float *roll, float *yaw) {
  // in [degree].
  *pitch = filter_.getPitch();
  *roll = filter_.getRoll();
  *yaw = filter_.getYaw();
}

void M5StackIMUManager::Init() {
  M5.IMU.Init();
#ifdef ARDUINO_M5Stick_C_PLUS
  M5.IMU.SetGyroFsr(MPU6886::GFS_2000DPS);
  M5.IMU.SetAccelFsr(MPU6886::AFS_2G);
#else
  M5.IMU.setGyroFsr(MPU6886::GFS_2000DPS);
  M5.IMU.setAccelFsr(MPU6886::AFS_2G);
#endif
  StartCalibration();
}

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
