#include "imu.h"

#ifdef ARDUINO_M5STACK_STICKC_PLUS

#include <M5StickCPlus.h>

#elif defined(ARDUINO_M5STACK_FIRE)

#define M5STACK_MPU6886
#include <M5Stack.h>

#else
#error("selected board is not yet supported")
#endif

#include <MadgwickAHRS.h>

float Average(float *data, size_t n) {
  float sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += data[i];
  }
  return sum / n;
}

void GyroCalibrationData::Add(float x, float y, float z) {
  if (count_ < CALIB_DATA_SIZE) {
    x_[count_] = x;
    y_[count_] = y;
    z_[count_] = z;
    count_++;
    if (count_ == CALIB_DATA_SIZE) {
      // TODO: Evaluate the variance to make sure the unit was kept stable
      // during calibration.
      offset_.x = Average(x_, CALIB_DATA_SIZE);
      offset_.y = Average(y_, CALIB_DATA_SIZE);
      offset_.z = Average(z_, CALIB_DATA_SIZE);
      ready_ = true;
    }
  }
}

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
#ifdef ARDUINO_M5STACK_STICKC_PLUS
  M5.IMU.SetGyroFsr(MPU6886::GFS_2000DPS);
  M5.IMU.SetAccelFsr(MPU6886::AFS_2G);
#else
  M5.IMU.setGyroFsr(MPU6886::GFS_2000DPS);
  M5.IMU.setAccelFsr(MPU6886::AFS_2G);
#endif
  StartCalibration();
}
