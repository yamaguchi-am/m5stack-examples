#ifndef IMU_H_
#define IMU_H_

#include <MadgwickAHRS.h>

#define CALIB_DATA_SIZE 100

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
  float x_[CALIB_DATA_SIZE];
  float y_[CALIB_DATA_SIZE];
  float z_[CALIB_DATA_SIZE];
  int count_;
  bool ready_;
  Gyro offset_;
};

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

#endif  // IMU_H_
