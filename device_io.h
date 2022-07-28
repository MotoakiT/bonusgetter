#ifndef ETRC22_DEVICE_IO_H_
#define ETRC22_DEVICE_IO_H_

#include "ev3api.h"

class MotorIo {
 public:
  MotorIo();
  ~MotorIo();
  void Update();
  void SetWheelsPower(int8_t power_l, int8_t power_r);
  void StopWheels(bool brake);
  void TestRun();
  void TurnLeft();

  int32_t counts_l_;
  int32_t counts_r_;
  int8_t power_l_;
  int8_t power_r_;
  float counts_lowpassed_l_ = 0;
  float counts_lowpassed_r_ = 0;

 private:
  void ResetCounts();

  /////////////////lowpass/////////////////
  void LowPass(int* counts_l_, int* counts_r_);
  float Ad[2][2] = {{1.044 ,-0.5447},{0.5, 0}};
  float Bd[1][2] = {{0.5}, {0}};
  float Cd[2] = {0.3479, 0.1663};
  float Dd = 0.05715;
  float x_k_l[1][2] = {};
  float x_k_r[1][2] = {};
  float x_kn_l[1][2] = {};
  float x_kn_r[1][2] = {};
  float y_k_l = 0;
  float y_k_r = 0;
  /////////////////lowpass/////////////////
};

class SensorIo {
 public:
  SensorIo();
  ~SensorIo();
  void Update();

  bool enter_button_pressed_;
  rgb_raw_t color_rgb_raw_;
  int16_t ultrasonic_sensor_distance_;
};

class Camera {
 public:
  Camera();
  ~Camera();
  void Update();
};

#endif  // ETRC22_DEVICE_IO_H_
