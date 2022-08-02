#include "device_io.h"
#include <stdio.h>

MotorIo::MotorIo() : counts_l_(0), counts_r_(0) {
  ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);
  ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
  ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);
  ResetCounts();
}

MotorIo::~MotorIo() {
  StopWheels(false);
}

void MotorIo::Update() {
  counts_l_ = ev3_motor_get_counts(EV3_PORT_C);
  counts_r_ = ev3_motor_get_counts(EV3_PORT_B);
  power_l_ = static_cast<int8_t>(ev3_motor_get_power(EV3_PORT_C));
  power_r_ = static_cast<int8_t>(ev3_motor_get_power(EV3_PORT_B));
  /////////////////lowpass/////////////////
  LowPass();
  /////////////////lowpass/////////////////
  
  /////////////////save data/////////////////
  now_angle[0][index] = counts_l_;
  now_angle[1][index] = counts_r_;
  lowpass_angle[0][index] = counts_lowpassed_l_;
  lowpass_angle[1][index] = counts_lowpassed_r_;
  index ++;
  /////////////////save data/////////////////
}

void MotorIo::SetWheelsPower(int8_t power_l, int8_t power_r) {
  const int8_t kUpperLimit = 100;
  const int8_t kLowerLimit = -100;

  if (power_l > kUpperLimit) {
    power_l = kUpperLimit;
  } else if (power_l < kLowerLimit) {
    power_l = kLowerLimit;
  }
  ev3_motor_set_power(EV3_PORT_C, power_l);

  if (power_r > kUpperLimit) {
    power_r = kUpperLimit;
  } else if (power_r < kLowerLimit) {
    power_r = kLowerLimit;
  }
  ev3_motor_set_power(EV3_PORT_B, power_r);
}

void MotorIo::StopWheels(bool brake) {
  ev3_motor_stop(EV3_PORT_B, brake);
  ev3_motor_stop(EV3_PORT_C, brake);
}

void MotorIo::ResetCounts() {
  ev3_motor_reset_counts(EV3_PORT_B);
  ev3_motor_reset_counts(EV3_PORT_C);
}

void MotorIo::TurnLeft() {
  int turn_power = 50;
  int turn_ratio = 50;
  ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, turn_power, turn_ratio);
}

void MotorIo::TestRun() {
  int left_power = 50;
  int right_power = 50;
  ev3_motor_set_power(EV3_PORT_C, left_power);
  ev3_motor_set_power(EV3_PORT_B, right_power);
}

/////////////////lowpass/////////////////
void MotorIo::LowPass(){
  x_k_l[0][0] = x_kn_l[0][0];
  x_k_l[1][0] = x_kn_l[1][0];
  x_k_r[0][0] = x_kn_r[0][0];
  x_k_r[1][0] = x_kn_r[1][0];
  x_kn_l[0][0] = Ad[0][0] * x_k_l[0][0] + Ad[0][1] * x_k_l[1][0] + Bd[0][0] * counts_l_;
  x_kn_r[0][0] = Ad[0][0] * x_k_r[0][0] + Ad[0][1] * x_k_r[1][0] + Bd[0][0] * counts_r_;
  x_kn_l[0][1] = Ad[1][0] * x_k_l[0][0] + Ad[1][1] * x_k_l[1][0] + Bd[1][0] * counts_l_;
  x_kn_r[0][1] = Ad[1][0] * x_k_r[0][0] + Ad[1][1] * x_k_r[1][0] + Bd[1][0] * counts_r_;
  counts_lowpassed_l_ = Cd[0] * x_k_l[0][0] + Cd[1] * x_k_l[1][0] + Dd * counts_l_;
  counts_lowpassed_r_ = Cd[0] * x_k_r[0][0] + Cd[1] * x_k_r[1][0] + Dd * counts_r_;
}
/////////////////lowpass/////////////////

/////////////////save data/////////////////
void MotorIo::SaveData(){
  char str [256];
  char file_name[64];
  FILE* fp;

  int i = 1;
  while(true){
    snprintf(file_name,sizeof(char)*64,"bonusgetter/data/comparison_lowpass%i.csv",i);

    if(fp = fopen(file_name,"r")){
      fclose(fp);
    } else {
      break;
    }
    i++;
  }

  fp = fopen(file_name, "w");
  sprintf(str, "motor_l ,motar_r ,motor_l_lowpass ,motor_r_lowpass\n");
  fprintf(fp, str);
  for (int i = 0; i < index;  i++) {
    sprintf(str,"%d, %d, %f, %f\n",now_angle[0][i],now_angle[1][i],lowpass_angle[0][i],lowpass_angle[1][i]);
    fprintf(fp, str);
  }

  fclose(fp);
}
/////////////////save data/////////////////

SensorIo::SensorIo()
    : enter_button_pressed_(false), color_rgb_raw_({0, 0, 0}) {
  ev3_sensor_config(EV3_PORT_1, TOUCH_SENSOR);
  ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);
  ev3_sensor_config(EV3_PORT_3, ULTRASONIC_SENSOR);
}

SensorIo::~SensorIo() {
}

void SensorIo::Update() {
  ultrasonic_sensor_distance_ = ev3_ultrasonic_sensor_get_distance(EV3_PORT_3);
  enter_button_pressed_ = ev3_button_is_pressed(ENTER_BUTTON);
  ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &color_rgb_raw_);
}

Camera::Camera() {

}

Camera::~Camera() {
}

void Camera::Update() {
}
