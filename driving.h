#ifndef ETRC22_DRIVING_H_
#define ETRC22_DRIVING_H_

#include <list>

#include "info_type.h"
#include "etrc_info.h"
#include "utils.h"

class WheelsControl {
 public:
  WheelsControl(MotorIo* motor_io);
  void Exec(int8_t target_power_l, int8_t target_power_r);
  float counts_lowpassed_l__ = 0;
  float counts_lowpassed_r__ = 0;

 private:
  MotorIo* motor_io_;
};

class BasicDriver {
 public:
  BasicDriver(WheelsControl* wheels_control);
  ~BasicDriver();
  void SetParam(Move move_type, int8_t base_power);
  void Run();
  void Stop();
  void Lowpass();
  void PidControlVelocity();
  void SavePidData();

 private:
  WheelsControl* wheels_control_;
  Move move_type_;
  int8_t base_power_;

  float counts_l_now = 0;
  float counts_r_now = 0;

  int8_t power_l;
  int8_t power_r;

  /////////////////PIDcontrol/////////////////
  float counts_l_last = 0;
  float counts_r_last = 0;
  float velocity_target_l = 0;
  float velocity_target_r = 0;
  float velocity_l = 0;
  float velocity_r = 0;
  float delta_time = 0.05;
  float error_l = 0;
  float error_r = 0;
  float error_last_l = 0;
  float error_last_r = 0;
  float error_integral_l = 0;
  float error_integral_r = 0;
  float error_differential_l = 0;
  float error_differential_r = 0;
  float gain_velocity_control[2][3] = {{0.05, 0, 0}, {0.05, 0, 0}};
  /////////////////PIDcontrol/////////////////
  
  /////////////////save data/////////////////
  float motor[2][100000] = {};
  float velocity[2][100000] = {};
  float error[2][100000] = {};
  int motorpower[2][100000] = {};
  int index = 0;
  /////////////////save data/////////////////
};

class LineTracer {
 public:
  LineTracer(WheelsControl* wheels_control, Luminous* luminous);
  ~LineTracer();
  void SetParam(Move move_type, int8_t base_power, Gain gain);
  void Run();
  void Stop();

 private:
  WheelsControl* wheels_control_;
  Luminous* luminous_;
  Move move_type_;
  int8_t base_power_;
  const int8_t line_trace_threshold = 40;
  PidControl* pid_control_;
};

class EndCondition {
 public:
  EndCondition(Luminous* luminous, Localize* localize);
  void SetParam(End end_type, Color end_color, float end_threshold);
  bool IsSatisfied();

 private:
  Luminous* luminous_;
  Localize* localize_;
  End end_type_;
  Color end_color_;
  float end_threshold_;
  bool end_state_;
  float ref_distance_;
  float ref_theta_;
};

class DrivingManager {
 public:
  DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition);
  void Update();
  void AddDrivingParam(DrivingParam param);
  bool DrivingParamsEmpty();

 private:
  void SetMoveParam(DrivingParam& param);
  void SetEndParam(DrivingParam& param);
  void Drive(DrivingParam& param);
  BasicDriver* basic_driver_;
  LineTracer* line_tracer_;
  EndCondition* end_condition_;
  //???????????????????????????
  /*
              ??????????????????????????????????????????????????????
  std::vector O(1)	           O(N)
  std::list	  O(N)	           O(1)
  */
  std::list<DrivingParam> driving_params_;
};

#endif  // ETRC22_DRIVING_H_
