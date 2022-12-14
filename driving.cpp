#include "driving.h"
#include "ctime"

WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
  counts_lowpassed_l__ = motor_io -> counts_lowpassed_l_;
  counts_lowpassed_r__ = motor_io -> counts_lowpassed_r_;
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  int8_t curr_power_l = motor_io_->power_l_;
  if (target_power_l > curr_power_l) {
    curr_power_l += 1;
  } else if (target_power_l < curr_power_l) {
    curr_power_l -= 1;
  }

  int8_t curr_power_r = motor_io_->power_r_;
  if (target_power_r > curr_power_r) {
    curr_power_r += 1;
  } else if (target_power_r < curr_power_r) {
    curr_power_r -= 1;
  }

  if (target_power_l == 0 && target_power_r == 0) {
    motor_io_->StopWheels(true);
  } else {
    motor_io_->SetWheelsPower(curr_power_l, curr_power_r);
  }
}

BasicDriver::BasicDriver(WheelsControl* wheels_control)
    : wheels_control_(wheels_control),
      move_type_(kInvalidMove), base_power_(0) {
}

BasicDriver::~BasicDriver() {
}

void BasicDriver::SetParam(Move move_type, int8_t base_power) {
  move_type_ = move_type;
  base_power_ = base_power;
}

void BasicDriver::Run() {
  counts_l_last = counts_l_now;
  counts_r_last = counts_r_now;
  counts_l_now = wheels_control_ -> counts_lowpassed_l__;
  counts_r_now = wheels_control_ -> counts_lowpassed_r__;
  
  velocity_l = (counts_l_now - counts_l_last)/delta_time;
  velocity_r = (counts_r_now - counts_r_last)/delta_time;
  
  if (move_type_ == kGoForward) {
    velocity_target_l = 720;
    velocity_target_r = 720;
    PidControlVelocity();
  } else if (move_type_ == kGoBackward) {
    power_l = power_r = -base_power_;
  } else if (move_type_ == kRotateLeft) {
    power_l = -base_power_;
    power_r = base_power_;
  } else if (move_type_ == kRotateRight) {
    power_l = base_power_;
    power_r = -base_power_;
  } else {
    power_l = power_r = 0;
  }

  wheels_control_->Exec(power_l, power_r);

  /////////////////save data/////////////////
  motor[0][index] = counts_l_now;
  motor[1][index] = counts_r_now;
  velocity[0][index] = velocity_l;
  velocity[1][index] = velocity_r;
  error[0][index] = error_l;
  error[1][index] = error_r;
  motorpower[0][index] = power_l;
  motorpower[1][index] = power_r;
  index ++;
  /////////////////save data/////////////////
}

void BasicDriver::Stop() {
  wheels_control_->Exec(0, 0);
}

/////////////////PIDcontrol/////////////////
void BasicDriver::PidControlVelocity(){
  error_last_l = error_l;
  error_last_r = error_r;
  error_l = velocity_target_l - velocity_l;
  error_r = velocity_target_r - velocity_r;
  error_integral_l += error_l * delta_time;
  error_integral_r += error_r * delta_time;
  error_differential_l = (error_l - error_integral_l) / delta_time;
  error_differential_r = (error_r - error_integral_r) / delta_time;

  power_l = (int)(gain_velocity_control[0][0] * error_l + gain_velocity_control[0][1] * error_integral_l + gain_velocity_control[0][2] * error_differential_l);
  power_r = (int)(gain_velocity_control[1][0] * error_r + gain_velocity_control[1][1] * error_integral_r + gain_velocity_control[1][2] * error_differential_r);

  if(power_l > 100){
    power_l = 100;
  }else if(power_l < -100){
    power_l = -100;
  }
  if(power_r > 100){
    power_r = 100;
  }else if(power_r < -100){
    power_r = -100;
  }
}
/////////////////PIDcontrol/////////////////

/////////////////save data/////////////////
void BasicDriver::SavePidData(){
char str [256];
  char file_name[64];
  FILE* fp;

  time_t timer = time(NULL);
  struct tm* t = localtime(&timer);

  sprintf(file_name, "bonusgetter/data/comparison_pid (%d???%d???%d:%d:%d).csv", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
  fp = fopen(file_name, "w");

  sprintf(str, "counts_l_now ,velocity_l ,error_l ,motorpower_l ,counts_r_now ,velocity_r ,error_r ,motorpower_r\n");
  fprintf(fp, str);
  for (int i = 0; i < index;  i++) {
    sprintf(str,"%f,%f,%f,%d,%f,%f,%f,%d\n",motor[0][index],velocity[0][i],error[0][i],motorpower[0][i],motor[1][index],velocity[1][i],error[1][i],motorpower[1][i]);
    fprintf(fp, str);
  }

  fclose(fp);
}
/////////////////save data/////////////////

LineTracer::LineTracer(WheelsControl* wheels_control, Luminous* luminous)
    : wheels_control_(wheels_control), luminous_(luminous),
      move_type_(kInvalidMove), base_power_(0) {
  pid_control_ = new PidControl();
}

LineTracer::~LineTracer() {
  delete pid_control_;
}

void LineTracer::SetParam(Move move_type, int8_t base_power, Gain gain) {
  move_type_ = move_type;
  base_power_ = base_power;
  pid_control_->SetGain(gain.kp, gain.ki, gain.kd);
}

void LineTracer::Run() {
  float curr_hsv = luminous_->hsv_.v;
  float mv = pid_control_->CalcMv(line_trace_threshold, curr_hsv);

  if (move_type_ == kTraceLeftEdge) {
    mv *= -1;
  }

  int8_t power_l = static_cast<int8_t>(base_power_ + mv);
  int8_t power_r = static_cast<int8_t>(base_power_ - mv);
  wheels_control_->Exec(power_l, power_r);
}

void LineTracer::Stop() {
  wheels_control_->Exec(0, 0);
}

EndCondition::EndCondition(Luminous* luminous, Localize* localize)
    : luminous_(luminous), localize_(localize),
      end_type_(kInvalidEnd), end_color_(kInvalidColor), end_threshold_(0),
      end_state_(false), ref_distance_(0), ref_theta_(0) {
}

void EndCondition::SetParam(End end_type, Color end_color, float end_threshold) {
  end_type_ = end_type;
  end_color_ = end_color;
  end_threshold_ = end_threshold;
  end_state_ = false;

  if(end_type_ == kDistanceEnd){
    ref_distance_ = localize_->distance_;
  }
  //}else if(end_type_ == kThetaEnd){
  //  ref_theta_ = localize_->pose_.theta;
  //}
}

bool EndCondition::IsSatisfied() {
  switch (end_type_) {
    case kColorEnd:
      if (end_color_ == luminous_->color_)
        end_state_ = true;
      break;

    case kDistanceEnd:
      if (end_threshold_ > 0 && localize_->distance_ - ref_distance_ > end_threshold_)
        end_state_ = true;
      else if (end_threshold_ < 0 && localize_->distance_ - ref_distance_ < end_threshold_)
        end_state_ = true;
      break;

    //case kThetaEnd:
    //  if (end_threshold_ > 0 && localize_->pose_.theta - ref_theta_ > end_threshold_)
    //    end_state_ = true;
    //  else if (end_threshold_ < 0 && localize_->pose_.theta - ref_theta_ < end_threshold_)
    //    end_state_ = true;
    //  break;

    default:
      break;
  }

  return end_state_;
}

DrivingManager::DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition)
    : basic_driver_(basic_driver), line_tracer_(line_tracer), end_condition_(end_condition) {
}

void DrivingManager::Update() {
  if (driving_params_.size() <= 0) {
    return;
  }

  DrivingParam& curr_param = driving_params_.front();
  if (!curr_param.is_started) {
    SetMoveParam(curr_param);
    SetEndParam(curr_param);
    curr_param.is_started = true;
  }

  Drive(curr_param);

  if (end_condition_->IsSatisfied()) {
    curr_param.is_finished = true;
  }

  if (curr_param.is_finished) {
    driving_params_.pop_front();
  }

  if (driving_params_.empty()) {
    basic_driver_->Stop();
  }
}

void DrivingManager::AddDrivingParam(DrivingParam param) {
  driving_params_.push_back(param);
}

bool DrivingManager::DrivingParamsEmpty() {
  return driving_params_.empty();
}

void DrivingManager::SetMoveParam(DrivingParam& param) {
  Move move_type = param.move_type;
  int8_t base_power = param.base_power;
  Gain gain = param.gain;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
      line_tracer_->SetParam(move_type, base_power, gain);
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->SetParam(move_type, base_power);
      break;

    default:
      break;
  }
}

void DrivingManager::SetEndParam(DrivingParam& param) {
  End end_type = param.end_type;
  Color end_color = param.end_color;
  float end_threshold = param.end_threshold;

  end_condition_->SetParam(end_type, end_color, end_threshold);
}

void DrivingManager::Drive(DrivingParam& param) {
  Move move_type = param.move_type;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
      line_tracer_->Run();
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->Run();
      break;

    case kStopWheels:
      basic_driver_->Stop();
      break;

    default:
      break;
  }
}
