#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_

#include <list>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tuple>

#include "device_io.h"
#include "info_type.h"

const int kCourseParamNum = 23;

class Luminous {
 public:
  Luminous(SensorIo* sensor_io, Camera* camera);
  void Update();
  Color color_;
  Rgb rgb_;
  Hsv hsv_;

 private:
  void SetColorReference(Color c, Hsv hsv);
  void UpdateRgb();
  void UpdateHsv();
  void UpdateColor();
  SensorIo* sensor_io_;
  Camera* camera_;
  Hsv color_ref_[kColorNum];
};

class Odometry {
  public:
   Odometry(MotorIo* motor_io);
   void Update();
   double distance = 0;
   double x = 0;
   double y = 0;

  private:
   MotorIo* motor_io_;
   const int8_t R = 45;
   const int8_t D = 126;
   double theta_wa = 0;
};

class CubicSpline {
  // public:
  //  CubicSpline();
  //  void setCourseParam();
  //  double CalcEndpoint(const std::list<double> y);
  //  double Calc(double t);
  //  double accl;

  // private:
  //  std::list<double> a_;
  //  std::list<double> b_;
  //  std::list<double> c_;
  //  std::list<double> d_;
  //  std::list<double> w_;
};

class PurePursuit {
  public:
   PurePursuit();
   double x, y, yaw;
   void Update(double x, double y);

  private:
   double calc_distance(double point_x, double point_y);
   void readTargetCourseCoordinate();
   std::tuple<int, double> pursuit_control(int ind);
   std::tuple<int, double> search_target_index();

   int pre_point_index;
   const double lf = 0;

   const float course_x[kCourseParamNum] = {};
   const float course_y[kCourseParamNum] = {};
   
  //  CubicSpline* cubic_spline_;
};

class Localize {
 public:
  Localize(MotorIo* motor_io);
  void Update();
  double distance_ = 0;
  double odometry_x = 0;
  double odometry_y = 0;

 private:
  Odometry* odometry_;
  PurePursuit* pure_pursuit_;
};

#endif  // ETRC22_ETRC_INFO_H_
