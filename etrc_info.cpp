#include "etrc_info.h"

Luminous::Luminous(SensorIo* sensor_io, Camera* camera)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io), camera_(camera) {
}

void Luminous::Update() {
  UpdateRgb();
  UpdateHsv();
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv) {
  color_ref_[c] = hsv;
}

void Luminous::UpdateRgb() {
  rgb_raw_t val = sensor_io_->color_rgb_raw_;

  rgb_.r = val.r;
  rgb_.g = val.g;
  rgb_.b = val.b;
}

void Luminous::UpdateHsv() {
  float r = static_cast<float>(rgb_.r);
  float g = static_cast<float>(rgb_.g);
  float b = static_cast<float>(rgb_.b);

  float max = r > g ? r : g;
  max = max > b ? max : b;
  float min = r < g ? r : g;
  min = min < b ? min : b;
  float c = max - min;

  float h;
  if (c == 0) {
    h = -1;
  } else if (max == r) {
    h = fmodf(((g - b) / c), 6);
  } else if (max == g) {
    h = ((b - r) / c) + 2;
  } else if (max == b) {
    h = ((r - g) / c) + 4;
  } else {
    h = -1;
  }

  if (h != -1) {
    h = 60 * h;
  }

  float s;
  if (max == 0) {
    s = 0;
  } else {
    s = c / max;
  }

  float v = max;
  if (v > 100) {
    v = 100;
  }

  hsv_.h = h;
  hsv_.s = s * 100;
  hsv_.v = v;
}

void Luminous::UpdateColor() {
}

Odometry::Odometry(MotorIo* motor_io)
  : motor_io_(motor_io) {
}

void Odometry::Update() {
  double Ll = R * motor_io_->counts_l_ * M_PI / 180;
  double Lr = R * motor_io_->counts_r_ * M_PI / 180;

  double theta = (Lr - Ll) / D;
  theta_wa += theta;
  double A = (Lr + Ll) / 2 * (1 - 0);
  double dx = A * cos(theta_wa + theta / 2);
  double dy = A * sin(theta_wa + theta / 2);
  double dd = sqrt(dx * dx + dy * dy);

  x += dx;
  y += dy;
  distance += dd;
}

// CubicSpline::CubicSpline() {
//   setCourseParam();
// }
/*
void CubicSpline::setCourseParam() {
  int data = kCourseParamNum - 1;

  for (int i = 0; i <= data; i++) {
    a_.push_back(y[i]);
  }

  for (int i = 0; i < data; i++) {
    if (i == 0) {
      c_.push_back(0.0);
    } else if (i == data) {
      c_.push_back(0.0);
    } else {
      c_.push_back(3.0 * (a_[i-1] - 2.0 * a_[i] + a_[i+1]));
    }
  }

  for (int i = 0; i < data; i++) {
    if (i == 0) {
      w_.push_back(0.0);
    } else {
      double tmp = 4.0 - w_[i-1];
      c_[i] = (c_[i] - c_[i-1]) / tmp;
      w_.push_back(1.0 / tmp);
    }
  }

  for (int i = (data-1); i > 0; i--){
      c_[i] = c_[i] - c_[i+1] * w_[i];
  }

  for (int i = 0; i <= data; i++) {
    if (i == data) {
      d_.push_back(0.0);
      b_.push_back(0.0);

    } else {
      d_.push_back((c_[i+1] - c_[i]) / 3.0);
      b_.push_back(a_[i+1] - a_[i] - c_[i] - d_[i]);
    }
  }
}

double CubicSpline::CalcEndpoint(const std::list<double> y){
    int dt = y.size();

    double dy = b_[j] + (c_[j] + d_[j] * dt) * dt;
    return dy * dy;

    return 0;
}

double CubicSpline::Calc(double t) {
  int j = int(floor(t));
  if (j < 0) {
    j = 0;
  } else if(j >= a_.size()) {
    j = a_.size() - 1;
  }

  double dt = t - j;
  double result = a_[j] + (b_[j] + (c_[j] + d_[j] * dt) * dt) * dt;
  accl = 2 * c_[j] + 6 * d_[j] * dt;

  return result;
}
*/

PurePursuit::PurePursuit()
  : x(0), y(0), yaw(0) {
  // cubic_spline_ = new CubicSpline();
  readTargetCourseCoordinate();
  pre_point_index = INT_MAX;
}

void PurePursuit::readTargetCourseCoordinate() {
  // for (int i=0; i<size; i++) {
  //   course_x[i] = ;
  //   course_y[i] = ;
  // }
}

double PurePursuit::calc_distance(double point_x, double point_y) {
  double dx = x - point_x;
  double dy = y - point_y;

  return hypot(dx, dy);
}


std::tuple<int, double> PurePursuit::search_target_index() {
  int ind;
  if (pre_point_index == INT_MAX) {
    std::list<int> d;

    for (int i = 0; i < kCourseParamNum; i++) {
      double dx = x - course_x[i];
      double dy = y - course_y[i];
      d.push_back(hypot(dx, dy));
    }
    std::list<int>::iterator minIt = std::min_element(d.begin(), d.end());
    ind = std::distance(d.begin(), minIt);
    pre_point_index = ind;

  } else {
    ind = pre_point_index;
    double distance = calc_distance(course_x[ind],course_y[ind]);

    while (true) {
      double next_distance = calc_distance(course_x[ind+1], course_y[ind+1]);
      if (distance < next_distance) break;
      if (ind + 1 < kCourseParamNum) {
        ind++;
      }

      distance = next_distance;
    }

    pre_point_index = ind;
  }

  while (lf > calc_distance(course_x[ind], course_y[ind])) {
    if (ind > kCourseParamNum) break;
    ind += 1;
  }

  return std::forward_as_tuple(ind, lf);
}

std::tuple<int, double> PurePursuit::pursuit_control(int pind) {
  int target_ind;
  double lf;
  std::tie(target_ind, lf) = search_target_index();

  if (pind >= target_ind) {
    target_ind = pind;
  }

  double tx,ty;
  if (target_ind < kCourseParamNum) {
      tx = course_x[target_ind];
      ty = course_y[target_ind];

  } else {
      tx = course_x[kCourseParamNum-1];
      ty = course_y[kCourseParamNum-1];
      target_ind = kCourseParamNum-1;
  }

  double alpha = atan2(ty - y, tx - x);

  return std::forward_as_tuple(target_ind, alpha);
}


void PurePursuit::Update(double odometry_x, double odometry_y) {
  double lf;
  int target_ind;
  x = odometry_x;
  y = odometry_y;

  std::tie(target_ind, lf) = search_target_index();

  double delta;
  std::tie(target_ind, delta) = pursuit_control(target_ind);
}

Localize::Localize(MotorIo* motor_io) {
  odometry_ = new Odometry(motor_io);
  pure_pursuit_ = new PurePursuit();
}

void Localize::Update() {
  odometry_->Update();
  distance_ = odometry_->distance;
  odometry_x = odometry_->x;
  odometry_y = odometry_->y;

  //pure_pursuit_->Update(odometry_x, odometry_y);
}
