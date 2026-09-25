// Dead-reckoning odometry for the real robot: there is no odometry sensor,
// so the pose is integrated from the twist the gait actually walks (after
// acceleration limits and the heading correction) and the IMU heading.
// Slip makes it drift (the terrain tests walked 60-100 % of the command):
// good for the hazard guard (it remembers a hazard for seconds and metres),
// not for a map of a whole room. In simulation Gazebo publishes the true
// pose on the same topic instead.
#pragma once

#include <cmath>

namespace dog_control
{

class DeadReckoning
{
public:
  /// Advance by dt with the body twist (vx, vy [m/s], wz [rad/s]).
  /// yaw_imu: measured heading [rad], NaN = none (then wz is integrated).
  void update(double dt, double vx, double vy, double wz, double yaw_imu = std::nan(""))
  {
    if (dt <= 0.0) {return;}
    if (std::isfinite(yaw_imu)) {
      if (!have_imu_) {offset_ = yaw_ - yaw_imu;}  // continue from the current heading
      have_imu_ = true;
      yaw_ = yaw_imu + offset_;
    } else {
      have_imu_ = false;
      yaw_ += wz * dt;
    }
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    x_ += (c * vx - s * vy) * dt;
    y_ += (s * vx + c * vy) * dt;
  }
  double x() const {return x_;}
  double y() const {return y_;}
  double yaw() const {return yaw_;}

private:
  double x_{0.0}, y_{0.0}, yaw_{0.0}, offset_{0.0};
  bool have_imu_{false};
};

}  // namespace dog_control
