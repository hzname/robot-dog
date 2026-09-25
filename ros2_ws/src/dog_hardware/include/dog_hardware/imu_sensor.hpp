// Body attitude from an MPU6050-family IMU over Linux i2c-dev, and the
// orientation filter that turns raw gyro + accelerometer into roll / pitch.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dog_hardware
{

using Vec3d = std::array<double, 3>;

struct ImuReading
{
  Vec3d accel{0.0, 0.0, 0.0};  // [m/s^2], sensor frame, +9.81 on the "up" axis at rest
  Vec3d gyro{0.0, 0.0, 0.0};   // [rad/s], sensor frame
};

class ImuSensor
{
public:
  virtual ~ImuSensor() = default;
  virtual bool read(ImuReading & out) = 0;
  virtual std::string describe() const = 0;
};

/// Probes `addresses` for an MPU6050 / MPU6500 / MPU9250 (WHO_AM_I), configures
/// it (+-2 g, +-250 deg/s, 44 Hz low-pass) and returns it, or nullptr.
std::unique_ptr<ImuSensor> probeImu(const std::string & device,
  const std::vector<int> & addresses, std::string & found);

namespace mpu
{
constexpr double kAccelLsbPerG = 16384.0;   // +-2 g
constexpr double kGyroLsbPerDps = 131.0;    // +-250 deg/s
bool knownWhoAmI(uint8_t id);
/// 14 bytes from ACCEL_XOUT_H (0x3B): accel xyz, temperature, gyro xyz.
ImuReading convert(const uint8_t * raw);
}  // namespace mpu

/// Sensor-to-body axis mapping, e.g. "x,y,z" (sensor aligned with the body:
/// x forward, y left, z up) or "-y,x,z" (sensor rotated 90 deg). Each entry
/// says which sensor axis, with sign, is the body axis.
class AxisMap
{
public:
  explicit AxisMap(const std::string & spec = "x,y,z");
  Vec3d apply(const Vec3d & v) const;

private:
  std::array<int, 3> index_{0, 1, 2};
  std::array<double, 3> sign_{1.0, 1.0, 1.0};
};

/// Complementary filter: the gyro carries the attitude, the accelerometer
/// pulls roll / pitch towards gravity with time constant `tau`. Accelerometer
/// samples far from 1 g (steps, bumps) are not trusted. Yaw is gyro-only.
class AttitudeFilter
{
public:
  explicit AttitudeFilter(double tau = 1.0, double accel_gate = 0.15)
  : tau_(tau), gate_(accel_gate) {}

  /// `dt` [s]; the first call (or dt <= 0) initialises from the accelerometer.
  void update(const ImuReading & body, double dt);
  bool initialised() const {return init_;}
  double roll() const {return roll_;}
  double pitch() const {return pitch_;}
  double yaw() const {return yaw_;}
  /// Orientation quaternion (x, y, z, w), REP-103: R = Rz(yaw) Ry(pitch) Rx(roll).
  std::array<double, 4> quaternion() const;

private:
  double tau_;
  double gate_;
  bool init_{false};
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
};

/// Averages the gyro while the robot stands still at start-up.
class GyroBias
{
public:
  explicit GyroBias(int samples) : needed_(samples) {}
  /// Feed one reading; returns true once the bias is known. Motion
  /// (rate > 0.05 rad/s) restarts the average.
  bool add(const Vec3d & gyro);
  bool done() const {return n_ >= needed_;}
  Vec3d bias() const;

private:
  int needed_;
  int n_{0};
  Vec3d sum_{0.0, 0.0, 0.0};
};

}  // namespace dog_hardware
