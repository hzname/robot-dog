// imu: body attitude from an MPU6050-family IMU (v1 has one at 0x68).
//
// The sensor is optional, like the current sensor: on start the node probes
// the configured I2C addresses; if nothing answers it logs that and exits
// cleanly, and locomotion walks without slope compensation.
//
// Publishes (relative names, launched in the /dog namespace):
//   imu/data  sensor_msgs/Imu  orientation (roll / pitch vs. gravity, yaw from
//             the gyro only), angular velocity, linear acceleration; body axes
//             (x forward, y left, z up) after the `axes` remapping
//
// The gyro bias is measured during the first `bias_time` seconds: keep the
// robot still while the stack starts (it is lying or on its stand anyway).
#include "dog_diagnostics/reporter.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "dog_hardware/imu_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

namespace dog_hardware
{

namespace
{
/// Level, motionless IMU with a settable tilt, for PC runs and tests.
class MockImu : public ImuSensor
{
public:
  explicit MockImu(rclcpp::Node & node) : node_(node) {}
  bool read(ImuReading & out) override
  {
    const double r = node_.get_parameter("mock.roll_deg").as_double() * M_PI / 180.0;
    const double p = node_.get_parameter("mock.pitch_deg").as_double() * M_PI / 180.0;
    const double g = 9.80665;
    out.accel = {-g * std::sin(p), g * std::cos(p) * std::sin(r), g * std::cos(p) * std::cos(r)};
    out.gyro = {0.0, 0.0, 0.0};
    return true;
  }
  std::string describe() const override {return "mock IMU (parameters mock.roll_deg / mock.pitch_deg)";}

private:
  rclcpp::Node & node_;
};
}  // namespace

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : rclcpp::Node("imu")
  {
    const auto backend = declare_parameter("backend", std::string("auto"));
    const auto device = declare_parameter("i2c.device", std::string("/dev/i2c-0"));
    const auto addresses = declare_parameter("i2c.addresses", std::vector<int64_t>{0x68, 0x69});
    const auto rate = declare_parameter("rate", 100.0);
    axes_ = AxisMap(declare_parameter("axes", std::string("x,y,z")));
    filter_ = AttitudeFilter(declare_parameter("filter_tau", 1.0), declare_parameter("accel_gate", 0.15));
    bias_ = GyroBias(std::max(1, static_cast<int>(declare_parameter("bias_time", 1.0) * rate)));
    frame_ = declare_parameter("frame_id", std::string("base_link"));
    declare_parameter("mock.roll_deg", 0.0);
    declare_parameter("mock.pitch_deg", 0.0);

    if (backend == "off") {
      RCLCPP_INFO(get_logger(), "IMU disabled (backend:=off)");
      return;
    }
    if (backend == "mock") {
      sensor_ = std::make_unique<MockImu>(*this);
    } else if (backend == "auto" || backend == "i2c") {
      std::vector<int> addrs(addresses.begin(), addresses.end());
      std::string probed;
      sensor_ = probeImu(device, addrs, probed);
      if (!sensor_) {
        RCLCPP_INFO(get_logger(), "no IMU found (%s) - slope compensation off", probed.c_str());
        return;
      }
    } else {
      throw std::runtime_error("unknown backend '" + backend + "' (auto, mock or off)");
    }
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    // Whether the gyro bias is known. Latched: a controller that starts after the
    // IMU still learns it, and the floor profile refuses to stand without it.
    cal_pub_ = create_publisher<std_msgs::msg::Bool>(
      "imu/calibrated", rclcpp::QoS(1).transient_local().reliable());
    // Test hook: after this many seconds the node goes quiet, exactly like a dead
    // sensor. Without it the "IMU lost" path cannot be reproduced in CI (SAF-06).
    silence_after_ = declare_parameter("mock.silence_after", 0.0);
    diag_ = std::make_unique<dog_diagnostics::Reporter>(this, "imu");
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {report();});
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "IMU: %s; measuring gyro bias, keep the robot still",
      sensor_->describe().c_str());
  }

  bool active() const {return sensor_ != nullptr;}

private:
  /// Calibrated or not, and whether the numbers are still coming.
  void report()
  {
    using dog_diagnostics::Level;
    Level level = calibrated_ ? Level::OK : Level::WARN;
    std::string reason = calibrated_ ? "calibrated" : "calibrating the gyro";
    if (quiet_announced_) {
      level = Level::ERROR;
      reason = "imu silent (test hook)";
    } else if (read_failures_ > 0) {
      level = level == Level::OK ? Level::WARN : level;
      reason = "read failures: " + std::to_string(read_failures_);
    }
    diag_->publish(level, reason, {
      {"calibrated", calibrated_ ? "true" : "false"},
      {"read_failures", std::to_string(read_failures_)}});
  }

  void tick()
  {
    if (silence_after_ > 0.0) {
      const double t = now().seconds();
      if (quiet_since_ == 0.0) {quiet_since_ = t;}
      if (t - quiet_since_ > silence_after_) {
        if (!quiet_announced_) {
          quiet_announced_ = true;
          RCLCPP_WARN(get_logger(), "imu silent on purpose after %.1f s (test hook)", silence_after_);
        }
        return;
      }
    }
    ImuReading raw;
    if (!sensor_->read(raw)) {
      ++read_failures_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU read failed");
      return;
    }
    ImuReading b{axes_.apply(raw.accel), axes_.apply(raw.gyro)};
    const auto t = std::chrono::steady_clock::now();
    if (!bias_.done()) {
      if (bias_.add(b.gyro)) {
        const auto g = bias_.bias();
        RCLCPP_INFO(get_logger(), "gyro bias %.4f %.4f %.4f rad/s", g[0], g[1], g[2]);
        calibrated_ = true;
        cal_pub_->publish(std_msgs::msg::Bool().set__data(true));
      }
      filter_.update(b, 0.0);
      last_ = t;
      return;
    }
    const auto g = bias_.bias();
    for (int k = 0; k < 3; ++k) {b.gyro[k] -= g[k];}
    const double dt = std::min(std::chrono::duration<double>(t - last_).count(), 0.1);
    last_ = t;
    filter_.update(b, dt);
    if (!filter_.initialised()) {return;}

    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = frame_;
    const auto q = filter_.quaternion();
    m.orientation.x = q[0];
    m.orientation.y = q[1];
    m.orientation.z = q[2];
    m.orientation.w = q[3];
    m.orientation_covariance = {0.0003, 0, 0, 0, 0.0003, 0, 0, 0, 1.0};  // yaw drifts
    m.angular_velocity.x = b.gyro[0];
    m.angular_velocity.y = b.gyro[1];
    m.angular_velocity.z = b.gyro[2];
    m.angular_velocity_covariance = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};
    m.linear_acceleration.x = b.accel[0];
    m.linear_acceleration.y = b.accel[1];
    m.linear_acceleration.z = b.accel[2];
    m.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    pub_->publish(m);
  }

  std::unique_ptr<ImuSensor> sensor_;
  AxisMap axes_;
  AttitudeFilter filter_;
  GyroBias bias_{100};
  std::string frame_;
  std::chrono::steady_clock::time_point last_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cal_pub_;
  bool calibrated_{false};
  double silence_after_{0.0};   // [s] test hook: go quiet after this
  double quiet_since_{0.0};
  bool quiet_announced_{false};
  int read_failures_{0};
  std::unique_ptr<dog_diagnostics::Reporter> diag_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace dog_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<dog_hardware::ImuNode>();
    if (node->active()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("imu"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
