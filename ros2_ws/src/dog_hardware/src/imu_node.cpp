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
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "dog_hardware/imu_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "IMU: %s; measuring gyro bias, keep the robot still",
      sensor_->describe().c_str());
  }

  bool active() const {return sensor_ != nullptr;}

private:
  void tick()
  {
    ImuReading raw;
    if (!sensor_->read(raw)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU read failed");
      return;
    }
    ImuReading b{axes_.apply(raw.accel), axes_.apply(raw.gyro)};
    const auto t = std::chrono::steady_clock::now();
    if (!bias_.done()) {
      if (bias_.add(b.gyro)) {
        const auto g = bias_.bias();
        RCLCPP_INFO(get_logger(), "gyro bias %.4f %.4f %.4f rad/s", g[0], g[1], g[2]);
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
