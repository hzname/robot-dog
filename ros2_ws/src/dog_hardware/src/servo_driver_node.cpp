// servo_driver: joint targets -> PCA9685 PWM.
//
// Subscribes (relative names, launched in the /dog namespace):
//   joint_commands sensor_msgs/JointState  target positions [rad]
//   estop          std_msgs/Bool           true = all outputs off until released
// Publishes:
//   joint_states   sensor_msgs/JointState  commanded (slew-limited) positions
//
// Calibration parameters (<joint>.offset_deg, .direction, .pulse_min_us, ...)
// can be changed at runtime with `ros2 param set`; an enabled servo moves to
// the new mapping immediately so offsets can be tuned by eye.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dog_hardware/servo_bus.hpp"
#include "dog_hardware/servo_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace dog_hardware
{

namespace
{
const std::vector<std::string> kDefaultJoints = {
  "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
  "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
  "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
  "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
};

// Defaults for joint i in the order above; real values live in servos.yaml.
ServoCalibration defaultCalibration(size_t i)
{
  ServoCalibration c;
  c.channel = static_cast<int>(i);
  const bool right = (i / 3) == 1 || (i / 3) == 3;
  c.direction = right ? -1 : 1;
  switch (i % 3) {
    case 0: c.offset_deg = 0.0; c.min_deg = -40.0; c.max_deg = 40.0; break;
    case 1: c.offset_deg = 45.0; c.min_deg = -45.0; c.max_deg = 135.0; break;
    default: c.offset_deg = -90.0; c.min_deg = -165.0; c.max_deg = -15.0; break;
  }
  return c;
}

// Numbers from YAML or `ros2 param set` may arrive as integer or double.
double asNumber(const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  return p.as_double();  // throws InvalidParameterTypeException otherwise
}

double steadySeconds()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class ServoDriverNode : public rclcpp::Node
{
public:
  explicit ServoDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("servo_driver", options)
  {
    const auto backend = declare_parameter("backend", std::string("mock"));
    const auto device = declare_parameter("i2c.device", std::string("/dev/i2c-0"));
    const auto address = declare_parameter("i2c.address", 0x40);
    const auto pwm_hz = declare_parameter("pwm.frequency", 50.0);
    const auto osc_hz = declare_parameter("pwm.oscillator_hz", 25e6);
    const auto rate = declare_parameter("update_rate", 100.0);
    cmd_timeout_ = declare_parameter("command_timeout", 0.5);
    relax_on_exit_ = declare_parameter("relax_on_exit", true);

    DriverParams dp;
    dp.max_joint_speed = declare_parameter("max_joint_speed", dp.max_joint_speed);
    dp.enable_stagger = declare_parameter("enable_stagger", dp.enable_stagger);

    names_ = declare_parameter("joint_names", kDefaultJoints);
    std::vector<ServoCalibration> cals;
    for (size_t i = 0; i < names_.size(); ++i) {
      const auto d = defaultCalibration(i);
      const auto & n = names_[i];
      ServoCalibration c;
      c.channel = static_cast<int>(declareNumber(n + ".channel", d.channel));
      c.direction = static_cast<int>(declareNumber(n + ".direction", d.direction));
      c.offset_deg = declareNumber(n + ".offset_deg", d.offset_deg);
      c.pulse_min_us = declareNumber(n + ".pulse_min_us", d.pulse_min_us);
      c.pulse_max_us = declareNumber(n + ".pulse_max_us", d.pulse_max_us);
      c.range_deg = declareNumber(n + ".range_deg", d.range_deg);
      c.min_deg = declareNumber(n + ".min_deg", d.min_deg);
      c.max_deg = declareNumber(n + ".max_deg", d.max_deg);
      const auto err = c.validate();
      if (!err.empty()) {
        throw std::runtime_error("invalid calibration for " + n + ": " + err);
      }
      cals.push_back(c);
    }

    std::shared_ptr<ServoBus> bus;
    if (backend == "pca9685") {
      auto pca = std::make_shared<Pca9685Bus>();
      std::string error;
      if (!pca->open(device, static_cast<int>(address), pwm_hz, osc_hz, error)) {
        throw std::runtime_error(error);
      }
      bus = pca;
    } else if (backend == "mock") {
      bus = std::make_shared<MockBus>();
    } else {
      throw std::runtime_error("unknown backend '" + backend + "' (use pca9685 or mock)");
    }
    bus_ = bus;
    driver_ = std::make_unique<ServoDriver>(bus, names_, cals, dp);

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_commands", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        if (driver_->setTargets(msg->name, msg->position, steadySeconds()) > 0) {
          last_cmd_ = steadySeconds();
          if (timed_out_) {
            RCLCPP_INFO(get_logger(), "joint commands resumed");
            timed_out_ = false;
          }
        }
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "estop", rclcpp::QoS(10).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != driver_->estop()) {
          RCLCPP_WARN(get_logger(), "E-STOP %s", msg->data ? "ENGAGED - servos off" : "released");
        }
        driver_->setEstop(msg->data);
      });
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParams(params);});

    state_msg_.name = names_;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "servo driver on %s, %zu joints", bus->describe().c_str(), names_.size());
  }

  ~ServoDriverNode() override
  {
    if (relax_on_exit_ && driver_) {
      driver_->relax();
    }
  }

private:
  double declareNumber(const std::string & name, double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const auto v = declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
    return asNumber(rclcpp::Parameter(name, v));
  }

  void tick()
  {
    const double now = steadySeconds();
    if (!timed_out_ && driver_->anyEnabled() && now - last_cmd_ > cmd_timeout_) {
      timed_out_ = true;
      RCLCPP_WARN(get_logger(), "no joint commands for %.2fs - holding position", cmd_timeout_);
    }
    driver_->update(now);
    if (driver_->clampedCount() > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%d joint(s) clamped by limits/servo range - check calibration", driver_->clampedCount());
    }
    state_msg_.header.stamp = get_clock()->now();
    state_msg_.position = driver_->positions();
    state_pub_->publish(state_msg_);
  }

  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const auto & name = p.get_name();
      const auto dot = name.rfind('.');
      if (dot == std::string::npos) {continue;}
      const auto joint = name.substr(0, dot);
      const auto field = name.substr(dot + 1);
      const auto it = std::find(names_.begin(), names_.end(), joint);
      if (it == names_.end()) {continue;}
      const size_t i = static_cast<size_t>(it - names_.begin());
      ServoCalibration c = driver_->calibrations()[i];
      try {
        const double v = asNumber(p);
        if (field == "channel" || field == "direction") {
          if (v != std::floor(v)) {
            result.successful = false;
            result.reason = name + " must be a whole number";
            return result;
          }
        }
        if (field == "channel") {c.channel = static_cast<int>(v);}
        else if (field == "direction") {c.direction = static_cast<int>(v);}
        else if (field == "offset_deg") {c.offset_deg = v;}
        else if (field == "pulse_min_us") {c.pulse_min_us = v;}
        else if (field == "pulse_max_us") {c.pulse_max_us = v;}
        else if (field == "range_deg") {c.range_deg = v;}
        else if (field == "min_deg") {c.min_deg = v;}
        else if (field == "max_deg") {c.max_deg = v;}
        else {continue;}
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        result.successful = false;
        result.reason = name + ": " + e.what();
        return result;
      }
      const auto err = c.validate();
      if (!err.empty() || !driver_->setCalibration(i, c)) {
        result.successful = false;
        result.reason = name + ": " + err;
        return result;
      }
      RCLCPP_INFO(get_logger(), "calibration %s updated", name.c_str());
    }
    return result;
  }

  std::vector<std::string> names_;
  std::shared_ptr<ServoBus> bus_;
  std::unique_ptr<ServoDriver> driver_;
  double cmd_timeout_{0.5};
  bool relax_on_exit_{true};
  double last_cmd_{0.0};
  bool timed_out_{false};
  sensor_msgs::msg::JointState state_msg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<dog_hardware::ServoDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_driver"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
