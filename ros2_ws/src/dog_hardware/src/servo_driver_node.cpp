// servo_driver: joint targets -> PCA9685 PWM.
//
// Subscribes (relative names, launched in the /dog namespace):
//   joint_commands sensor_msgs/JointState  target positions [rad]
//   estop          std_msgs/Bool           true = all outputs off until released
// Publishes:
//   joint_states   sensor_msgs/JointState  commanded (slew-limited) positions
//   servo_pulses   sensor_msgs/JointState  position = pulse written [us], 0 = off
//
// Calibration parameters (<joint>.offset_deg, .direction, .pulse_min_us, ...,
// rod linkage <joint>.servo_arm_mm / .joint_arm_mm / .rod_mm / .axis_distance_mm,
// coupling <joint>.coupled_to / .coupling) can be changed at runtime with
// `ros2 param set`; an enabled servo moves to the new mapping immediately.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
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
#include "std_srvs/srv/trigger.hpp"

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
    // Silence longer than the hold plus this grace relaxes the outputs and
    // latches: a robot whose controller died must not stand there with hot
    // servos until the battery dies (SAF-04).
    // The robot only starts with a hand on it, so the driver comes up disarmed
    // and waits for ~/arm (SAF-12). Mock and tests can ask for the old behaviour.
    const bool armed_on_start = declare_parameter("armed_on_start", false);
    relax_timeout_ = declare_parameter("command_relax_timeout", 2.0);
    if (!(relax_timeout_ > 0.0)) {
      throw std::runtime_error(
        "command_relax_timeout must be > 0 s, got " + std::to_string(relax_timeout_));
    }
    relax_on_exit_ = declare_parameter("relax_on_exit", true);
    // Test hook: the mock backend starts failing every write after this many
    // seconds, so a launch test can drive the whole fault path (SAF-01/03).
    mock_fail_after_ = declare_parameter("mock.fail_after", 0.0);

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
      c.linkage.servo_arm_mm = declareNumber(n + ".servo_arm_mm", 0.0);
      c.linkage.joint_arm_mm = declareNumber(n + ".joint_arm_mm", 0.0);
      c.linkage.rod_mm = declareNumber(n + ".rod_mm", 0.0);
      c.linkage.axis_distance_mm = declareNumber(n + ".axis_distance_mm", 0.0);
      c.coupling = declareNumber(n + ".coupling", 0.0);
      coupled_names_.push_back(declare_parameter(n + ".coupled_to", std::string("")));
      const auto err = c.validate();
      if (!err.empty()) {
        throw std::runtime_error("invalid calibration for " + n + ": " + err);
      }
      cals.push_back(c);
    }
    for (size_t i = 0; i < names_.size(); ++i) {
      cals[i].coupled_to = resolveJoint(coupled_names_[i], i);
      if (!cals[i].linkage.direct()) {
        RCLCPP_INFO(get_logger(), "%s: rod drive, servo arm %.1f mm, joint arm %.1f mm, rod %.1f mm, axes %.1f mm",
          names_[i].c_str(), cals[i].linkage.a(), cals[i].linkage.b(), cals[i].linkage.c(), cals[i].linkage.d());
      }
      if (cals[i].coupled_to >= 0) {
        RCLCPP_INFO(get_logger(), "%s: coupled to %s x %.2f", names_[i].c_str(),
          coupled_names_[i].c_str(), cals[i].coupling);
      }
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
    dp.armed_on_start = true;  // decided once below, after the driver exists
    driver_ = std::make_unique<ServoDriver>(bus, names_, cals, dp);
    if (!armed_on_start) {
      driver_->disarm();
      RCLCPP_INFO(get_logger(), "driver disarmed: release_fault then ~/arm or stand to move");
    }

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pulse_pub_ = create_publisher<sensor_msgs::msg::JointState>("servo_pulses", 10);
    // The driver's own emergency release must reach whoever drives the robot:
    // it publishes on the same `estop` topic it listens to (SAF-03).
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("estop", rclcpp::QoS(10).reliable());
    release_srv_ = create_service<std_srvs::srv::Trigger>("release_fault",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          if (!driver_->fault()) {
            res->success = true;
            res->message = "no fault latched";
            return;
          }
          driver_->clearFault();
          driver_->setEstop(false);
          announced_ = false;
          publishEstop(false);
          res->success = true;
          res->message = driver_->outputsConfirmedOff() ?
            "fault released, outputs confirmed off - stand to move" :
            "fault released, outputs NOT confirmed off - check the bus";
          RCLCPP_WARN(get_logger(), "fault released: %s", res->message.c_str());
        });
    arm_srv_ = create_service<std_srvs::srv::Trigger>("arm",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          if (driver_->fault()) {
            res->success = false;
            res->message = "fault latched: fix the cause and call ~/release_fault first";
            return;
          }
          if (driver_->estop()) {
            res->success = false;
            res->message = "e-stop engaged: release it before arming";
            return;
          }
          driver_->arm();
          res->success = driver_->armed();
          res->message = res->success ? "armed: the servos follow joint_commands" :
            "not armed";
          RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        });
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
        if (driver_->fault() && !msg->data) {
          return;  // a latched fault is cleared by the service, not by the topic
        }
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
  /// Index of a joint name for coupling; "" -> -1. Throws on unknown names.
  int resolveJoint(const std::string & name, size_t self) const
  {
    if (name.empty()) {return -1;}
    const auto it = std::find(names_.begin(), names_.end(), name);
    if (it == names_.end() || static_cast<size_t>(it - names_.begin()) == self) {
      throw std::runtime_error(names_[self] + ".coupled_to: unknown joint '" + name + "'");
    }
    return static_cast<int>(it - names_.begin());
  }

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
      ++timeouts_;
      RCLCPP_WARN(get_logger(), "no joint commands for %.2fs - holding position", cmd_timeout_);
    }
    armMockFault(now);
    relaxIfCommandStreamDied(now);
    driver_->update(now);
    logBusFaults();
    announceFault();
    if (driver_->clampedCount() > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%d joint(s) clamped by limits/servo range - check calibration", driver_->clampedCount());
    }
    state_msg_.header.stamp = get_clock()->now();
    state_msg_.position = driver_->positions();
    state_pub_->publish(state_msg_);
    state_msg_.position = driver_->pulses();
    pulse_pub_->publish(state_msg_);
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
      if (field == "coupled_to") {
        try {
          c.coupled_to = resolveJoint(p.as_string(), i);
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = e.what();
          return result;
        }
      } else try {
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
        else if (field == "servo_arm_mm") {c.linkage.servo_arm_mm = v;}
        else if (field == "joint_arm_mm") {c.linkage.joint_arm_mm = v;}
        else if (field == "rod_mm") {c.linkage.rod_mm = v;}
        else if (field == "axis_distance_mm") {c.linkage.axis_distance_mm = v;}
        else if (field == "coupling") {c.coupling = v;}
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

  /// Test hook: after `mock.fail_after` seconds every write on the mock bus
  /// fails, which is how a launch test reaches the fault path without hardware.
  void armMockFault(double now)
  {
    if (mock_fail_after_ <= 0.0 || mock_fault_armed_) {return;}
    if (mock_start_ < 0.0) {mock_start_ = now;}
    if (now - mock_start_ < mock_fail_after_) {return;}
    mock_fault_armed_ = true;
    if (auto mock = std::dynamic_pointer_cast<MockBus>(bus_)) {
      mock->failNext(1000000);
      RCLCPP_WARN(get_logger(), "mock bus: writes fail from now on (test hook)");
    }
  }

  /// The hold keeps the pose for `command_timeout`; past `command_relax_timeout`
  /// more, the pose is not worth holding any more and the driver latches.
  void relaxIfCommandStreamDied(double now)
  {
    if (driver_->fault() || driver_->estop() || !driver_->anyEnabled()) {return;}
    if (now - last_cmd_ <= cmd_timeout_ + relax_timeout_) {return;}
    char reason[128];
    std::snprintf(reason, sizeof(reason),
      "no joint commands for %.1f s (hold %.1f s + relax %.1f s)",
      now - last_cmd_, cmd_timeout_, relax_timeout_);
    driver_->latchFault(reason);
  }

  void publishEstop(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    estop_pub_->publish(msg);
  }

  /// A fault the driver latched is announced once, so the control node can drop
  /// to PASSIVE instead of walking a robot whose servos stopped listening.
  void announceFault()
  {
    if (!driver_->fault() || announced_) {return;}
    announced_ = true;
    publishEstop(true);
    RCLCPP_ERROR(get_logger(), "fault latched (%s) - estop published, release with ~/release_fault",
      driver_->faultReason().c_str());
  }

  /// A failed write is the one failure the servos cannot show themselves: the
  /// chip keeps the old pulse, the robot looks alive but is frozen. Every
  /// channel gets a line once a second, and the latch gets one summary with
  /// the counters DIA-03 asks for.
  void logBusFaults()
  {
    for (int ch = 0; ch < kPwmChannels; ++ch) {
      const int faults = driver_->channelFaults(ch);
      if (faults == logged_faults_[ch]) {continue;}
      logged_faults_[ch] = faults;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "write failed ch=%d (faults=%d, failed frames=%d)", ch, faults, driver_->failedFrames());
    }
    if (driver_->fault() && !fault_logged_) {
      fault_logged_ = true;
      RCLCPP_ERROR(get_logger(),
        "bus fault latched: %s; %d bus fault(s), %d clamped, %d command timeout(s)",
        driver_->faultReason().c_str(), driver_->busFaults(), driver_->clampedCount(), timeouts_);
    }
  }

  std::vector<std::string> names_;
  std::vector<std::string> coupled_names_;
  std::shared_ptr<ServoBus> bus_;
  std::unique_ptr<ServoDriver> driver_;
  double cmd_timeout_{0.5};
  double relax_timeout_{2.0};
  bool relax_on_exit_{true};
  double last_cmd_{0.0};
  bool timed_out_{false};
  int timeouts_{0};
  std::array<int, kPwmChannels> logged_faults_{};
  bool fault_logged_{false};
  bool announced_{false};
  double mock_fail_after_{0.0};
  double mock_start_{-1.0};
  bool mock_fault_armed_{false};
  sensor_msgs::msg::JointState state_msg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pulse_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
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
