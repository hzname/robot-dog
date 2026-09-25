// power_monitor: servo-rail current / voltage from an INA226 or INA219.
//
// The sensor is optional. On start the node probes the configured I2C
// addresses; if nothing answers it logs that power monitoring is off and exits
// cleanly, and the rest of the robot runs as before.
//
// Publishes (relative names, launched in the /dog namespace):
//   power    sensor_msgs/BatteryState  voltage [V], current [A] (negative =
//            drawn by the servos, as BatteryState specifies), filtered
//   estop    std_msgs/Bool             true on sustained overcurrent (stall)
//   command  std_msgs/String           "lie" on sustained undervoltage
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "dog_hardware/power_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace dog_hardware
{

namespace
{
double steadySeconds()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

/// Simulated sensor for PC runs and tests; values come from parameters.
class MockPowerSensor : public PowerSensor
{
public:
  explicit MockPowerSensor(rclcpp::Node & node) : node_(node) {}
  bool read(PowerReading & out) override
  {
    out.voltage = node_.get_parameter("mock.voltage").as_double();
    out.current = node_.get_parameter("mock.current").as_double();
    return true;
  }
  std::string describe() const override {return "mock sensor (parameters mock.voltage / mock.current)";}

private:
  rclcpp::Node & node_;
};
}  // namespace

class PowerMonitorNode : public rclcpp::Node
{
public:
  PowerMonitorNode()
  : rclcpp::Node("power_monitor")
  {
    const auto backend = declare_parameter("backend", std::string("auto"));
    const auto device = declare_parameter("i2c.device", std::string("/dev/i2c-0"));
    const auto addresses = declare_parameter("i2c.addresses", std::vector<int64_t>{0x41, 0x44, 0x45});
    const auto chip = declare_parameter("chip", std::string("auto"));
    const auto shunt = declare_parameter("shunt_ohm", 0.01);
    const auto rate = declare_parameter("rate", 20.0);
    declare_parameter("mock.voltage", 6.0);
    declare_parameter("mock.current", 1.0);
    PowerGuardParams gp;
    gp.overcurrent_a = declare_parameter("overcurrent_a", gp.overcurrent_a);
    gp.overcurrent_time = declare_parameter("overcurrent_time", gp.overcurrent_time);
    gp.undervoltage_v = declare_parameter("undervoltage_v", gp.undervoltage_v);
    gp.undervoltage_time = declare_parameter("undervoltage_time", gp.undervoltage_time);
    gp.filter_tau = declare_parameter("filter_tau", gp.filter_tau);
    overcurrent_action_ = declare_parameter("overcurrent_action", std::string("estop"));
    undervoltage_action_ = declare_parameter("undervoltage_action", std::string("lie"));
    guard_ = std::make_unique<PowerGuard>(gp);

    if (backend == "off") {
      RCLCPP_INFO(get_logger(), "power monitoring disabled (backend:=off)");
      return;
    }
    if (backend == "mock") {
      sensor_ = std::make_unique<MockPowerSensor>(*this);
    } else if (backend == "auto" || backend == "i2c") {
      std::vector<int> addrs(addresses.begin(), addresses.end());
      std::string probed;
      sensor_ = probePowerSensor(device, addrs, chip, shunt, probed);
      if (!sensor_) {
        RCLCPP_INFO(get_logger(), "no current sensor found (%s) - power monitoring off", probed.c_str());
        return;
      }
    } else {
      throw std::runtime_error("unknown backend '" + backend + "' (auto, mock or off)");
    }

    power_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("power", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("estop", rclcpp::QoS(10).reliable());
    command_pub_ = create_publisher<std_msgs::msg::String>("command", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "power monitoring on %s: stall > %.1f A for %.1f s -> %s, "
      "rail < %.1f V for %.1f s -> %s", sensor_->describe().c_str(), gp.overcurrent_a,
      gp.overcurrent_time, overcurrent_action_.c_str(), gp.undervoltage_v, gp.undervoltage_time,
      undervoltage_action_.c_str());
  }

  bool active() const {return sensor_ != nullptr;}

private:
  void act(const std::string & action)
  {
    if (action == "estop") {
      std_msgs::msg::Bool m;
      m.data = true;
      estop_pub_->publish(m);
    } else if (action == "lie") {
      std_msgs::msg::String m;
      m.data = "lie";
      command_pub_->publish(m);
    }
  }

  void tick()
  {
    PowerReading r;
    if (!sensor_->read(r)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "current sensor read failed");
      return;
    }
    const auto ev = guard_->update(r, steadySeconds());
    if (ev == PowerGuard::Event::OVERCURRENT) {
      RCLCPP_ERROR(get_logger(), "servo current %.2f A held too long (stalled leg?) -> %s",
        guard_->filteredCurrent(), overcurrent_action_.c_str());
      act(overcurrent_action_);
    } else if (ev == PowerGuard::Event::UNDERVOLTAGE) {
      RCLCPP_ERROR(get_logger(), "servo rail at %.2f V (supply sagging / battery low) -> %s",
        r.voltage, undervoltage_action_.c_str());
      act(undervoltage_action_);
    }
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.voltage = static_cast<float>(r.voltage);
    msg.current = static_cast<float>(-guard_->filteredCurrent());
    msg.percentage = std::nanf("");
    msg.charge = msg.capacity = msg.design_capacity = std::nanf("");
    msg.temperature = std::nanf("");
    msg.present = true;
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    power_pub_->publish(msg);
  }

  std::unique_ptr<PowerSensor> sensor_;
  std::unique_ptr<PowerGuard> guard_;
  std::string overcurrent_action_;
  std::string undervoltage_action_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr power_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<dog_hardware::PowerMonitorNode>();
    if (node->active()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("power_monitor"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
