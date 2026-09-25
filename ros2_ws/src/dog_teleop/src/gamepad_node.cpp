// gamepad_node: reads a USB / Bluetooth gamepad through the Linux joystick
// API (/dev/input/jsN) and publishes sensor_msgs/Joy on `joy`.
//
// Axes follow the ROS `joy` package convention (left / up = +1, triggers rest
// at +1), so joy_teleop works the same with this node or with joy_node.
// The device is reopened automatically after a disconnect (e.g. Bluetooth).
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace dog_teleop
{

class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode()
  : rclcpp::Node("gamepad")
  {
    device_ = declare_parameter("device", std::string("/dev/input/js0"));
    const double rate = declare_parameter("publish_rate", 20.0);
    pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {poll();});
    pub_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {if (fd_ >= 0) {publish();}});
    reopen();
  }

  ~GamepadNode() override {closeDevice();}

private:
  void reopen()
  {
    fd_ = ::open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "waiting for gamepad %s (%s)", device_.c_str(), std::strerror(errno));
      return;
    }
    uint8_t n_axes = 0, n_buttons = 0;
    char name[128] = "unknown";
    ::ioctl(fd_, JSIOCGAXES, &n_axes);
    ::ioctl(fd_, JSIOCGBUTTONS, &n_buttons);
    ::ioctl(fd_, JSIOCGNAME(sizeof(name)), name);
    msg_.axes.assign(n_axes, 0.0f);
    msg_.buttons.assign(n_buttons, 0);
    RCLCPP_INFO(get_logger(), "gamepad connected: %s (%u axes, %u buttons) on %s",
      name, n_axes, n_buttons, device_.c_str());
  }

  void closeDevice()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void poll()
  {
    if (fd_ < 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_attempt_ > std::chrono::seconds(1)) {
        last_attempt_ = now;
        reopen();
      }
      return;
    }
    bool changed = false;
    js_event e{};
    ssize_t n = 0;
    while ((n = ::read(fd_, &e, sizeof(e))) == static_cast<ssize_t>(sizeof(e))) {
      const uint8_t type = e.type & ~JS_EVENT_INIT;
      // Grow on demand in case the ioctl sizes were unavailable.
      if (type == JS_EVENT_AXIS && e.number >= msg_.axes.size()) {
        msg_.axes.resize(e.number + 1u, 0.0f);
      } else if (type == JS_EVENT_BUTTON && e.number >= msg_.buttons.size()) {
        msg_.buttons.resize(e.number + 1u, 0);
      }
      if (type == JS_EVENT_AXIS) {
        // Linux: left / up are negative. ROS joy: left / up are positive.
        msg_.axes[e.number] = std::clamp(-static_cast<float>(e.value) / 32767.0f, -1.0f, 1.0f);
        changed = true;
      } else if (type == JS_EVENT_BUTTON) {
        const int v = e.value ? 1 : 0;
        if (msg_.buttons[e.number] != v) {
          msg_.buttons[e.number] = v;
          // Publish every button change: a press + release arriving in one
          // batch (Bluetooth) must not collapse into "nothing happened".
          publish();
          changed = false;
        }
      }
    }
    if (n < 0 && errno != EAGAIN) {
      RCLCPP_WARN(get_logger(), "gamepad disconnected (%s)", std::strerror(errno));
      closeDevice();
      return;
    }
    if (changed) {
      publish();
    }
  }

  void publish()
  {
    msg_.header.stamp = now();
    pub_->publish(msg_);
  }

  std::string device_;
  int fd_{-1};
  std::chrono::steady_clock::time_point last_attempt_{};
  sensor_msgs::msg::Joy msg_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

}  // namespace dog_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_teleop::GamepadNode>());
  rclcpp::shutdown();
  return 0;
}
