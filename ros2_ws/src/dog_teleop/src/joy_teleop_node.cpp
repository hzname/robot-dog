// joy_teleop: sensor_msgs/Joy -> cmd_vel / command / estop / body_pose.
// Button and axis indices come from parameters (see config/teleop.yaml for
// Xbox and PlayStation profiles). Movement requires holding the deadman
// button; releasing it or losing the joystick stream stops the robot.
#include <chrono>
#include <memory>

#include "dog_teleop/mapping.hpp"
#include "dog_teleop/teleop_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace dog_teleop
{

class JoyTeleopNode : public rclcpp::Node
{
public:
  JoyTeleopNode()
  : rclcpp::Node("joy_teleop"), publisher_(*this)
  {
    JoyProfile p;
    p.axis_vx = static_cast<int>(declare_parameter("axis_vx", p.axis_vx));
    p.axis_vy = static_cast<int>(declare_parameter("axis_vy", p.axis_vy));
    p.axis_wz = static_cast<int>(declare_parameter("axis_wz", p.axis_wz));
    p.axis_pitch = static_cast<int>(declare_parameter("axis_pitch", p.axis_pitch));
    p.axis_height = static_cast<int>(declare_parameter("axis_height", p.axis_height));
    p.button_deadman = static_cast<int>(declare_parameter("button_deadman", p.button_deadman));
    p.button_turbo = static_cast<int>(declare_parameter("button_turbo", p.button_turbo));
    p.button_stand = static_cast<int>(declare_parameter("button_stand", p.button_stand));
    p.button_lie = static_cast<int>(declare_parameter("button_lie", p.button_lie));
    p.button_estop = static_cast<int>(declare_parameter("button_estop", p.button_estop));
    p.button_release = static_cast<int>(declare_parameter("button_release", p.button_release));
    p.deadzone = declare_parameter("deadzone", p.deadzone);
    p.normal_scale = declare_parameter("normal_scale", p.normal_scale);
    p.height_step = declare_parameter("height_step", p.height_step);
    timeout_ = declare_parameter("joy_timeout", 0.5);
    mapper_ = std::make_unique<JoyMapper>(p, declareLimits(*this));

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        last_msg_ = std::chrono::steady_clock::now();
        stale_ = false;
        const auto out = mapper_->process(msg->axes, msg->buttons);
        if (out.estop) {
          RCLCPP_WARN(get_logger(), "gamepad: E-STOP %s", *out.estop ? "ENGAGED" : "released");
        }
        if (out.command) {
          RCLCPP_INFO(get_logger(), "gamepad: %s", out.command->c_str());
        }
        publisher_.publish(out);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        const double age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_msg_).count();
        if (!stale_ && age > timeout_) {
          stale_ = true;
          RCLCPP_WARN(get_logger(), "no joystick data for %.1fs - stopping", timeout_);
          publisher_.publish(mapper_->timeout());
        }
      });
    RCLCPP_INFO(get_logger(), "joy teleop ready: hold deadman button %d to move", p.button_deadman);
  }

private:
  TeleopPublisher publisher_;
  std::unique_ptr<JoyMapper> mapper_;
  double timeout_{0.5};
  bool stale_{true};
  std::chrono::steady_clock::time_point last_msg_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_teleop::JoyTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
