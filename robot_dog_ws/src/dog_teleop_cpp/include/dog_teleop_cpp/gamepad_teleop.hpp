#ifndef DOG_TELEOP_CPP__GAMEPAD_TELEOP_HPP_
#define DOG_TELEOP_CPP__GAMEPAD_TELEOP_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <mutex>
#include <vector>

namespace dog_teleop_cpp
{

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Gamepad/Joystick teleoperation node (LifecycleNode)
 * 
 * Xbox controller mappings:
 * - Left stick: Forward/backward, Left/right strafe
 * - Right stick X: Rotation
 * - LT/RT: Height adjustment
 * - A: Stand up, B: Sit down
 * - X: Emergency stop, Y: Reset emergency stop
 * - Start: Enable servos, Back: Disable servos
 */
class GamepadTeleop : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GamepadTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GamepadTeleop() override;

  // Lifecycle transitions
  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  void handleJoy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void handleButtons(const std::vector<int32_t> & buttons);
  void handleAxes(const std::vector<float> & axes);
  double applyDeadzone(double value);
  double lerp(double current, double target, double factor);
  void publishLoop();
  void watchdogLoop();
  void publishHeight();
  void publishEmergencyStop(bool stop = true);
  void publishServoEnable(bool enable);
  void publishZeroVelocity();

  // Axis mappings (Xbox controller)
  static constexpr int AXIS_LEFT_X = 0;
  static constexpr int AXIS_LEFT_Y = 1;
  static constexpr int AXIS_LT = 2;
  static constexpr int AXIS_RIGHT_X = 3;
  static constexpr int AXIS_RIGHT_Y = 4;
  static constexpr int AXIS_RT = 5;

  // Button mappings
  static constexpr int BTN_A = 0;
  static constexpr int BTN_B = 1;
  static constexpr int BTN_X = 2;
  static constexpr int BTN_Y = 3;
  static constexpr int BTN_LB = 4;
  static constexpr int BTN_RB = 5;
  static constexpr int BTN_BACK = 6;
  static constexpr int BTN_START = 7;

  // Parameters
  double max_linear_speed_;
  double max_angular_speed_;
  double max_lateral_speed_;
  double axis_deadzone_;
  double publish_rate_;
  double watchdog_timeout_;
  double twist_smoothing_;

  // State
  geometry_msgs::msg::Twist current_twist_;
  geometry_msgs::msg::Twist smoothed_twist_;
  std::mutex twist_mutex_;
  double target_height_;
  double gait_speed_;
  bool emergency_stop_;
  bool servos_enabled_;
  std::atomic<bool> active_;
  rclcpp::Time last_joy_time_;
  std::mutex time_mutex_;

  // Button state
  std::vector<int32_t> prev_buttons_;

  // Publishers (lifecycle)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr cmd_height_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr servo_enable_pub_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace dog_teleop_cpp

#endif  // DOG_TELEOP_CPP__GAMEPAD_TELEOP_HPP_
