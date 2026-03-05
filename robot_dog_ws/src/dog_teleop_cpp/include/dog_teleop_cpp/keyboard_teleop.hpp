#ifndef DOG_TELEOP_CPP__KEYBOARD_TELEOP_HPP_
#define DOG_TELEOP_CPP__KEYBOARD_TELEOP_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <mutex>

namespace dog_teleop_cpp
{

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Keyboard teleoperation node (LifecycleNode)
 * 
 * Controls:
 * - WASD / Arrow keys: Movement
 * - Q/E: Rotation
 * - Space: Stop
 * - +/-: Height adjustment
 * - ESC: Emergency stop
 * - R: Reset emergency stop
 */
class KeyboardTeleop : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit KeyboardTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~KeyboardTeleop() override;

  // Lifecycle transitions
  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  void printInstructions();
  char getKey();
  void inputLoop();
  void publishLoop();
  void watchdogLoop();
  void publishHeight();
  void publishEmergencyStop(bool stop = true);
  void publishZeroVelocity();

  // Parameters
  double max_linear_speed_;
  double max_angular_speed_;
  double max_lateral_speed_;
  double publish_rate_;
  double watchdog_timeout_;

  // State
  geometry_msgs::msg::Twist current_twist_;
  std::mutex twist_mutex_;
  double target_height_;
  std::atomic<bool> emergency_stop_;
  std::atomic<bool> running_;
  std::atomic<bool> active_;
  rclcpp::Time last_key_time_;
  std::mutex time_mutex_;

  // Terminal settings
  struct termios original_termios_;
  bool terminal_configured_;

  // Publishers (lifecycle)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr cmd_height_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Thread
  std::thread input_thread_;
};

}  // namespace dog_teleop_cpp

#endif  // DOG_TELEOP_CPP__KEYBOARD_TELEOP_HPP_
