/**
 * @file servo_driver_node.hpp
 * @brief Lifecycle node for servo driver with safety features
 * 
 * Safety features:
 * - Position clamps per joint
 * - Rate limiting
 * - Command timeout detection (watchdog)
 * - Emergency stop handling
 * - Smooth trajectory following
 */

#ifndef DOG_HARDWARE_CPP__SERVO_DRIVER_NODE_HPP_
#define DOG_HARDWARE_CPP__SERVO_DRIVER_NODE_HPP_

#include <memory>
#include <array>
#include <string>
#include <math>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "dog_hardware_cpp/servo_controller.hpp"

namespace dog_hardware_cpp
{

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ServoDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ServoDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ServoDriverNode() override;

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

protected:
  void controlLoop();
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void positionCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  
  void publishJointStates();
  void loadParameters();
  void loadJointLimitsFromParams();

private:
  // Configuration
  ControllerConfig controller_config_;
  double publish_rate_hz_;
  bool safety_enabled_;
  
  // Joint configuration
  static constexpr int NUM_JOINTS = 12;
  std::array<std::string, NUM_JOINTS> joint_names_;
  
  // Controller
  std::unique_ptr<ServoController> controller_;
  
  // Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>> joint_state_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> servo_feedback_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> emergency_stop_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> watchdog_pub_;
  
  // Subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // State
  bool servos_enabled_ = false;
};

} // namespace dog_hardware_cpp

#endif // DOG_HARDWARE_CPP__SERVO_DRIVER_NODE_HPP_
