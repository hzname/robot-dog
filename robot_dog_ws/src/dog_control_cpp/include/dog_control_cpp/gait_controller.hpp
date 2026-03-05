#ifndef DOG_CONTROL_CPP__GAIT_CONTROLLER_HPP_
#define DOG_CONTROL_CPP__GAIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <array>
#include <math>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"

namespace dog_control_cpp
{

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Gait controller implementing trot gait with Inverse Kinematics
 * 
 * Features:
 * - Trot gait: diagonal legs move together (LF+RR, RF+LR)
 * - 3-DOF Inverse Kinematics for each leg
 * - Balance compensation integration
 * - Lifecycle management for safe operation
 */
class GaitController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GaitController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GaitController() override;

  // Lifecycle callbacks
  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

protected:
  // Main control loop
  void gaitLoop();
  
  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void balanceAdjustmentCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  
  // Gait generation
  void calculateFootTrajectory(int leg_idx, double phase, 
                               double &x, double &y, double &z);
  void calculateAllFootPositions();
  
  // Apply balance corrections to foot positions
  void applyBalanceCorrections(double &x, double &y, double &z, int leg_idx);
  
  // Inverse kinematics
  void solveLegIK(int leg_idx, double foot_x, double foot_y, double foot_z,
                  double &hip_angle, double &thigh_angle, double &calf_angle);
  
  // Publishing
  void publishJointTrajectory();
  void publishFootPositions();
  void publishGaitState();

private:
  // Leg indices
  static constexpr int LF = 0;  // Left Front
  static constexpr int RF = 1;  // Right Front
  static constexpr int LR = 2;  // Left Rear
  static constexpr int RR = 3;  // Right Rear

  // Gait parameters
  std::string gait_type_;
  std::array<double, 4> gait_phase_offset_;
  double stance_height_;
  double step_height_;
  double step_length_;
  double gait_period_;
  
  // Robot dimensions (meters)
  double body_length_;
  double body_width_;
  double hip_length_;
  double thigh_length_;
  double calf_length_;
  
  // Leg hip positions relative to body center
  std::array<std::array<double, 3>, 4> leg_positions_;
  
  // Current state
  double current_phase_;
  geometry_msgs::msg::Twist velocity_cmd_;
  geometry_msgs::msg::Pose body_pose_;
  
  // Balance compensation
  geometry_msgs::msg::Vector3 balance_adjustment_;
  bool balance_enabled_;
  double balance_response_factor_;
  
  // Joint configuration
  std::array<std::string, 12> joint_names_;
  std::array<double, 12> joint_angles_;
  std::array<std::array<double, 3>, 4> foot_positions_;
  std::array<std::array<double, 3>, 4> foot_positions_compensated_;
  
  // Timing
  rclcpp::Time last_time_;
  double control_rate_;
  
  // Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>> joint_traj_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> foot_positions_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> gait_state_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr balance_adjustment_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr gait_timer_;
};

}  // namespace dog_control_cpp

#endif  // DOG_CONTROL_CPP__GAIT_CONTROLLER_HPP_
