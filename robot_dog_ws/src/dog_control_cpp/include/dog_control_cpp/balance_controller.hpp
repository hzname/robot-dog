#ifndef DOG_CONTROL_CPP__BALANCE_CONTROLLER_HPP_
#define DOG_CONTROL_CPP__BALANCE_CONTROLLER_HPP_

#include <memory>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace dog_control_cpp
{

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Balance controller using PID for body stabilization
 * 
 * Controls:
 * - Roll/Pitch stabilization based on IMU
 * - Publishes balance_adjustment for gait controller
 * 
 * Output /balance_adjustment:
 * - x: roll correction (rad)
 * - y: pitch correction (rad)  
 * - z: height adjustment (m)
 */
class BalanceController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BalanceController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~BalanceController() override;

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

protected:
  void controlLoop();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void heightCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void poseCmdCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  
  void publishBalanceAdjustment(double roll_corr, double pitch_corr, double height_adj);
  
  // Utility
  void quaternionToEuler(double w, double x, double y, double z,
                         double &roll, double &pitch, double &yaw);
  
  // PID computation
  double computePID(double error, double &integral, double &prev_error, 
                    double dt, double kp, double ki, double kd);

private:
  // PID gains for roll
  double kp_roll_, ki_roll_, kd_roll_;
  // PID gains for pitch
  double kp_pitch_, ki_pitch_, kd_pitch_;
  // PID gains for height
  double kp_height_, ki_height_, kd_height_;
  
  // PID state for roll
  double roll_error_integral_, roll_error_prev_;
  // PID state for pitch
  double pitch_error_integral_, pitch_error_prev_;
  // PID state for height
  double height_error_integral_, height_error_prev_;
  
  // Target values
  double target_roll_, target_pitch_, target_height_;
  
  // Current IMU state
  double current_roll_, current_pitch_, current_yaw_;
  double current_height_;
  
  // Output limits
  double max_roll_correction_;
  double max_pitch_correction_;
  double max_height_correction_;
  
  // Anti-windup limits
  double max_integral_roll_;
  double max_integral_pitch_;
  double max_integral_height_;
  
  // Rate limiting
  double max_correction_rate_;
  std::array<double, 3> last_correction_;
  
  // Control rate
  double control_rate_;
  rclcpp::Time last_time_;
  
  // Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>> balance_adjustment_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> balance_state_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_cmd_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace dog_control_cpp

#endif // DOG_CONTROL_CPP__BALANCE_CONTROLLER_HPP_
