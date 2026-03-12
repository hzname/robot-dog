#include "dog_control_cpp/balance_controller.hpp"

#include <rclcpp/qos.hpp>
#include <cmath>
#include <limits>

namespace dog_control_cpp
{

BalanceController::BalanceController(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("balance_controller", options),
      // Default PID gains for roll
      kp_roll_(0.15), ki_roll_(0.01), kd_roll_(0.05),
      // Default PID gains for pitch
      kp_pitch_(0.15), ki_pitch_(0.01), kd_pitch_(0.05),
      // Default PID gains for height
      kp_height_(1.0), ki_height_(0.1), kd_height_(0.2),
      // PID state initialization
      roll_error_integral_(0.0), roll_error_prev_(0.0),
      pitch_error_integral_(0.0), pitch_error_prev_(0.0),
      height_error_integral_(0.0), height_error_prev_(0.0),
      // Target values
      target_roll_(0.0), target_pitch_(0.0), target_height_(0.25),
      // Current state
      current_roll_(0.0), current_pitch_(0.0), current_yaw_(0.0), current_height_(0.25),
      // Output limits
      max_roll_correction_(0.08), max_pitch_correction_(0.08), max_height_correction_(0.05),
      // Integral limits (anti-windup)
      max_integral_roll_(0.5), max_integral_pitch_(0.5), max_integral_height_(0.2),
      // Rate limiting
      max_correction_rate_(0.5), last_correction_({0.0, 0.0, 0.0}),
      // Control rate
      control_rate_(100.0)
{
  // Declare all parameters
  declare_parameter("kp_roll", kp_roll_);
  declare_parameter("ki_roll", ki_roll_);
  declare_parameter("kd_roll", kd_roll_);
  declare_parameter("kp_pitch", kp_pitch_);
  declare_parameter("ki_pitch", ki_pitch_);
  declare_parameter("kd_pitch", kd_pitch_);
  declare_parameter("kp_height", kp_height_);
  declare_parameter("ki_height", ki_height_);
  declare_parameter("kd_height", kd_height_);
  declare_parameter("target_height", target_height_);
  declare_parameter("max_roll_correction", max_roll_correction_);
  declare_parameter("max_pitch_correction", max_pitch_correction_);
  declare_parameter("max_height_correction", max_height_correction_);
  declare_parameter("max_integral_roll", max_integral_roll_);
  declare_parameter("max_integral_pitch", max_integral_pitch_);
  declare_parameter("max_integral_height", max_integral_height_);
  declare_parameter("max_correction_rate", max_correction_rate_);
  declare_parameter("control_rate", control_rate_);
}

BalanceController::~BalanceController()
{
  if (control_timer_)
  {
    control_timer_->cancel();
  }
}

LifecycleCallbackReturn BalanceController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Balance Controller");

  // Load all parameters
  kp_roll_ = get_parameter("kp_roll").as_double();
  ki_roll_ = get_parameter("ki_roll").as_double();
  kd_roll_ = get_parameter("kd_roll").as_double();
  kp_pitch_ = get_parameter("kp_pitch").as_double();
  ki_pitch_ = get_parameter("ki_pitch").as_double();
  kd_pitch_ = get_parameter("kd_pitch").as_double();
  kp_height_ = get_parameter("kp_height").as_double();
  ki_height_ = get_parameter("ki_height").as_double();
  kd_height_ = get_parameter("kd_height").as_double();
  target_height_ = get_parameter("target_height").as_double();
  max_roll_correction_ = get_parameter("max_roll_correction").as_double();
  max_pitch_correction_ = get_parameter("max_pitch_correction").as_double();
  max_height_correction_ = get_parameter("max_height_correction").as_double();
  max_integral_roll_ = get_parameter("max_integral_roll").as_double();
  max_integral_pitch_ = get_parameter("max_integral_pitch").as_double();
  max_integral_height_ = get_parameter("max_integral_height").as_double();
  max_correction_rate_ = get_parameter("max_correction_rate").as_double();
  control_rate_ = get_parameter("control_rate").as_double();

  RCLCPP_INFO(get_logger(), "PID Roll:  Kp=%.3f Ki=%.3f Kd=%.3f", kp_roll_, ki_roll_, kd_roll_);
  RCLCPP_INFO(get_logger(), "PID Pitch: Kp=%.3f Ki=%.3f Kd=%.3f", kp_pitch_, ki_pitch_, kd_pitch_);
  RCLCPP_INFO(get_logger(), "Max correction: roll=%.3f pitch=%.3f height=%.3f",
              max_roll_correction_, max_pitch_correction_, max_height_correction_);

  // QoS profiles
  rclcpp::QoS sensor_qos(10);
  sensor_qos.best_effort();

  rclcpp::QoS control_qos(5);
  control_qos.reliable();

  // Publishers
  balance_adjustment_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      "/balance_adjustment", control_qos);
  balance_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/balance_state", 10);

  // Subscribers
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

  height_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/cmd_height", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { heightCmdCallback(msg); });

  pose_cmd_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/cmd_body_pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) { poseCmdCallback(msg); });

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn BalanceController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Balance Controller");

  balance_adjustment_pub_->on_activate();
  balance_state_pub_->on_activate();

  last_time_ = get_clock()->now();
  last_correction_ = {0.0, 0.0, 0.0};

  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { controlLoop(); });

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn BalanceController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Balance Controller");

  if (control_timer_)
  {
    control_timer_->cancel();
  }

  balance_adjustment_pub_->on_deactivate();
  balance_state_pub_->on_deactivate();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn BalanceController::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Balance Controller");

  balance_adjustment_pub_.reset();
  balance_state_pub_.reset();
  imu_sub_.reset();
  height_cmd_sub_.reset();
  pose_cmd_sub_.reset();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn BalanceController::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Balance Controller");
  if (control_timer_)
  {
    control_timer_->cancel();
  }
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn BalanceController::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_ERROR(get_logger(), "Balance Controller error state");
  if (control_timer_)
  {
    control_timer_->cancel();
  }
  return LifecycleCallbackReturn::SUCCESS;
}

void BalanceController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  quaternionToEuler(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z,
                    current_roll_, current_pitch_, current_yaw_);
}

void BalanceController::heightCmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_height_ = std::clamp(msg->data, 0.15, 0.35);
  RCLCPP_DEBUG(get_logger(), "Target height set to: %.3f", target_height_);
}

void BalanceController::poseCmdCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  double roll, pitch, yaw;
  quaternionToEuler(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z,
                    roll, pitch, yaw);
  target_roll_ = std::clamp(roll, -0.3, 0.3);
  target_pitch_ = std::clamp(pitch, -0.3, 0.3);
  RCLCPP_DEBUG(get_logger(), "Target pose: roll=%.3f pitch=%.3f", target_roll_, target_pitch_);
}

double BalanceController::computePID(double error, double &integral, double &prev_error,
                                      double dt, double kp, double ki, double kd)
{
  if (dt <= std::numeric_limits<double>::epsilon()) {
    return kp * error; // Только пропорциональная часть
  }
  
  // Update integral with anti-windup
  integral += error * dt;
  integral = std::clamp(integral, -max_integral_roll_, max_integral_roll_);
  
  // Compute derivative
  double derivative = (error - prev_error) / dt;
  prev_error = error;
  
  // PID output
  return kp * error + ki * integral + kd * derivative;
}

void BalanceController::controlLoop()
{
  auto current_time = get_clock()->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // Protect against invalid dt
  if (dt <= 0 || dt > 0.1)
  {
    dt = 1.0 / control_rate_;
  }

  // Compute errors
  double roll_error = target_roll_ - current_roll_;
  double pitch_error = target_pitch_ - current_pitch_;
  double height_error = target_height_ - current_height_;

  // Clamp integrals (anti-windup)
  roll_error_integral_ = std::clamp(roll_error_integral_, -max_integral_roll_, max_integral_roll_);
  pitch_error_integral_ = std::clamp(pitch_error_integral_, -max_integral_pitch_, max_integral_pitch_);
  height_error_integral_ = std::clamp(height_error_integral_, -max_integral_height_, max_integral_height_);

  // Compute PID outputs
  double roll_correction = computePID(roll_error, roll_error_integral_, roll_error_prev_,
                                       dt, kp_roll_, ki_roll_, kd_roll_);
  double pitch_correction = computePID(pitch_error, pitch_error_integral_, pitch_error_prev_,
                                        dt, kp_pitch_, ki_pitch_, kd_pitch_);
  double height_correction = computePID(height_error, height_error_integral_, height_error_prev_,
                                         dt, kp_height_, ki_height_, kd_height_);

  // Clamp corrections
  roll_correction = std::clamp(roll_correction, -max_roll_correction_, max_roll_correction_);
  pitch_correction = std::clamp(pitch_correction, -max_pitch_correction_, max_pitch_correction_);
  height_correction = std::clamp(height_correction, -max_height_correction_, max_height_correction_);

  // Apply rate limiting
  std::array<double, 3> correction = {roll_correction, pitch_correction, height_correction};
  double max_delta = max_correction_rate_ * dt;
  
  for (size_t i = 0; i < 3; ++i)
  {
    double delta = correction[i] - last_correction_[i];
    delta = std::clamp(delta, -max_delta, max_delta);
    last_correction_[i] += delta;
  }

  // Publish balance adjustment
  publishBalanceAdjustment(last_correction_[0], last_correction_[1], last_correction_[2]);

  // Publish debug state
  std_msgs::msg::Float64MultiArray state_msg;
  state_msg.data = {
      current_roll_, current_pitch_,      // [0, 1] current orientation
      roll_error, pitch_error,            // [2, 3] errors
      last_correction_[0], last_correction_[1], last_correction_[2],  // [4, 5, 6] corrections
      target_roll_, target_pitch_, target_height_  // [7, 8, 9] targets
  };
  balance_state_pub_->publish(state_msg);
}

void BalanceController::publishBalanceAdjustment(double roll_corr, double pitch_corr, double height_adj)
{
  geometry_msgs::msg::Vector3 adjustment;
  adjustment.x = roll_corr;    // Roll correction (rad)
  adjustment.y = pitch_corr;   // Pitch correction (rad)
  adjustment.z = height_adj;   // Height adjustment (m)
  
  balance_adjustment_pub_->publish(adjustment);
}

void BalanceController::quaternionToEuler(double w, double x, double y, double z,
                                          double &roll, double &pitch, double &yaw)
{
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0)
    pitch = std::copysign(M_PI / 2.0, sinp);  // Use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

} // namespace dog_control_cpp

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_control_cpp::BalanceController>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
