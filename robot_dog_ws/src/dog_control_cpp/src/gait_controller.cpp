#include "dog_control_cpp/gait_controller.hpp"

#include <rclcpp/qos.hpp>
#include <math>

namespace dog_control_cpp
{

GaitController::GaitController(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gait_controller", options),
  gait_type_("trot"),
  gait_phase_offset_({0.0, 0.5, 0.5, 0.0}),
  stance_height_(0.25),
  step_height_(0.08),
  step_length_(0.0),
  gait_period_(0.5),
  body_length_(0.35),
  body_width_(0.20),
  hip_length_(0.045),
  thigh_length_(0.11),
  shin_length_(0.11),
  current_phase_(0.0),
  control_rate_(100.0),
  balance_enabled_(true),
  balance_response_factor_(1.0),
  joint_names_({
    "lf_hip_joint", "lf_thigh_joint", "lf_shin_joint",
    "rf_hip_joint", "rf_thigh_joint", "rf_shin_joint",
    "lr_hip_joint", "lr_thigh_joint", "lr_shin_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_shin_joint",
  }),
  joint_angles_({0.0, 0.0, -1.2, 0.0, 0.0, -1.2, 0.0, 0.0, -1.2, 0.0, 0.0, -1.2})
{
  // Declare parameters
  declare_parameter("gait_type", gait_type_);
  declare_parameter("stance_height", stance_height_);
  declare_parameter("step_height", step_height_);
  declare_parameter("gait_period", gait_period_);
  declare_parameter("body_length", body_length_);
  declare_parameter("body_width", body_width_);
  declare_parameter("thigh_length", thigh_length_);
  declare_parameter("shin_length", shin_length_);
  declare_parameter("control_rate", control_rate_);
  declare_parameter("balance_enabled", balance_enabled_);
  declare_parameter("balance_response_factor", balance_response_factor_);
  
  // Initialize leg positions
  leg_positions_ = {{
    {{ body_length_ / 2.0,  body_width_ / 2.0, 0.0}},   // LF
    {{ body_length_ / 2.0, -body_width_ / 2.0, 0.0}},   // RF
    {{-body_length_ / 2.0,  body_width_ / 2.0, 0.0}},   // LR
    {{-body_length_ / 2.0, -body_width_ / 2.0, 0.0}}    // RR
  }};
  
  // Initialize balance adjustment
  balance_adjustment_.x = 0.0;
  balance_adjustment_.y = 0.0;
  balance_adjustment_.z = 0.0;
}

GaitController::~GaitController()
{
  if (gait_timer_) {
    gait_timer_->cancel();
  }
}

LifecycleCallbackReturn GaitController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Gait Controller");
  
  // Get parameters
  gait_type_ = get_parameter("gait_type").as_string();
  stance_height_ = get_parameter("stance_height").as_double();
  step_height_ = get_parameter("step_height").as_double();
  gait_period_ = get_parameter("gait_period").as_double();
  body_length_ = get_parameter("body_length").as_double();
  body_width_ = get_parameter("body_width").as_double();
  thigh_length_ = get_parameter("thigh_length").as_double();
  shin_length_ = get_parameter("shin_length").as_double();
  control_rate_ = get_parameter("control_rate").as_double();
  balance_enabled_ = get_parameter("balance_enabled").as_bool();
  balance_response_factor_ = get_parameter("balance_response_factor").as_double();
  
  RCLCPP_INFO(get_logger(), "Gait type: %s, Balance: %s", 
              gait_type_.c_str(), balance_enabled_ ? "enabled" : "disabled");
  
  // Update leg positions with new dimensions
  leg_positions_ = {{
    {{ body_length_ / 2.0,  body_width_ / 2.0, 0.0}},   // LF
    {{ body_length_ / 2.0, -body_width_ / 2.0, 0.0}},   // RF
    {{-body_length_ / 2.0,  body_width_ / 2.0, 0.0}},   // LR
    {{-body_length_ / 2.0, -body_width_ / 2.0, 0.0}}    // RR
  }};
  
  // QoS profiles
  rclcpp::QoS traj_qos(10);
  traj_qos.reliable();
  
  rclcpp::QoS cmd_qos(1);
  cmd_qos.best_effort();
  cmd_qos.durability_volatile();
  
  rclcpp::QoS balance_qos(5);
  balance_qos.reliable();
  
  // Create publishers
  joint_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory", traj_qos);
  foot_positions_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/foot_positions", 10);
  gait_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/gait_state", 10);
  
  // Create subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", cmd_qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmdVelCallback(msg);
    });
    
  body_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
    "/body_pose", 10,
    [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
      bodyPoseCallback(msg);
    });
  
  // Subscribe to balance adjustments
  balance_adjustment_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/balance_adjustment", balance_qos,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
      balanceAdjustmentCallback(msg);
    });
  
  RCLCPP_INFO(get_logger(), "Gait Controller configured (%s gait)", gait_type_.c_str());
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GaitController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Gait Controller");
  
  joint_traj_pub_->on_activate();
  foot_positions_pub_->on_activate();
  gait_state_pub_->on_activate();
  
  last_time_ = get_clock()->now();
  
  // Start control loop
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  gait_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { gaitLoop(); });
  
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GaitController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Gait Controller");
  
  if (gait_timer_) {
    gait_timer_->cancel();
  }
  
  joint_traj_pub_->on_deactivate();
  foot_positions_pub_->on_deactivate();
  gait_state_pub_->on_deactivate();
  
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GaitController::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Gait Controller");
  
  joint_traj_pub_.reset();
  foot_positions_pub_.reset();
  gait_state_pub_.reset();
  cmd_vel_sub_.reset();
  body_pose_sub_.reset();
  balance_adjustment_sub_.reset();
  
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GaitController::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Gait Controller");
  
  if (gait_timer_) {
    gait_timer_->cancel();
  }
  
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GaitController::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_ERROR(get_logger(), "Gait Controller error state");
  
  if (gait_timer_) {
    gait_timer_->cancel();
  }
  
  return LifecycleCallbackReturn::SUCCESS;
}

void GaitController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  velocity_cmd_ = *msg;
  step_length_ = msg->linear.x * gait_period_ * 0.5;
}

void GaitController::bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  body_pose_ = *msg;
}

void GaitController::balanceAdjustmentCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  balance_adjustment_ = *msg;
  RCLCPP_DEBUG(get_logger(), "Balance adjustment: roll=%.3f pitch=%.3f height=%.3f",
               msg->x, msg->y, msg->z);
}

void GaitController::gaitLoop()
{
  auto current_time = get_clock()->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;
  
  if (dt <= 0 || dt > 0.1) {
    dt = 1.0 / control_rate_;
  }
  
  // Update gait phase
  current_phase_ += dt / gait_period_;
  current_phase_ = std::fmod(current_phase_, 1.0);
  
  // Calculate foot positions and solve IK
  calculateAllFootPositions();
  
  // Publish
  publishJointTrajectory();
  publishFootPositions();
  publishGaitState();
}

void GaitController::calculateFootTrajectory(int leg_idx, double phase,
                                              double &x, double &y, double &z)
{
  // Default foot position (neutral stance)
  double default_x = leg_positions_[leg_idx][0] * 0.5;
  double default_y = leg_positions_[leg_idx][1] * 0.8;
  double default_z = -stance_height_;
  
  double direction = (std::abs(velocity_cmd_.linear.x) > 0.01) ? 
                     std::copysign(1.0, velocity_cmd_.linear.x) : 1.0;
  
  double x_offset = 0.0;
  double z_offset = 0.0;
  
  if (phase < 0.5) {
    // Swing phase
    double swing_phase = phase * 2.0;
    x_offset = -step_length_ + (2.0 * step_length_ * swing_phase);
    z_offset = step_height_ * std::sin(swing_phase * M_PI);
  } else {
    // Stance phase
    double stance_phase = (phase - 0.5) * 2.0;
    x_offset = step_length_ - (2.0 * step_length_ * stance_phase);
    z_offset = 0.0;
  }
  
  // Apply turning
  double turn_offset = 0.0;
  if (std::abs(velocity_cmd_.angular.z) > 0.01) {
    double leg_sign = (leg_idx == LF || leg_idx == LR) ? 1.0 : -1.0;
    turn_offset = leg_sign * velocity_cmd_.angular.z * 0.05 * std::sin(phase * 2.0 * M_PI);
  }
  
  x = default_x + x_offset + turn_offset;
  y = default_y;
  z = default_z + z_offset;
}

void GaitController::applyBalanceCorrections(double &x, double &y, double &z, int leg_idx)
{
  if (!balance_enabled_) {
    return;
  }
  
  // balance_adjustment:
  // x = roll correction (rad)
  // y = pitch correction (rad)
  // z = height adjustment (m)
  
  double roll_corr = balance_adjustment_.x * balance_response_factor_;
  double pitch_corr = balance_adjustment_.y * balance_response_factor_;
  double height_corr = balance_adjustment_.z * balance_response_factor_;
  
  // Apply height correction to all legs
  z -= height_corr;
  
  // Apply roll correction (side-to-side tilt)
  // Positive roll = tilt to left, so left legs need to go down, right legs up
  double leg_sign_y = (leg_idx == LF || leg_idx == LR) ? 1.0 : -1.0;
  z += roll_corr * leg_sign_y * (body_width_ / 2.0);
  
  // Apply pitch correction (front-to-back tilt)
  // Positive pitch = tilt up, so front legs need to go down, rear legs up
  double leg_sign_x = (leg_idx == LF || leg_idx == RF) ? 1.0 : -1.0;
  z -= pitch_corr * leg_sign_x * (body_length_ / 2.0);
  
  // Small horizontal adjustment for CoG shifting
  x += pitch_corr * 0.02;  // Move feet slightly forward/backward for pitch
  y -= roll_corr * 0.02;   // Move feet slightly left/right for roll
}

void GaitController::calculateAllFootPositions()
{
  for (int leg_idx = 0; leg_idx < 4; ++leg_idx) {
    double leg_phase = std::fmod(current_phase_ + gait_phase_offset_[leg_idx], 1.0);
    
    double fx, fy, fz;
    calculateFootTrajectory(leg_idx, leg_phase, fx, fy, fz);
    
    // Store original positions
    foot_positions_[leg_idx] = {{fx, fy, fz}};
    
    // Apply balance corrections
    applyBalanceCorrections(fx, fy, fz, leg_idx);
    foot_positions_compensated_[leg_idx] = {{fx, fy, fz}};
    
    // Solve IK with compensated positions
    double hip, thigh, shin;
    solveLegIK(leg_idx, fx, fy, fz, hip, thigh, shin);
    
    joint_angles_[leg_idx * 3 + 0] = hip;
    joint_angles_[leg_idx * 3 + 1] = thigh;
    joint_angles_[leg_idx * 3 + 2] = shin;
  }
}

void GaitController::solveLegIK(int leg_idx, double foot_x, double foot_y, double foot_z,
                                 double &hip_angle, double &thigh_angle, double &shin_angle)
{
  // Hip angle (abduction/adduction)
  hip_angle = std::atan2(foot_y, -foot_z);
  
  // Project to leg plane
  double d = std::sqrt(foot_y * foot_y + foot_z * foot_z);
  double leg_plane_x = foot_x;
  double leg_plane_z = d - hip_length_;
  
  // 2D IK in leg plane using law of cosines
  double target_dist = std::sqrt(leg_plane_x * leg_plane_x + leg_plane_z * leg_plane_z);
  target_dist = std::min(target_dist, thigh_length_ + shin_length_ - 0.001);
  
  // Shin angle (knee)
  double cos_shin = (thigh_length_ * thigh_length_ + shin_length_ * shin_length_ - target_dist * target_dist) /
                    (2.0 * thigh_length_ * shin_length_);
  cos_shin = std::max(-1.0, std::min(1.0, cos_shin));
  shin_angle = M_PI - std::acos(cos_shin);
  
  // Thigh angle (hip pitch)
  double cos_thigh = (target_dist * target_dist + thigh_length_ * thigh_length_ - shin_length_ * shin_length_) /
                     (2.0 * target_dist * thigh_length_);
  cos_thigh = std::max(-1.0, std::min(1.0, cos_thigh));
  
  double target_angle = std::atan2(leg_plane_x, -leg_plane_z);
  thigh_angle = target_angle - std::acos(cos_thigh);
  
  // Mirror angles for right legs (RF and RR)
  if (leg_idx == RF || leg_idx == RR) {
    hip_angle = -hip_angle;
    thigh_angle = -thigh_angle;
    shin_angle = -shin_angle;
  }
}

void GaitController::publishJointTrajectory()
{
  auto msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  msg->header.stamp = get_clock()->now();
  msg->joint_names = std::vector<std::string>(
    joint_names_.begin(), joint_names_.end());
  
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = std::vector<double>(joint_angles_.begin(), joint_angles_.end());
  point.velocities.resize(12, 0.0);
  point.accelerations.resize(12, 0.0);
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 10'000'000;  // 10ms
  
  msg->points.push_back(point);
  joint_traj_pub_->publish(std::move(msg));
}

void GaitController::publishFootPositions()
{
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  // Publish compensated positions
  for (const auto &pos : foot_positions_compensated_) {
    msg->data.push_back(pos[0]);
    msg->data.push_back(pos[1]);
    msg->data.push_back(pos[2]);
  }
  foot_positions_pub_->publish(std::move(msg));
}

void GaitController::publishGaitState()
{
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  msg->data.push_back(current_phase_);  // Current gait phase
  msg->data.push_back(step_length_);    // Current step length
  msg->data.push_back(velocity_cmd_.linear.x);   // X velocity
  msg->data.push_back(velocity_cmd_.angular.z);  // Angular velocity
  msg->data.push_back(balance_adjustment_.x);    // Roll correction
  msg->data.push_back(balance_adjustment_.y);    // Pitch correction
  msg->data.push_back(balance_adjustment_.z);    // Height correction
  gait_state_pub_->publish(std::move(msg));
}

}  // namespace dog_control_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<dog_control_cpp::GaitController>(options);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
