#include "dog_teleop_cpp/gamepad_teleop.hpp"

#include <rclcpp/qos.hpp>
#include <cmath>
#include <algorithm>

namespace dog_teleop_cpp
{

GamepadTeleop::GamepadTeleop(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gamepad_teleop", options),
  max_linear_speed_(0.5),
  max_angular_speed_(2.0),
  max_lateral_speed_(0.3),
  axis_deadzone_(0.1),
  publish_rate_(50.0),
  watchdog_timeout_(0.5),
  twist_smoothing_(0.3),
  target_height_(0.25),
  gait_speed_(1.0),
  emergency_stop_(false),
  servos_enabled_(false),
  active_(false)
{
  declareParameters();
}

GamepadTeleop::~GamepadTeleop()
{
  on_shutdown(get_current_state());
}

void GamepadTeleop::declareParameters()
{
  declare_parameter("max_linear_speed", max_linear_speed_);
  declare_parameter("max_angular_speed", max_angular_speed_);
  declare_parameter("max_lateral_speed", max_lateral_speed_);
  declare_parameter("axis_deadzone", axis_deadzone_);
  declare_parameter("publish_rate", publish_rate_);
  declare_parameter("watchdog_timeout", watchdog_timeout_);
  declare_parameter("twist_smoothing", twist_smoothing_);
}

LifecycleCallbackReturn GamepadTeleop::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Gamepad Teleop...");

  // Get parameters
  max_linear_speed_ = get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_parameter("max_angular_speed").as_double();
  max_lateral_speed_ = get_parameter("max_lateral_speed").as_double();
  axis_deadzone_ = get_parameter("axis_deadzone").as_double();
  publish_rate_ = get_parameter("publish_rate").as_double();
  watchdog_timeout_ = get_parameter("watchdog_timeout").as_double();
  twist_smoothing_ = get_parameter("twist_smoothing").as_double();

  // QoS
  rclcpp::QoS cmd_qos(5);
  cmd_qos.reliable();

  // Create publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);
  cmd_height_pub_ = create_publisher<std_msgs::msg::Float64>("/cmd_height", 10);
  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop_trigger", 10);
  servo_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/servo_enable", 10);

  // Subscriber (created but only active when node is active)
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
      if (active_) {
        handleJoy(msg);
      }
    });

  RCLCPP_INFO(get_logger(), "Gamepad Teleop configured");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadTeleop::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Gamepad Teleop...");

  // Activate publishers
  cmd_vel_pub_->on_activate();
  cmd_height_pub_->on_activate();
  emergency_stop_pub_->on_activate();
  servo_enable_pub_->on_activate();

  // Start timers
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publishLoop(); });

  auto watchdog_period = std::chrono::duration<double>(watchdog_timeout_ / 2.0);
  watchdog_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(watchdog_period),
    [this]() { watchdogLoop(); });

  last_joy_time_ = now();
  active_ = true;

  RCLCPP_INFO(get_logger(), "Gamepad Teleop activated");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadTeleop::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Gamepad Teleop...");

  active_ = false;

  // Cancel timers
  publish_timer_.reset();
  watchdog_timer_.reset();

  // Deactivate publishers
  cmd_vel_pub_->on_deactivate();
  cmd_height_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();
  servo_enable_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Gamepad Teleop deactivated");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadTeleop::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Gamepad Teleop...");

  cmd_vel_pub_.reset();
  cmd_height_pub_.reset();
  emergency_stop_pub_.reset();
  servo_enable_pub_.reset();
  joy_sub_.reset();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadTeleop::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Gamepad Teleop...");
  active_ = false;
  return LifecycleCallbackReturn::SUCCESS;
}

void GamepadTeleop::handleJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->axes.size() < 6 || msg->buttons.size() < 8) {
    return;
  }

  // Store previous buttons for edge detection
  if (prev_buttons_.empty()) {
    prev_buttons_.resize(msg->buttons.size(), 0);
  }

  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    last_joy_time_ = now();
  }

  // Handle buttons
  handleButtons(msg->buttons);

  // Handle axes
  handleAxes(msg->axes);

  prev_buttons_ = msg->buttons;
}

void GamepadTeleop::handleButtons(const std::vector<int32_t> & buttons)
{
  // Emergency stop (X button)
  if (buttons[BTN_X] && !prev_buttons_[BTN_X]) {
    emergency_stop_ = true;
    publishEmergencyStop(true);
    RCLCPP_WARN(get_logger(), "EMERGENCY STOP ACTIVATED (gamepad)!");
  }

  // Reset emergency stop (Y button)
  if (buttons[BTN_Y] && !prev_buttons_[BTN_Y]) {
    emergency_stop_ = false;
    publishEmergencyStop(false);
    RCLCPP_INFO(get_logger(), "Emergency stop reset (gamepad)");
  }

  // Stand up (A button)
  if (buttons[BTN_A] && !prev_buttons_[BTN_A]) {
    target_height_ = 0.25;
    publishHeight();
    RCLCPP_INFO(get_logger(), "Stand up command");
  }

  // Sit down (B button)
  if (buttons[BTN_B] && !prev_buttons_[BTN_B]) {
    target_height_ = 0.15;
    publishHeight();
    RCLCPP_INFO(get_logger(), "Sit down command");
  }

  // Enable servos (Start button)
  if (buttons[BTN_START] && !prev_buttons_[BTN_START]) {
    servos_enabled_ = true;
    publishServoEnable(true);
    RCLCPP_INFO(get_logger(), "Servos enabled");
  }

  // Disable servos (Back button)
  if (buttons[BTN_BACK] && !prev_buttons_[BTN_BACK]) {
    servos_enabled_ = false;
    publishServoEnable(false);
    RCLCPP_INFO(get_logger(), "Servos disabled");
  }

  // Gait speed adjustment (LB/RB)
  if (buttons[BTN_LB] && !prev_buttons_[BTN_LB]) {
    gait_speed_ = std::max(0.5, gait_speed_ - 0.1);
    RCLCPP_INFO(get_logger(), "Gait speed: %.1f", gait_speed_);
  }
  if (buttons[BTN_RB] && !prev_buttons_[BTN_RB]) {
    gait_speed_ = std::min(2.0, gait_speed_ + 0.1);
    RCLCPP_INFO(get_logger(), "Gait speed: %.1f", gait_speed_);
  }
}

void GamepadTeleop::handleAxes(const std::vector<float> & axes)
{
  // Left stick for movement
  double left_x = applyDeadzone(axes[AXIS_LEFT_X]);
  double left_y = applyDeadzone(-axes[AXIS_LEFT_Y]);  // Invert Y
  double right_x = applyDeadzone(axes[AXIS_RIGHT_X]);

  // Calculate velocities with smoothing
  double target_x = left_y * max_linear_speed_ * gait_speed_;
  double target_y = left_x * max_lateral_speed_ * gait_speed_;
  double target_z = right_x * max_angular_speed_ * gait_speed_;

  std::lock_guard<std::mutex> lock(twist_mutex_);

  // Smooth interpolation
  smoothed_twist_.linear.x = lerp(smoothed_twist_.linear.x, target_x, twist_smoothing_);
  smoothed_twist_.linear.y = lerp(smoothed_twist_.linear.y, target_y, twist_smoothing_);
  smoothed_twist_.angular.z = lerp(smoothed_twist_.angular.z, target_z, twist_smoothing_);

  current_twist_ = smoothed_twist_;

  // Height adjustment (triggers - LT/RT)
  if (axes.size() > AXIS_RT) {
    double lt = (axes[AXIS_LT] + 1.0) / 2.0;  // Normalize [-1,1] to [0,1]
    double rt = (axes[AXIS_RT] + 1.0) / 2.0;

    if (lt > 0.5) {
      target_height_ = std::max(0.15, target_height_ - 0.002);
      publishHeight();
    } else if (rt > 0.5) {
      target_height_ = std::min(0.35, target_height_ + 0.002);
      publishHeight();
    }
  }
}

double GamepadTeleop::applyDeadzone(double value)
{
  if (std::abs(value) < axis_deadzone_) {
    return 0.0;
  }
  double sign = (value > 0) ? 1.0 : -1.0;
  return sign * (std::abs(value) - axis_deadzone_) / (1.0 - axis_deadzone_);
}

double GamepadTeleop::lerp(double current, double target, double factor)
{
  return current + (target - current) * factor;
}

void GamepadTeleop::publishLoop()
{
  if (!active_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  if (emergency_stop_) {
    publishZeroVelocity();
    return;
  }

  geometry_msgs::msg::Twist twist;
  {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    twist = current_twist_;
  }

  auto msg = std::make_unique<geometry_msgs::msg::Twist>(twist);
  cmd_vel_pub_->publish(std::move(msg));
}

void GamepadTeleop::watchdogLoop()
{
  if (!active_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  rclcpp::Time last_time;
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    last_time = last_joy_time_;
  }

  double elapsed = (now() - last_time).seconds();

  if (elapsed > watchdog_timeout_) {
    // No joy message for too long - stop the robot
    {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      current_twist_ = geometry_msgs::msg::Twist();
      smoothed_twist_ = geometry_msgs::msg::Twist();
    }
    publishZeroVelocity();

    static rclcpp::Time last_warn = now();
    if ((now() - last_warn).seconds() > 2.0) {
      RCLCPP_WARN(get_logger(), "Watchdog: No gamepad input for %.2f seconds, stopping", elapsed);
      last_warn = now();
    }
  }
}

void GamepadTeleop::publishHeight()
{
  if (!active_ || !cmd_height_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<std_msgs::msg::Float64>();
  msg->data = target_height_;
  cmd_height_pub_->publish(std::move(msg));
}

void GamepadTeleop::publishEmergencyStop(bool stop)
{
  if (!active_ || !emergency_stop_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = stop;
  emergency_stop_pub_->publish(std::move(msg));
}

void GamepadTeleop::publishServoEnable(bool enable)
{
  if (!active_ || !servo_enable_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = enable;
  servo_enable_pub_->publish(std::move(msg));
}

void GamepadTeleop::publishZeroVelocity()
{
  if (!cmd_vel_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_pub_->publish(std::move(msg));
}

}  // namespace dog_teleop_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<dog_teleop_cpp::GamepadTeleop>();
  exec.add_node(node->get_node_base_interface());

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
