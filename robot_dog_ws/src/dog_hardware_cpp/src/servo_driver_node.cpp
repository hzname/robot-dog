/**
 * @file servo_driver_node.cpp
 * @brief Servo driver node implementation
 */

#include "dog_hardware_cpp/servo_driver_node.hpp"

#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <algorithm>

namespace dog_hardware_cpp
{

ServoDriverNode::ServoDriverNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("servo_driver_node", options),
      publish_rate_hz_(100.0),
      safety_enabled_(true),
      joint_names_({
          "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
          "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
          "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
          "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint"})
{
  // Declare parameters
  declare_parameter("bus_type", "simulation");
  declare_parameter("device_port", "");
  declare_parameter("servo_count", 12);
  declare_parameter("pwm_frequency", 50);
  declare_parameter("publish_rate", publish_rate_hz_);
  declare_parameter("control_rate", 100.0);
  declare_parameter("safety_enabled", safety_enabled_);
  declare_parameter("emergency_stop_on_timeout", true);
  declare_parameter("watchdog_timeout_s", 0.5);
  declare_parameter("enable_watchdog", true);
  declare_parameter("max_position_delta_rad", 0.15);
  declare_parameter("max_velocity_rad_s", 10.0);
  declare_parameter("enable_interpolation", true);
  declare_parameter("interpolation_time_s", 0.05);
}

ServoDriverNode::~ServoDriverNode()
{
  if (control_timer_)
  {
    control_timer_->cancel();
  }
}

void ServoDriverNode::loadParameters()
{
  // Load controller config
  controller_config_.bus_type = get_parameter("bus_type").as_string();
  controller_config_.device_port = get_parameter("device_port").as_string();
  controller_config_.servo_count = static_cast<uint8_t>(get_parameter("servo_count").as_int());
  controller_config_.control_rate_hz = get_parameter("control_rate").as_double();
  controller_config_.max_position_delta_rad = get_parameter("max_position_delta_rad").as_double();
  controller_config_.max_velocity_rad_s = get_parameter("max_velocity_rad_s").as_double();
  controller_config_.watchdog_timeout_s = get_parameter("watchdog_timeout_s").as_double();
  controller_config_.enable_watchdog = get_parameter("enable_watchdog").as_bool();
  controller_config_.enable_interpolation = get_parameter("enable_interpolation").as_bool();
  controller_config_.interpolation_time_s = get_parameter("interpolation_time_s").as_double();

  publish_rate_hz_ = get_parameter("publish_rate").as_double();
  safety_enabled_ = get_parameter("safety_enabled").as_bool();
}

void ServoDriverNode::loadJointLimitsFromParams()
{
  // Try to load joint limits from parameters
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    std::string prefix = "joints." + joint_names_[i] + ".";
    
    JointLimits limits;
    limits.name = joint_names_[i];

    if (has_parameter(prefix + "min_position"))
    {
      limits.min_position_rad = get_parameter(prefix + "min_position").as_double();
    }
    if (has_parameter(prefix + "max_position"))
    {
      limits.max_position_rad = get_parameter(prefix + "max_position").as_double();
    }
    if (has_parameter(prefix + "neutral_position"))
    {
      limits.neutral_position_rad = get_parameter(prefix + "neutral_position").as_double();
    }
    if (has_parameter(prefix + "max_velocity"))
    {
      limits.max_velocity_rad_s = get_parameter(prefix + "max_velocity").as_double();
    }

    if (controller_)
    {
      controller_->configureJointLimits(i, limits);
    }
  }
}

LifecycleCallbackReturn ServoDriverNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Servo Driver Node");

  loadParameters();

  // Create controller
  controller_ = std::make_unique<ServoController>(get_logger());

  if (!controller_->initialize(controller_config_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to initialize servo controller");
    return LifecycleCallbackReturn::ERROR;
  }

  loadJointLimitsFromParams();

  // QoS profiles
  rclcpp::QoS cmd_qos(5);
  cmd_qos.reliable();

  rclcpp::QoS state_qos(5);
  state_qos.best_effort();

  // Publishers
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", state_qos);
  servo_feedback_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/servo/feedback", 10);
  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
  watchdog_pub_ = create_publisher<std_msgs::msg::Bool>("/servo/watchdog_triggered", 10);

  // Subscribers
  joint_traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory", cmd_qos,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
      { trajectoryCallback(msg); });

  joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_commands", cmd_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      { jointCommandCallback(msg); });

  position_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joint_position_command", cmd_qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      { positionCmdCallback(msg); });

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop_trigger", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      { emergencyStopCallback(msg); });

  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/servo_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      { enableCallback(msg); });

  RCLCPP_INFO(get_logger(), "Servo Driver Node configured successfully");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ServoDriverNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Servo Driver Node");

  joint_state_pub_->on_activate();
  servo_feedback_pub_->on_activate();
  emergency_stop_pub_->on_activate();
  watchdog_pub_->on_activate();

  // Start control loop
  auto period = std::chrono::duration<double>(1.0 / controller_config_.control_rate_hz);
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { controlLoop(); });

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ServoDriverNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Servo Driver Node");

  if (control_timer_)
  {
    control_timer_->cancel();
  }

  // Disable all servos
  if (controller_)
  {
    controller_->setAllEnabled(false);
  }

  joint_state_pub_->on_deactivate();
  servo_feedback_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();
  watchdog_pub_->on_deactivate();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ServoDriverNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Servo Driver Node");

  if (controller_)
  {
    controller_->shutdown();
    controller_.reset();
  }

  joint_state_pub_.reset();
  servo_feedback_pub_.reset();
  emergency_stop_pub_.reset();
  watchdog_pub_.reset();
  joint_traj_sub_.reset();
  joint_cmd_sub_.reset();
  position_cmd_sub_.reset();
  emergency_stop_sub_.reset();
  enable_sub_.reset();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ServoDriverNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Servo Driver Node");
  
  if (control_timer_)
  {
    control_timer_->cancel();
  }

  if (controller_)
  {
    controller_->emergencyStop();
    controller_->shutdown();
  }

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ServoDriverNode::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_ERROR(get_logger(), "Servo Driver Node error state");
  
  if (control_timer_)
  {
    control_timer_->cancel();
  }

  if (controller_)
  {
    controller_->emergencyStop();
  }

  return LifecycleCallbackReturn::SUCCESS;
}

void ServoDriverNode::controlLoop()
{
  if (!controller_)
  {
    return;
  }

  // Update controller (safety limits, interpolation, hardware communication)
  controller_->update();

  // Publish joint states
  publishJointStates();

  // Check watchdog and publish status
  if (watchdog_pub_->is_activated())
  {
    std_msgs::msg::Bool watchdog_msg;
    watchdog_msg.data = controller_->isWatchdogTriggered();
    watchdog_pub_->publish(watchdog_msg);
  }
}

void ServoDriverNode::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty())
  {
    return;
  }

  const auto &point = msg->points[0];
  std::array<double, 12> positions{};
  positions.fill(0.0);  // Default to neutral

  // Apply neutral positions for joints not in trajectory
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    positions[i] = 0.0;  // Will be clamped by controller
  }

  if (!msg->joint_names.empty())
  {
    // Map by joint names
    for (size_t i = 0; i < msg->joint_names.size() && i < point.positions.size(); ++i)
    {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->joint_names[i]);
      if (it != joint_names_.end())
      {
        size_t idx = std::distance(joint_names_.begin(), it);
        positions[idx] = point.positions[i];
      }
    }
  }
  else if (point.positions.size() == NUM_JOINTS)
  {
    // Use array index
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      positions[i] = point.positions[i];
    }
  }

  controller_->setTargetPositions(positions);
}

void ServoDriverNode::jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::array<double, 12> positions{};
  positions.fill(0.0);

  if (!msg->name.empty())
  {
    // Map by joint names
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
      if (it != joint_names_.end())
      {
        size_t idx = std::distance(joint_names_.begin(), it);
        positions[idx] = msg->position[i];
      }
    }
  }
  else if (msg->position.size() == NUM_JOINTS)
  {
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      positions[i] = msg->position[i];
    }
  }

  controller_->setTargetPositions(positions);
}

void ServoDriverNode::positionCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() == NUM_JOINTS)
  {
    std::array<double, 12> positions;
    std::copy(msg->data.begin(), msg->data.end(), positions.begin());
    controller_->setTargetPositions(positions);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Position command has wrong size: %zu (expected %d)",
                msg->data.size(), NUM_JOINTS);
  }
}

void ServoDriverNode::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
  {
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP triggered via topic");
    controller_->emergencyStop();
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Emergency stop reset via topic");
    controller_->resetEmergencyStop();
  }

  if (emergency_stop_pub_->is_activated())
  {
    emergency_stop_pub_->publish(*msg);
  }
}

void ServoDriverNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  servos_enabled_ = msg->data;
  
  if (controller_)
  {
    controller_->setAllEnabled(servos_enabled_);
  }

  RCLCPP_INFO(get_logger(), "Servos %s", servos_enabled_ ? "ENABLED" : "DISABLED");
}

void ServoDriverNode::publishJointStates()
{
  if (!joint_state_pub_->is_activated())
  {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = get_clock()->now();
  msg->name = std::vector<std::string>(joint_names_.begin(), joint_names_.end());

  // Get current positions from controller
  auto positions = controller_->getCurrentPositions();
  auto targets = controller_->getTargetPositions();
  auto states = controller_->getJointStates();

  msg->position.resize(NUM_JOINTS);
  msg->velocity.resize(NUM_JOINTS);
  msg->effort.resize(NUM_JOINTS);

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    msg->position[i] = positions[i];
    
    if (i < static_cast<int>(states.size()))
    {
      msg->velocity[i] = states[i].velocity_rad_s;
      msg->effort[i] = states[i].effort;
    }
  }

  joint_state_pub_->publish(std::move(msg));

  // Publish feedback as Float64MultiArray
  if (servo_feedback_pub_->is_activated())
  {
    auto feedback_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    feedback_msg->data = std::vector<double>(positions.begin(), positions.end());
    servo_feedback_pub_->publish(std::move(feedback_msg));
  }
}

} // namespace dog_hardware_cpp

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<dog_hardware_cpp::ServoDriverNode>();
  
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
