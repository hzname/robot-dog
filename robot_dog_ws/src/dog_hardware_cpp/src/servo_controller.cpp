/**
 * @file servo_controller.cpp
 * @brief Servo controller implementation
 */

#include "dog_hardware_cpp/servo_controller.hpp"

#include <math>

namespace dog_hardware_cpp
{

ServoController::ServoController(rclcpp::Logger logger)
    : logger_(logger),
      target_positions_{0},
      smoothed_positions_{0},
      current_positions_{0},
      previous_positions_{0}
{
  // Initialize default joint limits
  for (size_t i = 0; i < 12; ++i)
  {
    joint_limits_[i].name = DEFAULT_JOINT_NAMES[i];
    
    // Hip joints: roll motion
    if (i % 3 == 0)
    {
      joint_limits_[i].min_position_rad = -0.8;
      joint_limits_[i].max_position_rad = 0.8;
      joint_limits_[i].neutral_position_rad = 0.0;
    }
    // Thigh joints: pitch up/down
    else if (i % 3 == 1)
    {
      joint_limits_[i].min_position_rad = -1.5;
      joint_limits_[i].max_position_rad = 1.5;
      joint_limits_[i].neutral_position_rad = 0.0;
    }
    // Shin joints: knee bend
    else
    {
      joint_limits_[i].min_position_rad = -2.5;
      joint_limits_[i].max_position_rad = -0.5;
      joint_limits_[i].neutral_position_rad = -1.2;
    }
  }
}

ServoController::~ServoController()
{
  shutdown();
}

bool ServoController::initialize(const ControllerConfig& config)
{
  if (initialized_)
  {
    RCLCPP_WARN(logger_, "Controller already initialized");
    return true;
  }

  config_ = config;

  // Create hardware interface
  if (config_.bus_type == "simulation")
  {
    hardware_ = std::make_shared<SimulationDriver>();
    if (config_.device_port.empty())
    {
      config_.device_port = "sim://localhost";
    }
  }
  else if (config_.bus_type == "i2c")
  {
    hardware_ = std::make_shared<PCA9685Driver>();
    if (config_.device_port.empty())
    {
      config_.device_port = "/dev/i2c-1";
    }
  }
  else
  {
    last_error_ = "Unknown bus type: " + config_.bus_type;
    RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
    return false;
  }

  // Initialize hardware
  if (!hardware_->initialize(config_.device_port, config_.servo_count))
  {
    last_error_ = "Failed to initialize hardware: " + hardware_->getLastError();
    RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
    hardware_.reset();
    return false;
  }

  RCLCPP_INFO(logger_, "Hardware initialized: %s on %s",
              hardware_->getInterfaceName().c_str(),
              config_.device_port.c_str());

  // Configure servos
  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    ServoConfig servo_config;
    servo_config.id = i;
    servo_config.name = joint_limits_[i].name;
    servo_config.min_angle_rad = joint_limits_[i].min_position_rad;
    servo_config.max_angle_rad = joint_limits_[i].max_position_rad;
    servo_config.neutral_angle_rad = joint_limits_[i].neutral_position_rad;
    servo_config.max_velocity_rad_s = joint_limits_[i].max_velocity_rad_s;
    servo_config.max_torque = config_.max_torque;

    if (!hardware_->configureServo(servo_config))
    {
      RCLCPP_WARN(logger_, "Failed to configure servo %d", i);
    }

    servo_configs_[i] = servo_config;
    target_positions_[i] = servo_config.neutral_angle_rad;
    smoothed_positions_[i] = servo_config.neutral_angle_rad;
    current_positions_[i] = servo_config.neutral_angle_rad;
  }

  // Read initial positions from hardware if available
  auto states = hardware_->getAllStates();
  for (size_t i = 0; i < states.size() && i < 12; ++i)
  {
    current_positions_[i] = states[i].position_rad;
  }

  last_command_time_ = rclcpp::Clock().now();
  last_update_time_ = last_command_time_;

  initialized_ = true;
  RCLCPP_INFO(logger_, "Servo controller initialized with %d servos", config_.servo_count);

  return true;
}

void ServoController::shutdown()
{
  if (!initialized_)
  {
    return;
  }

  RCLCPP_INFO(logger_, "Shutting down servo controller");

  emergencyStop();

  if (hardware_)
  {
    hardware_->shutdown();
    hardware_.reset();
  }

  initialized_ = false;
}

bool ServoController::configureJointLimits(uint8_t servo_id, const JointLimits& limits)
{
  if (servo_id >= config_.servo_count)
  {
    last_error_ = "Invalid servo ID: " + std::to_string(servo_id);
    return false;
  }

  joint_limits_[servo_id] = limits;

  // Update hardware config
  ServoConfig servo_config;
  servo_config.id = servo_id;
  servo_config.name = limits.name;
  servo_config.min_angle_rad = limits.min_position_rad;
  servo_config.max_angle_rad = limits.max_position_rad;
  servo_config.neutral_angle_rad = limits.neutral_position_rad;
  servo_config.max_velocity_rad_s = limits.max_velocity_rad_s;

  servo_configs_[servo_id] = servo_config;

  if (hardware_)
  {
    return hardware_->configureServo(servo_config);
  }

  return true;
}

bool ServoController::setTargetPositions(const std::array<double, 12>& positions)
{
  if (!initialized_)
  {
    last_error_ = "Controller not initialized";
    return false;
  }

  if (emergency_stopped_)
  {
    last_error_ = "Emergency stop active";
    return false;
  }

  // Validate all positions
  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    double pos = positions[i];
    if (!applySafetyLimits(i, pos))
    {
      RCLCPP_WARN(logger_, "Position for %s violates safety limits",
                  joint_limits_[i].name.c_str());
    }
    target_positions_[i] = pos;
  }

  resetWatchdog();
  return true;
}

bool ServoController::setTargetPosition(uint8_t servo_id, double position_rad)
{
  if (servo_id >= config_.servo_count)
  {
    last_error_ = "Invalid servo ID";
    return false;
  }

  if (!initialized_ || emergency_stopped_)
  {
    return false;
  }

  applySafetyLimits(servo_id, position_rad);
  target_positions_[servo_id] = position_rad;
  resetWatchdog();
  return true;
}

bool ServoController::setAllEnabled(bool enable)
{
  if (!hardware_)
  {
    return false;
  }

  bool success = true;
  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    if (!hardware_->setServoEnabled(i, enable))
    {
      RCLCPP_WARN(logger_, "Failed to %s servo %d",
                  enable ? "enable" : "disable", i);
      success = false;
    }
  }

  RCLCPP_INFO(logger_, "All servos %s", enable ? "enabled" : "disabled");
  return success;
}

bool ServoController::setServoEnabled(uint8_t servo_id, bool enable)
{
  if (!hardware_ || servo_id >= config_.servo_count)
  {
    return false;
  }

  return hardware_->setServoEnabled(servo_id, enable);
}

void ServoController::update()
{
  if (!initialized_)
  {
    return;
  }

  auto now = rclcpp::Clock().now();

  // Check watchdog
  if (config_.enable_watchdog)
  {
    checkWatchdog();
  }

  // Interpolate if enabled
  if (config_.enable_interpolation)
  {
    interpolatePositions();
  }
  else
  {
    smoothed_positions_ = target_positions_;
  }

  // Apply rate limiting
  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    applyRateLimiting(i, smoothed_positions_[i]);
  }

  // Send to hardware
  sendToHardware();

  // Update previous positions
  previous_positions_ = smoothed_positions_;
  last_update_time_ = now;
}

bool ServoController::applySafetyLimits(uint8_t servo_id, double& position)
{
  const auto& limits = joint_limits_[servo_id];

  // Position clamps
  position = std::max(limits.min_position_rad,
                      std::min(limits.max_position_rad, position));

  return true;
}

bool ServoController::applyRateLimiting(uint8_t servo_id, double& position)
{
  double delta = position - previous_positions_[servo_id];
  double max_delta = config_.max_position_delta_rad;

  // Also consider velocity limit
  double dt = 1.0 / config_.control_rate_hz;
  double max_vel_delta = servo_configs_[servo_id].max_velocity_rad_s * dt;
  max_delta = std::min(max_delta, max_vel_delta);

  if (std::abs(delta) > max_delta)
  {
    delta = std::copysign(max_delta, delta);
    position = previous_positions_[servo_id] + delta;
  }

  return true;
}

void ServoController::interpolatePositions()
{
  double dt = 1.0 / config_.control_rate_hz;
  double alpha = dt / config_.interpolation_time_s;
  alpha = std::min(1.0, alpha);  // Clamp to avoid overshoot

  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    double error = target_positions_[i] - smoothed_positions_[i];
    smoothed_positions_[i] += error * alpha;
  }
}

void ServoController::checkWatchdog()
{
  if (!config_.enable_watchdog || emergency_stopped_)
  {
    return;
  }

  auto now = rclcpp::Clock().now();
  double elapsed = (now - last_command_time_).seconds();

  if (elapsed > config_.watchdog_timeout_s)
  {
    RCLCPP_ERROR(logger_, "Watchdog timeout! No commands for %.2f seconds", elapsed);
    watchdog_triggered_ = true;
    emergencyStop();
  }
}

void ServoController::sendToHardware()
{
  if (!hardware_)
  {
    return;
  }

  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    hardware_->setPosition(i, smoothed_positions_[i]);
  }

  // Read back actual positions
  auto states = hardware_->getAllStates();
  for (size_t i = 0; i < states.size() && i < 12; ++i)
  {
    current_positions_[i] = states[i].position_rad;
  }
}

void ServoController::emergencyStop()
{
  if (emergency_stopped_)
  {
    return;
  }

  RCLCPP_ERROR(logger_, "EMERGENCY STOP ACTIVATED!");
  emergency_stopped_ = true;

  if (hardware_)
  {
    hardware_->emergencyStop();
  }

  // Set targets to neutral positions
  for (uint8_t i = 0; i < config_.servo_count; ++i)
  {
    target_positions_[i] = servo_configs_[i].neutral_angle_rad;
  }
}

void ServoController::resetEmergencyStop()
{
  if (!emergency_stopped_)
  {
    return;
  }

  RCLCPP_INFO(logger_, "Emergency stop reset");
  emergency_stopped_ = false;
  watchdog_triggered_ = false;

  if (hardware_)
  {
    hardware_->resetEmergencyStop();
  }
}

bool ServoController::isEmergencyStopped() const
{
  return emergency_stopped_;
}

std::array<double, 12> ServoController::getCurrentPositions() const
{
  return current_positions_;
}

std::array<double, 12> ServoController::getTargetPositions() const
{
  return target_positions_;
}

std::vector<ServoState> ServoController::getJointStates() const
{
  if (hardware_)
  {
    return hardware_->getAllStates();
  }
  return {};
}

bool ServoController::isWatchdogTriggered() const
{
  return watchdog_triggered_;
}

void ServoController::resetWatchdog()
{
  last_command_time_ = rclcpp::Clock().now();
  watchdog_triggered_ = false;
}

std::shared_ptr<ServoInterface> ServoController::getHardwareInterface() const
{
  return hardware_;
}

} // namespace dog_hardware_cpp
