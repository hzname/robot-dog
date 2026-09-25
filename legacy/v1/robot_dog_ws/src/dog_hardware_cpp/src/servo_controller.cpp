/**
 * @file servo_controller.cpp
 * @brief Servo controller implementation
 */

#include "dog_hardware_cpp/servo_controller.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

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
  const std::string& name = joint_limits_[servo_id].name;

  // Use calibrated limits if loaded, otherwise fall back to defaults
  double min_pos, max_pos;
  auto min_it = calibrated_min_.find(name);
  auto max_it = calibrated_max_.find(name);
  if (min_it != calibrated_min_.end() && max_it != calibrated_max_.end()) {
    min_pos = min_it->second;
    max_pos = max_it->second;
  } else {
    min_pos = joint_limits_[servo_id].min_position_rad;
    max_pos = joint_limits_[servo_id].max_position_rad;
  }

  double original = position;

  // Position clamps
  position = std::max(min_pos, std::min(max_pos, position));

  // Return false if clamping occurred so caller can warn
  return (std::abs(position - original) < 1e-9);
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
    // Apply zero offset + inversion before sending to hardware
    double hw_pos = toHardware(i, smoothed_positions_[i]);
    hardware_->setPosition(i, hw_pos);
  }

  // Read back actual positions from hardware
  auto states = hardware_->getAllStates();
  for (size_t i = 0; i < states.size() && i < 12; ++i)
  {
    // Convert hardware position back to internal coordinates
    current_positions_[i] = fromHardware(i, states[i].position_rad);
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

// ─── Calibration and limits loading ───────────────────────────────

static std::string read_file_str(const std::string& path)
{
  std::ifstream f(path);
  if (!f.is_open()) return "";
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static double find_json_number(const std::string& json, const std::string& key)
{
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0.0;
  auto colon = json.find(':', pos);
  if (colon == std::string::npos) return 0.0;
  auto val_start = colon + 1;
  while (val_start < json.size() && (json[val_start] == ' ' || json[val_start] == '\t'))
    val_start++;
  return std::stod(json.substr(val_start));
}

bool ServoController::loadZeroOffsets(const std::string& path)
{
  std::string content = read_file_str(path);
  if (content.empty()) {
    RCLCPP_WARN(logger_, "Calibration file not found: %s, using zero offsets", path.c_str());
    return false;
  }

  // Parse "zero_offsets" section
  auto zpos = content.find("\"zero_offsets\"");
  if (zpos == std::string::npos) {
    RCLCPP_WARN(logger_, "No zero_offsets in %s, using defaults", path.c_str());
    return false;
  }

  // Find the object after "zero_offsets"
  auto obj_start = content.find('{', zpos);
  auto obj_end = content.find('}', obj_start);
  if (obj_start == std::string::npos || obj_end == std::string::npos) return false;

  std::string offsets_json = content.substr(obj_start, obj_end - obj_start + 1);

  for (const char* name : DEFAULT_JOINT_NAMES) {
    zero_offsets_[name] = find_json_number(offsets_json, name);
  }

  RCLCPP_INFO(logger_, "Loaded zero offsets from %s", path.c_str());
  for (const auto& [name, val] : zero_offsets_) {
    RCLCPP_DEBUG(logger_, "  %s: %.4f rad", name, val);
  }
  return true;
}

bool ServoController::loadCalibratedLimits(const std::string& path)
{
  std::string content = read_file_str(path);
  if (content.empty()) {
    RCLCPP_WARN(logger_, "Joint limits file not found: %s, using defaults", path.c_str());
    return false;
  }

  auto jpos = content.find("\"joints\"");
  if (jpos == std::string::npos) {
    RCLCPP_WARN(logger_, "No joints section in %s, using defaults", path.c_str());
    return false;
  }

  // Find each joint's min_rad and max_rad
  for (const char* name : DEFAULT_JOINT_NAMES) {
    auto jjoint = content.find("\"" + std::string(name) + "\"", jpos);
    if (jjoint == std::string::npos) continue;

    auto jobj_start = content.find('{', jjoint);
    auto jobj_end = content.find('}', jobj_start);
    if (jobj_start == std::string::npos || jobj_end == std::string::npos) continue;

    std::string joint_json = content.substr(jobj_start, jobj_end - jobj_start + 1);
    calibrated_min_[name] = find_json_number(joint_json, "min_rad");
    calibrated_max_[name] = find_json_number(joint_json, "max_rad");
  }

  RCLCPP_INFO(logger_, "Loaded calibrated limits from %s", path.c_str());
  for (const auto& [name, min_val] : calibrated_min_) {
    auto max_it = calibrated_max_.find(name);
    if (max_it != calibrated_max_.end()) {
      RCLCPP_DEBUG(logger_, "  %s: [%.3f, %.3f] rad", name, min_val, max_it->second);
    }
  }
  return true;
}

double ServoController::toHardware(uint8_t servo_id, double position_rad)
{
  const std::string& name = joint_limits_[servo_id].name;
  double pos = position_rad;

  // Apply zero offset
  auto it = zero_offsets_.find(name);
  if (it != zero_offsets_.end()) {
    pos += it->second;
  }

  // Apply inversion for right-side servos
  if (inverted_[servo_id]) {
    pos = -pos;
  }

  return pos;
}

double ServoController::fromHardware(uint8_t servo_id, double position_rad)
{
  const std::string& name = joint_limits_[servo_id].name;
  double pos = position_rad;

  // Undo inversion for right-side servos
  if (inverted_[servo_id]) {
    pos = -pos;
  }

  // Undo zero offset
  auto it = zero_offsets_.find(name);
  if (it != zero_offsets_.end()) {
    pos -= it->second;
  }

  return pos;
}

double ServoController::applyServoTransform(uint8_t servo_id, double position_rad)
{
  return toHardware(servo_id, position_rad);
}

} // namespace dog_hardware_cpp
