/**
 * @file simulation_driver.cpp
 * @brief Simulation driver implementation
 */

#include "dog_hardware_cpp/simulation_driver.hpp"

#include <random>

namespace dog_hardware_cpp
{

SimulationDriver::SimulationDriver()
{
}

SimulationDriver::~SimulationDriver()
{
  shutdown();
}

bool SimulationDriver::initialize(const std::string& device_port, uint8_t servo_count)
{
  if (initialized_)
  {
    last_error_ = "Already initialized";
    return false;
  }

  device_port_ = device_port;
  servo_count_ = servo_count;

  servos_.resize(servo_count);

  // Initialize default configs
  for (uint8_t i = 0; i < servo_count; ++i)
  {
    servos_[i].config.id = i;
    servos_[i].config.name = "joint_" + std::to_string(i);
    servos_[i].actual_position_rad = 0.0;
    servos_[i].target_position_rad = 0.0;
    servos_[i].enabled = false;
    servos_[i].state.is_enabled = false;
    servos_[i].state.position_rad = 0.0;
    servos_[i].last_update = std::chrono::steady_clock::now();
  }

  running_ = true;
  if (physics_enabled_)
  {
    physics_thread_ = std::thread(&SimulationDriver::physicsLoop, this);
  }

  initialized_ = true;
  return true;
}

void SimulationDriver::shutdown()
{
  running_ = false;
  cv_.notify_all();

  if (physics_thread_.joinable())
  {
    physics_thread_.join();
  }

  initialized_ = false;
}

bool SimulationDriver::isInitialized() const
{
  return initialized_;
}

void SimulationDriver::physicsLoop()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> noise(0.0, 1.0);

  auto next_tick = std::chrono::steady_clock::now();

  while (running_)
  {
    next_tick += std::chrono::duration<double>(PHYSICS_DT);
    std::this_thread::sleep_until(next_tick);

    std::lock_guard<std::mutex> lock(servos_mutex_);

    for (auto& servo : servos_)
    {
      if (!servo.enabled || emergency_stopped_)
      {
        servo.state.velocity_rad_s = 0.0;
        continue;
      }

      // Simple first-order lag model
      double error = servo.target_position_rad - servo.actual_position_rad;
      double max_step = servo.config.max_velocity_rad_s * PHYSICS_DT;

      double step = std::copysign(std::min(std::abs(error), max_step), error);
      servo.actual_position_rad += step;

      // Add noise if configured
      if (position_noise_rad_ > 0.0)
      {
        servo.actual_position_rad += noise(gen) * position_noise_rad_;
      }

      // Update state
      servo.state.position_rad = servo.actual_position_rad;
      servo.state.velocity_rad_s = step / PHYSICS_DT;
      servo.state.is_moving = std::abs(error) > 0.01;  // 0.01 rad threshold

      // Simulate temperature rise under load
      double load = std::abs(error) / (servo.config.max_angle_rad - servo.config.min_angle_rad);
      servo.state.temperature = 25.0 + load * 30.0;  // 25-55C range
    }
  }
}

bool SimulationDriver::configureServo(const ServoConfig& config)
{
  if (config.id >= servo_count_)
  {
    last_error_ = "Invalid servo ID: " + std::to_string(config.id);
    return false;
  }

  std::lock_guard<std::mutex> lock(servos_mutex_);
  servos_[config.id].config = config;
  servos_[config.id].actual_position_rad = config.neutral_angle_rad;
  servos_[config.id].target_position_rad = config.neutral_angle_rad;
  return true;
}

bool SimulationDriver::setServoEnabled(uint8_t servo_id, bool enable)
{
  if (servo_id >= servo_count_)
  {
    last_error_ = "Invalid servo ID";
    return false;
  }

  if (emergency_stopped_ && enable)
  {
    last_error_ = "Cannot enable servo during emergency stop";
    return false;
  }

  std::lock_guard<std::mutex> lock(servos_mutex_);
  servos_[servo_id].enabled = enable;
  servos_[servo_id].state.is_enabled = enable;

  return true;
}

bool SimulationDriver::setPosition(uint8_t servo_id, double position_rad)
{
  if (servo_id >= servo_count_)
  {
    last_error_ = "Invalid servo ID: " + std::to_string(servo_id);
    return false;
  }

  if (emergency_stopped_)
  {
    last_error_ = "Emergency stop active";
    return false;
  }

  if (!servos_[servo_id].enabled)
  {
    last_error_ = "Servo " + std::to_string(servo_id) + " not enabled";
    return false;
  }

  std::lock_guard<std::mutex> lock(servos_mutex_);

  // Apply position limits
  const auto& config = servos_[servo_id].config;
  position_rad = std::max(config.min_angle_rad, std::min(config.max_angle_rad, position_rad));

  servos_[servo_id].target_position_rad = position_rad;

  // Simulate response delay
  if (!physics_enabled_)
  {
    servos_[servo_id].actual_position_rad = position_rad;
    servos_[servo_id].state.position_rad = position_rad;
  }

  return true;
}

bool SimulationDriver::setPositionWithVelocity(uint8_t servo_id, double position_rad, double velocity_rad_s)
{
  if (servo_id >= servo_count_)
  {
    last_error_ = "Invalid servo ID";
    return false;
  }

  std::lock_guard<std::mutex> lock(servos_mutex_);

  // Override max velocity for this move
  double original_velocity = servos_[servo_id].config.max_velocity_rad_s;
  servos_[servo_id].config.max_velocity_rad_s = velocity_rad_s;

  bool result = setPosition(servo_id, position_rad);

  // Restore original
  servos_[servo_id].config.max_velocity_rad_s = original_velocity;

  return result;
}

bool SimulationDriver::setTorqueLimit(uint8_t servo_id, double torque)
{
  if (servo_id >= servo_count_)
  {
    last_error_ = "Invalid servo ID";
    return false;
  }

  std::lock_guard<std::mutex> lock(servos_mutex_);
  servos_[servo_id].state.effort = torque;
  return true;
}

std::optional<ServoState> SimulationDriver::getState(uint8_t servo_id)
{
  if (servo_id >= servo_count_)
  {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(servos_mutex_);
  return servos_[servo_id].state;
}

std::vector<ServoState> SimulationDriver::getAllStates()
{
  std::lock_guard<std::mutex> lock(servos_mutex_);

  std::vector<ServoState> states;
  states.reserve(servo_count_);

  for (const auto& servo : servos_)
  {
    states.push_back(servo.state);
  }

  return states;
}

void SimulationDriver::emergencyStop()
{
  emergency_stopped_ = true;

  std::lock_guard<std::mutex> lock(servos_mutex_);
  for (auto& servo : servos_)
  {
    servo.enabled = false;
    servo.state.is_enabled = false;
    servo.target_position_rad = servo.config.neutral_angle_rad;
  }
}

void SimulationDriver::resetEmergencyStop()
{
  emergency_stopped_ = false;
}

bool SimulationDriver::isEmergencyStopped() const
{
  return emergency_stopped_;
}

} // namespace dog_hardware_cpp
