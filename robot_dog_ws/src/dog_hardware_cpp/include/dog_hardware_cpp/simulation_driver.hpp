/**
 * @file simulation_driver.hpp
 * @brief Simulation driver for testing without real hardware
 * 
 * Simulates servo behavior including:
 * - Position tracking
 * - Velocity limits
 * - Position limits
 * - Response delays
 */

#ifndef DOG_HARDWARE_CPP__SIMULATION_DRIVER_HPP_
#define DOG_HARDWARE_CPP__SIMULATION_DRIVER_HPP_

#include "dog_hardware_cpp/servo_interface.hpp"

#include <chrono>
#include <math>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace dog_hardware_cpp
{

/**
 * @brief Simulated servo state with physics
 */
struct SimulatedServo
{
  ServoConfig config;
  ServoState state;
  double target_position_rad = 0.0;
  double actual_position_rad = 0.0;
  bool enabled = false;
  std::chrono::steady_clock::time_point last_update;
};

class SimulationDriver : public ServoInterface
{
public:
  SimulationDriver();
  ~SimulationDriver() override;

  bool initialize(const std::string& device_port, uint8_t servo_count) override;
  void shutdown() override;
  bool isInitialized() const override;

  bool configureServo(const ServoConfig& config) override;
  bool setServoEnabled(uint8_t servo_id, bool enable) override;
  bool setPosition(uint8_t servo_id, double position_rad) override;
  bool setPositionWithVelocity(uint8_t servo_id, double position_rad, double velocity_rad_s) override;
  bool setTorqueLimit(uint8_t servo_id, double torque) override;
  std::optional<ServoState> getState(uint8_t servo_id) override;
  std::vector<ServoState> getAllStates() override;

  void emergencyStop() override;
  void resetEmergencyStop() override;
  bool isEmergencyStopped() const override;

  std::string getInterfaceName() const override { return "SIMULATION"; }
  std::string getLastError() const override { return last_error_; }

  // Simulation specific settings
  void setSimulationDelay(std::chrono::milliseconds delay) { sim_delay_ = delay; }
  void setPositionNoise(double noise_rad) { position_noise_rad_ = noise_rad; }
  void enablePhysicsSimulation(bool enable) { physics_enabled_ = enable; }

private:
  void physicsLoop();
  double applyPhysics(uint8_t servo_id, double dt);

  bool initialized_ = false;
  bool emergency_stopped_ = false;
  std::string last_error_;
  std::string device_port_;
  uint8_t servo_count_ = 0;

  std::vector<SimulatedServo> servos_;

  // Simulation parameters
  std::chrono::milliseconds sim_delay_{10};
  double position_noise_rad_ = 0.0;
  bool physics_enabled_ = true;

  // Physics thread
  std::thread physics_thread_;
  std::atomic<bool> running_{false};
  std::mutex servos_mutex_;
  std::condition_variable cv_;
  
  static constexpr double PHYSICS_DT = 0.001;  // 1ms physics step
};

} // namespace dog_hardware_cpp

#endif // DOG_HARDWARE_CPP__SIMULATION_DRIVER_HPP_
