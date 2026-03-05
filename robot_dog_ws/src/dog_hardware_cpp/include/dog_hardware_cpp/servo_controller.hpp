/**
 * @file servo_controller.hpp
 * @brief High-level servo controller for robot dog
 * 
 * Manages 12 servos (3 per leg × 4 legs) with:
 * - Safety limits (position, velocity, torque)
 * - Rate limiting (max delta per cycle)
 * - Emergency stop handling
 * - Watchdog timer
 * - Smooth interpolation
 */

#ifndef DOG_HARDWARE_CPP__SERVO_CONTROLLER_HPP_
#define DOG_HARDWARE_CPP__SERVO_CONTROLLER_HPP_

#include "dog_hardware_cpp/servo_interface.hpp"
#include "dog_hardware_cpp/pca9685_driver.hpp"
#include "dog_hardware_cpp/simulation_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <array>
#include <functional>
#include <chrono>

namespace dog_hardware_cpp
{

/**
 * @brief Controller configuration
 */
struct ControllerConfig
{
  // Hardware settings
  std::string bus_type = "simulation";     ///< "i2c", "uart", "simulation"
  std::string device_port = "";            ///< Device path
  uint8_t servo_count = 12;                ///< Number of servos
  
  // Safety limits
  double max_position_delta_rad = 0.15;    ///< Max position change per cycle
  double max_velocity_rad_s = 10.0;        ///< Max velocity
  double max_torque = 1.0;                 ///< Max torque (0-1)
  
  // Watchdog
  double watchdog_timeout_s = 0.5;         ///< Command timeout before emergency stop
  bool enable_watchdog = true;             ///< Enable watchdog timer
  
  // Update rate
  double control_rate_hz = 100.0;          ///< Control loop frequency
  
  // Interpolation
  bool enable_interpolation = true;        ///< Smooth motion between commands
  double interpolation_time_s = 0.05;      ///< Time to reach target
};

/**
 * @brief Joint limits for each servo
 */
struct JointLimits
{
  std::string name;
  double min_position_rad = -M_PI;
  double max_position_rad = M_PI;
  double max_velocity_rad_s = 10.0;
  double max_acceleration_rad_s2 = 50.0;
  double neutral_position_rad = 0.0;
};

/**
 * @brief Servo controller managing 12 joints
 */
class ServoController
{
public:
  explicit ServoController(rclcpp::Logger logger);
  ~ServoController();

  /**
   * @brief Initialize controller with configuration
   */
  bool initialize(const ControllerConfig& config);

  /**
   * @brief Shutdown controller
   */
  void shutdown();

  /**
   * @brief Configure joint limits
   */
  bool configureJointLimits(uint8_t servo_id, const JointLimits& limits);

  /**
   * @brief Set target positions for all joints
   * @param positions Target positions in radians (size must be 12)
   * @return true if all positions are valid
   */
  bool setTargetPositions(const std::array<double, 12>& positions);

  /**
   * @brief Set target position for single joint
   */
  bool setTargetPosition(uint8_t servo_id, double position_rad);

  /**
   * @brief Enable/disable all servos
   */
  bool setAllEnabled(bool enable);

  /**
   * @brief Enable/disable single servo
   */
  bool setServoEnabled(uint8_t servo_id, bool enable);

  /**
   * @brief Update control loop - call at control_rate_hz
   */
  void update();

  /**
   * @brief Emergency stop - immediately stops all servos
   */
  void emergencyStop();

  /**
   * @brief Reset emergency stop
   */
  void resetEmergencyStop();

  /**
   * @brief Check if emergency stop is active
   */
  bool isEmergencyStopped() const;

  /**
   * @brief Get current positions
   */
  std::array<double, 12> getCurrentPositions() const;

  /**
   * @brief Get target positions
   */
  std::array<double, 12> getTargetPositions() const;

  /**
   * @brief Get joint states
   */
  std::vector<ServoState> getJointStates() const;

  /**
   * @brief Check if watchdog triggered
   */
  bool isWatchdogTriggered() const;

  /**
   * @brief Reset watchdog (call when new commands arrive)
   */
  void resetWatchdog();

  /**
   * @brief Get servo interface (for direct access)
   */
  std::shared_ptr<ServoInterface> getHardwareInterface() const;

  /**
   * @brief Get last error message
   */
  std::string getLastError() const { return last_error_; }

private:
  bool applySafetyLimits(uint8_t servo_id, double& position);
  bool applyRateLimiting(uint8_t servo_id, double& position);
  void interpolatePositions();
  void checkWatchdog();
  void sendToHardware();

  rclcpp::Logger logger_;
  ControllerConfig config_;
  std::string last_error_;

  std::shared_ptr<ServoInterface> hardware_;
  std::array<JointLimits, 12> joint_limits_;
  std::array<ServoConfig, 12> servo_configs_;

  // State
  std::array<double, 12> target_positions_;
  std::array<double, 12> smoothed_positions_;
  std::array<double, 12> current_positions_;
  std::array<double, 12> previous_positions_;

  // Timing
  rclcpp::Time last_command_time_;
  rclcpp::Time last_update_time_;
  bool watchdog_triggered_ = false;
  bool emergency_stopped_ = false;
  bool initialized_ = false;

  // Default joint names for quadruped
  static constexpr std::array<const char*, 12> DEFAULT_JOINT_NAMES = {
    "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
    "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
    "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint"
  };
};

} // namespace dog_hardware_cpp

#endif // DOG_HARDWARE_CPP__SERVO_CONTROLLER_HPP_
