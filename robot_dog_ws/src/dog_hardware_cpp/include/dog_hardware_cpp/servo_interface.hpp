/**
 * @file servo_interface.hpp
 * @brief Hardware abstraction layer for servo drivers
 * 
 * Provides common interface for different servo communication protocols:
 * - I2C (PCA9685 PWM controller)
 * - UART (Dynamixel protocol)
 * - Simulation mode (no hardware)
 */

#ifndef DOG_HARDWARE_CPP__SERVO_INTERFACE_HPP_
#define DOG_HARDWARE_CPP__SERVO_INTERFACE_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <optional>

namespace dog_hardware_cpp
{

/**
 * @brief Servo configuration structure
 */
struct ServoConfig
{
  uint8_t id;                           ///< Servo ID (0-15 for PCA9685, 1-253 for Dynamixel)
  std::string name;                     ///< Joint name
  double min_angle_rad = -M_PI;         ///< Minimum angle in radians
  double max_angle_rad = M_PI;          ///< Maximum angle in radians
  double neutral_angle_rad = 0.0;       ///< Neutral position in radians
  double max_velocity_rad_s = 10.0;     ///< Maximum velocity rad/s
  double max_torque = 1.0;              ///< Maximum torque (0.0 - 1.0)
  bool inverted = false;                ///< Invert direction
  
  // PWM specific (PCA9685)
  double pwm_min_ms = 0.5;              ///< Minimum pulse width in ms
  double pwm_max_ms = 2.5;              ///< Maximum pulse width in ms
  double pwm_neutral_ms = 1.5;          ///< Neutral pulse width in ms
  double pwm_freq_hz = 50.0;            ///< PWM frequency
};

/**
 * @brief Servo state structure
 */
struct ServoState
{
  double position_rad = 0.0;            ///< Current position in radians
  double velocity_rad_s = 0.0;          ///< Current velocity rad/s
  double effort = 0.0;                  ///< Current effort/torque
  double temperature = 0.0;             ///< Temperature in Celsius (if available)
  bool is_moving = false;               ///< True if servo is moving
  bool is_enabled = false;              ///< True if servo is enabled
  uint16_t error_code = 0;              ///< Error code from hardware
};

/**
 * @brief Base class for servo hardware interfaces
 * 
 * All concrete implementations (PCA9685, Dynamixel, Simulation) must inherit from this.
 */
class ServoInterface
{
public:
  virtual ~ServoInterface() = default;

  /**
   * @brief Initialize the hardware interface
   * @param device_port Device path (e.g., "/dev/i2c-1" or "/dev/ttyUSB0")
   * @param servo_count Number of servos to control
   * @return true on success
   */
  virtual bool initialize(const std::string& device_port, uint8_t servo_count) = 0;

  /**
   * @brief Shutdown the hardware interface
   */
  virtual void shutdown() = 0;

  /**
   * @brief Check if interface is initialized
   */
  virtual bool isInitialized() const = 0;

  /**
   * @brief Configure a specific servo
   * @param config Servo configuration
   * @return true on success
   */
  virtual bool configureServo(const ServoConfig& config) = 0;

  /**
   * @brief Enable/disable a servo
   * @param servo_id Servo ID
   * @param enable true to enable, false to disable
   * @return true on success
   */
  virtual bool setServoEnabled(uint8_t servo_id, bool enable) = 0;

  /**
   * @brief Set target position for a servo
   * @param servo_id Servo ID
   * @param position_rad Target position in radians
   * @return true on success
   */
  virtual bool setPosition(uint8_t servo_id, double position_rad) = 0;

  /**
   * @brief Set target position with velocity limit
   * @param servo_id Servo ID
   * @param position_rad Target position in radians
   * @param velocity_rad_s Maximum velocity rad/s
   * @return true on success
   */
  virtual bool setPositionWithVelocity(uint8_t servo_id, double position_rad, double velocity_rad_s) = 0;

  /**
   * @brief Set torque limit for a servo
   * @param servo_id Servo ID
   * @param torque 0.0 to 1.0
   * @return true on success
   */
  virtual bool setTorqueLimit(uint8_t servo_id, double torque) = 0;

  /**
   * @brief Get current state of a servo
   * @param servo_id Servo ID
   * @return ServoState if available, nullopt otherwise
   */
  virtual std::optional<ServoState> getState(uint8_t servo_id) = 0;

  /**
   * @brief Get states of all servos
   * @return Vector of servo states
   */
  virtual std::vector<ServoState> getAllStates() = 0;

  /**
   * @brief Emergency stop - immediately disable all servos
   */
  virtual void emergencyStop() = 0;

  /**
   * @brief Reset emergency stop state
   */
  virtual void resetEmergencyStop() = 0;

  /**
   * @brief Check if emergency stop is active
   */
  virtual bool isEmergencyStopped() const = 0;

  /**
   * @brief Get the name of the interface implementation
   */
  virtual std::string getInterfaceName() const = 0;

  /**
   * @brief Get last error message
   */
  virtual std::string getLastError() const = 0;
};

} // namespace dog_hardware_cpp

#endif // DOG_HARDWARE_CPP__SERVO_INTERFACE_HPP_
