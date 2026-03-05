/**
 * @file pca9685_driver.hpp
 * @brief PCA9685 16-channel PWM driver via I2C
 * 
 * Controls up to 16 servos using the PCA9685 PWM controller.
 * Default I2C address: 0x40
 * 
 * Features:
 * - 12-bit PWM resolution (4096 steps)
 * - 50Hz default frequency (standard servo)
 * - Configurable pulse width: 0.5ms - 2.5ms
 */

#ifndef DOG_HARDWARE_CPP__PCA9685_DRIVER_HPP_
#define DOG_HARDWARE_CPP__PCA9685_DRIVER_HPP_

#include "dog_hardware_cpp/servo_interface.hpp"

#include <fstream>
#include <array>
#include <math>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

namespace dog_hardware_cpp
{

// PCA9685 registers
constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_SUBADR1 = 0x02;
constexpr uint8_t PCA9685_SUBADR2 = 0x03;
constexpr uint8_t PCA9685_SUBADR3 = 0x04;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;
constexpr uint8_t PCA9685_LED0_ON_H = 0x07;
constexpr uint8_t PCA9685_LED0_OFF_L = 0x08;
constexpr uint8_t PCA9685_LED0_OFF_H = 0x09;
constexpr uint8_t PCA9685_ALL_LED_ON_L = 0xFA;
constexpr uint8_t PCA9685_ALL_LED_ON_H = 0xFB;
constexpr uint8_t PCA9685_ALL_LED_OFF_L = 0xFC;
constexpr uint8_t PCA9685_ALL_LED_OFF_H = 0xFD;

// Mode1 bits
constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE1_EXTCLK = 0x40;
constexpr uint8_t MODE1_AI = 0x20;
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_SUB1 = 0x08;
constexpr uint8_t MODE1_SUB2 = 0x04;
constexpr uint8_t MODE1_SUB3 = 0x02;
constexpr uint8_t MODE1_ALLCALL = 0x01;

// Mode2 bits
constexpr uint8_t MODE2_INVRT = 0x10;
constexpr uint8_t MODE2_OCH = 0x08;
constexpr uint8_t MODE2_OUTDRV = 0x04;
constexpr uint8_t MODE2_OUTNE1 = 0x02;
constexpr uint8_t MODE2_OUTNE0 = 0x01;

class PCA9685Driver : public ServoInterface
{
public:
  PCA9685Driver();
  ~PCA9685Driver() override;

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

  std::string getInterfaceName() const override { return "PCA9685_I2C"; }
  std::string getLastError() const override { return last_error_; }

  // PCA9685 specific methods
  bool setPWMFrequency(double freq_hz);
  bool setPWM(uint8_t channel, uint16_t on, uint16_t off);
  bool setAllPWM(uint16_t on, uint16_t off);
  bool reset();

private:
  bool writeByte(uint8_t reg, uint8_t data);
  bool readByte(uint8_t reg, uint8_t& data);
  bool setPin(uint8_t channel, uint16_t value);
  uint16_t angleToPWM(uint8_t servo_id, double angle_rad);

  int i2c_fd_ = -1;
  uint8_t i2c_addr_ = 0x40;
  bool initialized_ = false;
  bool emergency_stopped_ = false;
  std::string last_error_;
  std::string device_port_;
  uint8_t servo_count_ = 0;

  static constexpr double OSC_CLOCK = 25000000.0;  // 25MHz internal oscillator
  static constexpr uint8_t MAX_CHANNELS = 16;

  std::array<ServoConfig, MAX_CHANNELS> servo_configs_;
  std::array<ServoState, MAX_CHANNELS> servo_states_;
  std::array<bool, MAX_CHANNELS> servos_enabled_;
  
  double pwm_frequency_ = 50.0;
  double pulse_width_ms_per_count_ = 0.0;
};

} // namespace dog_hardware_cpp

#endif // DOG_HARDWARE_CPP__PCA9685_DRIVER_HPP_
