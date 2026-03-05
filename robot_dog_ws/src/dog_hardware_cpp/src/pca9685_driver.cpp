/**
 * @file pca9685_driver.cpp
 * @brief PCA9685 I2C driver implementation
 */

#include "dog_hardware_cpp/pca9685_driver.hpp"

#include <chrono>
#include <math>
#include <errno.h>

namespace dog_hardware_cpp
{

PCA9685Driver::PCA9685Driver()
{
  servo_configs_.fill(ServoConfig{});
  servo_states_.fill(ServoState{});
  servos_enabled_.fill(false);
}

PCA9685Driver::~PCA9685Driver()
{
  shutdown();
}

bool PCA9685Driver::initialize(const std::string& device_port, uint8_t servo_count)
{
  if (initialized_)
  {
    last_error_ = "Already initialized";
    return false;
  }

  if (servo_count > MAX_CHANNELS)
  {
    last_error_ = "Too many servos (max " + std::to_string(MAX_CHANNELS) + ")";
    return false;
  }

  device_port_ = device_port;
  servo_count_ = servo_count;

  // Open I2C device
  i2c_fd_ = open(device_port.c_str(), O_RDWR);
  if (i2c_fd_ < 0)
  {
    last_error_ = "Failed to open " + device_port + ": " + std::strerror(errno);
    return false;
  }

  // Set I2C slave address
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0)
  {
    last_error_ = "Failed to set I2C address: " + std::strerror(errno);
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Reset the device
  if (!reset())
  {
    last_error_ = "Failed to reset PCA9685";
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Set default PWM frequency
  if (!setPWMFrequency(pwm_frequency_))
  {
    last_error_ = "Failed to set PWM frequency";
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Initialize all channels to neutral/off
  for (uint8_t i = 0; i < servo_count_; ++i)
  {
    setPWM(i, 0, 0);
    servo_states_[i].position_rad = servo_configs_[i].neutral_angle_rad;
    servo_states_[i].is_enabled = false;
  }

  initialized_ = true;
  return true;
}

void PCA9685Driver::shutdown()
{
  if (!initialized_)
  {
    return;
  }

  // Disable all servos
  emergencyStop();

  // Wait a bit for servos to settle
  usleep(100000);  // 100ms

  if (i2c_fd_ >= 0)
  {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }

  initialized_ = false;
  emergency_stopped_ = false;
}

bool PCA9685Driver::isInitialized() const
{
  return initialized_;
}

bool PCA9685Driver::writeByte(uint8_t reg, uint8_t data)
{
  if (i2c_fd_ < 0) return false;

  uint8_t buffer[2] = {reg, data};
  if (write(i2c_fd_, buffer, 2) != 2)
  {
    last_error_ = "I2C write failed: " + std::string(std::strerror(errno));
    return false;
  }
  return true;
}

bool PCA9685Driver::readByte(uint8_t reg, uint8_t& data)
{
  if (i2c_fd_ < 0) return false;

  if (write(i2c_fd_, &reg, 1) != 1)
  {
    return false;
  }

  if (read(i2c_fd_, &data, 1) != 1)
  {
    last_error_ = "I2C read failed: " + std::string(std::strerror(errno));
    return false;
  }
  return true;
}

bool PCA9685Driver::reset()
{
  // Send software reset
  if (!writeByte(PCA9685_MODE1, MODE1_RESTART))
  {
    return false;
  }
  usleep(1000);  // Wait for restart

  // Read MODE1
  uint8_t mode1;
  if (!readByte(PCA9685_MODE1, mode1))
  {
    return false;
  }

  // Set MODE1: auto-increment, normal mode
  mode1 &= ~MODE1_SLEEP;  // Clear sleep bit
  mode1 |= MODE1_AI;       // Enable auto-increment
  if (!writeByte(PCA9685_MODE1, mode1))
  {
    return false;
  }
  usleep(1000);  // Wait for oscillator

  // Set MODE2: totem pole drive, non-inverted
  if (!writeByte(PCA9685_MODE2, MODE2_OUTDRV))
  {
    return false;
  }

  return true;
}

bool PCA9685Driver::setPWMFrequency(double freq_hz)
{
  if (freq_hz < 24.0 || freq_hz > 1526.0)
  {
    last_error_ = "Frequency out of range (24-1526 Hz)";
    return false;
  }

  pwm_frequency_ = freq_hz;
  pulse_width_ms_per_count_ = 1000.0 / (4096.0 * freq_hz);

  // Calculate prescale value
  double prescaleval = OSC_CLOCK / (4096.0 * freq_hz) - 1.0;
  uint8_t prescale = static_cast<uint8_t>(std::round(prescaleval));

  // Read current MODE1
  uint8_t oldmode;
  if (!readByte(PCA9685_MODE1, oldmode))
  {
    return false;
  }

  // Sleep
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
  if (!writeByte(PCA9685_MODE1, newmode))
  {
    return false;
  }

  // Set prescale
  if (!writeByte(PCA9685_PRESCALE, prescale))
  {
    return false;
  }

  // Restore old mode
  if (!writeByte(PCA9685_MODE1, oldmode))
  {
    return false;
  }

  usleep(1000);

  // Enable auto-increment and restart
  if (!writeByte(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI))
  {
    return false;
  }

  return true;
}

bool PCA9685Driver::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
  if (channel >= MAX_CHANNELS)
  {
    last_error_ = "Invalid channel: " + std::to_string(channel);
    return false;
  }

  uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
  uint8_t data[4] = {
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F)
  };

  if (write(i2c_fd_, &reg, 1) != 1)
  {
    last_error_ = "Failed to write register address";
    return false;
  }

  if (write(i2c_fd_, data, 4) != 4)
  {
    last_error_ = "Failed to write PWM values";
    return false;
  }

  return true;
}

bool PCA9685Driver::setAllPWM(uint16_t on, uint16_t off)
{
  uint8_t data[5] = {
    PCA9685_ALL_LED_ON_L,
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F)
  };

  if (write(i2c_fd_, data, 5) != 5)
  {
    last_error_ = "Failed to write all PWM values";
    return false;
  }

  return true;
}

bool PCA9685Driver::setPin(uint8_t channel, uint16_t value)
{
  if (value >= 4096)
  {
    // Full on
    return setPWM(channel, 4096, 0);
  }
  else if (value == 0)
  {
    // Full off
    return setPWM(channel, 0, 4096);
  }
  else
  {
    return setPWM(channel, 0, value);
  }
}

uint16_t PCA9685Driver::angleToPWM(uint8_t servo_id, double angle_rad)
{
  if (servo_id >= MAX_CHANNELS)
  {
    return 0;
  }

  const auto& config = servo_configs_[servo_id];

  // Clamp to limits
  angle_rad = std::max(config.min_angle_rad, std::min(config.max_angle_rad, angle_rad));

  // Invert if needed
  if (config.inverted)
  {
    angle_rad = -angle_rad;
  }

  // Map angle to pulse width
  double angle_range = config.max_angle_rad - config.min_angle_rad;
  double pulse_range = config.pwm_max_ms - config.pwm_min_ms;
  
  double normalized = (angle_rad - config.min_angle_rad) / angle_range;
  double pulse_ms = config.pwm_min_ms + normalized * pulse_range;

  // Convert to PWM counts
  double period_ms = 1000.0 / pwm_frequency_;
  uint16_t counts = static_cast<uint16_t>((pulse_ms / period_ms) * 4096.0);

  return counts;
}

bool PCA9685Driver::configureServo(const ServoConfig& config)
{
  if (config.id >= MAX_CHANNELS)
  {
    last_error_ = "Invalid servo ID: " + std::to_string(config.id);
    return false;
  }

  servo_configs_[config.id] = config;
  return true;
}

bool PCA9685Driver::setServoEnabled(uint8_t servo_id, bool enable)
{
  if (servo_id >= MAX_CHANNELS)
  {
    last_error_ = "Invalid servo ID";
    return false;
  }

  if (emergency_stopped_ && enable)
  {
    last_error_ = "Cannot enable servo during emergency stop";
    return false;
  }

  servos_enabled_[servo_id] = enable;
  servo_states_[servo_id].is_enabled = enable;

  if (!enable)
  {
    // Set to neutral position when disabling
    setPin(servo_id, 0);  // Full off
  }

  return true;
}

bool PCA9685Driver::setPosition(uint8_t servo_id, double position_rad)
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

  if (!servos_enabled_[servo_id])
  {
    last_error_ = "Servo " + std::to_string(servo_id) + " not enabled";
    return false;
  }

  // Clamp position
  const auto& config = servo_configs_[servo_id];
  position_rad = std::max(config.min_angle_rad, std::min(config.max_angle_rad, position_rad));

  // Convert and set PWM
  uint16_t pwm_value = angleToPWM(servo_id, position_rad);
  if (!setPin(servo_id, pwm_value))
  {
    return false;
  }

  // Update state
  servo_states_[servo_id].position_rad = position_rad;

  return true;
}

bool PCA9685Driver::setPositionWithVelocity(uint8_t servo_id, double position_rad, double velocity_rad_s)
{
  // PCA9685 doesn't support velocity control natively
  // We could implement it with interpolation, but for now just set position
  // The velocity limit will be enforced at the controller level
  (void)velocity_rad_s;
  return setPosition(servo_id, position_rad);
}

bool PCA9685Driver::setTorqueLimit(uint8_t servo_id, double torque)
{
  // PCA9685 doesn't support torque limiting
  // This would require current sensing which the chip doesn't have
  (void)servo_id;
  (void)torque;
  return true;  // Silently succeed
}

std::optional<ServoState> PCA9685Driver::getState(uint8_t servo_id)
{
  if (servo_id >= servo_count_)
  {
    return std::nullopt;
  }
  return servo_states_[servo_id];
}

std::vector<ServoState> PCA9685Driver::getAllStates()
{
  std::vector<ServoState> states;
  states.reserve(servo_count_);
  for (uint8_t i = 0; i < servo_count_; ++i)
  {
    states.push_back(servo_states_[i]);
  }
  return states;
}

void PCA9685Driver::emergencyStop()
{
  emergency_stopped_ = true;

  // Disable all PWM outputs
  for (uint8_t i = 0; i < servo_count_; ++i)
  {
    setPin(i, 0);  // Full off
    servos_enabled_[i] = false;
    servo_states_[i].is_enabled = false;
  }
}

void PCA9685Driver::resetEmergencyStop()
{
  emergency_stopped_ = false;
}

bool PCA9685Driver::isEmergencyStopped() const
{
  return emergency_stopped_;
}

} // namespace dog_hardware_cpp
