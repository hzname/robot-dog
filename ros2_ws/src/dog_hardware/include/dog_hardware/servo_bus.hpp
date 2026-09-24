// PWM servo output abstraction: real PCA9685 over Linux i2c-dev, or a mock.
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace dog_hardware
{

constexpr int kPwmChannels = 16;

class ServoBus
{
public:
  virtual ~ServoBus() = default;
  virtual bool setPulseUs(int channel, double us) = 0;
  virtual bool disable(int channel) = 0;
  virtual bool disableAll() = 0;
  virtual std::string describe() const = 0;
};

/// In-memory bus for tests and PC runs. pulse(ch) == 0 means output off.
class MockBus : public ServoBus
{
public:
  bool setPulseUs(int channel, double us) override;
  bool disable(int channel) override;
  bool disableAll() override;
  std::string describe() const override {return "mock";}

  double pulse(int channel) const {return pulses_.at(channel);}
  int writes() const {return writes_;}

private:
  std::array<double, kPwmChannels> pulses_{};
  int writes_{0};
};

/// NXP PCA9685 16-channel PWM driver on a Linux I2C bus.
class Pca9685Bus : public ServoBus
{
public:
  Pca9685Bus() = default;
  ~Pca9685Bus() override;
  Pca9685Bus(const Pca9685Bus &) = delete;
  Pca9685Bus & operator=(const Pca9685Bus &) = delete;

  /// Opens the bus and configures the PWM frequency. If the chip is already
  /// awake at the right prescaler the outputs are left untouched (a restart
  /// of the driver does not make the servos twitch). Returns false and fills
  /// `error` on failure.
  bool open(const std::string & device, int address, double pwm_hz, double oscillator_hz,
    std::string & error);
  void close();

  bool setPulseUs(int channel, double us) override;
  bool disable(int channel) override;
  bool disableAll() override;
  std::string describe() const override;

  /// PRE_SCALE value for a PWM frequency (datasheet eq. 1).
  static uint8_t prescaleFor(double pwm_hz, double oscillator_hz);
  /// Actual PWM frequency produced by a prescaler.
  static double frequencyFor(uint8_t prescale, double oscillator_hz);
  /// 12-bit OFF tick count for a pulse width at the given frequency.
  static uint16_t ticksFor(double us, double pwm_hz);

private:
  bool writeReg(uint8_t reg, uint8_t value);
  bool readReg(uint8_t reg, uint8_t & value);
  bool writeChannel(int channel, uint16_t on, uint16_t off);

  int fd_{-1};
  int address_{0x40};
  std::string device_;
  double actual_hz_{50.0};
};

}  // namespace dog_hardware
