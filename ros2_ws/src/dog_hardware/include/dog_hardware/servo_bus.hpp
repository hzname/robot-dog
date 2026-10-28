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
///
/// Failures can be injected: a real I2C write can fail, and a driver that
/// swallows that failure leaves the servos frozen at the last pulse (SAF-01).
class MockBus : public ServoBus
{
public:
  bool setPulseUs(int channel, double us) override;
  bool disable(int channel) override;
  bool disableAll() override;
  std::string describe() const override {return "mock";}

  double pulse(int channel) const {return pulses_.at(channel);}
  int writes() const {return writes_;}
  /// Calls that reached the bus for a channel, failed ones included.
  int attempts(int channel) const {return attempts_.at(channel);}

  /// The next `n` setPulseUs calls fail, whatever the channel.
  void failNext(int n) {fail_next_ += n;}
  /// The next `n` calls on `channel` fail.
  void failChannel(int channel, int n) {fail_channel_.at(channel) += n;}
  /// The next `n` disableAll calls fail.
  void failDisableAll(int n) {fail_disable_all_ += n;}
  /// The next `n` single-channel disable calls on `channel` fail.
  void failDisable(int channel, int n) {fail_disable_.at(channel) += n;}
  /// Single-channel disable calls that reached the bus.
  int disableAttempts(int channel) const {return disable_attempts_.at(channel);}
  /// The wire is fixed: every pending injected failure is dropped.
  void clearFailures();

private:
  std::array<double, kPwmChannels> pulses_{};
  std::array<int, kPwmChannels> attempts_{};
  std::array<int, kPwmChannels> fail_channel_{};
  std::array<int, kPwmChannels> fail_disable_{};
  std::array<int, kPwmChannels> disable_attempts_{};
  int fail_next_{0};
  int fail_disable_all_{0};
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

  /// Opens the bus, configures the PWM frequency and turns every output off:
  /// the robot is always started by hand, so a pulse inherited from a previous
  /// process is more dangerous than a limp leg (D-06, SAF-12). Returns false and
  /// fills `error` on failure, including an unconfirmed `disableAll`.
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
