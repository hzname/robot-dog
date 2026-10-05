// PWM output abstraction of the servo bench tool: the interface the speed
// measurement session drives, a recording fake for the gtests, and the real
// PCA9685 output that only ever touches its own channel (D-09, D-10).
#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"

namespace dog_bench
{

/// PCA9685 registers and the 50 Hz frame; the formulas are copied from the
/// driver package on purpose, its headers are never included (D-09).
namespace pca
{
/// The PCA9685 answers at 0x40 on the I2C bus.
constexpr int kAddress = 0x40;
static_assert(kAddress == ina::kPca9685Address, "the PCA9685 address moved");
constexpr int kPwmChannels = 16;
constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kLed0OnL = 0x06;
constexpr uint8_t kPrescale = 0xFE;
constexpr uint8_t kAllLedOnL = 0xFA;
constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode1AutoInc = 0x20;  // without it a 5-byte write corrupts registers
constexpr uint8_t kFullOffBit = 0x10;    // in LEDn_OFF_H
/// Prescale for 50 Hz at the nominal 25 MHz oscillator (the real one drifts
/// up to 5%; the error folds into the protractor us_per_deg term).
constexpr uint8_t kPrescale50Hz = 121;
constexpr double kOscillatorHz = 25e6;
/// release() attempts before giving up.
constexpr int kReleaseAttempts = 3;
/// ALL_LED_OFF: one write takes every channel output down (the signal handler
/// of the measurement session reuses this buffer).
constexpr std::array<uint8_t, 5> kAllLedOffBytes = {kAllLedOnL, 0, 0, 0, kFullOffBit};
/// Absolute output limits [us], as in pca9685_probe.
constexpr double kMinPulseUs = 500.0;
constexpr double kMaxPulseUs = 2500.0;

/// PWM ticks for a pulse width [us] at the 50 Hz frame, clamped 0..4095.
uint16_t ticksFor(double us);
}  // namespace pca

/// One PWM output for one servo channel. preflight() must succeed before any
/// pulse is written; release() takes all outputs down and is repeatable.
class PwmOut
{
public:
  virtual ~PwmOut() = default;

  /// Check that the chip can be driven. Reads only; false on any doubt with
  /// the reason in error(). A failed object is never armed.
  [[nodiscard]] virtual bool preflight() = 0;
  /// Write one pulse width [us]. False and nothing written before a
  /// successful preflight() or outside [pulse_min_us, pulse_max_us].
  [[nodiscard]] virtual bool setPulseUs(double us) = 0;
  /// Take all outputs down; repeatable. False when every attempt failed.
  [[nodiscard]] virtual bool release() = 0;
  /// True while the output holds nothing.
  virtual bool released() const = 0;
  /// Why the last call failed, or empty.
  virtual std::string error() const = 0;
};

/// Recording fake for the gtests: no bus, no clocks. Pulses before a
/// successful preflight() and outside [min_us, max_us] are rejected and not
/// recorded.
class FakePwm : public PwmOut
{
public:
  FakePwm(double min_us, double max_us) : min_us_(min_us), max_us_(max_us) {}

  /// Make every preflight() fail with `reason`.
  void failPreflight(const std::string & reason)
  {
    fail_preflight_ = reason;
    has_fail_preflight_ = true;
  }
  /// Writes from the n-th attempt on fail (1-based): the first n-1 succeed.
  void failWritesFrom(int index) {fail_from_ = index;}

  const std::vector<double> & pulses() const {return pulses_;}
  int releases() const {return releases_;}

  bool preflight() override
  {
    if (has_fail_preflight_) {
      error_ = fail_preflight_;
      armed_ = false;
      return false;
    }
    armed_ = true;
    error_.clear();
    return true;
  }

  bool setPulseUs(double us) override
  {
    if (!armed_) {
      error_ = "setPulseUs before a successful preflight";
      return false;
    }
    if (!std::isfinite(us) || us < min_us_ || us > max_us_) {
      error_ = "pulse " + number(us) + " us is outside the output window";
      return false;
    }
    ++write_index_;
    if (fail_from_ > 0 && write_index_ >= fail_from_) {
      error_ = "injected write failure";
      return false;
    }
    pulses_.push_back(us);
    released_ = false;
    return true;
  }

  bool release() override
  {
    if (!armed_) {return true;}
    ++write_index_;
    if (fail_from_ > 0 && write_index_ >= fail_from_) {
      error_ = "injected write failure";
      return false;
    }
    released_ = true;
    ++releases_;
    return true;
  }

  bool released() const override {return !armed_ || released_;}
  std::string error() const override {return error_;}

private:
  static std::string number(double value)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", value);
    return buf;
  }

  double min_us_;
  double max_us_;
  std::string fail_preflight_;
  std::string error_;
  bool has_fail_preflight_{false};
  bool armed_{false};
  bool released_{true};
  int fail_from_{0};
  int write_index_{0};
  std::vector<double> pulses_;
  int releases_{0};
};

/// The real PCA9685 output of one servo channel. preflight() only reads: the
/// chip must be awake with auto-increment, already at 50 Hz and free of other
/// live channels; the chip is never initialized (D-09). After a successful
/// preflight the destructor always releases all outputs (D-10). The bus must
/// outlive the object; copying is forbidden.
class Pca9685Out : public PwmOut
{
public:
  Pca9685Out(I2cBus & bus, int channel, double pulse_min_us, double pulse_max_us);
  ~Pca9685Out() override;
  Pca9685Out(const Pca9685Out &) = delete;
  Pca9685Out & operator=(const Pca9685Out &) = delete;

  bool preflight() override;
  bool setPulseUs(double us) override;
  bool release() override;
  bool released() const override {return !armed_ || released_;}
  std::string error() const override {return error_;}

private:
  I2cBus * bus_;
  int channel_;
  double pulse_min_us_;
  double pulse_max_us_;
  bool armed_{false};
  bool released_{true};
  std::string error_;
};

}  // namespace dog_bench
