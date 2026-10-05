// PWM output abstraction of the servo bench tool: the interface the speed
// measurement session drives, and a recording fake for the gtests. The real
// PCA9685 output is added by a later layer and only ever touches its own
// channel (D-09, D-10).
#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace dog_bench
{

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

}  // namespace dog_bench
