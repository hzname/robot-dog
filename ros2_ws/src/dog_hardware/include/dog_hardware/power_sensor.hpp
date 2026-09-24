// Current / voltage sensing on the servo supply rail (INA226 or INA219 over
// Linux i2c-dev) and the protection logic that uses it.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dog_hardware
{

struct PowerReading
{
  double voltage{0.0};  // bus voltage [V]
  double current{0.0};  // [A], positive = drawn by the servos
};

class PowerSensor
{
public:
  virtual ~PowerSensor() = default;
  virtual bool read(PowerReading & out) = 0;
  virtual std::string describe() const = 0;
};

/// Probes the bus and returns a configured sensor, or nullptr when none of
/// `addresses` answers like an INA226 / INA219. `chip` = "auto", "ina226" or
/// "ina219". `found` describes what was probed (for the log).
std::unique_ptr<PowerSensor> probePowerSensor(const std::string & device,
  const std::vector<int> & addresses, const std::string & chip, double shunt_ohm,
  std::string & found);

/// Register-level helpers, exposed for tests.
namespace ina
{
constexpr uint16_t kIna226Manufacturer = 0x5449;  // "TI"
constexpr uint16_t kIna226Die = 0x2260;
/// INA226: 64-sample averaging, 1.1 ms conversions, continuous (~140 ms / result).
constexpr uint16_t kIna226Config = 0x4727;
/// INA219: 16 V range, +-320 mV shunt, 128-sample averaging, continuous.
constexpr uint16_t kIna219Config = 0x1FFF;
double ina226ShuntVolts(uint16_t raw);
double ina226BusVolts(uint16_t raw);
double ina219ShuntVolts(uint16_t raw);
double ina219BusVolts(uint16_t raw);
/// Plausibility of an INA219 config register read back after writing it.
bool looksLikeIna219(uint16_t config_readback);
}  // namespace ina

/// Protection on top of the readings: a smoothed current and time windows so
/// single spikes (servo PWM bursts) never trigger anything.
struct PowerGuardParams
{
  double overcurrent_a{5.0};     // sustained total servo current [A]
  double overcurrent_time{0.5};  // [s]
  double undervoltage_v{5.0};    // servo rail [V]; 0 disables
  double undervoltage_time{0.3}; // [s]
  double filter_tau{0.1};        // current low-pass time constant [s]
};

class PowerGuard
{
public:
  enum class Event { NONE, OVERCURRENT, UNDERVOLTAGE };

  explicit PowerGuard(PowerGuardParams p) : p_(p) {}
  /// Feed one reading taken at `now` [s]; returns an event once when a
  /// condition has held for its whole window (re-armed after it clears).
  Event update(const PowerReading & r, double now);
  double filteredCurrent() const {return current_;}

private:
  PowerGuardParams p_;
  double current_{0.0};
  double last_{-1.0};
  double over_since_{-1.0};
  double under_since_{-1.0};
  bool over_fired_{false};
  bool under_fired_{false};
};

}  // namespace dog_hardware
