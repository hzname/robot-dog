// Fail-safes of the servo bench tool: sustained overcurrent, ADC saturation,
// shunt plausibility, consecutive INA errors, PCA write errors, tick overrun
// and the global timeout (D-10). ROS-free; validation returns a message
// instead of throwing, and a guard that cannot watch its inputs lets go at
// once (fail closed). No current filter: a spike would be hidden by it.
#pragma once

#include <cstdint>
#include <string>

#include "dog_bench/ina219_fast.hpp"

namespace dog_bench
{

struct SafetyParams
{
  double overcurrent_a{2.0};          // trip current [A] (MG996R stall 2.5 A, docs/HARDWARE.md)
  double overcurrent_time_s{0.05};    // the current must stay above it this long [s]
  double hard_fraction{0.95};         // instant trip at this fraction of the PGA full scale
  double plausibility_min_a{0.03};    // peak of the first stroke pair [A]: at least this
  double plausibility_max_a{3.0};     // ... and at most this (a wrong shunt shows up here)
  int max_consecutive_ina_errors{3};  // let go after this many failed INA reads in a row
  double max_tick_overrun_s{0.05};    // a longer gap between updates lets go [s]
  double max_seconds{600.0};          // global run timeout [s]
};

enum class SafetyEvent {
  NONE,
  OVERCURRENT,
  SATURATION,
  IMPLAUSIBLE_SHUNT,
  INA_ERRORS,
  PCA_ERROR,
  TICK_OVERRUN,
  TIMEOUT
};

/// Lower-case name with underscores (`overcurrent`, `implausible_shunt`, ...)
/// for the `stop_reason` of the measurement session.
const char * eventName(SafetyEvent event);

struct SafetyReading
{
  bool ina_ok{true};                // the shunt read succeeded
  uint16_t shunt_raw{0};            // the shunt register code
  bool pca_ok{true};                // the last PCA write succeeded
  bool plausibility_window{false};  // the first stroke pair is running
};

/// Returns an empty string when OK, otherwise a message naming the field.
/// `overcurrent_a` must stay below `hard_fraction` of the PGA full scale for
/// `shunt_ohm` (below the sensor limit, D-10).
std::string validateSafety(const SafetyParams & params, double shunt_ohm, uint16_t ina_config);

class SafetyGuard
{
public:
  /// Calls validateSafety; an invalid configuration never watches.
  SafetyGuard(const SafetyParams & params, double shunt_ohm, uint16_t ina_config);

  /// Feed one reading taken at `now` [s]. Updates the whole state, then
  /// returns the event by priority (saturation, overcurrent, PCA error, INA
  /// errors, tick overrun, implausible shunt, timeout) once on the call it
  /// fires; later calls return NONE, the first event stays in event().
  SafetyEvent update(const SafetyReading & reading, double now);

  /// false when the configuration was invalid: update() lets go at once.
  bool ok() const {return error_.empty();}
  const std::string & error() const {return error_;}
  bool tripped() const {return tripped_;}
  /// The first event seen; NONE before any.
  SafetyEvent event() const {return event_;}
  /// Magnitude of the last successful reading [A].
  double currentA() const {return current_a_;}
  /// Peak magnitude over the plausibility window [A].
  double windowPeakA() const {return window_peak_a_;}
  /// True once the first window fall has been judged.
  bool plausibilityChecked() const {return plausibility_checked_;}

private:
  SafetyParams params_;
  double shunt_ohm_{1.0};
  uint16_t ina_config_{0};
  std::string error_;

  bool started_{false};
  double start_s_{0.0};
  double last_s_{0.0};
  bool tripped_{false};
  SafetyEvent event_{SafetyEvent::NONE};

  int ina_errors_{0};
  double over_since_{-1.0};
  bool window_seen_{false};
  bool plausibility_checked_{false};
  double window_peak_a_{0.0};
  double current_a_{0.0};
};

}  // namespace dog_bench
