// Polling-rate self test for the servo speed measurement (D-08): polls the
// shunt every kSelftestPollPeriodS and the bus every kSelftestBusEvery-th tick,
// then judges the interval distribution. Clocks and pauses are injected, so
// the gtests are deterministic; PWM is not touched.
#pragma once

#include <functional>
#include <string>

#include "dog_bench/ina219_fast.hpp"

namespace dog_bench
{

/// The planned sample period of the speed measurement: 1 ms (1 kHz).
constexpr double kSelftestPollPeriodS = 0.001;
/// Also read the bus every 8th tick; the next readShunt restores the pointer.
constexpr int kSelftestBusEvery = 8;
/// A median interval above this fails the test: the bus is too slow.
constexpr double kSelftestMaxMedianMs = 1.5;
/// A p99 interval above this fails the test: scheduling or bus jitter.
constexpr double kSelftestMaxP99Ms = 5.0;
/// This many failed transactions in a row abort the run.
constexpr int kSelftestMaxConsecutiveErrors = 3;
/// Fewer samples than this make the p99 meaningless.
constexpr int kSelftestMinSamples = 100;
constexpr int kSelftestDefaultSamples = 2000;
constexpr int kSelftestMaxSamples = 20000;

/// Monotonic seconds.
using ClockFn = std::function<double()>;
/// Sleep for the given number of seconds; a non-positive value does nothing.
using SleepFn = std::function<void(double)>;

struct SelftestResult
{
  double median_ms{0.0};   ///< median interval between tick starts [ms]
  double p99_ms{0.0};      ///< 99th percentile of the intervals [ms]
  double max_ms{0.0};      ///< longest interval [ms]
  int errors{0};           ///< failed transactions over the whole run
  bool ok{false};          ///< verdict: PASS only when reason is empty
  std::string reason;      ///< why it failed, or empty
  int samples{0};          ///< ticks performed
  double bus_volts{0.0};   ///< the last successful bus reading [V]
  int shunt_raw_min{0};    ///< smallest shunt code as int16
  int shunt_raw_max{0};    ///< largest shunt code as int16
};

/// Poll the already-configured sensor `samples` times and judge the intervals:
/// PASS needs median <= 1.5 ms, p99 <= 5 ms and fewer than 3 consecutive
/// transaction errors. Fewer than kSelftestMinSamples returns a reason without
/// touching the bus.
SelftestResult runSelftest(Ina219Fast & ina, int samples, const ClockFn & now, const SleepFn & pause);

/// Seconds on std::chrono::steady_clock (CLOCK_MONOTONIC on Linux).
double monotonicSeconds();
/// Sleep for the given seconds; a non-positive value does nothing.
void sleepSeconds(double seconds);

}  // namespace dog_bench
