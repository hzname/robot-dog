#include "dog_bench/selftest.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace dog_bench
{

namespace
{

std::string medianReason(double median_ms)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "median interval %.2f ms exceeds %g ms: the bus is too slow for 1 kHz polling; "
    "try 400 kHz (docs/REVIEW.md item 12) and stop every other I2C master",
    median_ms, kSelftestMaxMedianMs);
  return buf;
}

std::string p99Reason(double p99_ms)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "p99 interval %.2f ms exceeds %g ms: scheduling or bus jitter is too large "
    "for the current cutoff",
    p99_ms, kSelftestMaxP99Ms);
  return buf;
}

std::string consecutiveReason(int tick)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "%d consecutive I2C errors at tick %d (check wiring, address and bus speed; "
    "I2C_TIMEOUT behaviour on the H618 is unknown)",
    kSelftestMaxConsecutiveErrors, tick);
  return buf;
}

std::string joinReasons(const std::vector<std::string> & reasons)
{
  std::string out;
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0) {out += "; ";}
    out += reasons[i];
  }
  return out;
}

}  // namespace

SelftestResult runSelftest(Ina219Fast & ina, int samples, const ClockFn & now, const SleepFn & pause)
{
  SelftestResult result;
  if (samples < kSelftestMinSamples) {
    result.reason = "too few samples (" + std::to_string(samples) +
      ", need at least " + std::to_string(kSelftestMinSamples) + ")";
    return result;
  }

  // The interval between tick starts carries the transaction cost, the pause
  // overrun and any scheduler stall, so the pause is counted from the start of
  // the tick rather than from an absolute grid.
  std::vector<double> intervals;
  intervals.reserve(static_cast<std::size_t>(samples));
  int consecutive = 0;
  bool aborted = false;
  double previous_t0 = 0.0;
  bool have_shunt = false;

  for (int i = 0; i < samples; ++i) {
    const double t0 = now();
    if (i > 0) {intervals.push_back(t0 - previous_t0);}
    previous_t0 = t0;
    ++result.samples;

    uint16_t raw = 0;
    if (!ina.readShunt(raw)) {
      ++result.errors;
      ++consecutive;
      if (consecutive >= kSelftestMaxConsecutiveErrors) {
        aborted = true;
        result.reason = consecutiveReason(i);
        break;
      }
    } else {
      consecutive = 0;
      const int counts = static_cast<int16_t>(raw);
      if (!have_shunt || counts < result.shunt_raw_min) {result.shunt_raw_min = counts;}
      if (!have_shunt || counts > result.shunt_raw_max) {result.shunt_raw_max = counts;}
      have_shunt = true;
    }

    if (i % kSelftestBusEvery == kSelftestBusEvery - 1) {
      uint16_t bus_raw = 0;
      if (!ina.readBus(bus_raw)) {
        ++result.errors;
        ++consecutive;
        if (consecutive >= kSelftestMaxConsecutiveErrors) {
          aborted = true;
          result.reason = consecutiveReason(i);
          break;
        }
      } else {
        consecutive = 0;
        result.bus_volts = ina::busVolts(bus_raw);
      }
    }

    const double remain = t0 + kSelftestPollPeriodS - now();
    if (remain > 0.0) {pause(remain);}
  }

  // Statistics over the intervals collected so far, even after an abort.
  if (!intervals.empty()) {
    std::sort(intervals.begin(), intervals.end());
    const std::size_t n = intervals.size();
    const double median_s = (n % 2 == 1) ?
      intervals[n / 2] : 0.5 * (intervals[n / 2 - 1] + intervals[n / 2]);
    const std::size_t rank = static_cast<std::size_t>(
      std::ceil(0.99 * static_cast<double>(n))) - 1;
    result.median_ms = median_s * 1000.0;
    result.p99_ms = intervals[rank] * 1000.0;
    result.max_ms = intervals.back() * 1000.0;
  }

  std::vector<std::string> reasons;
  if (result.median_ms > kSelftestMaxMedianMs) {reasons.push_back(medianReason(result.median_ms));}
  if (result.p99_ms > kSelftestMaxP99Ms) {reasons.push_back(p99Reason(result.p99_ms));}
  if (aborted) {reasons.push_back(result.reason);}
  result.ok = reasons.empty();
  result.reason = joinReasons(reasons);
  return result;
}

double monotonicSeconds()
{
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

void sleepSeconds(double seconds)
{
  if (seconds <= 0.0) {return;}
  const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
  std::this_thread::sleep_for(interval);
}

}  // namespace dog_bench
