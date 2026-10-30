// Tests for the fail-safes of the servo bench tool: sustained overcurrent,
// ADC saturation, shunt plausibility, consecutive INA errors, PCA write
// errors, tick overrun and the global timeout (D-10). Deterministic: the
// current is fed as raw shunt codes and time as numbers; no clocks, no
// sleeps.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "dog_bench/ina219_fast.hpp"
#include "dog_bench/safety.hpp"

namespace
{
/// Shunt raw code for a current [A] on a shunt [Ohm]: 10 uV per LSB.
uint16_t rawForA(double amps, double shunt_ohm)
{
  return static_cast<uint16_t>(static_cast<int16_t>(std::lround(amps * shunt_ohm / 1e-5)));
}
}  // namespace

TEST(Safety, SpikesDoNotTrip)
{
  using namespace dog_bench;
  SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
  ASSERT_TRUE(g.ok()) << g.error();
  for (int i = 0; i <= 5000; ++i) {
    const double t = i * 0.001;
    double amps = 0.4;                // walking background
    if (i % 500 < 40) {amps = 2.5;}   // 40 ms bursts every 0.5 s
    if (i == 2500) {amps = 2.9;}      // a single 2.9 A sample
    const auto ev = g.update({true, rawForA(amps, 0.1), true, false}, t);
    EXPECT_EQ(ev, SafetyEvent::NONE) << "t=" << t;
  }
  EXPECT_FALSE(g.tripped());
  EXPECT_EQ(g.event(), SafetyEvent::NONE);
}

TEST(Safety, SustainedOvercurrentTripsOnce)
{
  using namespace dog_bench;
  SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
  ASSERT_TRUE(g.ok()) << g.error();
  int events = 0;
  double tripped_at = -1.0;
  for (int i = 0; i <= 500; ++i) {
    const double t = i * 0.001;
    const double amps = (i >= 100) ? 2.5 : 0.4;
    const auto ev = g.update({true, rawForA(amps, 0.1), true, false}, t);
    if (ev == SafetyEvent::OVERCURRENT) {++events; tripped_at = t;}
    else {EXPECT_EQ(ev, SafetyEvent::NONE) << "t=" << t;}
  }
  EXPECT_EQ(events, 1);
  EXPECT_GE(tripped_at, 0.150);
  EXPECT_LE(tripped_at, 0.152);
  EXPECT_TRUE(g.tripped());
  EXPECT_EQ(g.event(), SafetyEvent::OVERCURRENT);
  EXPECT_EQ(std::string(eventName(g.event())), "overcurrent");

  // 49 ms over the threshold does not trip; dropping below resets the window.
  {
    SafetyGuard h(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    int events2 = 0;
    for (int i = 0; i <= 400; ++i) {
      const double t = i * 0.001;
      const double amps = (i >= 100 && i < 149) ? 2.5 : 0.4;  // 49 ms over
      if (h.update({true, rawForA(amps, 0.1), true, false}, t) != SafetyEvent::NONE) {++events2;}
    }
    EXPECT_EQ(events2, 0);
    EXPECT_FALSE(h.tripped());
  }
  // A negative current (two's complement) trips the same way.
  {
    SafetyGuard h(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    int events2 = 0;
    double at = -1.0;
    for (int i = 0; i <= 300; ++i) {
      const double t = i * 0.001;
      const double amps = (i >= 100) ? -2.5 : 0.4;
      if (h.update({true, rawForA(amps, 0.1), true, false}, t) == SafetyEvent::OVERCURRENT) {
        ++events2;
        at = t;
      }
    }
    EXPECT_EQ(events2, 1);
    EXPECT_GE(at, 0.150);
    EXPECT_LE(at, 0.152);
  }
}

TEST(Safety, HardSaturationTripsImmediately)
{
  using namespace dog_bench;
  // 0.1 Ohm, PGA /8 (320 mV): 95% is 304 mV, 30400 counts.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    EXPECT_EQ(g.update({true, 30500, true, false}, 0.0), SafetyEvent::SATURATION);
    EXPECT_TRUE(g.tripped());
  }
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    EXPECT_EQ(g.update({true, 30300, true, false}, 0.0), SafetyEvent::NONE);
    EXPECT_FALSE(g.tripped());
  }
  // 10 mOhm, PGA /2 (80 mV): 95% is 76 mV, 7600 counts.
  {
    SafetyGuard g(SafetyParams{}, 0.01, ina::kIna219Fast80mv);
    ASSERT_TRUE(g.ok()) << g.error();
    EXPECT_EQ(g.update({true, 7500, true, false}, 0.0), SafetyEvent::NONE);
    EXPECT_FALSE(g.tripped());
  }
  {
    SafetyGuard g(SafetyParams{}, 0.01, ina::kIna219Fast80mv);
    ASSERT_TRUE(g.ok()) << g.error();
    EXPECT_EQ(g.update({true, 7700, true, false}, 0.0), SafetyEvent::SATURATION);
  }
}

TEST(Safety, ImplausibleShuntRefuses)
{
  using namespace dog_bench;
  // Helper: a 300 ms plausibility window; every sample carries `low_raw`
  // except one spike sample at `spike_raw` (i == 150).
  const auto run_window = [](SafetyGuard & g, uint16_t low_raw, uint16_t spike_raw,
                              bool & fired_during) {
    fired_during = false;
    for (int i = 0; i < 300; ++i) {
      SafetyReading r;
      r.plausibility_window = true;
      r.shunt_raw = (i == 150) ? spike_raw : low_raw;
      if (g.update(r, i * 0.001) != SafetyEvent::NONE) {fired_during = true;}
    }
  };

  // A too-quiet peak (0.01 A) fires on the first call after the window falls.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    bool during = false;
    run_window(g, 100, 100, during);  // 0.01 A throughout the window
    EXPECT_FALSE(during);
    EXPECT_FALSE(g.plausibilityChecked());
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.300),
      SafetyEvent::IMPLAUSIBLE_SHUNT);
    EXPECT_TRUE(g.plausibilityChecked());
    EXPECT_EQ(std::string(eventName(SafetyEvent::IMPLAUSIBLE_SHUNT)), "implausible_shunt");
  }
  // 0.02 A is below the plausibility floor as well.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    bool during = false;
    run_window(g, rawForA(0.02, 0.1), rawForA(0.02, 0.1), during);
    EXPECT_FALSE(during);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.300),
      SafetyEvent::IMPLAUSIBLE_SHUNT);
  }
  // 3.01 A (a spike shorter than the 50 ms overcurrent window) is above the
  // ceiling and fires the same way.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    bool during = false;
    run_window(g, rawForA(0.4, 0.1), 30100, during);
    EXPECT_FALSE(during);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.300),
      SafetyEvent::IMPLAUSIBLE_SHUNT);
  }
  // A plausible peak (0.5 A) passes, checked exactly once.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    bool during = false;
    run_window(g, rawForA(0.5, 0.1), rawForA(0.5, 0.1), during);
    EXPECT_FALSE(during);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.300), SafetyEvent::NONE);
    EXPECT_TRUE(g.plausibilityChecked());
    EXPECT_FALSE(g.tripped());
    EXPECT_NEAR(g.windowPeakA(), 0.5, 1e-6);
  }
  // A window without a single successful reading peaks at 0: refused too.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    g.update({false, 0, true, true}, 0.0);
    g.update({false, 0, true, true}, 0.001);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.002),
      SafetyEvent::IMPLAUSIBLE_SHUNT);
    EXPECT_TRUE(g.plausibilityChecked());
  }
  // Without any window there is nothing to check.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.0), SafetyEvent::NONE);
    EXPECT_FALSE(g.plausibilityChecked());
  }
}

TEST(Safety, ConsecutiveInaErrorsTrip)
{
  using namespace dog_bench;
  // Single errors and two in a row with a success in between never trip.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    g.update({false, 0, true, false}, 0.0);
    g.update({true, rawForA(0.4, 0.1), true, false}, 0.001);
    g.update({false, 0, true, false}, 0.002);
    g.update({false, 0, true, false}, 0.003);
    g.update({true, rawForA(0.4, 0.1), true, false}, 0.004);
    EXPECT_FALSE(g.tripped());
  }
  // Three in a row trip on the third.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    EXPECT_EQ(g.update({false, 0, true, false}, 0.0), SafetyEvent::NONE);
    EXPECT_EQ(g.update({false, 0, true, false}, 0.001), SafetyEvent::NONE);
    EXPECT_EQ(g.update({false, 0, true, false}, 0.002), SafetyEvent::INA_ERRORS);
    EXPECT_TRUE(g.tripped());
    EXPECT_EQ(std::string(eventName(g.event())), "ina_errors");
  }
  // The raw code of a failed read is ignored: a 30500 there is not saturation.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    EXPECT_EQ(g.update({false, 30500, true, false}, 0.0), SafetyEvent::NONE);
    EXPECT_EQ(g.update({false, 30500, true, false}, 0.001), SafetyEvent::NONE);
    EXPECT_EQ(g.update({false, 30500, true, false}, 0.002), SafetyEvent::INA_ERRORS);
  }
  // An error between over-threshold samples does not clear the 50 ms window.
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    int events = 0;
    double tripped_at = -1.0;
    for (int i = 0; i <= 300; ++i) {
      const double t = i * 0.001;
      SafetyReading r{true, rawForA((i >= 100) ? 2.5 : 0.4, 0.1), true, false};
      if (i == 130) {r.ina_ok = false;}
      const auto ev = g.update(r, t);
      if (ev == SafetyEvent::OVERCURRENT) {++events; tripped_at = t;}
    }
    EXPECT_EQ(events, 1);
    EXPECT_GE(tripped_at, 0.150);
    EXPECT_LE(tripped_at, 0.152);
  }
}

TEST(Safety, PcaErrorTripsImmediately)
{
  using namespace dog_bench;
  SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
  EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), false, false}, 0.0), SafetyEvent::PCA_ERROR);
  EXPECT_TRUE(g.tripped());
  EXPECT_EQ(g.event(), SafetyEvent::PCA_ERROR);
  EXPECT_EQ(std::string(eventName(g.event())), "pca_error");
  // Once.
  EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), false, false}, 0.001), SafetyEvent::NONE);
}

TEST(Safety, TickOverrunAndTimeout)
{
  using namespace dog_bench;
  // 49 ms between calls is fine; 60 ms is a tick overrun (first call exempt).
  {
    SafetyGuard g(SafetyParams{}, 0.1, ina::kIna219Fast320mv);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.0), SafetyEvent::NONE);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.049), SafetyEvent::NONE);
    EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.109),
      SafetyEvent::TICK_OVERRUN);
  }
  // TIMEOUT: now - start >= max_seconds (2.0 s here).
  {
    SafetyParams p;
    p.max_seconds = 2.0;
    SafetyGuard g(p, 0.1, ina::kIna219Fast320mv);
    ASSERT_TRUE(g.ok()) << g.error();
    int events = 0;
    double at = -1.0;
    for (int i = 0; i <= 2200; ++i) {
      const double t = i * 0.001;
      const auto ev = g.update({true, rawForA(0.4, 0.1), true, false}, t);
      if (ev == SafetyEvent::TIMEOUT) {++events; at = t;}
      else {EXPECT_EQ(ev, SafetyEvent::NONE) << "t=" << t;}
    }
    EXPECT_EQ(events, 1);
    EXPECT_GE(at, 1.998);
    EXPECT_LE(at, 2.002);
  }
}

TEST(Safety, InvalidConfigurationFailsClosed)
{
  using namespace dog_bench;
  const SafetyParams p;
  SafetyParams bad = p;

  bad.overcurrent_a = 3.1;  // at/above 95% of the 3.04 A limit on 0.1 Ohm
  EXPECT_NE(validateSafety(bad, 0.1, ina::kIna219Fast320mv).find("overcurrent_a"),
    std::string::npos);
  bad.overcurrent_a = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(validateSafety(bad, 0.1, ina::kIna219Fast320mv).find("overcurrent_a"),
    std::string::npos);
  bad = p;
  bad.overcurrent_a = 7.7;  // at/above 95% of the 7.6 A limit on 10 mOhm
  EXPECT_FALSE(validateSafety(bad, 0.01, ina::kIna219Fast80mv).empty());
  bad.overcurrent_a = 7.5;
  EXPECT_EQ(validateSafety(bad, 0.01, ina::kIna219Fast80mv), "");

  EXPECT_NE(validateSafety(p, 0.0, ina::kIna219Fast320mv).find("shunt_ohm"),
    std::string::npos);
  bad = p;
  bad.max_consecutive_ina_errors = 0;
  EXPECT_NE(validateSafety(bad, 0.1, ina::kIna219Fast320mv).find("max_consecutive_ina_errors"),
    std::string::npos);
  bad = p;
  bad.hard_fraction = 1.0;
  EXPECT_NE(validateSafety(bad, 0.1, ina::kIna219Fast320mv).find("hard_fraction"),
    std::string::npos);
  EXPECT_EQ(validateSafety(p, 0.1, ina::kIna219Fast320mv), "");

  // A guard with a broken configuration lets go at once: fail closed.
  SafetyParams broken = p;
  broken.overcurrent_a = 3.1;
  SafetyGuard g(broken, 0.1, ina::kIna219Fast320mv);
  EXPECT_FALSE(g.ok());
  EXPECT_FALSE(g.error().empty());
  EXPECT_EQ(g.update({true, rawForA(0.4, 0.1), true, false}, 0.0), SafetyEvent::INA_ERRORS);
}

TEST(Safety, BothShuntScales)
{
  using namespace dog_bench;
  const SafetyParams p;
  EXPECT_EQ(validateSafety(p, 0.1, ina::kIna219Fast320mv), "");
  EXPECT_EQ(validateSafety(p, 0.01, ina::kIna219Fast80mv), "");
  // On 10 mOhm: the scale is 0.08 V and 2.0 A is 2000 counts.
  EXPECT_NEAR(ina::fullScaleShuntVolts(ina::kIna219Fast80mv), 0.08, 1e-12);
  EXPECT_EQ(rawForA(2.0, 0.01), 2000);

  // 2.5 A trips at the same time on both scales: 25000 and 2500 counts.
  const double shunt[2] = {0.1, 0.01};
  const uint16_t config[2] = {ina::kIna219Fast320mv, ina::kIna219Fast80mv};
  const uint16_t raw25[2] = {25000, 2500};
  double at[2] = {-1.0, -1.0};
  for (int k = 0; k < 2; ++k) {
    SafetyGuard g(p, shunt[k], config[k]);
    ASSERT_TRUE(g.ok()) << g.error();
    for (int i = 0; i <= 300; ++i) {
      const double t = i * 0.001;
      const uint16_t raw = (i >= 100) ? raw25[k] : rawForA(0.4, shunt[k]);
      if (g.update({true, raw, true, false}, t) == SafetyEvent::OVERCURRENT) {at[k] = t;}
    }
    EXPECT_NEAR(at[k], 0.150, 0.002) << "scale " << k;
  }
  EXPECT_DOUBLE_EQ(at[0], at[1]);

  // currentA() carries the magnitude of the last successful reading.
  SafetyGuard g(p, 0.1, ina::kIna219Fast320mv);
  g.update({true, 25000, true, false}, 0.0);
  EXPECT_NEAR(g.currentA(), 2.5, 1e-6);
}
