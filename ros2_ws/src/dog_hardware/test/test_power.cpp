#include <gtest/gtest.h>

#include "dog_hardware/power_sensor.hpp"

using namespace dog_hardware;

TEST(InaRegisters, Scaling)
{
  // INA226: 2.5 uV/LSB shunt, 1.25 mV/LSB bus.
  EXPECT_NEAR(ina::ina226ShuntVolts(4000), 0.010, 1e-12);      // 10 mV -> 1 A on 10 mOhm
  EXPECT_NEAR(ina::ina226ShuntVolts(0xF060), -0.010, 1e-12);   // two's complement
  EXPECT_NEAR(ina::ina226BusVolts(4800), 6.0, 1e-12);
  // INA219: 10 uV/LSB shunt, bus in bits 15..3 at 4 mV.
  EXPECT_NEAR(ina::ina219ShuntVolts(1000), 0.010, 1e-12);
  EXPECT_NEAR(ina::ina219BusVolts(1500 << 3), 6.0, 1e-12);
  EXPECT_TRUE(ina::looksLikeIna219(ina::kIna219Config));
  EXPECT_FALSE(ina::looksLikeIna219(0x399F));
}

TEST(InaProbe, MissingBusReturnsNull)
{
  std::string found;
  auto s = probePowerSensor("/dev/i2c-does-not-exist", {0x41}, "auto", 0.01, found);
  EXPECT_EQ(s, nullptr);
  EXPECT_NE(found.find("cannot open"), std::string::npos);
}

namespace
{
PowerGuardParams params()
{
  PowerGuardParams p;
  p.overcurrent_a = 5.0;
  p.overcurrent_time = 0.5;
  p.undervoltage_v = 5.0;
  p.undervoltage_time = 0.3;
  p.filter_tau = 0.1;
  return p;
}
}  // namespace

TEST(PowerGuard, SpikesDoNotTrip)
{
  PowerGuard g(params());
  for (int i = 0; i < 200; ++i) {
    // 50 ms bursts of 12 A every 0.5 s on a 2 A walking load.
    const double t = i * 0.01;
    const double amps = (i % 50) < 5 ? 12.0 : 2.0;
    EXPECT_EQ(g.update({6.0, amps}, t), PowerGuard::Event::NONE) << t;
  }
}

TEST(PowerGuard, SustainedStallTripsOnce)
{
  PowerGuard g(params());
  int events = 0;
  double tripped_at = -1;
  for (int i = 0; i < 200; ++i) {
    const auto ev = g.update({6.0, 7.0}, i * 0.01);
    if (ev == PowerGuard::Event::OVERCURRENT) {++events; tripped_at = i * 0.01;}
  }
  EXPECT_EQ(events, 1);
  EXPECT_GE(tripped_at, 0.5);  // first sample seeds the filter
  EXPECT_LT(tripped_at, 0.9);  // filter delay + window
  // Clears, then can trip again.
  for (int i = 200; i < 300; ++i) {g.update({6.0, 1.0}, i * 0.01);}
  int again = 0;
  for (int i = 300; i < 500; ++i) {
    if (g.update({6.0, 7.0}, i * 0.01) == PowerGuard::Event::OVERCURRENT) {++again;}
  }
  EXPECT_EQ(again, 1);
}

TEST(PowerGuard, UndervoltageNeedsItsWindowAndIgnoresPowerOff)
{
  PowerGuard g(params());
  EXPECT_EQ(g.update({4.6, 1.0}, 0.0), PowerGuard::Event::NONE);
  EXPECT_EQ(g.update({4.6, 1.0}, 0.2), PowerGuard::Event::NONE);
  EXPECT_EQ(g.update({4.6, 1.0}, 0.31), PowerGuard::Event::UNDERVOLTAGE);
  EXPECT_EQ(g.update({4.6, 1.0}, 0.5), PowerGuard::Event::NONE);  // once
  PowerGuard off(params());
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(off.update({0.0, 0.0}, i * 0.01), PowerGuard::Event::NONE);  // rail switched off
  }
}

// --- SAF-07: a lost rail latches, a stalled leg does not ----------------------

TEST(PowerGuard, RailLossLatchesUntilSomebodyClearsIt)
{
  PowerGuard g(params());
  EXPECT_EQ(g.update({4.6, 1.0}, 0.0), PowerGuard::Event::NONE);
  EXPECT_EQ(g.update({4.6, 1.0}, 0.31), PowerGuard::Event::UNDERVOLTAGE);
  EXPECT_TRUE(g.latched()) << "a collapsed rail must latch";
  EXPECT_NEAR(g.latchedVoltage(), 4.6, 1e-9);

  // The rail comes back: the protection does not re-arm by itself.
  for (int i = 0; i < 200; ++i) {
    EXPECT_EQ(g.update({6.0, 1.0}, 0.4 + i * 0.01), PowerGuard::Event::NONE);
  }
  EXPECT_TRUE(g.latched()) << "a robot must not re-arm on a supply that keeps sagging";

  g.clear();   // a human looked at the supply
  EXPECT_FALSE(g.latched());
  EXPECT_EQ(g.update({4.6, 1.0}, 3.0), PowerGuard::Event::NONE);
  EXPECT_EQ(g.update({4.6, 1.0}, 3.4), PowerGuard::Event::UNDERVOLTAGE);
}

TEST(PowerGuard, OvercurrentStillRearmsByItself)
{
  PowerGuard g(params());
  for (int i = 0; i <= 45; ++i) {EXPECT_EQ(g.update({6.0, 7.0}, i * 0.01), PowerGuard::Event::NONE) << i;}
  EXPECT_EQ(g.update({6.0, 7.0}, 0.6), PowerGuard::Event::OVERCURRENT);
  EXPECT_FALSE(g.latched()) << "a stalled leg is a passing event, not a latch";
  for (int i = 0; i <= 100; ++i) {
    EXPECT_EQ(g.update({6.0, 1.0}, 0.7 + i * 0.01), PowerGuard::Event::NONE) << i;
  }
  for (int i = 0; i <= 45; ++i) {
    EXPECT_EQ(g.update({6.0, 7.0}, 1.8 + i * 0.01), PowerGuard::Event::NONE) << i;
  }
  EXPECT_EQ(g.update({6.0, 7.0}, 2.4), PowerGuard::Event::OVERCURRENT) << "it fires again after clearing";
}
