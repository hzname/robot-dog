// Tests for runSelftest: the polling-rate check of the servo speed tool.
// Deterministic fake clocks and a fake bus (D-08, Claude's Discretion); no
// sleeps and no wall time.
#include <gtest/gtest.h>

#include <string>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"
#include "dog_bench/selftest.hpp"

using namespace dog_bench;

namespace
{

/// Deterministic fake time: every I2C transaction costs `tx_cost`, every pause
/// costs what was asked plus `pause_overhead`, and every `extend_every`-th
/// pause is lengthened by `extend_s` (a scheduling stall).
struct Sim
{
  double t{0.0};
  double tx_cost{0.00025};
  double pause_overhead{0.00005};
  double extend_every{0.0};
  double extend_s{0.0};
  int pauses{0};

  double now() const {return t;}
  void advance(double by) {t += by;}
  void pause(double seconds)
  {
    ++pauses;
    t += seconds + pause_overhead;
    if (extend_every > 0.0 && pauses % static_cast<int>(extend_every) == 0) {t += extend_s;}
  }
};

/// Freshest Sim + FakeI2cBus + Ina219Fast on 0x41 with the registers a
/// configured sensor shows: shunt 300 (30 mA on 0.1 Ohm), bus 6.0 V.
struct Rig
{
  Sim sim;
  FakeI2cBus bus;
  Ina219Fast sensor{bus, 0x41};

  Rig()
  {
    bus.addDevice(0x41);
    bus.setReg16(0x41, ina::kRegConfig, 0x399F);
    bus.setReg16(0x41, ina::kRegShunt, 300);
    bus.setReg16(0x41, ina::kRegBus, 1500 << 3);
  }

  bool configure()
  {
    bus.setTransactionHook([this]() {sim.advance(sim.tx_cost);});
    return sensor.configure(ina::kIna219Fast320mv);
  }

  SelftestResult run(int samples)
  {
    return runSelftest(sensor, samples,
      [this]() {return sim.now();}, [this](double s) {sim.pause(s);});
  }
};

}  // namespace

TEST(Selftest, AcceptsFastBus)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  const auto r = rig.run(1000);
  EXPECT_TRUE(r.ok) << r.reason;
  EXPECT_TRUE(r.reason.empty());
  EXPECT_NEAR(r.median_ms, 1.05, 1e-3);
  EXPECT_NEAR(r.p99_ms, 1.05, 1e-3);
  EXPECT_NEAR(r.max_ms, 1.05, 1e-3);
  EXPECT_EQ(r.errors, 0);
  EXPECT_EQ(r.samples, 1000);
  EXPECT_NEAR(r.bus_volts, 6.0, 1e-12);
  EXPECT_EQ(r.shunt_raw_min, 300);
  EXPECT_EQ(r.shunt_raw_max, 300);
}

TEST(Selftest, RejectsSlowMedian)
{
  Rig rig;
  rig.sim.tx_cost = 0.0018;  // 1.8 ms per transaction
  rig.sim.pause_overhead = 0.0;
  ASSERT_TRUE(rig.configure());
  const auto r = rig.run(1000);
  EXPECT_FALSE(r.ok);
  EXPECT_NEAR(r.median_ms, 1.8, 1e-3);
  EXPECT_NEAR(r.p99_ms, 3.6, 1e-3);  // below the p99 cutoff
  EXPECT_NE(r.reason.find("median"), std::string::npos);
  EXPECT_EQ(r.reason.find("p99"), std::string::npos);
}

TEST(Selftest, RejectsSlowP99)
{
  Rig rig;
  rig.sim.extend_every = 25.0;  // every 25th pause is 8 ms too long
  rig.sim.extend_s = 0.008;
  ASSERT_TRUE(rig.configure());
  const auto r = rig.run(1000);
  EXPECT_FALSE(r.ok);
  EXPECT_NEAR(r.median_ms, 1.05, 1e-3);
  EXPECT_NEAR(r.p99_ms, 9.05, 1e-3);
  EXPECT_NE(r.reason.find("p99"), std::string::npos);
  EXPECT_EQ(r.reason.find("median"), std::string::npos);
}

TEST(Selftest, CountsConsecutiveErrors)
{
  {  // two single failures far apart: counted, no abort
    Rig rig;
    ASSERT_TRUE(rig.configure());
    const int first = rig.bus.transactions();
    rig.bus.failRange(first + 10, 1);
    rig.bus.failRange(first + 40, 1);
    const auto r = rig.run(1000);
    EXPECT_TRUE(r.ok) << r.reason;
    EXPECT_EQ(r.errors, 2);
    EXPECT_EQ(r.samples, 1000);
  }
  {  // two consecutive failures, then success: still the boundary (no abort)
    Rig rig;
    ASSERT_TRUE(rig.configure());
    const int first = rig.bus.transactions();
    rig.bus.failRange(first + 10, 2);
    const auto r = rig.run(1000);
    EXPECT_TRUE(r.ok) << r.reason;
    EXPECT_EQ(r.errors, 2);
    EXPECT_EQ(r.samples, 1000);
  }
  {  // three consecutive failures: the run stops
    Rig rig;
    ASSERT_TRUE(rig.configure());
    const int first = rig.bus.transactions();
    rig.bus.failRange(first + 10, 3);
    const auto r = rig.run(1000);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.errors, 3);
    EXPECT_LT(r.samples, 1000);
    EXPECT_NE(r.reason.find("consecutive"), std::string::npos);
  }
}

TEST(Selftest, RejectsTooFewSamples)
{
  Rig rig;
  const int before = rig.bus.transactions();
  const auto r = rig.run(10);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("samples"), std::string::npos);
  EXPECT_EQ(rig.bus.transactions(), before);  // the bus is not touched
}
