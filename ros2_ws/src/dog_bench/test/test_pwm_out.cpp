// Tests for the real PCA9685 output of the servo bench tool: the destructor
// always releases after a successful preflight, a foreign live channel or a
// chip that is not configured for 50 Hz is refused without a single write,
// and only the servo's own channel registers are ever written (D-09, D-10).
// Deterministic: the FakeI2cBus models the chip registers, no clocks.
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/pwm_out.hpp"

namespace
{
using namespace dog_bench;

/// A healthy PCA9685 in the fake: awake with auto-increment, prescale 121
/// (50 Hz), every channel output off (LEDn_OFF_H full-off bit).
void setupChip(FakeI2cBus & bus)
{
  bus.addDevice(pca::kAddress);
  bus.setReg8(pca::kAddress, pca::kMode1, 0x21);
  bus.setReg8(pca::kAddress, pca::kPrescale, pca::kPrescale50Hz);
  for (int n = 0; n < pca::kPwmChannels; ++n) {
    bus.setReg8(pca::kAddress, static_cast<uint8_t>(0x09 + 4 * n), pca::kFullOffBit);
  }
}
}  // namespace

TEST(PwmOut, DestructorReleasesAllChannels)
{
  using namespace dog_bench;
  FakeI2cBus bus;
  setupChip(bus);
  {
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    ASSERT_TRUE(out.preflight()) << out.error();
    ASSERT_TRUE(out.setPulseUs(1370.0));
    ASSERT_TRUE(out.setPulseUs(1375.0));
    ASSERT_TRUE(out.setPulseUs(1380.0));
    EXPECT_EQ(bus.reg8(pca::kAddress, 0xFD), 0);  // the outputs are live
    EXPECT_EQ(bus.writes(), 3);
  }
  EXPECT_EQ(bus.reg8(pca::kAddress, 0xFA), 0);
  EXPECT_EQ(bus.reg8(pca::kAddress, 0xFB), 0);
  EXPECT_EQ(bus.reg8(pca::kAddress, 0xFC), 0);
  EXPECT_EQ(bus.reg8(pca::kAddress, 0xFD), 0x10);
  EXPECT_EQ(bus.writes(), 3 + 1);  // the pulses plus the release

  // The destructor also releases when the scope exits with an exception.
  {
    const int w0 = bus.writes();
    bool threw = false;
    try {
      Pca9685Out out(bus, 1, 1000.0, 1740.0);
      ASSERT_TRUE(out.preflight()) << out.error();
      ASSERT_TRUE(out.setPulseUs(1370.0));
      throw std::runtime_error("test");
    } catch (const std::runtime_error &) {threw = true;}
    EXPECT_TRUE(threw);
    EXPECT_EQ(bus.writes(), w0 + 1 + 1);  // the pulse plus the release
  }
  EXPECT_EQ(bus.reg8(pca::kAddress, 0xFD), 0x10);

  // ... and when setPulseUs refused an out-of-window value: nothing was
  // written by it, the release still goes out.
  {
    const int w0 = bus.writes();
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    ASSERT_TRUE(out.preflight()) << out.error();
    EXPECT_FALSE(out.setPulseUs(1740.1));
    EXPECT_FALSE(out.setPulseUs(999.9));
    EXPECT_EQ(bus.writes(), w0);
  }
  EXPECT_EQ(bus.writes(), 6 + 1);
}

TEST(PwmOut, RefusesForeignActiveChannel)
{
  using namespace dog_bench;
  FakeI2cBus bus;
  setupChip(bus);
  bus.setReg8(pca::kAddress, 0x09 + 4 * 5, 0x00);  // channel 5 is live
  {
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_NE(out.error().find("channel 5"), std::string::npos) << out.error();
    EXPECT_NE(out.error().find("docker compose stop"), std::string::npos) << out.error();
    EXPECT_FALSE(out.setPulseUs(1370.0));  // a refused output stays silent
  }
  EXPECT_EQ(bus.writes(), 0);  // nothing was written, not even on destruction

  // A live own channel is not a refusal.
  FakeI2cBus own;
  setupChip(own);
  own.setReg8(pca::kAddress, 0x09 + 4 * 1, 0x00);
  {
    Pca9685Out out(own, 1, 1000.0, 1740.0);
    EXPECT_TRUE(out.preflight()) << out.error();
    EXPECT_TRUE(out.setPulseUs(1370.0));
  }
  EXPECT_EQ(own.reg8(pca::kAddress, 0xFD), 0x10);
}

TEST(PwmOut, RefusesSleepingOrWrongPrescale)
{
  using namespace dog_bench;
  // MODE1 sleep bit set.
  {
    FakeI2cBus bus;
    setupChip(bus);
    bus.setReg8(pca::kAddress, pca::kMode1, 0x31);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_NE(out.error().find("pca9685_probe check"), std::string::npos) << out.error();
    EXPECT_EQ(bus.writes(), 0);
  }
  // MODE1 without the auto-increment bit.
  {
    FakeI2cBus bus;
    setupChip(bus);
    bus.setReg8(pca::kAddress, pca::kMode1, 0x01);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_NE(out.error().find("pca9685_probe check"), std::string::npos) << out.error();
    EXPECT_EQ(bus.writes(), 0);
  }
  // Prescale 30: the power-on default, 200 Hz.
  {
    FakeI2cBus bus;
    setupChip(bus);
    bus.setReg8(pca::kAddress, pca::kPrescale, 30);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_NE(out.error().find("pca9685_probe check"), std::string::npos) << out.error();
    EXPECT_NE(out.error().find("121"), std::string::npos) << out.error();
    EXPECT_EQ(bus.writes(), 0);
  }
  // Prescale 122 is also not the 50 Hz value.
  {
    FakeI2cBus bus;
    setupChip(bus);
    bus.setReg8(pca::kAddress, pca::kPrescale, 122);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_NE(out.error().find("121"), std::string::npos) << out.error();
    EXPECT_EQ(bus.writes(), 0);
  }
  // No chip answers at all.
  {
    FakeI2cBus bus;
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_FALSE(out.error().empty());
    EXPECT_EQ(bus.writes(), 0);
  }
  // The very first read fails.
  {
    FakeI2cBus bus;
    setupChip(bus);
    bus.failRange(0, 1);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_FALSE(out.preflight());
    EXPECT_NE(out.error().find("0x40"), std::string::npos) << out.error();
    EXPECT_EQ(bus.writes(), 0);
  }
  // The healthy chip passes.
  {
    FakeI2cBus bus;
    setupChip(bus);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    EXPECT_TRUE(out.preflight()) << out.error();
  }
}

TEST(PwmOut, PulseEncodingAndLimits)
{
  using namespace dog_bench;
  // 50 Hz frame: prescale 121 -> 25 MHz / (4096 * 122) = 50.029 Hz.
  const double hz = pca::kOscillatorHz / (4096.0 * (pca::kPrescale50Hz + 1.0));
  EXPECT_NEAR(hz, 50.029, 0.001);
  EXPECT_NEAR(1e6 / hz, 19988.5, 0.5);

  EXPECT_EQ(pca::ticksFor(1370.0), 281);
  EXPECT_EQ(pca::ticksFor(1000.0), 205);
  EXPECT_EQ(pca::ticksFor(2500.0), 512);

  FakeI2cBus bus;
  setupChip(bus);
  Pca9685Out out(bus, 1, 1000.0, 1740.0);
  // Before a successful preflight nothing may be written.
  EXPECT_FALSE(out.setPulseUs(1370.0));
  EXPECT_EQ(bus.writes(), 0);
  ASSERT_TRUE(out.preflight()) << out.error();
  EXPECT_FALSE(out.setPulseUs(999.9));
  EXPECT_FALSE(out.setPulseUs(1740.1));
  EXPECT_FALSE(out.setPulseUs(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(out.setPulseUs(std::numeric_limits<double>::infinity()));
  EXPECT_EQ(bus.writes(), 0);  // none of the refusals reached the bus
  ASSERT_TRUE(out.setPulseUs(1370.0));
  // Channel 1 registers: LED1_ON_L 0x0A .. LED1_OFF_H 0x0D.
  EXPECT_EQ(bus.reg8(pca::kAddress, 0x0A), 0x00);
  EXPECT_EQ(bus.reg8(pca::kAddress, 0x0B), 0x00);
  EXPECT_EQ(bus.reg8(pca::kAddress, 0x0C), 0x19);  // 281 & 0xFF
  EXPECT_EQ(bus.reg8(pca::kAddress, 0x0D), 0x01);  // 281 >> 8
  EXPECT_EQ(bus.writes(), 1);

  // A structurally invalid output refuses before touching the bus at all.
  const double limits[4][2] = {
    {400.0, 1000.0}, {1000.0, 2600.0}, {1200.0, 1200.0}, {1500.0, 1200.0}};
  for (const auto & l : limits) {
    FakeI2cBus fresh;
    setupChip(fresh);
    Pca9685Out o(fresh, 1, l[0], l[1]);
    EXPECT_FALSE(o.preflight());
    EXPECT_EQ(fresh.transactions(), 0);
    EXPECT_FALSE(o.setPulseUs(1200.0));
    EXPECT_EQ(fresh.writes(), 0);
  }
  for (const int channel : {-1, 16}) {
    FakeI2cBus fresh;
    setupChip(fresh);
    Pca9685Out o(fresh, channel, 1000.0, 1740.0);
    EXPECT_FALSE(o.preflight());
    EXPECT_EQ(fresh.transactions(), 0);
  }
}

TEST(PwmOut, WritesOnlyOwnChannel)
{
  using namespace dog_bench;
  FakeI2cBus bus;
  setupChip(bus);
  Pca9685Out out(bus, 1, 1000.0, 1740.0);
  ASSERT_TRUE(out.preflight()) << out.error();
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(out.setPulseUs(1000.0 + 10.0 * i));
  }
  ASSERT_TRUE(out.release());
  EXPECT_EQ(bus.reg8(pca::kAddress, pca::kMode1), 0x21);
  EXPECT_EQ(bus.reg8(pca::kAddress, pca::kPrescale), pca::kPrescale50Hz);
  for (int n = 0; n < pca::kPwmChannels; ++n) {
    if (n == 1) {continue;}
    EXPECT_EQ(bus.reg8(pca::kAddress, static_cast<uint8_t>(0x09 + 4 * n)), pca::kFullOffBit)
      << "channel " << n;
  }
  EXPECT_EQ(bus.writes(), 51);  // 50 pulses and one release
  EXPECT_TRUE(out.released());
}

TEST(PwmOut, ReleaseRetriesAndReportsFailure)
{
  using namespace dog_bench;
  // Two attempts fail, the third succeeds: release() reports true.
  {
    FakeI2cBus bus;
    setupChip(bus);
    Pca9685Out out(bus, 1, 1000.0, 1740.0);
    ASSERT_TRUE(out.preflight()) << out.error();
    ASSERT_TRUE(out.setPulseUs(1370.0));
    const int first = bus.transactions();
    const int w0 = bus.writes();
    bus.failRange(first, 2);
    EXPECT_TRUE(out.release());
    EXPECT_TRUE(out.released());
    EXPECT_EQ(bus.writes(), w0 + 3);
    EXPECT_EQ(bus.reg8(pca::kAddress, 0xFD), 0x10);
  }
  // All three attempts fail: release() reports failure and the output stays
  // live; the destructor retries once the injected failures are over.
  {
    FakeI2cBus bus;
    setupChip(bus);
    {
      Pca9685Out out(bus, 1, 1000.0, 1740.0);
      ASSERT_TRUE(out.preflight()) << out.error();
      ASSERT_TRUE(out.setPulseUs(1370.0));
      const int first = bus.transactions();
      const int w0 = bus.writes();
      bus.failRange(first, 3);
      EXPECT_FALSE(out.release());
      EXPECT_FALSE(out.released());
      EXPECT_FALSE(out.error().empty());
      EXPECT_EQ(bus.writes(), w0 + 3);
      EXPECT_EQ(bus.reg8(pca::kAddress, 0xFD), 0);  // still holding the pulse
    }
    EXPECT_EQ(bus.reg8(pca::kAddress, 0xFD), 0x10);  // the destructor got it out
  }
}
