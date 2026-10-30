// Tests for the fast INA219 layer: register field decomposition, shunt/bus
// scaling and the fake bus behaviour the servo bench tool relies on
// (D-08, D-09, D-10). No hardware and no real time: the fake bus records
// transactions and models the INA219 register pointer.
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"

TEST(Ina219Fast, ConfigRegisterValues)
{
  using namespace dog_bench;
  // 0x199F = 16 V range (BRNG 0), PGA /8 (PG 3), 12-bit shunt and bus
  // conversions (BADC 3, SADC 3), continuous shunt + bus (MODE 7):
  // a new result every 1.064 ms on a 0.1 Ohm shunt (+-3.2 A).
  EXPECT_EQ(ina::kIna219Fast320mv, 0x199F);
  EXPECT_EQ((ina::kIna219Fast320mv >> 13) & 0x1, 0);
  EXPECT_EQ((ina::kIna219Fast320mv >> 11) & 0x3, 3);
  EXPECT_EQ((ina::kIna219Fast320mv >> 7) & 0xF, 3);
  EXPECT_EQ((ina::kIna219Fast320mv >> 3) & 0xF, 3);
  EXPECT_EQ(ina::kIna219Fast320mv & 0x7, 7);
  // 10 mOhm shunt: PGA /2 (PG 1, +-8 A), the other fields unchanged.
  EXPECT_EQ(ina::kIna219Fast80mv, 0x099F);
  EXPECT_EQ((ina::kIna219Fast80mv >> 13) & 0x1, 0);
  EXPECT_EQ((ina::kIna219Fast80mv >> 11) & 0x3, 1);
  EXPECT_EQ((ina::kIna219Fast80mv >> 7) & 0xF, 3);
  EXPECT_EQ((ina::kIna219Fast80mv >> 3) & 0xF, 3);
  EXPECT_EQ(ina::kIna219Fast80mv & 0x7, 7);

  EXPECT_NEAR(ina::fullScaleShuntVolts(ina::kIna219Fast320mv), 0.32, 1e-12);
  EXPECT_NEAR(ina::fullScaleShuntVolts(ina::kIna219Fast80mv), 0.08, 1e-12);

  uint16_t config = 0;
  EXPECT_TRUE(ina::configForShunt(0.1, config));
  EXPECT_EQ(config, ina::kIna219Fast320mv);
  EXPECT_TRUE(ina::configForShunt(0.01, config));
  EXPECT_EQ(config, ina::kIna219Fast80mv);
  EXPECT_FALSE(ina::configForShunt(0.05, config));
  EXPECT_FALSE(ina::configForShunt(0.0, config));
  EXPECT_FALSE(ina::configForShunt(-0.1, config));
  EXPECT_FALSE(ina::configForShunt(std::numeric_limits<double>::quiet_NaN(), config));
}

TEST(Ina219Fast, ShuntScaling)
{
  using namespace dog_bench;
  // Shunt: 10 uV per LSB, two's complement; bus: 4 mV per LSB in bits 15..3.
  EXPECT_NEAR(ina::shuntVolts(1000), 0.010, 1e-12);
  EXPECT_NEAR(ina::shuntVolts(0xFC18), -0.010, 1e-12);  // -1000 counts
  EXPECT_NEAR(ina::busVolts(1500 << 3), 6.0, 1e-12);
  // CNVR (b1) and OVF (b0) do not add volts.
  EXPECT_NEAR(ina::busVolts((1500 << 3) | 0x3), 6.0, 1e-12);
  // Current is computed on the host: I = Vshunt / Rshunt.
  EXPECT_NEAR(Ina219Fast::currentA(1000, 0.1), 0.1, 1e-12);
  EXPECT_NEAR(Ina219Fast::currentA(1000, 0.01), 1.0, 1e-12);
  EXPECT_NEAR(Ina219Fast::currentA(32000, 0.1), 3.2, 1e-12);  // PGA /8 full scale
}

TEST(Ina219Fast, ConfigureWritesAndReadsBack)
{
  using namespace dog_bench;
  FakeI2cBus bus;
  bus.addDevice(0x41);
  bus.setReg16(0x41, ina::kRegConfig, 0x399F);  // power-on default, reset bit 0
  Ina219Fast ina(bus, 0x41);
  EXPECT_TRUE(ina.configure(ina::kIna219Fast320mv));
  EXPECT_EQ(bus.reg16(0x41, ina::kRegConfig), ina::kIna219Fast320mv);
  EXPECT_EQ(bus.transactions(0x40), 0);  // the PCA9685 address is never touched
}

TEST(Ina219Fast, RefusesPca9685Address)
{
  using namespace dog_bench;
  EXPECT_TRUE(ina::isSafeAddress(0x41));
  EXPECT_TRUE(ina::isSafeAddress(0x4F));
  EXPECT_FALSE(ina::isSafeAddress(0x40));
  EXPECT_FALSE(ina::isSafeAddress(0x50));
  EXPECT_FALSE(ina::isSafeAddress(0x70));
  EXPECT_FALSE(ina::isSafeAddress(0x03));
  EXPECT_FALSE(ina::isSafeAddress(0));

  FakeI2cBus bus;
  bus.addDevice(0x40);  // a device answers there: the PCA9685
  Ina219Fast ina(bus, 0x40);
  uint16_t raw = 0;
  EXPECT_FALSE(ina.configure(ina::kIna219Fast320mv));
  EXPECT_FALSE(ina.readShunt(raw));
  EXPECT_FALSE(ina.readBus(raw));
  EXPECT_EQ(bus.transactions(), 0);
}

TEST(Ina219Fast, ConfigureRefusesBadDevice)
{
  using namespace dog_bench;
  {
    FakeI2cBus bus;  // nothing answers on the bus
    Ina219Fast ina(bus, 0x41);
    EXPECT_FALSE(ina.configure(ina::kIna219Fast320mv));
    EXPECT_EQ(bus.writes(), 0);
  }
  {
    FakeI2cBus bus;
    bus.addDevice(0x41);
    bus.setReg16(0x41, ina::kRegConfig, 0x8000);  // reset bit set: not an INA219
    Ina219Fast ina(bus, 0x41);
    EXPECT_FALSE(ina.configure(ina::kIna219Fast320mv));
    EXPECT_EQ(bus.writes(), 0);
  }
  {
    FakeI2cBus bus;
    bus.addDevice(0x41);
    bus.setReg16(0x41, ina::kRegConfig, 0x399F);
    bus.dropWrites(0x41, true);  // the write "succeeds" but does not stick
    Ina219Fast ina(bus, 0x41);
    EXPECT_FALSE(ina.configure(ina::kIna219Fast320mv));
  }
  {
    FakeI2cBus bus;
    bus.addDevice(0x41);
    Ina219Fast ina(bus, 0x41);
    EXPECT_FALSE(ina.configure(0x999F));  // reset bit b15 in the requested config
    EXPECT_EQ(bus.writes(), 0);
  }
}

TEST(Ina219Fast, PointerIsReused)
{
  using namespace dog_bench;
  FakeI2cBus bus;
  bus.addDevice(0x41);
  bus.setReg16(0x41, ina::kRegConfig, 0x399F);
  bus.setReg16(0x41, ina::kRegShunt, 300);      // 30 mA on 0.1 Ohm
  bus.setReg16(0x41, ina::kRegBus, 1500 << 3);  // 6.0 V
  Ina219Fast ina(bus, 0x41);
  ASSERT_TRUE(ina.configure(ina::kIna219Fast320mv));

  const int p0 = bus.pointerReads();
  const int b0 = bus.bareReads();
  uint16_t raw = 0;
  ASSERT_TRUE(ina.readShunt(raw));
  EXPECT_EQ(raw, 300);
  EXPECT_EQ(bus.pointerReads(), p0 + 1);  // the pointer moved with read16
  ASSERT_TRUE(ina.readShunt(raw));
  ASSERT_TRUE(ina.readShunt(raw));
  EXPECT_EQ(bus.bareReads(), b0 + 2);  // repeated reads reuse the pointer

  ASSERT_TRUE(ina.readBus(raw));
  EXPECT_NEAR(ina::busVolts(raw), 6.0, 1e-12);
  ASSERT_TRUE(ina.readShunt(raw));
  EXPECT_EQ(bus.pointerReads(), p0 + 3);  // readBus left the pointer elsewhere

  // A failed read leaves the pointer unknown; the next read sets it again.
  const int p1 = bus.pointerReads();
  const int b1 = bus.bareReads();
  const int t = bus.transactions();
  bus.failRange(t, 1);
  EXPECT_FALSE(ina.readShunt(raw));     // bare read fails: pointer now unknown
  EXPECT_EQ(bus.bareReads(), b1 + 1);
  ASSERT_TRUE(ina.readShunt(raw));      // pointer set again
  EXPECT_EQ(bus.pointerReads(), p1 + 1);
  EXPECT_EQ(raw, 300);
}
