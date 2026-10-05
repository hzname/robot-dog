// Fast INA219 current sensor access for the servo bench tool: the polling
// configuration (12-bit conversions, continuous shunt + bus) and the register
// pointer handling that keeps reads short. The scale formulas are copied from
// the driver package on purpose; its headers are not included (D-08, D-09).
#pragma once

#include <cstdint>
#include <string>

#include "dog_bench/i2c_bus.hpp"

namespace dog_bench
{

namespace ina
{
/// INA219 register addresses.
constexpr uint16_t kRegConfig = 0x00;
constexpr uint16_t kRegShunt = 0x01;
constexpr uint16_t kRegBus = 0x02;

/// The PCA9685 answers at 0x40 and is never probed or written as an INA.
constexpr int kPca9685Address = 0x40;
constexpr int kIna219AddrMin = 0x41;
constexpr int kIna219AddrMax = 0x4F;

/// 16 V range, PGA /8 (+-320 mV), 12-bit shunt and bus conversions, continuous
/// shunt + bus: a new result every 1.064 ms. On a 0.1 Ohm shunt: +-3.2 A.
constexpr uint16_t kIna219Fast320mv = 0x199F;
/// The same, but PGA /2 (+-80 mV) for a 10 mOhm shunt: +-8 A.
constexpr uint16_t kIna219Fast80mv = 0x099F;

/// True for 0x41..0x4f only: 0x40 is the PCA9685, 0x70..0x74 its all-call.
bool isSafeAddress(int addr);
/// Shunt voltage in volts: 10 uV per count, two's complement.
double shuntVolts(uint16_t raw);
/// Bus voltage in volts: 4 mV per count in bits 15..3.
double busVolts(uint16_t raw);
/// Full-scale shunt voltage in volts for the PGA field (bits 12:11).
double fullScaleShuntVolts(uint16_t config);
/// Configuration for a known shunt, or false for anything but 0.1 Ohm
/// (marked R100, +-3.2 A) and 0.01 Ohm (R010, +-8 A). The PGA is never
/// guessed: the owner reads the shunt marking.
bool configForShunt(double shunt_ohm, uint16_t & config);
}  // namespace ina

/// One INA219 polled fast. Does not own the bus.
class Ina219Fast
{
public:
  Ina219Fast(I2cBus & bus, int address) : bus_(&bus), address_(address) {}

  /// Verify the device answers like an INA219 (read before write, reset bit
  /// 0) and that `config` reads back exactly. False on any doubt, with the
  /// reason in error(); a config with the reset bit set is refused untouched,
  /// and so is any address outside 0x41..0x4f.
  bool configure(uint16_t config);
  /// Read the shunt register; reuses the register pointer when it is already
  /// on it. On failure the pointer state is dropped.
  bool readShunt(uint16_t & raw);
  /// Read the bus register (always moves the pointer there).
  bool readBus(uint16_t & raw);
  /// Shunt current in amperes, computed on the host: I = Vshunt / Rshunt.
  static double currentA(uint16_t shunt_raw, double shunt_ohm);

  const std::string & error() const {return error_;}
  int address() const {return address_;}

private:
  I2cBus * bus_;
  int address_;
  int pointer_{-1};  // -1 = unknown
  std::string error_;
};

}  // namespace dog_bench
