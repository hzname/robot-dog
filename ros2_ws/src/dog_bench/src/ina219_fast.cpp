#include "dog_bench/ina219_fast.hpp"

#include <cmath>
#include <cstdio>

namespace dog_bench
{

namespace ina
{

bool isSafeAddress(int addr)
{
  return addr >= kIna219AddrMin && addr <= kIna219AddrMax;
}

double shuntVolts(uint16_t raw)
{
  return static_cast<int16_t>(raw) * 10e-6;
}

double busVolts(uint16_t raw)
{
  return static_cast<double>(raw >> 3) * 4e-3;
}

double fullScaleShuntVolts(uint16_t config)
{
  switch ((config >> 11) & 0x3) {
    case 0: {return 0.04;}
    case 1: {return 0.08;}
    case 2: {return 0.16;}
    default: {return 0.32;}
  }
}

bool configForShunt(double shunt_ohm, uint16_t & config)
{
  if (!std::isfinite(shunt_ohm)) {return false;}
  if (std::fabs(shunt_ohm - 0.1) <= 0.005) {
    config = kIna219Fast320mv;
    return true;
  }
  if (std::fabs(shunt_ohm - 0.01) <= 0.0005) {
    config = kIna219Fast80mv;
    return true;
  }
  return false;
}

}  // namespace ina

namespace
{
std::string hexByte(int value)
{
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", value);
  return buf;
}

std::string hexWord(uint16_t value)
{
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%04x", value);
  return buf;
}
}  // namespace

bool Ina219Fast::configure(uint16_t config)
{
  if (!ina::isSafeAddress(address_)) {
    error_ = "refusing address " + hexByte(address_) +
      ": an INA219 lives at 0x41..0x4f, 0x40 is the PCA9685 and is never probed as an INA";
    return false;
  }
  if ((config & 0x8000) != 0) {
    error_ = "refusing config " + hexWord(config) + ": the reset bit b15 is set";
    return false;
  }
  uint16_t value = 0;
  if (!bus_->read16(address_, ina::kRegConfig, value)) {
    error_ = "no answer from an INA219 at " + hexByte(address_);
    return false;
  }
  if ((value & 0x8000) != 0) {
    error_ = "device at " + hexByte(address_) + " has the reset bit set: not an INA219";
    return false;
  }
  if (!bus_->write16(address_, ina::kRegConfig, config)) {
    error_ = "could not write the config to " + hexByte(address_);
    return false;
  }
  if (!bus_->read16(address_, ina::kRegConfig, value) || value != config) {
    error_ = "config read-back from " + hexByte(address_) + " does not match " + hexWord(config);
    return false;
  }
  pointer_ = ina::kRegConfig;
  error_.clear();
  return true;
}

bool Ina219Fast::readShunt(uint16_t & raw)
{
  if (!ina::isSafeAddress(address_)) {
    error_ = "refusing address " + hexByte(address_) +
      ": an INA219 lives at 0x41..0x4f, 0x40 is the PCA9685 and is never probed as an INA";
    return false;
  }
  const bool on_shunt = pointer_ == ina::kRegShunt;
  const bool ok = on_shunt ?
    bus_->readBare16(address_, raw) : bus_->read16(address_, ina::kRegShunt, raw);
  if (!ok) {
    pointer_ = -1;  // the pointer write may or may not have gone through
    error_ = "could not read the shunt register of " + hexByte(address_);
    return false;
  }
  pointer_ = ina::kRegShunt;
  return true;
}

bool Ina219Fast::readBus(uint16_t & raw)
{
  if (!ina::isSafeAddress(address_)) {
    error_ = "refusing address " + hexByte(address_) +
      ": an INA219 lives at 0x41..0x4f, 0x40 is the PCA9685 and is never probed as an INA";
    return false;
  }
  if (!bus_->read16(address_, ina::kRegBus, raw)) {
    pointer_ = -1;
    error_ = "could not read the bus register of " + hexByte(address_);
    return false;
  }
  pointer_ = ina::kRegBus;
  return true;
}

double Ina219Fast::currentA(uint16_t shunt_raw, double shunt_ohm)
{
  return ina::shuntVolts(shunt_raw) / shunt_ohm;
}

}  // namespace dog_bench
