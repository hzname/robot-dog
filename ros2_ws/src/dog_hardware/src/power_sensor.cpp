#include "dog_hardware/power_sensor.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace dog_hardware
{

namespace ina
{
double ina226ShuntVolts(uint16_t raw) {return static_cast<int16_t>(raw) * 2.5e-6;}
double ina226BusVolts(uint16_t raw) {return raw * 1.25e-3;}
double ina219ShuntVolts(uint16_t raw) {return static_cast<int16_t>(raw) * 10e-6;}
double ina219BusVolts(uint16_t raw) {return (raw >> 3) * 4e-3;}
bool looksLikeIna219(uint16_t config_readback) {return config_readback == kIna219Config;}
}  // namespace ina

namespace
{

class I2cRegs
{
public:
  I2cRegs(int fd, int addr) : fd_(fd), addr_(addr) {}

  bool read16(uint8_t reg, uint16_t & out) const
  {
    uint8_t buf[2] = {0, 0};
    uint8_t r = reg;
    i2c_msg msgs[2] = {
      {static_cast<uint16_t>(addr_), 0, 1, &r},
      {static_cast<uint16_t>(addr_), I2C_M_RD, 2, buf}};
    i2c_rdwr_ioctl_data data{msgs, 2};
    if (::ioctl(fd_, I2C_RDWR, &data) != 2) {return false;}
    out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    return true;
  }

  bool write16(uint8_t reg, uint16_t v) const
  {
    uint8_t buf[3] = {reg, static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v & 0xFF)};
    i2c_msg msg{static_cast<uint16_t>(addr_), 0, 3, buf};
    i2c_rdwr_ioctl_data data{&msg, 1};
    return ::ioctl(fd_, I2C_RDWR, &data) == 1;
  }

private:
  int fd_;
  int addr_;
};

std::string hexAddr(int a)
{
  char b[8];
  std::snprintf(b, sizeof(b), "0x%02x", a);
  return b;
}

class InaSensor : public PowerSensor
{
public:
  InaSensor(int fd, int addr, bool is226, double shunt, std::string device)
  : fd_(fd), regs_(fd, addr), addr_(addr), is226_(is226), shunt_(shunt), device_(std::move(device)) {}
  ~InaSensor() override {::close(fd_);}

  bool read(PowerReading & out) override
  {
    uint16_t sh = 0, bus = 0;
    if (!regs_.read16(0x01, sh) || !regs_.read16(0x02, bus)) {return false;}
    const double v_shunt = is226_ ? ina::ina226ShuntVolts(sh) : ina::ina219ShuntVolts(sh);
    out.current = v_shunt / shunt_;
    out.voltage = is226_ ? ina::ina226BusVolts(bus) : ina::ina219BusVolts(bus);
    return true;
  }

  std::string describe() const override
  {
    std::ostringstream s;
    s << (is226_ ? "INA226" : "INA219") << " at " << hexAddr(addr_) << " on " << device_
      << ", shunt " << shunt_ * 1000.0 << " mOhm";
    return s.str();
  }

private:
  int fd_;
  I2cRegs regs_;
  int addr_;
  bool is226_;
  double shunt_;
  std::string device_;
};

}  // namespace

std::unique_ptr<PowerSensor> probePowerSensor(const std::string & device,
  const std::vector<int> & addresses, const std::string & chip, double shunt_ohm,
  std::string & found)
{
  found.clear();
  for (int addr : addresses) {
    if (addr == 0x40) {continue;}  // PCA9685 lives there; never write to it
    const int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0) {
      found = "cannot open " + device;
      return nullptr;
    }
    I2cRegs regs(fd, addr);
    uint16_t v = 0;
    if (!regs.read16(0x00, v)) {  // nothing answers here
      ::close(fd);
      found += hexAddr(addr) + ": no device; ";
      continue;
    }
    // INA226 identifies itself (read-only registers, safe to probe).
    uint16_t man = 0, die = 0;
    const bool is226 = regs.read16(0xFE, man) && regs.read16(0xFF, die) &&
      man == ina::kIna226Manufacturer && (die & 0xFFF0) == ina::kIna226Die;
    if (is226 && (chip == "auto" || chip == "ina226")) {
      regs.write16(0x00, ina::kIna226Config);
      return std::make_unique<InaSensor>(fd, addr, true, shunt_ohm, device);
    }
    // INA219 has no ID: accept only a plausible config (bit 15 = reset, reads 0)
    // and a successful write/read-back of our own configuration.
    if (!is226 && (chip == "auto" || chip == "ina219") && (v & 0x8000) == 0) {
      if (regs.write16(0x00, ina::kIna219Config) && regs.read16(0x00, v) && ina::looksLikeIna219(v)) {
        return std::make_unique<InaSensor>(fd, addr, false, shunt_ohm, device);
      }
    }
    found += hexAddr(addr) + ": unknown device; ";
    ::close(fd);
  }
  return nullptr;
}

// ------------------------------------------------------------ PowerGuard

PowerGuard::Event PowerGuard::update(const PowerReading & r, double now)
{
  const double dt = last_ < 0.0 ? 0.0 : std::max(0.0, now - last_);
  last_ = now;
  const double k = p_.filter_tau > 0.0 ? 1.0 - std::exp(-dt / p_.filter_tau) : 1.0;
  current_ = (dt == 0.0 && k < 1.0) ? r.current : current_ + (r.current - current_) * k;

  Event ev = Event::NONE;
  if (current_ > p_.overcurrent_a) {
    if (over_since_ < 0.0) {over_since_ = now;}
    if (!over_fired_ && now - over_since_ >= p_.overcurrent_time) {
      over_fired_ = true;
      ev = Event::OVERCURRENT;
    }
  } else {
    over_since_ = -1.0;
    over_fired_ = false;
  }
  if (p_.undervoltage_v > 0.0 && r.voltage > 0.5 && r.voltage < p_.undervoltage_v) {
    if (under_since_ < 0.0) {under_since_ = now;}
    if (!under_fired_ && now - under_since_ >= p_.undervoltage_time && ev == Event::NONE) {
      under_fired_ = true;
      ev = Event::UNDERVOLTAGE;
    }
  } else {
    under_since_ = -1.0;
    under_fired_ = false;
  }
  return ev;
}

}  // namespace dog_hardware
