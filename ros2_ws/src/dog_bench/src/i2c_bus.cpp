#include "dog_bench/i2c_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace dog_bench
{

namespace
{
constexpr std::size_t kMaxWriteBytes = 32;
}  // namespace

// ------------------------------------------------------------ LinuxI2cBus

LinuxI2cBus::LinuxI2cBus(const std::string & device)
{
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    last_errno_ = errno;
    error_ = std::strerror(errno);
  }
}

LinuxI2cBus::~LinuxI2cBus()
{
  if (fd_ >= 0) {::close(fd_);}
}

bool LinuxI2cBus::failed(int err)
{
  last_errno_ = err;
  error_ = std::strerror(err);
  return false;
}

bool LinuxI2cBus::read16(int addr, uint8_t reg, uint16_t & out)
{
  uint8_t buf[2] = {0, 0};
  uint8_t r = reg;
  i2c_msg msgs[2] = {
    {static_cast<uint16_t>(addr), 0, 1, &r},
    {static_cast<uint16_t>(addr), I2C_M_RD, 2, buf}};
  i2c_rdwr_ioctl_data data{msgs, 2};
  if (::ioctl(fd_, I2C_RDWR, &data) != 2) {return failed(errno);}
  out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  return true;
}

bool LinuxI2cBus::write16(int addr, uint8_t reg, uint16_t value)
{
  uint8_t buf[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  i2c_msg msg{static_cast<uint16_t>(addr), 0, 3, buf};
  i2c_rdwr_ioctl_data data{&msg, 1};
  if (::ioctl(fd_, I2C_RDWR, &data) != 1) {return failed(errno);}
  return true;
}

bool LinuxI2cBus::readBare16(int addr, uint16_t & out)
{
  uint8_t buf[2] = {0, 0};
  i2c_msg msg{static_cast<uint16_t>(addr), I2C_M_RD, 2, buf};
  i2c_rdwr_ioctl_data data{&msg, 1};
  if (::ioctl(fd_, I2C_RDWR, &data) != 1) {return failed(errno);}
  out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  return true;
}

bool LinuxI2cBus::read8(int addr, uint8_t reg, uint8_t & out)
{
  uint8_t buf[1] = {0};
  uint8_t r = reg;
  i2c_msg msgs[2] = {
    {static_cast<uint16_t>(addr), 0, 1, &r},
    {static_cast<uint16_t>(addr), I2C_M_RD, 1, buf}};
  i2c_rdwr_ioctl_data data{msgs, 2};
  if (::ioctl(fd_, I2C_RDWR, &data) != 2) {return failed(errno);}
  out = buf[0];
  return true;
}

bool LinuxI2cBus::writeBytes(int addr, const uint8_t * data, std::size_t len)
{
  if (data == nullptr || len == 0 || len > kMaxWriteBytes) {return false;}
  uint8_t buf[kMaxWriteBytes];
  std::memcpy(buf, data, len);
  i2c_msg msg{static_cast<uint16_t>(addr), 0, static_cast<uint16_t>(len), buf};
  i2c_rdwr_ioctl_data io{&msg, 1};
  if (::ioctl(fd_, I2C_RDWR, &io) != 1) {return failed(errno);}
  return true;
}

// ------------------------------------------------------------ FakeI2cBus

void FakeI2cBus::addDevice(int addr)
{
  devices_[addr];
}

void FakeI2cBus::setReg16(int addr, uint8_t reg, uint16_t value)
{
  devices_[addr].regs[reg] = value;
}

uint16_t FakeI2cBus::reg16(int addr, uint8_t reg) const
{
  const Device * dev = deviceFor(addr);
  if (dev == nullptr) {return 0;}
  const auto it = dev->regs.find(reg);
  return it == dev->regs.end() ? 0 : it->second;
}

void FakeI2cBus::setReg8(int addr, uint8_t reg, uint8_t value)
{
  devices_[addr].regs[reg] = value;
}

uint8_t FakeI2cBus::reg8(int addr, uint8_t reg) const
{
  return static_cast<uint8_t>(reg16(addr, reg) & 0xFF);
}

void FakeI2cBus::dropWrites(int addr, bool drop)
{
  devices_[addr].drop_writes = drop;
}

void FakeI2cBus::failRange(int first, int count)
{
  if (count > 0) {fail_ranges_.emplace_back(first, count);}
}

void FakeI2cBus::setTransactionHook(std::function<void()> hook)
{
  hook_ = std::move(hook);
}

int FakeI2cBus::transactions(int addr) const
{
  const auto it = counts_.find(addr);
  return it == counts_.end() ? 0 : it->second;
}

FakeI2cBus::Device * FakeI2cBus::deviceFor(int addr)
{
  const auto it = devices_.find(addr);
  return it == devices_.end() ? nullptr : &it->second;
}

const FakeI2cBus::Device * FakeI2cBus::deviceFor(int addr) const
{
  const auto it = devices_.find(addr);
  return it == devices_.end() ? nullptr : &it->second;
}

bool FakeI2cBus::fails(int number) const
{
  for (const auto & range : fail_ranges_) {
    if (number >= range.first && number < range.first + range.second) {return true;}
  }
  return false;
}

int FakeI2cBus::beginTransaction(int addr)
{
  const int number = transactions_++;
  ++counts_[addr];
  if (hook_) {hook_();}
  return number;
}

bool FakeI2cBus::read16(int addr, uint8_t reg, uint16_t & out)
{
  ++pointer_reads_;
  const int number = beginTransaction(addr);
  Device * dev = deviceFor(addr);
  if (dev == nullptr || fails(number)) {return false;}
  dev->pointer = reg;
  out = reg16(addr, reg);
  return true;
}

bool FakeI2cBus::write16(int addr, uint8_t reg, uint16_t value)
{
  ++writes_;
  const int number = beginTransaction(addr);
  Device * dev = deviceFor(addr);
  if (dev == nullptr || fails(number)) {return false;}
  dev->pointer = reg;
  if (!dev->drop_writes) {dev->regs[reg] = value;}
  return true;
}

bool FakeI2cBus::readBare16(int addr, uint16_t & out)
{
  ++bare_reads_;
  const int number = beginTransaction(addr);
  Device * dev = deviceFor(addr);
  if (dev == nullptr || fails(number)) {return false;}
  const auto it = dev->regs.find(dev->pointer);
  out = it == dev->regs.end() ? 0 : it->second;
  return true;
}

bool FakeI2cBus::read8(int addr, uint8_t reg, uint8_t & out)
{
  const int number = beginTransaction(addr);
  Device * dev = deviceFor(addr);
  if (dev == nullptr || fails(number)) {return false;}
  dev->pointer = reg;
  out = reg8(addr, reg);
  return true;
}

bool FakeI2cBus::writeBytes(int addr, const uint8_t * data, std::size_t len)
{
  if (data == nullptr || len == 0 || len > kMaxWriteBytes) {return false;}
  ++writes_;
  const int number = beginTransaction(addr);
  Device * dev = deviceFor(addr);
  if (dev == nullptr || fails(number)) {return false;}
  const uint8_t first = data[0];
  if (!dev->drop_writes) {
    for (std::size_t i = 1; i < len; ++i) {
      dev->regs[static_cast<uint8_t>(first + i)] = data[i];
    }
  }
  dev->pointer = static_cast<uint8_t>(first + len - 1);
  return true;
}

}  // namespace dog_bench
