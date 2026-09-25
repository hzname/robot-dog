#include "dog_hardware/servo_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <thread>

namespace dog_hardware
{

// ---------------------------------------------------------------- MockBus

bool MockBus::setPulseUs(int channel, double us)
{
  if (channel < 0 || channel >= kPwmChannels) {return false;}
  pulses_[channel] = us;
  ++writes_;
  return true;
}

bool MockBus::disable(int channel)
{
  if (channel < 0 || channel >= kPwmChannels) {return false;}
  pulses_[channel] = 0.0;
  ++writes_;
  return true;
}

bool MockBus::disableAll()
{
  pulses_.fill(0.0);
  ++writes_;
  return true;
}

// ------------------------------------------------------------- Pca9685Bus

namespace
{
constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kMode2 = 0x01;
constexpr uint8_t kLed0OnL = 0x06;
constexpr uint8_t kAllLedOnL = 0xFA;
constexpr uint8_t kPrescale = 0xFE;

constexpr uint8_t kMode1Restart = 0x80;
constexpr uint8_t kMode1AutoInc = 0x20;
constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode1AllCall = 0x01;
constexpr uint8_t kMode2OutDrv = 0x04;
constexpr uint8_t kFullOffBit = 0x10;  // in LEDn_OFF_H
}  // namespace

Pca9685Bus::~Pca9685Bus() {close();}

uint8_t Pca9685Bus::prescaleFor(double pwm_hz, double oscillator_hz)
{
  const double v = std::round(oscillator_hz / (4096.0 * pwm_hz)) - 1.0;
  return static_cast<uint8_t>(std::clamp(v, 3.0, 255.0));
}

double Pca9685Bus::frequencyFor(uint8_t prescale, double oscillator_hz)
{
  return oscillator_hz / (4096.0 * (static_cast<double>(prescale) + 1.0));
}

uint16_t Pca9685Bus::ticksFor(double us, double pwm_hz)
{
  const double period_us = 1e6 / pwm_hz;
  const double ticks = std::round(us / period_us * 4096.0);
  return static_cast<uint16_t>(std::clamp(ticks, 0.0, 4095.0));
}

bool Pca9685Bus::open(const std::string & device, int address, double pwm_hz,
  double oscillator_hz, std::string & error)
{
  close();
  device_ = device;
  address_ = address;
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    error = "open(" + device + "): " + std::strerror(errno);
    return false;
  }
  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    error = "ioctl(I2C_SLAVE): " + std::string(std::strerror(errno));
    close();
    return false;
  }

  const uint8_t prescale = prescaleFor(pwm_hz, oscillator_hz);
  actual_hz_ = frequencyFor(prescale, oscillator_hz);

  uint8_t mode1 = 0;
  uint8_t current_prescale = 0;
  if (!readReg(kMode1, mode1) || !readReg(kPrescale, current_prescale)) {
    error = "no PCA9685 answering at " + device + " addr " + std::to_string(address) +
      " (check wiring / i2cdetect)";
    close();
    return false;
  }
  const bool already_running = !(mode1 & kMode1Sleep) && current_prescale == prescale;
  if (already_running) {
    if (!writeReg(kMode1, (mode1 & ~kMode1Restart) | kMode1AutoInc | kMode1AllCall)) {
      error = "PCA9685 MODE1 write failed: " + std::string(std::strerror(errno));
      close();
      return false;
    }
    return true;
  }

  // Prescaler can only be written while asleep.
  const bool ok =
    writeReg(kMode1, kMode1Sleep | kMode1AutoInc | kMode1AllCall) &&
    writeReg(kPrescale, prescale) &&
    writeReg(kMode2, kMode2OutDrv) &&
    disableAll() &&
    writeReg(kMode1, kMode1AutoInc | kMode1AllCall);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));  // oscillator start-up (500 us)
  if (!ok) {
    error = "PCA9685 init failed: " + std::string(std::strerror(errno));
    close();
    return false;
  }
  return true;
}

void Pca9685Bus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Pca9685Bus::writeReg(uint8_t reg, uint8_t value)
{
  const uint8_t buf[2] = {reg, value};
  return fd_ >= 0 && ::write(fd_, buf, 2) == 2;
}

bool Pca9685Bus::readReg(uint8_t reg, uint8_t & value)
{
  return fd_ >= 0 && ::write(fd_, &reg, 1) == 1 && ::read(fd_, &value, 1) == 1;
}

bool Pca9685Bus::writeChannel(int channel, uint16_t on, uint16_t off)
{
  if (channel < 0 || channel >= kPwmChannels || fd_ < 0) {return false;}
  const uint8_t buf[5] = {
    static_cast<uint8_t>(kLed0OnL + 4 * channel),
    static_cast<uint8_t>(on & 0xFF), static_cast<uint8_t>((on >> 8) & 0x1F),
    static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>((off >> 8) & 0x1F)};
  return ::write(fd_, buf, 5) == 5;
}

bool Pca9685Bus::setPulseUs(int channel, double us)
{
  return writeChannel(channel, 0, ticksFor(us, actual_hz_));
}

bool Pca9685Bus::disable(int channel)
{
  return writeChannel(channel, 0, static_cast<uint16_t>(kFullOffBit) << 8);
}

bool Pca9685Bus::disableAll()
{
  if (fd_ < 0) {return false;}
  const uint8_t buf[5] = {kAllLedOnL, 0, 0, 0, kFullOffBit};
  return ::write(fd_, buf, 5) == 5;
}

std::string Pca9685Bus::describe() const
{
  std::ostringstream s;
  s << "pca9685 " << device_ << " @0x" << std::hex << address_ << std::dec
    << " " << actual_hz_ << " Hz";
  return s.str();
}

}  // namespace dog_hardware
