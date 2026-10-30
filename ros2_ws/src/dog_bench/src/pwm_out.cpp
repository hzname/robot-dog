#include "dog_bench/pwm_out.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace dog_bench
{

namespace pca
{
uint16_t ticksFor(double us)
{
  const double pwm_hz = kOscillatorHz / (4096.0 * (static_cast<double>(kPrescale50Hz) + 1.0));
  const double period_us = 1e6 / pwm_hz;
  const double ticks = std::round(us / period_us * 4096.0);
  return static_cast<uint16_t>(std::clamp(ticks, 0.0, 4095.0));
}
}  // namespace pca

namespace
{
std::string hexAddress()
{
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", pca::kAddress);
  return buf;
}

std::string formatNumber(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", value);
  return buf;
}
}  // namespace

Pca9685Out::Pca9685Out(I2cBus & bus, int channel, double pulse_min_us, double pulse_max_us)
  : bus_(&bus), channel_(channel), pulse_min_us_(pulse_min_us), pulse_max_us_(pulse_max_us)
{
  // The constructor never touches the bus: preflight() does the talking.
}

Pca9685Out::~Pca9685Out()
{
  // A servo must never stay under control; a never-armed object writes nothing.
  if (armed_) {release();}
}

bool Pca9685Out::preflight()
{
  armed_ = false;
  // Structural checks first: a bad configuration gets no bus access at all.
  if (channel_ < 0 || channel_ >= pca::kPwmChannels) {
    error_ = "channel " + std::to_string(channel_) + " is outside 0..15";
    return false;
  }
  if (!std::isfinite(pulse_min_us_) || !std::isfinite(pulse_max_us_) ||
    pulse_min_us_ < pca::kMinPulseUs || pulse_max_us_ > pca::kMaxPulseUs ||
    !(pulse_min_us_ < pulse_max_us_)) {
    error_ = "pulse window " + formatNumber(pulse_min_us_) + ".." + formatNumber(pulse_max_us_) +
      " us must be finite, within 500..2500 us and min < max";
    return false;
  }

  // The chip must be free and already configured; it is never initialized.
  uint8_t mode1 = 0;
  if (!bus_->read8(pca::kAddress, pca::kMode1, mode1)) {
    error_ = "no PCA9685 answering at " + hexAddress() + " (check wiring / i2cdetect); run pca9685_probe check";
    return false;
  }
  if ((mode1 & pca::kMode1Sleep) != 0) {
    error_ = "the PCA9685 at " + hexAddress() + " is asleep; run pca9685_probe check";
    return false;
  }
  if ((mode1 & pca::kMode1AutoInc) == 0) {
    error_ = "the PCA9685 at " + hexAddress() +
      " has auto-increment off (a 5-byte write would corrupt registers); run pca9685_probe check";
    return false;
  }
  uint8_t prescale = 0;
  if (!bus_->read8(pca::kAddress, pca::kPrescale, prescale)) {
    error_ = "could not read PRE_SCALE of the PCA9685 at " + hexAddress();
    return false;
  }
  if (prescale != pca::kPrescale50Hz) {
    error_ = "the PCA9685 at " + hexAddress() + " has prescale " + std::to_string(prescale) +
      ", expected 121 (50 Hz); run pca9685_probe check";
    return false;
  }
  for (int n = 0; n < pca::kPwmChannels; ++n) {
    uint8_t off_high = 0;
    if (!bus_->read8(pca::kAddress, static_cast<uint8_t>(0x09 + 4 * n), off_high)) {
      error_ = "could not read LED" + std::to_string(n) + "_OFF_H of the PCA9685 at " + hexAddress();
      return false;
    }
    if (n != channel_ && (off_high & pca::kFullOffBit) == 0) {
      error_ = "channel " + std::to_string(n) +
        " is still live - is the robot stack running? stop it (docker compose stop) or release it (pca9685_probe off)";
      return false;
    }
  }
  armed_ = true;
  error_.clear();
  return true;
}

bool Pca9685Out::setPulseUs(double us)
{
  if (!armed_) {
    error_ = "setPulseUs before a successful preflight";
    return false;
  }
  if (!std::isfinite(us) || us < pulse_min_us_ || us > pulse_max_us_) {
    error_ = "pulse " + formatNumber(us) + " us is outside the output window";
    return false;
  }
  const uint16_t ticks = pca::ticksFor(us);
  const uint8_t buf[5] = {
    static_cast<uint8_t>(pca::kLed0OnL + 4 * channel_),
    0,
    0,
    static_cast<uint8_t>(ticks & 0xFF),
    static_cast<uint8_t>((ticks >> 8) & 0x0F)};
  if (!bus_->writeBytes(pca::kAddress, buf, sizeof(buf))) {
    error_ = "PCA9685 write to channel " + std::to_string(channel_) + " at " + hexAddress() + " failed";
    return false;
  }
  released_ = false;
  return true;
}

bool Pca9685Out::release()
{
  if (!armed_) {return true;}  // a foreign stack is never touched
  for (int attempt = 0; attempt < pca::kReleaseAttempts; ++attempt) {
    if (bus_->writeBytes(pca::kAddress, pca::kAllLedOffBytes.data(), pca::kAllLedOffBytes.size())) {
      released_ = true;
      error_.clear();
      return true;
    }
  }
  error_ = "could not release the PCA9685 outputs at " + hexAddress() + " in 3 attempts";
  return false;
}

}  // namespace dog_bench
