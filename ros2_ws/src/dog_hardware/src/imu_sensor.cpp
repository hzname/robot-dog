#include "dog_hardware/imu_sensor.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace dog_hardware
{

namespace
{
constexpr double kG = 9.80665;

std::string hexAddr(int a)
{
  char b[8];
  std::snprintf(b, sizeof(b), "0x%02x", a);
  return b;
}

bool readRegs(int fd, int addr, uint8_t reg, uint8_t * buf, uint16_t len)
{
  i2c_msg msgs[2] = {
    {static_cast<uint16_t>(addr), 0, 1, &reg},
    {static_cast<uint16_t>(addr), I2C_M_RD, len, buf}};
  i2c_rdwr_ioctl_data data{msgs, 2};
  return ::ioctl(fd, I2C_RDWR, &data) == 2;
}

bool writeReg(int fd, int addr, uint8_t reg, uint8_t v)
{
  uint8_t buf[2] = {reg, v};
  i2c_msg msg{static_cast<uint16_t>(addr), 0, 2, buf};
  i2c_rdwr_ioctl_data data{&msg, 1};
  return ::ioctl(fd, I2C_RDWR, &data) == 1;
}

double wrapPi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

class MpuSensor : public ImuSensor
{
public:
  MpuSensor(int fd, int addr, uint8_t id, std::string device)
  : fd_(fd), addr_(addr), id_(id), device_(std::move(device)) {}
  ~MpuSensor() override {::close(fd_);}

  bool read(ImuReading & out) override
  {
    uint8_t raw[14];
    if (!readRegs(fd_, addr_, 0x3B, raw, sizeof(raw))) {return false;}
    out = mpu::convert(raw);
    return true;
  }

  std::string describe() const override
  {
    char b[16];
    std::snprintf(b, sizeof(b), "0x%02x", id_);
    return "MPU6050-family (WHO_AM_I " + std::string(b) + ") at " + hexAddr(addr_) + " on " + device_;
  }

private:
  int fd_;
  int addr_;
  uint8_t id_;
  std::string device_;
};
}  // namespace

namespace mpu
{
bool knownWhoAmI(uint8_t id)
{
  // MPU6050 0x68 (clones: 0x72, 0x98), MPU6500 0x70, MPU9250 0x71, MPU9255 0x73
  return id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72 || id == 0x73 || id == 0x98;
}

ImuReading convert(const uint8_t * raw)
{
  auto s16 = [raw](int i) {return static_cast<int16_t>((raw[i] << 8) | raw[i + 1]);};
  ImuReading r;
  for (int k = 0; k < 3; ++k) {
    r.accel[k] = s16(2 * k) / kAccelLsbPerG * kG;
    r.gyro[k] = s16(8 + 2 * k) / kGyroLsbPerDps * M_PI / 180.0;
  }
  return r;
}
}  // namespace mpu

std::unique_ptr<ImuSensor> probeImu(const std::string & device,
  const std::vector<int> & addresses, std::string & found)
{
  std::ostringstream log;
  const int fd = ::open(device.c_str(), O_RDWR);
  if (fd < 0) {
    found = "cannot open " + device;
    return nullptr;
  }
  for (int addr : addresses) {
    uint8_t id = 0;
    if (!readRegs(fd, addr, 0x75, &id, 1)) {
      log << hexAddr(addr) << ": no answer; ";
      continue;
    }
    if (!mpu::knownWhoAmI(id)) {
      log << hexAddr(addr) << ": WHO_AM_I " << hexAddr(id) << " unknown; ";
      continue;
    }
    const bool ok =
      writeReg(fd, addr, 0x6B, 0x01) &&  // wake up, PLL on gyro X
      writeReg(fd, addr, 0x1A, 0x03) &&  // DLPF 44 Hz (as in v1)
      writeReg(fd, addr, 0x19, 0x09) &&  // 1 kHz / (1 + 9) = 100 Hz
      writeReg(fd, addr, 0x1B, 0x00) &&  // +-250 deg/s
      writeReg(fd, addr, 0x1C, 0x00);    // +-2 g
    if (!ok) {
      log << hexAddr(addr) << ": configuration failed; ";
      continue;
    }
    ::usleep(50000);
    found = log.str();
    return std::make_unique<MpuSensor>(fd, addr, id, device);
  }
  ::close(fd);
  found = log.str();
  return nullptr;
}

AxisMap::AxisMap(const std::string & spec)
{
  std::stringstream ss(spec);
  std::string item;
  int k = 0;
  bool used[3] = {false, false, false};
  while (std::getline(ss, item, ',')) {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    if (k >= 3 || item.empty()) {throw std::invalid_argument("axes: expected 3 entries: " + spec);}
    double sign = 1.0;
    if (item[0] == '-' || item[0] == '+') {
      sign = item[0] == '-' ? -1.0 : 1.0;
      item.erase(0, 1);
    }
    if (item.size() != 1 || item[0] < 'x' || item[0] > 'z') {
      throw std::invalid_argument("axes: bad entry '" + item + "' in " + spec);
    }
    const int idx = item[0] - 'x';
    if (used[idx]) {throw std::invalid_argument("axes: axis used twice in " + spec);}
    used[idx] = true;
    index_[k] = idx;
    sign_[k] = sign;
    ++k;
  }
  if (k != 3) {throw std::invalid_argument("axes: expected 3 entries: " + spec);}
}

Vec3d AxisMap::apply(const Vec3d & v) const
{
  return {sign_[0] * v[index_[0]], sign_[1] * v[index_[1]], sign_[2] * v[index_[2]]};
}

void AttitudeFilter::update(const ImuReading & b, double dt)
{
  const auto & a = b.accel;
  const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  const double acc_roll = std::atan2(a[1], a[2]);
  const double acc_pitch = std::atan2(-a[0], std::sqrt(a[1] * a[1] + a[2] * a[2]));
  if (!init_ || dt <= 0.0) {
    if (norm > 0.5 * kG) {
      roll_ = acc_roll;
      pitch_ = acc_pitch;
      init_ = true;
    }
    return;
  }
  // Euler angle rates from body rates.
  const double p = b.gyro[0], q = b.gyro[1], r = b.gyro[2];
  const double sr = std::sin(roll_), cr = std::cos(roll_);
  const double cp = std::max(std::cos(pitch_), 1e-3), tp = std::tan(pitch_);
  roll_ += (p + (q * sr + r * cr) * tp) * dt;
  pitch_ += (q * cr - r * sr) * dt;
  yaw_ = wrapPi(yaw_ + (q * sr + r * cr) / cp * dt);
  if (std::abs(norm / kG - 1.0) < gate_) {
    const double k = dt / (tau_ + dt);
    roll_ += wrapPi(acc_roll - roll_) * k;
    pitch_ += (acc_pitch - pitch_) * k;
  }
  roll_ = wrapPi(roll_);
}

std::array<double, 4> AttitudeFilter::quaternion() const
{
  const double cr = std::cos(roll_ / 2), sr = std::sin(roll_ / 2);
  const double cp = std::cos(pitch_ / 2), sp = std::sin(pitch_ / 2);
  const double cy = std::cos(yaw_ / 2), sy = std::sin(yaw_ / 2);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

bool GyroBias::add(const Vec3d & g)
{
  if (done()) {return true;}
  if (std::abs(g[0] - (n_ ? sum_[0] / n_ : g[0])) > 0.05 ||
    std::abs(g[1] - (n_ ? sum_[1] / n_ : g[1])) > 0.05 ||
    std::abs(g[2] - (n_ ? sum_[2] / n_ : g[2])) > 0.05)
  {
    n_ = 0;
    sum_ = {0.0, 0.0, 0.0};
    return false;  // this sample is motion, not bias
  }
  for (int k = 0; k < 3; ++k) {sum_[k] += g[k];}
  ++n_;
  return done();
}

Vec3d GyroBias::bias() const
{
  if (n_ == 0) {return {0.0, 0.0, 0.0};}
  return {sum_[0] / n_, sum_[1] / n_, sum_[2] / n_};
}

}  // namespace dog_hardware
