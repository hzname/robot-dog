/**
 * @file mpu6050_driver.cpp
 * @brief MPU6050 driver implementation with I2C and Mahony AHRS
 */

#include "dog_sensors_cpp/mpu6050_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <math>
#include <iostream>

namespace dog_sensors_cpp
{

// MPU6050 Register Map
namespace regs
{
constexpr uint8_t SELF_TEST_X = 0x0D;
constexpr uint8_t SELF_TEST_Y = 0x0E;
constexpr uint8_t SELF_TEST_Z = 0x0F;
constexpr uint8_t SELF_TEST_A = 0x10;
constexpr uint8_t SMPLRT_DIV = 0x19;
constexpr uint8_t CONFIG = 0x1A;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t FIFO_EN = 0x23;
constexpr uint8_t I2C_MST_CTRL = 0x24;
constexpr uint8_t I2C_SLV0_ADDR = 0x25;
constexpr uint8_t I2C_SLV0_REG = 0x26;
constexpr uint8_t I2C_SLV0_DO = 0x63;
constexpr uint8_t I2C_MST_STATUS = 0x36;
constexpr uint8_t INT_PIN_CFG = 0x37;
constexpr uint8_t INT_ENABLE = 0x38;
constexpr uint8_t INT_STATUS = 0x3A;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t ACCEL_XOUT_L = 0x3C;
constexpr uint8_t ACCEL_YOUT_H = 0x3D;
constexpr uint8_t ACCEL_YOUT_L = 0x3E;
constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L = 0x40;
constexpr uint8_t TEMP_OUT_H = 0x41;
constexpr uint8_t TEMP_OUT_L = 0x42;
constexpr uint8_t GYRO_XOUT_H = 0x43;
constexpr uint8_t GYRO_XOUT_L = 0x44;
constexpr uint8_t GYRO_YOUT_H = 0x45;
constexpr uint8_t GYRO_YOUT_L = 0x46;
constexpr uint8_t GYRO_ZOUT_H = 0x47;
constexpr uint8_t GYRO_ZOUT_L = 0x48;
constexpr uint8_t USER_CTRL = 0x6A;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t PWR_MGMT_2 = 0x6C;
constexpr uint8_t FIFO_COUNTH = 0x72;
constexpr uint8_t FIFO_COUNTL = 0x73;
constexpr uint8_t FIFO_R_W = 0x74;
constexpr uint8_t WHO_AM_I = 0x75;
}  // namespace regs

// Expected WHO_AM_I value
constexpr uint8_t WHO_AM_I_VALUE = 0x68;

Mpu6050Driver::Mpu6050Driver(const std::string & i2c_bus, uint8_t address)
: i2c_bus_(i2c_bus), address_(address)
{
}

Mpu6050Driver::~Mpu6050Driver()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool Mpu6050Driver::initialize()
{
  // Open I2C device
  fd_ = open(i2c_bus_.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }

  // Set I2C address
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Verify device
  if (!is_connected()) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Reset device
  if (!i2c_write_byte(regs::PWR_MGMT_1, 0x80)) {
    return false;
  }
  usleep(100000);  // Wait 100ms after reset

  // Wake up and select internal oscillator
  if (!i2c_write_byte(regs::PWR_MGMT_1, 0x00)) {
    return false;
  }

  // Disable FIFO
  if (!i2c_write_byte(regs::FIFO_EN, 0x00)) {
    return false;
  }

  // Configure DLPF (44Hz bandwidth for gyro, 21Hz delay)
  if (!set_dlpf_bandwidth(Mpu6050DlpfBandwidth::BW_44HZ)) {
    return false;
  }

  // Set sample rate to 1kHz (divider = 0)
  if (!set_sample_rate_divider(0)) {
    return false;
  }

  // Configure accelerometer range (±2g)
  if (!set_accel_range(Mpu6050AccelRange::RANGE_2G)) {
    return false;
  }

  // Configure gyroscope range (±250°/s)
  if (!set_gyro_range(Mpu6050GyroRange::RANGE_250DPS)) {
    return false;
  }

  // Reset Mahony filter
  reset_filter();

  return true;
}

bool Mpu6050Driver::is_connected() const
{
  if (fd_ < 0) {
    return false;
  }
  
  uint8_t who_am_i = 0;
  // const_cast needed for the non-const i2c_read_byte
  if (!const_cast<Mpu6050Driver*>(this)->i2c_read_byte(regs::WHO_AM_I, who_am_i)) {
    return false;
  }
  
  // MPU6050 returns 0x68, some clones may return different values
  return (who_am_i & 0x7E) == WHO_AM_I_VALUE;
}

bool Mpu6050Driver::set_accel_range(Mpu6050AccelRange range)
{
  uint8_t config = static_cast<uint8_t>(range) << 3;
  if (!i2c_write_byte(regs::ACCEL_CONFIG, config)) {
    return false;
  }
  accel_range_ = range;
  
  // Update scale factor (m/s² per LSB)
  switch (range) {
    case Mpu6050AccelRange::RANGE_2G:
      accel_scale_ = 9.80665 / 16384.0;
      break;
    case Mpu6050AccelRange::RANGE_4G:
      accel_scale_ = 9.80665 / 8192.0;
      break;
    case Mpu6050AccelRange::RANGE_8G:
      accel_scale_ = 9.80665 / 4096.0;
      break;
    case Mpu6050AccelRange::RANGE_16G:
      accel_scale_ = 9.80665 / 2048.0;
      break;
  }
  return true;
}

bool Mpu6050Driver::set_gyro_range(Mpu6050GyroRange range)
{
  uint8_t config = static_cast<uint8_t>(range) << 3;
  if (!i2c_write_byte(regs::GYRO_CONFIG, config)) {
    return false;
  }
  gyro_range_ = range;
  
  // Update scale factor (rad/s per LSB)
  switch (range) {
    case Mpu6050GyroRange::RANGE_250DPS:
      gyro_scale_ = 0.01745329252 / 131.0;  // (π/180) / 131
      break;
    case Mpu6050GyroRange::RANGE_500DPS:
      gyro_scale_ = 0.01745329252 / 65.5;
      break;
    case Mpu6050GyroRange::RANGE_1000DPS:
      gyro_scale_ = 0.01745329252 / 32.8;
      break;
    case Mpu6050GyroRange::RANGE_2000DPS:
      gyro_scale_ = 0.01745329252 / 16.4;
      break;
  }
  return true;
}

bool Mpu6050Driver::set_dlpf_bandwidth(Mpu6050DlpfBandwidth bandwidth)
{
  // DLPF_CFG bits: [2:0] of CONFIG register
  uint8_t config = static_cast<uint8_t>(bandwidth) & 0x07;
  return i2c_write_byte(regs::CONFIG, config);
}

bool Mpu6050Driver::set_sample_rate_divider(uint8_t divider)
{
  return i2c_write_byte(regs::SMPLRT_DIV, divider);
}

Mpu6050RawData Mpu6050Driver::read_raw()
{
  Mpu6050RawData raw;
  uint8_t buffer[14];  // 6 accel + 2 temp + 6 gyro
  
  if (!i2c_read_bytes(regs::ACCEL_XOUT_H, buffer, 14)) {
    // Return zeros on failure
    return raw;
  }
  
  // Combine high and low bytes (big-endian)
  raw.accel_x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
  raw.accel_y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
  raw.accel_z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
  raw.temperature = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);
  raw.gyro_x = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
  raw.gyro_y = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
  raw.gyro_z = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);
  
  return raw;
}

Mpu6050Data Mpu6050Driver::read(double dt)
{
  Mpu6050RawData raw = read_raw();
  scale_data(raw);
  
  // Update Mahony filter
  mahony_update(data_.accel_x, data_.accel_y, data_.accel_z,
                data_.gyro_x, data_.gyro_y, data_.gyro_z, dt);
  compute_euler_angles();
  
  return data_;
}

bool Mpu6050Driver::calibrate_gyro(size_t samples)
{
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  
  for (size_t i = 0; i < samples; ++i) {
    Mpu6050RawData raw = read_raw();
    sum_x += raw.gyro_x * gyro_scale_;
    sum_y += raw.gyro_y * gyro_scale_;
    sum_z += raw.gyro_z * gyro_scale_;
    usleep(1000);  // 1ms delay between samples
  }
  
  gyro_bias_x_ = sum_x / samples;
  gyro_bias_y_ = sum_y / samples;
  gyro_bias_z_ = sum_z / samples;
  
  return true;
}

void Mpu6050Driver::set_accel_bias(double x, double y, double z)
{
  accel_bias_x_ = x;
  accel_bias_y_ = y;
  accel_bias_z_ = z;
}

void Mpu6050Driver::set_gyro_bias(double x, double y, double z)
{
  gyro_bias_x_ = x;
  gyro_bias_y_ = y;
  gyro_bias_z_ = z;
}

void Mpu6050Driver::reset_filter()
{
  mahony_q0_ = 1.0;
  mahony_q1_ = 0.0;
  mahony_q2_ = 0.0;
  mahony_q3_ = 0.0;
  mahony_integral_x_ = 0.0;
  mahony_integral_y_ = 0.0;
  mahony_integral_z_ = 0.0;
  
  data_.q_w = 1.0;
  data_.q_x = 0.0;
  data_.q_y = 0.0;
  data_.q_z = 0.0;
}

// Private methods

bool Mpu6050Driver::i2c_write_byte(uint8_t reg, uint8_t data)
{
  if (fd_ < 0) return false;
  
  uint8_t buffer[2] = {reg, data};
  return write(fd_, buffer, 2) == 2;
}

bool Mpu6050Driver::i2c_read_bytes(uint8_t reg, uint8_t * buffer, size_t length)
{
  if (fd_ < 0) return false;
  
  // Write register address
  if (write(fd_, &reg, 1) != 1) {
    return false;
  }
  
  // Read data
  return read(fd_, buffer, length) == static_cast<ssize_t>(length);
}

bool Mpu6050Driver::i2c_read_byte(uint8_t reg, uint8_t & data)
{
  return i2c_read_bytes(reg, &data, 1);
}

void Mpu6050Driver::scale_data(const Mpu6050RawData & raw)
{
  // Acceleration (m/s²)
  data_.accel_x = raw.accel_x * accel_scale_ - accel_bias_x_;
  data_.accel_y = raw.accel_y * accel_scale_ - accel_bias_y_;
  data_.accel_z = raw.accel_z * accel_scale_ - accel_bias_z_;
  
  // Angular velocity (rad/s)
  data_.gyro_x = raw.gyro_x * gyro_scale_ - gyro_bias_x_;
  data_.gyro_y = raw.gyro_y * gyro_scale_ - gyro_bias_y_;
  data_.gyro_z = raw.gyro_z * gyro_scale_ - gyro_bias_z_;
  
  // Temperature (°C): (raw - offset) / sensitivity + 35
  data_.temperature = (raw.temperature / 340.0) + 36.53;
}

void Mpu6050Driver::mahony_update(double ax, double ay, double az,
                                   double gx, double gy, double gz, double dt)
{
  // Normalize accelerometer measurement
  double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm > 0.0) {
    norm = 1.0 / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
  }
  
  // Estimated direction of gravity in body frame (from quaternion)
  double vx = 2.0 * (mahony_q1_ * mahony_q3_ - mahony_q0_ * mahony_q2_);
  double vy = 2.0 * (mahony_q0_ * mahony_q1_ + mahony_q2_ * mahony_q3_);
  double vz = mahony_q0_ * mahony_q0_ - mahony_q1_ * mahony_q1_ -
              mahony_q2_ * mahony_q2_ + mahony_q3_ * mahony_q3_;
  
  // Error is sum of cross product between estimated and measured direction of gravity
  double ex = ay * vz - az * vy;
  double ey = az * vx - ax * vz;
  double ez = ax * vy - ay * vx;
  
  // Accumulate integral error
  mahony_integral_x_ += ex * MAHONY_KI_ * dt;
  mahony_integral_y_ += ey * MAHONY_KI_ * dt;
  mahony_integral_z_ += ez * MAHONY_KI_ * dt;
  
  // Apply feedback terms
  gx += MAHONY_KP_ * ex + mahony_integral_x_;
  gy += MAHONY_KP_ * ey + mahony_integral_y_;
  gz += MAHONY_KP_ * ez + mahony_integral_z_;
  
  // Integrate rate of change of quaternion
  double q0_dot = 0.5 * (-mahony_q1_ * gx - mahony_q2_ * gy - mahony_q3_ * gz);
  double q1_dot = 0.5 * (mahony_q0_ * gx + mahony_q2_ * gz - mahony_q3_ * gy);
  double q2_dot = 0.5 * (mahony_q0_ * gy - mahony_q1_ * gz + mahony_q3_ * gx);
  double q3_dot = 0.5 * (mahony_q0_ * gz + mahony_q1_ * gy - mahony_q2_ * gx);
  
  // Pre-multiply by dt
  q0_dot *= dt;
  q1_dot *= dt;
  q2_dot *= dt;
  q3_dot *= dt;
  
  // Integrate to yield quaternion
  mahony_q0_ += q0_dot;
  mahony_q1_ += q1_dot;
  mahony_q2_ += q2_dot;
  mahony_q3_ += q3_dot;
  
  // Normalize quaternion
  norm = std::sqrt(mahony_q0_ * mahony_q0_ + mahony_q1_ * mahony_q1_ +
                   mahony_q2_ * mahony_q2_ + mahony_q3_ * mahony_q3_);
  if (norm > 0.0) {
    norm = 1.0 / norm;
    mahony_q0_ *= norm;
    mahony_q1_ *= norm;
    mahony_q2_ *= norm;
    mahony_q3_ *= norm;
  }
  
  // Update data
  data_.q_w = mahony_q0_;
  data_.q_x = mahony_q1_;
  data_.q_y = mahony_q2_;
  data_.q_z = mahony_q3_;
}

void Mpu6050Driver::compute_euler_angles()
{
  // Convert quaternion to Euler angles (roll, pitch, yaw)
  double q0 = mahony_q0_, q1 = mahony_q1_, q2 = mahony_q2_, q3 = mahony_q3_;
  
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
  double cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
  data_.roll = std::atan2(sinr_cosp, cosr_cosp);
  
  // Pitch (y-axis rotation)
  double sinp = 2.0 * (q0 * q2 - q3 * q1);
  if (std::abs(sinp) >= 1.0) {
    data_.pitch = std::copysign(M_PI / 2.0, sinp);  // use 90 degrees if out of range
  } else {
    data_.pitch = std::asin(sinp);
  }
  
  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
  double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
  data_.yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace dog_sensors_cpp
