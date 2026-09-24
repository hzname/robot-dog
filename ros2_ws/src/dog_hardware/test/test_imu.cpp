#include <gtest/gtest.h>

#include <cmath>

#include "dog_hardware/imu_sensor.hpp"

using dog_hardware::AttitudeFilter;
using dog_hardware::AxisMap;
using dog_hardware::GyroBias;
using dog_hardware::ImuReading;

namespace
{
constexpr double kG = 9.80665;

ImuReading tilted(double roll, double pitch)
{
  ImuReading r;
  r.accel = {-kG * std::sin(pitch), kG * std::cos(pitch) * std::sin(roll),
    kG * std::cos(pitch) * std::cos(roll)};
  return r;
}
}  // namespace

TEST(Imu, ConvertsRawRegisters)
{
  // accel x = +1 g, z = -0.5 g; gyro y = +131 LSB = 1 deg/s
  const uint8_t raw[14] = {0x40, 0x00, 0, 0, 0xE0, 0x00, 0, 0, 0, 0, 0, 131, 0, 0};
  const ImuReading r = dog_hardware::mpu::convert(raw);
  EXPECT_NEAR(r.accel[0], kG, 1e-9);
  EXPECT_NEAR(r.accel[2], -0.5 * kG, 1e-9);
  EXPECT_NEAR(r.gyro[1], M_PI / 180.0, 1e-9);
  EXPECT_TRUE(dog_hardware::mpu::knownWhoAmI(0x68));
  EXPECT_FALSE(dog_hardware::mpu::knownWhoAmI(0x00));
}

TEST(Imu, AxisMapRemapsAndRejectsNonsense)
{
  const AxisMap m("-y,x,z");
  const auto v = m.apply({1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(v[0], -2.0);
  EXPECT_DOUBLE_EQ(v[1], 1.0);
  EXPECT_DOUBLE_EQ(v[2], 3.0);
  EXPECT_THROW(AxisMap("x,x,z"), std::invalid_argument);
  EXPECT_THROW(AxisMap("x,y"), std::invalid_argument);
  EXPECT_THROW(AxisMap("x,y,w"), std::invalid_argument);
}

TEST(Imu, StaticTiltFromGravity)
{
  AttitudeFilter f;
  f.update(tilted(0.1, -0.2), 0.0);
  ASSERT_TRUE(f.initialised());
  EXPECT_NEAR(f.roll(), 0.1, 1e-9);
  EXPECT_NEAR(f.pitch(), -0.2, 1e-9);
  // quaternion of pure pitch
  AttitudeFilter g;
  g.update(tilted(0.0, 0.3), 0.0);
  const auto q = g.quaternion();
  EXPECT_NEAR(q[1], std::sin(0.15), 1e-9);
  EXPECT_NEAR(q[3], std::cos(0.15), 1e-9);
}

TEST(Imu, GyroTracksFastMotionAccelCorrectsDrift)
{
  AttitudeFilter f(1.0);
  f.update(tilted(0.0, 0.0), 0.0);
  // Pitch up at 0.5 rad/s for 0.4 s: gyro carries it, accel follows.
  const double dt = 0.01;
  for (int i = 1; i <= 40; ++i) {
    ImuReading r = tilted(0.0, 0.005 * i);
    r.gyro = {0.0, 0.5, 0.0};
    f.update(r, dt);
  }
  EXPECT_NEAR(f.pitch(), 0.2, 0.01);
  // Gyro bias left uncorrected: the accelerometer bounds the drift.
  for (int i = 0; i < 2000; ++i) {
    ImuReading r = tilted(0.05, 0.2);
    r.gyro = {0.01, 0.0, 0.0};
    f.update(r, dt);
  }
  EXPECT_NEAR(f.roll(), 0.05, 0.02);
  EXPECT_NEAR(f.pitch(), 0.2, 0.01);
}

TEST(Imu, ImpactsDoNotPullTheAttitude)
{
  AttitudeFilter f(0.5, 0.15);
  f.update(tilted(0.0, 0.0), 0.0);
  ImuReading kick = tilted(0.0, 0.0);
  kick.accel = {-6.0, 0.0, 14.0};  // footfall: 1.55 g, looks like 23 deg pitch
  for (int i = 0; i < 20; ++i) {f.update(kick, 0.01);}
  EXPECT_NEAR(f.pitch(), 0.0, 1e-9);
}

TEST(Imu, GyroBiasRestartsOnMotion)
{
  GyroBias b(10);
  for (int i = 0; i < 5; ++i) {b.add({0.01, -0.02, 0.0});}
  b.add({0.5, 0.0, 0.0});  // robot moved
  EXPECT_FALSE(b.done());
  for (int i = 0; i < 9; ++i) {b.add({0.01, -0.02, 0.003});}
  EXPECT_FALSE(b.done());
  b.add({0.01, -0.02, 0.003});
  EXPECT_TRUE(b.done());
  EXPECT_NEAR(b.bias()[0], 0.01, 1e-9);
  EXPECT_NEAR(b.bias()[2], 0.003, 1e-9);
}
