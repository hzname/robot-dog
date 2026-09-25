#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "dog_hardware/servo_bus.hpp"
#include "dog_hardware/servo_driver.hpp"

using namespace dog_hardware;

namespace
{
constexpr double kDeg = M_PI / 180.0;

ServoCalibration knee()
{
  ServoCalibration c;
  c.channel = 2;
  c.direction = 1;
  c.offset_deg = -90.0;
  c.min_deg = -165.0;
  c.max_deg = -15.0;
  return c;
}
}  // namespace

TEST(ServoMap, CentreAndScale)
{
  const ServoCalibration c = knee();
  EXPECT_DOUBLE_EQ(c.centerUs(), 1370.0);  // v1 measured range 520..2220 us
  EXPECT_NEAR(jointToPulseUs(c, -90.0 * kDeg), 1370.0, 1e-9);
  EXPECT_NEAR(jointToPulseUs(c, -45.0 * kDeg), 1370.0 + 45.0 * c.usPerDeg(), 1e-9);
  ServoCalibration inv = c;
  inv.direction = -1;
  EXPECT_NEAR(jointToPulseUs(inv, -45.0 * kDeg), 1370.0 - 45.0 * c.usPerDeg(), 1e-9);
}

TEST(ServoMap, RoundTrip)
{
  for (int dir : {1, -1}) {
    ServoCalibration c = knee();
    c.direction = dir;
    c.offset_deg = -80.0;
    for (double deg = -160.0; deg <= -20.0; deg += 5.0) {
      bool clamped = true;
      const double us = jointToPulseUs(c, deg * kDeg, &clamped);
      EXPECT_FALSE(clamped);
      EXPECT_NEAR(pulseUsToJoint(c, us), deg * kDeg, 1e-12);
    }
  }
}

TEST(ServoMap, ClampsToJointLimitsAndServoRange)
{
  ServoCalibration c = knee();
  bool clamped = false;
  EXPECT_NEAR(jointToPulseUs(c, 0.0, &clamped), jointToPulseUs(c, -15.0 * kDeg), 1e-9);
  EXPECT_TRUE(clamped);
  c.min_deg = -300;
  c.max_deg = 300;
  EXPECT_NEAR(jointToPulseUs(c, 170.0 * kDeg, &clamped), c.pulse_max_us, 1e-9);
  EXPECT_TRUE(clamped);
  EXPECT_NEAR(jointToPulseUs(c, -300.0 * kDeg, &clamped), c.pulse_min_us, 1e-9);
}

TEST(ServoMap, Validation)
{
  ServoCalibration c = knee();
  EXPECT_TRUE(c.validate().empty());
  c.direction = 0;
  EXPECT_FALSE(c.validate().empty());
  c = knee();
  c.channel = 16;
  EXPECT_FALSE(c.validate().empty());
  c = knee();
  c.pulse_min_us = 2500;
  EXPECT_FALSE(c.validate().empty());
}

TEST(Pca9685Math, PrescaleAndTicks)
{
  EXPECT_EQ(Pca9685Bus::prescaleFor(50.0, 25e6), 121);  // same as v1
  EXPECT_NEAR(Pca9685Bus::frequencyFor(121, 25e6), 50.03, 0.01);
  EXPECT_EQ(Pca9685Bus::ticksFor(1500.0, 50.0), 307);
  EXPECT_EQ(Pca9685Bus::ticksFor(0.0, 50.0), 0);
  EXPECT_EQ(Pca9685Bus::ticksFor(1e9, 50.0), 4095);
}

class DriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bus = std::make_shared<MockBus>();
    names = {"a0", "a1", "a2", "b0", "b1", "b2"};
    std::vector<ServoCalibration> cals;
    for (int i = 0; i < 6; ++i) {
      ServoCalibration c;
      c.channel = i;
      cals.push_back(c);
    }
    DriverParams p;
    p.max_joint_speed = 1.0;
    p.enable_stagger = 0.1;
    driver = std::make_unique<ServoDriver>(bus, names, cals, p);
  }

  std::shared_ptr<MockBus> bus;
  std::vector<std::string> names;
  std::unique_ptr<ServoDriver> driver;
};

TEST_F(DriverTest, StaggeredPowerOnJumpsToTarget)
{
  EXPECT_EQ(driver->setTargets(names, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, 10.0), 6);
  driver->update(10.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(driver->enabled(i));
    EXPECT_NEAR(bus->pulse(i), jointToPulseUs(driver->calibrations()[i], 0.1 * (i + 1)), 1e-9);
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_FALSE(driver->enabled(i));
    EXPECT_EQ(bus->pulse(i), 0.0);
  }
  driver->update(10.1);
  for (int i = 3; i < 6; ++i) {EXPECT_TRUE(driver->enabled(i));}
}

TEST_F(DriverTest, SlewRateLimited)
{
  driver->setTargets({"a0"}, {0.0}, 0.0);
  driver->update(0.0);
  driver->setTargets({"a0"}, {1.0}, 0.0);
  driver->update(0.1);
  EXPECT_NEAR(driver->positions()[0], 0.1, 1e-12);  // 1 rad/s * 0.1 s
  for (int i = 2; i <= 20; ++i) {driver->update(0.1 * i);}
  EXPECT_NEAR(driver->positions()[0], 1.0, 1e-12);
}

TEST_F(DriverTest, EstopTurnsOffAndBlocks)
{
  driver->setTargets(names, std::vector<double>(6, 0.2), 0.0);
  driver->update(0.0);
  driver->update(0.2);
  ASSERT_TRUE(driver->anyEnabled());
  driver->setEstop(true);
  for (int i = 0; i < 6; ++i) {EXPECT_EQ(bus->pulse(i), 0.0);}
  EXPECT_EQ(driver->setTargets(names, std::vector<double>(6, 0.5), 0.3), 0);
  driver->update(0.4);
  EXPECT_FALSE(driver->anyEnabled());
  driver->setEstop(false);
  EXPECT_EQ(driver->setTargets(names, std::vector<double>(6, 0.5), 0.5), 6);
  driver->update(0.5);
  EXPECT_TRUE(driver->enabled(0));
}

TEST_F(DriverTest, IgnoresUnknownAndNonFinite)
{
  EXPECT_EQ(driver->setTargets({"zz", "a1"}, {1.0, NAN}, 0.0), 0);
  EXPECT_EQ(driver->setTargets({"a1"}, {0.3}, 0.0), 1);
}

TEST_F(DriverTest, LiveCalibrationRewritesPulse)
{
  driver->setTargets({"a0"}, {0.0}, 0.0);
  driver->update(0.0);
  const double before = bus->pulse(0);
  ServoCalibration c = driver->calibrations()[0];
  c.offset_deg = 10.0;
  ASSERT_TRUE(driver->setCalibration(0, c));
  EXPECT_NEAR(bus->pulse(0), before - 10.0 * c.usPerDeg(), 1e-9);
  c.channel = 9;
  ASSERT_TRUE(driver->setCalibration(0, c));
  EXPECT_EQ(bus->pulse(0), 0.0);
  EXPECT_GT(bus->pulse(9), 0.0);
  c.direction = 5;
  EXPECT_FALSE(driver->setCalibration(0, c));
}

// ------------------------------------------------------------- linkages

TEST(Linkage, ZeroArmIsDirectDrive)
{
  Linkage l;
  EXPECT_TRUE(l.direct());
  EXPECT_DOUBLE_EQ(l.jointDelta(37.0), 37.0);
  EXPECT_TRUE(l.validate(90.0).empty());
}

TEST(Linkage, ParallelogramTurnsOneToOne)
{
  Linkage l;
  l.servo_arm_mm = 20.0;
  l.axis_distance_mm = 90.0;  // joint arm = servo arm, rod = axis distance
  EXPECT_TRUE(l.validate(90.0).empty()) << l.validate(90.0);
  for (double s = -80.0; s <= 80.0; s += 10.0) {
    EXPECT_NEAR(l.jointDelta(s), s, 1e-9) << s;
  }
}

TEST(Linkage, LongerJointArmGearsDown)
{
  Linkage l;
  l.servo_arm_mm = 15.0;
  l.joint_arm_mm = 25.0;
  l.axis_distance_mm = 90.0;
  ASSERT_TRUE(l.validate(90.0).empty()) << l.validate(90.0);
  // Small angles: ratio ~ servo_arm / joint_arm.
  EXPECT_NEAR(l.jointDelta(2.0) / 2.0, 15.0 / 25.0, 0.02);
  EXPECT_LT(std::abs(l.jointDelta(60.0)), 60.0);
  EXPECT_GT(l.jointDelta(30.0), 0.0);  // same rotational sense
  // This geometry hits a dead point near -75 deg: the usable travel stops there.
  const auto [lo, hi] = l.usableServoRange(90.0);
  EXPECT_GT(lo, -80.0);
  EXPECT_LT(lo, -60.0);
  EXPECT_DOUBLE_EQ(hi, 90.0);
  ServoCalibration c;
  c.linkage = l;
  c.min_deg = -180.0;
  bool clamped = false;
  const double us = jointToPulseUs(c, -60.0 / 180.0 * M_PI, &clamped);  // beyond the dead point
  EXPECT_TRUE(clamped);
  EXPECT_NEAR(us, c.centerUs() + lo * c.usPerDeg(), 1e-6);
}

TEST(Linkage, ImpossibleGeometryIsRejected)
{
  Linkage l;
  l.servo_arm_mm = 20.0;
  l.axis_distance_mm = 0.0;
  EXPECT_FALSE(l.validate(90.0).empty());
  l.axis_distance_mm = 90.0;
  l.rod_mm = 20.0;  // far too short to span 90 mm
  EXPECT_FALSE(l.validate(90.0).empty());
  ServoCalibration c;
  c.linkage = l;
  EXPECT_FALSE(c.validate().empty());
}

TEST(Linkage, PulseRoundTripThroughRod)
{
  ServoCalibration c = knee();
  c.linkage.servo_arm_mm = 15.0;
  c.linkage.joint_arm_mm = 22.0;
  c.linkage.rod_mm = 92.0;
  c.linkage.axis_distance_mm = 90.0;
  ASSERT_TRUE(c.validate().empty()) << c.validate();
  // Rod ratio ~0.68: the joint gets roughly +-55 deg around its offset.
  for (double deg = -130.0; deg <= -50.0; deg += 5.0) {
    bool clamped = true;
    const double us = jointToPulseUs(c, deg * kDeg, &clamped);
    EXPECT_FALSE(clamped) << deg;
    EXPECT_NEAR(pulseUsToJoint(c, us), deg * kDeg, 1e-6) << deg;
  }
  // Beyond what the rod can reach inside the servo travel: clamped, finite.
  bool clamped = false;
  const double us = jointToPulseUs(c, -164.0 * kDeg, &clamped);
  EXPECT_TRUE(clamped);
  EXPECT_TRUE(std::isfinite(us));
}

TEST(Coupling, ServoFollowsParentAngle)
{
  // Knee driven from the body: servo sets (knee + thigh).
  ServoCalibration c = knee();
  c.offset_deg = -45.0;
  c.coupled_to = 0;
  c.coupling = 1.0;
  c.min_deg = -170.0;
  const double us_a = jointToPulseUs(c, -90.0 * kDeg, nullptr, 45.0 * kDeg);
  EXPECT_NEAR(us_a, c.centerUs(), 1e-9);  // -90 + 45 = -45 = offset
  const double us_b = jointToPulseUs(c, -90.0 * kDeg, nullptr, 25.0 * kDeg);
  EXPECT_NEAR(us_b, c.centerUs() - 20.0 * c.usPerDeg(), 1e-9);
  EXPECT_NEAR(pulseUsToJoint(c, us_b, 25.0 * kDeg), -90.0 * kDeg, 1e-12);
}

TEST_F(DriverTest, CoupledServoIsRewrittenWhenParentMoves)
{
  ServoCalibration child = driver->calibrations()[1];
  child.coupled_to = 0;
  child.coupling = 1.0;
  ASSERT_TRUE(driver->setCalibration(1, child));
  driver->setTargets({"a0", "a1"}, {0.0, 0.0}, 0.0);
  driver->update(0.0);
  const double before = bus->pulse(1);
  driver->setTargets({"a0"}, {0.2}, 0.0);  // only the parent moves
  driver->update(0.1);                       // 1 rad/s * 0.1 s
  EXPECT_NEAR(driver->positions()[1], 0.0, 1e-12);
  EXPECT_NEAR(bus->pulse(1) - before, 0.1 * 180.0 / M_PI * child.usPerDeg(), 1e-6);
  EXPECT_NEAR(driver->pulses()[1], bus->pulse(1), 1e-12);
  // Self-coupling is refused.
  child.coupled_to = 1;
  EXPECT_FALSE(driver->setCalibration(1, child));
}
