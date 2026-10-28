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

// The shipped knee row of servos.yaml for a left (+1) or right (-1) leg: the servo sits in the
// thigh block and sets s = thigh + knee, window s = -32.5 +- 47.5 = -80..+15 deg, knee against the
// thigh -140..-40 deg (CAD collision map 2026-10-13, docs/CALIBRATION.md). The constants mirror
// servos.yaml; tools/robot_setup/test/test_yaml_contract.py pins the yaml side.
ServoCalibration coupledKnee(int direction)
{
  ServoCalibration c;
  c.channel = 2;
  c.direction = direction;
  c.offset_deg = -32.5;
  c.range_deg = 95.0;
  c.min_deg = -140.0;
  c.max_deg = -40.0;
  c.pulse_min_us = direction > 0 ? 1039.5 : 803.5;
  c.pulse_max_us = direction > 0 ? 1936.5 : 1700.5;
  c.coupled_to = 0;
  c.coupling = 1.0;
  return c;
}

/// Pulse for thigh q1 and knee q2 given in degrees.
double kneePulse(const ServoCalibration & c, double q1_deg, double q2_deg, bool * clamped)
{
  return jointToPulseUs(c, q2_deg * kDeg, clamped, q1_deg * kDeg);
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

// ------------------------------------------------- knee coupled to the thigh

TEST(CoupledKnee, CentreOfTheWindowIsNotClamped)
{
  for (const int dir : {1, -1}) {
    SCOPED_TRACE(dir);
    const ServoCalibration c = coupledKnee(dir);
    bool clamped = true;
    // s = 57.5 - 90 = -32.5 = offset_deg
    EXPECT_NEAR(kneePulse(c, 57.5, -90.0, &clamped), c.centerUs(), 1e-9);
    EXPECT_FALSE(clamped);
    EXPECT_NEAR(c.centerUs(), dir > 0 ? 1488.0 : 1252.0, 1e-9);
  }
}

TEST(CoupledKnee, MountPoseIsTheCentrePulseOfTheServo)
{
  // The assembly reference "all servos centred (1370 us) = thigh 45, knee -90" is s = -45.
  for (const int dir : {1, -1}) {
    SCOPED_TRACE(dir);
    const ServoCalibration c = coupledKnee(dir);
    bool clamped = true;
    EXPECT_NEAR(kneePulse(c, 45.0, -90.0, &clamped), 1370.0, 0.1);
    EXPECT_FALSE(clamped);
  }
}

TEST(CoupledKnee, WindowEdgesFollowTheShinAngle)
{
  for (const int dir : {1, -1}) {
    SCOPED_TRACE(dir);
    const ServoCalibration c = coupledKnee(dir);
    const double at_max = dir > 0 ? c.pulse_max_us : c.pulse_min_us;  // s = +15 side
    const double at_min = dir > 0 ? c.pulse_min_us : c.pulse_max_us;  // s = -80 side
    bool clamped = true;
    // Half a degree inside the window: free; half a degree outside: pinned to the edge pulse.
    kneePulse(c, 74.5, -60.0, &clamped);  // s = +14.5
    EXPECT_FALSE(clamped);
    kneePulse(c, 20.5, -100.0, &clamped);  // s = -79.5
    EXPECT_FALSE(clamped);
    EXPECT_NEAR(kneePulse(c, 75.5, -60.0, &clamped), at_max, 1e-9);  // s = +15.5
    EXPECT_TRUE(clamped);
    EXPECT_NEAR(kneePulse(c, 19.5, -100.0, &clamped), at_min, 1e-9);  // s = -80.5
    EXPECT_TRUE(clamped);
  }
}

TEST(CoupledKnee, ThighPlusKneeBeyondTheWindowIsClamped)
{
  for (const int dir : {1, -1}) {
    SCOPED_TRACE(dir);
    const ServoCalibration c = coupledKnee(dir);
    bool clamped = false;
    // q1 = 100, q2 = -60: the knee is inside -140..-40 but the sum 40 deg > 15 deg.
    EXPECT_NEAR(kneePulse(c, 100.0, -60.0, &clamped), dir > 0 ? c.pulse_max_us : c.pulse_min_us, 1e-9);
    EXPECT_TRUE(clamped);
    // q1 = 10, q2 = -100: s = -90 deg < -80 deg, the other edge.
    EXPECT_NEAR(kneePulse(c, 10.0, -100.0, &clamped), dir > 0 ? c.pulse_min_us : c.pulse_max_us, 1e-9);
    EXPECT_TRUE(clamped);
  }
}

TEST(CoupledKnee, KneeLimitAppliesBeforeTheSum)
{
  for (const int dir : {1, -1}) {
    SCOPED_TRACE(dir);
    const ServoCalibration c = coupledKnee(dir);
    bool clamped = false;
    // The knee beyond -140 is cut to -140 first; the sum is then inside the window.
    const double bent = kneePulse(c, 80.0, -150.0, &clamped);
    EXPECT_TRUE(clamped);
    bool edge_clamped = true;
    EXPECT_NEAR(bent, kneePulse(c, 80.0, -140.0, &edge_clamped), 1e-9);
    EXPECT_FALSE(edge_clamped);
    // Same on the straight side: -30 is cut to -40.
    const double straight = kneePulse(c, 30.0, -30.0, &clamped);
    EXPECT_TRUE(clamped);
    EXPECT_NEAR(straight, kneePulse(c, 30.0, -40.0, &edge_clamped), 1e-9);
    EXPECT_FALSE(edge_clamped);
  }
}

TEST(CoupledKnee, CadStanceWalkAndLieEnvelopeIsNotClamped)
{
  // (thigh q1, knee q2) [deg]: lie 0.08, min 0.10, stance 0.15, max 0.18 and the walk corners
  // (step x -60/-30/+30/+60 mm, lift 0/20 mm); `stance_envelope` of
  // .planning/quick/261013-e4x-*/thigh-knee-limits.json, s = q1 + q2 in -68.6..-16.2.
  const double poses[12][2] = {
    {79.9, -137.5}, {71.3, -125.7}, {50.9, -92.7}, {36.9, -67.8},
    {67.7, -83.8}, {78.6, -97.6}, {61.0, -90.5}, {70.8, -104.4},
    {38.3, -90.5}, {44.8, -104.4}, {24.1, -83.8}, {29.0, -97.6}};
  for (const int dir : {1, -1}) {
    SCOPED_TRACE(dir);
    const ServoCalibration c = coupledKnee(dir);
    for (const auto & pose : poses) {
      bool clamped = true;
      const double us = kneePulse(c, pose[0], pose[1], &clamped);
      EXPECT_FALSE(clamped) << pose[0] << " " << pose[1];
      EXPECT_GT(us, c.pulse_min_us);
      EXPECT_LT(us, c.pulse_max_us);
    }
  }
}

TEST(CoupledKnee, DriverClampsThePairAndCountsIt)
{
  auto bus = std::make_shared<MockBus>();
  ServoCalibration thigh;
  thigh.channel = 1;
  thigh.offset_deg = 45.0;
  thigh.min_deg = -45.0;
  thigh.max_deg = 135.0;
  const ServoCalibration calf = coupledKnee(1);
  ServoDriver driver(bus, {"lf_thigh_joint", "lf_calf_joint"}, {thigh, calf}, DriverParams{});
  const std::vector<std::string> names = {"lf_thigh_joint", "lf_calf_joint"};

  // thigh 100, knee -60: the sum 40 deg is beyond the window (+15 deg).
  EXPECT_EQ(driver.setTargets(names, {100.0 * kDeg, -60.0 * kDeg}, 0.0), 2);
  driver.update(0.0);
  EXPECT_NEAR(driver.pulses()[1], calf.pulse_max_us, 1e-9);
  EXPECT_EQ(driver.clampedCount(), 1);

  // The centre of the window: thigh 57.5, knee -90 (s = -32.5), reached by slew limiting.
  EXPECT_EQ(driver.setTargets(names, {57.5 * kDeg, -90.0 * kDeg}, 0.05), 2);
  for (int k = 1; k <= 20; ++k) {driver.update(0.05 * k);}
  EXPECT_NEAR(driver.pulses()[1], calf.centerUs(), 1e-6);
  EXPECT_EQ(driver.clampedCount(), 0);
}

// ------------------------------------------------------------- bus faults
//
// A write that comes back false means the chip kept its previous pulse. The
// driver must count it, retry the channel, and after three consecutive bad
// frames latch its own emergency release (SAF-01).

class FaultTest : public ::testing::Test
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

TEST_F(FaultTest, AFailedWriteIsRetriedUntilThePulseLands)
{
  bus->failNext(1);
  driver->setTargets({"a0"}, {0.3}, 0.0);
  driver->update(0.0);  // power-on write fails: the output stays where it was
  EXPECT_DOUBLE_EQ(bus->pulse(0), 0.0);
  EXPECT_EQ(driver->channelFaults(0), 1);
  EXPECT_EQ(driver->busFaults(), 1);
  EXPECT_EQ(driver->failedFrames(), 1);
  EXPECT_FALSE(driver->fault());

  driver->update(0.1);  // nothing moved, but the channel is dirty: retry
  EXPECT_NEAR(bus->pulse(0), jointToPulseUs(driver->calibrations()[0], 0.3), 1e-9);
  EXPECT_EQ(bus->attempts(0), 2);
  EXPECT_EQ(driver->failedFrames(), 0);
  EXPECT_FALSE(driver->fault());
}

TEST_F(FaultTest, TwoBadFramesAreCountedButDoNotLatch)
{
  bus->failNext(2);
  driver->setTargets({"a0"}, {0.3}, 0.0);
  driver->update(0.0);
  driver->update(0.1);
  EXPECT_FALSE(driver->fault());
  EXPECT_EQ(driver->failedFrames(), 2);
  driver->update(0.2);  // third attempt succeeds
  EXPECT_FALSE(driver->fault());
  EXPECT_EQ(driver->failedFrames(), 0);
  EXPECT_NEAR(bus->pulse(0), jointToPulseUs(driver->calibrations()[0], 0.3), 1e-9);
}

TEST_F(FaultTest, ThreeBadFramesLatchTheDriver)
{
  driver->setTargets({"a0"}, {0.0}, 0.0);
  driver->update(0.0);  // enabled and written once, clean
  ASSERT_FALSE(driver->fault());
  bus->failNext(100);
  for (int f = 1; f <= ServoDriver::kFaultFrames; ++f) {
    driver->setTargets({"a0"}, {0.1 * f}, 0.1 * f);  // keep the joint moving
    driver->update(0.1 * f);
    if (f < ServoDriver::kFaultFrames) {
      EXPECT_FALSE(driver->fault()) << "latched after " << f << " frame(s)";
    }
  }
  EXPECT_TRUE(driver->fault());
  EXPECT_EQ(driver->failedFrames(), ServoDriver::kFaultFrames);
  EXPECT_EQ(driver->channelFaults(0), ServoDriver::kFaultFrames);
  EXPECT_EQ(driver->channelFaults(1), 0) << "another channel must not be blamed";
  EXPECT_FALSE(driver->faultReason().empty());
}

TEST_F(FaultTest, LatchedDriverIgnoresCommandsAndStopsWriting)
{
  driver->setTargets({"a0"}, {0.0}, 0.0);
  driver->update(0.0);
  bus->failNext(100);
  for (int f = 1; f <= ServoDriver::kFaultFrames; ++f) {
    driver->setTargets({"a0"}, {0.1 * f}, 0.1 * f);
    driver->update(0.1 * f);
  }
  ASSERT_TRUE(driver->fault());
  EXPECT_FALSE(driver->anyEnabled());
  for (int i = 0; i < 6; ++i) {EXPECT_DOUBLE_EQ(bus->pulse(i), 0.0);}

  const int attempts = bus->attempts(0);
  EXPECT_EQ(driver->setTargets({"a0"}, {0.5}, 2.0), 0);
  driver->update(2.0);
  EXPECT_EQ(bus->attempts(0), attempts) << "a latched driver must not touch the bus";

  bus->clearFailures();  // the wire is fixed before anyone asks to move again
  driver->clearFault();  // clearing the latch does not arm the driver
  EXPECT_FALSE(driver->fault());
  EXPECT_FALSE(driver->armed()) << "a released fault must leave the driver disarmed";
  EXPECT_EQ(driver->setTargets({"a0"}, {0.5}, 2.1), 1);
  driver->update(2.1);
  EXPECT_FALSE(driver->enabled(0)) << "a disarmed driver must not power the joint";

  driver->arm();
  driver->update(2.2);
  EXPECT_TRUE(driver->enabled(0));
  EXPECT_NEAR(bus->pulse(0), jointToPulseUs(driver->calibrations()[0], 0.5), 1e-9);
}

// ------------------------------------------------------------ e-stop proof
//
// An E-STOP counts only when the outputs are confirmed off: disableAll is
// checked, retried, and falls back to per-channel disables (SAF-02).

class RelaxTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bus = std::make_shared<MockBus>();
    names = {"a0", "a1"};
    std::vector<ServoCalibration> cals;
    for (int i = 0; i < 2; ++i) {
      ServoCalibration c;
      c.channel = i;
      cals.push_back(c);
    }
    driver = std::make_unique<ServoDriver>(bus, names, cals, DriverParams{});
  }

  std::shared_ptr<MockBus> bus;
  std::vector<std::string> names;
  std::unique_ptr<ServoDriver> driver;
};

TEST_F(RelaxTest, OneFailedDisableAllIsRetried)
{
  driver->setTargets({"a0"}, {0.2}, 0.0);
  bus->failDisableAll(1);
  EXPECT_TRUE(driver->relax());
  EXPECT_EQ(driver->relaxFailures(), 0);
  EXPECT_TRUE(driver->outputsConfirmedOff());
  EXPECT_DOUBLE_EQ(bus->pulse(0), 0.0);
}

TEST_F(RelaxTest, AFailedAllCallFallsBackToChannels)
{
  driver->setTargets({"a0"}, {0.2}, 0.0);
  bus->failDisableAll(2);  // both attempts fail, the channels must be tried
  EXPECT_TRUE(driver->relax());
  for (int ch = 0; ch < kPwmChannels; ++ch) {
    EXPECT_EQ(bus->disableAttempts(ch), 1) << "channel " << ch << " was not disabled";
    EXPECT_DOUBLE_EQ(bus->pulse(ch), 0.0);
  }
  EXPECT_EQ(driver->relaxFailures(), 0);
}

TEST_F(RelaxTest, AnUnconfirmedRelaxIsReportedAsSuch)
{
  driver->setTargets({"a0"}, {0.2}, 0.0);
  bus->failDisableAll(2);
  for (int ch = 0; ch < kPwmChannels; ++ch) {bus->failDisable(ch, 1);}
  EXPECT_FALSE(driver->relax());
  EXPECT_EQ(driver->relaxFailures(), 1);
  EXPECT_FALSE(driver->outputsConfirmedOff());
}

TEST_F(RelaxTest, EstopIsDoneOnlyWhenOutputsAreConfirmed)
{
  driver->setTargets({"a0"}, {0.2}, 0.0);
  bus->failDisableAll(2);
  for (int ch = 0; ch < kPwmChannels; ++ch) {bus->failDisable(ch, 1);}
  driver->setEstop(true);
  EXPECT_TRUE(driver->estop());
  EXPECT_FALSE(driver->estopDone()) << "an unconfirmed release is not an E-STOP";

  bus->failDisableAll(0);
  driver->setEstop(true);  // the operator asks again, the bus answers this time
  EXPECT_TRUE(driver->estopDone());
  EXPECT_TRUE(driver->outputsConfirmedOff());
}

// ------------------------------------------------------------- arming
//
// The robot only starts with a hand on it: a disarmed driver remembers the pose
// but leaves the bus alone, and arming is a separate, explicit step (SAF-12).

class ArmTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bus = std::make_shared<MockBus>();
    names = {"a0", "a1"};
    std::vector<ServoCalibration> cals;
    for (int i = 0; i < 2; ++i) {
      ServoCalibration c;
      c.channel = i;
      cals.push_back(c);
    }
    DriverParams p;
    p.max_joint_speed = 1.0;
    p.enable_stagger = 0.1;
    p.armed_on_start = false;
    driver = std::make_unique<ServoDriver>(bus, names, cals, p);
  }

  std::shared_ptr<MockBus> bus;
  std::vector<std::string> names;
  std::unique_ptr<ServoDriver> driver;
};

TEST_F(ArmTest, ADisarmedDriverNeverTouchesTheBus)
{
  EXPECT_FALSE(driver->armed());
  EXPECT_EQ(driver->setTargets({"a0", "a1"}, {0.2, 0.3}, 1.0), 2);
  for (int i = 0; i < 10; ++i) {driver->update(1.0 + 0.1 * i);}
  EXPECT_EQ(bus->writes(), 0) << "nothing may be written before the driver is armed";
  EXPECT_FALSE(driver->anyEnabled());
  EXPECT_EQ(driver->positions()[0], 0.0);  // the pose is remembered, not applied
}

TEST_F(ArmTest, ArmingAppliesTheRememberedPose)
{
  driver->setTargets({"a0"}, {0.2}, 1.0);
  driver->arm();
  driver->update(1.0);
  EXPECT_TRUE(driver->enabled(0));
  EXPECT_NEAR(bus->pulse(0), jointToPulseUs(driver->calibrations()[0], 0.2), 1e-9);
}

TEST_F(ArmTest, ArmingWithoutATargetPowersNothing)
{
  driver->arm();
  driver->update(1.0);
  EXPECT_TRUE(driver->armed());
  EXPECT_EQ(bus->writes(), 0);
  EXPECT_FALSE(driver->anyEnabled());
}

TEST_F(ArmTest, DisarmingDropsTheOutputsAndTheOrderOfPowerOn)
{
  driver->arm();
  driver->setTargets({"a0"}, {0.2}, 1.0);
  driver->update(1.0);
  ASSERT_TRUE(driver->enabled(0));

  driver->disarm();
  EXPECT_FALSE(driver->armed());
  EXPECT_FALSE(driver->anyEnabled());
  EXPECT_DOUBLE_EQ(bus->pulse(0), 0.0);

  driver->setTargets({"a0"}, {0.3}, 2.0);
  driver->update(2.0);
  EXPECT_FALSE(driver->enabled(0)) << "a command while disarmed must not power anything";
}

TEST_F(ArmTest, TheFirstMoveAfterArmingIsSlewLimited)
{
  driver->setTargets({"a0"}, {0.0}, 0.0);
  driver->arm();
  driver->update(0.0);
  ASSERT_TRUE(driver->enabled(0));

  driver->setTargets({"a0"}, {0.5}, 0.0);
  driver->update(0.01);  // 1 rad/s * 0.01 s = 0.01 rad, not the whole 0.5
  EXPECT_NEAR(driver->positions()[0], 0.01, 1e-9);
  EXPECT_NEAR(bus->pulse(0), jointToPulseUs(driver->calibrations()[0], 0.01), 1e-9);
}

// --- CAL-04: a live calibration change may not snap an enabled servo -----------

TEST(LiveCalibrationTest, AnEnabledChannelReportsHowFarItWouldJump)
{
  auto bus = std::make_shared<MockBus>();
  DriverParams p;
  p.armed_on_start = true;
  const ServoCalibration c = knee();
  ServoDriver driver(bus, {"lf_calf_joint"}, {c}, p);
  driver.setTargets({"lf_calf_joint"}, {-90.0 * kDeg}, 0.0);
  for (int step = 1; step <= 5; ++step) {
    driver.update(step * 0.1);
  }
  ASSERT_TRUE(driver.anyEnabled());

  ServoCalibration nudge = c;
  nudge.offset_deg += 1.0;
  EXPECT_NEAR(driver.liveCalibrationStepDeg(0, nudge), 1.0, 0.2);

  ServoCalibration wide = c;
  wide.offset_deg += 25.0;             // inside the 30 degree limit: allowed
  EXPECT_NEAR(driver.liveCalibrationStepDeg(0, wide), 25.0, 0.5);

  ServoCalibration jump = c;
  jump.offset_deg += 45.0;             // beyond it: the node refuses
  EXPECT_GT(driver.liveCalibrationStepDeg(0, jump), 30.0);
}

TEST(LiveCalibrationTest, ADisabledChannelCannotJump)
{
  auto bus = std::make_shared<MockBus>();
  ServoDriver driver(bus, {"lf_calf_joint"}, {knee()}, DriverParams{});  // not armed
  ServoCalibration c = knee();
  c.offset_deg += 40.0;
  EXPECT_DOUBLE_EQ(driver.liveCalibrationStepDeg(0, c), 0.0);
}
