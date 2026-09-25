#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dog_control/locomotion.hpp"

using dog_control::BodyPose;
using dog_control::forwardKinematics;
using dog_control::kNumLegs;
using dog_control::legSide;
using dog_control::LocomotionController;
using dog_control::LocomotionParams;
using dog_control::Mode;
using dog_control::Vec3;

namespace
{
constexpr double kDt = 0.02;

void run(LocomotionController & c, double seconds)
{
  for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {c.update(kDt);}
}

Vec3 footInBody(const LocomotionController & c, const LocomotionParams & p, int leg)
{
  const auto & q = c.joints();
  const Vec3 f = forwardKinematics(p.leg, legSide(leg), {q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2]});
  return f + c.hipPosition(leg);
}
}  // namespace

TEST(Locomotion, PassiveUntilStand)
{
  LocomotionController c(LocomotionParams{});
  EXPECT_EQ(c.mode(), Mode::PASSIVE);
  EXPECT_FALSE(c.update(kDt));
  c.setVelocity({0.1, 0, 0});
  EXPECT_FALSE(c.update(kDt));
  EXPECT_EQ(c.mode(), Mode::PASSIVE);
}

TEST(Locomotion, StandUpReachesStandHeight)
{
  LocomotionParams p;
  LocomotionController c(p);
  ASSERT_TRUE(c.request("stand"));
  EXPECT_TRUE(c.update(kDt));
  EXPECT_EQ(c.mode(), Mode::STANDING_UP);
  // First frame starts from the lying height.
  EXPECT_NEAR(footInBody(c, p, 0).z, -p.lie_height, 2e-3);
  run(c, p.transition_time + 0.1);
  EXPECT_EQ(c.mode(), Mode::STAND);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 f = footInBody(c, p, leg);
    EXPECT_NEAR(f.z, -p.stand_height, 1e-9);
    EXPECT_NEAR(f.x, c.neutralFoot(leg).x, 1e-9);
    EXPECT_NEAR(f.y, c.neutralFoot(leg).y, 1e-9);
  }
  EXPECT_EQ(c.unreachableCount(), 0);
}

TEST(Locomotion, WalksAndStops)
{
  LocomotionController c(LocomotionParams{});
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.1, 0, 0});
  run(c, 0.5);
  EXPECT_EQ(c.mode(), Mode::WALK);
  EXPECT_GT(c.velocity().vx, 0.09);
  c.setVelocity({});
  run(c, 1.5);
  EXPECT_EQ(c.mode(), Mode::STAND);
}

TEST(Locomotion, VelocityIsClampedAndRateLimited)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({10.0, -10.0, 10.0});
  c.update(kDt);
  EXPECT_NEAR(c.velocity().vx, p.max_accel.vx * kDt, 1e-12);
  run(c, 3.0);
  EXPECT_NEAR(c.velocity().vx, p.max_velocity.vx, 1e-12);
  EXPECT_NEAR(c.velocity().vy, -p.max_velocity.vy, 1e-12);
  EXPECT_NEAR(c.velocity().wz, p.max_velocity.wz, 1e-12);
  EXPECT_EQ(c.unreachableCount(), 0);
}

TEST(Locomotion, LieWhileWalkingFinishesStepsFirst)
{
  LocomotionController c(LocomotionParams{});
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.1, 0, 0});
  run(c, 1.0);
  ASSERT_TRUE(c.request("lie"));
  c.update(kDt);
  EXPECT_EQ(c.mode(), Mode::WALK);
  run(c, 4.0);
  EXPECT_EQ(c.mode(), Mode::LYING);
}

TEST(Locomotion, EstopGoesPassiveAndBlocksCommands)
{
  LocomotionController c(LocomotionParams{});
  c.request("stand");
  run(c, 2.0);
  c.setEstop(true);
  EXPECT_EQ(c.mode(), Mode::PASSIVE);
  EXPECT_FALSE(c.update(kDt));
  EXPECT_FALSE(c.request("stand"));
  c.setEstop(false);
  EXPECT_EQ(c.mode(), Mode::PASSIVE);
  EXPECT_TRUE(c.request("stand"));
}

TEST(Locomotion, BodyPitchLowersTheNose)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setBodyPose({0.0, 0.15, 0.0});
  run(c, 1.0);
  // Positive pitch = nose down: front feet closer to the body than rear feet.
  EXPECT_GT(footInBody(c, p, 0).z, footInBody(c, p, 2).z + 0.01);
  c.setBodyPose({0.0, 0.0, -0.03});
  run(c, 2.0);
  EXPECT_NEAR(footInBody(c, p, 0).z, -(p.stand_height - 0.03), 1e-9);
}

TEST(Locomotion, UnknownCommandRejected)
{
  LocomotionController c(LocomotionParams{});
  EXPECT_FALSE(c.request("dance"));
}

TEST(Locomotion, JointSpeedsFitTheServos)
{
  // Shipped gait (robot.yaml): the fastest command must not ask the joints
  // for more than the servo driver's slew limit (MG996R ~6-7 rad/s).
  const LocomotionParams p;  // defaults == robot.yaml
  const double servo_limit = 5.5;  // rad/s, margin below servos.yaml max_joint_speed (6)
  for (const auto & cmd : std::vector<dog_control::BodyVelocity>{
      {0.15, 0.0, 0.0}, {-0.15, 0.0, 0.0}, {0.0, 0.08, 0.0}, {0.0, 0.0, 0.6}, {0.15, 0.08, 0.6}}) {
    LocomotionController c(p);
    c.request("stand");
    run(c, 2.0);
    c.setVelocity(cmd);
    run(c, 1.0);  // accelerate
    auto prev = c.joints();
    double max_speed = 0.0;
    for (int i = 0; i < 200; ++i) {
      c.update(kDt);
      for (int j = 0; j < dog_control::kNumJoints; ++j) {
        max_speed = std::max(max_speed, std::abs(c.joints()[j] - prev[j]) / kDt);
      }
      prev = c.joints();
    }
    EXPECT_LT(max_speed, servo_limit) << "cmd " << cmd.vx << "," << cmd.vy << "," << cmd.wz;
    EXPECT_EQ(c.unreachableCount(), 0);
  }
}

TEST(Locomotion, SlopeCompensationShiftsFeetDownhill)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  const Vec3 flat = footInBody(c, p, 0);
  // Standing on a slope rising ahead: body pitched nose-up by 10 deg.
  const double slope = -10.0 * M_PI / 180.0;
  for (int i = 0; i < 200; ++i) {
    c.setImuAttitude(0.0, slope, kDt);
    c.update(kDt);
  }
  EXPECT_NEAR(c.slopePitch(), slope, 1e-3);
  const double expected = p.stand_height * std::tan(slope);  // ~ -26 mm (feet move downhill)
  EXPECT_NEAR(footInBody(c, p, 0).x - flat.x, expected, 1e-3);
  EXPECT_NEAR(footInBody(c, p, 2).x - c.neutralFoot(2).x, expected, 1e-3);
  // Left side uphill (positive roll): feet move to the right.
  for (int i = 0; i < 200; ++i) {
    c.setImuAttitude(0.08, 0.0, kDt);
    c.update(kDt);
  }
  EXPECT_LT(footInBody(c, p, 0).y - c.neutralFoot(0).y, -0.005);
}

TEST(Locomotion, SlopeCompensationIgnoresFallsAndCanBeDisabled)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setImuAttitude(0.0, 1.2, kDt);  // 69 deg: falling or picked up
  EXPECT_DOUBLE_EQ(c.slopePitch(), 0.0);
  LocomotionParams off = p;
  off.slope_compensation = false;
  LocomotionController d(off);
  d.request("stand");
  run(d, 2.0);
  const Vec3 before = footInBody(d, off, 0);
  for (int i = 0; i < 100; ++i) {
    d.setImuAttitude(0.0, -0.17, kDt);
    d.update(kDt);
  }
  EXPECT_NEAR(footInBody(d, off, 0).x, before.x, 1e-12);
}

TEST(Locomotion, HeadingHoldCountersYawDrift)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.1, 0.0, 0.0});
  // Robot drifts to the left (measured +0.2 rad/s) though commanded straight.
  for (int i = 0; i < 50; ++i) {
    c.setYawRate(0.2);
    c.update(kDt);
  }
  EXPECT_LT(c.headingError(), 0.0);
  EXPECT_LT(c.gaitVelocity().wz, 0.0);  // steers right
  for (int i = 0; i < 500; ++i) {
    c.setYawRate(0.2);
    c.update(kDt);
  }
  // blocked robot: error and correction stay bounded
  EXPECT_NEAR(c.headingError(), -p.heading_max_error, 1e-9);
  EXPECT_NEAR(c.gaitVelocity().wz, -p.heading_max_rate, 1e-9);
}

TEST(Locomotion, HeadingHoldClosedLoopKeepsCourse)
{
  // Plant: the robot turns at the gait's yaw rate plus a constant slip bias.
  for (bool hold : {false, true}) {
    LocomotionParams p;
    p.heading_hold = hold;
    LocomotionController c(p);
    c.request("stand");
    run(c, 2.0);
    c.setVelocity({0.12, 0.0, 0.0});
    double yaw = 0.0, rate = 0.0;
    for (int i = 0; i < static_cast<int>(6.0 / kDt); ++i) {
      c.setYawRate(rate);
      c.update(kDt);
      rate = c.gaitVelocity().wz + 0.1;  // 0.1 rad/s = 34 deg over 6 s
      yaw += rate * kDt;
    }
    if (hold) {
      EXPECT_LT(std::abs(yaw), 0.02) << "PI: drift offset removed (< 1.2 deg)";
    } else {
      EXPECT_GT(std::abs(yaw), 0.5);
    }
  }
}

TEST(Locomotion, HeadingHoldTracksCommandedTurns)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.0, 0.0, 0.5});
  double yaw = 0.0, rate = 0.0;
  const double T = 4.0;
  for (int i = 0; i < static_cast<int>(T / kDt); ++i) {
    c.setYawRate(rate);
    c.update(kDt);
    rate = 0.7 * c.gaitVelocity().wz;  // feet slip: only 70 % of the turn happens
    yaw += rate * kDt;
  }
  // Without hold 70 %; with hold the missing turn is made up (ramp-up aside).
  const double commanded = 0.5 * T - 0.5 * 0.5 / p.max_accel.wz;
  EXPECT_GT(yaw / commanded, 0.9);
  // Release the stick: the robot catches up, then stops without overshooting.
  // Target = integral of the (accel-limited) command: ramp-up and ramp-down
  // cancel, so it is 0.5 rad/s * T.
  c.setVelocity({});
  for (int i = 0; i < static_cast<int>(3.0 / kDt); ++i) {
    c.setYawRate(rate);
    c.update(kDt);
    rate = 0.7 * c.gaitVelocity().wz;
    yaw += rate * kDt;
  }
  EXPECT_NEAR(yaw / (0.5 * T), 1.0, 0.05);
}

TEST(Locomotion, HeadingHoldIdleWithoutImuOrWhenStanding)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  // Standing still: gyro noise must not make it turn on the spot.
  for (int i = 0; i < 200; ++i) {
    c.setYawRate(0.05);
    c.update(kDt);
  }
  EXPECT_DOUBLE_EQ(c.headingError(), 0.0);
  EXPECT_EQ(c.mode(), Mode::STAND);
  // Walking with the IMU gone: plain command.
  c.clearYawRate();
  c.setVelocity({0.1, 0.0, 0.0});
  run(c, 1.0);
  EXPECT_DOUBLE_EQ(c.gaitVelocity().wz, 0.0);
  EXPECT_DOUBLE_EQ(c.headingError(), 0.0);
}
