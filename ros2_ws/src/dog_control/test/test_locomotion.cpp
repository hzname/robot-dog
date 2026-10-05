#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "dog_control/locomotion.hpp"
#include "dog_control/odometry.hpp"

using dog_control::BodyPose;
using dog_control::BodyVelocity;
using dog_control::DeadReckoning;
using dog_control::GaitType;
using dog_control::forwardKinematics;
using dog_control::kNumLegs;
using dog_control::kSpeedTolerance;
using dog_control::legSide;
using dog_control::LocomotionController;
using dog_control::LocomotionParams;
using dog_control::Mode;
using dog_control::ServoSpeedModel;
using dog_control::Vec3;

namespace
{
constexpr double kDt = 0.02;

// Pinned reference set (the v1 robot as shipped before Phase 1): these tests check the
// algorithm, so they must not move when plan 01-15 syncs the shipped defaults with the measured robot.
LocomotionParams pinnedParams()
{
  LocomotionParams p;
  p.leg.hip = 0.055;
  p.leg.thigh = 0.105;
  p.leg.calf = 0.105;
  p.hip_x = 0.09;
  p.hip_y = 0.06;
  p.foot_offset_x = 0.0;
  p.foot_offset_y = 0.0;
  p.knee_direction = -1;
  p.stand_height = 0.15;
  p.lie_height = 0.08;
  p.min_height = 0.10;
  p.max_height = 0.18;
  p.max_velocity = {0.15, 0.08, 0.6};
  p.max_accel = {0.5, 0.3, 2.0};
  p.gait.period = 0.55;
  p.gait.duty = 0.65;
  p.gait.step_height = 0.02;
  p.gait.max_step = 0.06;
  p.gait.phase_offsets = {0.0, 0.5, 0.5, 0.0};
  p.auto_period = false;
  p.min_period = 0.55;
  p.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  return p;
}

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

// The five extreme commands of the JointSpeedsFitTheServos procedure, in its order.
const std::array<BodyVelocity, 5> kExtremeCommands{{
  {0.15, 0.0, 0.0},
  {-0.15, 0.0, 0.0},
  {0.0, 0.08, 0.0},
  {0.0, 0.0, 0.6},
  {0.15, 0.08, 0.6},
}};

// The JointSpeedsFitTheServos procedure, unchanged: stand (2 s), ramp (1 s),
// then the max joint speed over the 4 s window; the knee (j % 3 == 2) is
// scaled into the space of the servos (D-20).
struct SpeedMeasure
{
  double peak{0.0};    // [rad/s] in the space of the servos
  int unreachable{0};  // IK targets clamped in the last tick of the window
};

SpeedMeasure measureServoSpeeds(const LocomotionParams & p, const BodyVelocity & cmd, double knee_ratio)
{
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity(cmd);
  run(c, 1.0);  // accelerate
  auto prev = c.joints();
  SpeedMeasure out;
  for (int i = 0; i < 200; ++i) {
    c.update(kDt);
    for (int j = 0; j < dog_control::kNumJoints; ++j) {
      double speed = std::abs(c.joints()[j] - prev[j]) / kDt;
      if (j % 3 == 2) {speed *= knee_ratio;}
      out.peak = std::max(out.peak, speed);
    }
    prev = c.joints();
  }
  out.unreachable = c.unreachableCount();
  return out;
}

// One controller tick; the gain of gait().phase() modulo 1 (the phase grows
// by dt / period and wraps).
double phaseAdvance(LocomotionController & c)
{
  const double before = c.gait().phase();
  c.update(kDt);
  const double d = c.gait().phase() - before;
  return d - std::floor(d);
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

TEST(Locomotion, CrawlPutsItsFeetWithoutTheSlopeShift)
{
  // The crawl pitches the body on stairs itself; the IMU reads that as a
  // slope, and the trot's shift would put every foot a few cm off the
  // foothold the crawl chose on the map.
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.request("crawl");
  run(c, 1.0);
  ASSERT_EQ(c.gaitType(), GaitType::CRAWL);
  const Vec3 flat = footInBody(c, p, 0);
  for (int i = 0; i < 200; ++i) {
    c.setImuAttitude(0.0, -10.0 * M_PI / 180.0, kDt);
    c.update(kDt);
  }
  EXPECT_NEAR(footInBody(c, p, 0).x, flat.x, 1e-6);
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

TEST(Locomotion, HeadingHoldCountsEveryGyroSample)
{
  // IMU at twice the control rate, the robot jerked round by 0.5 rad/s for
  // one IMU sample in every two: the latest rate alone would read zero.
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.1, 0.0, 0.0});
  run(c, 1.0);
  const double e0 = c.headingError();
  for (int i = 0; i < 20; ++i) {
    c.addYawRate(0.5, kDt / 2);
    c.addYawRate(0.0, kDt / 2);
    c.update(kDt);
  }
  // turned 20 * 0.5 * kDt / 2; the hold has turned it back a little meanwhile
  EXPECT_LT(c.headingError() - e0, -0.8 * 20 * 0.5 * kDt / 2);
}

TEST(Locomotion, GuardLimitsForwardSpeedAndRaisesSwingPerLeg)
{
  LocomotionParams p;
  LocomotionController c(p);
  const double nan = std::nan("");
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.12, 0.0, 0.0});
  run(c, 1.0);
  EXPECT_NEAR(c.velocity().vx, 0.12, 1e-9);
  // stone in front of the left front foot: slow down, lift only that leg (ramped)
  c.setGuard(0.05, {0.045, nan, nan, nan});
  c.update(kDt);
  EXPECT_LT(c.stepHeight(0), 0.045);
  run(c, 1.0);
  EXPECT_NEAR(c.velocity().vx, 0.05, 1e-9);
  EXPECT_NEAR(c.stepHeight(0), 0.045, 1e-9);
  for (int leg = 1; leg < kNumLegs; ++leg) {EXPECT_NEAR(c.stepHeight(leg), p.gait.step_height, 1e-9);}
  // the swing of that leg really goes higher
  double apex[kNumLegs] = {};
  for (int i = 0; i < static_cast<int>(2.0 / kDt); ++i) {
    c.update(kDt);
    for (int leg = 0; leg < kNumLegs; ++leg) {apex[leg] = std::max(apex[leg], c.gait().feet()[leg].z);}
  }
  EXPECT_NEAR(apex[0], 0.045, 0.002);
  EXPECT_NEAR(apex[1], p.gait.step_height, 0.002);
  // stop: forward blocked, backing off and turning are not
  c.setGuard(0.0, {nan, nan, nan, nan});
  run(c, 1.0);
  EXPECT_NEAR(c.velocity().vx, 0.0, 1e-9);
  EXPECT_NEAR(c.stepHeight(0), p.gait.step_height, 1e-9);
  c.setVelocity({-0.05, 0.0, 0.3});
  run(c, 1.0);
  EXPECT_NEAR(c.velocity().vx, -0.05, 1e-9);
  EXPECT_NEAR(c.velocity().wz, 0.3, 1e-9);
  // guard gone: full command again
  c.clearGuard();
  c.setVelocity({0.12, 0.0, 0.0});
  run(c, 1.0);
  EXPECT_NEAR(c.velocity().vx, 0.12, 1e-9);
}

TEST(DeadReckoning, IntegratesTwistWithImuHeading)
{
  DeadReckoning o;
  for (int i = 0; i < 100; ++i) {o.update(0.01, 0.1, 0.0, 0.0);}  // 1 s straight
  EXPECT_NEAR(o.x(), 0.1, 1e-9);
  EXPECT_NEAR(o.y(), 0.0, 1e-9);
  // IMU appears at yaw 1.0 rad (its own zero): continue from the current heading
  o.update(0.01, 0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(o.yaw(), 0.0, 1e-9);
  for (int i = 0; i < 100; ++i) {o.update(0.01, 0.1, 0.0, 0.0, 1.0 + M_PI / 2);}  // turned left 90 deg
  EXPECT_NEAR(o.yaw(), M_PI / 2, 1e-9);
  EXPECT_NEAR(o.x(), 0.1, 1e-3);
  EXPECT_NEAR(o.y(), 0.1, 1e-3);
  // IMU gone: integrate the commanded yaw rate
  for (int i = 0; i < 100; ++i) {o.update(0.01, 0.0, 0.0, 0.5);}
  EXPECT_NEAR(o.yaw(), M_PI / 2 + 0.5, 1e-9);
}

TEST(Locomotion, SwitchesToCrawlOnlyWhenStoppedAndGoesRound)
{
  LocomotionParams p;
  LocomotionController c(p);
  const double nan = std::nan("");
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.12, 0.0, 0.0});
  run(c, 1.0);
  ASSERT_EQ(c.gaitType(), GaitType::TROT);
  // the guard asks for the crawl (a step ahead): stop first, then switch
  c.setGuard(nan, {nan, nan, nan, nan});
  c.setGuardGait(GaitType::CRAWL, 0.0);
  c.update(kDt);
  EXPECT_EQ(c.gaitType(), GaitType::TROT);
  run(c, 3.0);
  EXPECT_EQ(c.gaitType(), GaitType::CRAWL);
  run(c, 3.0);
  EXPECT_LE(c.gaitVelocity().vx, c.crawl().maxSpeed() + 1e-9);  // slow
  EXPECT_GT(c.gaitVelocity().vx, 0.0);
  // back to the trot once the guard is done (feet level)
  c.setGuardGait(GaitType::TROT, 0.0);
  run(c, 8.0);
  EXPECT_EQ(c.gaitType(), GaitType::TROT);
  // going round: sideways while the operator asks forward, never on its own
  c.setGuard(0.0, {nan, nan, nan, nan});
  c.setGuardGait(GaitType::TROT, 0.05);
  run(c, 2.0);
  EXPECT_NEAR(c.velocity().vx, 0.0, 1e-9);
  EXPECT_NEAR(c.velocity().vy, 0.05, 1e-9);
  c.setVelocity({});
  run(c, 1.0);
  EXPECT_NEAR(c.velocity().vy, 0.0, 1e-9);
  // the operator's own "crawl" command
  EXPECT_TRUE(c.request("crawl"));
  c.clearGuard();
  run(c, 3.0);
  EXPECT_EQ(c.gaitType(), GaitType::CRAWL);
}

TEST(Locomotion, LeavesTheCrawlWhileTheHeadingDrifts)
{
  // the sim: stopped in the crawl in front of a block, the avoider asked for
  // the trot; the heading hold kept turning the crawl (the gyro read the
  // body swaying), the crawl never stood still and the switch never came
  LocomotionParams p;
  p.heading_hold = true;
  LocomotionController c(p);
  const double nan = std::nan("");
  c.request("stand");
  run(c, 2.0);
  ASSERT_TRUE(c.request("crawl"));
  c.setVelocity({0.1, 0.0, 0.0});
  for (int i = 0; i < static_cast<int>(3.0 / kDt); ++i) {
    c.addYawRate(0.03 * std::sin(i * kDt * 3.0), kDt);
    c.update(kDt);
  }
  ASSERT_EQ(c.gaitType(), GaitType::CRAWL);
  // the guard wants the trot to go round: the crawl must come to rest
  c.request("trot");
  c.setGuard(0.0, {nan, nan, nan, nan});
  c.setGuardGait(GaitType::TROT, 0.0);
  int i = 0;
  for (; i < static_cast<int>(30.0 / kDt) && c.gaitType() != GaitType::TROT; ++i) {
    c.addYawRate(0.03 * std::sin(i * kDt * 3.0), kDt);  // the body sways
    c.update(kDt);
  }
  EXPECT_EQ(c.gaitType(), GaitType::TROT) << "still crawling after " << i * kDt << " s";
}

TEST(Locomotion, LeavesTheCrawlWithItsFeetOnALowStep)
{
  // the sim: crawling up to the 150 mm block, the front footholds on the
  // map's smeared edge 15 mm up, the guard asked for the trot to go round;
  // "feet level" (within 1 cm) never came, and the robot stood there
  LocomotionParams p;
  p.heading_hold = false;
  LocomotionController c(p);
  const double nan = std::nan("");
  c.request("stand");
  run(c, 2.0);
  ASSERT_TRUE(c.request("crawl"));
  double X = 0.0;  // walked, world
  auto step = [&](double dt) {
      dog_control::TerrainProfile t;
      t.x0 = -0.3;
      t.dx = 0.01;
      for (int i = 0; i < 100; ++i) {
        const double h = X + t.x0 + i * t.dx > 0.12 ? 0.015 : 0.0;
        t.left.push_back(h);
        t.right.push_back(h);
      }
      c.setTerrain(t);
      c.update(dt);
      X += c.gaitVelocity().vx * dt;
    };
  c.setVelocity({0.1, 0.0, 0.0});
  for (int i = 0; i < static_cast<int>(40.0 / kDt) && X < 0.12; ++i) {step(kDt);}
  ASSERT_EQ(c.gaitType(), GaitType::CRAWL);
  ASSERT_GT(X, 0.1);
  // the front feet up on it, the rear ones not: the guard wants the trot
  c.request("trot");
  c.setGuard(0.0, {nan, nan, nan, nan});
  c.setGuardGait(GaitType::TROT, 0.0);
  int i = 0;
  for (; i < static_cast<int>(30.0 / kDt) && c.gaitType() != GaitType::TROT; ++i) {step(kDt);}
  EXPECT_EQ(c.gaitType(), GaitType::TROT) << "still crawling after " << i * kDt << " s";
  // and the body does not jerk: its pitch goes on from the crawl's
  const double pitch = c.bodyPitch();
  step(kDt);
  EXPECT_NEAR(c.bodyPitch(), pitch, p.pose_rate * kDt + 1e-9);
}

TEST(Locomotion, SurveyLooksAroundWithTheFeetWhereTheyStand)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  ASSERT_EQ(c.mode(), Mode::STAND);
  const auto standing = c.joints();
  std::array<Vec3, kNumLegs> feet0{};
  for (int leg = 0; leg < kNumLegs; ++leg) {feet0[leg] = footInBody(c, p, leg);}
  // not while walking
  c.setVelocity({0.1, 0.0, 0.0});
  run(c, 0.5);
  EXPECT_FALSE(c.request("survey"));
  c.setVelocity({});
  run(c, 3.0);
  ASSERT_TRUE(c.request("survey"));
  EXPECT_EQ(c.mode(), Mode::SURVEY);
  dog_control::SurveySequence s(p.survey);  // the same sequence alongside, for the body attitude
  s.start();
  double min_pitch = 0.0, max_pitch = 0.0, max_yaw = 0.0, worst = 0.0;
  int ticks = 0;
  for (; ticks < 5000 && c.mode() == Mode::SURVEY; ++ticks) {
    c.update(kDt);
    s.update(kDt);
    ASSERT_EQ(c.unreachableCount(), 0) << "a foot out of reach at tick " << ticks;
    const double pt = s.frame().pitch, yw = s.frame().yaw;
    min_pitch = std::min(min_pitch, pt);
    max_pitch = std::max(max_pitch, pt);
    max_yaw = std::max(max_yaw, std::abs(yw));
    for (int leg = 0; leg < kNumLegs; ++leg) {
      // body -> ground under the body: R = Rz(yaw) Ry(pitch)
      const Vec3 b = footInBody(c, p, leg);
      const double gx = std::cos(pt) * b.x + std::sin(pt) * b.z, gz = -std::sin(pt) * b.x + std::cos(pt) * b.z;
      const Vec3 w{std::cos(yw) * gx - std::sin(yw) * b.y, std::sin(yw) * gx + std::cos(yw) * b.y, gz};
      worst = std::max({worst, std::abs(w.x - feet0[leg].x), std::abs(w.y - feet0[leg].y), std::abs(w.z - feet0[leg].z)});
    }
  }
  EXPECT_EQ(c.mode(), Mode::STAND);
  EXPECT_NEAR(ticks * kDt, s.duration(), 0.1);
  EXPECT_LT(worst, 0.001) << "the feet moved on the ground";
  EXPECT_NEAR(min_pitch, -p.survey.pitch_up_deg * M_PI / 180.0, 1e-3);
  EXPECT_NEAR(max_pitch, p.survey.pitch_down_deg * M_PI / 180.0, 1e-3);
  EXPECT_NEAR(max_yaw, p.survey.yaw_deg * M_PI / 180.0, 1e-3);
  for (int j = 0; j < 12; ++j) {EXPECT_NEAR(c.joints()[j], standing[j], 1e-6) << j;}
}

TEST(Locomotion, AutoPeriodAtStart)
{
  // D-12: with auto_period the period comes from servo.* at construction;
  // 6.0 rad/s, margin 0.8, min_period 0.55 give 0.6000 s (the plan 01-04 table).
  LocomotionParams p = pinnedParams();
  p.auto_period = true;
  p.min_period = 0.55;
  p.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  LocomotionController c(p);
  EXPECT_NEAR(c.gaitPeriod(), 0.6000, 1e-9);
  EXPECT_NEAR(c.gait().params().period, 0.6000, 1e-9);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.1, 0.0, 0.0});
  run(c, 1.0);
  EXPECT_EQ(c.mode(), Mode::WALK);
  EXPECT_NEAR(phaseAdvance(c), kDt / 0.6, 1e-9);

  // auto_period off: the manual gait.period stays an explicit override.
  LocomotionParams m = pinnedParams();
  m.auto_period = false;
  m.gait.period = 0.7;
  m.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  LocomotionController d(m);
  EXPECT_DOUBLE_EQ(d.gaitPeriod(), 0.7);
  d.request("stand");
  run(d, 2.0);
  d.setVelocity({0.1, 0.0, 0.0});
  run(d, 1.0);
  EXPECT_EQ(d.mode(), Mode::WALK);
  EXPECT_NEAR(phaseAdvance(d), kDt / 0.7, 1e-9);
}

TEST(Locomotion, JointSpeedsFitTheServosAuto)
{
  // The period is computed from the assumed servo speed (D-13) and the peak
  // stays under margin * max_speed (D-11) on every row of the pinned table.
  const std::array<double, 6> speeds{3.5, 4.0, 5.0, 6.0, 6.35, 7.0};
  const std::array<double, 6> periods{1.0300, 0.9050, 0.7200, 0.6000, 0.5575, 0.5500};
  for (int i = 0; i < 6; ++i) {
    LocomotionParams p = pinnedParams();
    p.auto_period = true;
    p.min_period = 0.55;
    p.servo = ServoSpeedModel{speeds[i], 0.8, 1.0};
    LocomotionController c(p);
    EXPECT_NEAR(c.gaitPeriod(), periods[i], 1e-9) << "speed " << speeds[i];
    EXPECT_GE(c.gaitPeriod(), p.min_period);
    for (const auto & cmd : kExtremeCommands) {
      const SpeedMeasure m = measureServoSpeeds(p, cmd, p.servo.knee_ratio);
      EXPECT_LE(m.peak, 0.8 * speeds[i] + kSpeedTolerance) << "speed " << speeds[i];
      EXPECT_EQ(m.unreachable, 0) << "speed " << speeds[i];
    }
  }
  // The knee rod drive is faster than the joint (D-20): the scaled knee peak
  // makes the period grow past the 0.55 s lower bound so it still fits 6.4.
  LocomotionParams k = pinnedParams();
  k.auto_period = true;
  k.min_period = 0.55;
  k.servo = ServoSpeedModel{8.0, 0.8, 1.388};
  LocomotionController c(k);
  EXPECT_GT(c.gaitPeriod(), 0.55 + 1e-9);
  for (const auto & cmd : kExtremeCommands) {
    const SpeedMeasure m = measureServoSpeeds(k, cmd, k.servo.knee_ratio);
    EXPECT_LE(m.peak, 6.4 + kSpeedTolerance);
  }
}

TEST(Locomotion, NoFitThrows)
{
  const double nan = std::nan("");
  LocomotionParams p = pinnedParams();
  p.auto_period = true;
  p.servo = ServoSpeedModel{1.0, 0.8, 1.0};
  try {
    LocomotionController c(p);
    FAIL() << "1.0 rad/s with margin 0.8 must not fit up to kMaxAutoPeriod";
  } catch (const std::runtime_error & e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("servo.max_speed"), std::string::npos) << msg;
    EXPECT_NE(msg.find("gait.auto_period"), std::string::npos) << msg;
  }

  LocomotionParams margin = p;
  margin.servo = ServoSpeedModel{6.0, 0.0, 1.0};
  EXPECT_THROW({LocomotionController c(margin);}, std::runtime_error);

  LocomotionParams nan_speed = p;
  nan_speed.servo = ServoSpeedModel{nan, 0.8, 1.0};
  EXPECT_THROW({LocomotionController c(nan_speed);}, std::runtime_error);

  LocomotionParams no_ratio = p;
  no_ratio.servo = ServoSpeedModel{6.0, 0.8, 0.0};
  EXPECT_THROW({LocomotionController c(no_ratio);}, std::runtime_error);

  LocomotionParams short_min = p;
  short_min.min_period = 0.05;
  short_min.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  EXPECT_THROW({LocomotionController c(short_min);}, std::runtime_error);

  // Manual mode: a period that is not finite and above 0 has no fallback.
  for (const double period : {0.0, -1.0, nan}) {
    LocomotionParams m = pinnedParams();
    m.auto_period = false;
    m.gait.period = period;
    EXPECT_THROW({LocomotionController c(m);}, std::runtime_error) << "period " << period;
  }

  // Manual mode never reads servo.*: NaN there must not throw.
  LocomotionParams ok = pinnedParams();
  ok.auto_period = false;
  ok.gait.period = 0.7;
  ok.servo = ServoSpeedModel{nan, nan, nan};
  LocomotionController good(ok);
  EXPECT_DOUBLE_EQ(good.gaitPeriod(), 0.7);
}

TEST(Locomotion, ReconfigureAcceptedWhenStanding)
{
  const double nan = std::nan("");
  LocomotionParams p = pinnedParams();
  p.auto_period = false;
  p.gait.period = 0.55;
  p.min_period = 0.55;
  p.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  ASSERT_EQ(c.mode(), Mode::STAND);
  // The guard's swing height must survive the rebuild (it is not reset).
  c.setGuard(nan, {0.045, nan, nan, nan});
  run(c, 1.0);
  EXPECT_NEAR(c.stepHeight(0), 0.045, 1e-9);
  ASSERT_TRUE(c.reconfigureGait(0.55, true, 0.55, {5.0, 0.8, 1.0}));
  EXPECT_NEAR(c.gaitPeriod(), 0.7200, 1e-9);
  EXPECT_NEAR(c.stepHeight(0), 0.045, 1e-9);
  // Standing still: the next tick gives exactly the same joints.
  const auto standing = c.joints();
  c.update(kDt);
  for (int j = 0; j < dog_control::kNumJoints; ++j) {EXPECT_NEAR(c.joints()[j], standing[j], 1e-9) << j;}
  EXPECT_EQ(c.mode(), Mode::STAND);
  // Walking: the phase grows with the new 0.72 s period.
  c.setVelocity({0.1, 0.0, 0.0});
  run(c, 1.0);
  EXPECT_EQ(c.mode(), Mode::WALK);
  EXPECT_NEAR(phaseAdvance(c), kDt / 0.72, 1e-9);
  // Back to a manual period once stopped.
  c.setVelocity({});
  run(c, 2.0);
  EXPECT_EQ(c.mode(), Mode::STAND);
  EXPECT_TRUE(c.reconfigureGait(0.8, false, 0.55, {5.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.8);
  // PASSIVE is accepted too.
  c.setEstop(true);
  EXPECT_EQ(c.mode(), Mode::PASSIVE);
  EXPECT_TRUE(c.reconfigureGait(0.55, true, 0.55, {6.0, 0.8, 1.0}));
  EXPECT_NEAR(c.gaitPeriod(), 0.6000, 1e-9);

  // LYING: the third accepted mode.
  LocomotionParams q = pinnedParams();
  q.auto_period = false;
  q.gait.period = 0.55;
  q.min_period = 0.55;
  q.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  LocomotionController d(q);
  ASSERT_TRUE(d.request("lie"));
  ASSERT_EQ(d.mode(), Mode::LYING);
  EXPECT_TRUE(d.reconfigureGait(0.55, true, 0.55, {6.0, 0.8, 1.0}));
  EXPECT_NEAR(d.gaitPeriod(), 0.6000, 1e-9);
}

TEST(Locomotion, ReconfigureRejectedWhenWalking)
{
  LocomotionParams p = pinnedParams();
  p.auto_period = false;
  p.gait.period = 0.55;
  p.min_period = 0.55;
  p.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  c.setVelocity({0.1, 0.0, 0.0});
  run(c, 1.0);
  ASSERT_EQ(c.mode(), Mode::WALK);
  EXPECT_FALSE(c.gaitReconfigurable());
  EXPECT_FALSE(c.reconfigureGait(0.55, true, 0.55, {5.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.55);
  EXPECT_TRUE(c.gait().stepping());
  EXPECT_NEAR(phaseAdvance(c), kDt / 0.55, 1e-9);  // the step is not reset

  // STANDING_UP: the transition blocks it.
  LocomotionController u(p);
  ASSERT_TRUE(u.request("stand"));
  ASSERT_EQ(u.mode(), Mode::STANDING_UP);
  u.update(kDt);
  EXPECT_FALSE(u.gaitReconfigurable());
  EXPECT_FALSE(u.reconfigureGait(0.55, true, 0.55, {5.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(u.gaitPeriod(), 0.55);
  EXPECT_FALSE(u.gait().stepping());

  // LYING_DOWN, GREETING, SURVEY: a fresh controller in STAND each time.
  for (int which = 0; which < 3; ++which) {
    LocomotionController d(p);
    d.request("stand");
    run(d, 2.0);
    ASSERT_EQ(d.mode(), Mode::STAND);
    const char * cmd = which == 0 ? "lie" : (which == 1 ? "greet" : "survey");
    ASSERT_TRUE(d.request(cmd));
    const Mode m = d.mode();
    ASSERT_TRUE(m == Mode::LYING_DOWN || m == Mode::GREETING || m == Mode::SURVEY) << dog_control::modeName(m);
    EXPECT_FALSE(d.gaitReconfigurable());
    EXPECT_FALSE(d.reconfigureGait(0.55, true, 0.55, {5.0, 0.8, 1.0}));
    EXPECT_DOUBLE_EQ(d.gaitPeriod(), 0.55);
    EXPECT_FALSE(d.gait().stepping());
  }

  // The mode blocks, not the values: the same call is accepted in STAND.
  c.setVelocity({});
  run(c, 2.0);
  ASSERT_EQ(c.mode(), Mode::STAND);
  EXPECT_TRUE(c.gaitReconfigurable());
  EXPECT_TRUE(c.reconfigureGait(0.55, true, 0.55, {5.0, 0.8, 1.0}));
  EXPECT_NEAR(c.gaitPeriod(), 0.7200, 1e-9);
}

TEST(Locomotion, ReconfigureNoFitKeepsTheGait)
{
  const double nan = std::nan("");
  LocomotionParams p = pinnedParams();
  p.auto_period = false;
  p.gait.period = 0.55;
  p.min_period = 0.55;
  p.servo = ServoSpeedModel{6.0, 0.8, 1.0};
  LocomotionController c(p);
  c.request("stand");
  run(c, 2.0);
  ASSERT_EQ(c.mode(), Mode::STAND);
  // A refused call changes nothing and leaves the controller usable.
  EXPECT_FALSE(c.reconfigureGait(0.55, true, 0.55, {1.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.55);
  EXPECT_FALSE(c.reconfigureGait(0.55, true, 0.55, {nan, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.55);
  EXPECT_FALSE(c.reconfigureGait(nan, false, 0.55, {6.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.55);
  EXPECT_FALSE(c.reconfigureGait(0.0, false, 0.55, {6.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.55);
  EXPECT_TRUE(c.reconfigureGait(0.7, false, 0.55, {6.0, 0.8, 1.0}));
  EXPECT_DOUBLE_EQ(c.gaitPeriod(), 0.7);
}
