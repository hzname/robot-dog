#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "dog_control/locomotion.hpp"
#include "dog_control/odometry.hpp"

using dog_control::BodyPose;
using dog_control::DeadReckoning;
using dog_control::GaitType;
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
