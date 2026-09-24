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
