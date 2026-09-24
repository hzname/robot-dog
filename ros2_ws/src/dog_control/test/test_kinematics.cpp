#include <gtest/gtest.h>

#include <cmath>

#include "dog_control/kinematics.hpp"

using dog_control::forwardKinematics;
using dog_control::inverseKinematics;
using dog_control::JointAngles;
using dog_control::LegGeometry;
using dog_control::Vec3;

namespace
{
const LegGeometry kLeg{0.055, 0.105, 0.105};

void expectNear(const Vec3 & a, const Vec3 & b, double tol = 1e-9)
{
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(a.z, b.z, tol);
}
}  // namespace

TEST(Kinematics, ZeroAnglesHangStraightDown)
{
  for (int side : {1, -1}) {
    expectNear(forwardKinematics(kLeg, side, {0, 0, 0}), {0.0, side * 0.055, -0.21});
  }
}

TEST(Kinematics, JointSignConventions)
{
  // Positive thigh pitch swings the foot backwards.
  EXPECT_LT(forwardKinematics(kLeg, 1, {0, 0.3, 0}).x, 0.0);
  // Positive hip roll moves the foot to the left (+y), for both sides.
  EXPECT_GT(forwardKinematics(kLeg, 1, {0.2, 0, 0}).y, 0.055);
  EXPECT_GT(forwardKinematics(kLeg, -1, {0.2, 0, 0}).y, -0.055);
}

TEST(Kinematics, StandingPoseIsSymmetricAndKneeBacks)
{
  const auto ik = inverseKinematics(kLeg, 1, {0.0, 0.055, -0.15}, -1);
  ASSERT_TRUE(ik.reachable);
  EXPECT_NEAR(ik.q[0], 0.0, 1e-9);
  EXPECT_LT(ik.q[2], 0.0);           // knee bent backwards
  EXPECT_GT(ik.q[1], 0.0);           // thigh leans back to keep the foot under the hip
  EXPECT_NEAR(ik.q[1], -ik.q[2] / 2.0, 1e-9);  // equal links -> isosceles triangle
  const Vec3 knee{-kLeg.thigh * std::sin(ik.q[1]), 0, 0};
  EXPECT_LT(knee.x, 0.0);
}

TEST(Kinematics, RoundTripOverWorkspace)
{
  int checked = 0;
  for (int side : {1, -1}) {
    for (int dir : {-1, 1}) {
      for (double x = -0.08; x <= 0.08; x += 0.02) {
        for (double dy = -0.04; dy <= 0.04; dy += 0.02) {
          for (double z = -0.19; z <= -0.08; z += 0.01) {
            const Vec3 target{x, side * 0.055 + dy, z};
            const auto ik = inverseKinematics(kLeg, side, target, dir);
            if (!ik.reachable) {
              continue;
            }
            expectNear(forwardKinematics(kLeg, side, ik.q), target, 1e-9);
            EXPECT_EQ(ik.q[2] <= 0.0, dir < 0);
            ++checked;
          }
        }
      }
    }
  }
  EXPECT_GT(checked, 1000);
}

TEST(Kinematics, UnreachableTargetsAreClampedAndFlagged)
{
  const auto far = inverseKinematics(kLeg, 1, {0.0, 0.055, -0.5});
  EXPECT_FALSE(far.reachable);
  const Vec3 f = forwardKinematics(kLeg, 1, far.q);
  EXPECT_NEAR(std::hypot(f.x, f.z), 0.21, 1e-3);  // fully stretched
  for (double q : far.q) {EXPECT_TRUE(std::isfinite(q));}

  const auto near = inverseKinematics(kLeg, 1, {0.0, 0.055, -0.001});
  EXPECT_FALSE(near.reachable);
  for (double q : near.q) {EXPECT_TRUE(std::isfinite(q));}
}
