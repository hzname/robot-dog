/**
 * @file test_inverse_kinematics.cpp
 * @brief Unit tests for Inverse Kinematics calculations
 */

#include <gtest/gtest.h>
#include <cmath>

constexpr double HIP_LENGTH = 0.083;
constexpr double THIGH_LENGTH = 0.25;
constexpr double CALF_LENGTH = 0.25;
constexpr double MAX_REACH = THIGH_LENGTH + CALF_LENGTH;

void solveLegIK(int leg_idx, double foot_x, double foot_y, double foot_z,
                double &hip_angle, double &thigh_angle, double &calf_angle)
{
  hip_angle = std::atan2(foot_y, -foot_z);
  
  double d = std::sqrt(foot_y * foot_y + foot_z * foot_z);
  double d_adj = d - HIP_LENGTH;
  
  double target_dist = std::sqrt(foot_x * foot_x + d_adj * d_adj);
  target_dist = std::min(target_dist, MAX_REACH - 0.001);
  
  double cos_calf = (THIGH_LENGTH*THIGH_LENGTH + CALF_LENGTH*CALF_LENGTH - target_dist*target_dist) /
                    (2.0 * THIGH_LENGTH * CALF_LENGTH);
  cos_calf = std::max(-1.0, std::min(1.0, cos_calf));
  calf_angle = M_PI - std::acos(cos_calf);
  
  double cos_thigh = (target_dist*target_dist + THIGH_LENGTH*THIGH_LENGTH - CALF_LENGTH*CALF_LENGTH) /
                     (2.0 * target_dist * THIGH_LENGTH);
  cos_thigh = std::max(-1.0, std::min(1.0, cos_thigh));
  
  double target_angle = std::atan2(foot_x, d_adj);
  thigh_angle = target_angle - std::acos(cos_thigh);
  
  if (leg_idx == 1 || leg_idx == 3) {
    hip_angle = -hip_angle;
    thigh_angle = -thigh_angle;
    calf_angle = -calf_angle;
  }
}

TEST(IKTest, ValidAnglesForTypicalStance)
{
  double h, t, c;
  solveLegIK(0, 0.15, 0.08, -0.25, h, t, c);
  
  EXPECT_TRUE(std::isfinite(h));
  EXPECT_TRUE(std::isfinite(t));
  EXPECT_TRUE(std::isfinite(c));
  EXPECT_GT(std::abs(h), 0.0);
  EXPECT_LT(std::abs(h), M_PI/2);
}

TEST(IKTest, ExtendedLeg)
{
  double h, t, c;
  solveLegIK(0, MAX_REACH * 0.9, 0, -HIP_LENGTH, h, t, c);
  EXPECT_LT(std::abs(c), M_PI / 2);
  EXPECT_GT(t, 0.0);
}

TEST(IKTest, FoldedLeg)
{
  double h, t, c;
  solveLegIK(0, 0.0, 0, -(HIP_LENGTH + 0.05), h, t, c);
  EXPECT_GT(std::abs(c), M_PI / 2);
}

TEST(IKTest, AllFourLegs)
{
  for (int leg = 0; leg < 4; ++leg) {
    double h, t, c;
    EXPECT_NO_THROW(solveLegIK(leg, 0.15, 0.08, -0.25, h, t, c));
    EXPECT_TRUE(std::isfinite(h));
    EXPECT_TRUE(std::isfinite(t));
    EXPECT_TRUE(std::isfinite(c));
  }
}

TEST(IKTest, UnreachableTarget)
{
  double h, t, c;
  EXPECT_NO_THROW(solveLegIK(0, MAX_REACH * 2, 0, 0, h, t, c));
  EXPECT_TRUE(std::isfinite(h));
}

TEST(IKTest, ZeroPosition)
{
  double h, t, c;
  solveLegIK(0, 0.0, 0.0, -(HIP_LENGTH + 0.1), h, t, c);
  EXPECT_TRUE(std::isfinite(h));
  EXPECT_TRUE(std::isfinite(t));
  EXPECT_TRUE(std::isfinite(c));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}