/**
 * @file test_inverse_kinematics.cpp
 * @brief Unit tests for Inverse Kinematics calculations
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

// Test leg dimensions matching URDF (dog.urdf.xacro)
constexpr double HIP_LENGTH = 0.083;   // hip to thigh
constexpr double THIGH_LENGTH = 0.25;  // thigh to calf
constexpr double CALF_LENGTH = 0.25;   // calf to foot
constexpr double BODY_LENGTH = 0.40;
constexpr double BODY_WIDTH = 0.20;

/**
 * @brief Solve leg IK (mirrors GaitController::solveLegIK)
 */
void solveLegIK(int leg_idx, double foot_x, double foot_y, double foot_z,
                double thigh_len, double calf_len, double hip_len,
                double &hip_angle, double &thigh_angle, double &calf_angle)
{
  // Hip angle (abduction/adduction)
  hip_angle = std::atan2(foot_y, -foot_z);
  
  // Project to leg plane
  double d = std::sqrt(foot_y * foot_y + foot_z * foot_z);
  double leg_plane_x = foot_x;
  double leg_plane_z = d - hip_len;
  
  // 2D IK in leg plane using law of cosines
  double target_dist = std::sqrt(leg_plane_x * leg_plane_x + leg_plane_z * leg_plane_z);
  target_dist = std::min(target_dist, thigh_len + calf_len - 0.001);
  
  // Shin angle (knee)
  double cos_calf = (thigh_len * thigh_len + calf_len * calf_len - target_dist * target_dist) /
                    (2.0 * thigh_len * calf_len);
  cos_calf = std::max(-1.0, std::min(1.0, cos_calf));
  calf_angle = M_PI - std::acos(cos_calf);
  
  // Thigh angle (hip pitch)
  double cos_thigh = (target_dist * target_dist + thigh_len * thigh_len - calf_len * calf_len) /
                     (2.0 * target_dist * thigh_len);
  cos_thigh = std::max(-1.0, std::min(1.0, cos_thigh));
  
  double target_angle = std::atan2(leg_plane_x, -leg_plane_z);
  thigh_angle = target_angle - std::acos(cos_thigh);
  
  // Mirror angles for right legs (RF=1 and RR=3)
  if (leg_idx == 1 || leg_idx == 3) {
    hip_angle = -hip_angle;
    thigh_angle = -thigh_angle;
    calf_angle = -calf_angle;
  }
}

/**
 * @brief Forward kinematics to verify IK solution
 */
void forwardKinematics(double hip_angle, double thigh_angle, double calf_angle,
                       double thigh_len, double calf_len, double hip_len,
                       double &foot_x, double &foot_y, double &foot_z)
{
  // Hip offset in YZ plane
  double d = hip_len;
  double y_hip = d * std::sin(hip_angle);
  double z_hip = -d * std::cos(hip_angle);
  
  // Thigh position
  double x_thigh = thigh_len * std::sin(thigh_angle);
  double z_thigh = -thigh_len * std::cos(thigh_angle);
  
  // Calf position (relative to thigh)
  double total_knee_angle = thigh_angle + calf_angle;
  double x_calf = calf_len * std::sin(total_knee_angle);
  double z_calf = -calf_len * std::cos(total_knee_angle);
  
  foot_x = x_thigh + x_calf;
  foot_y = y_hip;
  foot_z = z_hip + z_thigh + z_calf;
}

class InverseKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Default leg dimensions
    thigh_len_ = THIGH_LENGTH;
    calf_len_ = CALF_LENGTH;
    hip_len_ = HIP_LENGTH;
  }
  
  double thigh_len_;
  double calf_len_;
  double hip_len_;
};

// Test 1: Neutral stance position
TEST_F(InverseKinematicsTest, NeutralStance)
{
  // Neutral stance: foot directly below hip, at stance height
  double stance_height = 0.25;
  double foot_x = BODY_LENGTH / 4.0;  // LF leg default X
  double foot_y = BODY_WIDTH / 2.0 * 0.8;  // LF leg default Y
  double foot_z = -stance_height;
  
  double hip_angle, thigh_angle, calf_angle;
  
  // Test for left front leg (LF = 0)
  solveLegIK(0, foot_x, foot_y, foot_z, thigh_len_, calf_len_, hip_len_,
             hip_angle, thigh_angle, calf_angle);
  
  // Verify IK solution by running forward kinematics
  double result_x, result_y, result_z;
  forwardKinematics(hip_angle, thigh_angle, calf_angle,
                    thigh_len_, calf_len_, hip_len_,
                    result_x, result_y, result_z);
  
  // Check that FK(IK(x)) ≈ x (within tolerance)
  EXPECT_NEAR(result_x, foot_x, 1e-6);
  EXPECT_NEAR(result_y, foot_y, 1e-6);
  EXPECT_NEAR(result_z, foot_z, 1e-6);
}

// Test 2: Verify symmetry between left and right legs
TEST_F(InverseKinematicsTest, LeftRightSymmetry)
{
  double stance_height = 0.25;
  double foot_x = 0.05;
  double foot_y_left = 0.08;
  double foot_y_right = -0.08;
  double foot_z = -stance_height;
  
  double hip_l, thigh_l, calf_l;
  double hip_r, thigh_r, calf_r;
  
  // Left front leg (LF = 0)
  solveLegIK(0, foot_x, foot_y_left, foot_z, thigh_len_, calf_len_, hip_len_,
             hip_l, thigh_l, calf_l);
  
  // Right front leg (RF = 1)
  solveLegIK(1, foot_x, foot_y_right, foot_z, thigh_len_, calf_len_, hip_len_,
             hip_r, thigh_r, calf_r);
  
  // Hip angles should be mirrored
  EXPECT_NEAR(hip_l, -hip_r, 1e-10);
  
  // Thigh and calf angles should be mirrored
  EXPECT_NEAR(thigh_l, -thigh_r, 1e-10);
  EXPECT_NEAR(calf_l, -calf_r, 1e-10);
}

// Test 3: Test maximum reach (fully extended leg)
TEST_F(InverseKinematicsTest, MaximumReach)
{
  // Maximum reach when leg is fully extended
  double max_reach = thigh_len_ + calf_len_;
  double foot_x = max_reach * 0.99;  // 99% of max to avoid numerical issues
  double foot_y = 0.0;
  double foot_z = -HIP_LENGTH;  // At hip height
  
  double hip_angle, thigh_angle, calf_angle;
  
  solveLegIK(0, foot_x, foot_y, foot_z, thigh_len_, calf_len_, hip_len_,
             hip_angle, thigh_angle, calf_angle);
  
  // At maximum reach, calf angle should approach 0 (fully extended)
  EXPECT_NEAR(calf_angle, 0.0, 0.1);
  
  // Verify with forward kinematics
  double result_x, result_y, result_z;
  forwardKinematics(hip_angle, thigh_angle, calf_angle,
                    thigh_len_, calf_len_, hip_len_,
                    result_x, result_y, result_z);
  
  EXPECT_NEAR(result_x, foot_x, 1e-4);
  EXPECT_NEAR(result_y, foot_y, 1e-4);
  EXPECT_NEAR(result_z, foot_z, 1e-4);
}

// Test 4: Test leg folded position (minimum reach)
TEST_F(InverseKinematicsTest, MinimumReach)
{
  // Minimum reach when leg is folded back
  double foot_x = 0.0;
  double foot_y = 0.0;
  double foot_z = -(thigh_len_ + calf_len_ - 0.01);  // Almost fully folded
  
  double hip_angle, thigh_angle, calf_angle;
  
  solveLegIK(0, foot_x, foot_y, foot_z, thigh_len_, calf_len_, hip_len_,
             hip_angle, thigh_angle, calf_angle);
  
  // When folded, calf angle should be near PI (180 degrees)
  EXPECT_GT(std::abs(calf_angle), M_PI / 2.0);
  
  // Verify with forward kinematics
  double result_x, result_y, result_z;
  forwardKinematics(hip_angle, thigh_angle, calf_angle,
                    thigh_len_, calf_len_, hip_len_,
                    result_x, result_y, result_z);
  
  EXPECT_NEAR(result_x, foot_x, 1e-3);
  EXPECT_NEAR(result_y, foot_y, 1e-3);
  EXPECT_NEAR(result_z, foot_z, 1e-3);
}

// Test 5: Test all four legs produce valid solutions
TEST_F(InverseKinematicsTest, AllLegsValid)
{
  std::array<std::array<double, 3>, 4> leg_positions = {{
    {{ BODY_LENGTH / 2.0,  BODY_WIDTH / 2.0, 0.0}},   // LF
    {{ BODY_LENGTH / 2.0, -BODY_WIDTH / 2.0, 0.0}},   // RF
    {{-BODY_LENGTH / 2.0,  BODY_WIDTH / 2.0, 0.0}},   // LR
    {{-BODY_LENGTH / 2.0, -BODY_WIDTH / 2.0, 0.0}}    // RR
  }};
  
  double stance_height = 0.25;
  
  for (int leg = 0; leg < 4; ++leg) {
    double foot_x = leg_positions[leg][0] * 0.5;
    double foot_y = leg_positions[leg][1] * 0.8;
    double foot_z = -stance_height;
    
    double hip_angle, thigh_angle, calf_angle;
    
    EXPECT_NO_THROW({
      solveLegIK(leg, foot_x, foot_y, foot_z, thigh_len_, calf_len_, hip_len_,
                 hip_angle, thigh_angle, calf_angle);
    });
    
    // Verify angles are finite
    EXPECT_TRUE(std::isfinite(hip_angle));
    EXPECT_TRUE(std::isfinite(thigh_angle));
    EXPECT_TRUE(std::isfinite(calf_angle));
    
    // Verify IK solution with forward kinematics
    double result_x, result_y, result_z;
    forwardKinematics(hip_angle, thigh_angle, calf_angle,
                      thigh_len_, calf_len_, hip_len_,
                      result_x, result_y, result_z);
    
    EXPECT_NEAR(result_x, foot_x, 1e-5);
    EXPECT_NEAR(result_y, foot_y, 1e-5);
    EXPECT_NEAR(result_z, foot_z, 1e-5);
  }
}

// Test 6: Test unreachable target (should clamp)
TEST_F(InverseKinematicsTest, UnreachableTarget)
{
  // Target beyond maximum reach
  double max_reach = thigh_len_ + calf_len_;
  double foot_x = max_reach * 1.5;  // Way beyond reach
  double foot_y = 0.0;
  double foot_z = -HIP_LENGTH;
  
  double hip_angle, thigh_angle, calf_angle;
  
  // Should not throw and should return valid (clamped) angles
  EXPECT_NO_THROW({
    solveLegIK(0, foot_x, foot_y, foot_z, thigh_len_, calf_len_, hip_len_,
               hip_angle, thigh_angle, calf_angle);
  });
  
  // Verify angles are finite
  EXPECT_TRUE(std::isfinite(hip_angle));
  EXPECT_TRUE(std::isfinite(thigh_angle));
  EXPECT_TRUE(std::isfinite(calf_angle));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
