/**
 * @file test_gait_controller.cpp
 * @brief Unit tests for Gait Controller phases and trajectory generation
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

// Leg indices
constexpr int LF = 0;  // Left Front
constexpr int RF = 1;  // Right Front
constexpr int LR = 2;  // Left Rear
constexpr int RR = 3;  // Right Rear

// Gait parameters
constexpr double STANCE_HEIGHT = 0.25;
constexpr double STEP_HEIGHT = 0.08;

/**
 * @brief Calculate foot trajectory (mirrors GaitController::calculateFootTrajectory)
 */
void calculateFootTrajectory(int leg_idx, double phase, double step_length,
                             double linear_vel_x, double angular_vel_z,
                             double &x, double &y, double &z)
{
  // Leg hip positions
  std::array<std::array<double, 3>, 4> leg_positions = {{
    {{ 0.35 / 2.0,  0.20 / 2.0, 0.0}},   // LF
    {{ 0.35 / 2.0, -0.20 / 2.0, 0.0}},   // RF
    {{-0.35 / 2.0,  0.20 / 2.0, 0.0}},   // LR
    {{-0.35 / 2.0, -0.20 / 2.0, 0.0}}    // RR
  }};
  
  // Default foot position (neutral stance)
  double default_x = leg_positions[leg_idx][0] * 0.5;
  double default_y = leg_positions[leg_idx][1] * 0.8;
  double default_z = -STANCE_HEIGHT;
  
  double x_offset = 0.0;
  double z_offset = 0.0;
  
  if (phase < 0.5) {
    // Swing phase
    double swing_phase = phase * 2.0;
    x_offset = -step_length + (2.0 * step_length * swing_phase);
    z_offset = STEP_HEIGHT * std::sin(swing_phase * M_PI);
  } else {
    // Stance phase
    double stance_phase = (phase - 0.5) * 2.0;
    x_offset = step_length - (2.0 * step_length * stance_phase);
    z_offset = 0.0;
  }
  
  // Apply turning
  double turn_offset = 0.0;
  if (std::abs(angular_vel_z) > 0.01) {
    double leg_sign = (leg_idx == LF || leg_idx == LR) ? 1.0 : -1.0;
    turn_offset = leg_sign * angular_vel_z * 0.05 * std::sin(phase * 2.0 * M_PI);
  }
  
  x = default_x + x_offset + turn_offset;
  y = default_y;
  z = default_z + z_offset;
}

/**
 * @brief Get trot gait phase offsets
 */
std::array<double, 4> getTrotPhaseOffsets()
{
  return {{0.0, 0.5, 0.5, 0.0}};  // LF, RF, LR, RR
}

class GaitControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    phase_offsets_ = getTrotPhaseOffsets();
  }
  
  std::array<double, 4> phase_offsets_;
};

// Test 1: Trot gait phase offsets
TEST_F(GaitControllerTest, TrotPhaseOffsets)
{
  // In trot gait, diagonal legs move together
  // LF (0) and RR (3) should have same phase
  // RF (1) and LR (2) should have same phase
  // Diagonal pairs should be 180° out of phase
  
  EXPECT_DOUBLE_EQ(phase_offsets_[LF], phase_offsets_[RR]);
  EXPECT_DOUBLE_EQ(phase_offsets_[RF], phase_offsets_[LR]);
  
  // Diagonals should be half a cycle apart
  double phase_diff = std::fmod(phase_offsets_[RF] - phase_offsets_[LF] + 1.0, 1.0);
  EXPECT_DOUBLE_EQ(phase_diff, 0.5);
}

// Test 2: Foot trajectory swing phase
TEST_F(GaitControllerTest, SwingPhaseTrajectory)
{
  double step_length = 0.1;
  double linear_vel = 0.2;
  double angular_vel = 0.0;
  
  // Test at the start of swing phase (phase = 0.0)
  double x_start, y_start, z_start;
  calculateFootTrajectory(LF, 0.0, step_length, linear_vel, angular_vel,
                          x_start, y_start, z_start);
  
  // At start of swing, x should be at -step_length (back)
  EXPECT_NEAR(x_start, 0.35 / 2.0 * 0.5 - step_length, 1e-6);
  EXPECT_NEAR(z_start, -STANCE_HEIGHT, 1e-6);  // On ground
  
  // Test at middle of swing phase (phase = 0.25)
  double x_mid, y_mid, z_mid;
  calculateFootTrajectory(LF, 0.25, step_length, linear_vel, angular_vel,
                          x_mid, y_mid, z_mid);
  
  // At middle of swing, x should be at center (0 offset)
  EXPECT_NEAR(x_mid, 0.35 / 2.0 * 0.5, 1e-6);
  // Z should be at step height (apex)
  EXPECT_NEAR(z_mid, -STANCE_HEIGHT + STEP_HEIGHT, 1e-6);
  
  // Test at end of swing phase (phase = 0.5)
  double x_end, y_end, z_end;
  calculateFootTrajectory(LF, 0.49, step_length, linear_vel, angular_vel,
                          x_end, y_end, z_end);
  
  // At end of swing, x should be at +step_length (forward)
  EXPECT_NEAR(x_end, 0.35 / 2.0 * 0.5 + step_length, 0.01);
  EXPECT_NEAR(z_end, -STANCE_HEIGHT, 0.01);  // Back on ground
}

// Test 3: Foot trajectory stance phase
TEST_F(GaitControllerTest, StancePhaseTrajectory)
{
  double step_length = 0.1;
  double linear_vel = 0.2;
  double angular_vel = 0.0;
  
  // Test at start of stance phase (phase = 0.5)
  double x_start, y_start, z_start;
  calculateFootTrajectory(LF, 0.5, step_length, linear_vel, angular_vel,
                          x_start, y_start, z_start);
  
  // Z should be on ground during stance
  EXPECT_NEAR(z_start, -STANCE_HEIGHT, 1e-6);
  
  // Test at middle of stance phase (phase = 0.75)
  double x_mid, y_mid, z_mid;
  calculateFootTrajectory(LF, 0.75, step_length, linear_vel, angular_vel,
                          x_mid, y_mid, z_mid);
  
  // Z should stay on ground
  EXPECT_NEAR(z_mid, -STANCE_HEIGHT, 1e-6);
  
  // Test at end of stance phase (phase = 1.0)
  double x_end, y_end, z_end;
  calculateFootTrajectory(LF, 0.99, step_length, linear_vel, angular_vel,
                          x_end, y_end, z_end);
  
  // Back to start position
  EXPECT_NEAR(z_end, -STANCE_HEIGHT, 1e-6);
}

// Test 4: Diagonal leg synchronization
TEST_F(GaitControllerTest, DiagonalLegSynchronization)
{
  double step_length = 0.1;
  double linear_vel = 0.2;
  double angular_vel = 0.0;
  
  // Test at multiple phases
  for (double phase = 0.0; phase < 1.0; phase += 0.1) {
    double lf_x, lf_y, lf_z;
    double rr_x, rr_y, rr_z;
    
    calculateFootTrajectory(LF, phase, step_length, linear_vel, angular_vel,
                            lf_x, lf_y, lf_z);
    calculateFootTrajectory(RR, phase, step_length, linear_vel, angular_vel,
                            rr_x, rr_y, rr_z);
    
    // LF and RR (diagonal pair) should have same Z trajectory
    EXPECT_NEAR(lf_z, rr_z, 1e-10) << "Phase: " << phase;
    
    // X trajectory should be mirrored (but same pattern)
    // Y should have opposite signs
    EXPECT_NEAR(lf_y, -rr_y, 1e-10);
  }
}

// Test 5: Turning trajectory offset
TEST_F(GaitControllerTest, TurningOffset)
{
  double step_length = 0.1;
  double linear_vel = 0.0;
  double angular_vel = 1.0;  // High turning rate
  
  // Test at multiple phases
  for (double phase = 0.0; phase < 1.0; phase += 0.25) {
    double lf_x, lf_y, lf_z;
    double rf_x, rf_y, rf_z;
    
    calculateFootTrajectory(LF, phase, step_length, linear_vel, angular_vel,
                            lf_x, lf_y, lf_z);
    calculateFootTrajectory(RF, phase, step_length, linear_vel, angular_vel,
                            rf_x, rf_y, rf_z);
    
    // Left and right legs should have different X positions during turning
    // (opposite sides move in opposite directions for turning in place)
    if (phase > 0.0 && phase < 0.5) {
      // During swing phase, turning offsets should differ
      EXPECT_NE(lf_x, rf_x) << "Phase: " << phase;
    }
  }
}

// Test 6: Zero velocity (standing still)
TEST_F(GaitControllerTest, ZeroVelocity)
{
  double step_length = 0.0;  // No movement
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  
  for (double phase = 0.0; phase < 1.0; phase += 0.1) {
    double x, y, z;
    
    calculateFootTrajectory(LF, phase, step_length, linear_vel, angular_vel,
                            x, y, z);
    
    // When not moving, foot should stay at default position
    double expected_x = 0.35 / 2.0 * 0.5;
    double expected_y = 0.20 / 2.0 * 0.8;
    double expected_z = -STANCE_HEIGHT;
    
    EXPECT_NEAR(x, expected_x, 1e-6) << "Phase: " << phase;
    EXPECT_NEAR(y, expected_y, 1e-6) << "Phase: " << phase;
    EXPECT_NEAR(z, expected_z, 1e-6) << "Phase: " << phase;
  }
}

// Test 7: Phase wraparound
TEST_F(GaitControllerTest, PhaseWraparound)
{
  double step_length = 0.1;
  double linear_vel = 0.2;
  double angular_vel = 0.0;
  
  double x_0, y_0, z_0;
  double x_1, y_1, z_1;
  
  // Phase 0 and phase 1 should be the same (periodic)
  calculateFootTrajectory(LF, 0.0, step_length, linear_vel, angular_vel,
                          x_0, y_0, z_0);
  calculateFootTrajectory(LF, 1.0, step_length, linear_vel, angular_vel,
                          x_1, y_1, z_1);
  
  EXPECT_NEAR(x_0, x_1, 1e-6);
  EXPECT_NEAR(y_0, y_1, 1e-6);
  EXPECT_NEAR(z_0, z_1, 1e-6);
}

// Test 8: Swing height profile
TEST_F(GaitControllerTest, SwingHeightProfile)
{
  double step_length = 0.1;
  double linear_vel = 0.2;
  double angular_vel = 0.0;
  
  // Track Z position through swing phase
  double max_z = -STANCE_HEIGHT;
  for (double phase = 0.0; phase <= 0.5; phase += 0.01) {
    double x, y, z;
    calculateFootTrajectory(LF, phase, step_length, linear_vel, angular_vel,
                            x, y, z);
    max_z = std::max(max_z, z);
  }
  
  // Maximum height should be at step_height
  EXPECT_NEAR(max_z, -STANCE_HEIGHT + STEP_HEIGHT, 0.01);
  
  // Z at start and end of swing should be stance height
  double x_start, y_start, z_start;
  double x_end, y_end, z_end;
  calculateFootTrajectory(LF, 0.0, step_length, linear_vel, angular_vel,
                          x_start, y_start, z_start);
  calculateFootTrajectory(LF, 0.5, step_length, linear_vel, angular_vel,
                          x_end, y_end, z_end);
  
  EXPECT_NEAR(z_start, -STANCE_HEIGHT, 1e-6);
  EXPECT_NEAR(z_end, -STANCE_HEIGHT, 1e-6);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
