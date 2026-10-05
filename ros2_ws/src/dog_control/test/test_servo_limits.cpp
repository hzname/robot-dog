#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "dog_control/locomotion.hpp"
#include "dog_control/servo_limits.hpp"

using dog_control::LocomotionParams;
using dog_control::PeakSpeed;
using dog_control::peakServoSpeed;
using dog_control::ServoSpeedModel;

namespace
{
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
  return p;
}

// Explicit model: the struct defaults move in plan 01-15, the tests must not read them.
ServoSpeedModel model(double max_speed)
{
  ServoSpeedModel s;
  s.max_speed = max_speed;
  s.margin = 0.8;
  s.knee_ratio = 1.0;
  return s;
}
}  // namespace

TEST(ServoLimits, PeakAtShippedGaitIs5p077)
{
  const LocomotionParams p = pinnedParams();
  const PeakSpeed peak = peakServoSpeed(p, model(6.0));
  EXPECT_NEAR(peak.peak, 5.077, 1e-3);
  EXPECT_EQ(peak.joint_kind, 2);

  LocomotionParams still = pinnedParams();
  still.max_velocity = {0.0, 0.0, 0.0};
  const PeakSpeed zero = peakServoSpeed(still, model(6.0));
  EXPECT_DOUBLE_EQ(zero.peak, 0.0);
  EXPECT_EQ(zero.joint_kind, 0);
}

TEST(ServoLimits, KneeRatioScalesOnlyTheKnee)
{
  ServoSpeedModel high = model(6.0);
  high.knee_ratio = 1.388;
  const PeakSpeed scaled = peakServoSpeed(pinnedParams(), high);
  EXPECT_NEAR(scaled.peak, 1.388 * 5.0771, 2e-3);
  EXPECT_EQ(scaled.joint_kind, 2);

  ServoSpeedModel low = model(6.0);
  low.knee_ratio = 0.5;
  const PeakSpeed thigh = peakServoSpeed(pinnedParams(), low);
  EXPECT_NEAR(thigh.peak, 4.232, 1e-3);
  EXPECT_EQ(thigh.joint_kind, 1);
}
