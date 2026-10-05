#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "dog_control/locomotion.hpp"
#include "dog_control/servo_limits.hpp"

using dog_control::BodyVelocity;
using dog_control::kMaxAutoPeriod;
using dog_control::kNumJoints;
using dog_control::kPeriodScanStep;
using dog_control::kSpeedTolerance;
using dog_control::LocomotionController;
using dog_control::LocomotionParams;
using dog_control::minimalPeriod;
using dog_control::PeakSpeed;
using dog_control::peakServoSpeed;
using dog_control::ServoSpeedModel;

namespace
{
constexpr double kTickDt = 0.02;  // [s] the controller tick the procedure runs at
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

// Explicit model: the struct defaults move in plan 01-15, the tests must not read them.
ServoSpeedModel model(double max_speed)
{
  ServoSpeedModel s;
  s.max_speed = max_speed;
  s.margin = 0.8;
  s.knee_ratio = 1.0;
  return s;
}

// Peak of the pinned set at one period (the public peakServoSpeed path).
double peakAt(double period, const ServoSpeedModel & s)
{
  LocomotionParams p = pinnedParams();
  p.gait.period = period;
  return peakServoSpeed(p, s).peak;
}

// The five extreme commands of the JointSpeedsFitTheServos procedure.
std::array<BodyVelocity, 5> extremeCommandsOf(const LocomotionParams & p)
{
  const BodyVelocity m = p.max_velocity;
  return {{
    {m.vx, m.vy, m.wz},
    {m.vx, 0.0, 0.0},
    {-m.vx, 0.0, 0.0},
    {0.0, m.vy, 0.0},
    {0.0, 0.0, m.wz},
  }};
}

// The JointSpeedsFitTheServos procedure, unchanged: stand (2 s), ramp (1 s),
// then the max joint speed over the 4 s window. The knee is not scaled here.
double controllerPeak(const LocomotionParams & p, const BodyVelocity & cmd)
{
  LocomotionController c(p);
  c.request("stand");
  for (int i = 0; i < 100; ++i) {c.update(kTickDt);}
  c.setVelocity(cmd);
  for (int i = 0; i < 50; ++i) {c.update(kTickDt);}
  auto prev = c.joints();
  double max_speed = 0.0;
  for (int i = 0; i < 200; ++i) {
    c.update(kTickDt);
    for (int j = 0; j < kNumJoints; ++j) {
      max_speed = std::max(max_speed, std::abs(c.joints()[j] - prev[j]) / kTickDt);
    }
    prev = c.joints();
  }
  return max_speed;
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

TEST(ServoLimits, MinimalPeriodTable)
{
  // With the guard window, planned on the real sources 2026-09-30 (GCC 14).
  const std::array<double, 7> speeds{3.5, 4.0, 5.0, 6.0, 6.35, 6.4, 7.0};
  const std::array<double, 7> expected{1.0300, 0.9050, 0.7200, 0.6000, 0.5575, 0.5500, 0.5500};
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(minimalPeriod(pinnedParams(), model(speeds[i]), 0.55, kMaxAutoPeriod),
                expected[i], 1e-9) << "speed " << speeds[i];
  }
}

TEST(ServoLimits, PeriodNeverBelowMinimum)
{
  for (double v : {7.0, 8.0, 12.0, 100.0}) {
    EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(v), 0.55, kMaxAutoPeriod), 0.55);
  }
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(7.0), 0.60, kMaxAutoPeriod), 0.60);
  for (double v : {3.5, 4.0, 5.0, 6.0, 6.35, 6.4, 7.0}) {
    EXPECT_GE(minimalPeriod(pinnedParams(), model(v), 0.55, kMaxAutoPeriod), 0.55);
  }
}

TEST(ServoLimits, NoFitReturnsZero)
{
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(1.0), 0.55, kMaxAutoPeriod), 0.0);

  ServoSpeedModel no_margin = model(6.0);
  no_margin.margin = 0.0;
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), no_margin, 0.55, kMaxAutoPeriod), 0.0);

  ServoSpeedModel nan_speed = model(6.0);
  nan_speed.max_speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), nan_speed, 0.55, kMaxAutoPeriod), 0.0);

  ServoSpeedModel no_ratio = model(6.0);
  no_ratio.knee_ratio = 0.0;
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), no_ratio, 0.55, kMaxAutoPeriod), 0.0);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(6.0), nan, kMaxAutoPeriod), 0.0);
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(6.0), 0.9, 0.6), 0.0);
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(6.0), 0.05, kMaxAutoPeriod), 0.0);

  // The upper boundary is inclusive: 1.03 fits (window clipped), 1.0275 does not.
  EXPECT_NEAR(minimalPeriod(pinnedParams(), model(3.5), 0.55, 1.03), 1.03, 1e-9);
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), model(3.5), 0.55, 1.0275), 0.0);
}

TEST(ServoLimits, WindowSkipsFragileFirstFit)
{
  // Premise: the peak is not monotonic on the fine grid (it rises somewhere).
  LocomotionParams p = pinnedParams();
  double previous = 0.0;
  bool grows = false;
  for (int i = 0; i <= 650; ++i) {
    p.gait.period = 0.55 + i * 0.001;
    const double peak = peakServoSpeed(p, model(6.0)).peak;
    if (i > 0 && peak > previous + kSpeedTolerance) {grows = true;}
    previous = peak;
  }
  EXPECT_TRUE(grows) << "the peak must rise somewhere on the 1 ms grid";

  // Speed 4.0: 0.8975 fits alone, but a period within the guard window does not.
  EXPECT_LE(peakAt(0.8975, model(4.0)), 0.8 * 4.0 + kSpeedTolerance);
  bool neighbor_fails = false;
  for (int i = 0; i <= 20; ++i) {
    if (peakAt(0.8975 + i * kPeriodScanStep, model(4.0)) > 0.8 * 4.0 + kSpeedTolerance) {
      neighbor_fails = true;
      break;
    }
  }
  EXPECT_TRUE(neighbor_fails) << "0.9025 must sit above the gate";
  EXPECT_GT(minimalPeriod(pinnedParams(), model(4.0), 0.55, kMaxAutoPeriod), 0.8975);

  // Speed 6.35: the same at the verified minimum period.
  EXPECT_LE(peakAt(0.55, model(6.35)), 0.8 * 6.35 + kSpeedTolerance);
  neighbor_fails = false;
  for (int i = 0; i <= 20; ++i) {
    if (peakAt(0.55 + i * kPeriodScanStep, model(6.35)) > 0.8 * 6.35 + kSpeedTolerance) {
      neighbor_fails = true;
      break;
    }
  }
  EXPECT_TRUE(neighbor_fails) << "0.555 must sit above the gate";
  EXPECT_GT(minimalPeriod(pinnedParams(), model(6.35), 0.55, kMaxAutoPeriod), 0.55);
}

TEST(ServoLimits, MinimalPeriodMatchesBruteForce)
{
  const int last = static_cast<int>(std::floor((kMaxAutoPeriod - 0.55) / kPeriodScanStep + kSpeedTolerance));
  for (double v : {3.5, 4.0, 5.0, 6.0, 6.35, 6.4}) {
    const ServoSpeedModel s = model(v);
    std::vector<double> peaks(last + 1);
    LocomotionParams q = pinnedParams();
    for (int k = 0; k <= last; ++k) {
      q.gait.period = 0.55 + k * kPeriodScanStep;
      peaks[k] = peakServoSpeed(q, s).peak;
    }
    const double allowed = s.margin * s.max_speed;
    double expected = 0.0;
    for (int k = 0; k <= last; ++k) {
      bool ok = true;
      const int end = std::min(k + 20, last);
      for (int j = k; j <= end; ++j) {
        if (peaks[j] > allowed + kSpeedTolerance) {ok = false; break;}
      }
      if (ok) {
        expected = 0.55 + k * kPeriodScanStep;
        break;
      }
    }
    EXPECT_NEAR(minimalPeriod(pinnedParams(), s, 0.55, kMaxAutoPeriod), expected, 1e-12)
        << "speed " << v;
  }
}

TEST(ServoLimits, ToleranceIsOneNanoUnit)
{
  ServoSpeedModel s = model(6.0);
  s.margin = 1.0;
  const double peak = peakAt(0.55, s);
  s.max_speed = peak - 5e-10;
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), s, 0.55, 0.55), 0.55);
  s.max_speed = peak - 2e-9;
  EXPECT_DOUBLE_EQ(minimalPeriod(pinnedParams(), s, 0.55, 0.55), 0.0);
}

TEST(ServoLimits, PeakMatchesController)
{
  // Set A: the reference periods, the v1 geometry of the pinned set spelled out.
  const std::array<double, 5> periods{0.45, 0.55, 0.65, 0.75, 0.95};
  const std::array<double, 5> references{5.971, 5.077, 4.419, 3.831, 3.027};
  for (int i = 0; i < 5; ++i) {
    LocomotionParams p = pinnedParams();
    p.gait.period = periods[i];
    p.foot_offset_x = 0.0;
    p.foot_offset_y = 0.0;
    p.hip_x = 0.09;
    p.stand_height = 0.15;
    p.transition_time = 1.5;
    const double model_peak = peakServoSpeed(p, model(6.0)).peak;
    EXPECT_NEAR(model_peak, references[i], 1e-3) << "period " << periods[i];
    double controller_max = 0.0;
    for (const BodyVelocity & cmd : extremeCommandsOf(p)) {
      controller_max = std::max(controller_max, controllerPeak(p, cmd));
    }
    EXPECT_NEAR(model_peak, controller_max, 5e-4) << "period " << periods[i];
  }

  // Set B: foot offsets shift the neutral feet away from set A.
  LocomotionParams b = pinnedParams();
  b.gait.period = 0.65;
  b.foot_offset_x = 0.01;
  b.foot_offset_y = 0.01;
  b.hip_x = 0.09;
  b.stand_height = 0.15;
  b.transition_time = 1.5;
  const double b_model = peakServoSpeed(b, model(6.0)).peak;
  EXPECT_NEAR(b_model, 4.737, 1e-3) << "set B reference";
  double b_controller = 0.0;
  for (const BodyVelocity & cmd : extremeCommandsOf(b)) {
    b_controller = std::max(b_controller, controllerPeak(b, cmd));
  }
  EXPECT_NEAR(b_model, b_controller, 5e-4) << "set B controller";

  // Set C: different hip_x and stand_height.
  LocomotionParams c = pinnedParams();
  c.gait.period = 0.65;
  c.foot_offset_x = 0.0;
  c.foot_offset_y = 0.0;
  c.hip_x = 0.10;
  c.stand_height = 0.14;
  c.transition_time = 1.5;
  const double c_model = peakServoSpeed(c, model(6.0)).peak;
  EXPECT_NEAR(c_model, 4.226, 1e-3) << "set C reference";
  double c_controller = 0.0;
  for (const BodyVelocity & cmd : extremeCommandsOf(c)) {
    c_controller = std::max(c_controller, controllerPeak(c, cmd));
  }
  EXPECT_NEAR(c_model, c_controller, 5e-4) << "set C controller";
}
