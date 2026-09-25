#include <gtest/gtest.h>

#include <cmath>

#include "dog_control/gait.hpp"

using dog_control::BodyVelocity;
using dog_control::GaitParams;
using dog_control::kNumLegs;
using dog_control::TrotGait;
using dog_control::Vec3;

namespace
{
const std::array<Vec3, kNumLegs> kNeutral{{
  {0.09, 0.115, 0}, {0.09, -0.115, 0}, {-0.09, 0.115, 0}, {-0.09, -0.115, 0}}};
constexpr double kDt = 0.02;

double dist(const Vec3 & a, const Vec3 & b) {return std::hypot(a.x - b.x, a.y - b.y);}
}  // namespace

TEST(Gait, FourLegSupportWithDutyAboveHalf)
{
  TrotGait gait(GaitParams{}, kNeutral);  // shipped duty 0.65
  int all_down = 0;
  for (int i = 0; i < 100; ++i) {
    gait.update(kDt, {0.1, 0, 0});
    EXPECT_EQ(gait.inSwing(0), gait.inSwing(3));
    EXPECT_EQ(gait.inSwing(1), gait.inSwing(2));
    EXPECT_FALSE(gait.inSwing(0) && gait.inSwing(1));  // never 4 legs in the air
    if (!gait.inSwing(0) && !gait.inSwing(1)) {++all_down;}
  }
  EXPECT_GT(all_down, 20);
}

TEST(Gait, IdleDoesNotStep)
{
  TrotGait gait(GaitParams{}, kNeutral);
  for (int i = 0; i < 100; ++i) {gait.update(kDt, {});}
  EXPECT_FALSE(gait.stepping());
  for (int l = 0; l < kNumLegs; ++l) {
    EXPECT_DOUBLE_EQ(gait.feet()[l].z, 0.0);
    EXPECT_NEAR(dist(gait.feet()[l], kNeutral[l]), 0.0, 1e-12);
  }
}

TEST(Gait, DiagonalPairsSwingTogether)
{
  GaitParams p;
  p.duty = 0.5;  // pure trot: exactly one pair in the air at any time
  TrotGait gait(p, kNeutral);
  for (int i = 0; i < 60; ++i) {
    gait.update(kDt, {0.1, 0, 0});
    EXPECT_EQ(gait.inSwing(0), gait.inSwing(3));  // LF & RR
    EXPECT_EQ(gait.inSwing(1), gait.inSwing(2));  // RF & LR
    EXPECT_NE(gait.inSwing(0), gait.inSwing(1));  // pure trot: pairs alternate
    int lifted = 0;
    for (int l = 0; l < kNumLegs; ++l) {
      if (!gait.inSwing(l)) {EXPECT_DOUBLE_EQ(gait.feet()[l].z, 0.0);}
      if (gait.feet()[l].z > 1e-6) {++lifted;}
    }
    EXPECT_LE(lifted, 2);
  }
}

TEST(Gait, StanceFeetMoveBackwardsWhenWalkingForward)
{
  GaitParams p;
  TrotGait gait(p, kNeutral);
  const double v = 0.1;
  for (int i = 0; i < 200; ++i) {
    const auto before = gait.feet();
    const std::array<bool, 4> swing_before{gait.inSwing(0), gait.inSwing(1), gait.inSwing(2), gait.inSwing(3)};
    gait.update(kDt, {v, 0, 0});
    for (int l = 0; l < kNumLegs; ++l) {
      if (swing_before[l] || gait.inSwing(l)) {continue;}
      EXPECT_NEAR(gait.feet()[l].x - before[l].x, -v * kDt, 1e-9);
      EXPECT_NEAR(gait.feet()[l].y - before[l].y, 0.0, 1e-9);
    }
  }
  // Feet stay within half a step of neutral (plus swing interpolation slack).
  const double half_step = 0.5 * v * p.duty * p.period;
  for (int l = 0; l < kNumLegs; ++l) {
    EXPECT_LE(std::abs(gait.feet()[l].x - kNeutral[l].x), half_step + 0.005);
  }
}

TEST(Gait, StepLengthIsClamped)
{
  GaitParams p;
  p.max_step = 0.04;
  TrotGait gait(p, kNeutral);
  double max_dev = 0.0;
  for (int i = 0; i < 300; ++i) {
    gait.update(kDt, {5.0, 0, 0});  // absurd command
    for (int l = 0; l < kNumLegs; ++l) {
      const double dev = std::abs(gait.feet()[l].x - kNeutral[l].x);
      // The very first stance starts from neutral and travels a full step.
      EXPECT_LE(dev, p.max_step + 1e-3);
      if (i >= 25) {max_dev = std::max(max_dev, dev);}
    }
  }
  // Steady state: feet oscillate within +-half a step around neutral, up to
  // one tick of travel (stance is 12.5 ticks long at 50 Hz).
  const double tick_travel = p.max_step / (p.duty * p.period) * kDt;
  EXPECT_LE(max_dev, 0.5 * p.max_step + tick_travel);
}

TEST(Gait, TurningInPlaceMovesFeetTangentially)
{
  TrotGait gait(GaitParams{}, kNeutral);
  for (int i = 0; i < 200; ++i) {
    const auto before = gait.feet();
    const std::array<bool, 4> swing_before{gait.inSwing(0), gait.inSwing(1), gait.inSwing(2), gait.inSwing(3)};
    gait.update(kDt, {0, 0, 0.5});
    for (int l = 0; l < kNumLegs; ++l) {
      if (swing_before[l] || gait.inSwing(l)) {continue;}
      // CCW body yaw -> stance feet rotate CW about the body centre.
      const double cross = before[l].x * gait.feet()[l].y - before[l].y * gait.feet()[l].x;
      EXPECT_LT(cross, 0.0);
      EXPECT_NEAR(std::hypot(gait.feet()[l].x, gait.feet()[l].y),
        std::hypot(before[l].x, before[l].y), 1e-9);
    }
  }
}

TEST(Gait, StopsWithAllFeetAtNeutral)
{
  TrotGait gait(GaitParams{}, kNeutral);
  for (int i = 0; i < 77; ++i) {gait.update(kDt, {0.12, 0.03, 0.3});}
  ASSERT_TRUE(gait.stepping());
  int ticks = 0;
  while (gait.stepping() && ticks < 200) {
    gait.update(kDt, {});
    ++ticks;
  }
  EXPECT_FALSE(gait.stepping());
  EXPECT_LE(ticks * kDt, 2.0 * GaitParams{}.period + 1e-9);  // at most one full cycle + margin
  for (int l = 0; l < kNumLegs; ++l) {
    EXPECT_LT(dist(gait.feet()[l], kNeutral[l]), 0.005);
    EXPECT_DOUBLE_EQ(gait.feet()[l].z, 0.0);
  }
}

TEST(Gait, SwingIsContinuous)
{
  TrotGait gait(GaitParams{}, kNeutral);
  auto prev = gait.feet();
  for (int i = 0; i < 300; ++i) {
    const double vx = (i / 50) % 2 ? 0.15 : -0.05;  // abrupt command changes
    gait.update(kDt, {vx, 0.0, 0.0});
    for (int l = 0; l < kNumLegs; ++l) {
      const Vec3 & f = gait.feet()[l];
      // Instant command reversals (impossible behind locomotion's accel limit)
      // still only speed the swing up; nothing teleports (< 1.25 m/s here).
      EXPECT_LT(dist(f, prev[l]), 0.025) << "leg " << l << " tick " << i;
      EXPECT_LT(std::abs(f.z - prev[l].z), 0.02);
    }
    prev = gait.feet();
  }
}
