#include "dog_control/servo_limits.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "dog_control/locomotion.hpp"

namespace dog_control
{

namespace
{
constexpr double kTickDt = 0.02;  // [s] control tick; the procedure runs at 50 Hz
constexpr double kWarmUp = 1.0;   // [s] acceleration before the measured window
constexpr double kWindow = 4.0;   // [s] measured window
constexpr double kMinGaitPeriod = 0.1;  // [s] TrotGait clamps its own period to this

double approach(double current, double target, double max_delta)
{
  return current + std::clamp(target - current, -max_delta, max_delta);
}

// Same formula as the anonymous helper in locomotion.cpp (private there, and
// that file belongs to plan 01-08); PeakMatchesController catches any drift.
std::array<Vec3, kNumLegs> neutralFeet(const LocomotionParams & p)
{
  std::array<Vec3, kNumLegs> out{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    out[leg] = {
      legFront(leg) * p.hip_x + p.foot_offset_x,
      legSide(leg) * (p.hip_y + p.leg.hip + p.foot_offset_y),
      0.0};
  }
  return out;
}

// Joint targets for the current gait pose: flat ground, body pose zero, the
// stand height of the controller before any pose or slope shift.
std::array<double, kNumJoints> solveFeet(const LocomotionParams & p, const TrotGait & gait)
{
  std::array<double, kNumJoints> q{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 f = gait.feet()[leg];
    const Vec3 rel{f.x, f.y, -p.stand_height + f.z};
    const Vec3 hip{legFront(leg) * p.hip_x, legSide(leg) * p.hip_y, 0.0};
    const IkResult ik = inverseKinematics(p.leg, legSide(leg), rel - hip, p.knee_direction);
    for (int j = 0; j < 3; ++j) {q[leg * 3 + j] = ik.q[j];}
  }
  return q;
}

// The five extreme commands of JointSpeedsFitTheServos, combined command first
// (it violates the gate first in most cases and saves time in the scan).
std::array<BodyVelocity, 5> extremeCommands(const LocomotionParams & p)
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

PeakSpeed peakForCommand(const LocomotionParams & p, const BodyVelocity & cmd, double knee_ratio)
{
  TrotGait gait(p.gait, neutralFeet(p));
  BodyVelocity v{};
  const int warm_up_ticks = static_cast<int>(kWarmUp / kTickDt);
  const int window_ticks = static_cast<int>(kWindow / kTickDt);
  std::array<double, kNumJoints> prev{};
  for (int i = 0; i < warm_up_ticks; ++i) {
    v = {approach(v.vx, cmd.vx, p.max_accel.vx * kTickDt),
         approach(v.vy, cmd.vy, p.max_accel.vy * kTickDt),
         approach(v.wz, cmd.wz, p.max_accel.wz * kTickDt)};
    gait.update(kTickDt, v);
    prev = solveFeet(p, gait);
  }
  PeakSpeed out{};
  for (int i = 0; i < window_ticks; ++i) {
    v = {approach(v.vx, cmd.vx, p.max_accel.vx * kTickDt),
         approach(v.vy, cmd.vy, p.max_accel.vy * kTickDt),
         approach(v.wz, cmd.wz, p.max_accel.wz * kTickDt)};
    gait.update(kTickDt, v);
    const std::array<double, kNumJoints> q = solveFeet(p, gait);
    for (int j = 0; j < kNumJoints; ++j) {
      double speed = std::abs(q[j] - prev[j]) / kTickDt;
      if (j % 3 == 2) {speed *= knee_ratio;}
      if (speed > out.peak) {out = {speed, j % 3};}
    }
    prev = q;
  }
  return out;
}

bool fits(const LocomotionParams & q, const ServoSpeedModel & s, double allowed)
{
  for (const BodyVelocity & cmd : extremeCommands(q)) {
    if (peakForCommand(q, cmd, s.knee_ratio).peak > allowed + kSpeedTolerance) {
      return false;
    }
  }
  return true;
}
}  // namespace

PeakSpeed peakServoSpeed(const LocomotionParams & p, const ServoSpeedModel & s)
{
  PeakSpeed out{};
  for (const BodyVelocity & cmd : extremeCommands(p)) {
    const PeakSpeed peak = peakForCommand(p, cmd, s.knee_ratio);
    if (peak.peak > out.peak) {out = peak;}
  }
  return out;
}

double minimalPeriod(const LocomotionParams & p, const ServoSpeedModel & s, double min_period, double max_period)
{
  const double allowed = s.margin * s.max_speed;
  if (!std::isfinite(allowed) || allowed <= 0.0 || !std::isfinite(s.knee_ratio) || s.knee_ratio <= 0.0 ||
      !std::isfinite(min_period) || !std::isfinite(max_period) ||
      min_period < kMinGaitPeriod || max_period < min_period) {
    return 0.0;
  }
  // Integer grid index: the period of point k is min_period + k * kPeriodScanStep,
  // never an accumulated sum; a fitting point sits on the grid to kSpeedTolerance.
  const int last = static_cast<int>(std::floor((max_period - min_period) / kPeriodScanStep + kSpeedTolerance));
  const int window = static_cast<int>(std::ceil(kPeriodGuard / kPeriodScanStep - kSpeedTolerance));
  LocomotionParams q = p;
  int run_start = -1;
  for (int k = 0; k <= last; ++k) {
    q.gait.period = min_period + k * kPeriodScanStep;
    if (fits(q, s, allowed)) {
      if (run_start < 0) {run_start = k;}
      if (k - run_start >= window || k == last) {
        return min_period + run_start * kPeriodScanStep;
      }
    } else {
      run_start = -1;
    }
  }
  return 0.0;
}

}  // namespace dog_control
