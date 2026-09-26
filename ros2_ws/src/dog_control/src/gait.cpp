#include "dog_control/gait.hpp"

#include <algorithm>
#include <cmath>

namespace dog_control
{

namespace
{
constexpr double kLinearEps = 0.005;   // [m/s]
constexpr double kAngularEps = 0.02;   // [rad/s]
constexpr double kNeutralTol = 0.005;  // [m]

double frac(double v) {return v - std::floor(v);}
}  // namespace

TrotGait::TrotGait(const GaitParams & params, const std::array<Vec3, kNumLegs> & neutral)
: params_(params), neutral_(neutral)
{
  params_.period = std::max(params_.period, 0.1);
  params_.duty = std::clamp(params_.duty, 0.5, 0.9);
  for (auto & n : neutral_) {n.z = 0.0;}
  step_heights_.fill(params_.step_height);
  reset();
}

bool TrotGait::isIdle(const BodyVelocity & v)
{
  return std::abs(v.vx) < kLinearEps && std::abs(v.vy) < kLinearEps && std::abs(v.wz) < kAngularEps;
}

void TrotGait::reset()
{
  feet_ = neutral_;
  swing_blend_.fill(0.0);
  in_swing_.fill(false);
  phase_ = 0.0;
  stepping_ = false;
}

void TrotGait::update(double dt, const BodyVelocity & cmd)
{
  const bool idle = isIdle(cmd);
  if (!stepping_) {
    if (idle) {
      return;
    }
    stepping_ = true;
    phase_ = 0.0;
    in_swing_.fill(false);
  }

  const double swing_frac = 1.0 - params_.duty;
  const double t_stance = params_.duty * params_.period;
  phase_ = frac(phase_ + dt / params_.period);

  bool touchdown = false;
  std::array<bool, kNumLegs> swing_now{};
  std::array<double, kNumLegs> swing_s{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const double psi = frac(phase_ + params_.phase_offsets[leg]);
    swing_now[leg] = psi < swing_frac;
    swing_s[leg] = psi / swing_frac;
    if (in_swing_[leg] && !swing_now[leg]) {
      touchdown = true;
    }
  }

  // Stop on a touchdown once every foot is back home. Landing feet are at the
  // end of their arc (lift ~0) and the others have not moved yet this tick.
  if (idle && touchdown) {
    bool home = true;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const Vec3 d = feet_[leg] - neutral_[leg];
      if (std::hypot(d.x, d.y) > kNeutralTol) {home = false;}
    }
    if (home) {
      reset();
      return;
    }
  }

  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 & n = neutral_[leg];
    // Foot travel over one stance for the commanded twist, clamped.
    Vec3 step{(cmd.vx - cmd.wz * n.y) * t_stance, (cmd.vy + cmd.wz * n.x) * t_stance, 0.0};
    const double len = std::hypot(step.x, step.y);
    double k = 1.0;
    if (len > params_.max_step && len > 0.0) {
      k = params_.max_step / len;
      step = step * k;
    }

    Vec3 & p = feet_[leg];
    if (swing_now[leg]) {
      if (!in_swing_[leg]) {
        swing_blend_[leg] = 0.0;
      }
      const double s = std::clamp(swing_s[leg], 0.0, 1.0);
      const Vec3 target{n.x + 0.5 * step.x, n.y + 0.5 * step.y, 0.0};
      // Cover the matching share of the *remaining* distance each tick: the
      // path stays continuous even if the touchdown target jumps mid-swing
      // and still lands exactly on it at s = 1.
      const double blend = 0.5 * (1.0 - std::cos(M_PI * s));
      const double remaining = 1.0 - swing_blend_[leg];
      const double k_move = remaining > 1e-9 ? (blend - swing_blend_[leg]) / remaining : 1.0;
      swing_blend_[leg] = blend;
      p.x += (target.x - p.x) * k_move;
      p.y += (target.y - p.y) * k_move;
      p.z = step_heights_[leg] * std::sin(M_PI * s);
    } else {
      // Foot stays on the ground: move it against the (clamped) body twist.
      const double yaw = -cmd.wz * k * dt;
      const double c = std::cos(yaw);
      const double sn = std::sin(yaw);
      const double x = c * p.x - sn * p.y - cmd.vx * k * dt;
      const double y = sn * p.x + c * p.y - cmd.vy * k * dt;
      p = {x, y, 0.0};
    }
    in_swing_[leg] = swing_now[leg];
  }
}

}  // namespace dog_control
