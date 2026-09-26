#include "dog_control/crawl.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_control
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double smooth(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return 0.5 * (1.0 - std::cos(M_PI * s));
}

double approach(double v, double target, double step)
{
  return v + std::clamp(target - v, -step, step);
}
}  // namespace

// ------------------------------------------------------------------ terrain
double TerrainProfile::height(double y, double x) const
{
  if (!valid() || dx <= 0.0) {return kNaN;}
  const auto & line = y > 0.0 ? left : right;
  const double f = (x - x0) / dx;
  if (f < 0.0 || f > static_cast<double>(line.size() - 1)) {return kNaN;}
  const size_t i = static_cast<size_t>(std::floor(f));
  const size_t j = std::min(i + 1, line.size() - 1);
  const double a = line[i], b = line[j], w = f - static_cast<double>(i);
  if (std::isfinite(a) && std::isfinite(b)) {return a + (b - a) * w;}
  if (std::isfinite(a)) {return a;}
  return b;
}

void TerrainProfile::fillGaps()
{
  for (auto * line : {&left, &right}) {
    auto & v = *line;
    int last = -1;  // last known index
    for (int i = 0; i < static_cast<int>(v.size()); ++i) {
      if (!std::isfinite(v[i])) {continue;}
      if (last >= 0 && i - last > 1) {
        const double h = std::min(v[last], v[i]);
        for (int k = last + 1; k < i; ++k) {v[k] = h;}
      }
      last = i;
    }
  }
}

double TerrainProfile::highest(double y, double xa, double xb) const
{
  if (!valid() || dx <= 0.0) {return kNaN;}
  const auto & line = y > 0.0 ? left : right;
  if (xa > xb) {std::swap(xa, xb);}
  const long i0 = std::max(0L, static_cast<long>(std::floor((xa - x0) / dx)));
  const long i1 = std::min(static_cast<long>(line.size()) - 1, static_cast<long>(std::ceil((xb - x0) / dx)));
  double best = kNaN;
  for (long i = i0; i <= i1; ++i) {
    const double h = line[static_cast<size_t>(i)];
    if (std::isfinite(h) && !(best >= h)) {best = h;}
  }
  return best;
}

// ------------------------------------------------------------------ gait
CrawlGait::CrawlGait(const CrawlParams & params, const std::array<Vec3, kNumLegs> & neutral)
: p_(params), neutral_(neutral)
{
  p_.shift_time = std::max(p_.shift_time, 0.05);
  p_.swing_time = std::max(p_.swing_time, 0.1);
  p_.shift_rate = std::max(p_.shift_rate, 0.005);
  p_.shift_accel = std::max(p_.shift_accel, 0.01);
  for (auto & n : neutral_) {n.z = 0.0;}
  // How long the body takes from one support triangle to the next
  // (neutral feet, mean over the cycle): the phase lasts at least that.
  double d = 0.0;
  for (int k = 0; k < kNumLegs; ++k) {
    const Vec3 & a = neutral_[order_[k]], & b = neutral_[order_[(k + 1) % kNumLegs]];
    d += std::hypot(a.x - b.x, a.y - b.y) / 3.0 / kNumLegs;  // centroids differ by (a - b) / 3
  }
  const double v = p_.shift_rate, acc = p_.shift_accel;
  const double t_move = d > v * v / acc ? d / v + v / acc : 2.0 * std::sqrt(d / acc);
  shift_duration_ = std::max(p_.shift_time, t_move);
  phase_time_.fill(shift_duration_ + p_.swing_time);
  reset(neutral_);
}

void CrawlGait::reset(const std::array<Vec3, kNumLegs> & feet)
{
  for (int leg = 0; leg < kNumLegs; ++leg) {ground_[leg] = {feet[leg].x, feet[leg].y, 0.0};}
  shift_ = {};
  shift_v_ = {};
  phase_clock_ = 0.0;
  phase_ = Phase::SHIFT;
  have_target_ = false;
  k_ = 0;
  t_ = 0.0;
  stepping_ = false;
  have_ref_ = false;
  ref_ = 0.0;
  base_z_ = 0.0;
  pitch_ = 0.0;
}

double CrawlGait::maxSpeed() const
{
  return p_.max_stride / (kNumLegs * (shift_duration_ + p_.swing_time));
}

std::array<Vec3, kNumLegs> CrawlGait::feet() const
{
  std::array<Vec3, kNumLegs> out;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    out[leg] = {ground_[leg].x - shift_.x, ground_[leg].y - shift_.y, ground_[leg].z};
  }
  return out;
}

Vec3 CrawlGait::supportCentroid(int lifted) const
{
  Vec3 c{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (leg != lifted) {c = c + ground_[leg];}
  }
  return {c.x / 3.0, c.y / 3.0, 0.0};
}

double CrawlGait::supportMargin(int lifted, const Vec3 & p) const
{
  std::array<Vec3, 3> t{};
  int n = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (leg != lifted && n < 3) {t[n++] = ground_[leg];}
  }
  const double cross = (t[1].x - t[0].x) * (t[2].y - t[0].y) - (t[1].y - t[0].y) * (t[2].x - t[0].x);
  if (cross < 0) {std::swap(t[1], t[2]);}
  double m = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 3; ++k) {
    const Vec3 & a = t[k], & b = t[(k + 1) % 3];
    const double ex = b.x - a.x, ey = b.y - a.y, len = std::hypot(ex, ey);
    if (len < 1e-9) {return -1.0;}
    m = std::min(m, (ex * (p.y - a.y) - ey * (p.x - a.x)) / len);
  }
  return m;
}

Vec3 CrawlGait::shiftTarget(int lifted, const BodyVelocity & cmd) const
{
  // the body (shift_, ground frame) walks on by the twist until the leg is down
  const double t = shift_duration_ + 2.0 * p_.swing_time;  // a high lift swings up to twice as long
  const Vec3 walk{cmd.vx * t, cmd.vy * t, 0.0};
  auto ok = [&](const Vec3 & p) {
      return std::min(supportMargin(lifted, p), supportMargin(lifted, p + walk));
    };
  const Vec3 c = supportCentroid(lifted), cur = shift_;
  if (ok(cur) >= p_.margin) {return cur;}
  if (ok(c) < p_.margin) {return c;}  // as good as it gets
  double lo = 0.0, hi = 1.0;  // the least move that is enough
  for (int i = 0; i < 12; ++i) {
    const double mid = 0.5 * (lo + hi);
    (ok(cur + (c - cur) * mid) >= p_.margin ? hi : lo) = mid;
  }
  return cur + (c - cur) * hi;
}

double CrawlGait::footholdX(double y, double x, double dir, double max_ahead, const TerrainProfile * t) const
{
  if (!t || !t->valid()) {return x;}
  // height spread around xc, +-4.5 cm: the map blurs an edge over two cells
  // (a riser's points fall into the cell before it), and the foot lands a
  // centimetre or two off (yaw, slip), a stance foot creeps a few cm before
  // it lifts. Partly unknown counts as an edge: the
  // tread below a drop is in the lidars' shadow right beyond the edge.
  // All unknown: nothing to go by, flat.
  auto spread = [&](double xc) {
      double lo = std::numeric_limits<double>::infinity(), hi = -lo;
      int n = 0;
      for (double dx : {-0.045, -0.0225, 0.0, 0.0225, 0.045}) {  // a bar as thin as 2 cm shows
        const double h = t->height(y, xc + dx);
        if (!std::isfinite(h)) {continue;}
        lo = std::min(lo, h);
        hi = std::max(hi, h);
        ++n;
      }
      if (n == 0) {return 0.0;}
      return n < 5 ? std::max(hi - lo, p_.edge) : hi - lo;
    };
  // nearest flat spot, the far side of an edge first (a foot put down short
  // of a riser may meet it again on the next step), but at most max_ahead
  // further; a shorter step down to 12 cm. None flat: the least uneven.
  double best = x, best_spread = std::numeric_limits<double>::infinity();
  for (double d : {0.0, 0.01, -0.01, 0.02, -0.02, 0.03, -0.03, -0.04, 0.04, -0.05, 0.05, -0.06, -0.07, -0.08,
      -0.09, -0.10, -0.11, -0.12})
  {
    if (d > max_ahead + 1e-9) {continue;}
    const double xc = x + d * dir, sp = spread(xc);
    if (sp < p_.edge) {return xc;}
    if (sp < best_spread) {
      best = xc;
      best_spread = sp;
    }
  }
  return best;
}

void CrawlGait::startSwing(const TerrainProfile * t, const BodyVelocity & cmd)
{
  const int leg = order_[k_];
  const Vec3 & n = neutral_[leg];
  // one cycle as the last four legs took it (a high lift or a wait for the
  // support makes it longer): the stance feet travel cmd * T meanwhile
  double T = 0.0;
  for (double pt : phase_time_) {T += pt;}
  const double T0 = kNumLegs * (shift_duration_ + p_.swing_time);
  T = std::clamp(T, T0, 3.0 * T0);
  Vec3 step{(cmd.vx - cmd.wz * n.y) * T, (cmd.vy + cmd.wz * n.x) * T, 0.0};
  const double len = std::hypot(step.x, step.y);
  if (len > p_.max_stride) {step = step * (p_.max_stride / len);}
  from_ = ground_[leg];
  to_ = {n.x + 0.5 * step.x, n.y + 0.5 * step.y, from_.z};
  // terrain lookups are in the body frame (ground minus the body shift)
  const double sx = shift_.x;
  const bool terrain = t && t->valid() && have_ref_;
  if (terrain) {
    // a front foot may go further past an edge than a rear one (whose
    // support triangle it would leave behind the body)
    const bool leading = (step.x < -1e-4) ? n.x < 0.0 : n.x > 0.0;
    to_.x = footholdX(n.y, to_.x - sx, step.x < -1e-4 ? -1.0 : 1.0, leading ? 0.05 : 0.03, t) + sx;
    const double h = t->height(n.y, to_.x - sx);
    if (std::isfinite(h)) {to_.z = h - ref_;}
  }
  const double base = std::max(from_.z, to_.z);
  double top = base;
  if (terrain) {
    const double h = t->highest(n.y, std::min(from_.x, to_.x) - sx - 0.03, std::max(from_.x, to_.x) - sx + 0.03);
    if (std::isfinite(h)) {top = std::max(top, h - ref_);}
  }
  clear_z_ = std::min(std::max(top + p_.clearance, base + 0.03), base + p_.max_lift);
  // a high lift takes longer: the servos run at their speed limit, and a
  // foot that moves on before it is up hits what it should clear
  swing_T_ = p_.swing_time * std::clamp((clear_z_ - std::min(from_.z, to_.z)) / 0.04, 1.0, 2.0);
}

void CrawlGait::update(double dt, const BodyVelocity & cmd_in, const TerrainProfile * t)
{
  twist_ = BodyVelocity{};
  if (dt <= 0.0) {return;}
  const bool idle = TrotGait::isIdle(cmd_in);
  if (t && t->valid() && !have_ref_) {  // the ground under the feet now is the start level
    double sum = 0.0;
    int n = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const double h = t->height(ground_[leg].y, ground_[leg].x - shift_.x);
      if (std::isfinite(h)) {
        sum += h - ground_[leg].z;
        ++n;
      }
    }
    if (n) {
      ref_ = sum / n;
      have_ref_ = true;
    }
  }
  if (!stepping_) {
    moveShift(Vec3{}, dt);  // settle over the middle
    if (idle) {return;}
    stepping_ = true;
    phase_ = Phase::SHIFT;
    have_target_ = false;
    k_ = 0;
    t_ = 0.0;
  }
  // slow enough for three-leg support
  BodyVelocity cmd = cmd_in;
  const double vmax = maxSpeed();
  const double v = std::hypot(cmd.vx, cmd.vy);
  if (v > vmax) {
    cmd.vx *= vmax / v;
    cmd.vy *= vmax / v;
  }
  cmd.wz = std::clamp(cmd.wz, -0.15, 0.15);

  const int swing = phase_ == Phase::SWING ? order_[k_] : -1;
  // three feet down: the body waits rather than walk out of the triangle
  // (a long step, or the best shift not enough)
  if (swing >= 0) {
    const Vec3 next{shift_.x + cmd.vx * dt, shift_.y + cmd.vy * dt, 0.0};
    if (supportMargin(swing, next) < p_.margin - 0.008 &&
      supportMargin(swing, next) < supportMargin(swing, shift_))
    {
      cmd.vx = 0.0;
      cmd.vy = 0.0;
    }
  }
  twist_ = cmd;
  // stance feet stay on the ground: move them against the body twist
  const double yaw = -cmd.wz * dt, c = std::cos(yaw), s = std::sin(yaw);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (leg == swing) {continue;}
    Vec3 & g = ground_[leg];
    const double x = c * g.x - s * g.y - cmd.vx * dt;
    const double y = s * g.x + c * g.y - cmd.vy * dt;
    g.x = x;
    g.y = y;
  }
  if (swing >= 0) {  // the swing's ends are points on the ground as well
    for (Vec3 * p : {&from_, &to_}) {
      const double x = c * p->x - s * p->y - cmd.vx * dt;
      const double y = s * p->x + c * p->y - cmd.vy * dt;
      p->x = x;
      p->y = y;
    }
  }
  // body over the triangle of the feet that stay down
  // body over the triangle of the three feet that stay down: once found
  // for this leg, kept (the stance feet move, it walks on relative to them)
  if (!have_target_) {
    target_ = shiftTarget(order_[k_], cmd);
    have_target_ = true;
  }
  const Vec3 target = target_;
  moveShift(target, dt);

  t_ += dt;
  phase_clock_ += dt;
  double support_swing_z = 0.0;
  if (phase_ == Phase::SHIFT) {
    const bool there = std::hypot(shift_.x - target.x, shift_.y - target.y) < 0.004;
    // never lift a leg before the body is over the other three
    if (t_ >= p_.shift_time && there) {
      startSwing(t, cmd);
      phase_ = Phase::SWING;
      t_ = 0.0;
    }
  }
  if (phase_ == Phase::SWING) {
    const int leg = order_[k_];
    const double u = std::min(t_ / swing_T_, 1.0);
    const double bxy = smooth((u - 0.35) / 0.4);  // along once up, down once there
    Vec3 & g = ground_[leg];
    g.x = from_.x + (to_.x - from_.x) * bxy;
    g.y = from_.y + (to_.y - from_.y) * bxy;
    if (u < 0.35) {
      g.z = from_.z + (clear_z_ - from_.z) * smooth(u / 0.35);
    } else if (u < 0.75) {
      g.z = clear_z_;
    } else {
      g.z = clear_z_ + (to_.z - clear_z_) * smooth((u - 0.75) / 0.25);
    }
    support_swing_z = from_.z + (to_.z - from_.z) * smooth(u);
    if (u >= 1.0) {
      g = to_;
      phase_time_[k_] = phase_clock_;
      phase_clock_ = 0.0;
      k_ = (k_ + 1) % kNumLegs;
      phase_ = Phase::SHIFT;
      have_target_ = false;
      t_ = 0.0;
      if (idle) {  // stop once every foot is home
        bool home = true;
        for (int l = 0; l < kNumLegs; ++l) {
          if (std::hypot(ground_[l].x - neutral_[l].x, ground_[l].y - neutral_[l].y) > 0.01) {home = false;}
        }
        if (home) {stepping_ = false;}
      }
    }
  }
  // the body follows the mean support height
  double sum = 0.0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    sum += (phase_ == Phase::SWING && leg == order_[k_]) ? support_swing_z : ground_[leg].z;
  }
  // (low-passed: no step in the body's vertical speed)
  base_z_ = approach(base_z_, base_z_ + (sum / kNumLegs - base_z_) * std::min(1.0, dt / 0.3), 0.1 * dt);
  // pitch along the support: front minus rear footholds (the swing leg at
  // the height it goes to)
  double zf = 0.0, zr = 0.0, xf = 0.0, xr = 0.0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const bool swinging = phase_ == Phase::SWING && leg == order_[k_];
    const double z = swinging ? to_.z : ground_[leg].z;
    (neutral_[leg].x > 0 ? zf : zr) += 0.5 * z;
    (neutral_[leg].x > 0 ? xf : xr) += 0.5 * neutral_[leg].x;
  }
  const double goal = std::clamp(-std::atan2(zf - zr, std::max(xf - xr, 0.05)), -p_.max_pitch, p_.max_pitch);
  pitch_ = approach(pitch_, pitch_ + (goal - pitch_) * std::min(1.0, dt / 0.5), p_.pitch_rate * dt);
}

void CrawlGait::moveShift(const Vec3 & target, double dt)
{
  // Speed limited, accelerating and braking at shift_accel: the body is
  // pushed by the feet, and a step change of its speed slides them.
  const double dx = target.x - shift_.x, dy = target.y - shift_.y, d = std::hypot(dx, dy);
  const double a = std::max(p_.shift_accel, 1e-3);
  if (d < 1e-4 && std::hypot(shift_v_.x, shift_v_.y) < 2.0 * a * dt) {  // there
    shift_ = target;
    shift_v_ = {};
    return;
  }
  Vec3 want{};
  if (d > 1e-6) {
    const double v = std::min(p_.shift_rate, std::sqrt(2.0 * a * d));
    want = {dx / d * v, dy / d * v, 0.0};
  }
  const double ex = want.x - shift_v_.x, ey = want.y - shift_v_.y, e = std::hypot(ex, ey);
  const double k = e > a * dt ? a * dt / e : 1.0;
  shift_v_.x += ex * k;
  shift_v_.y += ey * k;
  shift_.x += shift_v_.x * dt;
  shift_.y += shift_v_.y * dt;
}

}  // namespace dog_control
