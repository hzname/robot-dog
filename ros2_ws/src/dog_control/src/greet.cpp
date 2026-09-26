#include "dog_control/greet.hpp"

#include <algorithm>
#include <cmath>

namespace dog_control
{

namespace
{
constexpr double kDeg = M_PI / 180.0;

double minJerk(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

Vec3 lerp(const Vec3 & a, const Vec3 & b, double s)
{
  return a + (b - a) * s;
}

/// Mean of the feet other than `lifted` (x, y; z = 0).
Vec3 centroid(const std::array<Vec3, kNumLegs> & f, int lifted)
{
  Vec3 c{};
  int n = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (leg == lifted) {continue;}
    c = c + f[leg];
    ++n;
  }
  return {c.x / n, c.y / n, 0.0};
}

/// Body frame (nose up th) -> world offset.
Vec3 rotate(double th, const Vec3 & b)
{
  return {b.x * std::cos(th) - b.z * std::sin(th), b.y, b.x * std::sin(th) + b.z * std::cos(th)};
}
}  // namespace

GreetSequence::GreetSequence(
  const GreetParams & params, const std::array<Vec3, kNumLegs> & neutral, double stand_height,
  double thigh, double calf)
: p_(params), neutral_(neutral), stand_height_(stand_height), thigh_(thigh), calf_(calf)
{
  p_.speed = std::max(p_.speed, 0.1);
  for (auto & n : neutral_) {n.z = 0.0;}
  hip_x_ = -neutral_[LR].x;
  build();
}

double GreetSequence::thighFor(double x, double th) const
{
  const double s = (x - hip_x_ * std::cos(th) - knee_x_) / thigh_;
  return std::asin(std::clamp(s, -1.0, 1.0));
}

GreetFrame GreetSequence::kneelFrame(double th, double b, const GreetFrame & feet_from) const
{
  GreetFrame f = feet_from;
  // rear knees on the ground at knee_x_, calves flat: the hips on a circle
  // of the thigh's length about the knees
  const Vec3 hip{knee_x_ + thigh_ * std::sin(b), 0.0, p_.contact_r + thigh_ * std::cos(b)};
  f.body = hip + rotate(th, {hip_x_, 0.0, 0.0});
  f.pitch = -th;
  for (int leg : {LR, RR}) {
    f.feet[leg] = {knee_x_ + calf_, neutral_[leg].y, p_.contact_r};
    f.down[leg] = true;
  }
  f.kneeling = true;
  f.knee_x = knee_x_;
  // paws in the air go with the body: in front of the chest
  for (int leg : {LF, RF}) {
    if (!f.down[leg]) {
      f.feet[leg] = f.body + rotate(th, {neutral_[leg].x + 0.06, neutral_[leg].y, -0.08});
    }
  }
  return f;
}

void GreetSequence::build()
{
  segs_.clear();
  const double v = 1.0 / p_.speed;
  const double h = stand_height_;
  GreetFrame f;
  f.body = {0.0, 0.0, h};
  f.feet = neutral_;
  start_frame_ = f;

  auto add = [&](const GreetFrame & to, double time, int swing = -1) {
      Segment s;
      s.to = to;
      s.time = time * v;
      s.swing = swing;
      segs_.push_back(s);
      f = to;
    };
  auto addKneel = [&](double th0, double b0, double th1, double b1, const GreetFrame & feet_from, double time,
    int swing = -1) {
      Segment s;
      s.to = kneelFrame(th1, b1, feet_from);
      s.time = time * v;
      s.swing = swing;
      s.kneel = true;
      s.th0 = th0;
      s.b0 = b0;
      s.th1 = th1;
      s.b1 = b1;
      segs_.push_back(s);
      f = s.to;
    };

  // 1. rear feet forward, one at a time over the other three
  for (int leg : {LR, RR}) {
    GreetFrame g = f;
    const Vec3 c = centroid(f.feet, leg);
    g.body = {c.x, c.y, h};
    add(g, 1.2);
    g.feet[leg].x = p_.rear_x;
    add(g, 0.9, leg);
  }
  {
    GreetFrame g = f;
    g.body = {centroid(f.feet, -1).x, 0.0, h};
    add(g, 1.0);
  }
  const GreetFrame standing = f;
  // the knees go down where the flat calves put the rear feet's contacts at rear_x
  knee_x_ = p_.rear_x - (calf_ - p_.contact_r);
  const double contact = rearContactX();
  // 2. kneel: the hips stay ahead of the knees, the centre of mass over the
  // four feet (margin ahead of the rear feet)
  const double b_kneel = thighFor(contact + p_.margin, 0.0);
  add(kneelFrame(0.0, b_kneel, f), 2.5);
  // 3. lean back: the centre of mass to the middle of knees..rear feet, the
  // front feet still down
  const double mid = 0.5 * (knee_x_ + contact);
  const double sit = p_.sit_deg * kDeg, beg = p_.beg_deg * kDeg;
  const double b_sit = thighFor(mid, sit), b_beg = thighFor(mid, beg);
  addKneel(0.0, b_kneel, sit, b_sit, f, 2.0);
  const GreetFrame sitting = f;
  // 4. front paws up, then lean back a little more
  {
    GreetFrame g = f;
    g.down[LF] = g.down[RF] = false;
    addKneel(sit, b_sit, sit, b_sit, g, 1.2, -2);
  }
  addKneel(sit, b_sit, beg, b_beg, f, 1.2);
  const GreetFrame begging = f;
  add(begging, p_.hold);
  // 5. wave the left paw
  for (int k = 0; k < p_.waves; ++k) {
    GreetFrame w = begging;
    w.feet[LF].z += 0.04;
    w.feet[LF].x += 0.01;
    add(w, 0.35);
    add(begging, 0.35);
  }
  add(begging, p_.hold);
  // 6. the way back
  addKneel(beg, b_beg, sit, b_sit, f, 1.2);
  addKneel(sit, b_sit, sit, b_sit, sitting, 1.2, -2);  // paws down where they were
  addKneel(sit, b_sit, 0.0, b_kneel, f, 2.0);
  add(standing, 2.5);
  for (int leg : {RR, LR}) {
    GreetFrame g = f;
    const Vec3 c = centroid(f.feet, leg);
    g.body = {c.x, c.y, h};
    add(g, 1.2);
    g.feet[leg] = neutral_[leg];
    add(g, 0.9, leg);
  }
  add(start_frame_, 1.0);
}

double GreetSequence::duration() const
{
  double t = 0.0;
  for (const auto & s : segs_) {t += s.time;}
  return t;
}

void GreetSequence::start()
{
  active_ = true;
  k_ = 0;
  t_ = 0.0;
  from_ = start_frame_;
  frame_ = start_frame_;
}

GreetFrame GreetSequence::blend(const Segment & s, double u) const
{
  const double m = minJerk(u);
  GreetFrame out;
  if (s.kneel) {
    // knees stay on the ground: the body moves by pitch and thigh angle
    out = kneelFrame(s.th0 + (s.th1 - s.th0) * m, s.b0 + (s.b1 - s.b0) * m, s.to);
  } else {
    out = s.to;
    out.body = lerp(from_.body, s.to.body, m);
    out.pitch = from_.pitch + (s.to.pitch - from_.pitch) * m;
    out.kneeling = from_.kneeling && s.to.kneeling;
    for (int leg = 0; leg < kNumLegs; ++leg) {out.feet[leg] = lerp(from_.feet[leg], s.to.feet[leg], m);}
  }
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const bool swings = s.swing == leg || (s.swing == -2 && (leg == LF || leg == RF));
    if (!swings) {continue;}
    // up first, then along, then down: never dragged over the ground
    out.feet[leg] = lerp(from_.feet[leg], s.to.feet[leg], minJerk((u - 0.2) / 0.6));
    out.feet[leg].z += p_.lift * std::sin(M_PI * std::clamp(u, 0.0, 1.0));
    out.down[leg] = false;
  }
  return out;
}

void GreetSequence::update(double dt)
{
  if (!active_) {return;}
  t_ += dt;
  while (k_ < segs_.size() && t_ >= segs_[k_].time) {
    t_ -= segs_[k_].time;
    from_ = segs_[k_].to;
    ++k_;
  }
  if (k_ >= segs_.size()) {
    frame_ = start_frame_;
    active_ = false;
    return;
  }
  frame_ = blend(segs_[k_], t_ / segs_[k_].time);
}

}  // namespace dog_control
