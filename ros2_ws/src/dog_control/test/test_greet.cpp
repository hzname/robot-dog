#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "dog_control/greet.hpp"
#include "dog_control/locomotion.hpp"

using dog_control::GreetFrame;
using dog_control::GreetParams;
using dog_control::GreetSequence;
using dog_control::kNumLegs;
using dog_control::legFront;
using dog_control::legSide;
using dog_control::LocomotionController;
using dog_control::LocomotionParams;
using dog_control::Mode;
using dog_control::Vec3;

namespace
{
constexpr double kDt = 0.02;

std::array<Vec3, kNumLegs> neutral()
{
  return {{{0.09, 0.115, 0.0}, {0.09, -0.115, 0.0}, {-0.09, 0.115, 0.0}, {-0.09, -0.115, 0.0}}};
}

/// How far p lies inside the convex hull of pts (< 0: outside).
double insideHull(std::vector<Vec3> pts, const Vec3 & p)
{
  std::sort(pts.begin(), pts.end(), [](const Vec3 & a, const Vec3 & b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
  auto cross = [](const Vec3 & o, const Vec3 & a, const Vec3 & b) {
      return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
  std::vector<Vec3> h(2 * pts.size());
  size_t k = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) {--k;}
    h[k++] = pts[i];
  }
  for (size_t i = pts.size() - 1, t = k + 1; i-- > 0; ) {
    while (k >= t && cross(h[k - 2], h[k - 1], pts[i]) <= 0) {--k;}
    h[k++] = pts[i];
  }
  h.resize(k - 1);
  if (h.size() < 3) {return -1.0;}
  double m = 1e9;
  for (size_t i = 0; i < h.size(); ++i) {
    const Vec3 & a = h[i], & b = h[(i + 1) % h.size()];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    m = std::min(m, ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) / len);
  }
  return m;
}

/// Knee of a leg in the world from its joint angles and the body pose.
Vec3 kneeWorld(const LocomotionParams & p, int leg, const std::array<double, 12> & q, const GreetFrame & f)
{
  const double q0 = q[leg * 3], q1 = q[leg * 3 + 1];
  const Vec3 t{0.0, legSide(leg) * p.leg.hip, 0.0};
  const Vec3 k{t.x - p.leg.thigh * std::sin(q1), t.y, t.z - p.leg.thigh * std::cos(q1)};
  const Vec3 leg_frame{k.x, k.y * std::cos(q0) - k.z * std::sin(q0), k.y * std::sin(q0) + k.z * std::cos(q0)};
  const Vec3 b{legFront(leg) * p.hip_x + leg_frame.x, legSide(leg) * p.hip_y + leg_frame.y, leg_frame.z};
  const double th = -f.pitch;
  return f.body + Vec3{b.x * std::cos(th) - b.z * std::sin(th), b.y, b.x * std::sin(th) + b.z * std::cos(th)};
}
}  // namespace

TEST(Greet, KneelsWithThePawsUpAndNeverLosesItsSupport)
{
  GreetParams gp;
  GreetSequence g(gp, neutral(), 0.15, 0.105, 0.105);
  g.start();
  double min_margin = 1.0, min_paws_margin = 1.0, min_pitch = 0.0, max_paw = 0.0;
  bool paws_up = false;
  for (int i = 0; i < 10000 && g.active(); ++i) {
    g.update(kDt);
    const GreetFrame & f = g.frame();
    std::vector<Vec3> support;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (!f.down[leg]) {continue;}
      // a flat calf touches 1.2 cm behind its end
      const bool flat = f.kneeling && legFront(leg) < 0;
      support.push_back({flat ? g.rearContactX() : f.feet[leg].x, f.feet[leg].y, 0.0});
    }
    if (f.kneeling) {
      support.push_back({f.knee_x, 0.115, 0.0});
      support.push_back({f.knee_x, -0.115, 0.0});
    }
    const double m = insideHull(support, {f.body.x, f.body.y, 0.0});  // centre of mass over the support
    min_margin = std::min(min_margin, m);
    if (!f.down[0] && !f.down[1]) {
      paws_up = true;
      ASSERT_TRUE(f.kneeling) << i;
      min_paws_margin = std::min(min_paws_margin, m);
    }
    min_pitch = std::min(min_pitch, f.pitch);
    max_paw = std::max({max_paw, f.feet[0].z, f.feet[1].z});
    for (int leg = 0; leg < kNumLegs; ++leg) {EXPECT_GT(f.feet[leg].z, -1e-9);}
  }
  EXPECT_FALSE(g.active());
  EXPECT_GT(min_margin, 0.02);
  EXPECT_TRUE(paws_up);
  EXPECT_GT(min_paws_margin, 0.03);
  EXPECT_LT(min_pitch, -gp.beg_deg * M_PI / 180.0 + 1e-6);  // nose up that far
  EXPECT_GT(max_paw, 0.08);
  // back where it started
  const GreetFrame & f = g.frame();
  EXPECT_NEAR(f.body.z, 0.15, 1e-9);
  EXPECT_NEAR(f.pitch, 0.0, 1e-9);
  for (int leg = 0; leg < kNumLegs; ++leg) {EXPECT_NEAR(f.feet[leg].x, neutral()[leg].x, 1e-9);}
}

TEST(Greet, LocomotionRunsItFromStandingKneesNeverInTheGround)
{
  LocomotionParams p;
  LocomotionController c(p);
  c.request("stand");
  for (int i = 0; i < 150; ++i) {c.update(kDt);}
  ASSERT_EQ(c.mode(), Mode::STAND);
  const auto standing = c.joints();
  // walking: no
  c.setVelocity({0.1, 0.0, 0.0});
  for (int i = 0; i < 50; ++i) {c.update(kDt);}
  EXPECT_FALSE(c.request("greet"));
  c.setVelocity({});
  for (int i = 0; i < 300; ++i) {c.update(kDt);}
  ASSERT_EQ(c.mode(), Mode::STAND);
  ASSERT_TRUE(c.request("greet"));
  EXPECT_EQ(c.mode(), Mode::GREETING);
  // the same sequence alongside for the body pose (the controller's is private)
  const auto n = neutral();
  GreetSequence g(p.greet, n, p.stand_height, p.leg.thigh, p.leg.calf);
  g.start();
  int ticks = 0, kneeling_ticks = 0;
  for (; ticks < 5000 && c.mode() == Mode::GREETING; ++ticks) {
    c.update(kDt);
    g.update(kDt);
    ASSERT_EQ(c.unreachableCount(), 0) << "a foot out of reach at tick " << ticks;
    const GreetFrame & f = g.frame();
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const Vec3 k = kneeWorld(p, leg, c.joints(), f);
      // the knee is a contact of the same radius as the foot: never into the ground
      ASSERT_GT(k.z, p.greet.contact_r - 0.003) << "knee " << leg << " in the ground at tick " << ticks;
      if (f.kneeling && legFront(leg) < 0) {
        EXPECT_NEAR(k.z, p.greet.contact_r, 0.003) << ticks;
        EXPECT_NEAR(k.x, f.knee_x, 0.003) << ticks;
      }
    }
    kneeling_ticks += f.kneeling;
  }
  EXPECT_EQ(c.mode(), Mode::STAND);
  EXPECT_GT(ticks * kDt, 25.0);
  EXPECT_GT(kneeling_ticks * kDt, 10.0);
  for (int j = 0; j < 12; ++j) {EXPECT_NEAR(c.joints()[j], standing[j], 1e-6) << j;}
  // and it walks again
  c.setVelocity({0.1, 0.0, 0.0});
  for (int i = 0; i < 50; ++i) {c.update(kDt);}
  EXPECT_EQ(c.mode(), Mode::WALK);
}
