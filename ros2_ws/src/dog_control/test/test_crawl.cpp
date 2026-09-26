#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>

#include "dog_control/crawl.hpp"

using dog_control::BodyVelocity;
using dog_control::CrawlGait;
using dog_control::CrawlParams;
using dog_control::kNumLegs;
using dog_control::TerrainProfile;
using dog_control::Vec3;

namespace
{
constexpr double kDt = 0.01;

std::array<Vec3, kNumLegs> neutral()
{
  return {{{0.09, 0.115, 0.0}, {0.09, -0.115, 0.0}, {-0.09, 0.115, 0.0}, {-0.09, -0.115, 0.0}}};
}

/// Signed distance of p from the edge a->b of a counter-clockwise triangle (> 0 inside).
double edgeDist(const Vec3 & a, const Vec3 & b, const Vec3 & p)
{
  const double ex = b.x - a.x, ey = b.y - a.y;
  return (ex * (p.y - a.y) - ey * (p.x - a.x)) / std::hypot(ex, ey);
}

/// How far the body centre lies inside the triangle of the three feet down (< 0 = outside).
double stabilityMargin(std::array<Vec3, 3> t)
{
  const double cross = (t[1].x - t[0].x) * (t[2].y - t[0].y) - (t[1].y - t[0].y) * (t[2].x - t[0].x);
  if (cross < 0) {std::swap(t[1], t[2]);}
  const Vec3 o{};
  return std::min({edgeDist(t[0], t[1], o), edgeDist(t[1], t[2], o), edgeDist(t[2], t[0], o)});
}

/// Walks the gait forward over a world h(x) (same on both foot lines) and
/// checks what every test needs: one foot in the air at most, the body over
/// the support, the swing foot above the terrain. Returns the final gait.
struct Walk
{
  double min_margin{1.0};
  double min_swing_clear{1.0};  // swing foot above the terrain under it
  int max_in_air{0};
  std::vector<double> landings_x;  // world x of every touchdown
  double X{0.0};  // distance walked
  double max_lag{0.0};  // furthest a foot fell behind its neutral point
};

Walk walk(CrawlGait & g, const std::function<double(double)> & h, double distance, bool with_terrain = true,
  double smear = 0.0)
{
  Walk w;
  const BodyVelocity go{g.maxSpeed(), 0.0, 0.0};
  std::array<double, kNumLegs> last_z{};
  std::array<bool, kNumLegs> air{};
  const auto base_level = [&](const Vec3 & f) {return h(w.X + f.x + g.shift().x);};
  // walk the distance, then stop and let the last step finish
  for (int tick = 0; tick < 100000 && (w.X < distance || g.stepping()); ++tick) {
    const BodyVelocity cmd = w.X < distance ? go : BodyVelocity{};
    TerrainProfile t;
    t.x0 = -0.3;
    t.dx = 0.01;
    for (int i = 0; i < 100; ++i) {
      // smear > 0: the map's view, mean height over +-smear (edges blurred)
      const double xw = w.X + g.shift().x + t.x0 + i * t.dx;
      double hx = h(xw);
      if (smear > 0.0) {
        hx = 0.0;
        for (int k = -4; k <= 4; ++k) {hx += h(xw + smear * k / 4.0) / 9.0;}
      }
      t.left.push_back(hx);
      t.right.push_back(hx);
    }
    g.update(kDt, cmd, with_terrain ? &t : nullptr);
    w.X += g.twist().vx * kDt;
    const auto f = g.feet();
    int in_air = 0;
    std::array<Vec3, 3> down{};
    int nd = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const double ground = base_level(f[leg]) - h(0.0);
      const bool up = f[leg].z > ground + 0.002;
      if (up) {
        ++in_air;
        w.min_swing_clear = std::min(w.min_swing_clear, f[leg].z - ground);
      } else if (nd < 3) {
        down[nd++] = f[leg];
      }
      if (air[leg] && !up) {w.landings_x.push_back(w.X + f[leg].x + g.shift().x);}
      w.max_lag = std::max(w.max_lag, neutral()[leg].x - (f[leg].x + g.shift().x));
      air[leg] = up;
      last_z[leg] = f[leg].z;
    }
    w.max_in_air = std::max(w.max_in_air, in_air);
    if (in_air == 1 && nd == 3) {w.min_margin = std::min(w.min_margin, stabilityMargin(down));}
  }
  return w;
}
}  // namespace

TEST(Crawl, ThreeFeetDownAndBodyOverTheSupport)
{
  CrawlGait g(CrawlParams{}, neutral());
  // a cycle of four (shift + swing); the shift takes 8 cm at 6 cm/s, 0.3 m/s^2
  const double shift = 0.0794 / 0.06 + 0.06 / 0.3;
  EXPECT_NEAR(g.maxSpeed(), 0.10 / (4 * (shift + 0.65)), 2e-4);
  const Walk w = walk(g, [](double) {return 0.0;}, 0.4, false);
  EXPECT_EQ(w.max_in_air, 1);
  EXPECT_GT(w.min_margin, 0.03);  // centre of mass >= 3 cm inside the triangle
  EXPECT_GT(w.min_swing_clear, 0.0);
}

TEST(Crawl, ClimbsAStepAndAStaircase)
{
  CrawlGait g(CrawlParams{}, neutral());
  // 50 mm step at 0.35 m; then two more every 0.30 m (a staircase)
  auto stairs = [](double x) {return 0.05 * std::clamp(std::floor((x - 0.35) / 0.30) + 1.0, 0.0, 3.0);};
  const Walk w = walk(g, stairs, 1.6);
  EXPECT_EQ(w.max_in_air, 1);
  EXPECT_GT(w.min_margin, 0.025);
  EXPECT_GT(w.min_swing_clear, -0.001);  // never through a riser
  for (const auto & f : g.feet()) {EXPECT_NEAR(f.z, 0.15, 0.002);}  // all feet on the top
  EXPECT_NEAR(g.baseHeight(), 0.15, 0.01);  // and the body climbed with them
  for (double x : w.landings_x) {  // no foot put down on an edge
    for (double edge : {0.35, 0.65, 0.95}) {EXPECT_GT(std::abs(x - edge), 0.008) << x;}
  }
}

TEST(Crawl, ClimbsStairsSeenBlurredWithoutLeavingAFootBehind)
{
  // The elevation map blurs the edges (2 cm cells, mean heights): the foot
  // must go past such an edge, not back from it, or it meets it again on the
  // next cycle and stays behind.
  CrawlGait g(CrawlParams{}, neutral());
  auto stairs = [](double x) {return 0.05 * std::clamp(std::floor((x - 0.35) / 0.30) + 1.0, 0.0, 3.0);};
  const Walk w = walk(g, stairs, 1.6, true, 0.02);
  EXPECT_EQ(w.max_in_air, 1);
  EXPECT_GT(w.min_margin, 0.02);
  // 0.102 on flat ground (the last leg of the first cycle), up to 0.12 more
  // when a foothold is taken short of an edge; stuck behind a riser: 0.19
  EXPECT_LT(w.max_lag, 0.16);
  for (const auto & f : g.feet()) {EXPECT_NEAR(f.z, 0.15, 0.002);}
}

TEST(Crawl, NeverStepsIntoTheShadowBeyondADrop)
{
  // Going down: the map knows the landing and the lower tread further on,
  // not the strip right below the edge (the lidars' shadow).
  CrawlGait g(CrawlParams{}, neutral());
  auto drop = [](double x) {return x < 0.40 ? 0.05 : 0.0;};
  CrawlGait probe(CrawlParams{}, neutral());
  Walk w;
  const BodyVelocity go{g.maxSpeed(), 0.0, 0.0};
  std::array<bool, kNumLegs> air{};
  double X = 0.0;
  for (int tick = 0; tick < 100000 && X < 0.8; ++tick) {
    TerrainProfile t;
    t.x0 = -0.3;
    t.dx = 0.01;
    for (int i = 0; i < 100; ++i) {
      const double xw = X + g.shift().x + t.x0 + i * t.dx;
      const double h = (xw > 0.40 && xw < 0.46) ? std::nan("") : drop(xw);
      t.left.push_back(h);
      t.right.push_back(h);
    }
    t.fillGaps();
    g.update(kDt, go, &t);
    X += g.twist().vx * kDt;
    const auto f = g.feet();
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const bool up = g.swingLeg() == leg;
      if (air[leg] && !up) {
        const double xw = X + f[leg].x + g.shift().x;
        // the shadow is the lower tread (filled from beyond it): off the edge by the window
        EXPECT_FALSE(xw > 0.37 && xw < 0.43) << "foot at the edge: " << xw;
      }
      air[leg] = up;
    }
  }
}

TEST(Crawl, StepsOverABarWithTheFloorBehindItUnseen)
{
  // The lidars do not see the floor right behind a 60 mm bar: the profile has
  // a gap there, filled with the floor's height (fillGaps). In the simulation,
  // unfilled, every foothold went short of the gap and the feet piled up under
  // the body until it tipped over; this model gets across even unfilled (it
  // falls back on the least uneven spot), so it checks the crossing, not that.
  CrawlGait g(CrawlParams{}, neutral());
  auto bar = [](double x) {return (x > 0.40 && x < 0.44) ? 0.06 : 0.0;};
  Walk w;
  const BodyVelocity go{g.maxSpeed(), 0.0, 0.0};
  std::array<bool, kNumLegs> air{};
  double lag = 0.0;
  for (int tick = 0; tick < 200000 && (w.X < 1.0 || g.stepping()); ++tick) {
    TerrainProfile t;
    t.x0 = -0.3;
    t.dx = 0.01;
    for (int i = 0; i < 100; ++i) {
      const double xw = w.X + g.shift().x + t.x0 + i * t.dx;
      const bool shadow = xw >= 0.44 && xw < 0.56 && w.X + g.shift().x < 0.42;  // until the body is over it
      t.left.push_back(shadow ? std::nan("") : bar(xw));
      t.right.push_back(shadow ? std::nan("") : bar(xw));
    }
    t.fillGaps();
    g.update(kDt, w.X < 1.0 ? go : BodyVelocity{}, &t);
    w.X += g.twist().vx * kDt;
    const auto f = g.feet();
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const bool up = g.swingLeg() == leg;
      const double xw = w.X + f[leg].x + g.shift().x;
      if (air[leg] && !up) {EXPECT_FALSE(xw > 0.39 && xw < 0.45) << "foot on the bar at " << xw;}
      air[leg] = up;
      lag = std::max(lag, neutral()[leg].x - (f[leg].x + g.shift().x));
    }
  }
  EXPECT_GT(w.X, 0.99);
  EXPECT_LT(lag, 0.16);  // no pile-up behind the bar
  for (const auto & f : g.feet()) {EXPECT_NEAR(f.z, 0.0, 0.002);}
}

TEST(Crawl, StepsOverAHighBar)
{
  CrawlGait g(CrawlParams{}, neutral());
  auto bar = [](double x) {return (x > 0.40 && x < 0.44) ? 0.06 : 0.0;};  // 60 mm high, 40 mm deep
  const Walk w = walk(g, bar, 1.0);
  EXPECT_EQ(w.max_in_air, 1);
  EXPECT_GT(w.min_swing_clear, -0.001);
  for (double x : w.landings_x) {EXPECT_FALSE(x > 0.39 && x < 0.45) << "foot on the bar at " << x;}
  for (const auto & f : g.feet()) {EXPECT_NEAR(f.z, 0.0, 0.002);}
  EXPECT_NEAR(g.baseHeight(), 0.0, 0.01);
}

TEST(Crawl, StopsWithFeetHomeAndBodyCentred)
{
  CrawlGait g(CrawlParams{}, neutral());
  walk(g, [](double) {return 0.0;}, 0.2, false);
  for (int i = 0; i < 2000 && g.stepping(); ++i) {g.update(kDt, BodyVelocity{});}
  EXPECT_FALSE(g.stepping());
  for (int i = 0; i < 300; ++i) {g.update(kDt, BodyVelocity{});}  // body back over the middle
  const auto f = g.feet();
  const auto n = neutral();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_NEAR(f[leg].x, n[leg].x, 0.011);
    EXPECT_NEAR(f[leg].y, n[leg].y, 0.011);
  }
  EXPECT_NEAR(g.shift().x, 0.0, 1e-9);
}

TEST(Crawl, TerrainProfileGapsTakeTheLowerSide)
{
  const double n = std::nan("");
  TerrainProfile t;
  t.x0 = 0.0;
  t.dx = 0.1;
  t.left = {n, 0.0, 0.06, n, n, 0.0, n};   // bar, its shadow, the floor
  t.right = {0.05, n, n, 0.0, 0.0, n, n};  // a drop and its shadow
  t.fillGaps();
  EXPECT_TRUE(std::isnan(t.left[0]));
  EXPECT_DOUBLE_EQ(t.left[3], 0.0);
  EXPECT_DOUBLE_EQ(t.left[4], 0.0);
  EXPECT_TRUE(std::isnan(t.left[6]));
  EXPECT_DOUBLE_EQ(t.right[1], 0.0);  // the lower tread starts at the edge
  EXPECT_DOUBLE_EQ(t.right[2], 0.0);
  EXPECT_TRUE(std::isnan(t.right[5]));
}

TEST(Crawl, TerrainProfileLookup)
{
  TerrainProfile t;
  t.x0 = 0.0;
  t.dx = 0.1;
  t.left = {0.0, 0.1, std::nan(""), 0.3};
  t.right = {0.0, 0.0, 0.0, 0.0};
  EXPECT_NEAR(t.height(0.1, 0.05), 0.05, 1e-12);
  EXPECT_NEAR(t.height(0.1, 0.15), 0.1, 1e-12);  // one side unknown: the known one
  EXPECT_TRUE(std::isnan(t.height(0.1, 0.5)));
  EXPECT_NEAR(t.highest(0.1, 0.0, 0.3), 0.3, 1e-12);
  EXPECT_NEAR(t.highest(-0.1, 0.0, 0.3), 0.0, 1e-12);
}
