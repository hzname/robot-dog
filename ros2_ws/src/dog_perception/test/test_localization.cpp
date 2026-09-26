#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "dog_perception/localization.hpp"

using dog_perception::GlobalParams;
using dog_perception::MatchParams;
using dog_perception::P2;
using dog_perception::Pose2;
using dog_perception::WallGrid;
using dog_perception::wrapAngle;

namespace
{
struct Seg {P2 a, b;};

// 5 x 4 m room with a cabinet in one corner and a sofa along a wall: not symmetric
std::vector<Seg> room(bool furniture = true)
{
  std::vector<Seg> s = {
    {{-2.5, -2.0}, {2.5, -2.0}}, {{2.5, -2.0}, {2.5, 2.0}}, {{2.5, 2.0}, {-2.5, 2.0}}, {{-2.5, 2.0}, {-2.5, -2.0}}};
  if (furniture) {
    // cabinet 0.4 x 1.0 at the +x wall, sofa 1.8 x 0.8 at the -y wall
    s.push_back({{2.1, 0.8}, {2.1, 1.8}});
    s.push_back({{2.1, 0.8}, {2.5, 0.8}});
    s.push_back({{2.1, 1.8}, {2.5, 1.8}});
    s.push_back({{-1.5, -1.2}, {0.3, -1.2}});
    s.push_back({{-1.5, -1.2}, {-1.5, -2.0}});
    s.push_back({{0.3, -1.2}, {0.3, -2.0}});
  }
  return s;
}

std::vector<P2> sample(const std::vector<Seg> & segs, double step)
{
  std::vector<P2> out;
  for (const auto & s : segs) {
    const double L = std::hypot(s.b.x - s.a.x, s.b.y - s.a.y);
    for (double t = 0.0; t <= L; t += step) {
      out.push_back({s.a.x + (s.b.x - s.a.x) * t / L, s.a.y + (s.b.y - s.a.y) * t / L});
    }
  }
  return out;
}

WallGrid mapOf(const std::vector<Seg> & segs)
{
  WallGrid g;
  const auto pts = sample(segs, 0.01);
  g.insert(pts);
  g.insert(pts);  // min_hits 2
  g.updateField();
  return g;
}

// what a robot at `pose` sees within `range`, in its own frame, with noise
std::vector<P2> view(const std::vector<Seg> & segs, const Pose2 & pose, double range, int seed = 1)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.01);
  const Pose2 inv = pose.inverse();
  std::vector<P2> out;
  for (const auto & p : sample(segs, 0.03)) {
    if (std::hypot(p.x - pose.x, p.y - pose.y) > range) {continue;}
    P2 q = inv.apply(p);
    q.x += noise(rng);
    q.y += noise(rng);
    out.push_back(q);
  }
  return out;
}
}  // namespace

TEST(Localization, PoseAlgebra)
{
  const Pose2 a{1.0, 2.0, 0.5}, b{-0.3, 0.7, -1.2};
  const Pose2 c = a.compose(b).compose(b.inverse());
  EXPECT_NEAR(c.x, a.x, 1e-12);
  EXPECT_NEAR(c.y, a.y, 1e-12);
  EXPECT_NEAR(c.yaw, a.yaw, 1e-12);
  const P2 p = a.inverse().apply(a.apply({0.4, -0.9}));
  EXPECT_NEAR(p.x, 0.4, 1e-12);
  EXPECT_NEAR(p.y, -0.9, 1e-12);
  EXPECT_NEAR(std::abs(wrapAngle(3.0 * M_PI)), M_PI, 1e-9);
  EXPECT_NEAR(wrapAngle(0.5 + 4.0 * M_PI), 0.5, 1e-9);
}

TEST(Localization, DistanceFieldToAWall)
{
  WallGrid g(0.05, 1, 0.5);
  g.insert(sample({{{0.0, -1.0}, {0.0, 1.0}}}, 0.01));
  g.updateField();
  double gx = 0.0, gy = 0.0;
  // measured to where the wall is (x = 0), not to its cells' centres
  EXPECT_NEAR(g.distance(0.2, 0.0, &gx, &gy), 0.2, 0.005);
  EXPECT_NEAR(gx, 1.0, 0.05);
  EXPECT_NEAR(gy, 0.0, 0.1);
  EXPECT_NEAR(g.distance(-0.3, 0.3), 0.3, 0.005);
  EXPECT_NEAR(g.distance(0.0, 0.5), 0.0, 0.03);
  EXPECT_DOUBLE_EQ(g.distance(5.0, 0.0), 0.5);  // capped, off the grid
}

TEST(Localization, SaveAndLoadKeepTheWalls)
{
  const WallGrid g = mapOf(room());
  const std::string path = testing::TempDir() + "/loc_map";
  ASSERT_TRUE(g.save(path));
  WallGrid h;
  ASSERT_TRUE(h.load(path));
  EXPECT_EQ(h.width(), g.width());
  EXPECT_EQ(h.height(), g.height());
  EXPECT_NEAR(h.origin().x, g.origin().x, 1e-9);
  EXPECT_EQ(h.occupiedCount(), g.occupiedCount());
  for (double x = -2.4; x < 2.4; x += 0.37) {
    EXPECT_NEAR(h.distance(x, 0.3), g.distance(x, 0.3), 1e-5);
  }
  std::remove((path + ".pgm").c_str());
  std::remove((path + ".yaml").c_str());
  std::remove((path + ".walls").c_str());
}

TEST(Localization, MatchPullsAnOffsetCloudOntoTheWalls)
{
  const auto segs = room();
  const WallGrid g = mapOf(segs);
  const Pose2 truth{0.4, 0.3, 0.6};
  const auto cloud = view(segs, truth, 4.0);
  const Pose2 guess{truth.x + 0.15, truth.y - 0.12, truth.yaw + 0.08};
  const auto r = dog_perception::match(g, cloud, guess);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.pose.x, truth.x, 0.01);
  EXPECT_NEAR(r.pose.y, truth.y, 0.01);
  EXPECT_NEAR(wrapAngle(r.pose.yaw - truth.yaw), 0.0, 0.005);
  EXPECT_GT(r.inlier_fraction, 0.95);
}

TEST(Localization, CorridorKeepsThePriorAlongIt)
{
  // two long parallel walls: nothing fixes x, the prior (the guess) must
  const std::vector<Seg> segs = {{{-10.0, -0.6}, {10.0, -0.6}}, {{-10.0, 0.6}, {10.0, 0.6}}};
  const WallGrid g = mapOf(segs);
  const Pose2 truth{0.0, 0.1, 0.0};
  const auto cloud = view(segs, truth, 3.0);
  const Pose2 guess{0.3, 0.0, 0.03};
  const auto r = dog_perception::match(g, cloud, guess);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.pose.y, truth.y, 0.01);                    // across: from the walls
  EXPECT_NEAR(wrapAngle(r.pose.yaw), 0.0, 0.005);
  EXPECT_NEAR(r.pose.x, guess.x, 0.02);                    // along: stays where it was
}

TEST(Localization, GlobalSearchFindsTheRobotInTheRoom)
{
  const auto segs = room();
  const WallGrid g = mapOf(segs);
  for (const Pose2 & truth : {Pose2{0.4, 0.3, 0.6}, Pose2{-1.6, 1.2, -2.5}, Pose2{1.3, -0.6, 3.0}}) {
    const auto res = dog_perception::globalSearch(g, view(segs, truth, 6.0));
    ASSERT_TRUE(res.ok) << truth.x << " " << truth.y << " score " << res.score << " second " << res.second;
    EXPECT_NEAR(res.best.pose.x, truth.x, 0.03);
    EXPECT_NEAR(res.best.pose.y, truth.y, 0.03);
    EXPECT_NEAR(wrapAngle(res.best.pose.yaw - truth.yaw), 0.0, 0.02);
  }
}

TEST(Localization, EmptyRectangleIsAmbiguous)
{
  // a bare rectangle looks the same turned 180 deg: the search must say so
  const auto segs = room(false);
  const WallGrid g = mapOf(segs);
  const auto res = dog_perception::globalSearch(g, view(segs, {0.7, 0.4, 0.3}, 6.0));
  EXPECT_FALSE(res.ok);
  EXPECT_GT(res.second, 0.9 * res.score);
}

TEST(Localization, MappingWhileMovingStaysConsistent)
{
  // build the map from views along a path, each placed by matching to what
  // was mapped before (as the node does): the walls come out where they are
  const auto segs = room();
  WallGrid g;
  Pose2 truth{-1.5, 0.0, 0.0};
  auto first = view(segs, truth, 4.0, 0);
  std::vector<P2> w;
  for (const auto & p : first) {w.push_back(truth.apply(p));}
  g.insert(w);
  g.insert(w);
  g.updateField();
  Pose2 est = truth;
  for (int k = 1; k <= 30; ++k) {
    truth = {-1.5 + 0.1 * k, 0.02 * k, 0.03 * k};
    const auto cloud = view(segs, truth, 4.0, k);
    // dead reckoning with 10 % scale error on the step
    const Pose2 guess{est.x + 0.11, est.y + 0.022, est.yaw + 0.033};
    const auto r = dog_perception::match(g, cloud, guess);
    ASSERT_TRUE(r.ok);
    est = r.pose;
    std::vector<P2> pw;
    for (const auto & p : cloud) {pw.push_back(est.apply(p));}
    g.insert(pw);
    g.updateField();
  }
  EXPECT_NEAR(est.x, truth.x, 0.03);
  EXPECT_NEAR(est.y, truth.y, 0.03);
  EXPECT_NEAR(wrapAngle(est.yaw - truth.yaw), 0.0, 0.02);
}

TEST(Localization, MappingABareCorridorDoesNotDragTheRobotBack)
{
  // a corridor 1.5 m wide with an end wall behind the start; the robot sees
  // 5 m behind but only 1 m ahead (the tilted lidars look down ahead), walks
  // 8 m with perfect dead reckoning while mapping. Wall points ahead of the
  // mapped part used to pull it back to the map's edge: 1.4 m short in 8 m.
  const std::vector<Seg> segs = {{{-1.0, -0.75}, {12.0, -0.75}}, {{-1.0, 0.75}, {12.0, 0.75}},
    {{-1.0, -0.75}, {-1.0, 0.75}}};
  WallGrid g;
  Pose2 est;
  for (int i = 0; i <= 80; ++i) {
    const Pose2 truth{0.1 * i, 0.0, 0.0};
    if (i > 0) {est = est.compose(Pose2{0.1, 0.0, 0.0});}
    std::vector<P2> cloud;
    for (const auto & q : view(segs, truth, 5.0, i)) {
      if (q.x < 1.0) {cloud.push_back(q);}
    }
    if (g.fieldValid()) {
      const auto r = dog_perception::match(g, cloud, est);
      if (r.ok) {est = r.pose;}
    }
    std::vector<P2> w;
    for (const auto & q : cloud) {w.push_back(est.apply(q));}
    g.insert(w);
    g.updateField();
  }
  EXPECT_NEAR(est.x, 8.0, 0.25);
}
