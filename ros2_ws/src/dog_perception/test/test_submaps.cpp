#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "dog_perception/submaps.hpp"

using dog_perception::GraphEdge;
using dog_perception::P2;
using dog_perception::Pose2;
using dog_perception::SubmapMap;
using dog_perception::SubmapParams;
using dog_perception::wrapAngle;

namespace
{
struct Seg {P2 a, b;};

void box(std::vector<Seg> & s, double x0, double y0, double x1, double y1)
{
  s.push_back({{x0, y0}, {x1, y0}});
  s.push_back({{x1, y0}, {x1, y1}});
  s.push_back({{x1, y1}, {x0, y1}});
  s.push_back({{x0, y1}, {x0, y0}});
}

// a ring corridor 1.5 m wide round a 7 x 4 m core (as the sim's house world),
// a few pieces of furniture so that the corners are not all alike, and long
// bare stretches where only dead reckoning knows how far it has gone
std::vector<Seg> ring()
{
  std::vector<Seg> s;
  box(s, -5.0, -3.5, 5.0, 3.5);
  box(s, -3.5, -2.0, 3.5, 2.0);
  box(s, -1.0, 3.1, 0.2, 3.5);     // cabinet on the north wall
  box(s, 4.6, -1.0, 5.0, 0.2);     // shelf on the east wall
  box(s, -5.0, -3.5, -4.4, -2.9);  // chest in the south-west corner
  box(s, 1.0, -2.3, 1.6, -2.0);    // bench against the core, south side
  return s;
}

// what the lidars would see from `pose`: rays every `step` rad, nearest hit
std::vector<P2> view(const std::vector<Seg> & segs, const Pose2 & pose, double range, int seed, double step = 0.01)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<P2> out;
  for (double a = -M_PI; a < M_PI; a += step) {
    const double dx = std::cos(pose.yaw + a), dy = std::sin(pose.yaw + a);
    double best = range;
    for (const auto & s : segs) {
      const double ex = s.b.x - s.a.x, ey = s.b.y - s.a.y;
      const double den = dx * ey - dy * ex;
      if (std::abs(den) < 1e-12) {continue;}
      const double wx = s.a.x - pose.x, wy = s.a.y - pose.y;
      const double t = (wx * ey - wy * ex) / den, u = (wx * dy - wy * dx) / den;
      if (t > 0.05 && u >= 0.0 && u <= 1.0) {best = std::min(best, t);}
    }
    if (best >= range) {continue;}
    const double r = best + noise(rng);
    out.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return out;
}

// the route round the core, a step every ~0.1 m
std::vector<Pose2> route(int laps_plus_quarter = 1)
{
  const std::vector<P2> corners = {{-4.25, -2.75}, {4.25, -2.75}, {4.25, 2.75}, {-4.25, 2.75}};
  std::vector<Pose2> out;
  const int legs = 4 * laps_plus_quarter + 1;  // back past the start
  for (int l = 0; l < legs; ++l) {
    const P2 a = corners[l % 4], b = corners[(l + 1) % 4];
    const double L = std::hypot(b.x - a.x, b.y - a.y), yaw = std::atan2(b.y - a.y, b.x - a.x);
    for (double t = 0.0; t < L; t += 0.1) {out.push_back({a.x + (b.x - a.x) * t / L, a.y + (b.y - a.y) * t / L, yaw});}
  }
  return out;
}

struct MapRun
{
  SubmapMap map;
  Pose2 est, truth;
};

// map the ring walking round it, dead reckoning with 8 % scale error and a
// 1.5 deg/m heading drift, every scan (3 m range) matched to the merged map
// (as the node does)
MapRun mapRing(bool loop_closure)
{
  SubmapParams p;
  p.loop_closure = loop_closure;
  MapRun run{SubmapMap(p), {}, {}};
  const auto segs = ring();
  const auto path = route();
  Pose2 est = path[0];
  double walked = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    const Pose2 & truth = path[i];
    if (i > 0) {
      const Pose2 d = path[i - 1].inverse().compose(truth);
      const double len = std::hypot(d.x, d.y);
      walked += len;
      const Pose2 dr{d.x * 1.08, d.y * 1.08, d.yaw + 0.026 * len};
      est = est.compose(dr);
    }
    const auto cloud = view(segs, truth, 3.0, static_cast<int>(i), 0.02);
    if (!run.map.empty() && run.map.merged().fieldValid()) {
      dog_perception::MatchParams mp;
      mp.prior_xy = 0.05;
      const auto r = dog_perception::match(run.map.merged(), cloud, est, mp);
      if (r.ok && r.inlier_fraction > 0.5) {est = r.pose;}
    }
    std::vector<P2> pts;
    for (const auto & q : cloud) {pts.push_back(est.apply(q));}
    const Pose2 corr = run.map.insert(pts, est, walked);
    est = corr.compose(est);
    run.map.refresh();
    run.truth = truth;
  }
  run.est = est;
  return run;
}

// how far the map's walls are from the true ones (mean over the merged walls
// of the distance to the nearest true wall segment)
double wallError(const SubmapMap & m)
{
  const auto segs = ring();
  double sum = 0.0;
  const auto walls = m.merged().walls();
  for (const auto & w : walls) {
    double best = 1e9;
    for (const auto & s : segs) {
      const double ex = s.b.x - s.a.x, ey = s.b.y - s.a.y, L2 = ex * ex + ey * ey;
      const double t = std::clamp(((w.x - s.a.x) * ex + (w.y - s.a.y) * ey) / L2, 0.0, 1.0);
      best = std::min(best, std::hypot(w.x - s.a.x - t * ex, w.y - s.a.y - t * ey));
    }
    sum += best;
  }
  return walls.empty() ? 1e9 : sum / walls.size();
}
}  // namespace

TEST(Submaps, PoseGraphSpreadsTheLoopError)
{
  // a square walked with 3 deg too much turn at each corner; the loop edge
  // says the last node is back at the first
  std::vector<Pose2> nodes;
  std::vector<GraphEdge> edges;
  Pose2 p;
  nodes.push_back(p);
  const Pose2 step{2.0, 0.0, M_PI / 2 + 0.05};
  for (int k = 1; k <= 4; ++k) {
    p = p.compose(step);
    nodes.push_back(p);
    GraphEdge e;
    e.a = k - 1;
    e.b = k;
    e.z = step;
    e.sigma_xy = 0.05;
    e.sigma_yaw = 0.05;
    edges.push_back(e);
  }
  GraphEdge loop;
  loop.a = 0;
  loop.b = 4;
  loop.z = Pose2{0.0, 0.0, 0.0};
  loop.sigma_xy = 0.01;
  loop.sigma_yaw = 0.01;
  loop.loop = true;
  edges.push_back(loop);
  const double before = std::hypot(nodes[4].x, nodes[4].y);
  ASSERT_GT(before, 0.1);
  dog_perception::optimizePoseGraph(nodes, edges);
  EXPECT_LT(std::hypot(nodes[4].x, nodes[4].y), 0.03);
  EXPECT_NEAR(wrapAngle(nodes[4].yaw), 0.0, 0.02);
  EXPECT_NEAR(nodes[0].x, 0.0, 1e-12);  // node 0 stays
}

TEST(Submaps, ScanContextKnowsAPlaceTurnedAndTellsPlacesApart)
{
  const auto segs = ring();
  dog_perception::ScanContextParams sc;
  const auto here = dog_perception::scanContext(view(segs, {-4.25, 2.75, 0.0}, 4.0, 1), sc);
  const auto turned = dog_perception::scanContext(view(segs, {-4.2, 2.7, 1.0}, 4.0, 2), sc);
  const auto other = dog_perception::scanContext(view(segs, {0.0, -2.75, 0.0}, 4.0, 3), sc);  // corridor
  double yaw = 0.0;
  const double same = dog_perception::scanContextDistance(here, turned, sc, &yaw);
  EXPECT_LT(same, 0.3);
  EXPECT_NEAR(wrapAngle(yaw - 1.0), 0.0, 0.12);  // turned's frame onto here's: +1 rad
  EXPECT_GT(dog_perception::scanContextDistance(here, other, sc), same + 0.1);
}

TEST(Submaps, LoopClosureKeepsTheRingStraight)
{
  const MapRun open = mapRing(false);
  const MapRun closed = mapRing(true);
  ASSERT_FALSE(closed.map.loops().empty()) << "no loop closed";
  const double e_open = std::hypot(open.est.x - open.truth.x, open.est.y - open.truth.y);
  const double e_closed = std::hypot(closed.est.x - closed.truth.x, closed.est.y - closed.truth.y);
  std::printf("submaps %zu, loops %zu; final pose error open %.3f m, closed %.3f m; "
    "wall error open %.3f m, closed %.3f m\n", closed.map.submaps().size(), closed.map.loops().size(),
    e_open, e_closed, wallError(open.map), wallError(closed.map));
  // a scale error the same all round (bare corridors) is no loop error: the
  // closed map is consistent, but a little stretched - the walls tell more
  // with the matching kept off the directions the walls do not fix, the ring
  // is mapped well even open; closing the loops must keep it so
  EXPECT_LT(e_closed, 0.15);
  EXPECT_LT(wallError(closed.map), 0.05);
  EXPECT_LT(wallError(closed.map), 1.2 * wallError(open.map) + 0.005);
}

TEST(Submaps, RelocalizesByPlacesAndSurvivesSaveAndLoad)
{
  const MapRun run = mapRing(true);
  const auto segs = ring();
  const std::string path = testing::TempDir() + "/ring_map";
  ASSERT_TRUE(run.map.save(path));
  SubmapMap loaded;
  ASSERT_TRUE(loaded.load(path));
  EXPECT_EQ(loaded.submaps().size(), run.map.submaps().size());
  for (const Pose2 & truth : {Pose2{4.2, -0.8, 2.0}, Pose2{-4.2, -2.3, 0.5}, Pose2{1.3, -2.7, 3.0}}) {
    const auto cloud = view(segs, truth, 4.0, 7);
    for (const SubmapMap * m : std::vector<const SubmapMap *>{&run.map, &loaded}) {
      const auto r = m->relocalize(cloud);
      ASSERT_TRUE(r.ok) << truth.x << " " << truth.y << " score " << r.score << " second " << r.second
                        << " best at " << r.best.pose.x << " " << r.best.pose.y << " " << r.best.pose.yaw;
      // the map is a few cm stretched along the bare corridors: where the map
      // puts the robot, not quite where it is
      EXPECT_NEAR(r.best.pose.x, truth.x, 0.15);
      EXPECT_NEAR(r.best.pose.y, truth.y, 0.15);
      EXPECT_NEAR(wrapAngle(r.best.pose.yaw - truth.yaw), 0.0, 0.05);
    }
  }
  std::remove((path + ".graph").c_str());
}

TEST(Submaps, ABareCornerIsNoAnswer)
{
  // the north-east corner sees no furniture: it looks like the other bare
  // corners turned; in the north corridor by the cabinet a shift along the
  // corridor fits nearly as well. Better "not sure" than a wrong place.
  const MapRun run = mapRing(true);
  for (const Pose2 & truth : {Pose2{4.1, 2.6, 2.0}, Pose2{-0.4, 2.8, -1.4}}) {
    const auto r = run.map.relocalize(view(ring(), truth, 4.0, 7));
    if (r.ok) {
      EXPECT_NEAR(r.best.pose.x, truth.x, 0.2);
      EXPECT_NEAR(r.best.pose.y, truth.y, 0.2);
    } else {
      EXPECT_GT(r.second, 0.9 * r.score);
    }
  }
}
