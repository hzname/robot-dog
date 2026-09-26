// Same cases as test_core.py (the numpy twin), for the C++ core.
#include <gtest/gtest.h>

#include <algorithm>

#include <cmath>
#include <functional>
#include <random>

#include "dog_perception/core.hpp"

using namespace dog_perception;

namespace
{
constexpr double H = 0.150;  // body centre (hip axes) above the floor = stand_height
const Plane kFloor{{0.0, 0.0, 1.0}, -H};
constexpr double kDeg = M_PI / 180.0;

SensorParams sensors()
{
  SensorParams s;
  s.gs2 = true;
  return s;
}

std::vector<V3> scanFloor(const SensorMount & m, double a0, double a1, int n,
  const std::function<double(double, double)> & floor_z, double rmin, double rmax)
{
  std::vector<float> r;
  const double da = (a1 - a0) / (n - 1);
  for (int i = 0; i < n; ++i) {
    const double a = a0 + da * i;
    const V3 u = mul(m.R, V3{std::cos(a), std::sin(a), 0.0});
    double hit = kInf;
    for (double t = 0.02; t < rmax; t += 0.0005) {
      const V3 q = m.p + u * t;
      if (q.z <= floor_z(q.x, q.y)) {
        hit = t;
        break;
      }
    }
    r.push_back(static_cast<float>(hit));
  }
  return scanToBody(m, r, a0, da, rmin, rmax);
}
}  // namespace

TEST(Core, TofBeamsHitWhereDesigned)
{
  const auto m = mountsFromParams(sensors());
  const auto & fl = m.at("tof_fl");
  const double t = rayToPlane(fl.p, fl.beam(), kFloor);
  EXPECT_NEAR(t, 0.15 / std::sin(40 * kDeg), 1e-9);
  const V3 spot = fl.p + fl.beam() * t;
  EXPECT_NEAR(spot.y, 0.115, 0.005);
  EXPECT_NEAR(spot.x, 0.28, 0.01);
  const auto & rc = m.at("tof_rc");
  EXPECT_LT((rc.p + rc.beam() * rayToPlane(rc.p, rc.beam(), kFloor)).x, -0.2);
}

TEST(Core, XLidarsDrawAnXAndTheirPlaneIsExact)
{
  const auto m = mountsFromParams(sensors());
  std::vector<V3> both;
  for (const char * name : {"lidar_left", "lidar_right"}) {
    const auto & s = m.at(name);
    std::vector<float> r;
    for (int i = 0; i < 721; ++i) {
      const double a = -M_PI + i * (2 * M_PI / 720);
      r.push_back(static_cast<float>(rayToPlane(s.p, mul(s.R, V3{std::cos(a), std::sin(a), 0.0}), kFloor)));
    }
    const auto pts = scanToBody(s, r, -M_PI, 2 * M_PI / 720);
    for (const auto & p : pts) {EXPECT_NEAR(p.z, -H, 1e-5);}  // float ranges
    both.insert(both.end(), pts.begin(), pts.end());
  }
  const auto fit = robustPlane(both);
  ASSERT_TRUE(fit);
  EXPECT_LT(fit->rms, 1e-5);
  EXPECT_NEAR(fit->plane.c, -H, 1e-5);
  double roll, pitch;
  rollPitchOfNormal(fit->plane.n, roll, pitch);
  EXPECT_NEAR(roll, 0.0, 1e-5);
  EXPECT_NEAR(pitch, 0.0, 1e-5);
}

TEST(Core, PlaneAttitudeSignsAndTiltedFit)
{
  const double th = 10 * kDeg;
  double roll, pitch;
  rollPitchOfNormal(mul(transpose(rotRpy(0, th, 0)), V3{0, 0, 1}), roll, pitch);
  EXPECT_NEAR(pitch, th, 1e-9);
  EXPECT_NEAR(roll, 0.0, 1e-9);
  rollPitchOfNormal(mul(transpose(rotRpy(th, 0, 0)), V3{0, 0, 1}), roll, pitch);
  EXPECT_NEAR(roll, th, 1e-9);
  // Jacobi eigenvector: a plane tilted in both axes is recovered exactly
  const V3 n = mul(rotRpy(0.2, -0.3, 0.7), V3{0, 0, 1});
  std::vector<V3> pts;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      const double x = 0.1 * i - 0.5, y = 0.1 * j - 0.5;
      pts.push_back({x, y, (0.3 - n.x * x - n.y * y) / n.z});
    }
  }
  const auto p = fitPlane(pts);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->n.x, n.x, 1e-9);
  EXPECT_NEAR(p->n.y, n.y, 1e-9);
  EXPECT_NEAR(p->c, 0.3, 1e-9);
}

TEST(Core, RobustPlaneIgnoresAStone)
{
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> u(-0.5, 0.5);
  std::normal_distribution<double> noise(0.0, 0.003);
  std::vector<V3> pts;
  for (int i = 0; i < 400; ++i) {pts.push_back({u(rng), u(rng), -H + noise(rng) + (i < 30 ? 0.04 : 0.0)});}
  const auto fit = robustPlane(pts);
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->plane.c, -H, 0.002);
  EXPECT_LT(fit->inlier_fraction, 371.0 / 400.0);
}

TEST(Core, FeetPlaneFromStanceAndImuCarry)
{
  Geometry g;
  const double L2 = g.thigh;
  const double calf = -std::acos((0.15 * 0.15 - 2 * L2 * L2) / (2 * L2 * L2));
  const double thigh = std::atan2(-L2 * std::sin(calf), L2 + L2 * std::cos(calf));
  std::array<double, 12> q{};
  for (int leg = 0; leg < 4; ++leg) {
    q[leg * 3 + 1] = thigh;
    q[leg * 3 + 2] = calf;
  }
  auto feet = feetBody(g, q);
  for (const auto & f : feet) {EXPECT_NEAR(f.z, -0.150, 1e-6);}
  FeetPlane fp;
  EXPECT_TRUE(fp.update(feet, identity()));
  feet[0].z += 0.02;  // a swing foot: not a four-leg support phase, plane kept
  EXPECT_FALSE(fp.update(feet, identity()));
  const auto p = fp.current(rotRpy(0, 5 * kDeg, 0));  // body pitched 5 deg since
  ASSERT_TRUE(p);
  double roll, pitch;
  rollPitchOfNormal(p->n, roll, pitch);
  EXPECT_NEAR(pitch, 5 * kDeg, 1e-9);
  // the legs themselves: points on a calf are the leg, the ground ahead is not
  const auto legs = legsBody(g, q);
  for (int leg = 1; leg < 4; ++leg) {EXPECT_NEAR(legs[leg][2].z, feet[leg].z, 1e-9);}  // (feet[0] was raised above)
  const V3 mid = (legs[0][1] + legs[0][2]) * 0.5;
  EXPECT_TRUE(onLeg(legs, mid + V3{0.02, 0.0, 0.0}, 0.04));
  EXPECT_FALSE(onLeg(legs, legs[0][2] + V3{0.10, 0.0, 0.0}, 0.04));
  // a front leg swung forward and up: its calf reaches past the footprint
  // filter (|x| < 0.22), still the leg
  q[1] = -0.9;
  q[2] = -0.6;
  const auto up = legsBody(g, q);
  const V3 calf_mid = (up[0][1] + up[0][2]) * 0.5;
  EXPECT_GT(up[0][2].x, 0.22);
  EXPECT_TRUE(onLeg(up, up[0][2] + V3{0.0, 0.0, 0.02}, 0.04));
  EXPECT_TRUE(onLeg(up, calf_mid, 0.04));
}

TEST(Core, LidarHazardsAndElevationMap)
{
  std::vector<V3> pts;
  for (int i = 0; i < 3; ++i) {pts.push_back({0.5, 0.12, -H + 0.02});}
  for (int i = 0; i < 3; ++i) {pts.push_back({0.6, -0.12, -H - 0.03});}
  for (int i = 0; i < 5; ++i) {pts.push_back({0.5, 0.0, -H});}
  const auto hz = lidarHazards(pts, kFloor);
  bool left_up = false, right_down = false;
  for (const auto & h : hz) {
    left_up |= h.corridor == "left" && h.kind == "up";
    right_down |= h.corridor == "right" && h.kind == "down";
    EXPECT_NE(h.corridor, "centre");
  }
  EXPECT_TRUE(left_up && right_down);
  ElevationMap em(1.0, 0.02);
  em.recenter(0.0, 0.0);
  em.insert({{0.101, 0.101, 0.05}, {0.101, 0.101, 0.07}});
  auto mean = em.mean();
  int k = -1;
  for (size_t i = 0; i < mean.size(); ++i) {
    if (std::isfinite(mean[i])) {k = static_cast<int>(i);}
  }
  ASSERT_GE(k, 0);
  EXPECT_NEAR(mean[k], 0.06, 1e-6);
  em.recenter(0.3, 0.0);  // move 0.3 m: the cell moves with the world
  mean = em.mean();
  for (size_t i = 0; i < mean.size(); ++i) {
    if (std::isfinite(mean[i])) {k = static_cast<int>(i);}
  }
  const double cx = em.originX() + (k / em.size() + 0.5) * em.resolution();
  const double cy = em.originY() + (k % em.size() + 0.5) * em.resolution();
  EXPECT_NEAR(cx, 0.11, 0.011);
  EXPECT_NEAR(cy, 0.11, 0.011);
}

TEST(Core, LidarEdgeHeightTellsAWallFromARamp)
{
  std::vector<V3> ramp, wall;
  for (int i = 0; i < 300; ++i) {
    const double x = 0.25 + 0.75 * i / 299.0;
    ramp.push_back({x, 0.12, -H + std::max(0.0, x - 0.5) * std::tan(10 * kDeg)});
    wall.push_back({x, 0.12, -H + (x > 0.5 ? 0.10 : 0.0)});
  }
  auto hz = lidarHazards(ramp, kFloor);
  ASSERT_EQ(hz.size(), 1u);
  EXPECT_GT(hz[0].x, 0.55);
  EXPECT_LT(hz[0].x, 0.6);
  EXPECT_GT(hz[0].h, 0.04);     // high above the plane ...
  EXPECT_LT(hz[0].jump, 0.015);  // ... but no edge
  hz = lidarHazards(wall, kFloor);
  ASSERT_EQ(hz.size(), 1u);
  EXPECT_NEAR(hz[0].x, 0.5, 0.01);
  EXPECT_GT(hz[0].jump, 0.09);
}

TEST(Core, TofDetectorCalibratesAndDebounces)
{
  const auto m = mountsFromParams(sensors()).at("tof_fl");
  TofDetector det(m, 0.015, 2, 0.0, 5);
  const double exp = rayToPlane(m.p, m.beam(), kFloor);
  for (int i = 0; i < 5; ++i) {det.calibrate(exp + 0.004, kFloor);}  // reads 4 mm long
  EXPECT_TRUE(det.calibrated());
  EXPECT_EQ(det.check(exp + 0.004, kFloor).verdict, "");
  EXPECT_EQ(det.check(exp - 0.03, kFloor).verdict, "");    // first reading: not yet
  EXPECT_EQ(det.check(exp - 0.03, kFloor).verdict, "up");  // second: stone
  EXPECT_EQ(det.check(kInf, kFloor).verdict, "");
  EXPECT_EQ(det.check(kInf, kFloor).verdict, "down");      // no floor: edge or hole
}

TEST(Core, Gs2LineAndHazards)
{
  const auto m = mountsFromParams(sensors()).at("gs2");
  const double half = 50 * kDeg;
  auto scan = [&](const std::function<double(double, double)> & f) {
      return scanFloor(m, -half, half, 160, f, 0.025, 0.30);
    };
  const auto flat = scan([](double, double) {return -H;});
  double xs = 0, ymin = 1, ymax = -1;
  for (const auto & p : flat) {
    EXPECT_NEAR(p.z, -H, 1e-3);
    xs += p.x;
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  EXPECT_NEAR(xs / flat.size(), 0.294, 0.01);
  EXPECT_LT(ymin, -0.17);
  EXPECT_GT(ymax, 0.17);
  EXPECT_TRUE(gs2Hazards(flat, kFloor, true).empty());
  // a 20 mm stone on the left foot line: seen by the line fit, reference-free
  const auto stone = scan([](double x, double y) {return -H + (y > 0.09 && y < 0.15 && x < 0.35 ? 0.02 : 0.0);});
  auto hz = gs2Hazards(stone, std::nullopt, false);
  ASSERT_EQ(hz.size(), 1u);
  EXPECT_EQ(hz[0].corridor, "left");
  EXPECT_EQ(hz[0].kind, "up");
  EXPECT_EQ(hz[0].how, "line");
  // a 20 mm step up across the path: only the reference plane sees it
  const auto step = scan([](double x, double) {return -H + (x > 0.25 ? 0.02 : 0.0);});
  hz = gs2Hazards(step, kFloor, true);
  ASSERT_EQ(hz.size(), 3u);
  for (const auto & h : hz) {
    EXPECT_EQ(h.kind, "up");
    EXPECT_EQ(h.how, "plane");
  }
  // a 50 mm step down: the centre of the line is out of range
  const auto down = scan([](double x, double) {return -H - (x > 0.25 ? 0.05 : 0.0);});
  bool gap = false;
  for (const auto & h : gs2Hazards(down, kFloor, true)) {gap |= h.corridor == "centre" && h.kind == "down";}
  EXPECT_TRUE(gap);
}

TEST(Core, HazardGuard)
{
  HazardGuard g;
  EXPECT_EQ(g.command(0, 0, 0, 0).state, "clear");
  EXPECT_EQ(g.verdict("up", 0.02), "step");
  EXPECT_EQ(g.verdict("up", 0.10), "stop");
  EXPECT_EQ(g.verdict("down", -0.03), "step");
  EXPECT_EQ(g.verdict("down", kNaN, true), "step");
  EXPECT_EQ(g.verdict("up", kNaN), "step");
  GuardParams deep;
  deep.deep_stop = true;
  EXPECT_EQ(HazardGuard(deep).verdict("down", kNaN, true), "stop");
  auto add = [&](double t, double x, double y, const char * v, double lift, int n) {
      for (int k = 0; k < n; ++k) {g.add(t, x + 0.01 * k, y, v, lift);}
    };
  add(0, 0.8, 0.115, "step", 0.02, 1);  // one stray report does nothing
  EXPECT_EQ(g.command(0, 0, 0, 0).state, "clear");
  add(0, 0.8, 0.115, "step", 0.02, 2);  // confirmed stone on the left foot line
  auto c = g.command(0, 0, 0, 0);
  EXPECT_EQ(c.state, "caution");
  EXPECT_DOUBLE_EQ(c.max_vx, 0.08);
  c = g.command(1, 0.6, 0, 0);  // 0.2 m ahead: left front foot only
  EXPECT_EQ(c.state, "step_over");
  EXPECT_DOUBLE_EQ(c.max_vx, 0.05);
  EXPECT_NEAR(c.step[0], 0.03, 1e-9);
  for (int leg = 1; leg < 4; ++leg) {EXPECT_TRUE(std::isnan(c.step[leg]));}
  c = g.command(2, 0.72, 0, 0);  // front foot nearly over, rear not yet
  EXPECT_TRUE(std::isfinite(c.step[0]));
  EXPECT_TRUE(std::isnan(c.step[2]));
  c = g.command(3, 0.95, 0, 0);  // left rear foot
  EXPECT_NEAR(c.step[2], 0.03, 1e-9);
  EXPECT_TRUE(std::isnan(c.step[0]) && std::isnan(c.step[3]));
  EXPECT_EQ(g.command(4, 1.1, 0, 0).state, "clear");
  add(4, 1.5, 0, "step", 0.0, 3);  // unknown height: slow down, no high swing
  c = g.command(4, 1.3, 0, 0);
  EXPECT_EQ(c.state, "caution");
  for (double s : c.step) {EXPECT_TRUE(std::isnan(s));}
  add(5, 2.0, 0, "stop", 0.1, 2);  // a wall: stop after 3 reports
  EXPECT_NE(g.command(5, 1.75, 0, 0).state, "stop");
  add(5, 2.0, 0.03, "stop", 0.1, 1);
  c = g.command(5, 1.75, 0, 0);
  EXPECT_EQ(c.state, "stop");
  EXPECT_DOUBLE_EQ(c.max_vx, 0.0);
  EXPECT_NEAR(c.d, 0.25, 0.01);  // cell mean
  EXPECT_EQ(g.command(5.5, 1.75, 0, M_PI / 2).state, "clear");  // turned away
  EXPECT_EQ(g.command(21, 1.75, 0, 0).state, "clear");  // forgotten after `memory`
  // right at the wall the lidars call it 'step' (no edge in view): the stop holds
  HazardGuard w;
  for (int k = 0; k < 3; ++k) {w.add(0, 2.0 + 0.01 * k, 0, "stop", 0.1);}
  for (int k = 0; k < 2000; ++k) {w.add(0.01 * k, 2.0, 0, "step", 0.0);}  // 60/s for 30 s
  EXPECT_EQ(w.command(20, 1.75, 0, 0).state, "stop");
  EXPECT_LE(w.size(), 2u);  // memory grows with the area, not with the reports
}

TEST(Core, GuardChoosesTheCrawlForStepsAndBars)
{
  HazardGuard g;
  EXPECT_EQ(g.verdict("up", 0.02), "step");     // trot, high swing
  EXPECT_EQ(g.verdict("up", 0.05), "crawl");    // step / stair / bar
  EXPECT_EQ(g.verdict("up", 0.09), "stop");     // too tall: go round
  EXPECT_EQ(g.verdict("down", -0.03), "step");
  EXPECT_EQ(g.verdict("down", -0.05), "crawl");
  EXPECT_EQ(g.verdict("down", -0.10), "stop");
  GuardParams no;
  no.crawl = false;
  EXPECT_EQ(HazardGuard(no).verdict("up", 0.05), "stop");
  for (int k = 0; k < 3; ++k) {g.add(0, 0.6 + 0.01 * k, 0.12, "crawl", 0.05);}
  EXPECT_EQ(g.command(0, 0.0, 0, 0).gait, 0);  // 0.6 m ahead: not yet
  auto c = g.command(1, 0.2, 0, 0);            // 0.4 m: stop and switch to the crawl
  EXPECT_EQ(c.state, "crawl");
  EXPECT_EQ(c.gait, 1);
  EXPECT_TRUE(std::isinf(c.max_vx));
  EXPECT_EQ(g.command(2, 0.85, 0, 0).gait, 1);  // under the body: keep crawling
  EXPECT_EQ(g.command(3, 0.95, 0, 0).gait, 0);  // rear feet past it: back to the trot

  // stairs: the next riser, seen from a tread, reads as a small step - the
  // crawl goes on while it is in the window, it does not start on it
  HazardGuard s;
  for (int k = 0; k < 3; ++k) {s.add(0, 0.8, 0.0, "crawl", 0.05);}
  EXPECT_EQ(s.command(0, 0.5, 0, 0).gait, 1);
  for (int k = 0; k < 3; ++k) {s.add(1, 1.1, 0.0, "step", 0.02);}
  EXPECT_EQ(s.command(2, 1.0, 0, 0).gait, 1);   // 0.8 behind the rear feet, 1.1 under the body
  EXPECT_EQ(s.command(3, 1.45, 0, 0).gait, 0);  // both passed
  EXPECT_EQ(s.command(4, 0.8, 0, 0).gait, 0);   // a 'step' alone never starts it
}

TEST(Core, GoesRoundAnObstacleAndBackToItsLine)
{
  ElevationMap map(3.0, 0.02);
  map.recenter(1.0, 0.0);
  std::vector<V3> pts;
  for (double x = -0.4; x < 2.4; x += 0.01) {
    for (double y = -1.4; y < 1.4; y += 0.01) {
      const bool block = x > 1.0 && x < 1.1 && std::abs(y) < 0.1;
      pts.push_back({x, y, block ? 0.10 : 0.0});
    }
  }
  map.insert(pts);
  Obstacle o = tallObstacle(map, 0.7, 0.0, 0.0, 0.0, 0.07, -0.35);
  ASSERT_TRUE(o.found);
  EXPECT_NEAR(o.lat_min, -0.10, 0.02);
  EXPECT_NEAR(o.lat_max, 0.10, 0.02);
  EXPECT_NEAR(o.d_min, 0.30, 0.02);
  Avoider a;
  double x = 0.7, y = 0.0, max_y = 0.0;
  const double dt = 0.05;
  bool passed = false;
  for (int k = 0; k < 2000 && !(passed && a.state() == "idle"); ++k) {
    o = tallObstacle(map, x, y, 0.0, 0.0, 0.07, -0.35);
    // the guard: no forward motion while the block is in the path and near
    const bool blocked = o.found && o.lat_max > -0.2 && o.lat_min < 0.2 && o.d_min < 0.35;
    const double vy = a.update(blocked, o, x, y, 0.0);
    x += (blocked ? 0.0 : 0.05) * dt;
    // the walk drifts back towards the block (heading wandering): the side
    // step is held until the block is behind
    y += (vy + (a.state() == "past" ? 0.02 : 0.0)) * dt;
    max_y = std::max(max_y, std::abs(y));
    passed = passed || x > 1.45;
    // the robot's body (+-0.15 x, +-0.12 y) never touches the block
    EXPECT_FALSE(std::abs(x - 1.05) < 0.05 + 0.15 && std::abs(y) < 0.1 + 0.12) << x << " " << y;
  }
  EXPECT_TRUE(passed);
  EXPECT_EQ(a.state(), "idle");
  EXPECT_NEAR(y, 0.0, 0.035);     // back on its line
  EXPECT_GT(max_y, 0.33);         // went round by the block's half width + the path + margin
  EXPECT_LT(max_y, 0.45);
}

TEST(Core, AWanderingHeadingDoesNotStopItBesideTheBlock)
{
  // the sim: beside a 150 mm block, the heading a few degrees towards it, the
  // corner of the block came back into the 0.20 m path and the map stopped
  // the robot for good
  ElevationMap map(3.0, 0.02);
  map.recenter(1.0, 0.0);
  std::vector<V3> pts;
  for (double x = -0.4; x < 2.4; x += 0.01) {
    for (double y = -1.4; y < 1.4; y += 0.01) {
      const bool block = x > 1.0 && x < 1.2 && std::abs(y) < 0.1;
      // the map smears its sides by 2 cm, taller than climb_max there too
      const bool smear = x > 1.0 && x < 1.2 && std::abs(y) < 0.12;
      pts.push_back({x, y, block ? 0.15 : (smear ? 0.10 : 0.0)});
    }
  }
  map.insert(pts);
  Avoider a;
  double x = 0.7, y = 0.0;
  const double dt = 0.05;
  bool passed = false;
  for (int k = 0; k < 3000 && !(passed && a.state() == "idle"); ++k) {
    // it goes round on the left and turns towards the block (right, 7 degrees: the sim saw 7.4) once past it
    const double yaw = a.state() == "past" ? -0.12 : 0.0;
    const Obstacle o = tallObstacle(map, x, y, yaw, 0.0, 0.09, -0.35);
    // the node: stop while the block is in the path (as narrow as the avoider says) and near
    const double hw = a.pathHalfWidth();
    const bool blocked = o.found && o.d_min > 0.0 && o.lat_max > -hw && o.lat_min < hw && o.d_min < 0.30;
    const double vy = a.update(blocked, o, x, y, yaw);
    x += (blocked || a.state() == "aside" ? 0.0 : 0.05) * dt;
    y += vy * dt;
    passed = passed || x > 1.6;
  }
  EXPECT_TRUE(passed) << "stuck at x " << x << " y " << y << " (" << a.state() << ")";
  EXPECT_EQ(a.state(), "idle");
}

TEST(Core, ABlockSeenFromTheSideIsTall)
{
  // lidar points on the near face of a 150 mm block only: the mean of the
  // face cell is half the height, its highest point the height
  ElevationMap map(3.0, 0.02);
  map.recenter(1.0, 0.0);
  std::vector<V3> pts;
  for (double x = 0.2; x < 1.0; x += 0.01) {
    for (double y = -0.5; y < 0.5; y += 0.01) {pts.push_back({x, y, 0.0});}
  }
  for (double z = 0.0; z <= 0.15; z += 0.01) {
    for (double y = -0.1; y < 0.1; y += 0.01) {pts.push_back({1.005, y, z});}
  }
  map.insert(pts);
  EXPECT_LT(map.heightAt(1.005, 0.0), 0.08);
  const Obstacle o = tallObstacle(map, 0.5, 0.0, 0.0, 0.0, 0.07, -0.35);
  ASSERT_TRUE(o.found);
  EXPECT_NEAR(o.d_min, 0.51, 0.02);
}

TEST(Core, StairsAreNotTallObstacles)
{
  // three 50 mm steps: 150 mm above the floor, but every riser is a step
  ElevationMap map(3.0, 0.02);
  map.recenter(1.0, 0.0);
  std::vector<V3> pts;
  for (double x = -0.4; x < 2.4; x += 0.01) {
    for (double y = -1.4; y < 1.4; y += 0.01) {
      pts.push_back({x, y, 0.05 * std::clamp(std::floor((x - 0.8) / 0.3) + 1.0, 0.0, 3.0)});
    }
  }
  map.insert(pts);
  EXPECT_FALSE(tallObstacle(map, 0.6, 0.0, 0.0, 0.0, 0.07, -0.35).found);
  EXPECT_FALSE(tallObstacle(map, 1.2, 0.0, 0.0, 0.10, 0.07, -0.35).found);
}

TEST(Core, DoesNotTryToGoRoundAWideWall)
{
  ElevationMap map(3.0, 0.02);
  map.recenter(1.0, 0.0);
  std::vector<V3> pts;
  for (double x = 1.0; x < 1.1; x += 0.01) {
    for (double y = -1.4; y < 1.4; y += 0.01) {pts.push_back({x, y, 0.1});}
  }
  map.insert(pts);
  Avoider a;
  const Obstacle o = tallObstacle(map, 0.75, 0.0, 0.0, 0.0, 0.07, -0.35);
  EXPECT_DOUBLE_EQ(a.update(true, o, 0.75, 0.0, 0.0), 0.0);
  EXPECT_EQ(a.state(), "idle");
}

TEST(Core, AWallSeenInPartIsNotNarrow)
{
  // the sim: an 80 mm wall, 0.8 m wide, only its middle mapped yet - the
  // mapped part alone would be narrow enough to go round
  ElevationMap map(3.0, 0.02);
  map.recenter(1.0, 0.0);
  std::vector<V3> pts;
  for (double x = -0.4; x < 1.3; x += 0.01) {
    for (double y = -0.25; y < 0.25; y += 0.01) {
      pts.push_back({x, y, x > 1.0 && x < 1.1 ? 0.08 : 0.0});
    }
  }
  map.insert(pts);
  const Obstacle o = tallObstacle(map, 0.75, 0.0, 0.0, 0.0, 0.07, -0.35);
  ASSERT_TRUE(o.found);
  EXPECT_TRUE(o.open_left);
  EXPECT_TRUE(o.open_right);
  Avoider a;
  EXPECT_DOUBLE_EQ(a.update(true, o, 0.75, 0.0, 0.0), 0.0);
  EXPECT_EQ(a.state(), "idle");
  // a block with the ground mapped on both sides: closed ends
  ElevationMap m2(3.0, 0.02);
  m2.recenter(1.0, 0.0);
  pts.clear();
  for (double x = -0.4; x < 1.3; x += 0.01) {
    for (double y = -0.6; y < 0.6; y += 0.01) {
      pts.push_back({x, y, x > 1.0 && x < 1.1 && std::abs(y) < 0.1 ? 0.15 : 0.0});
    }
  }
  m2.insert(pts);
  const Obstacle b = tallObstacle(m2, 0.75, 0.0, 0.0, 0.0, 0.07, -0.35);
  ASSERT_TRUE(b.found);
  EXPECT_FALSE(b.open_left);
  EXPECT_FALSE(b.open_right);
}
