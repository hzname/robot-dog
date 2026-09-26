#include "dog_perception/submaps.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace dog_perception
{

// ------------------------------------------------------------------ graph
namespace
{
/// Whitened residual of an edge for node poses a, b.
std::array<double, 3> residual(const GraphEdge & e, const Pose2 & a, const Pose2 & b)
{
  const Pose2 pred = a.inverse().compose(b);
  const double c = std::cos(e.z.yaw), s = std::sin(e.z.yaw);
  const double dx = pred.x - e.z.x, dy = pred.y - e.z.y;
  return {(c * dx + s * dy) / e.sigma_xy, (-s * dx + c * dy) / e.sigma_xy,
    wrapAngle(pred.yaw - e.z.yaw) / e.sigma_yaw};
}

double & at(Pose2 & p, int k) {return k == 0 ? p.x : k == 1 ? p.y : p.yaw;}

/// Solve H x = b (H symmetric positive definite, n x n, row-major) by Cholesky.
bool cholSolve(std::vector<double> H, std::vector<double> & b, int n)
{
  for (int j = 0; j < n; ++j) {
    double d = H[j * n + j];
    for (int k = 0; k < j; ++k) {d -= H[j * n + k] * H[j * n + k];}
    if (d <= 0.0) {return false;}
    d = std::sqrt(d);
    H[j * n + j] = d;
    for (int i = j + 1; i < n; ++i) {
      double v = H[i * n + j];
      for (int k = 0; k < j; ++k) {v -= H[i * n + k] * H[j * n + k];}
      H[i * n + j] = v / d;
    }
  }
  for (int i = 0; i < n; ++i) {  // L y = b
    double v = b[i];
    for (int k = 0; k < i; ++k) {v -= H[i * n + k] * b[k];}
    b[i] = v / H[i * n + i];
  }
  for (int i = n - 1; i >= 0; --i) {  // L^T x = y
    double v = b[i];
    for (int k = i + 1; k < n; ++k) {v -= H[k * n + i] * b[k];}
    b[i] = v / H[i * n + i];
  }
  return true;
}
}  // namespace

double optimizePoseGraph(std::vector<Pose2> & nodes, const std::vector<GraphEdge> & edges, int iterations)
{
  const int N = static_cast<int>(nodes.size());
  auto cost = [&]() {
      double c = 0.0;
      for (const auto & e : edges) {
        const auto r = residual(e, nodes[e.a], nodes[e.b]);
        c += r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
      }
      return c;
    };
  if (N < 2 || edges.empty()) {return cost();}
  const int n = 3 * (N - 1);  // node 0 fixed
  for (int it = 0; it < iterations; ++it) {
    std::vector<double> H(static_cast<size_t>(n) * n, 0.0), g(n, 0.0);
    for (const auto & e : edges) {
      const auto r0 = residual(e, nodes[e.a], nodes[e.b]);
      // numeric Jacobian over the 6 parameters of the two nodes
      double J[3][6];
      int col[6];
      for (int side = 0; side < 2; ++side) {
        const int node = side == 0 ? e.a : e.b;
        for (int k = 0; k < 3; ++k) {
          const int c = side * 3 + k;
          col[c] = node == 0 ? -1 : 3 * (node - 1) + k;
          Pose2 a = nodes[e.a], b = nodes[e.b];
          const double h = 1e-6;
          at(side == 0 ? a : b, k) += h;
          const auto r1 = residual(e, a, b);
          for (int m = 0; m < 3; ++m) {
            double d = r1[m] - r0[m];
            if (m == 2) {d = wrapAngle(d);}
            J[m][c] = d / h;
          }
        }
      }
      for (int u = 0; u < 6; ++u) {
        if (col[u] < 0) {continue;}
        for (int m = 0; m < 3; ++m) {g[col[u]] -= J[m][u] * r0[m];}
        for (int v = 0; v < 6; ++v) {
          if (col[v] < 0) {continue;}
          double s = 0.0;
          for (int m = 0; m < 3; ++m) {s += J[m][u] * J[m][v];}
          H[static_cast<size_t>(col[u]) * n + col[v]] += s;
        }
      }
    }
    for (int i = 0; i < n; ++i) {H[static_cast<size_t>(i) * n + i] += 1e-9;}
    if (!cholSolve(H, g, n)) {break;}
    double step = 0.0;
    for (int i = 1; i < N; ++i) {
      for (int k = 0; k < 3; ++k) {
        at(nodes[i], k) += g[3 * (i - 1) + k];
        step = std::max(step, std::abs(g[3 * (i - 1) + k]));
      }
      nodes[i].yaw = wrapAngle(nodes[i].yaw);
    }
    if (step < 1e-7) {break;}
  }
  return cost();
}

// ------------------------------------------------------------------ places
std::vector<float> scanContext(const std::vector<P2> & pts, const ScanContextParams & p)
{
  std::vector<float> d(static_cast<size_t>(p.rings) * p.sectors, 0.0f);
  for (const auto & q : pts) {
    const double r = std::hypot(q.x, q.y);
    if (r >= p.r_max || r < 1e-6) {continue;}
    const int ring = std::min(p.rings - 1, static_cast<int>(r / p.r_max * p.rings));
    double a = std::atan2(q.y, q.x);
    if (a < 0) {a += 2.0 * M_PI;}
    const int sec = std::min(p.sectors - 1, static_cast<int>(a / (2.0 * M_PI) * p.sectors));
    d[static_cast<size_t>(ring) * p.sectors + sec] = 1.0f;
  }
  return d;
}

double scanContextDistance(const std::vector<float> & a, const std::vector<float> & b,
  const ScanContextParams & p, double * yaw)
{
  const int R = p.rings, S = p.sectors;
  double best = 1.0;
  int best_shift = 0;
  for (int sh = 0; sh < S; ++sh) {
    double sum = 0.0;
    int cols = 0;
    for (int j = 0; j < S; ++j) {
      const int jb = ((j - sh) % S + S) % S;
      double ab = 0.0, aa = 0.0, bb = 0.0;
      for (int r = 0; r < R; ++r) {
        const double x = a[static_cast<size_t>(r) * S + j], y = b[static_cast<size_t>(r) * S + jb];
        ab += x * y;
        aa += x * x;
        bb += y * y;
      }
      if (aa == 0.0 && bb == 0.0) {continue;}
      ++cols;
      sum += (aa == 0.0 || bb == 0.0) ? 1.0 : 1.0 - ab / std::sqrt(aa * bb);
    }
    const double dist = cols ? sum / cols : 1.0;
    if (dist < best) {
      best = dist;
      best_shift = sh;
    }
  }
  if (yaw) {*yaw = wrapAngle(best_shift * 2.0 * M_PI / S);}
  return best;
}

// ------------------------------------------------------------------ submaps
SubmapMap::SubmapMap(const SubmapParams & p)
: p_(p), merged_(p.resolution, p.min_hits) {}

std::vector<P2> SubmapMap::wallsOf(const Submap & s) const {return s.grid.walls();}

void SubmapMap::rebuildMerged()
{
  merged_ = WallGrid(p_.resolution, p_.min_hits);
  std::vector<P2> all;
  for (const auto & s : subs_) {
    for (const auto & w : wallsOf(s)) {all.push_back(s.pose.apply(w));}
  }
  for (int k = 0; k < p_.min_hits; ++k) {merged_.insert(all);}
  merged_.updateField();
  merged_dirty_ = false;
}

void SubmapMap::refresh()
{
  if (merged_dirty_ || merged_.dirty() || !merged_.fieldValid()) {
    merged_.updateField();
    merged_dirty_ = false;
  }
}

Pose2 SubmapMap::insert(const std::vector<P2> & pts_map, const Pose2 & robot, double walked)
{
  Pose2 correction;
  if (subs_.empty()) {
    Submap s;
    s.grid = WallGrid(p_.resolution, p_.min_hits);
    s.pose = robot;
    s.walked = walked;
    subs_.push_back(std::move(s));
  } else if (walked - subs_.back().walked >= p_.length && !subs_.back().grid.walls().empty()) {
    // a new submap where the robot is; the finished one looked for among the old
    const int k = static_cast<int>(subs_.size()) - 1;
    finishCurrent();
    Submap s;
    s.grid = WallGrid(p_.resolution, p_.min_hits);
    s.pose = robot;
    s.walked = walked;
    subs_.push_back(std::move(s));
    const double len = walked - subs_[k].walked;
    GraphEdge e;
    e.a = k;
    e.b = k + 1;
    e.z = subs_[k].pose.inverse().compose(robot);
    e.sigma_xy = p_.odom_sigma_xy + 0.01 * len;
    e.sigma_yaw = p_.odom_sigma_yaw;
    edges_.push_back(e);
    if (p_.loop_closure) {
      const Pose2 before = subs_.back().pose;
      if (const auto lc = closeLoop(k)) {
        const Pose2 after = subs_.back().pose;
        correction = after.compose(before.inverse());
        LoopClosure l = *lc;
        l.moved_m = std::hypot(after.x - before.x, after.y - before.y);
        l.moved_yaw = wrapAngle(after.yaw - before.yaw);
        loops_.push_back(l);
      }
    }
  }
  Submap & cur = subs_.back();
  const Pose2 inv = cur.pose.inverse();
  std::vector<P2> local;
  local.reserve(pts_map.size());
  for (const auto & q : pts_map) {local.push_back(inv.apply(correction.apply(q)));}
  cur.grid.insert(local);
  std::vector<P2> m;
  m.reserve(local.size());
  for (const auto & q : local) {m.push_back(cur.pose.apply(q));}
  merged_.insert(m);
  return correction;
}

void SubmapMap::finishCurrent()
{
  Submap & s = subs_.back();
  s.finished = true;
  s.grid.updateField();
  // its place: the walls round its origin, its own and its neighbours'
  // (from the merged map, in the submap's frame)
  merged_.updateField();
  const Pose2 inv = s.pose.inverse();
  std::vector<P2> near;
  for (const auto & w : merged_.walls()) {
    const P2 q = inv.apply(w);
    if (std::hypot(q.x, q.y) < p_.place.r_max) {near.push_back(q);}
  }
  s.place = scanContext(near, p_.place);
}

std::optional<LoopClosure> SubmapMap::closeLoop(int k)
{
  const Submap & sk = subs_[k];
  const auto cloud = wallsOf(sk);
  if (cloud.size() < 30) {return std::nullopt;}
  struct Cand {int j; double d;};
  std::vector<Cand> cands;
  for (int j = 0; j < k - p_.loop_skip; ++j) {
    const auto & sj = subs_[j];
    const double d = std::hypot(sj.pose.x - sk.pose.x, sj.pose.y - sk.pose.y);
    if (d < p_.loop_radius + 0.1 * (sk.walked - sj.walked)) {cands.push_back({j, d});}
  }
  std::sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b) {return a.d < b.d;});
  if (cands.size() > 3) {cands.resize(3);}
  std::optional<LoopClosure> best;
  GraphEdge best_edge;
  for (const auto & c : cands) {
    // the old submap with its neighbours, in its frame: more of the place to match
    WallGrid ref(p_.resolution, p_.min_hits);
    const Pose2 inv_j = subs_[c.j].pose.inverse();
    std::vector<P2> pts;
    for (int i = std::max(0, c.j - 1); i <= std::min(k - p_.loop_skip - 1, c.j + 1); ++i) {
      for (const auto & w : wallsOf(subs_[i])) {pts.push_back(inv_j.compose(subs_[i].pose).apply(w));}
    }
    for (int h = 0; h < p_.min_hits; ++h) {ref.insert(pts);}
    ref.updateField();
    GlobalParams gp;
    gp.window = true;
    gp.center = inv_j.compose(sk.pose);
    gp.win_xy = p_.loop_win_xy;
    gp.win_yaw = p_.loop_win_yaw;
    gp.step_xy = 0.05;
    gp.step_yaw = 0.026;
    gp.clearance = 0.0;
    const auto r = globalSearch(ref, cloud, gp);
    if (!r.ok || r.score < p_.loop_min_inliers) {continue;}
    if (!best || r.score > best->inliers) {
      best = LoopClosure{c.j, k, r.score, 0.0, 0.0};
      best_edge.a = c.j;
      best_edge.b = k;
      best_edge.z = r.best.pose;
      best_edge.sigma_xy = p_.loop_sigma_xy;
      best_edge.sigma_yaw = p_.loop_sigma_yaw;
      best_edge.loop = true;
    }
  }
  if (!best) {return std::nullopt;}
  edges_.push_back(best_edge);
  std::vector<Pose2> nodes;
  for (const auto & s : subs_) {nodes.push_back(s.pose);}
  optimizePoseGraph(nodes, edges_);
  for (size_t i = 0; i < subs_.size(); ++i) {subs_[i].pose = nodes[i];}
  rebuildMerged();
  return best;
}

GlobalResult SubmapMap::relocalize(const std::vector<P2> & cloud, const GlobalParams & gp,
  const MatchParams & mp) const
{
  GlobalResult out;
  if (subs_.empty() || !merged_.fieldValid()) {return out;}
  const auto thin_cloud = thin(cloud, p_.resolution);
  const auto desc = scanContext(thin_cloud, p_.place);
  struct Cand {double d; Pose2 guess;};
  std::vector<Cand> cands;
  for (const auto & s : subs_) {
    if (s.place.empty()) {continue;}
    double yaw = 0.0;
    const double d = scanContextDistance(s.place, desc, p_.place, &yaw);
    cands.push_back({d, s.pose.compose(Pose2{0.0, 0.0, yaw})});
  }
  std::sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b) {return a.d < b.d;});
  if (cands.size() > 8) {cands.resize(8);}
  // each likely place searched round its origin; the answer must beat the
  // others: two places that fit alike (corners of a ring) are no answer
  std::vector<GlobalResult> res;
  for (const auto & c : cands) {
    GlobalParams w = gp;
    w.window = true;
    w.center = c.guess;
    w.win_xy = p_.length;    // the robot is within about a submap's length of some origin
    w.win_yaw = 0.35;
    res.push_back(globalSearch(merged_, thin_cloud, w, mp));
  }
  std::sort(res.begin(), res.end(), [](const GlobalResult & a, const GlobalResult & b) {return a.score > b.score;});
  if (!res.empty() && res[0].best.ok && res[0].score >= 0.5) {
    out = res[0];
    for (size_t i = 1; i < res.size(); ++i) {
      const auto & o = res[i].best.pose;
      if (std::hypot(o.x - out.best.pose.x, o.y - out.best.pose.y) > 0.3 ||
        std::abs(wrapAngle(o.yaw - out.best.pose.yaw)) > 0.26)
      {
        out.second = std::max(out.second, res[i].score);
      }
    }
    out.ok = out.second < gp.ambiguity * out.score;
    if (out.ok) {return out;}
  }
  return globalSearch(merged_, thin_cloud, gp, mp);  // no single place fits: everywhere
}

bool SubmapMap::save(const std::string & path) const
{
  std::ofstream g(path + ".graph");
  if (!g) {return false;}
  g.precision(10);
  g << "submaps " << subs_.size() << "\n";
  for (size_t k = 0; k < subs_.size(); ++k) {
    const auto & s = subs_[k];
    g << k << " " << s.pose.x << " " << s.pose.y << " " << s.pose.yaw << " " << s.walked << "\n";
    if (!s.grid.save(path + ".sub" + std::to_string(k))) {return false;}
  }
  g << "edges " << edges_.size() << "\n";
  for (const auto & e : edges_) {
    g << e.a << " " << e.b << " " << e.z.x << " " << e.z.y << " " << e.z.yaw << " " << e.sigma_xy << " "
      << e.sigma_yaw << " " << (e.loop ? 1 : 0) << "\n";
  }
  if (!g) {return false;}
  return merged_.save(path);
}

bool SubmapMap::load(const std::string & path)
{
  subs_.clear();
  edges_.clear();
  loops_.clear();
  std::ifstream g(path + ".graph");
  if (g) {
    std::string word;
    size_t n = 0;
    g >> word >> n;
    for (size_t i = 0; i < n; ++i) {
      Submap s;
      size_t k = 0;
      g >> k >> s.pose.x >> s.pose.y >> s.pose.yaw >> s.walked;
      s.grid = WallGrid(p_.resolution, p_.min_hits);
      if (!g || !s.grid.load(path + ".sub" + std::to_string(k))) {return false;}
      s.finished = true;
      subs_.push_back(std::move(s));
    }
    g >> word >> n;
    for (size_t i = 0; i < n; ++i) {
      GraphEdge e;
      int loop = 0;
      g >> e.a >> e.b >> e.z.x >> e.z.y >> e.z.yaw >> e.sigma_xy >> e.sigma_yaw >> loop;
      e.loop = loop != 0;
      if (g) {edges_.push_back(e);}
    }
  } else {
    Submap s;  // a single-grid map
    s.grid = WallGrid(p_.resolution, p_.min_hits);
    if (!s.grid.load(path)) {return false;}
    s.finished = true;
    subs_.push_back(std::move(s));
  }
  rebuildMerged();
  for (auto & s : subs_) {  // places from the merged walls round each origin
    const Pose2 inv = s.pose.inverse();
    std::vector<P2> near;
    for (const auto & w : merged_.walls()) {
      const P2 q = inv.apply(w);
      if (std::hypot(q.x, q.y) < p_.place.r_max) {near.push_back(q);}
    }
    s.place = scanContext(near, p_.place);
  }
  return true;
}

}  // namespace dog_perception
