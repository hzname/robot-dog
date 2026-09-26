// C++ port of dog_perception/core.py - see core.hpp.
#include "dog_perception/core.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace dog_perception
{

namespace
{
constexpr double kDeg = M_PI / 180.0;

/// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix (Jacobi).
V3 smallestEigenvector(M3 a)
{
  M3 v = identity();
  for (int sweep = 0; sweep < 50; ++sweep) {
    const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    if (off < 1e-30) {break;}
    for (int p = 0; p < 2; ++p) {
      for (int q = p + 1; q < 3; ++q) {
        if (std::abs(a[p][q]) < 1e-300) {continue;}
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for (int k = 0; k < 3; ++k) {  // A J
          const double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {  // J^T (A J)
          const double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; ++k) {
          const double vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  int i = 0;
  for (int k = 1; k < 3; ++k) {
    if (a[k][k] < a[i][i]) {i = k;}
  }
  return {v[0][i], v[1][i], v[2][i]};
}

std::pair<double, double> profileJumps(const std::vector<double> & x, const std::vector<double> & h,
  double bin, int bin_points)
{
  std::map<long, std::vector<double>> bins;
  for (size_t i = 0; i < x.size(); ++i) {
    bins[static_cast<long>(std::floor(x[i] / bin))].push_back(h[i]);
  }
  std::vector<long> keys;
  std::vector<double> med;
  for (auto & kv : bins) {
    if (static_cast<int>(kv.second.size()) >= bin_points) {
      keys.push_back(kv.first);
      med.push_back(median(kv.second));
    }
  }
  double up = 0.0, down = 0.0;
  for (size_t gap = 1; gap <= 2; ++gap) {
    for (size_t i = 0; i + gap < keys.size(); ++i) {
      if (keys[i + gap] - keys[i] > 2) {continue;}
      const double d = med[i + gap] - med[i];
      up = std::max(up, d);
      down = std::max(down, -d);
    }
  }
  return {up, down};
}
}  // namespace

// ------------------------------------------------------------------ math
M3 rotRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
    {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
    {-sp, cp * sr, cp * cr}}};
}

M3 quatToRot(double x, double y, double z, double w)
{
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
    {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
    {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

M3 identity()
{
  return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}

M3 transpose(const M3 & a)
{
  M3 t{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {t[i][j] = a[j][i];}
  }
  return t;
}

M3 mul(const M3 & a, const M3 & b)
{
  M3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
    }
  }
  return c;
}

V3 mul(const M3 & a, const V3 & v)
{
  return {a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
    a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
    a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z};
}

void rollPitchOfNormal(const V3 & n, double & roll, double & pitch)
{
  roll = std::atan2(n.y, n.z);
  pitch = std::atan2(-n.x, std::hypot(n.y, n.z));
}

double median(std::vector<double> v)
{
  if (v.empty()) {return kNaN;}
  const size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + m, v.end());
  const double hi = v[m];
  if (v.size() % 2) {return hi;}
  return 0.5 * (hi + *std::max_element(v.begin(), v.begin() + m));
}

// ------------------------------------------------------------------ sensors
std::map<std::string, SensorMount> mountsFromParams(const SensorParams & s)
{
  std::map<std::string, SensorMount> out;
  if (s.x_lidar) {
    const double tilt = s.x_lidar_tilt_deg * kDeg, yaw = s.x_lidar_yaw_deg * kDeg;
    for (const auto & [name, side] : {std::pair<const char *, double>{"lidar_left", 1.0}, {"lidar_right", -1.0}}) {
      out[name] = {name, {s.x_lidar_x, side * s.x_lidar_y, s.x_lidar_z}, rotRpy(0.0, tilt, -side * yaw)};
    }
  }
  if (s.gs2) {
    out["gs2"] = {"gs2", {s.gs2_x, s.gs2_y, s.gs2_z}, rotRpy(0.0, s.gs2_pitch_deg * kDeg, 0.0)};
  }
  if (s.tof) {
    const size_t n = s.tof_names.size();
    for (const auto * v : {&s.tof_x, &s.tof_y, &s.tof_z, &s.tof_pitch_deg, &s.tof_yaw_deg}) {
      if (v->size() != n) {throw std::invalid_argument("sensors.tof_*: every list needs one value per tof_names");}
    }
    for (size_t k = 0; k < n; ++k) {
      const std::string name = "tof_" + s.tof_names[k];
      out[name] = {name, {s.tof_x[k], s.tof_y[k], s.tof_z[k]},
        rotRpy(0.0, s.tof_pitch_deg[k] * kDeg, s.tof_yaw_deg[k] * kDeg)};
    }
  }
  return out;
}

std::vector<V3> scanToBody(const SensorMount & m, const std::vector<float> & ranges,
  double angle_min, double angle_inc, double rmin, double rmax)
{
  std::vector<V3> out;
  out.reserve(ranges.size());
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r <= rmin || r >= rmax) {continue;}
    const double a = angle_min + angle_inc * static_cast<double>(i);
    out.push_back(mul(m.R, V3{r * std::cos(a), r * std::sin(a), 0.0}) + m.p);
  }
  return out;
}

double rayToPlane(const V3 & p, const V3 & u, const Plane & plane)
{
  const double den = plane.n.dot(u);
  if (den >= -1e-6) {return kInf;}
  const double t = (plane.c - plane.n.dot(p)) / den;
  return t > 0 ? t : kInf;
}

// ------------------------------------------------------------------ planes
std::optional<Plane> fitPlane(const std::vector<V3> & pts)
{
  if (pts.size() < 3) {return std::nullopt;}
  V3 ctr;
  for (const auto & p : pts) {ctr = ctr + p;}
  ctr = ctr * (1.0 / static_cast<double>(pts.size()));
  M3 cov{};
  for (const auto & p : pts) {
    const V3 d = p - ctr;
    const double e[3] = {d.x, d.y, d.z};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {cov[i][j] += e[i] * e[j];}
    }
  }
  V3 n = smallestEigenvector(cov);
  n = n * (1.0 / n.norm());
  if (n.z < 0) {n = n * -1.0;}
  return Plane{n, n.dot(ctr)};
}

std::optional<RobustFit> robustPlane(const std::vector<V3> & pts, int iterations, double keep)
{
  if (pts.size() < 10) {return std::nullopt;}
  std::vector<bool> mask(pts.size(), true);
  Plane plane;
  std::vector<V3> sel;
  for (int it = 0; it < iterations; ++it) {
    sel.clear();
    for (size_t i = 0; i < pts.size(); ++i) {
      if (mask[i]) {sel.push_back(pts[i]);}
    }
    if (sel.size() < 10) {return std::nullopt;}
    plane = *fitPlane(sel);
    double s1 = 0.0, s2 = 0.0;
    for (const auto & p : sel) {
      const double r = plane.residual(p);
      s1 += r;
      s2 += r * r;
    }
    const double n = static_cast<double>(sel.size());
    const double sig = std::sqrt(std::max(0.0, s2 / n - (s1 / n) * (s1 / n)));
    const double lim = std::max(keep, 2.5 * sig);
    for (size_t i = 0; i < pts.size(); ++i) {mask[i] = std::abs(plane.residual(pts[i])) < lim;}
  }
  double s2 = 0.0;
  size_t n = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (mask[i]) {
      const double r = plane.residual(pts[i]);
      s2 += r * r;
      ++n;
    }
  }
  if (n == 0) {return std::nullopt;}
  return RobustFit{plane, std::sqrt(s2 / static_cast<double>(n)),
    static_cast<double>(n) / static_cast<double>(pts.size())};
}

// ------------------------------------------------------------------ legs
std::array<LegChain, 4> legsBody(const Geometry & g, const std::array<double, 12> & q)
{
  static const int kFront[4] = {1, 1, -1, -1}, kSide[4] = {1, -1, 1, -1};
  std::array<LegChain, 4> out;
  for (int leg = 0; leg < 4; ++leg) {
    const double sd = kSide[leg];
    const V3 hip{kFront[leg] * g.hip_x, sd * g.hip_y, 0.0};
    const M3 A = rotRpy(q[leg * 3], 0.0, 0.0);
    const V3 th = hip + mul(A, V3{0.0, sd * g.hip_offset, 0.0});
    const M3 B = mul(A, rotRpy(0.0, q[leg * 3 + 1], 0.0));
    const V3 kn = th + mul(B, V3{0.0, 0.0, -g.thigh});
    const V3 ft = kn + mul(mul(B, rotRpy(0.0, q[leg * 3 + 2], 0.0)), V3{0.0, 0.0, -g.calf});
    out[leg] = {th, kn, ft};
  }
  return out;
}

std::array<V3, 4> feetBody(const Geometry & g, const std::array<double, 12> & q, double foot_radius)
{
  const auto legs = legsBody(g, q);
  std::array<V3, 4> out;
  for (int leg = 0; leg < 4; ++leg) {out[leg] = legs[leg][2] - V3{0.0, 0.0, foot_radius};}
  return out;
}

namespace
{
double segmentDistance(const V3 & a, const V3 & b, const V3 & p)
{
  const V3 ab = b - a, ap = p - a;
  const double l2 = ab.dot(ab);
  const double t = l2 > 0.0 ? std::clamp(ap.dot(ab) / l2, 0.0, 1.0) : 0.0;
  return (p - (a + ab * t)).norm();
}
}  // namespace

bool onLeg(const std::array<LegChain, 4> & legs, const V3 & p, double r)
{
  for (const auto & l : legs) {
    if (segmentDistance(l[0], l[1], p) < r || segmentDistance(l[1], l[2], p) < r) {return true;}
  }
  return false;
}

bool FeetPlane::update(const std::array<V3, 4> & feet, const std::optional<M3> & R_imu)
{
  const auto plane = fitPlane(std::vector<V3>(feet.begin(), feet.end()));
  if (!plane) {return false;}
  double worst = 0.0;
  for (const auto & f : feet) {worst = std::max(worst, std::abs(plane->residual(f)));}
  if (worst >= tol_) {return false;}
  plane_ = plane;
  R_at_ = R_imu;
  return true;
}

std::optional<Plane> FeetPlane::current(const std::optional<M3> & R_imu) const
{
  if (!plane_) {return std::nullopt;}
  Plane p = *plane_;
  if (R_imu && R_at_) {
    // body rotated by dR since then: a fixed world plane moves by dR^T
    const M3 dR = mul(transpose(*R_at_), *R_imu);
    p.n = mul(transpose(dR), p.n);
  }
  return p;
}

// ------------------------------------------------------------------ elevation map
ElevationMap::ElevationMap(double size, double res)
: res_(res), n_(static_cast<int>(std::lround(size / res))),
  sum_(static_cast<size_t>(n_) * n_, 0.0), cnt_(static_cast<size_t>(n_) * n_, 0),
  max_(static_cast<size_t>(n_) * n_, -kInf)
{
}

void ElevationMap::recenter(double x, double y)
{
  const double nx = std::floor((x - n_ * res_ / 2) / res_) * res_;
  const double ny = std::floor((y - n_ * res_ / 2) / res_) * res_;
  const int dx = static_cast<int>(std::lround((nx - ox_) / res_));
  const int dy = static_cast<int>(std::lround((ny - oy_) / res_));
  if (dx == 0 && dy == 0) {return;}
  std::vector<double> sum(sum_.size(), 0.0);
  std::vector<int> cnt(cnt_.size(), 0);
  std::vector<double> mx(max_.size(), -kInf);
  for (int i = 0; i < n_; ++i) {
    const int si = i + dx;
    if (si < 0 || si >= n_) {continue;}
    for (int j = 0; j < n_; ++j) {
      const int sj = j + dy;
      if (sj < 0 || sj >= n_) {continue;}
      sum[i * n_ + j] = sum_[si * n_ + sj];
      cnt[i * n_ + j] = cnt_[si * n_ + sj];
      mx[i * n_ + j] = max_[si * n_ + sj];
    }
  }
  sum_.swap(sum);
  cnt_.swap(cnt);
  max_.swap(mx);
  ox_ = nx;
  oy_ = ny;
}

void ElevationMap::insert(const std::vector<V3> & pts)
{
  for (const auto & p : pts) {
    const int i = static_cast<int>(std::floor((p.x - ox_) / res_));
    const int j = static_cast<int>(std::floor((p.y - oy_) / res_));
    if (i < 0 || j < 0 || i >= n_ || j >= n_) {continue;}
    sum_[i * n_ + j] += p.z;
    cnt_[i * n_ + j] += 1;
    max_[i * n_ + j] = std::max(max_[i * n_ + j], p.z);
  }
}

std::vector<float> ElevationMap::mean() const
{
  std::vector<float> m(sum_.size());
  for (size_t k = 0; k < m.size(); ++k) {
    m[k] = cnt_[k] > 0 ? static_cast<float>(sum_[k] / cnt_[k]) : std::numeric_limits<float>::quiet_NaN();
  }
  return m;
}

// ------------------------------------------------------------------ detectors
const std::array<Corridor, 3> kCorridors{{{"left", 0.06, 0.18}, {"centre", -0.06, 0.06}, {"right", -0.18, -0.06}}};

std::vector<LidarHazard> lidarHazards(const std::vector<V3> & pts, const Plane & plane, double thr,
  double x_min, double x_max, int min_points, double bin, int bin_points)
{
  std::vector<LidarHazard> out;
  for (const auto & c : kCorridors) {
    std::vector<double> xs, rs;
    for (const auto & p : pts) {
      if (p.x > x_min && p.x < x_max && p.y > c.y0 && p.y < c.y1) {
        xs.push_back(p.x);
        rs.push_back(plane.residual(p));
      }
    }
    bool jumps_done = false;
    std::pair<double, double> jumps{0.0, 0.0};
    for (const char * kind : {"up", "down"}) {
      const bool up = kind[0] == 'u';
      std::vector<double> sel;
      double nearest = kInf;
      for (size_t i = 0; i < xs.size(); ++i) {
        if (up ? rs[i] > thr : rs[i] < -thr) {
          sel.push_back(rs[i]);
          nearest = std::min(nearest, xs[i]);
        }
      }
      if (static_cast<int>(sel.size()) < min_points) {continue;}
      if (!jumps_done) {
        jumps = profileJumps(xs, rs, bin, bin_points);
        jumps_done = true;
      }
      out.push_back({c.name, kind, nearest, median(sel), up ? jumps.first : -jumps.second});
    }
  }
  return out;
}

std::vector<Gs2Hazard> gs2Hazards(const std::vector<V3> & pts, const std::optional<Plane> & plane,
  bool expected_centre, double thr_local, double thr_abs, int min_points)
{
  std::vector<Gs2Hazard> out;
  const auto & centre = kCorridors[1];
  int in_centre = 0;
  for (const auto & p : pts) {
    if (p.y > centre.y0 && p.y < centre.y1) {++in_centre;}
  }
  if (expected_centre && in_centre < min_points) {
    out.push_back({"centre", "down", "gap", kNaN, 0.0, kNaN});
  }
  if (pts.size() < 10) {return out;}
  const size_t n = pts.size();
  std::vector<double> z(n), r(n);
  for (size_t i = 0; i < n; ++i) {z[i] = pts[i].z;}
  // robust line z(y): start from the median level (most of the line is floor)
  const double zm = median(z);
  for (size_t i = 0; i < n; ++i) {r[i] = z[i] - zm;}
  for (int it = 0; it < 4; ++it) {
    std::vector<double> ar(n);
    for (size_t i = 0; i < n; ++i) {ar[i] = std::abs(r[i]);}
    const double lim = std::max(0.006, 3.0 * 1.4826 * median(ar));
    double sy = 0, sz = 0, syy = 0, syz = 0;
    int m = 0;
    for (size_t i = 0; i < n; ++i) {
      if (std::abs(r[i]) < lim) {
        sy += pts[i].y;
        sz += z[i];
        syy += pts[i].y * pts[i].y;
        syz += pts[i].y * z[i];
        ++m;
      }
    }
    if (m < 8) {break;}
    const double den = m * syy - sy * sy;
    if (std::abs(den) < 1e-12) {break;}
    const double k = (m * syz - sy * sz) / den, b = (sz - k * sy) / m;
    for (size_t i = 0; i < n; ++i) {r[i] = z[i] - (k * pts[i].y + b);}
  }
  for (const auto & c : kCorridors) {
    std::vector<size_t> idx;
    for (size_t i = 0; i < n; ++i) {
      if (pts[i].y > c.y0 && pts[i].y < c.y1) {idx.push_back(i);}
    }
    if (static_cast<int>(idx.size()) < min_points) {continue;}
    for (const char * kind : {"up", "down"}) {
      const bool up = kind[0] == 'u';
      std::vector<double> sel;
      size_t best = n;
      for (size_t i : idx) {
        if (up ? r[i] > thr_local : r[i] < -thr_local) {
          sel.push_back(r[i]);
          if (best == n || pts[i].x < pts[best].x) {best = i;}
        }
      }
      if (static_cast<int>(sel.size()) >= min_points) {
        out.push_back({c.name, kind, "line", pts[best].x, pts[best].y, median(sel)});
      }
    }
    if (plane) {
      std::vector<double> res, xs;
      for (size_t i : idx) {
        res.push_back(plane->residual(pts[i]));
        xs.push_back(pts[i].x);
      }
      const double med = median(res);
      if (std::abs(med) > thr_abs) {
        out.push_back({c.name, med > 0 ? "up" : "down", "plane", median(xs), (c.y0 + c.y1) / 2, med});
      }
    }
  }
  return out;
}

TofDetector::TofDetector(const SensorMount & m, double thr, int confirm, double offset, int baseline_n,
  double max_expected)
: m_(m), thr_(thr), max_expected_(max_expected), confirm_(confirm), baseline_n_(baseline_n), offset_(offset)
{
}

TofDetector::Result TofDetector::check(double measured, const Plane & plane)
{
  Result r;
  r.expected = rayToPlane(m_.p, m_.beam(), plane);
  if (!std::isfinite(r.expected) || r.expected > max_expected_) {return r;}
  std::string kind;
  if (!std::isfinite(measured)) {
    kind = "down";
    r.residual = kInf;
  } else {
    r.residual = measured - r.expected - offset_;
    kind = r.residual < -thr_ ? "up" : (r.residual > thr_ ? "down" : "");
  }
  run_ = kind == last_ ? run_ + 1 : 1;
  last_ = kind;
  if (!kind.empty() && run_ >= confirm_) {r.verdict = kind;}
  return r;
}

bool TofDetector::calibrate(double measured, const Plane & plane)
{
  const double exp = rayToPlane(m_.p, m_.beam(), plane);
  if (std::isfinite(measured) && std::isfinite(exp) && static_cast<int>(offsets_.size()) < baseline_n_) {
    offsets_.push_back(measured - exp);
    offset_ = median(offsets_);
  }
  return calibrated();
}

// ------------------------------------------------------------------ guard
std::string HazardGuard::verdict(const std::string & kind, double edge, bool deep) const
{
  if (deep) {return p_.deep_stop ? "stop" : "step";}
  if (!std::isfinite(edge)) {return "step";}
  const double h = kind == "up" ? edge : -edge;
  const double trot = kind == "up" ? p_.trot_climb : p_.trot_descend;
  const double crawl = kind == "up" ? p_.climb_max : p_.descend_max;
  if (h <= trot) {return "step";}
  return (p_.crawl && h <= crawl) ? "crawl" : "stop";
}

void HazardGuard::add(double t, double x, double y, const std::string & verdict, double lift)
{
  auto & c = cells_[{static_cast<long>(std::floor(x / kCell)), static_cast<long>(std::floor(y / kCell))}];
  c.sx += x;
  c.sy += y;
  c.n += 1;
  c.n_stop += verdict == "stop";
  c.n_crawl += verdict == "crawl";
  c.t_last = t;
  if (std::isfinite(lift) && lift > c.lift) {c.lift = lift;}
}

HazardGuard::Command HazardGuard::command(double t, double x, double y, double yaw)
{
  Command out;
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  struct Live {const Cell * c; double d, lat;};
  std::map<std::pair<long, long>, Live> live;
  for (auto it = cells_.begin(); it != cells_.end(); ) {
    const Cell & c = it->second;
    const double dx = c.sx / c.n - x, dy = c.sy / c.n - y;
    const double d = cs * dx + sn * dy, lat = -sn * dx + cs * dy;
    const double behind = std::max(p_.pass_dist, p_.crawl_pass) + 0.2;
    if (t - c.t_last >= p_.memory || (d < -behind && std::abs(lat) < 0.6)) {
      it = cells_.erase(it);  // forgotten, or well behind
      continue;
    }
    live[it->first] = {&c, d, lat};
    ++it;
  }
  double dmin = kInf, dstop = kInf, dcrawl = kInf;
  bool crawl = false;
  for (const auto & [key, l] : live) {
    if (!(std::abs(l.lat) < p_.half_width)) {continue;}
    int n = 0, n_stop = 0, n_crawl = 0;  // the cell and its 8 neighbours
    for (long di = -1; di <= 1; ++di) {
      for (long dj = -1; dj <= 1; ++dj) {
        const auto nb = live.find({key.first + di, key.second + dj});
        if (nb != live.end()) {
          n += nb->second.c->n;
          n_stop += nb->second.c->n_stop;
          n_crawl += nb->second.c->n_crawl;
        }
      }
    }
    if (n < p_.confirm) {continue;}
    // an edge for the crawl: from crawl_dist ahead until the rear feet are past it
    const bool in_window = l.d < p_.crawl_dist && l.d > -p_.crawl_pass;
    if (in_window && ((n_crawl >= p_.stop_confirm && l.c->n_crawl > 0) || crawling_)) {
      // once crawling, any edge in the window keeps it: on stairs the next
      // riser, seen from a tread, may read lower than it is, and the trot
      // must not come back between two steps
      crawl = true;
      dcrawl = std::min(dcrawl, l.d);
    }
    if (l.d <= -p_.pass_dist) {continue;}
    dmin = std::min(dmin, l.d);
    if (n_stop >= p_.stop_confirm && l.c->n_stop > 0 && l.d > -0.1) {dstop = std::min(dstop, l.d);}
    if (l.c->lift > 0) {
      for (int leg = 0; leg < 4; ++leg) {
        const double fx = p_.feet[leg][0], fy = p_.feet[leg][1];
        if (std::abs(l.lat - fy) < p_.leg_width && l.d > fx - p_.leg_behind && l.d < fx + p_.leg_ahead) {
          const double h = std::min(p_.max_step, l.c->lift + p_.step_margin);
          out.step[leg] = std::isnan(out.step[leg]) ? h : std::max(out.step[leg], h);
        }
      }
    }
  }
  crawling_ = crawl;
  if (dstop < p_.stop_dist) {
    out.max_vx = 0.0;
    out.state = "stop";
    out.d = dstop;
    out.step = {kNaN, kNaN, kNaN, kNaN};
    out.gait = crawl ? 1 : 0;
    return out;
  }
  if (crawl) {  // the crawl is slow by itself and clears the terrain with its own swing
    out.state = "crawl";
    out.gait = 1;
    out.d = dcrawl;
    out.step = {kNaN, kNaN, kNaN, kNaN};
    return out;
  }
  if (!std::isfinite(dmin)) {
    out.step = {kNaN, kNaN, kNaN, kNaN};
    return out;
  }
  out.d = dmin;
  const bool lift = std::any_of(out.step.begin(), out.step.end(), [](double h) {return std::isfinite(h);});
  out.max_vx = lift ? p_.near_vx : p_.slow_vx;
  out.state = lift ? "step_over" : "caution";
  return out;
}

// ------------------------------------------------------------------ going round
double ElevationMap::maxAt(double x, double y) const
{
  const int i = static_cast<int>(std::floor((x - ox_) / res_));
  const int j = static_cast<int>(std::floor((y - oy_) / res_));
  if (i < 0 || j < 0 || i >= n_ || j >= n_) {return kNaN;}
  const int k = i * n_ + j;
  return cnt_[k] >= 3 ? max_[k] : kNaN;
}

double ElevationMap::heightAt(double x, double y) const
{
  const int i = static_cast<int>(std::floor((x - ox_) / res_));
  const int j = static_cast<int>(std::floor((y - oy_) / res_));
  if (i < 0 || j < 0 || i >= n_ || j >= n_) {return kNaN;}
  const int k = i * n_ + j;
  return cnt_[k] > 0 ? sum_[k] / cnt_[k] : kNaN;
}

Obstacle tallObstacle(const ElevationMap & map, double x, double y, double yaw, double ground_z,
  double height, double d0, double d1, double reach, bool highest)
{
  const double cs = std::cos(yaw), sn = std::sin(yaw), r = map.resolution();
  const int n = static_cast<int>(std::floor(2.0 * reach / r + 1e-9)) + 1;
  auto latOf = [&](int j) {return -reach + j * r;};
  // per lateral column: tall anywhere ahead, nearest, ends open
  std::vector<char> tall(n, 0), open_l(n, 0), open_r(n, 0);
  std::vector<double> dmin(n, kInf);
  for (double d = d0; d <= d1; d += r) {
    for (int j = 0; j < n; ++j) {
      const double lat = latOf(j);
      const double cx = x + cs * d - sn * lat, cy = y + sn * d + cs * lat;
      // (the mean as well only over the points a highest point needs: one
      // noisy point is no mean)
      const double hi = map.maxAt(cx, cy);
      const double h = highest ? hi : map.heightAt(cx, cy);
      if (!std::isfinite(hi) || h - ground_z <= height) {continue;}
      // tall over its surroundings too (a jump, not a height): a staircase is
      // high above the floor under the robot, but every riser is a step
      double low = h;
      for (double ex = -0.1; ex <= 0.1 + 1e-9; ex += r) {
        for (double ey = -0.1; ey <= 0.1 + 1e-9; ey += r) {
          const double hn = map.heightAt(cx + ex, cy + ey);
          if (std::isfinite(hn)) {low = std::min(low, hn);}
        }
      }
      if (h - low <= height) {continue;}
      // next to it, sideways, unmapped (fewer points than a tall cell needs)
      // or still raised (a wall's top read a little under `height`): it may
      // go on there - only the ground within 3 cells closes it (the map
      // smears a block's sides by 1-2 cells)
      auto open = [&](int dir) {
          for (int k = 1; k <= 3; ++k) {
            const double lk = lat + dir * k * r;
            if (lk < -reach - 1e-9 || lk > reach + 1e-9) {return true;}
            const double hn = map.maxAt(x + cs * d - sn * lk, y + sn * d + cs * lk);
            if (std::isfinite(hn) && hn - ground_z <= 0.5 * height) {return false;}
          }
          return true;
        };
      tall[j] = 1;
      dmin[j] = std::min(dmin[j], d);
      open_l[j] |= open(1);
      open_r[j] |= open(-1);
    }
  }
  // one obstacle: the run of tall columns (gaps of one cell bridged) nearest
  // the robot's line - a noisy cell far to the side is another thing, not
  // a wider one
  Obstacle o;
  double best = kInf;
  for (int j = 0; j < n; ) {
    if (!tall[j]) {++j; continue;}
    int end = j;
    while (end + 1 < n && (tall[end + 1] || (end + 2 < n && tall[end + 2]))) {end += tall[end + 1] ? 1 : 2;}
    double near = kInf, dm = kInf;
    for (int k = j; k <= end; ++k) {
      if (!tall[k]) {continue;}
      near = std::min(near, std::abs(latOf(k)));
      dm = std::min(dm, dmin[k]);
    }
    if (latOf(j) <= 0.0 && latOf(end) >= 0.0) {near = 0.0;}
    if (near < best) {
      best = near;
      o = {true, latOf(j) - r / 2, latOf(end) + r / 2, dm, open_l[end] != 0, open_r[j] != 0};
    }
    j = end + 1;
  }
  return o;
}

double Avoider::update(bool blocked, const Obstacle & o, double x, double y, double yaw)
{
  offset_ = state_ == "idle" ? 0.0 : -std::sin(yaw0_) * (x - x0_) + std::cos(yaw0_) * (y - y0_);
  const double clear = p_.half_width + 0.5 * p_.margin;
  const bool in_path = o.found && o.lat_max > -clear && o.lat_min < clear;
  if ((state_ == "idle" || state_ == "back") && blocked && o.found) {
    // shift needed to pass on the left / right (not where it runs into
    // unmapped ground: a wall seen in part is not narrow)
    const double left = o.open_left ? 1e9 : o.lat_max + p_.half_width + p_.margin;
    const double right = o.open_right ? 1e9 : -(o.lat_min - p_.half_width - p_.margin);
    needed_ = std::min(left, right);
    if (std::min(left, right) <= p_.max_shift) {
      if (state_ == "idle") {
        x0_ = x;
        y0_ = y;
        yaw0_ = yaw;
      }
      side_ = left <= right ? 1 : -1;
      state_ = "aside";
    }
  }
  if (state_ == "aside") {
    // the whole shift, as the obstacle reads now: walked + still needed that
    // side (an 80 mm wall's end read short from afar, the face's foot taken
    // for the ground: it looked narrow, and grew as the robot went aside)
    double still = 0.0;
    if (o.found) {
      const bool open = side_ > 0 ? o.open_left : o.open_right;
      still = open ? 1e9 : std::max(0.0, side_ > 0 ? o.lat_max + p_.half_width + p_.margin :
        -(o.lat_min - p_.half_width - p_.margin));
    }
    needed_ = std::abs(offset_) + still;
    if (!in_path) {
      state_ = "past";
      hold_ = offset_;
    } else if (std::abs(offset_) > p_.max_shift + 0.1 || needed_ > p_.max_shift + 0.02) {
      state_ = "idle";  // wider than it looked: give up, the guard keeps stopping
      return 0.0;
    } else {
      return side_ * p_.vy;
    }
  }
  if (state_ == "past") {
    // walk on until it is behind the rear feet, holding the side step: the
    // heading wanders a few degrees, and a free walk drifts back into it
    if (o.found) {return std::clamp(1.5 * (hold_ - offset_), -p_.vy, p_.vy);}
    state_ = "back";
  }
  if (state_ == "back") {
    if (std::abs(offset_) <= p_.back_tol) {
      state_ = "idle";
      return 0.0;
    }
    return offset_ > 0 ? -p_.vy : p_.vy;
  }
  return 0.0;
}

}  // namespace dog_perception
