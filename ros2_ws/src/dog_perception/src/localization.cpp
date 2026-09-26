#include "dog_perception/localization.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace dog_perception
{

double wrapAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

P2 Pose2::apply(const P2 & p) const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {x + c * p.x - s * p.y, y + s * p.x + c * p.y};
}

Pose2 Pose2::compose(const Pose2 & b) const
{
  const P2 t = apply({b.x, b.y});
  return {t.x, t.y, wrapAngle(yaw + b.yaw)};
}

Pose2 Pose2::inverse() const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {-(c * x + s * y), -(-s * x + c * y), wrapAngle(-yaw)};
}

// ------------------------------------------------------------------ grid
namespace
{
constexpr int kMaxCells = 2000;  // per side: 100 m at 5 cm
constexpr int kMargin = 20;      // cells added round the hits when growing

// Felzenszwalb & Huttenlocher squared distance transform in 1D.
void edt1d(const std::vector<double> & f, std::vector<double> & d, std::vector<int> & arg, int n,
  std::vector<int> & v, std::vector<double> & z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
    arg[q] = v[k];
  }
}
}  // namespace

WallGrid::WallGrid(double resolution, int min_hits, double max_dist)
: res_(resolution), min_hits_(std::max(min_hits, 1)), max_dist_(max_dist) {}

int WallGrid::occupiedCount() const
{
  return static_cast<int>(std::count_if(hits_.begin(), hits_.end(),
    [this](uint16_t h) {return h >= min_hits_;}));
}

bool WallGrid::occupied(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_) {return false;}
  return hits_[idx(ix, iy)] >= min_hits_;
}

bool WallGrid::occupiedAt(double x, double y) const
{
  return occupied(static_cast<int>(std::floor((x - ox_) / res_)),
           static_cast<int>(std::floor((y - oy_) / res_)));
}

void WallGrid::grow(int ix0, int iy0, int ix1, int iy1)
{
  // new bounds in cells of the current origin, inclusive
  ix0 = std::min(ix0, 0);
  iy0 = std::min(iy0, 0);
  ix1 = std::max(ix1, w_ - 1);
  iy1 = std::max(iy1, h_ - 1);
  const int nw = ix1 - ix0 + 1, nh = iy1 - iy0 + 1;
  const size_t n = static_cast<size_t>(nw) * nh;
  std::vector<uint16_t> hits(n, 0);
  std::vector<float> sx(n, 0.0f), sy(n, 0.0f);
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) {
      const size_t k = static_cast<size_t>(y - iy0) * nw + (x - ix0);
      hits[k] = hits_[idx(x, y)];
      sx[k] = sx_[idx(x, y)];
      sy[k] = sy_[idx(x, y)];
    }
  }
  hits_.swap(hits);
  sx_.swap(sx);
  sy_.swap(sy);
  ox_ += ix0 * res_;
  oy_ += iy0 * res_;
  w_ = nw;
  h_ = nh;
  // the nearest-wall table indexes cells: rebuild it now (matching goes on)
  if (field_valid_) {updateField();}
}

void WallGrid::insert(const std::vector<P2> & pts)
{
  if (pts.empty()) {return;}
  if (w_ == 0) {  // first points: a grid round them
    ox_ = std::floor(pts[0].x / res_) * res_;
    oy_ = std::floor(pts[0].y / res_) * res_;
    w_ = h_ = 1;
    hits_.assign(1, 0);
    sx_.assign(1, 0.0f);
    sy_.assign(1, 0.0f);
  }
  int x0 = 0, y0 = 0, x1 = w_ - 1, y1 = h_ - 1;
  for (const auto & p : pts) {
    const int ix = static_cast<int>(std::floor((p.x - ox_) / res_));
    const int iy = static_cast<int>(std::floor((p.y - oy_) / res_));
    x0 = std::min(x0, ix - kMargin / 2);
    y0 = std::min(y0, iy - kMargin / 2);
    x1 = std::max(x1, ix + kMargin / 2);
    y1 = std::max(y1, iy + kMargin / 2);
  }
  // keep a margin of free cells round every hit (the field reaches out)
  if (x0 < 0 || y0 < 0 || x1 >= w_ || y1 >= h_) {
    const int ax0 = x0 < 0 ? x0 - kMargin : 0, ay0 = y0 < 0 ? y0 - kMargin : 0;
    const int ax1 = x1 >= w_ ? x1 + kMargin : w_ - 1, ay1 = y1 >= h_ ? y1 + kMargin : h_ - 1;
    if (ax1 - ax0 < kMaxCells && ay1 - ay0 < kMaxCells) {grow(ax0, ay0, ax1, ay1);}
  }
  for (const auto & p : pts) {
    const int ix = static_cast<int>(std::floor((p.x - ox_) / res_));
    const int iy = static_cast<int>(std::floor((p.y - oy_) / res_));
    if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_) {continue;}  // beyond the size cap
    const int k = idx(ix, iy);
    if (hits_[k] < 60000) {
      if (hits_[k] + 1 == min_hits_) {dirty_ = true;}
      ++hits_[k];
      sx_[k] += static_cast<float>(p.x - (ox_ + ix * res_));
      sy_[k] += static_cast<float>(p.y - (oy_ + iy * res_));
    }
  }
}

P2 WallGrid::cellPoint(int k) const
{
  const int ix = k % w_, iy = k / w_;
  const double n = std::max<double>(hits_[k], 1.0);
  return {ox_ + ix * res_ + sx_[k] / n, oy_ + iy * res_ + sy_[k] / n};
}

void WallGrid::updateField()
{
  // exact Euclidean distance transform with the nearest occupied cell kept:
  // the distance is then measured to that cell's mean hit, not its centre
  // (a wall on a cell border would otherwise sit half a cell off)
  dirty_ = false;
  near_.assign(hits_.size(), -1);
  nx_.assign(hits_.size(), 0.0f);
  ny_.assign(hits_.size(), 0.0f);
  if (w_ == 0) {
    field_valid_ = true;
    return;
  }
  const double big = 1e12;
  const int n = std::max(w_, h_);
  std::vector<double> f(n), d(n), z(n + 1), col(hits_.size());
  std::vector<int> v(n), arg(n), rowarg(hits_.size());
  for (int x = 0; x < w_; ++x) {  // columns: nearest occupied row
    for (int y = 0; y < h_; ++y) {f[y] = hits_[idx(x, y)] >= min_hits_ ? 0.0 : big;}
    edt1d(f, d, arg, h_, v, z);
    for (int y = 0; y < h_; ++y) {
      col[idx(x, y)] = d[y];
      rowarg[idx(x, y)] = arg[y];
    }
  }
  computeNormals();
  const double cap = std::pow(max_dist_ / res_ + 2.0, 2);
  for (int y = 0; y < h_; ++y) {  // rows: nearest column, whose nearest row we know
    for (int x = 0; x < w_; ++x) {f[x] = col[idx(x, y)];}
    edt1d(f, d, arg, w_, v, z);
    for (int x = 0; x < w_; ++x) {
      if (d[x] <= cap) {near_[idx(x, y)] = idx(arg[x], rowarg[idx(arg[x], y)]);}
    }
  }
  field_valid_ = true;
}

void WallGrid::computeNormals()
{
  // a wall's normal from the mean hits of the occupied cells round it (5 x 5):
  // distances along a wall then do not pull points towards cell means
  for (int iy = 0; iy < h_; ++iy) {
    for (int ix = 0; ix < w_; ++ix) {
      const int k = idx(ix, iy);
      if (hits_[k] < min_hits_) {continue;}
      double mx = 0.0, my = 0.0;
      int n = 0;
      std::array<P2, 25> q;
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          if (occupied(ix + dx, iy + dy)) {
            q[n] = cellPoint(idx(ix + dx, iy + dy));
            mx += q[n].x;
            my += q[n].y;
            ++n;
          }
        }
      }
      if (n < 3) {continue;}
      mx /= n;
      my /= n;
      double cxx = 0.0, cxy = 0.0, cyy = 0.0;
      for (int i = 0; i < n; ++i) {
        cxx += (q[i].x - mx) * (q[i].x - mx);
        cxy += (q[i].x - mx) * (q[i].y - my);
        cyy += (q[i].y - my) * (q[i].y - my);
      }
      const double tr = cxx + cyy, det = cxx * cyy - cxy * cxy;
      const double l1 = 0.5 * tr + std::sqrt(std::max(0.25 * tr * tr - det, 0.0)), l2 = tr - l1;
      if (l1 <= 0.0 || l2 > 0.05 * l1) {continue;}  // a corner or a blob: no line
      // normal = eigenvector of the smaller eigenvalue
      double ex = cxy, ey = l2 - cxx;
      if (std::hypot(ex, ey) < 1e-12) {
        ex = l2 - cyy;
        ey = cxy;
      }
      const double en = std::hypot(ex, ey);
      if (en < 1e-12) {continue;}
      nx_[k] = static_cast<float>(ex / en);
      ny_[k] = static_cast<float>(ey / en);
    }
  }
}

double WallGrid::distance(double x, double y, double * gx, double * gy) const
{
  if (gx) {*gx = 0.0;}
  if (gy) {*gy = 0.0;}
  if (!field_valid_ || w_ == 0) {return max_dist_;}
  const int ix = static_cast<int>(std::floor((x - ox_) / res_));
  const int iy = static_cast<int>(std::floor((y - oy_) / res_));
  if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_) {return max_dist_;}
  const int k = near_[idx(ix, iy)];
  if (k < 0) {return max_dist_;}
  const P2 m = cellPoint(k);
  const double dx = x - m.x, dy = y - m.y;
  if (nx_[k] != 0.0f || ny_[k] != 0.0f) {  // to the wall's line
    const double s = nx_[k] * dx + ny_[k] * dy, d = std::abs(s);
    if (d >= max_dist_) {return max_dist_;}
    if (gx) {*gx = s >= 0.0 ? nx_[k] : -nx_[k];}
    if (gy) {*gy = s >= 0.0 ? ny_[k] : -ny_[k];}
    return d;
  }
  const double d = std::hypot(dx, dy);
  if (d >= max_dist_) {return max_dist_;}
  if (d > 1e-9) {
    if (gx) {*gx = dx / d;}
    if (gy) {*gy = dy / d;}
  }
  return d;
}

std::vector<int8_t> WallGrid::occupancy() const
{
  std::vector<int8_t> out(hits_.size(), 0);
  for (size_t k = 0; k < hits_.size(); ++k) {out[k] = hits_[k] >= min_hits_ ? 100 : 0;}
  return out;
}

bool WallGrid::save(const std::string & path) const
{
  const std::string pgm = path + ".pgm";
  std::ofstream img(pgm, std::ios::binary);
  if (!img) {return false;}
  img << "P5\n" << w_ << " " << h_ << "\n255\n";
  for (int y = h_ - 1; y >= 0; --y) {  // image top row = largest y
    for (int x = 0; x < w_; ++x) {img.put(static_cast<char>(occupied(x, y) ? 0 : 254));}
  }
  if (!img) {return false;}
  std::ofstream yml(path + ".yaml");
  if (!yml) {return false;}
  const auto slash = pgm.find_last_of('/');
  yml << "image: " << (slash == std::string::npos ? pgm : pgm.substr(slash + 1)) << "\n"
      << "mode: trinary\nresolution: " << res_ << "\norigin: [" << ox_ << ", " << oy_ << ", 0.0]\n"
      << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  if (!yml) {return false;}
  // the image keeps cells; the walls' positions inside them go alongside
  std::ofstream pts(path + ".walls");
  pts.precision(9);
  for (int k = 0; k < w_ * h_; ++k) {
    if (hits_[k] >= min_hits_) {
      const P2 m = cellPoint(k);
      pts << m.x << " " << m.y << "\n";
    }
  }
  return static_cast<bool>(pts);
}

bool WallGrid::load(const std::string & path)
{
  std::ifstream yml(path + ".yaml");
  if (!yml) {return false;}
  std::string line;
  double res = 0.0, ox = 0.0, oy = 0.0;
  while (std::getline(yml, line)) {
    if (line.rfind("resolution:", 0) == 0) {res = std::stod(line.substr(11));}
    if (line.rfind("origin:", 0) == 0) {
      std::string s = line.substr(line.find('[') + 1);
      std::replace(s.begin(), s.end(), ',', ' ');
      std::istringstream in(s);
      in >> ox >> oy;
    }
  }
  std::ifstream img(path + ".pgm", std::ios::binary);
  std::string magic;
  int w = 0, h = 0, maxv = 0;
  if (!img || res <= 0.0) {return false;}
  img >> magic >> w >> h >> maxv;
  img.get();
  if (magic != "P5" || w <= 0 || h <= 0 || w > kMaxCells || h > kMaxCells || maxv != 255) {return false;}
  std::vector<unsigned char> px(static_cast<size_t>(w) * h);
  img.read(reinterpret_cast<char *>(px.data()), static_cast<std::streamsize>(px.size()));
  if (!img) {return false;}
  res_ = res;
  ox_ = ox;
  oy_ = oy;
  w_ = w;
  h_ = h;
  hits_.assign(px.size(), 0);
  sx_.assign(px.size(), 0.0f);
  sy_.assign(px.size(), 0.0f);
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) {
      if (px[static_cast<size_t>(h_ - 1 - y) * w_ + x] < 128) {
        hits_[idx(x, y)] = static_cast<uint16_t>(min_hits_);
        sx_[idx(x, y)] = static_cast<float>(0.5 * res_ * min_hits_);  // cell centre
        sy_[idx(x, y)] = static_cast<float>(0.5 * res_ * min_hits_);
      }
    }
  }
  std::ifstream walls(path + ".walls");  // optional: where in the cells the walls are
  double wx = 0.0, wy = 0.0;
  while (walls >> wx >> wy) {
    const int ix = static_cast<int>(std::floor((wx - ox_) / res_));
    const int iy = static_cast<int>(std::floor((wy - oy_) / res_));
    if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_ || hits_[idx(ix, iy)] < min_hits_) {continue;}
    sx_[idx(ix, iy)] = static_cast<float>((wx - (ox_ + ix * res_)) * min_hits_);
    sy_[idx(ix, iy)] = static_cast<float>((wy - (oy_ + iy * res_)) * min_hits_);
  }
  updateField();
  return true;
}

// ------------------------------------------------------------------ matching
namespace
{
bool solve3(const double H[3][3], const double b[3], double x[3])
{
  const double det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) -
    H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
    H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);
  if (std::abs(det) < 1e-18) {return false;}
  for (int c = 0; c < 3; ++c) {
    double M[3][3];
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) {M[r][k] = k == c ? b[r] : H[r][k];}
    }
    x[c] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
      M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
      M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
  }
  return true;
}
}  // namespace

MatchResult match(const WallGrid & map, const std::vector<P2> & cloud, const Pose2 & guess,
  const MatchParams & p)
{
  MatchResult r;
  r.pose = guess;
  if (cloud.empty() || !map.fieldValid()) {return r;}
  const double sd2 = 0.05 * 0.05;  // point residual sigma^2 (the prior's scale)
  const double wp[3] = {1.0 / (p.prior_xy * p.prior_xy), 1.0 / (p.prior_xy * p.prior_xy),
    1.0 / (p.prior_yaw * p.prior_yaw)};
  Pose2 T = guess;
  for (int it = 0; it < p.iterations; ++it) {
    double H[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, b[3] = {0, 0, 0};
    for (const auto & pt : cloud) {
      const P2 q = T.apply(pt);
      double gx = 0.0, gy = 0.0;
      const double d = map.distance(q.x, q.y, &gx, &gy);
      if (d > p.outlier) {continue;}
      const double w = (d <= p.huber ? 1.0 : p.huber / d) / sd2;
      const double J[3] = {gx, gy, -gx * (q.y - T.y) + gy * (q.x - T.x)};
      for (int a = 0; a < 3; ++a) {
        b[a] += w * J[a] * d;
        for (int c = 0; c < 3; ++c) {H[a][c] += w * J[a] * J[c];}
      }
    }
    const double e[3] = {T.x - guess.x, T.y - guess.y, wrapAngle(T.yaw - guess.yaw)};
    for (int a = 0; a < 3; ++a) {
      H[a][a] += wp[a];
      b[a] += wp[a] * e[a];
    }
    double dx[3];
    const double nb[3] = {-b[0], -b[1], -b[2]};
    if (!solve3(H, nb, dx)) {break;}
    // no more than 10 cm / 6 deg a step: the field is only locally smooth
    const double sc = std::min({1.0, 0.1 / std::max(std::hypot(dx[0], dx[1]), 1e-12),
        0.1 / std::max(std::abs(dx[2]), 1e-12)});
    T.x += sc * dx[0];
    T.y += sc * dx[1];
    T.yaw = wrapAngle(T.yaw + sc * dx[2]);
    if (std::hypot(dx[0], dx[1]) < 1e-4 && std::abs(dx[2]) < 1e-4) {break;}
  }
  int used = 0, inl = 0;
  double ss = 0.0;
  for (const auto & pt : cloud) {
    const P2 q = T.apply(pt);
    const double d = map.distance(q.x, q.y);
    if (d > p.outlier) {continue;}
    ++used;
    ss += d * d;
    if (d < p.inlier) {++inl;}
  }
  r.pose = T;
  r.points = used;
  r.inlier_fraction = static_cast<double>(inl) / static_cast<double>(cloud.size());
  r.rms = used ? std::sqrt(ss / used) : 0.0;
  r.ok = used >= 10;
  return r;
}

std::vector<P2> thin(const std::vector<P2> & pts, double cell)
{
  std::unordered_set<int64_t> seen;
  std::vector<P2> out;
  for (const auto & p : pts) {
    const int64_t kx = static_cast<int64_t>(std::floor(p.x / cell)) + (1 << 20);
    const int64_t ky = static_cast<int64_t>(std::floor(p.y / cell)) + (1 << 20);
    if (seen.insert((kx << 32) | ky).second) {out.push_back(p);}
  }
  return out;
}

GlobalResult globalSearch(const WallGrid & map, const std::vector<P2> & cloud,
  const GlobalParams & gp, const MatchParams & mp)
{
  GlobalResult g;
  if (!map.fieldValid() || map.empty() || cloud.size() < 20) {return g;}
  // the cloud about its own centre, thinned
  std::vector<P2> pts = thin(cloud, map.resolution());
  if (static_cast<int>(pts.size()) > gp.max_points) {
    std::vector<P2> sub;
    const double stride = static_cast<double>(pts.size()) / gp.max_points;
    for (double k = 0.0; k < static_cast<double>(pts.size()); k += stride) {sub.push_back(pts[static_cast<size_t>(k)]);}
    pts.swap(sub);
  }
  std::vector<P2> coarse;  // every 5th point for the first pass
  for (size_t k = 0; k < pts.size(); k += 5) {coarse.push_back(pts[k]);}

  // candidate positions of the cloud frame's origin: free cells
  const double res = map.resolution();
  const P2 o = map.origin();
  std::vector<P2> pos;
  for (double y = o.y + 0.5 * res; y < o.y + map.height() * res; y += gp.step_xy) {
    for (double x = o.x + 0.5 * res; x < o.x + map.width() * res; x += gp.step_xy) {
      if (map.distance(x, y) >= gp.clearance) {pos.push_back({x, y});}
    }
  }
  const double sigma = 0.1;
  auto score = [&](const std::vector<P2> & rot, const P2 & at) {
      double s = 0.0;
      for (const auto & q : rot) {
        const double d = map.distance(q.x + at.x, q.y + at.y);
        if (d < sigma) {s += 1.0 - d / sigma;}
      }
      return s / static_cast<double>(rot.size());
    };
  struct Cand {double s; Pose2 p;};
  std::vector<Cand> cands;
  const int nyaw = std::max(1, static_cast<int>(std::round(2.0 * M_PI / gp.step_yaw)));
  for (int k = 0; k < nyaw; ++k) {
    const double yaw = wrapAngle(k * 2.0 * M_PI / nyaw);
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<P2> rot;
    for (const auto & q : coarse) {rot.push_back({c * q.x - s * q.y, s * q.x + c * q.y});}
    for (const auto & at : pos) {cands.push_back({score(rot, at), {at.x, at.y, yaw}});}
  }
  // second pass on the best 1 % with all points
  const size_t keep = std::min(cands.size(), std::max<size_t>(200, cands.size() / 100));
  std::partial_sort(cands.begin(), cands.begin() + keep, cands.end(),
    [](const Cand & a, const Cand & b) {return a.s > b.s;});
  cands.resize(keep);
  for (auto & cd : cands) {
    const double c = std::cos(cd.p.yaw), s = std::sin(cd.p.yaw);
    std::vector<P2> rot;
    for (const auto & q : pts) {rot.push_back({c * q.x - s * q.y, s * q.x + c * q.y});}
    cd.s = score(rot, {cd.p.x, cd.p.y});
  }
  std::sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b) {return a.s > b.s;});
  auto distinct = [](const Pose2 & a, const Pose2 & b) {
      return std::hypot(a.x - b.x, a.y - b.y) > 0.3 || std::abs(wrapAngle(a.yaw - b.yaw)) > 0.26;
    };
  std::vector<Pose2> seeds;
  for (const auto & cd : cands) {
    if (static_cast<int>(seeds.size()) >= gp.refine) {break;}
    if (std::all_of(seeds.begin(), seeds.end(), [&](const Pose2 & s) {return distinct(s, cd.p);})) {
      seeds.push_back(cd.p);
    }
  }
  // refine each with a weak prior; rank by inliers
  MatchParams loose = mp;
  loose.prior_xy = 1.0;
  loose.prior_yaw = 0.5;
  std::vector<MatchResult> refined;
  for (const auto & s : seeds) {refined.push_back(match(map, pts, s, loose));}
  std::sort(refined.begin(), refined.end(),
    [](const MatchResult & a, const MatchResult & b) {return a.inlier_fraction > b.inlier_fraction;});
  if (refined.empty()) {return g;}
  g.best = refined[0];
  g.score = refined[0].inlier_fraction;
  for (size_t k = 1; k < refined.size(); ++k) {
    if (distinct(refined[k].pose, g.best.pose)) {
      g.second = refined[k].inlier_fraction;
      break;
    }
  }
  g.ok = g.best.ok && g.score >= 0.5 && g.second < gp.ambiguity * g.score;
  return g;
}

}  // namespace dog_perception
