// Localization against a 2D map of walls and furniture, built from the two
// tilted lidars. Pure geometry, no ROS.
//
// Half of each tilted scan looks up: it hits walls, cabinets, the sofa. Those
// points, taken in a band of heights above the floor and dropped onto the
// floor plane, draw the room's outline - the same wherever the robot stands
// and however its body sways. The map is a grid of such hits (WallGrid) with a
// distance field to the nearest wall. A cloud of recent points (in the odom
// frame of dead reckoning) is matched to it by Gauss-Newton on (x, y, yaw) of
// the map -> odom correction, with the last correction as a prior: along a
// long corridor, where the walls fix nothing, the prior keeps what dead
// reckoning says. Roll and pitch come from the IMU.
//
// On start in a stored map the robot does not know where it is: globalSearch
// tries every free position and heading (the cloud from the survey, which
// sweeps the lidars over the room) and refines the best few.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace dog_perception
{

struct P2
{
  double x{0.0}, y{0.0};
};

struct Pose2
{
  double x{0.0}, y{0.0}, yaw{0.0};
  P2 apply(const P2 & p) const;
  Pose2 compose(const Pose2 & b) const;  // this * b
  Pose2 inverse() const;
};

double wrapAngle(double a);

class WallGrid
{
public:
  explicit WallGrid(double resolution = 0.05, int min_hits = 2, double max_dist = 0.5);

  double resolution() const {return res_;}
  int width() const {return w_;}
  int height() const {return h_;}
  P2 origin() const {return {ox_, oy_};}  // world position of cell (0, 0)'s corner
  bool empty() const {return w_ == 0;}
  int occupiedCount() const;

  /// Count a hit for each point (map frame). Grows the grid as needed.
  void insert(const std::vector<P2> & pts);
  bool occupied(int ix, int iy) const;
  bool occupiedAt(double x, double y) const;

  /// Recompute the distance field (after inserts; cheap for a room).
  void updateField();
  bool fieldValid() const {return field_valid_;}
  bool dirty() const {return dirty_;}  // new walls since the last updateField
  /// Distance to the wall line through the nearest occupied cell (its mean
  /// hit and the normal of its neighbourhood; at corners to the mean hit)
  /// [m], capped at max_dist. With the gradient (d/dx, d/dy) when gx, gy are given.
  /// `line` (if given) says whether it was a wall line (false: a corner, a
  /// wall's end, a blob - distance to a point).
  double distance(double x, double y, double * gx = nullptr, double * gy = nullptr, bool * line = nullptr) const;
  double maxDist() const {return max_dist_;}

  /// ROS map_server format <path>.pgm + <path>.yaml, and <path>.walls: the
  /// mean hit of each occupied cell (sub-cell positions). False on I/O error.
  bool save(const std::string & path_no_ext) const;
  bool load(const std::string & path_no_ext);
  /// The mean hit of every occupied cell (where the walls are).
  std::vector<P2> walls() const;
  /// Wall normals of the occupied cells that have one (corners have none).
  std::vector<P2> normals() const;
  /// Occupancy for nav_msgs/OccupancyGrid: 100 occupied, 0 elsewhere.
  std::vector<int8_t> occupancy() const;

private:
  void grow(int ix0, int iy0, int ix1, int iy1);
  int idx(int ix, int iy) const {return iy * w_ + ix;}
  P2 cellPoint(int k) const;
  void computeNormals();

  double res_;
  int min_hits_;
  double max_dist_;
  double ox_{0.0}, oy_{0.0};
  int w_{0}, h_{0};
  std::vector<uint16_t> hits_;
  std::vector<float> sx_, sy_;    // sums of the hits' offsets from the cell corner
  std::vector<int> near_;         // nearest occupied cell, -1 beyond max_dist
  std::vector<float> nx_, ny_;    // wall normal of an occupied cell, 0 = corner / blob
  bool field_valid_{false};
  bool dirty_{false};
};

struct MatchParams
{
  int iterations{12};
  double huber{0.05};         // [m] residuals beyond: weight falls as 1/r
  double outlier{0.3};        // [m] points further from any wall are ignored
  double inlier{0.08};        // [m] counted as on a wall
  double prior_xy{0.10};      // [m] 1 sigma of the prior (the last correction)
  double prior_yaw{0.09};     // [rad]
  double degenerate{10.0};    // walls fixing a direction less than this many priors: keep the prior there
};

struct MatchResult
{
  Pose2 pose;
  int points{0};              // used (within outlier)
  double inlier_fraction{0.0};  // of all points
  double fit{0.0};            // mean over all points of max(0, 1 - d / 10 cm): finer than the inliers
  double rms{0.0};            // of the used points [m]
  // how the inlier walls pin the translation down: mean of n n^T over the
  // inliers (n = wall normal); u^T C u = share of them fixing direction u
  double cxx{0.0}, cxy{0.0}, cyy{0.0};
  double constraint(double ux, double uy) const {return ux * ux * cxx + 2.0 * ux * uy * cxy + uy * uy * cyy;}
  bool ok{false};
};

/// Find T (map <- cloud frame) near `guess` that puts the cloud on the walls.
MatchResult match(const WallGrid & map, const std::vector<P2> & cloud, const Pose2 & guess,
  const MatchParams & p = MatchParams());

struct GlobalParams
{
  double step_xy{0.10};      // [m]
  double step_yaw{0.0524};   // [rad] 3 deg
  double clearance{0.15};    // [m] the robot's centre is at least this far from walls
  int max_points{300};       // the cloud is thinned to about this many
  int refine{8};             // best candidates refined by match()
  double ambiguity{0.9};     // a distinct second best scoring above this share of the best: ambiguous
  // search only round a guess (loop closure, a place recognised): positions
  // within win_xy of `center`, headings within win_yaw of its yaw
  bool window{false};
  Pose2 center;
  double win_xy{1.0};
  double win_yaw{M_PI};
};

struct GlobalResult
{
  MatchResult best;
  double score{0.0};         // fit of the best after refinement (MatchResult::fit)
  double second{0.0};        // fit of the best distinct alternative (> 0.3 m or 15 deg away)
  bool ok{false};
};

/// Where in the map is the cloud? Exhaustive over free positions and headings.
GlobalResult globalSearch(const WallGrid & map, const std::vector<P2> & cloud,
  const GlobalParams & gp = GlobalParams(), const MatchParams & mp = MatchParams());

/// Keep one point per cell of `cell` size (the first seen).
std::vector<P2> thin(const std::vector<P2> & pts, double cell);

}  // namespace dog_perception
