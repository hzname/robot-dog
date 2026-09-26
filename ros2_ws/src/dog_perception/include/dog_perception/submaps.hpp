// A map of many rooms as submaps joined by a pose graph (docs/LOCALIZATION.md).
// Pure geometry, no ROS.
//
// One big grid remembers everything and bends nowhere: once dead reckoning's
// drift has crept into it, it stays. Here the memory is split:
//  - submaps: small wall grids (a few metres of walking each) in their own
//    frames - precise locally, like the single-room map;
//  - a pose graph: where each submap's frame is, and how the neighbours sit
//    to one another (from scan matching while walking);
//  - places: a compact fingerprint of each submap's surroundings (a Scan
//    Context: wall presence in rings x sectors round its anchor), to
//    recognise a place seen before - "I have been here" - without searching
//    the whole map.
// When a finished submap is recognised against an old one and the two are
// matched, the loop edge goes into the graph, the graph is optimised, and
// every submap moves with its node: the drift gathered round the loop is
// spread over it. The grid used for matching (merged) is rebuilt from the
// submaps at their new poses.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dog_perception/localization.hpp"

namespace dog_perception
{

// ------------------------------------------------------------------ graph
struct GraphEdge
{
  int a{0}, b{0};
  Pose2 z;                 // b in a's frame, as measured
  double sigma_xy{0.05};   // [m]
  double sigma_yaw{0.02};  // [rad]
  // a direction (in a's frame) the walls did not pin down - along a bare
  // corridor: there the translation is only as good as dead reckoning
  double weak_dir{0.0};    // [rad]
  double sigma_weak{0.0};  // [m], 0 = none (sigma_xy both ways)
  bool loop{false};
};

/// The direction the walls with these normals pin down least (angle), and
/// how much less (smallest / largest eigenvalue of sum n n^T, 0..1).
double weakDirection(const std::vector<P2> & normals, double * ratio);

/// Gauss-Newton over the node poses; node 0 stays fixed. Returns the final
/// sum of squared (whitened) residuals.
double optimizePoseGraph(std::vector<Pose2> & nodes, const std::vector<GraphEdge> & edges, int iterations = 10);

// ------------------------------------------------------------------ places
struct ScanContextParams
{
  int rings{16};
  int sectors{60};
  double r_max{5.0};  // [m]
};

/// Wall presence in rings x sectors round the origin of the points' frame.
std::vector<float> scanContext(const std::vector<P2> & pts, const ScanContextParams & p = ScanContextParams());

/// Distance in [0, 1] (0 = the same) over all turns of b; `yaw` gets the turn
/// that puts b's frame onto a's.
double scanContextDistance(const std::vector<float> & a, const std::vector<float> & b,
  const ScanContextParams & p, double * yaw = nullptr);

// ------------------------------------------------------------------ submaps
struct SubmapParams
{
  double resolution{0.05};
  int min_hits{2};
  double length{2.5};             // [m] of walking per submap
  bool loop_closure{true};
  double loop_radius{3.0};        // [m] + 10 % of the way walked since: old submaps this near are candidates
  int loop_skip{2};               // the last submaps before this one are its neighbours, not loops
  int local_submaps{3};           // mapping matches against this many latest submaps
  double loop_min_inliers{0.55};  // share of the submap's walls that must fall on the old one's
  double loop_win_xy{1.0};        // [m] search window round the graph's guess
  double loop_win_yaw{0.35};      // [rad]
  double odom_sigma_xy{0.03};     // [m] per edge between neighbours, + 1 % of its length
  double odom_scale{0.15};        // dead reckoning's error share along what the walls do not fix
  double odom_sigma_yaw{0.017};   // [rad]
  double loop_sigma_xy{0.03};
  double loop_sigma_yaw{0.017};
  ScanContextParams place;
};

struct Submap
{
  WallGrid grid;               // in the submap's frame
  Pose2 pose;                  // its frame in the map (the graph node)
  std::vector<float> place;    // scan context of the walls round its origin
  double walked{0.0};          // [m] of walking when it was started
  int scans{0}, unpinned{0};   // scans inserted, those the walls did not fix along the way
  bool finished{false};
};

struct LoopClosure
{
  int from{0}, to{0};          // old submap, the one just finished
  double inliers{0.0};
  double moved_m{0.0};         // how far the newest submap's frame moved
  double moved_yaw{0.0};
};

class SubmapMap
{
public:
  explicit SubmapMap(const SubmapParams & p = SubmapParams());

  const SubmapParams & params() const {return p_;}
  const std::vector<Submap> & submaps() const {return subs_;}
  const std::vector<GraphEdge> & edges() const {return edges_;}
  const std::vector<LoopClosure> & loops() const {return loops_;}
  bool empty() const {return subs_.empty();}

  /// Every submap's walls at its current pose: to localize in a stored map.
  const WallGrid & merged() const {return merged_;}
  /// The last few submaps only: what mapping matches against. Coming back
  /// to old ground with the drift of a loop, the old walls would catch the
  /// robot in the wrong place; going back is the loop closure's job.
  const WallGrid & local() const {return local_;}
  /// Rebuild merged() if walls were added since (cheap: call it at a few Hz).
  void refresh();

  /// Mapping: add the points (map frame) seen with the robot at `robot`
  /// (map frame) after walking `walked` metres in all. Starts a new submap
  /// when the current one is long enough; a finished submap is looked for
  /// among the old ones (loop closure). Returns the correction to apply to
  /// the robot's pose (new = correction * old) - identity unless a loop moved
  /// the current submap.
  /// `pinned`: did the walls fix the robot along its way in this scan's
  /// match (false along a bare corridor) - how much the edge to the next
  /// submap may stretch.
  Pose2 insert(const std::vector<P2> & pts_map, const Pose2 & robot, double walked, bool pinned = true);

  /// Where is a robot whose recent points (in its own frame, centred on it)
  /// are `cloud`? Places first (Scan Context), then a windowed search round
  /// the best ones; the whole map if no place fits.
  GlobalResult relocalize(const std::vector<P2> & cloud, const GlobalParams & gp = GlobalParams(),
    const MatchParams & mp = MatchParams()) const;

  /// <path>.graph (text: submaps and edges), <path>.sub<k>.pgm/.yaml/.walls,
  /// and <path>.pgm/.yaml/.walls: the merged map (for viewers).
  bool save(const std::string & path) const;
  /// A map saved by save(), or a single-grid map (<path>.yaml) as one submap.
  bool load(const std::string & path);

private:
  void finishCurrent();
  std::optional<LoopClosure> closeLoop(int k);
  std::vector<P2> wallsOf(const Submap & s) const;  // occupied cell means, submap frame
  void rebuildMerged();
  void rebuildLocal();

  SubmapParams p_;
  std::vector<Submap> subs_;
  std::vector<GraphEdge> edges_;
  std::vector<LoopClosure> loops_;
  WallGrid merged_, local_;
  bool merged_dirty_{false};
};

}  // namespace dog_perception
