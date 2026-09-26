// Crawl gait: statically stable walk with three feet on the ground at any
// time, for terrain the trot cannot take - a step up or down of 25-70 mm, a
// staircase, a bar too high to step over in the trot.
//
// One leg swings at a time (LR, LF, RR, RF). Before a leg lifts, the body
// shifts over the centroid of the triangle of the other three feet, so the
// centre of mass stays ~4-5 cm inside the support. The swing goes straight
// up to clear the highest terrain between the old and the new foothold,
// across, and straight down onto the terrain height at the new foothold.
// Footholds on an edge are moved to flat ground nearby. The body height
// follows the mean height of the feet, so the body climbs with the steps.
//
// Terrain comes as height profiles along the left and right foot lines
// (perception: elevation map); without it the gait walks as on flat ground.
#pragma once

#include <array>
#include <vector>

#include "dog_control/gait.hpp"
#include "dog_control/kinematics.hpp"

namespace dog_control
{

/// Ground heights along the two foot lines, in the body frame: sample i is
/// at x = x0 + i * dx. Heights in any fixed vertical frame (odom): the gait
/// only uses differences. NaN = unknown.
struct TerrainProfile
{
  double x0{0.0};
  double dx{0.02};
  std::vector<double> left, right;
  bool valid() const {return !left.empty() && left.size() == right.size();}
  /// Height at body x on the foot line on side y (> 0 left), NaN if unknown.
  double height(double y, double x) const;
  /// Highest known height between xa and xb on that line (NaN if none known).
  double highest(double y, double xa, double xb) const;
  /// Unknown runs between two known heights take the lower of the two: the
  /// lidars' shadow behind a bar is the floor it stands on, the one beyond a
  /// drop the lower tread (so the edge stays where it is). Unknown at the
  /// ends stays unknown.
  void fillGaps();
};

struct CrawlParams
{
  double shift_time{0.45};   // body shift before each leg lifts [s]
  double swing_time{0.65};   // one leg in the air [s]
  double max_stride{0.10};   // foot travel per full cycle [m]
  double clearance{0.03};    // swing clears the terrain by this [m]
  double max_lift{0.10};     // highest swing above the higher of the two footholds [m]
  double edge{0.008};        // height change within +-4.5 cm that makes a foothold an edge [m]
  double shift_rate{0.06};   // body shift speed [m/s]: slow, see robot.yaml
  double shift_accel{0.3};   // [m/s^2]: a jerk of the whole body slides the feet
  double max_pitch{0.15};    // body pitched along the stairs up to this [rad]
  double pitch_rate{0.15};   // [rad/s]
  double margin{0.035};      // body at least this far inside the support triangle [m]
};

class CrawlGait
{
public:
  CrawlGait(const CrawlParams & params, const std::array<Vec3, kNumLegs> & neutral);

  /// Start from these feet (body frame x, y; z ignored - the ground under
  /// the feet becomes height 0).
  void reset(const std::array<Vec3, kNumLegs> & feet);
  void update(double dt, const BodyVelocity & cmd, const TerrainProfile * terrain = nullptr);

  /// Feet in the body frame: x, y (with the body shift); z = foot height
  /// above the ground level the gait started on.
  std::array<Vec3, kNumLegs> feet() const;
  /// Height of the support under the body above the start level: the body
  /// is held stand_height above it.
  double baseHeight() const {return base_z_;}
  /// Body pitch along the footholds (REP-103: + = nose down): on stairs the
  /// body follows their slope so that the front and the rear legs stay near
  /// their working height (a level body would fold the upper legs and
  /// stretch the lower ones to their reach).
  double pitch() const {return pitch_;}
  bool stepping() const {return stepping_;}
  int swingLeg() const {return phase_ == Phase::SWING ? order_[k_] : -1;}
  /// Where the swinging foot goes (ground frame: body frame plus the shift).
  const Vec3 & swingTarget() const {return to_;}
  /// Twist the body really walked in the last update (clamped to the gait's
  /// speed, zero while it waits for the support) - for odometry.
  const BodyVelocity & twist() const {return twist_;}
  /// Forward speed this gait can walk [m/s].
  double maxSpeed() const;
  /// Body shift (x, y) over the support [m].
  Vec3 shift() const {return shift_;}

private:
  enum class Phase { SHIFT, SWING };
  Vec3 supportCentroid(int lifted) const;
  /// Where the body goes before `lifted` lifts: no further from where it is
  /// than needed to stand margin inside the other three feet, also at the
  /// end of the swing (the body walks on meanwhile). Each shift makes all
  /// four servos move at once, and their lag drags the feet.
  Vec3 shiftTarget(int lifted, const BodyVelocity & cmd) const;
  double supportMargin(int lifted, const Vec3 & p) const;
  void moveShift(const Vec3 & target, double dt);
  /// Nearest spot to x off any edge, searching first in the walking direction dir (+1 / -1),
  /// at most max_ahead further that way.
  double footholdX(double y, double x, double dir, double max_ahead, const TerrainProfile * t) const;
  void startSwing(const TerrainProfile * t, const BodyVelocity & cmd);

  CrawlParams p_;
  std::array<Vec3, kNumLegs> neutral_;
  std::array<Vec3, kNumLegs> ground_{};  // body frame without shift; z above start level
  std::array<int, kNumLegs> order_{LR, LF, RR, RF};
  Vec3 shift_{};
  Vec3 shift_v_{};
  BodyVelocity twist_{};
  double shift_duration_{0.0};
  double swing_T_{0.65};
  std::array<double, kNumLegs> phase_time_{};  // last (shift + swing) per leg [s]
  double phase_clock_{0.0};  // this swing's duration (longer for a high lift)
  Vec3 target_{};
  bool have_target_{false};
  Phase phase_{Phase::SHIFT};
  int k_{0};
  double t_{0.0};
  bool stepping_{false};
  bool have_ref_{false};
  double ref_{0.0};  // terrain height of the start level
  double base_z_{0.0};
  double pitch_{0.0};
  // current swing
  Vec3 from_{}, to_{};
  double clear_z_{0.0};
};

}  // namespace dog_control
