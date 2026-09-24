// Locomotion state machine: turns operator commands (stand / lie / twist /
// body pose / e-stop) into 12 joint targets.
//
//   PASSIVE --stand--> STANDING_UP --> STAND <--twist--> WALK
//   STAND/WALK --lie--> LYING_DOWN --> LYING --stand--> STANDING_UP
//   any --estop--> PASSIVE (no joint output until "stand" after release)
#pragma once

#include <array>
#include <string>

#include "dog_control/gait.hpp"
#include "dog_control/kinematics.hpp"

namespace dog_control
{

enum class Mode { PASSIVE, STANDING_UP, STAND, WALK, LYING_DOWN, LYING };

const char * modeName(Mode m);

struct BodyPose
{
  double roll{0.0};    // [rad]
  double pitch{0.0};   // [rad], positive = nose down (REP-103)
  double height{0.0};  // offset from stand height [m]
};

struct LocomotionParams
{
  LegGeometry leg;
  double hip_x{0.09};          // hip abduction axis from body centre, forward [m]
  double hip_y{0.06};          // hip abduction axis from body centre, lateral [m]
  double foot_offset_x{0.0};   // neutral foot shift forward from under the hip [m]
  double foot_offset_y{0.0};   // neutral foot shift outwards beyond the hip link [m]
  int knee_direction{-1};      // -1 knee points backwards, +1 forwards

  double stand_height{0.15};   // hip axis above ground when standing [m]
  double lie_height{0.08};     // ... when lying [m]
  double min_height{0.10};     // stand height range for body pose control [m]
  double max_height{0.18};
  double transition_time{1.5}; // stand-up / lie-down duration [s]

  double max_roll{0.26};       // body pose limits [rad]
  double max_pitch{0.26};
  double pose_rate{0.5};       // roll/pitch slew [rad/s]
  double height_rate{0.05};    // height slew [m/s]

  BodyVelocity max_velocity{0.15, 0.08, 0.6};
  BodyVelocity max_accel{0.5, 0.3, 2.0};

  GaitParams gait;
};

class LocomotionController
{
public:
  explicit LocomotionController(const LocomotionParams & params);

  /// Mode request: "stand" or "lie". Returns false if rejected.
  bool request(const std::string & cmd);
  void setEstop(bool active);
  bool estopActive() const {return estop_;}

  /// Target twist; clamped to max_velocity and accel-limited internally.
  void setVelocity(const BodyVelocity & v);
  void setBodyPose(const BodyPose & pose);

  /// Advance the controller. Returns true when joints() should be sent.
  bool update(double dt);

  Mode mode() const {return mode_;}
  const std::array<double, kNumJoints> & joints() const {return joints_;}
  const BodyVelocity & velocity() const {return vel_;}
  const TrotGait & gait() const {return gait_;}
  /// Number of IK targets clamped to the workspace in the last update.
  int unreachableCount() const {return unreachable_;}

  /// Hip axis position of a leg in the body frame.
  Vec3 hipPosition(int leg) const;
  /// Neutral foot (x, y) in the body frame.
  Vec3 neutralFoot(int leg) const;

private:
  void solve(double height, const BodyPose & pose);
  void startTransition(Mode next, double from_height, double to_height);

  LocomotionParams p_;
  TrotGait gait_;
  Mode mode_{Mode::PASSIVE};
  bool estop_{false};
  bool pending_lie_{false};

  BodyVelocity vel_target_;
  BodyVelocity vel_;
  BodyPose pose_target_;
  BodyPose pose_;

  double trans_t_{0.0};
  double trans_from_{0.0};
  double trans_to_{0.0};
  double height_{0.0};

  std::array<double, kNumJoints> joints_{};
  int unreachable_{0};
};

}  // namespace dog_control
