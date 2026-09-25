// Locomotion state machine: turns operator commands (stand / lie / twist /
// body pose / e-stop) into 12 joint targets.
//
//   PASSIVE --stand--> STANDING_UP --> STAND <--twist--> WALK
//   STAND/WALK --lie--> LYING_DOWN --> LYING --stand--> STANDING_UP
//   any --estop--> PASSIVE (no joint output until "stand" after release)
#pragma once

#include <array>
#include <limits>
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

  // Slope compensation from the IMU: feet shift downhill by
  // gain * height * tan(slope) so the centre of mass stays over the support.
  bool slope_compensation{true};
  double slope_gain{1.0};
  double slope_filter_tau{0.8};  // [s] averages out the gait's own rocking
  double slope_max_shift{0.04};  // [m]
  double slope_max_deg{25.0};    // ignore readings beyond this (robot falling / lifted)

  // Heading hold from the gyro: the heading error (commanded minus measured
  // yaw, integrated while walking) is fed back (PI) into the yaw rate of the gait.
  bool heading_hold{true};
  double heading_kp{2.5};         // [1/s] yaw-rate correction per radian of error
  double heading_ki{1.0};         // [1/s^2] integral part: removes the steady drift offset
  double heading_max_rate{0.3};   // [rad/s] correction limit
  double heading_max_error{0.5};  // [rad] error clamp (anti-windup: robot blocked)

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
  /// Body attitude from the IMU [rad] (REP-103 roll / pitch vs. gravity).
  /// Without calls the controller behaves as on flat ground.
  void setImuAttitude(double roll, double pitch, double dt);
  double slopePitch() const {return slope_pitch_;}
  double slopeRoll() const {return slope_roll_;}
  /// Measured body yaw rate [rad/s] (gyro z). Call every IMU sample;
  /// clearYawRate() when the IMU goes silent. Without it there is no hold.
  void setYawRate(double wz);
  void clearYawRate() {yaw_rate_valid_ = false;}
  /// Integrated heading error [rad] (commanded minus measured).
  double headingError() const {return heading_error_;}
  /// Hazard guard from perception: forward speed limit [m/s] (inf = none)
  /// and swing height per leg (LF, RF, LR, RR) [m] (NaN = gait.step_height):
  /// only the legs whose path crosses the obstacle lift higher. clearGuard()
  /// when the guard goes silent. Backwards, sideways and turning stay free.
  void setGuard(double max_vx, const std::array<double, kNumLegs> & step_heights);
  void clearGuard();
  double guardMaxVx() const {return guard_vx_;}
  double stepHeight(int leg) const {return gait_.stepHeight(leg);}
  /// Twist actually given to the gait (command + heading correction).
  const BodyVelocity & gaitVelocity() const {return gait_vel_;}

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
  double slope_pitch_{0.0};  // estimated ground slope in the body frame [rad]
  double slope_roll_{0.0};
  bool slope_valid_{false};
  double shift_x_{0.0};  // applied (rate-limited) slope shift [m]
  double shift_y_{0.0};
  double yaw_rate_{0.0};
  bool yaw_rate_valid_{false};
  double heading_error_{0.0};
  double heading_integral_{0.0};
  BodyVelocity gait_vel_;
  double guard_vx_{std::numeric_limits<double>::infinity()};
  std::array<double, kNumLegs> guard_step_{};  // NaN = configured step height

  std::array<double, kNumJoints> joints_{};
  int unreachable_{0};
};

}  // namespace dog_control
