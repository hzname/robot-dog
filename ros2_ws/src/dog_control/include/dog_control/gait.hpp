// Periodic trot gait generator.
//
// Produces foot positions in the body frame: x/y on the ground plane and a
// lift height above the ground (z >= 0). Stance feet are integrated opposite
// to the commanded body twist (they stay fixed on the ground); swing feet
// travel on a smooth arc to a Raibert-style touchdown point.
#pragma once

#include <array>

#include "dog_control/kinematics.hpp"

namespace dog_control
{

struct BodyVelocity
{
  double vx{0.0};  // forward [m/s]
  double vy{0.0};  // left [m/s]
  double wz{0.0};  // yaw rate, CCW [rad/s]
};

struct GaitParams
{
  double period{0.55};      // full gait cycle [s]
  double duty{0.65};        // stance fraction of the cycle (0.5 = pure trot)
  double step_height{0.02}; // swing apex above ground [m]
  double max_step{0.06};    // max foot travel during one stance [m]
  // Phase offset per leg (LF, RF, LR, RR). Trot: diagonals in phase.
  std::array<double, kNumLegs> phase_offsets{0.0, 0.5, 0.5, 0.0};
};

class TrotGait
{
public:
  TrotGait(const GaitParams & params, const std::array<Vec3, kNumLegs> & neutral);

  /// Stop stepping and put every foot back to its neutral point.
  void reset();

  /// Advance by dt with the given body twist. Starts stepping when the twist
  /// is non-zero and stops on a touchdown once all feet are back at neutral.
  void update(double dt, const BodyVelocity & cmd);

  /// x, y: foot position in body frame [m]; z: lift above ground [m].
  const std::array<Vec3, kNumLegs> & feet() const {return feet_;}
  bool stepping() const {return stepping_;}
  double phase() const {return phase_;}
  bool inSwing(int leg) const {return in_swing_[leg];}
  const GaitParams & params() const {return params_;}

  static bool isIdle(const BodyVelocity & v);

private:
  GaitParams params_;
  std::array<Vec3, kNumLegs> neutral_;
  std::array<Vec3, kNumLegs> feet_;
  std::array<double, kNumLegs> swing_blend_{};  // progress of the current swing arc
  std::array<bool, kNumLegs> in_swing_{};
  double phase_{0.0};
  bool stepping_{false};
};

}  // namespace dog_control
