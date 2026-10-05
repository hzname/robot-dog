// Servo-speed limits for the trot: the fastest servo-space speed the gait asks
// of its joints at the extreme commands (peakServoSpeed, D-20).
//
// The peak is a pure core function with no ROS and no state: it runs the same
// procedure as the JointSpeedsFitTheServos test - the trot (TrotGait) at 50 Hz
// on flat ground with the body pose neutral, as the controller does without
// IMU input - and reports the strict maximum over the five extreme commands
// from LocomotionParams::max_velocity. Speeds are expressed in the space of
// the servos: the knee is scaled by ServoSpeedModel::knee_ratio because the
// rod drive turns the knee servo faster than the joint (D-20). The period
// computed from this peak arrives in a later step of this plan; the search
// window guards the non-monotonicity of the peak (the 20 ms tick quantises the
// step phase). Nothing here throws.
#pragma once

namespace dog_control
{

struct LocomotionParams;

/// Comparison tolerance [rad/s]: a peak exactly at the gate passes, 2e-9 above
/// it does not (the verified 0.55 s period fits a 6.35 rad/s servo by 0.003).
constexpr double kSpeedTolerance{1e-9};

/// Assumed servo speed model: `max_speed` is the number the period is
/// computed for (D-13), separate from the physical speed of the simulation.
struct ServoSpeedModel
{
  double max_speed{6.0};   // [rad/s] assumed servo speed, what the period is computed for (D-13)
  double margin{0.8};      // [-] gate D-11: the peak must stay at or below margin * max_speed
  double knee_ratio{1.0};  // [-] servo rad per knee joint rad, worst case of the range, 1 = direct drive (D-20)
};

/// Peak servo-space joint speed: `peak` [rad/s] at the joint `joint_kind`
/// (0 hip, 1 thigh, 2 knee).
struct PeakSpeed
{
  double peak{0.0};   // [rad/s] in the space of the servos
  int joint_kind{0};  // 0 hip, 1 thigh, 2 knee
};

/// Peak servo-space speed of the trot at the five extreme commands from
/// `p.max_velocity`, at the period `p.gait.period`, on flat ground without
/// IMU input (the JointSpeedsFitTheServos procedure at 50 Hz). Returns a zero
/// peak for zero commands; never throws.
PeakSpeed peakServoSpeed(const LocomotionParams & p, const ServoSpeedModel & s);

}  // namespace dog_control
