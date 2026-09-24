#include "dog_control/locomotion.hpp"

#include <algorithm>
#include <cmath>

namespace dog_control
{

namespace
{
double approach(double current, double target, double max_delta)
{
  return current + std::clamp(target - current, -max_delta, max_delta);
}

double minJerk(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

std::array<Vec3, kNumLegs> neutralFeet(const LocomotionParams & p)
{
  std::array<Vec3, kNumLegs> out{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    out[leg] = {
      legFront(leg) * p.hip_x + p.foot_offset_x,
      legSide(leg) * (p.hip_y + p.leg.hip + p.foot_offset_y),
      0.0};
  }
  return out;
}
}  // namespace

const char * modeName(Mode m)
{
  switch (m) {
    case Mode::PASSIVE: return "passive";
    case Mode::STANDING_UP: return "standing_up";
    case Mode::STAND: return "stand";
    case Mode::WALK: return "walk";
    case Mode::LYING_DOWN: return "lying_down";
    case Mode::LYING: return "lying";
  }
  return "unknown";
}

LocomotionController::LocomotionController(const LocomotionParams & params)
: p_(params), gait_(params.gait, neutralFeet(params))
{
  height_ = p_.lie_height;
}

Vec3 LocomotionController::hipPosition(int leg) const
{
  return {legFront(leg) * p_.hip_x, legSide(leg) * p_.hip_y, 0.0};
}

Vec3 LocomotionController::neutralFoot(int leg) const
{
  return neutralFeet(p_)[leg];
}

void LocomotionController::startTransition(Mode next, double from_height, double to_height)
{
  mode_ = next;
  trans_t_ = 0.0;
  trans_from_ = from_height;
  trans_to_ = to_height;
  gait_.reset();
}

bool LocomotionController::request(const std::string & cmd)
{
  if (estop_) {
    return false;
  }
  if (cmd == "stand") {
    pending_lie_ = false;
    switch (mode_) {
      case Mode::PASSIVE:
      case Mode::LYING:
        startTransition(Mode::STANDING_UP, p_.lie_height, p_.stand_height);
        return true;
      case Mode::LYING_DOWN:
        startTransition(Mode::STANDING_UP, height_, p_.stand_height);
        return true;
      default:
        return true;  // already up
    }
  }
  if (cmd == "lie") {
    switch (mode_) {
      case Mode::PASSIVE:
        mode_ = Mode::LYING;
        height_ = p_.lie_height;
        gait_.reset();
        return true;
      case Mode::STAND:
      case Mode::STANDING_UP:
        startTransition(Mode::LYING_DOWN, height_, p_.lie_height);
        return true;
      case Mode::WALK:
        pending_lie_ = true;  // finish the steps first
        return true;
      default:
        return true;
    }
  }
  return false;
}

void LocomotionController::setEstop(bool active)
{
  estop_ = active;
  if (active) {
    mode_ = Mode::PASSIVE;
    pending_lie_ = false;
    vel_ = BodyVelocity{};
    vel_target_ = BodyVelocity{};
    pose_ = BodyPose{};
    gait_.reset();
  }
}

void LocomotionController::setVelocity(const BodyVelocity & v)
{
  const auto & m = p_.max_velocity;
  vel_target_.vx = std::clamp(v.vx, -m.vx, m.vx);
  vel_target_.vy = std::clamp(v.vy, -m.vy, m.vy);
  vel_target_.wz = std::clamp(v.wz, -m.wz, m.wz);
}

void LocomotionController::setBodyPose(const BodyPose & pose)
{
  pose_target_.roll = std::clamp(pose.roll, -p_.max_roll, p_.max_roll);
  pose_target_.pitch = std::clamp(pose.pitch, -p_.max_pitch, p_.max_pitch);
  pose_target_.height = std::clamp(
    pose.height, p_.min_height - p_.stand_height, p_.max_height - p_.stand_height);
}

bool LocomotionController::update(double dt)
{
  unreachable_ = 0;
  if (dt <= 0.0) {
    return mode_ != Mode::PASSIVE;
  }
  const bool upright = mode_ == Mode::STAND || mode_ == Mode::WALK;

  // Accel-limited twist; zero unless upright and not about to lie down.
  const BodyVelocity target = (upright && !pending_lie_) ? vel_target_ : BodyVelocity{};
  vel_.vx = approach(vel_.vx, target.vx, p_.max_accel.vx * dt);
  vel_.vy = approach(vel_.vy, target.vy, p_.max_accel.vy * dt);
  vel_.wz = approach(vel_.wz, target.wz, p_.max_accel.wz * dt);

  // Rate-limited body pose; returns to neutral outside STAND/WALK.
  const BodyPose pose_goal = upright ? pose_target_ : BodyPose{};
  pose_.roll = approach(pose_.roll, pose_goal.roll, p_.pose_rate * dt);
  pose_.pitch = approach(pose_.pitch, pose_goal.pitch, p_.pose_rate * dt);
  pose_.height = approach(pose_.height, pose_goal.height, p_.height_rate * dt);

  switch (mode_) {
    case Mode::PASSIVE:
      return false;

    case Mode::STANDING_UP:
    case Mode::LYING_DOWN: {
      const double span = std::max(std::abs(p_.stand_height - p_.lie_height), 1e-3);
      const double duration =
        std::max(0.2, p_.transition_time * std::abs(trans_to_ - trans_from_) / span);
      trans_t_ += dt;
      const double s = trans_t_ / duration;
      height_ = trans_from_ + (trans_to_ - trans_from_) * minJerk(s);
      if (s >= 1.0) {
        height_ = trans_to_;
        mode_ = (mode_ == Mode::STANDING_UP) ? Mode::STAND : Mode::LYING;
      }
      solve(height_, BodyPose{});
      return true;
    }

    case Mode::STAND:
    case Mode::WALK: {
      gait_.update(dt, vel_);
      mode_ = gait_.stepping() ? Mode::WALK : Mode::STAND;
      height_ = std::clamp(p_.stand_height + pose_.height, p_.min_height, p_.max_height);
      if (pending_lie_ && !gait_.stepping()) {
        pending_lie_ = false;
        startTransition(Mode::LYING_DOWN, height_, p_.lie_height);
        solve(height_, BodyPose{});
        return true;
      }
      solve(height_, pose_);
      return true;
    }

    case Mode::LYING:
      height_ = p_.lie_height;
      solve(height_, BodyPose{});
      return true;
  }
  return false;
}

void LocomotionController::solve(double height, const BodyPose & pose)
{
  // Body orientation R = Ry(pitch) * Rx(roll); feet are expressed in the
  // yaw-aligned ground frame under the body centre and rotated into the body.
  const double cr = std::cos(pose.roll), sr = std::sin(pose.roll);
  const double cp = std::cos(pose.pitch), sp = std::sin(pose.pitch);
  const auto & feet = gait_.feet();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 g{feet[leg].x, feet[leg].y, -height + feet[leg].z};
    // R^T * g
    const Vec3 b{
      cp * g.x - sp * g.z,
      sr * sp * g.x + cr * g.y + sr * cp * g.z,
      cr * sp * g.x - sr * g.y + cr * cp * g.z};
    const IkResult ik = inverseKinematics(p_.leg, legSide(leg), b - hipPosition(leg), p_.knee_direction);
    if (!ik.reachable) {
      ++unreachable_;
    }
    for (int j = 0; j < 3; ++j) {
      joints_[leg * 3 + j] = ik.q[j];
    }
  }
}

}  // namespace dog_control
