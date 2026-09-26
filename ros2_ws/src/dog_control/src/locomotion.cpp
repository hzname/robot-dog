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
    case Mode::GREETING: return "greeting";
  }
  return "unknown";
}

LocomotionController::LocomotionController(const LocomotionParams & params)
: p_(params), gait_(params.gait, neutralFeet(params)), crawl_(params.crawl, neutralFeet(params)),
  greet_(params.greet, neutralFeet(params), params.stand_height, params.leg.thigh, params.leg.calf)
{
  height_ = p_.lie_height;
  guard_step_.fill(std::numeric_limits<double>::quiet_NaN());
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
  gait_type_ = GaitType::TROT;
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
  if (cmd == "crawl" || cmd == "trot") {  // operator's gait; switched when safe
    requestGait(cmd == "crawl" ? GaitType::CRAWL : GaitType::TROT);
    return true;
  }
  if (cmd == "greet") {
    // standing still on level feet in the trot (the sequence starts from the
    // neutral stance); the body pose and the slope shift return to neutral
    const bool still = std::abs(vel_.vx) < 1e-3 && std::abs(vel_.vy) < 1e-3 && std::abs(vel_.wz) < 1e-3;
    if (mode_ != Mode::STAND || gait_type_ != GaitType::TROT || gait_.stepping() || !still || pending_lie_) {
      return false;
    }
    greet_.start();
    mode_ = Mode::GREETING;
    return true;
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
    heading_error_ = 0.0;
    heading_integral_ = 0.0;
    gait_.reset();
    gait_type_ = GaitType::TROT;
  }
}

void LocomotionController::setVelocity(const BodyVelocity & v)
{
  const auto & m = p_.max_velocity;
  vel_target_.vx = std::clamp(v.vx, -m.vx, m.vx);
  vel_target_.vy = std::clamp(v.vy, -m.vy, m.vy);
  vel_target_.wz = std::clamp(v.wz, -m.wz, m.wz);
}

void LocomotionController::setGuard(double max_vx, const std::array<double, kNumLegs> & step_heights)
{
  guard_vx_ = std::isnan(max_vx) ? std::numeric_limits<double>::infinity() : std::max(0.0, max_vx);
  guard_step_ = step_heights;
}

void LocomotionController::setGuardGait(GaitType gait, double vy_bias)
{
  guard_gait_ = gait;
  guard_vy_ = std::isfinite(vy_bias) ? std::clamp(vy_bias, -p_.max_velocity.vy, p_.max_velocity.vy) : 0.0;
}

bool LocomotionController::activeGaitStepping() const
{
  return gait_type_ == GaitType::CRAWL ? crawl_.stepping() : gait_.stepping();
}

bool LocomotionController::feetLevel() const
{
  if (gait_type_ != GaitType::CRAWL) {return true;}
  const auto f = crawl_.feet();
  double lo = f[0].z, hi = f[0].z;
  for (const auto & p : f) {
    lo = std::min(lo, p.z);
    hi = std::max(hi, p.z);
  }
  return hi - lo < 0.01 && std::abs(crawl_.pitch()) < 0.01;
}

std::array<Vec3, kNumLegs> LocomotionController::activeFeet() const
{
  return gait_type_ == GaitType::CRAWL ? crawl_.feet() : gait_.feet();
}

void LocomotionController::clearGuard()
{
  guard_vx_ = std::numeric_limits<double>::infinity();
  guard_step_.fill(std::numeric_limits<double>::quiet_NaN());
  guard_gait_ = GaitType::TROT;
  guard_vy_ = 0.0;
}

void LocomotionController::setBodyPose(const BodyPose & pose)
{
  pose_target_.roll = std::clamp(pose.roll, -p_.max_roll, p_.max_roll);
  pose_target_.pitch = std::clamp(pose.pitch, -p_.max_pitch, p_.max_pitch);
  pose_target_.height = std::clamp(
    pose.height, p_.min_height - p_.stand_height, p_.max_height - p_.stand_height);
}

void LocomotionController::setImuAttitude(double roll, double pitch, double dt)
{
  if (!p_.slope_compensation || dt <= 0.0) {return;}
  const bool upright = mode_ == Mode::STAND || mode_ == Mode::WALK;
  if (!upright) {  // the hold starts from the heading at the next walk
    yaw_turned_ = 0.0;
    yaw_turned_dt_ = 0.0;
  }
  const double lim = p_.slope_max_deg * M_PI / 180.0;
  // The body is commanded parallel to the ground plus pose_: what remains is
  // the ground's slope. Only trust it while standing on the legs.
  const double gp = pitch - pose_.pitch;
  const double gr = roll - pose_.roll;
  if (!upright || std::abs(gp) > lim || std::abs(gr) > lim) {return;}
  const double k = 1.0 - std::exp(-dt / std::max(p_.slope_filter_tau, 1e-3));
  if (!slope_valid_) {
    slope_pitch_ = gp;
    slope_roll_ = gr;
    slope_valid_ = true;
  } else {
    slope_pitch_ += (gp - slope_pitch_) * k;
    slope_roll_ += (gr - slope_roll_) * k;
  }
}

void LocomotionController::setYawRate(double wz)
{
  if (std::isfinite(wz)) {
    yaw_rate_ = wz;
    yaw_rate_valid_ = true;
  }
}

void LocomotionController::addYawRate(double wz, double dt)
{
  if (!std::isfinite(wz)) {return;}
  setYawRate(wz);
  if (std::isfinite(dt) && dt > 0.0) {
    yaw_turned_ += wz * dt;
    yaw_turned_dt_ += dt;
  }
}

bool LocomotionController::update(double dt)
{
  unreachable_ = 0;
  if (dt <= 0.0) {
    return mode_ != Mode::PASSIVE;
  }
  const bool upright = mode_ == Mode::STAND || mode_ == Mode::WALK;

  // Accel-limited twist; zero unless upright and not about to lie down.
  BodyVelocity target = (upright && !pending_lie_) ? vel_target_ : BodyVelocity{};
  target.vx = std::min(target.vx, guard_vx_);  // hazard ahead: forward only
  if (vel_target_.vx > 0.01) {target.vy += guard_vy_;}  // going round: sideways while asked forward
  // Gait change (trot <-> crawl): stop, and switch once the gait is idle
  // with all feet on one level (never between two steps of a staircase).
  const GaitType want = (operator_gait_ == GaitType::CRAWL || guard_gait_ == GaitType::CRAWL) ?
    GaitType::CRAWL : GaitType::TROT;
  if (want != gait_type_ && upright) {
    target = BodyVelocity{};
    if (!activeGaitStepping() && feetLevel() &&
      std::abs(vel_.vx) < 1e-3 && std::abs(vel_.vy) < 1e-3 && std::abs(vel_.wz) < 1e-3)
    {
      if (want == GaitType::CRAWL) {
        crawl_.reset(gait_.feet());
      } else {
        gait_.reset();
      }
      gait_type_ = want;
    }
  }
  // Swing height: towards the guard's value (or the configured one) at 0.1 m/s,
  // so a foot in mid-swing is not jerked up or down.
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const double goal = std::isfinite(guard_step_[leg]) ?
      std::clamp(guard_step_[leg], 0.0, 0.08) : p_.gait.step_height;
    gait_.setStepHeight(leg, approach(gait_.stepHeight(leg), goal, 0.1 * dt));
  }
  vel_.vx = approach(vel_.vx, target.vx, p_.max_accel.vx * dt);
  vel_.vy = approach(vel_.vy, target.vy, p_.max_accel.vy * dt);
  vel_.wz = approach(vel_.wz, target.wz, p_.max_accel.wz * dt);

  // Rate-limited body pose; returns to neutral outside STAND/WALK.
  const BodyPose pose_goal = upright ? pose_target_ : BodyPose{};
  pose_.roll = approach(pose_.roll, pose_goal.roll, p_.pose_rate * dt);
  pose_.pitch = approach(pose_.pitch, pose_goal.pitch, p_.pose_rate * dt);
  pose_.height = approach(pose_.height, pose_goal.height, p_.height_rate * dt);

  // Slope compensation: gravity projects the centre of mass downhill by
  // height * tan(slope); move the feet the same way (body uphill of them).
  // Not in the crawl: it keeps the body over its support triangle in the
  // horizontal frame by itself, pitches the body on stairs on purpose (the
  // IMU would read that as a slope) and puts its feet by the terrain map -
  // shifted feet land centimetres off the footholds it chose.
  double shift_x = 0.0, shift_y = 0.0;
  if (p_.slope_compensation && slope_valid_ && upright && gait_type_ != GaitType::CRAWL) {
    const double m = p_.slope_max_shift;
    const double h = std::max(height_, p_.min_height);
    shift_x = std::clamp(p_.slope_gain * h * std::tan(slope_pitch_), -m, m);
    shift_y = std::clamp(-p_.slope_gain * h * std::tan(slope_roll_), -m, m);
  }
  shift_x_ = approach(shift_x_, shift_x, 0.05 * dt);  // <= 5 cm/s
  shift_y_ = approach(shift_y_, shift_y, 0.05 * dt);

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
      // Heading hold: only while commanded to move or still stepping, so the
      // robot never turns on the spot by itself when it stands.
      gait_vel_ = vel_;
      const bool moving = activeGaitStepping() || std::abs(vel_.vx) > 1e-3 ||
        std::abs(vel_.vy) > 1e-3 || std::abs(vel_.wz) > 1e-3;
      if (p_.heading_hold && yaw_rate_valid_ && moving && !pending_lie_) {
        // measured turn: the integrated samples if there are any, else the last rate
        const double turned = yaw_turned_dt_ > 0.0 ? yaw_turned_ : yaw_rate_ * dt;
        heading_error_ = std::clamp(heading_error_ + vel_.wz * dt - turned,
            -p_.heading_max_error, p_.heading_max_error);
        const bool crawl = gait_type_ == GaitType::CRAWL;
        const double kp = crawl ? p_.heading_crawl_kp : p_.heading_kp;
        const double ki = crawl ? p_.heading_crawl_ki : p_.heading_ki;
        const double max_rate = crawl ? p_.heading_crawl_max_rate : p_.heading_max_rate;
        const double i_max = ki > 0.0 ? max_rate / ki : 0.0;
        // The integral is for slow drift on straight lines; during commanded
        // turns or while catching up a large error it would wind up on the
        // gait's lag and overshoot.
        if (std::abs(vel_.wz) < 0.05 && std::abs(heading_error_) < 0.1) {
          heading_integral_ = std::clamp(heading_integral_ + heading_error_ * dt, -i_max, i_max);
        } else {
          heading_integral_ = 0.0;
        }
        gait_vel_.wz += std::clamp(kp * heading_error_ + ki * heading_integral_, -max_rate, max_rate);
      } else {
        heading_error_ = 0.0;
        heading_integral_ = 0.0;
      }
      yaw_turned_ = 0.0;
      yaw_turned_dt_ = 0.0;
      if (gait_type_ == GaitType::CRAWL) {
        // what the crawl really walks (odometry uses gait_vel_)
        const double vmax = crawl_.maxSpeed(), v = std::hypot(gait_vel_.vx, gait_vel_.vy);
        if (v > vmax) {
          gait_vel_.vx *= vmax / v;
          gait_vel_.vy *= vmax / v;
        }
        gait_vel_.wz = std::clamp(gait_vel_.wz, -0.15, 0.15);
        crawl_.update(dt, gait_vel_, terrain_.valid() ? &terrain_ : nullptr);
        gait_vel_ = crawl_.twist();  // what it walked (it may wait for its support)
      } else {
        gait_.update(dt, gait_vel_);
      }
      mode_ = activeGaitStepping() ? Mode::WALK : Mode::STAND;
      height_ = std::clamp(p_.stand_height + pose_.height, p_.min_height, p_.max_height) + baseHeight();
      if (pending_lie_ && gait_type_ == GaitType::CRAWL && !crawl_.stepping() && feetLevel()) {
        gait_.reset();  // lie down from the trot's stance (level feet only)
        gait_type_ = GaitType::TROT;
      }
      if (pending_lie_ && gait_type_ == GaitType::TROT && !gait_.stepping()) {
        pending_lie_ = false;
        startTransition(Mode::LYING_DOWN, height_, p_.lie_height);
        solve(height_, BodyPose{});
        return true;
      }
      BodyPose pose = pose_;
      if (gait_type_ == GaitType::CRAWL) {pose.pitch += crawl_.pitch();}
      solve(height_, pose);
      return true;
    }

    case Mode::LYING:
      height_ = p_.lie_height;
      solve(height_, BodyPose{});
      return true;

    case Mode::GREETING: {
      greet_.update(dt);
      const GreetFrame & f = greet_.frame();
      std::array<Vec3, kNumLegs> g{};
      for (int leg = 0; leg < kNumLegs; ++leg) {g[leg] = f.feet[leg] - f.body;}
      BodyPose pose;
      pose.pitch = f.pitch;
      solveRelative(g, pose);
      height_ = f.body.z;
      if (greet_.done()) {
        gait_.reset();
        height_ = p_.stand_height;
        mode_ = Mode::STAND;
      }
      return true;
    }
  }
  return false;
}

void LocomotionController::solve(double height, const BodyPose & pose)
{
  // Body orientation R = Ry(pitch) * Rx(roll); feet are expressed in the
  // yaw-aligned ground frame under the body centre and rotated into the body.
  const auto feet = activeFeet();
  std::array<Vec3, kNumLegs> rel{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    rel[leg] = {feet[leg].x + shift_x_, feet[leg].y + shift_y_, -height + feet[leg].z};
  }
  solveRelative(rel, pose);
}

void LocomotionController::solveRelative(const std::array<Vec3, kNumLegs> & rel, const BodyPose & pose)
{
  const double cr = std::cos(pose.roll), sr = std::sin(pose.roll);
  const double cp = std::cos(pose.pitch), sp = std::sin(pose.pitch);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 & g = rel[leg];
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
