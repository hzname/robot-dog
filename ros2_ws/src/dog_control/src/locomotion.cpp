#include "dog_control/locomotion.hpp"

#include <cstdio>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

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

// Period in force [s] (D-12): the manual gait.period, or the smallest period
// that fits servo.margin * servo.max_speed up to kMaxAutoPeriod. 0.0 when the
// manual period is not finite and above 0 or nothing fits.
double effectivePeriod(const LocomotionParams & p)
{
  if (!p.auto_period) {
    return std::isfinite(p.gait.period) && p.gait.period > 0.0 ? p.gait.period : 0.0;
  }
  return minimalPeriod(p, p.servo, p.min_period, kMaxAutoPeriod);
}

// Gait params with the effective period; throws std::runtime_error when there
// is none (the constructor fails fast, reconfigureGait refuses).
GaitParams gaitParamsFor(const LocomotionParams & p)
{
  GaitParams g = p.gait;
  g.period = effectivePeriod(p);
  if (g.period == 0.0) {
    std::ostringstream msg;
    if (p.auto_period) {
      msg << "no gait period up to " << kMaxAutoPeriod << " s fits servo.max_speed " << p.servo.max_speed
          << " * servo.margin " << p.servo.margin << " (servo.knee_ratio " << p.servo.knee_ratio
          << ", gait.min_period " << p.min_period << " s): raise servo.max_speed or servo.margin, "
          << "or set gait.auto_period: false and choose gait.period";
    } else {
      msg << "gait.period '" << p.gait.period << "' (use a finite number above 0 s, or gait.auto_period: true)";
    }
    throw std::runtime_error(msg.str());
  }
  return g;
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
    case Mode::SURVEY: return "survey";
    case Mode::FALLEN: return "fallen";
  }
  return "unknown";
}

LocomotionController::LocomotionController(const LocomotionParams & params)
: p_(params), gait_(gaitParamsFor(params), neutralFeet(params)), crawl_(params.crawl, neutralFeet(params)),
  greet_(params.greet, neutralFeet(params), params.stand_height, params.leg.thigh, params.leg.calf),
  survey_(params.survey)
{
  // The tilt governor must be able to interpolate at all: a stop at or below the
  // soft threshold would divide by zero, and past 45 deg the estimate itself is
  // out of its trusted range (slope.max_deg).
  if (!(p_.tilt_soft_deg > 0.0) || !(p_.tilt_stop_deg > p_.tilt_soft_deg) || p_.tilt_stop_deg > 45.0) {
    std::ostringstream msg;
    msg << "tilt.soft_deg " << p_.tilt_soft_deg << " and tilt.stop_deg " << p_.tilt_stop_deg
        << " must satisfy 0 < soft < stop <= 45";
    throw std::invalid_argument(msg.str());
  }
  // Fall protection (phase 4): the levels must nest, and the thermal deadline,
  // when it is on, sits inside the range the servos tolerate.
  if (!(p_.safety_soft_deg > 0.0) || !(p_.safety_hard_deg > p_.safety_soft_deg) ||
    p_.safety_hard_deg > 60.0 || !(p_.safety_debounce_s > 0.0) ||
    !(p_.safety_hysteresis_deg >= 0.0) || !(p_.safety_imu_timeout_s >= 0.0))
  {
    std::ostringstream msg;
    msg << "safety.soft_deg " << p_.safety_soft_deg << " and safety.hard_deg " << p_.safety_hard_deg
        << " must satisfy 0 < soft < hard <= 60, debounce " << p_.safety_debounce_s
        << " > 0, hysteresis >= 0, imu timeout >= 0";
    throw std::invalid_argument(msg.str());
  }
  if (p_.safety_idle_lie_s != 0.0 && (p_.safety_idle_lie_s < 60.0 || p_.safety_idle_lie_s > 120.0)) {
    std::ostringstream msg;
    msg << "safety.idle_lie_s " << p_.safety_idle_lie_s
        << " must be 0 (off) or within 60-120 s: shorter protects nothing, longer cooks the servos";
    throw std::invalid_argument(msg.str());
  }
  height_ = p_.lie_height;
  guard_step_.fill(std::numeric_limits<double>::quiet_NaN());
}

bool LocomotionController::gaitReconfigurable() const
{
  switch (mode_) {
    case Mode::PASSIVE:
    case Mode::STAND:
    case Mode::LYING:
      return true;
    default:
      return false;
  }
}

bool LocomotionController::reconfigureGait(
  double period, bool auto_period, double min_period, const ServoSpeedModel & servo)
{
  if (!gaitReconfigurable()) {
    return false;  // a gait in motion or a transition must not jump
  }
  LocomotionParams q = p_;
  q.gait.period = period;
  q.auto_period = auto_period;
  q.min_period = min_period;
  q.servo = servo;
  const double effective = effectivePeriod(q);
  if (effective == 0.0) {
    return false;
  }
  GaitParams g = q.gait;
  g.period = effective;
  // The rebuilt trot starts at the neutral stance; the per-leg step heights
  // (a leg raised by the guard) are put back so a swing is not reset.
  std::array<double, kNumLegs> heights{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    heights[leg] = gait_.stepHeight(leg);
  }
  gait_ = TrotGait(g, neutralFeet(q));
  for (int leg = 0; leg < kNumLegs; ++leg) {
    gait_.setStepHeight(leg, heights[leg]);
  }
  p_ = q;
  return true;
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
  refusal_.clear();
  if (mode_ == Mode::FALLEN) {
    // Only ~/recover raises a fallen robot: a stray command from a gamepad or a
    // stale web page must not put it back on its feet (SAF-05, D-12).
    refusal_ = "fallen (" + fall_reason_ + "): call ~/recover";
    return false;
  }
  if (estop_) {
    refusal_ = "e-stop engaged";
    return false;
  }
  if (cmd == "stand" && p_.safety_require_imu && !imu_calibrated_) {
    refusal_ = "imu not calibrated: the fall guard would be blind";
    return false;
  }
  idle_t_ = 0.0;  // the operator is here: the thermal timer starts over
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
  if (cmd == "greet" || cmd == "survey") {
    // standing still on level feet in the trot (the sequence starts from the
    // neutral stance); the body pose and the slope shift return to neutral
    const bool still = std::abs(vel_.vx) < 1e-3 && std::abs(vel_.vy) < 1e-3 && std::abs(vel_.wz) < 1e-3;
    if (mode_ != Mode::STAND || gait_type_ != GaitType::TROT || gait_.stepping() || !still || pending_lie_) {
      return false;
    }
    if (cmd == "greet") {
      greet_.start();
      mode_ = Mode::GREETING;
    } else {
      survey_.start();
      mode_ = Mode::SURVEY;
    }
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
    if (mode_ != Mode::FALLEN) {mode_ = Mode::PASSIVE;}  // a fall outranks an e-stop banner
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
  if (std::abs(v.vx) > 1e-6 || std::abs(v.vy) > 1e-6 || std::abs(v.wz) > 1e-6) {
    idle_t_ = 0.0;
  }
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

bool LocomotionController::feetLevel(double tol) const
{
  if (gait_type_ != GaitType::CRAWL) {return true;}
  const auto f = crawl_.feet();
  double lo = f[0].z, hi = f[0].z;
  for (const auto & p : f) {
    lo = std::min(lo, p.z);
    hi = std::max(hi, p.z);
  }
  return hi - lo < tol && (tol >= 0.02 || std::abs(crawl_.pitch()) < 0.01);
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
  if (std::abs(pose.roll) > 1e-6 || std::abs(pose.pitch) > 1e-6 ||
    std::abs(pose.height) > 1e-6)
  {
    idle_t_ = 0.0;
  }
  pose_target_.roll = std::clamp(pose.roll, -p_.max_roll, p_.max_roll);
  pose_target_.pitch = std::clamp(pose.pitch, -p_.max_pitch, p_.max_pitch);
  pose_target_.height = std::clamp(
    pose.height, p_.min_height - p_.stand_height, p_.max_height - p_.stand_height);
}

void LocomotionController::setImuAttitude(double roll, double pitch, double dt)
{
  // Default heartbeat: the filtered attitude. The node overrides it right after
  // with the accelerometer's own direction, which sees a fall while it happens.
  setFallTilt(roll, pitch);
  if (!p_.slope_compensation || dt <= 0.0) {return;}
  const bool upright = mode_ == Mode::STAND || mode_ == Mode::WALK;
  if (!upright) {  // the hold starts from the heading at the next walk
    yaw_turned_ = 0.0;
    yaw_turned_dt_ = 0.0;
    // The legs are about to move under the body: the old window describes a pose
    // that no longer exists, so the next upright sample starts a fresh mean.
    slope_count_ = 0;
    slope_head_ = 0;
    slope_sum_p_ = 0.0;
    slope_sum_r_ = 0.0;
    slope_sum_dt_ = 0.0;
    return;
  }
  const double lim = p_.slope_max_deg * M_PI / 180.0;
  // The body is commanded parallel to the ground plus pose_: what remains is
  // the ground's slope. Only trust it while standing on the legs.
  const double gp = pitch - pose_.pitch;
  const double gr = roll - pose_.roll;
  if (std::abs(gp) > lim || std::abs(gr) > lim) {return;}
  // A time-weighted mean over one gait period. The body pitches with every step
  // and that rocking is periodic, so a plain low-pass only smears it: on a 20 deg
  // ramp the filtered estimate swung 15...19 deg, the mean sat 3 deg low and the
  // tilt governor never reached its stop (GAIT-04). Averaging over exactly one
  // period cancels the periodic part and leaves the slope.
  const int slot = slope_head_;
  slope_buf_p_[slot] = gp;
  slope_buf_r_[slot] = gr;
  slope_buf_dt_[slot] = dt;
  slope_head_ = (slot + 1) % kSlopeSamples;
  if (slope_count_ < kSlopeSamples) {++slope_count_;}
  slope_sum_p_ += gp * dt;
  slope_sum_r_ += gr * dt;
  slope_sum_dt_ += dt;
  // The window follows the gait in force; the crawl's nine seconds would make the
  // estimate uselessly slow, so it is capped by the configured tau.
  const double window = std::clamp(gaitPeriod(), 0.30, 2.0 * p_.slope_filter_tau);
  while (slope_count_ > 1) {
    const int oldest = (slope_head_ - slope_count_ + kSlopeSamples) % kSlopeSamples;
    const double odt = slope_buf_dt_[oldest];
    if (slope_sum_dt_ - odt < window) {break;}
    slope_sum_p_ -= slope_buf_p_[oldest] * odt;
    slope_sum_r_ -= slope_buf_r_[oldest] * odt;
    slope_sum_dt_ -= odt;
    --slope_count_;
  }
  slope_pitch_ = slope_sum_p_ / slope_sum_dt_;
  slope_roll_ = slope_sum_r_ / slope_sum_dt_;
  slope_valid_ = true;
}

void LocomotionController::setFallTilt(double roll, double pitch)
{
  if (!std::isfinite(roll) || !std::isfinite(pitch)) {return;}
  imu_roll_ = roll;
  imu_pitch_ = pitch;
  tilt_deg_ = std::max(std::abs(roll), std::abs(pitch)) * 180.0 / M_PI;
  imu_seen_ = true;
  imu_age_ = 0.0;
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

void LocomotionController::evaluateSafety(double dt)
{
  if (imu_seen_) {imu_age_ += dt;}

  // The lie-down asked for by the thermal timeout was carried out: now the servos
  // may let go (doing it while the robot is still walking would drop it).
  if (idle_lie_pending_ && mode_ == Mode::LYING) {
    idle_lie_pending_ = false;
    relax_requested_ = true;
  }
  if (mode_ == Mode::FALLEN) {return;}

  // 1. Hard tilt: the robot is going over. Count time, not frames: the IMU rate
  //    may differ from the control loop's.
  const bool too_far = imu_seen_ && tilt_deg_ > p_.safety_hard_deg;
  hard_t_ = too_far ? hard_t_ + dt : 0.0;
  if (too_far && hard_t_ >= p_.safety_debounce_s) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "tilt %.0f deg above %.0f deg for %.2f s",
      tilt_deg_, p_.safety_hard_deg, hard_t_);
    enterFallen(buf);
    return;
  }

  // 2. A silent IMU: the fall guard is blind, so the robot must not keep walking.
  if (imu_seen_ && p_.safety_imu_timeout_s > 0.0 && imu_age_ > p_.safety_imu_timeout_s) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "imu silent for %.2f s (timeout %.2f s)",
      imu_age_, p_.safety_imu_timeout_s);
    enterFallen(buf);
    return;
  }

  // 3. Soft tilt: still recoverable - lie down instead of waiting for the fall.
  if (imu_seen_) {
    if (tilt_deg_ > p_.safety_soft_deg) {
      soft_active_ = true;
    } else if (tilt_deg_ < p_.safety_soft_deg - p_.safety_hysteresis_deg) {
      soft_active_ = false;  // hysteresis: no dithering on the threshold
    }
  }
  const bool upright = mode_ == Mode::STAND || mode_ == Mode::WALK ||
    mode_ == Mode::GREETING || mode_ == Mode::SURVEY || mode_ == Mode::STANDING_UP;
  if (soft_active_ && upright) {
    request("lie");
    return;
  }

  // 4. Thermal protection: the servos have no temperature feedback, so holding a
  //    pose for minutes is the only thing that heats them up (SAF-08).
  const bool holding = mode_ == Mode::STAND || mode_ == Mode::WALK ||
    mode_ == Mode::STANDING_UP || mode_ == Mode::LYING_DOWN;
  if (!holding) {
    idle_t_ = 0.0;
    return;
  }
  idle_t_ += dt;
  if (p_.safety_idle_lie_s > 0.0 && idle_t_ > p_.safety_idle_lie_s) {
    idle_t_ = 0.0;
    if (request("lie")) {idle_lie_pending_ = true;}
  }
}

void LocomotionController::enterFallen(const std::string & reason)
{
  mode_ = Mode::FALLEN;
  fall_reason_ = reason;
  pending_lie_ = false;
  vel_ = BodyVelocity{};
  vel_target_ = BodyVelocity{};
  pose_ = BodyPose{};
  pose_target_ = BodyPose{};
  gait_.reset();
  hard_t_ = 0.0;
  relax_requested_ = true;
}

std::string LocomotionController::recover()
{
  if (mode_ != Mode::FALLEN) {return "not fallen";}
  if (imu_seen_ && imu_age_ > p_.safety_imu_timeout_s) {return "imu is still silent";}
  if (p_.safety_require_imu && !imu_calibrated_) {return "imu not calibrated";}
  if (imu_seen_ && tilt_deg_ > p_.safety_soft_deg - p_.safety_hysteresis_deg) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "tilt %.0f deg: put the robot level first", tilt_deg_);
    return buf;
  }
  mode_ = Mode::LYING;
  height_ = p_.lie_height;
  gait_.reset();
  hard_t_ = 0.0;
  soft_active_ = false;
  idle_t_ = 0.0;
  idle_lie_pending_ = false;
  relax_requested_ = false;
  estop_ = false;  // the operator's gesture also clears the e-stop we raised
  fall_reason_.clear();
  refusal_.clear();
  return std::string();
}

bool LocomotionController::takeRelaxRequest()
{
  const bool requested = relax_requested_;
  relax_requested_ = false;
  return requested;
}

bool LocomotionController::update(double dt)
{
  unreachable_ = 0;
  if (dt <= 0.0) {
    return mode_ != Mode::PASSIVE;
  }
  evaluateSafety(dt);
  if (mode_ == Mode::FALLEN) {
    return false;  // the outputs are released, nothing is solved
  }
  const bool upright = mode_ == Mode::STAND || mode_ == Mode::WALK;

  // Accel-limited twist; zero unless upright and not about to lie down.
  BodyVelocity target = (upright && !pending_lie_) ? vel_target_ : BodyVelocity{};
  // Tilt governor (GAIT-04): the filtered ground slope scales the whole twist, so
  // a steep place slows the walk to a stand instead of waiting for the fall. A
  // zero twist in WALK already means standing on planted feet - the gait then
  // holds still, the same as when the operator releases the sticks.
  tilt_scale_ = 1.0;
  if (slope_valid_ && upright) {
    const double tilt = std::max(std::abs(slope_pitch_), std::abs(slope_roll_)) * 180.0 / M_PI;
    if (tilt >= p_.tilt_stop_deg) {
      tilt_scale_ = 0.0;
    } else if (tilt > p_.tilt_soft_deg) {
      tilt_scale_ = (p_.tilt_stop_deg - tilt) / (p_.tilt_stop_deg - p_.tilt_soft_deg);
    }
  }
  target.vx *= tilt_scale_;
  target.vy *= tilt_scale_;
  target.wz *= tilt_scale_;
  target.vx = std::min(target.vx, guard_vx_);  // hazard ahead: forward only
  if (vel_target_.vx > 0.01) {target.vy += guard_vy_;}  // going round: sideways while asked forward
  // Gait change (trot <-> crawl): stop, and switch once the gait is idle
  // with all feet on one level (never between two steps of a staircase).
  const GaitType want = (operator_gait_ == GaitType::CRAWL || guard_gait_ == GaitType::CRAWL) ?
    GaitType::CRAWL : GaitType::TROT;
  switching_gait_ = want != gait_type_ && upright;
  if (switching_gait_) {
    target = BodyVelocity{};
    // Into the trot: feet within 25 mm will do - it walks over that anyway
    // (the guard's own limit for it); the crawl's footholds follow the map,
    // and at an obstacle its smeared edge put the front ones 1-2 cm up, and
    // "within 1 cm" never came. Into the crawl: from the trot's flat stance.
    const double tol = want == GaitType::TROT ? 0.025 : 0.01;
    if (!activeGaitStepping() && feetLevel(tol) &&
      std::abs(vel_.vx) < 1e-3 && std::abs(vel_.vy) < 1e-3 && std::abs(vel_.wz) < 1e-3)
    {
      if (want == GaitType::CRAWL) {
        crawl_.reset(gait_.feet());
      } else {
        // the body's pitch goes on from the crawl's, back to the pose's at its rate
        pose_.pitch += crawl_.pitch();
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
    case Mode::FALLEN:   // unreachable: update() returns early, but the switch is total
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
      // (not while it waits to change gaits: its turns would keep the gait
      // stepping, and it would never come to rest to switch)
      if (p_.heading_hold && yaw_rate_valid_ && moving && !pending_lie_ && !switching_gait_) {
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
        const double correction =
          std::clamp(kp * heading_error_ + ki * heading_integral_, -max_rate, max_rate);
        gait_vel_.wz += correction;
        heading_correction_ = correction;
      } else {
        heading_error_ = 0.0;
        heading_integral_ = 0.0;
        heading_correction_ = 0.0;
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

    case Mode::SURVEY: {
      // feet where they stand; the body pitched and turned on them
      survey_.update(dt);
      const SurveyFrame & f = survey_.frame();
      const double c = std::cos(f.yaw), sn = std::sin(f.yaw);
      const auto feet = gait_.feet();
      std::array<Vec3, kNumLegs> rel{};
      for (int leg = 0; leg < kNumLegs; ++leg) {
        const double x = feet[leg].x + shift_x_, y = feet[leg].y + shift_y_;
        rel[leg] = {c * x + sn * y, -sn * x + c * y, -height_ + feet[leg].z};  // Rz(-yaw)
      }
      BodyPose pose;
      pose.pitch = f.pitch;
      solveRelative(rel, pose);
      if (survey_.done()) {mode_ = Mode::STAND;}
      return true;
    }

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
