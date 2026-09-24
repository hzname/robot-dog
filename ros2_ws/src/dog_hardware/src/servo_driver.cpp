#include "dog_hardware/servo_driver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace dog_hardware
{

namespace
{
constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double wrapDeg(double a)
{
  while (a > 180.0) {a -= 360.0;}
  while (a <= -180.0) {a += 360.0;}
  return a;
}

/// Lever angle [rad] at the joint for a servo arm angle theta [rad]
/// (servo axis at the origin, joint axis at (d, 0)); branch = +-1.
double leverAngle(const Linkage & l, double theta, int branch)
{
  const double ax = l.a() * std::cos(theta), ay = l.a() * std::sin(theta);
  const double dx = l.d() - ax, dy = -ay;
  const double r = std::hypot(dx, dy);
  if (r < 1e-9) {return kNaN;}
  const double k = (l.c() * l.c() - r * r - l.b() * l.b()) / (2.0 * l.b());
  const double ratio = k / r;
  if (ratio < -1.0 || ratio > 1.0) {return kNaN;}  // rod too short / too long here
  return std::atan2(dy, dx) + branch * std::acos(ratio);
}

/// The assembly branch: the one whose lever is closest to the servo arm
/// direction at centre (a parallelogram gives exactly 90 deg).
int assemblyBranch(const Linkage & l)
{
  const double t = M_PI / 2.0;
  const double p = leverAngle(l, t, 1), m = leverAngle(l, t, -1);
  if (std::isnan(m)) {return 1;}
  if (std::isnan(p)) {return -1;}
  return std::abs(wrapDeg((p - t) * kDegPerRad)) <= std::abs(wrapDeg((m - t) * kDegPerRad)) ? 1 : -1;
}

/// Servo angle [deg] from centre that turns the joint by `delta` [deg];
/// clamped to the linkage's usable servo range.
double servoForDelta(const Linkage & l, double delta, double half, bool * clamped)
{
  auto [lo, hi] = l.usableServoRange(half);
  const double glo = l.jointDelta(lo), ghi = l.jointDelta(hi);
  const bool increasing = ghi > glo;
  if ((increasing && delta <= glo) || (!increasing && delta >= glo)) {*clamped = true; return lo;}
  if ((increasing && delta >= ghi) || (!increasing && delta <= ghi)) {*clamped = true; return hi;}
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lo + hi);
    const double g = l.jointDelta(mid);
    if ((g < delta) == increasing) {lo = mid;} else {hi = mid;}
  }
  return 0.5 * (lo + hi);
}
}  // namespace

// --------------------------------------------------------------- Linkage

double Linkage::jointDelta(double servo_deg) const
{
  if (direct()) {return servo_deg;}
  const int br = assemblyBranch(*this);
  const double t0 = M_PI / 2.0;
  const double p0 = leverAngle(*this, t0, br);
  const double p = leverAngle(*this, t0 + servo_deg / kDegPerRad, br);
  if (std::isnan(p0) || std::isnan(p)) {return kNaN;}
  return wrapDeg((p - p0) * kDegPerRad);
}

std::pair<double, double> Linkage::usableServoRange(double half_range_deg) const
{
  if (direct()) {return {-half_range_deg, half_range_deg};}
  constexpr double kStep = 0.5;
  const double g0 = jointDelta(0.0);
  if (std::isnan(g0)) {return {0.0, 0.0};}
  const double gp = jointDelta(kStep);
  const int sense = std::isnan(gp) ? 0 : (gp > g0 ? 1 : (gp < g0 ? -1 : 0));
  if (sense == 0) {return {0.0, 0.0};}
  auto walk = [&](int dir) {
    double s = 0.0, prev = g0;
    while (std::abs(s) + kStep <= half_range_deg + 1e-9) {
      const double next = s + dir * kStep;
      const double g = jointDelta(next);
      if (std::isnan(g) || (g - prev) * sense * dir <= 0.0) {break;}
      s = next;
      prev = g;
    }
    return s;
  };
  return {walk(-1), walk(1)};
}

std::string Linkage::validate(double half_range_deg) const
{
  if (direct()) {return {};}
  if (axis_distance_mm <= 0.0) {return "linkage needs axis_distance_mm > 0";}
  if (joint_arm_mm < 0.0 || rod_mm < 0.0) {return "linkage lengths must be >= 0";}
  if (std::isnan(jointDelta(0.0))) {
    return "rod cannot close the loop at the servo centre: check servo_arm_mm / joint_arm_mm / "
           "rod_mm / axis_distance_mm";
  }
  const auto [lo, hi] = usableServoRange(half_range_deg);
  if (hi - lo < 40.0) {
    return "linkage gives only " + std::to_string(static_cast<int>(hi - lo)) +
           " deg of usable servo travel before a dead point (need >= 40)";
  }
  return {};
}

// ------------------------------------------------------ ServoCalibration

std::string ServoCalibration::validate() const
{
  if (channel < 0 || channel >= kPwmChannels) {return "channel must be 0..15";}
  if (direction != 1 && direction != -1) {return "direction must be 1 or -1";}
  if (pulse_min_us < 300.0 || pulse_max_us > 2800.0 || pulse_min_us >= pulse_max_us) {
    return "pulse range must satisfy 300 <= min < max <= 2800 us";
  }
  if (range_deg <= 0.0 || range_deg > 360.0) {return "range_deg must be in (0, 360]";}
  if (min_deg >= max_deg) {return "min_deg must be < max_deg";}
  if (!std::isfinite(coupling)) {return "coupling must be finite";}
  return linkage.validate(0.5 * range_deg);
}

double jointToPulseUs(const ServoCalibration & cal, double joint_rad, bool * clamped, double parent_rad)
{
  const double q_deg = joint_rad * kDegPerRad;
  const double q = std::clamp(q_deg, cal.min_deg, cal.max_deg);
  bool c = q != q_deg;
  const double parent = cal.coupled_to >= 0 ? parent_rad * kDegPerRad : 0.0;
  const double delta = cal.direction * (q + cal.coupling * parent - cal.offset_deg);
  const double half = 0.5 * cal.range_deg;
  double servo_deg = 0.0;
  if (cal.linkage.direct()) {
    servo_deg = delta;
    if (servo_deg > half || servo_deg < -half) {
      servo_deg = std::clamp(servo_deg, -half, half);
      c = true;
    }
  } else {
    servo_deg = servoForDelta(cal.linkage, delta, half, &c);
  }
  if (clamped) {*clamped = c;}
  return cal.centerUs() + servo_deg * cal.usPerDeg();
}

double pulseUsToJoint(const ServoCalibration & cal, double us, double parent_rad)
{
  const double servo_deg = (us - cal.centerUs()) / cal.usPerDeg();
  const double delta = cal.linkage.direct() ? servo_deg : cal.linkage.jointDelta(servo_deg);
  const double parent = cal.coupled_to >= 0 ? parent_rad * kDegPerRad : 0.0;
  return (cal.direction * delta + cal.offset_deg - cal.coupling * parent) / kDegPerRad;
}

// ----------------------------------------------------------- ServoDriver

ServoDriver::ServoDriver(std::shared_ptr<ServoBus> bus, std::vector<std::string> names,
  std::vector<ServoCalibration> cals, DriverParams params)
: bus_(std::move(bus)), names_(std::move(names)), cals_(std::move(cals)), params_(params)
{
  cals_.resize(names_.size());
  target_.assign(names_.size(), 0.0);
  current_.assign(names_.size(), 0.0);
  pulses_.assign(names_.size(), 0.0);
  state_.assign(names_.size(), State::OFF);
  enable_at_.assign(names_.size(), 0.0);
  params_.joints_per_group = std::max(params_.joints_per_group, 1);
  for (size_t i = 0; i < cals_.size(); ++i) {
    if (cals_[i].coupled_to == static_cast<int>(i) || cals_[i].coupled_to >= static_cast<int>(cals_.size())) {
      cals_[i].coupled_to = -1;
    }
  }
}

int ServoDriver::setTargets(const std::vector<std::string> & names,
  const std::vector<double> & positions, double now)
{
  if (estop_) {return 0;}
  int matched = 0;
  int group_rank = 0;  // stagger counter among servos powered by this call
  int last_group = -1;
  const size_t n = std::min(names.size(), positions.size());
  for (size_t k = 0; k < n; ++k) {
    const auto it = std::find(names_.begin(), names_.end(), names[k]);
    if (it == names_.end() || !std::isfinite(positions[k])) {continue;}
    const size_t i = static_cast<size_t>(it - names_.begin());
    target_[i] = positions[k];
    ++matched;
    if (state_[i] == State::OFF) {
      // Position is unknown while unpowered: jump straight to the target, one
      // leg at a time to limit the inrush current.
      const int group = static_cast<int>(i) / params_.joints_per_group;
      if (group != last_group) {
        if (last_group >= 0) {++group_rank;}
        last_group = group;
      }
      state_[i] = State::PENDING;
      enable_at_[i] = now + group_rank * params_.enable_stagger;
    }
  }
  return matched;
}

double ServoDriver::parentAngle(size_t i) const
{
  const int p = cals_[i].coupled_to;
  if (p < 0) {return 0.0;}
  return state_[p] == State::ON ? current_[p] : target_[p];
}

void ServoDriver::update(double now)
{
  const double dt = last_update_ < 0.0 ? 0.0 : std::clamp(now - last_update_, 0.0, 0.1);
  last_update_ = now;
  if (estop_) {return;}
  clamped_ = 0;
  const double max_step = params_.max_joint_speed * dt;
  std::vector<bool> changed(names_.size(), false);
  for (size_t i = 0; i < names_.size(); ++i) {
    switch (state_[i]) {
      case State::OFF:
        break;
      case State::PENDING:
        if (now >= enable_at_[i]) {
          state_[i] = State::ON;
          current_[i] = target_[i];
          changed[i] = true;
        }
        break;
      case State::ON: {
        const double next = current_[i] + std::clamp(target_[i] - current_[i], -max_step, max_step);
        if (next != current_[i]) {
          current_[i] = next;
          changed[i] = true;
        }
        break;
      }
    }
  }
  // A coupled servo must follow its parent even when its own target is still.
  for (size_t i = 0; i < names_.size(); ++i) {
    const int p = cals_[i].coupled_to;
    if (state_[i] == State::ON && (changed[i] || (p >= 0 && changed[p]))) {
      write(i);
    }
  }
}

void ServoDriver::write(size_t i)
{
  bool clamped = false;
  pulses_[i] = jointToPulseUs(cals_[i], current_[i], &clamped, parentAngle(i));
  bus_->setPulseUs(cals_[i].channel, pulses_[i]);
  if (clamped) {++clamped_;}
}

void ServoDriver::setEstop(bool active)
{
  if (active) {relax();}
  estop_ = active;
}

void ServoDriver::relax()
{
  bus_->disableAll();
  std::fill(state_.begin(), state_.end(), State::OFF);
  std::fill(pulses_.begin(), pulses_.end(), 0.0);
}

bool ServoDriver::setCalibration(size_t index, const ServoCalibration & cal)
{
  if (index >= cals_.size() || !cal.validate().empty()) {return false;}
  if (cal.coupled_to == static_cast<int>(index) || cal.coupled_to >= static_cast<int>(cals_.size())) {
    return false;
  }
  const int old_channel = cals_[index].channel;
  cals_[index] = cal;
  if (state_[index] == State::ON) {
    if (old_channel != cal.channel) {bus_->disable(old_channel);}
    write(index);  // live preview while calibrating
  }
  return true;
}

bool ServoDriver::anyEnabled() const
{
  return std::any_of(state_.begin(), state_.end(), [](State s) {return s == State::ON;});
}

}  // namespace dog_hardware
