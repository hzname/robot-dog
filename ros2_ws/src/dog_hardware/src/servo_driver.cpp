#include "dog_hardware/servo_driver.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace dog_hardware
{

namespace
{
constexpr double kDegPerRad = 180.0 / M_PI;
}

std::string ServoCalibration::validate() const
{
  if (channel < 0 || channel >= kPwmChannels) {return "channel must be 0..15";}
  if (direction != 1 && direction != -1) {return "direction must be 1 or -1";}
  if (pulse_min_us < 300.0 || pulse_max_us > 2800.0 || pulse_min_us >= pulse_max_us) {
    return "pulse range must satisfy 300 <= min < max <= 2800 us";
  }
  if (range_deg <= 0.0 || range_deg > 360.0) {return "range_deg must be in (0, 360]";}
  if (min_deg >= max_deg) {return "min_deg must be < max_deg";}
  return {};
}

double jointToPulseUs(const ServoCalibration & cal, double joint_rad, bool * clamped)
{
  const double q_deg = joint_rad * kDegPerRad;
  double q = std::clamp(q_deg, cal.min_deg, cal.max_deg);
  bool c = q != q_deg;
  const double half = 0.5 * cal.range_deg;
  double servo_deg = cal.direction * (q - cal.offset_deg);
  if (servo_deg > half || servo_deg < -half) {
    servo_deg = std::clamp(servo_deg, -half, half);
    c = true;
  }
  if (clamped) {*clamped = c;}
  return cal.centerUs() + servo_deg * cal.usPerDeg();
}

double pulseUsToJoint(const ServoCalibration & cal, double us)
{
  const double servo_deg = (us - cal.centerUs()) / cal.usPerDeg();
  return (cal.direction * servo_deg + cal.offset_deg) / kDegPerRad;
}

ServoDriver::ServoDriver(std::shared_ptr<ServoBus> bus, std::vector<std::string> names,
  std::vector<ServoCalibration> cals, DriverParams params)
: bus_(std::move(bus)), names_(std::move(names)), cals_(std::move(cals)), params_(params)
{
  cals_.resize(names_.size());
  target_.assign(names_.size(), 0.0);
  current_.assign(names_.size(), 0.0);
  state_.assign(names_.size(), State::OFF);
  enable_at_.assign(names_.size(), 0.0);
  params_.joints_per_group = std::max(params_.joints_per_group, 1);
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

void ServoDriver::update(double now)
{
  const double dt = last_update_ < 0.0 ? 0.0 : std::clamp(now - last_update_, 0.0, 0.1);
  last_update_ = now;
  if (estop_) {return;}
  clamped_ = 0;
  const double max_step = params_.max_joint_speed * dt;
  for (size_t i = 0; i < names_.size(); ++i) {
    switch (state_[i]) {
      case State::OFF:
        break;
      case State::PENDING:
        if (now >= enable_at_[i]) {
          state_[i] = State::ON;
          current_[i] = target_[i];
          write(i);
        }
        break;
      case State::ON: {
        const double next = current_[i] + std::clamp(target_[i] - current_[i], -max_step, max_step);
        if (next != current_[i]) {
          current_[i] = next;
          write(i);
        }
        break;
      }
    }
  }
}

void ServoDriver::write(size_t i)
{
  bool clamped = false;
  bus_->setPulseUs(cals_[i].channel, jointToPulseUs(cals_[i], current_[i], &clamped));
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
}

bool ServoDriver::setCalibration(size_t index, const ServoCalibration & cal)
{
  if (index >= cals_.size() || !cal.validate().empty()) {return false;}
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
