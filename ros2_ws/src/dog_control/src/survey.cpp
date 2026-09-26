#include "dog_control/survey.hpp"

#include <algorithm>

namespace dog_control
{

namespace
{
constexpr double kDeg = M_PI / 180.0;

double minJerk(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}
}  // namespace

SurveySequence::SurveySequence(const SurveyParams & p)
: p_(p)
{
  const double seg = std::max(p_.segment_time, 0.2);
  const double up = -p_.pitch_up_deg * kDeg, down = p_.pitch_down_deg * kDeg, yaw = p_.yaw_deg * kDeg;
  // up - down, cycles times, back to level; then left - right the same way.
  // The first and the last move of each sweep go half the way: half the time.
  for (int c = 0; c < std::max(p_.cycles, 1); ++c) {
    keys_.push_back({{up, 0.0}, c == 0 ? 0.5 * seg : seg});
    keys_.push_back({{down, 0.0}, seg});
  }
  keys_.push_back({{0.0, 0.0}, 0.5 * seg});
  for (int c = 0; c < std::max(p_.cycles, 1); ++c) {
    keys_.push_back({{0.0, yaw}, c == 0 ? 0.5 * seg : seg});
    keys_.push_back({{0.0, -yaw}, seg});
  }
  keys_.push_back({{0.0, 0.0}, 0.5 * seg});
}

double SurveySequence::duration() const
{
  double t = 0.0;
  for (const auto & k : keys_) {t += k.time;}
  return t;
}

void SurveySequence::start()
{
  active_ = true;
  k_ = 0;
  t_ = 0.0;
  from_ = frame_ = SurveyFrame{};
}

void SurveySequence::update(double dt)
{
  if (!active_) {return;}
  t_ += dt;
  while (k_ < keys_.size() && t_ >= keys_[k_].time) {
    t_ -= keys_[k_].time;
    from_ = keys_[k_].f;
    ++k_;
  }
  if (k_ >= keys_.size()) {
    frame_ = SurveyFrame{};
    active_ = false;
    return;
  }
  const double m = minJerk(t_ / keys_[k_].time);
  frame_.pitch = from_.pitch + (keys_[k_].f.pitch - from_.pitch) * m;
  frame_.yaw = from_.yaw + (keys_[k_].f.yaw - from_.yaw) * m;
}

}  // namespace dog_control
