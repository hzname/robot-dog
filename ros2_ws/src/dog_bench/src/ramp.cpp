#include "dog_bench/ramp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <utility>

namespace dog_bench
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string formatNumber(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", value);
  return buf;
}

/// Command speed of the ramp [us/s] for a joint speed [rad/s].
double rateUsPerSecond(double speed_rad_s, double us_per_deg)
{
  return speed_rad_s * (180.0 / kPi) * us_per_deg;
}
}  // namespace

std::string RampParams::validate() const
{
  if (!std::isfinite(amp_deg) || amp_deg <= 0.0 || amp_deg > kMaxAmpDeg) {
    return "amp_deg must be finite, > 0 and at most 30 deg (got " + formatNumber(amp_deg) + ")";
  }
  if (!std::isfinite(center_us) || center_us < kMinCenterUs || center_us > kMaxCenterUs) {
    return "center_us must be within 1000..1800 us (got " + formatNumber(center_us) + ")";
  }
  if (!std::isfinite(us_per_deg) || us_per_deg < 5.0 || us_per_deg > 15.0) {
    return "us_per_deg must be within 5..15 us/deg (got " + formatNumber(us_per_deg) + ")";
  }
  if (!std::isfinite(hold_s) || hold_s < 0.4 || hold_s > 10.0) {
    return "hold_s must be within 0.4..10 s (got " + formatNumber(hold_s) + ")";
  }
  if (!std::isfinite(rest_s) || rest_s < 0.3 || rest_s > 60.0) {
    return "rest_s must be within 0.3..60 s (got " + formatNumber(rest_s) + ")";
  }
  return "";
}

std::string validateSpeeds(const std::vector<double> & speeds)
{
  if (speeds.empty()) {return "speeds must not be empty";}
  if (speeds.size() > static_cast<std::size_t>(kMaxSpeeds)) {
    return "speeds has " + std::to_string(speeds.size()) + " entries, at most 32 are supported";
  }
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    const double v = speeds[i];
    if (!std::isfinite(v) || v <= 0.0 || v > kMaxSpeedRadS) {
      return "speeds: every value must be finite, > 0 and at most 10 rad/s (index " +
        std::to_string(i) + ", value " + formatNumber(v) + ")";
    }
    if (i > 0 && !(v > speeds[i - 1])) {
      return "speeds must be strictly ascending (index " + std::to_string(i) + ")";
    }
  }
  return "";
}

Ramp::Ramp(const RampParams & params, std::vector<double> speeds)
  : params_(params), speeds_(std::move(speeds))
{
  low_us_ = params_.center_us - params_.amp_deg * params_.us_per_deg;
  high_us_ = params_.center_us + params_.amp_deg * params_.us_per_deg;
  span_us_ = high_us_ - low_us_;
  error_ = params_.validate();
  if (error_.empty()) {error_ = validateSpeeds(speeds_);}
  if (!error_.empty()) {
    state_ = RampState{};
    state_.done = true;
    return;
  }
  beginApproach();
}

std::vector<double> Ramp::defaultSpeeds()
{
  return std::vector<double>(kDefaultSpeedsRadS.begin(), kDefaultSpeedsRadS.end());
}

void Ramp::beginApproach()
{
  state_.phase = RampPhase::APPROACH;
  state_.stroke_id = -1;
  state_.direction = 0;
  state_.speed_index = -1;
  state_.speed_rad_s = 0.0;
  state_.done = false;
  approach_stage_ = 0;
  phase_t_ = 0.0;
  phase_dur_ = params_.hold_s;
  group_ = 0;
  stroke_in_group_ = 0;
  updateCmd();
}

void Ramp::beginStroke()
{
  state_.phase = RampPhase::STROKE;
  state_.stroke_id = group_ * kStrokesPerSpeed + stroke_in_group_;
  state_.direction = (stroke_in_group_ % 2 == 0) ? 1 : -1;
  state_.speed_index = group_;
  state_.speed_rad_s = speeds_[static_cast<std::size_t>(group_)];
  phase_dur_ = span_us_ / rateUsPerSecond(speeds_[static_cast<std::size_t>(group_)],
    params_.us_per_deg);
}

void Ramp::enterNextPhase()
{
  phase_t_ = 0.0;
  switch (state_.phase) {
    case RampPhase::APPROACH:
      if (approach_stage_ == 0) {
        approach_stage_ = 1;
        phase_dur_ = (params_.amp_deg * params_.us_per_deg) /
          rateUsPerSecond(speeds_.front(), params_.us_per_deg);
      } else {
        beginStroke();
      }
      break;
    case RampPhase::STROKE:
      // HOLD carries the stroke id, direction and end command.
      state_.phase = RampPhase::HOLD;
      phase_dur_ = params_.hold_s;
      break;
    case RampPhase::HOLD:
      if (stroke_in_group_ + 1 < kStrokesPerSpeed) {
        ++stroke_in_group_;
        beginStroke();
      } else if (group_ + 1 < static_cast<int>(speeds_.size())) {
        // Rest between groups: cool the servo, let the bus recover.
        state_.phase = RampPhase::REST;
        state_.stroke_id = -1;
        state_.direction = 0;
        state_.speed_index = -1;
        state_.speed_rad_s = 0.0;
        phase_dur_ = params_.rest_s;
      } else {
        // The ramp does not drop the outputs; the session does.
        state_.phase = RampPhase::DONE;
        state_.stroke_id = -1;
        state_.direction = 0;
        state_.speed_index = -1;
        state_.speed_rad_s = 0.0;
        state_.done = true;
        phase_dur_ = 0.0;
      }
      break;
    case RampPhase::REST:
      ++group_;
      stroke_in_group_ = 0;
      beginStroke();
      break;
    case RampPhase::DONE:
      break;
  }
  updateCmd();
}

void Ramp::updateCmd()
{
  const auto clamp = [this](double us) {return std::clamp(us, low_us_, high_us_);};
  switch (state_.phase) {
    case RampPhase::APPROACH:
      if (approach_stage_ == 0) {
        state_.cmd_us = params_.center_us;
      } else {
        state_.cmd_us = clamp(params_.center_us -
          rateUsPerSecond(speeds_.front(), params_.us_per_deg) * phase_t_);
      }
      break;
    case RampPhase::STROKE: {
      const double rate = rateUsPerSecond(speeds_[static_cast<std::size_t>(group_)],
        params_.us_per_deg);
      const double start = (state_.direction > 0) ? low_us_ : high_us_;
      state_.cmd_us = clamp(start + state_.direction * rate * phase_t_);
      break;
    }
    case RampPhase::HOLD:
      state_.cmd_us = (state_.direction > 0) ? high_us_ : low_us_;
      break;
    case RampPhase::REST:
      state_.cmd_us = low_us_;
      break;
    case RampPhase::DONE:
      break;  // keep the last command; dropping the outputs is the session's job
  }
  // The plausibility peak is only judged on the first pair of strokes.
  state_.plausibility_window = (state_.stroke_id == 0 || state_.stroke_id == 1);
}

RampState Ramp::update(double dt)
{
  if (state_.done || !std::isfinite(dt) || dt <= 0.0) {return state_;}
  double remaining = dt;
  while (remaining > 0.0) {
    const double left = phase_dur_ - phase_t_;
    if (remaining < left) {
      phase_t_ += remaining;
      updateCmd();
      remaining = 0.0;
    } else {
      remaining -= left;
      enterNextPhase();
      if (state_.done) {break;}
    }
  }
  state_.t_s += dt;
  return state_;
}

double Ramp::plannedDurationS() const
{
  if (!ok()) {return 0.0;}
  double total = params_.hold_s;  // APPROACH: the hold at the centre
  total += (params_.amp_deg * params_.us_per_deg) /
    rateUsPerSecond(speeds_.front(), params_.us_per_deg);
  for (const double v : speeds_) {
    total += kStrokesPerSpeed *
      (span_us_ / rateUsPerSecond(v, params_.us_per_deg) + params_.hold_s);
  }
  total += params_.rest_s * static_cast<double>(speeds_.size() - 1);
  return total;
}

}  // namespace dog_bench
