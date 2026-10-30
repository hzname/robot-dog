#include "dog_bench/safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dog_bench
{

namespace
{
std::string formatNumber(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", value);
  return buf;
}
}  // namespace

const char * eventName(SafetyEvent event)
{
  switch (event) {
    case SafetyEvent::NONE: {return "none";}
    case SafetyEvent::OVERCURRENT: {return "overcurrent";}
    case SafetyEvent::SATURATION: {return "saturation";}
    case SafetyEvent::IMPLAUSIBLE_SHUNT: {return "implausible_shunt";}
    case SafetyEvent::INA_ERRORS: {return "ina_errors";}
    case SafetyEvent::PCA_ERROR: {return "pca_error";}
    case SafetyEvent::TICK_OVERRUN: {return "tick_overrun";}
    case SafetyEvent::TIMEOUT: {return "timeout";}
  }
  return "none";
}

std::string validateSafety(const SafetyParams & params, double shunt_ohm, uint16_t ina_config)
{
  if (!std::isfinite(shunt_ohm) || shunt_ohm <= 0.0) {
    return "shunt_ohm must be finite and > 0 (got " + formatNumber(shunt_ohm) + ")";
  }
  if (!std::isfinite(params.hard_fraction) || params.hard_fraction < 0.5 ||
    params.hard_fraction > 0.99) {
    return "hard_fraction must be within 0.5..0.99 (got " +
      formatNumber(params.hard_fraction) + ")";
  }
  const double limit_a = ina::fullScaleShuntVolts(ina_config) / shunt_ohm;
  if (!std::isfinite(params.overcurrent_a) || params.overcurrent_a <= 0.0 ||
    params.overcurrent_a >= params.hard_fraction * limit_a) {
    return "overcurrent_a must be finite, > 0 and below " +
      formatNumber(params.hard_fraction * limit_a) + " A (95% of the sensor limit on this shunt; got " +
      formatNumber(params.overcurrent_a) + ")";
  }
  if (!std::isfinite(params.overcurrent_time_s) || params.overcurrent_time_s <= 0.0 ||
    params.overcurrent_time_s > 0.5) {
    return "overcurrent_time_s must be within (0, 0.5] s (got " +
      formatNumber(params.overcurrent_time_s) + ")";
  }
  if (!std::isfinite(params.plausibility_min_a) || params.plausibility_min_a <= 0.0 ||
    !std::isfinite(params.plausibility_max_a) ||
    params.plausibility_min_a >= params.plausibility_max_a) {
    return "plausibility_min_a must be finite, > 0 and below plausibility_max_a (got " +
      formatNumber(params.plausibility_min_a) + ".." + formatNumber(params.plausibility_max_a) + ")";
  }
  if (params.max_consecutive_ina_errors < 1 || params.max_consecutive_ina_errors > 5) {
    return "max_consecutive_ina_errors must be within 1..5 (got " +
      std::to_string(params.max_consecutive_ina_errors) + ")";
  }
  if (!std::isfinite(params.max_tick_overrun_s) || params.max_tick_overrun_s <= 0.0 ||
    params.max_tick_overrun_s > 0.2) {
    return "max_tick_overrun_s must be within (0, 0.2] s (got " +
      formatNumber(params.max_tick_overrun_s) + ")";
  }
  if (!std::isfinite(params.max_seconds) || params.max_seconds <= 0.0 ||
    params.max_seconds > 3600.0) {
    return "max_seconds must be within (0, 3600] s (got " + formatNumber(params.max_seconds) + ")";
  }
  return "";
}

SafetyGuard::SafetyGuard(const SafetyParams & params, double shunt_ohm, uint16_t ina_config)
  : params_(params), shunt_ohm_(shunt_ohm), ina_config_(ina_config)
{
  error_ = validateSafety(params_, shunt_ohm_, ina_config_);
}

SafetyEvent SafetyGuard::update(const SafetyReading & reading, double now)
{
  // A guard that cannot watch its inputs lets go at once (fail closed).
  if (!ok()) {
    if (!tripped_) {
      tripped_ = true;
      event_ = SafetyEvent::INA_ERRORS;
      return SafetyEvent::INA_ERRORS;
    }
    return SafetyEvent::NONE;
  }

  // ---- update the whole state first
  bool tick_overrun_now = false;
  if (!started_) {
    started_ = true;
    start_s_ = now;
    last_s_ = now;
  } else {
    tick_overrun_now = (now - last_s_) > params_.max_tick_overrun_s;
    last_s_ = now;
  }
  const bool timeout_now = (now - start_s_) >= params_.max_seconds - 1e-9;

  if (reading.plausibility_window) {window_seen_ = true;}

  bool saturation_now = false;
  bool overcurrent_now = false;
  if (reading.ina_ok) {
    ina_errors_ = 0;
    // Every reading is judged as it is: smoothing would hide a jump.
    const double amps = std::fabs(Ina219Fast::currentA(reading.shunt_raw, shunt_ohm_));
    current_a_ = amps;
    saturation_now = std::fabs(ina::shuntVolts(reading.shunt_raw)) >=
      params_.hard_fraction * ina::fullScaleShuntVolts(ina_config_);
    if (amps > params_.overcurrent_a) {
      if (over_since_ < 0.0) {over_since_ = now;}
      overcurrent_now = (now - over_since_) >= params_.overcurrent_time_s - 1e-9;
    } else {
      over_since_ = -1.0;
    }
    if (reading.plausibility_window) {
      window_peak_a_ = std::max(window_peak_a_, amps);
    }
  } else {
    // The raw code is ignored; the overcurrent window stays untouched.
    ++ina_errors_;
  }

  bool implausible_now = false;
  if (!reading.plausibility_window && window_seen_ && !plausibility_checked_) {
    // The first call after the window falls judges the peak; a window
    // without a single successful reading peaks at 0.
    plausibility_checked_ = true;
    implausible_now = window_peak_a_ < params_.plausibility_min_a ||
      window_peak_a_ > params_.plausibility_max_a;
  }

  // ---- one event per call, by priority
  SafetyEvent fired = SafetyEvent::NONE;
  if (saturation_now) {fired = SafetyEvent::SATURATION;}
  else if (overcurrent_now) {fired = SafetyEvent::OVERCURRENT;}
  else if (!reading.pca_ok) {fired = SafetyEvent::PCA_ERROR;}
  else if (ina_errors_ >= params_.max_consecutive_ina_errors) {fired = SafetyEvent::INA_ERRORS;}
  else if (tick_overrun_now) {fired = SafetyEvent::TICK_OVERRUN;}
  else if (implausible_now) {fired = SafetyEvent::IMPLAUSIBLE_SHUNT;}
  else if (timeout_now) {fired = SafetyEvent::TIMEOUT;}

  if (fired != SafetyEvent::NONE && !tripped_) {
    tripped_ = true;
    event_ = fired;
    return fired;
  }
  return SafetyEvent::NONE;
}

}  // namespace dog_bench
