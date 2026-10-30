// Tests for the speed ramp generator of the servo bench tool: parameter and
// speed-grid validation, stroke numbering and timing (hold, rest), the
// command window, the command slope and the full ramp driving a fake PWM
// output (D-08, D-10). Deterministic: time is passed as numbers and the PWM
// layer is a recording fake; no clocks, no sleeps.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "dog_bench/pwm_out.hpp"
#include "dog_bench/ramp.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

/// Command speed [us/s] per rad/s of joint speed at a given calibration.
double rateUsPerRad(double us_per_deg)
{
  return (180.0 / kPi) * us_per_deg;
}
}  // namespace

TEST(Ramp, ValidateRejectsAmpAbove30)
{
  using namespace dog_bench;
  RampParams p;
  EXPECT_EQ(p.validate(), "");

  p.amp_deg = 30.0;
  EXPECT_EQ(p.validate(), "");
  p.amp_deg = 30.01;
  EXPECT_NE(p.validate().find("amp_deg"), std::string::npos);
  p.amp_deg = 0.0;
  EXPECT_NE(p.validate().find("amp_deg"), std::string::npos);
  p.amp_deg = -5.0;
  EXPECT_NE(p.validate().find("amp_deg"), std::string::npos);
  p.amp_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(p.validate().find("amp_deg"), std::string::npos);
  p.amp_deg = std::numeric_limits<double>::infinity();
  EXPECT_NE(p.validate().find("amp_deg"), std::string::npos);
  p.amp_deg = 25.0;

  p.center_us = 999.9;
  EXPECT_NE(p.validate().find("center_us"), std::string::npos);
  p.center_us = 1800.1;
  EXPECT_NE(p.validate().find("center_us"), std::string::npos);
  p.center_us = 1000.0;
  EXPECT_EQ(p.validate(), "");
  p.center_us = 1800.0;
  EXPECT_EQ(p.validate(), "");
  p.center_us = 1370.0;

  p.us_per_deg = 4.9;
  EXPECT_NE(p.validate().find("us_per_deg"), std::string::npos);
  p.us_per_deg = 15.1;
  EXPECT_NE(p.validate().find("us_per_deg"), std::string::npos);
  p.us_per_deg = 9.444;

  p.hold_s = 0.39;
  EXPECT_NE(p.validate().find("hold_s"), std::string::npos);
  p.hold_s = 0.4;
  EXPECT_EQ(p.validate(), "");

  p.rest_s = 0.29;
  EXPECT_NE(p.validate().find("rest_s"), std::string::npos);
  p.rest_s = 0.3;
  EXPECT_EQ(p.validate(), "");

  // An amplitude above the hard limit makes the whole ramp refuse to run:
  // ok() false, DONE at once with cmd_us 0 (the PWM output rejects that).
  RampParams bad;
  bad.amp_deg = 31.0;
  Ramp ramp(bad);
  EXPECT_FALSE(ramp.ok());
  EXPECT_FALSE(ramp.error().empty());
  EXPECT_TRUE(ramp.state().done);
  EXPECT_EQ(ramp.state().phase, RampPhase::DONE);
  EXPECT_DOUBLE_EQ(ramp.state().cmd_us, 0.0);
  const RampState after = ramp.update(0.001);
  EXPECT_TRUE(after.done);
  EXPECT_DOUBLE_EQ(after.cmd_us, 0.0);
  EXPECT_EQ(after.stroke_id, -1);
}

TEST(Ramp, SpeedsAscendAndCapped)
{
  using namespace dog_bench;
  const std::vector<double> def = Ramp::defaultSpeeds();
  ASSERT_EQ(def.size(), 15u);
  EXPECT_DOUBLE_EQ(def.front(), 1.5);
  EXPECT_DOUBLE_EQ(def.back(), 10.0);
  for (std::size_t i = 1; i < def.size(); ++i) {EXPECT_LT(def[i - 1], def[i]);}
  EXPECT_EQ(validateSpeeds(def), "");

  EXPECT_NE(validateSpeeds({}).find("empty"), std::string::npos);
  EXPECT_FALSE(validateSpeeds({2.0, 2.0}).empty());  // equal neighbours
  EXPECT_FALSE(validateSpeeds({2.5, 2.0}).empty());  // descending
  EXPECT_FALSE(validateSpeeds({10.5}).empty());      // above the hard cap
  EXPECT_FALSE(validateSpeeds({0.0}).empty());
  EXPECT_FALSE(validateSpeeds({-1.0}).empty());
  EXPECT_FALSE(validateSpeeds({std::numeric_limits<double>::quiet_NaN()}).empty());
  std::vector<double> many;
  for (int i = 0; i < 33; ++i) {many.push_back(0.1 * (i + 1));}
  EXPECT_FALSE(validateSpeeds(many).empty());

  // Inside every stroke the command moves at v * (180/pi) * us_per_deg us/s.
  // The run ticks at 1 ms; the first in-stroke neighbour pair of each group
  // is measured.
  Ramp ramp(RampParams{});
  ASSERT_TRUE(ramp.ok()) << ramp.error();
  const double dt = 0.001;
  std::vector<bool> measured(def.size(), false);
  bool prev_in_stroke = false;
  int prev_id = -1;
  double prev_cmd = 0.0;
  double prev_t = 0.0;
  while (!ramp.state().done) {
    const RampState st = ramp.update(dt);
    const double now = st.t_s;
    if (st.phase == RampPhase::STROKE && prev_in_stroke && st.stroke_id == prev_id) {
      const std::size_t g = static_cast<std::size_t>(st.stroke_id / kStrokesPerSpeed);
      if (!measured[g]) {
        const double slope = (st.cmd_us - prev_cmd) / (now - prev_t);
        const double expected = def[g] * rateUsPerRad(9.444);
        EXPECT_NEAR(std::fabs(slope), expected, 0.01 * expected) << "group " << g;
        EXPECT_EQ(st.direction, slope > 0.0 ? 1 : -1);
        measured[g] = true;
      }
    }
    prev_in_stroke = st.phase == RampPhase::STROKE;
    prev_id = st.stroke_id;
    prev_cmd = st.cmd_us;
    prev_t = now;
  }
  for (std::size_t g = 0; g < measured.size(); ++g) {EXPECT_TRUE(measured[g]) << "group " << g;}
}

TEST(Ramp, StrokesAreNumberedWithoutGaps)
{
  using namespace dog_bench;
  const RampParams p;
  Ramp ramp(p);
  ASSERT_TRUE(ramp.ok()) << ramp.error();
  const double dt = 0.001;

  int last_id = -1;         // the last stroke id seen; the next one is +1
  int strokes_seen = 0;
  int holds_seen = 0;
  int holds_without_stroke = 0;  // counts STROKE -> HOLD transitions
  int hold_run = 0;
  int rest_run = 0;
  int rest_runs = 0;
  std::vector<int> rest_after_ids;
  bool rest_before_first_stroke = false;

  RampPhase prev_phase = RampPhase::DONE;
  int prev_id = -1;
  int prev_dir = 0;
  double prev_cmd = 0.0;
  double stroke_first_cmd = 0.0;

  while (!ramp.state().done) {
    const RampState st = ramp.update(dt);
    const bool is_stroke = st.phase == RampPhase::STROKE;
    const bool is_hold = st.phase == RampPhase::HOLD;

    // ---- stroke ids: 0..149 contiguous; -1 outside strokes and holds
    if (is_stroke || is_hold) {
      ASSERT_GE(st.stroke_id, 0) << "t=" << st.t_s;
      if (st.stroke_id != last_id) {
        EXPECT_EQ(st.stroke_id, last_id + 1) << "gap before stroke " << st.stroke_id;
        last_id = st.stroke_id;
        ++strokes_seen;
      }
      EXPECT_EQ(st.speed_index, st.stroke_id / kStrokesPerSpeed) << "t=" << st.t_s;
      EXPECT_DOUBLE_EQ(st.speed_rad_s,
        kDefaultSpeedsRadS[static_cast<std::size_t>(st.speed_index)]);
    } else {
      EXPECT_EQ(st.stroke_id, -1) << "t=" << st.t_s;
      EXPECT_EQ(st.speed_index, -1) << "t=" << st.t_s;
      EXPECT_DOUBLE_EQ(st.speed_rad_s, 0.0) << "t=" << st.t_s;
      if (st.phase == RampPhase::REST && strokes_seen == 0) {rest_before_first_stroke = true;}
    }
    EXPECT_EQ(st.plausibility_window, st.stroke_id == 0 || st.stroke_id == 1) << "t=" << st.t_s;

    // ---- direction and monotonicity inside a stroke; the hold is constant
    if (is_stroke && !(prev_phase == RampPhase::STROKE && prev_id == st.stroke_id)) {
      EXPECT_EQ(st.direction, st.stroke_id % 2 == 0 ? 1 : -1) << "stroke " << st.stroke_id;
      if (strokes_seen == 1 || st.stroke_id == 0) {
        // the first group starts right after the approach, no rest in between
        if (st.stroke_id == 0) {EXPECT_EQ(prev_phase, RampPhase::APPROACH);}
      }
      stroke_first_cmd = st.cmd_us;
    } else if (is_stroke) {
      if (st.direction > 0) {EXPECT_GT(st.cmd_us, prev_cmd);}
      else {EXPECT_LT(st.cmd_us, prev_cmd);}
    } else if (is_hold) {
      EXPECT_EQ(st.direction, st.stroke_id % 2 == 0 ? 1 : -1);
      if (prev_phase == RampPhase::STROKE) {
        // the stroke just ended: hold carries its id, direction and end value
        EXPECT_EQ(st.stroke_id, prev_id);
        EXPECT_EQ(st.direction, prev_dir);
        if (prev_dir > 0) {EXPECT_GT(st.cmd_us, stroke_first_cmd);}
        else {EXPECT_LT(st.cmd_us, stroke_first_cmd);}
        ++holds_seen;
      } else {
        EXPECT_DOUBLE_EQ(st.cmd_us, prev_cmd);  // constant through the hold
      }
    }

    // ---- hold duration: not shorter than hold_s - dt, same stroke id
    if (is_hold) {
      ++hold_run;
      if (prev_phase == RampPhase::STROKE) {++holds_without_stroke;}
    } else if (hold_run > 0) {
      EXPECT_GE(hold_run * dt, p.hold_s - dt);
      hold_run = 0;
    }

    // ---- REST between groups: after stroke 9 of each group, except the last
    if (st.phase == RampPhase::REST) {
      EXPECT_DOUBLE_EQ(st.cmd_us, ramp.lowUs()) << "t=" << st.t_s;
      if (rest_run == 0) {
        ++rest_runs;
        rest_after_ids.push_back(last_id);
      }
      ++rest_run;
    } else if (rest_run > 0) {
      EXPECT_GE(rest_run * dt, p.rest_s - dt);
      rest_run = 0;
    }

    prev_phase = st.phase;
    prev_id = st.stroke_id;
    prev_dir = st.direction;
    prev_cmd = st.cmd_us;
  }
  if (hold_run > 0) {EXPECT_GE(hold_run * dt, p.hold_s - dt);}
  if (rest_run > 0) {EXPECT_GE(rest_run * dt, p.rest_s - dt);}

  EXPECT_EQ(strokes_seen, 150);
  EXPECT_EQ(holds_seen, 150);
  EXPECT_EQ(holds_without_stroke, 150);
  EXPECT_EQ(last_id, 149);
  EXPECT_EQ(rest_runs, 14);
  EXPECT_FALSE(rest_before_first_stroke);
  std::vector<int> expected_rest_ids;
  for (int g = 0; g < 14; ++g) {expected_rest_ids.push_back(g * kStrokesPerSpeed + 9);}
  EXPECT_EQ(rest_after_ids, expected_rest_ids);
}

TEST(Ramp, PulseStaysInsideWindow)
{
  using namespace dog_bench;
  const double cases[3][2] = {{1370.0, 25.0}, {1000.0, 30.0}, {1800.0, 30.0}};
  for (const auto & c : cases) {
    RampParams p;
    p.center_us = c[0];
    p.amp_deg = c[1];
    Ramp ramp(p);
    ASSERT_TRUE(ramp.ok()) << ramp.error();
    const double a = ramp.lowUs();
    const double b = ramp.highUs();
    EXPECT_NEAR(a, p.center_us - p.amp_deg * p.us_per_deg, 1e-9);
    EXPECT_NEAR(b, p.center_us + p.amp_deg * p.us_per_deg, 1e-9);

    const double dt = 0.001;
    const RampState first = ramp.update(dt);
    EXPECT_NEAR(first.cmd_us, p.center_us, 1e-9);  // the first value is the centre
    EXPECT_GE(first.cmd_us, a - 1e-9);
    EXPECT_LE(first.cmd_us, b + 1e-9);
    while (!ramp.state().done) {
      const RampState st = ramp.update(dt);
      EXPECT_GE(st.cmd_us, a - 1e-9) << "t=" << st.t_s;
      EXPECT_LE(st.cmd_us, b + 1e-9) << "t=" << st.t_s;
    }

    // Once DONE, further updates change nothing.
    const RampState done = ramp.state();
    const RampState again = ramp.update(dt);
    EXPECT_TRUE(again.done);
    EXPECT_EQ(again.phase, RampPhase::DONE);
    EXPECT_EQ(again.stroke_id, done.stroke_id);
    EXPECT_DOUBLE_EQ(again.cmd_us, done.cmd_us);
    const RampState false_dt = ramp.update(-1.0);
    EXPECT_DOUBLE_EQ(false_dt.cmd_us, done.cmd_us);
  }
}

TEST(Ramp, TotalTimeIndependentOfDt)
{
  using namespace dog_bench;
  Ramp planned_ramp(RampParams{});
  ASSERT_TRUE(planned_ramp.ok());
  const double planned = planned_ramp.plannedDurationS();
  EXPECT_GT(planned, 100.0);
  EXPECT_LT(planned, 200.0);
  // The defaults sit near 136 s: 42 s of REST (14 pauses of 3 s), 60 s of
  // stroke holds (150 x 0.4 s), ~33.5 s of stroke motion, ~0.7 s of approach.
  EXPECT_NEAR(planned, 136.19, 0.05);

  for (const double dt : {0.001, 0.0071}) {
    Ramp ramp(RampParams{});
    ASSERT_TRUE(ramp.ok()) << ramp.error();
    double t = 0.0;
    int ticks = 0;
    while (!ramp.state().done) {
      ramp.update(dt);
      t += dt;
      ++ticks;
      ASSERT_LT(ticks, 2000000) << "dt=" << dt;
    }
    EXPECT_NEAR(t, planned, dt + 1e-9) << "dt=" << dt;
  }
}

TEST(Ramp, TracerDrivesFakePwmOnOneSpeed)
{
  using namespace dog_bench;
  Ramp ramp(RampParams{}, {3.0});
  ASSERT_TRUE(ramp.ok()) << ramp.error();
  const double a = ramp.lowUs();
  const double b = ramp.highUs();

  FakePwm pwm(a, b);
  // Before a successful preflight() nothing may be written.
  EXPECT_FALSE(pwm.setPulseUs(1370.0));
  EXPECT_TRUE(pwm.pulses().empty());
  ASSERT_TRUE(pwm.preflight());

  // The session loop: 1 ms ticks, one PWM write every 20th tick (50 Hz).
  const double dt = 0.001;
  const int every = 20;
  int ticks = 0;
  while (!ramp.state().done) {
    const RampState st = ramp.update(dt);
    ++ticks;
    if (ticks % every == 0) {ASSERT_TRUE(pwm.setPulseUs(st.cmd_us)) << st.cmd_us;}
  }
  const double total = ticks * dt;
  ASSERT_TRUE(pwm.release());
  ASSERT_TRUE(pwm.released());
  EXPECT_EQ(pwm.releases(), 1);

  const std::vector<double> & pulses = pwm.pulses();
  ASSERT_FALSE(pulses.empty());
  EXPECT_NEAR(static_cast<double>(pulses.size()), total / 0.02, 2.0);
  EXPECT_NEAR(pulses.front(), 1370.0, 1e-9);  // the approach starts at the centre
  EXPECT_NEAR(pulses.back(), a, 1e-9);        // the ramp ends at the low edge
  for (const double us : pulses) {
    EXPECT_GE(us, a - 1e-9);
    EXPECT_LE(us, b + 1e-9);
  }
  EXPECT_NEAR(*std::max_element(pulses.begin(), pulses.end()), b, 1e-9);
  EXPECT_NEAR(*std::min_element(pulses.begin(), pulses.end()), a, 1e-9);

  // A fresh fake rejects out-of-window, non-finite and pre-preflight pulses
  // without recording them, and honours failWritesFrom(n).
  FakePwm limits(a, b);
  EXPECT_FALSE(limits.setPulseUs(1370.0));
  ASSERT_TRUE(limits.preflight());
  EXPECT_FALSE(limits.setPulseUs(a - 1.0));
  EXPECT_FALSE(limits.setPulseUs(b + 1.0));
  EXPECT_FALSE(limits.setPulseUs(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(limits.setPulseUs(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(limits.pulses().empty());

  FakePwm failing(a, b);
  ASSERT_TRUE(failing.preflight());
  failing.failWritesFrom(3);
  EXPECT_TRUE(failing.setPulseUs(a));
  EXPECT_TRUE(failing.setPulseUs(b));
  EXPECT_FALSE(failing.setPulseUs(1370.0));
  EXPECT_EQ(failing.pulses().size(), 2u);
  EXPECT_FALSE(failing.error().empty());

  FakePwm refused(a, b);
  refused.failPreflight("injected preflight failure");
  EXPECT_FALSE(refused.preflight());
  EXPECT_FALSE(refused.setPulseUs(1370.0));
  EXPECT_TRUE(refused.pulses().empty());
}
