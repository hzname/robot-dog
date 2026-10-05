// Speed ramp generator of the servo bench tool: a bounded pulse command [us]
// for one servo, from a centre hold and approach through 15 speed groups of
// 10 strokes each (D-08, D-10). ROS-free; validation returns a message
// instead of throwing, and the command never leaves [lowUs, highUs].
#pragma once

#include <array>
#include <string>
#include <vector>

namespace dog_bench
{

/// Hard limit on the sweep half-amplitude [deg] (D-10).
constexpr double kMaxAmpDeg = 30.0;
/// Centre pulse limits [us] (D-10).
constexpr double kMinCenterUs = 1000.0;
constexpr double kMaxCenterUs = 1800.0;
/// Hard limit on a commanded speed [rad/s] (D-10).
constexpr double kMaxSpeedRadS = 10.0;
/// Strokes per speed: 5 A->B / B->A pairs (D-08).
constexpr int kStrokePairsPerSpeed = 5;
constexpr int kStrokesPerSpeed = 10;
/// Upper bound on the speed grid length.
constexpr int kMaxSpeeds = 32;

/// The default speed grid [rad/s] (D-08): 1.5 .. 10.0, strictly ascending.
constexpr std::array<double, 15> kDefaultSpeedsRadS = {
  1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 8.0, 9.0, 10.0};

struct RampParams
{
  double amp_deg{25.0};      // sweep half-amplitude [deg], at most 30 (D-10)
  double center_us{1370.0};  // pulse at the centre [us]: the middle of the servos.yaml 520..2220 range
  double us_per_deg{9.444};  // 1700 us / 180 deg (servos.yaml nominal; to be checked with a protractor)
  double hold_s{0.4};        // pause at each stroke end [s] (D-08)
  double rest_s{3.0};        // pause between speed groups [s]: cooling and bus recovery

  /// Returns an empty string when OK, otherwise a message naming the field.
  std::string validate() const;
};

/// Returns an empty string when OK, otherwise a message naming the field.
std::string validateSpeeds(const std::vector<double> & speeds);

enum class RampPhase {APPROACH, REST, STROKE, HOLD, DONE};

struct RampState
{
  RampPhase phase{RampPhase::DONE};
  int stroke_id{-1};        // 0..149 across the run, -1 outside strokes and holds
  int direction{0};         // +1 A->B in a stroke, -1 B->A, 0 outside
  double cmd_us{0.0};       // pulse command [us], always within [lowUs, highUs]
  int speed_index{-1};      // stroke_id / 10 while a stroke or hold is active, else -1
  double speed_rad_s{0.0};  // the group speed [rad/s] while active, else 0
  bool plausibility_window{false};  // true exactly on strokes 0 and 1 (the first pair)
  bool done{false};
  double t_s{0.0};          // time since the start [s]
};

class Ramp
{
public:
  explicit Ramp(const RampParams & params, std::vector<double> speeds = defaultSpeeds());

  /// The default grid as a vector (kDefaultSpeedsRadS).
  static std::vector<double> defaultSpeeds();

  /// false when the params or the speeds are invalid: the ramp stays DONE
  /// with cmd_us 0 (the PWM output rejects that value).
  bool ok() const {return error_.empty();}
  const std::string & error() const {return error_;}

  /// Advance by dt [s]; a non-finite or non-positive dt keeps the state.
  /// The leftover dt at a phase boundary carries over, so the total time
  /// does not depend on the tick size.
  RampState update(double dt);

  const RampState & state() const {return state_;}
  /// Time from the start to DONE [s], 0 when the ramp is not ok.
  double plannedDurationS() const;
  /// Low edge of the command window [us]: centre - amplitude.
  double lowUs() const {return low_us_;}
  /// High edge of the command window [us]: centre + amplitude.
  double highUs() const {return high_us_;}

private:
  void beginApproach();
  void beginStroke();
  void enterNextPhase();
  void updateCmd();

  RampParams params_;
  std::vector<double> speeds_;
  std::string error_;
  double low_us_{0.0};
  double high_us_{0.0};
  double span_us_{0.0};

  RampState state_;
  double phase_t_{0.0};     // time inside the current phase [s]
  double phase_dur_{0.0};   // duration of the current phase [s]
  int approach_stage_{0};   // 0: hold at the centre, 1: ramp down to the low edge
  int group_{0};            // speed group index
  int stroke_in_group_{0};  // 0..kStrokesPerSpeed-1 inside the group
};

}  // namespace dog_bench
