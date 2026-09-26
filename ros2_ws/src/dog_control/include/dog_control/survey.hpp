// Survey: standing on the spot, the body looks up and down (pitch), then
// left and right (yaw about its centre, the feet stay where they are).
//
// The lidars are tilted 30 deg in a cross: a still body scans two slanted
// lines round itself. Swaying the body sweeps them over the walls and the
// furniture above and below those lines and to the sides - the map the
// localization matches against fills in, and on start the robot can find
// itself in a stored map before it walks. Pure geometry, no ROS.
#pragma once

#include <cmath>
#include <vector>

namespace dog_control
{

struct SurveyParams
{
  double pitch_up_deg{12.0};    // nose up
  double pitch_down_deg{10.0};  // nose down
  double yaw_deg{15.0};         // body turned on its feet, each way
  int cycles{2};                // up-down and left-right, each this many times
  double segment_time{1.5};     // [s] one move between two extremes
};

/// Body attitude relative to the feet at one instant of the survey.
struct SurveyFrame
{
  double pitch{0.0};  // [rad] REP-103: + nose down
  double yaw{0.0};    // [rad] + left
};

class SurveySequence
{
public:
  explicit SurveySequence(const SurveyParams & p = SurveyParams());

  void start();
  void update(double dt);
  bool active() const {return active_;}
  bool done() const {return !active_;}
  const SurveyFrame & frame() const {return frame_;}
  double duration() const;

private:
  struct Key
  {
    SurveyFrame f;
    double time;  // to get here from the previous key
  };
  SurveyParams p_;
  std::vector<Key> keys_;
  size_t k_{0};
  double t_{0.0};
  bool active_{false};
  SurveyFrame from_{}, frame_{};
};

}  // namespace dog_control
