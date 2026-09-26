// Greeting: the dog sits down on its hind legs, lifts the front of the body
// with both front paws in the air, waves one paw, and gets up again.
//
// Two rear feet alone are a line, not a support. A dog sits with its hocks
// on the ground; this robot's knees bend backwards and play that part: it
// kneels on its rear knees with the calves flat on the ground, and the rear
// knees and feet make the support (a 9 cm long rectangle). Sitting on the rear
// edge of the body instead is impossible here: the rear hips are 2.5 cm from
// it, and with the body pitched up a knee that bends backwards points into
// the ground (checked for every pitch, DEPLOYMENT.md / GAITS.md).
//
// Sequence (world = the ground frame the greeting starts in; x forward,
// origin under the body centre, z = 0 the ground):
//   1. the rear feet step forward one at a time (body over the other three);
//   2. the rear legs fold until the knees touch the ground, the hips still
//      ahead of the knees: the centre of mass stays over the four feet;
//   3. the body leans back (nose up), the hips swing back over the knees:
//      the centre of mass goes over the middle of knees..rear feet, the front
//      feet still on the ground (they still reach it at this pitch);
//   4. the front paws lift, the body leans back a little further, one paw
//      waves; then everything in reverse.
// Pure geometry, no ROS.
#pragma once

#include <array>
#include <vector>

#include "dog_control/kinematics.hpp"

namespace dog_control
{

struct GreetParams
{
  double rear_x{-0.03};      // rear feet step forward to this x [m]
  double contact_r{0.012};   // knee and foot contact radius: a flat calf lies this high [m]
  double margin{0.035};      // centre of mass inside the support by at least this [m]
  double sit_deg{20.0};      // body pitch (nose up) when the front paws lift [deg]
  double beg_deg{30.0};      // ... paws up, waving [deg]
  double lift{0.03};         // foot lift when stepping [m]
  int waves{2};              // paw waves
  double hold{1.0};          // paws up, still, before and after the waves [s]
  double speed{1.0};         // > 1 faster, < 1 slower
};

/// Body and feet at one instant of the greeting (world frame). feet are the
/// kinematic foot points (the calf's end) the legs are solved for.
struct GreetFrame
{
  Vec3 body{};    // body centre
  double pitch{0.0};  // REP-103: + nose down; the greeting pitches nose up (< 0)
  std::array<Vec3, kNumLegs> feet{};
  std::array<bool, kNumLegs> down{{true, true, true, true}};  // on the ground
  bool kneeling{false};  // rear knees on the ground (part of the support)
  double knee_x{0.0};    // ... here
};

class GreetSequence
{
public:
  GreetSequence(const GreetParams & params, const std::array<Vec3, kNumLegs> & neutral, double stand_height,
    double thigh, double calf);

  void start();
  void update(double dt);
  bool active() const {return active_;}
  bool done() const {return !active_;}
  const GreetFrame & frame() const {return frame_;}
  double duration() const;
  /// Where a rear foot touches the ground when its calf lies flat (world x).
  double rearContactX() const {return knee_x_ + calf_ - p_.contact_r;}

private:
  struct Segment
  {
    GreetFrame to;
    double time{1.0};
    int swing{-1};         // this foot goes through the air (-2: both front feet)
    bool kneel{false};     // knees down: the body moves by (pitch, thigh angle)
    double th0{0}, b0{0}, th1{0}, b1{0};  // pitch nose up, thigh angle from vertical (+ hip ahead)
  };
  void build();
  /// Knees down at knee_x_: body for nose-up th and thigh angle b (hip ahead of the knee > 0).
  GreetFrame kneelFrame(double th, double b, const GreetFrame & feet_from) const;
  /// Thigh angle that puts the centre of mass at x for nose-up th.
  double thighFor(double x, double th) const;
  GreetFrame blend(const Segment & s, double u) const;

  GreetParams p_;
  std::array<Vec3, kNumLegs> neutral_;
  double stand_height_, thigh_, calf_, hip_x_;
  double knee_x_{0.0};
  std::vector<Segment> segs_;
  GreetFrame start_frame_, from_, frame_;
  size_t k_{0};
  double t_{0.0};
  bool active_{false};
};

}  // namespace dog_control
