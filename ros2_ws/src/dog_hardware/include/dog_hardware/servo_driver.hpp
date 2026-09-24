// ROS-independent servo driver core: joint angle -> pulse mapping with
// calibration, rod linkages, coupled joints, slew-rate limiting, staggered
// power-on and e-stop.
#pragma once

#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "dog_hardware/servo_bus.hpp"

namespace dog_hardware
{

/// Rod (four-bar) drive between the servo horn and the joint.
///
///   servo axis S ──servo_arm── A
///                               ╲ rod
///   joint axis K ──joint_arm─── B
///
/// servo_arm_mm = 0 means the servo sits on the joint axis (direct drive) and
/// the other fields are ignored. With a rod, joint_arm_mm = 0 defaults to
/// servo_arm_mm and rod_mm = 0 defaults to axis_distance_mm: a parallelogram,
/// which turns the joint 1:1. Different lengths make the transfer nonlinear.
/// At the servo's centre pulse the servo arm is assumed perpendicular to the
/// S-K line (the usual way such linkages are assembled).
struct Linkage
{
  double servo_arm_mm{0.0};
  double joint_arm_mm{0.0};
  double rod_mm{0.0};
  double axis_distance_mm{0.0};

  bool direct() const {return servo_arm_mm <= 0.0;}
  double a() const {return servo_arm_mm;}
  double b() const {return joint_arm_mm > 0.0 ? joint_arm_mm : servo_arm_mm;}
  double c() const {return rod_mm > 0.0 ? rod_mm : axis_distance_mm;}
  double d() const {return axis_distance_mm;}

  /// Joint rotation [deg] produced by turning the servo `servo_deg` away from
  /// its centre. NaN if the rod cannot close the loop at that angle.
  double jointDelta(double servo_deg) const;
  /// Servo angles [deg] around centre over which the joint follows the
  /// servo monotonically (stops before a dead point), within +-half.
  std::pair<double, double> usableServoRange(double half_range_deg) const;
  /// Validates geometry; empty string when OK. Requires at least 40 deg of
  /// usable servo travel.
  std::string validate(double half_range_deg) const;
};

/// Per-joint calibration. Joint angles follow the URDF convention
/// (radians, zero = leg straight down, see dog_control/kinematics.hpp).
struct ServoCalibration
{
  int channel{0};           // PCA9685 output 0..15
  int direction{1};         // +1 / -1: servo rotation vs. joint rotation
  double offset_deg{0.0};   // (coupled) joint angle [deg] at the servo's centre pulse
  double pulse_min_us{520.0};   // pulse at servo -range/2 (measured on v1 hardware)
  double pulse_max_us{2220.0};  // pulse at servo +range/2
  double range_deg{180.0};  // servo travel between pulse_min_us and pulse_max_us
  double min_deg{-180.0};   // joint limits [deg]
  double max_deg{180.0};
  Linkage linkage;          // rod drive; servo_arm_mm = 0 -> servo on the joint axis
  int coupled_to{-1};       // index of the parent joint, -1 = none
  double coupling{0.0};     // servo drives (joint + coupling * parent)

  double centerUs() const {return 0.5 * (pulse_min_us + pulse_max_us);}
  double usPerDeg() const {return (pulse_max_us - pulse_min_us) / range_deg;}
  /// Validates the fields; returns an empty string when OK.
  std::string validate() const;
};

/// Pulse width for a joint angle. Clamps to the joint limits and to the
/// servo's pulse range; `clamped` reports whether any clamping happened.
/// `parent_rad` is the angle of the coupled parent joint (ignored if none).
double jointToPulseUs(const ServoCalibration & cal, double joint_rad, bool * clamped = nullptr,
  double parent_rad = 0.0);
/// Joint angle [rad] that a pulse width corresponds to.
double pulseUsToJoint(const ServoCalibration & cal, double us, double parent_rad = 0.0);

struct DriverParams
{
  double max_joint_speed{6.0};  // [rad/s] slew limit (MG996R no-load ~6 rad/s)
  double enable_stagger{0.15};  // [s] delay between legs when powering servos on
  int joints_per_group{3};      // servos enabled together (one leg)
};

class ServoDriver
{
public:
  ServoDriver(std::shared_ptr<ServoBus> bus, std::vector<std::string> names,
    std::vector<ServoCalibration> cals, DriverParams params);

  /// New target positions [rad]; unknown names are ignored. Returns the number
  /// of joints matched. Ignored while the e-stop is engaged.
  int setTargets(const std::vector<std::string> & names, const std::vector<double> & positions,
    double now);
  /// Advance slew limiting and write pulses. `now` is a monotonic time [s].
  void update(double now);
  /// true: all outputs off immediately, commands ignored until released.
  void setEstop(bool active);
  /// Turn every output off (servos go limp) without latching an e-stop.
  void relax();

  bool estop() const {return estop_;}
  bool setCalibration(size_t index, const ServoCalibration & cal);

  const std::vector<std::string> & names() const {return names_;}
  const std::vector<ServoCalibration> & calibrations() const {return cals_;}
  const std::vector<double> & positions() const {return current_;}
  /// Last pulse written per joint [us], 0 when the output is off.
  const std::vector<double> & pulses() const {return pulses_;}
  bool enabled(size_t index) const {return state_[index] == State::ON;}
  bool anyEnabled() const;
  int clampedCount() const {return clamped_;}

private:
  enum class State { OFF, PENDING, ON };
  void write(size_t i);
  double parentAngle(size_t i) const;

  std::shared_ptr<ServoBus> bus_;
  std::vector<std::string> names_;
  std::vector<ServoCalibration> cals_;
  DriverParams params_;

  std::vector<double> target_;
  std::vector<double> current_;
  std::vector<double> pulses_;
  std::vector<State> state_;
  std::vector<double> enable_at_;
  double last_update_{-1.0};
  bool estop_{false};
  int clamped_{0};
};

}  // namespace dog_hardware
