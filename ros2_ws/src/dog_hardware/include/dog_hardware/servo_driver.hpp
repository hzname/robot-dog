// ROS-independent servo driver core: joint angle -> pulse mapping with
// calibration, slew-rate limiting, staggered power-on and e-stop.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dog_hardware/servo_bus.hpp"

namespace dog_hardware
{

/// Per-joint calibration. Joint angles follow the URDF convention
/// (radians, zero = leg straight down, see dog_control/kinematics.hpp).
struct ServoCalibration
{
  int channel{0};           // PCA9685 output 0..15
  int direction{1};         // +1 / -1: servo rotation vs. joint rotation
  double offset_deg{0.0};   // joint angle [deg] when the servo is at its centre pulse
  double pulse_min_us{520.0};   // pulse at servo -range/2 (measured on v1 hardware)
  double pulse_max_us{2220.0};  // pulse at servo +range/2
  double range_deg{180.0};  // servo travel between pulse_min_us and pulse_max_us
  double min_deg{-180.0};   // joint limits [deg]
  double max_deg{180.0};

  double centerUs() const {return 0.5 * (pulse_min_us + pulse_max_us);}
  double usPerDeg() const {return (pulse_max_us - pulse_min_us) / range_deg;}
  /// Validates the fields; returns an empty string when OK.
  std::string validate() const;
};

/// Pulse width for a joint angle. Clamps to the joint limits and to the
/// servo's pulse range; `clamped` reports whether any clamping happened.
double jointToPulseUs(const ServoCalibration & cal, double joint_rad, bool * clamped = nullptr);
/// Joint angle [rad] that a pulse width corresponds to.
double pulseUsToJoint(const ServoCalibration & cal, double us);

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
  bool enabled(size_t index) const {return state_[index] == State::ON;}
  bool anyEnabled() const;
  int clampedCount() const {return clamped_;}

private:
  enum class State { OFF, PENDING, ON };
  void write(size_t i);

  std::shared_ptr<ServoBus> bus_;
  std::vector<std::string> names_;
  std::vector<ServoCalibration> cals_;
  DriverParams params_;

  std::vector<double> target_;
  std::vector<double> current_;
  std::vector<State> state_;
  std::vector<double> enable_at_;
  double last_update_{-1.0};
  bool estop_{false};
  int clamped_{0};
};

}  // namespace dog_hardware
