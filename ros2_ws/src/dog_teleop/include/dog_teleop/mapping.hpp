// Pure input-mapping logic for the teleop nodes (no ROS types, unit-tested).
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace dog_teleop
{

struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  bool isZero() const {return vx == 0.0 && vy == 0.0 && wz == 0.0;}
};

struct Limits
{
  double max_vx{0.15};
  double max_vy{0.08};
  double max_wz{0.6};
  double max_pitch{0.26};
  double min_height{-0.05};  // body height offset range [m]
  double max_height{0.03};
};

/// What a teleop input produced this cycle. Unset fields mean "no change".
struct TeleopOutput
{
  std::optional<Twist2D> twist;
  std::optional<std::string> command;  // "stand" | "lie" | "greet"
  std::optional<bool> estop;
  std::optional<double> pitch;   // [rad]
  std::optional<double> height;  // offset [m]
};

// ------------------------------------------------------------------ gamepad

/// Axis / button indices (sensor_msgs/Joy layout, axes: left/up = +1).
/// Defaults: Xbox pad on the Linux xpad driver. -1 disables a binding.
struct JoyProfile
{
  int axis_vx{1};        // left stick up/down
  int axis_vy{0};        // left stick left/right
  int axis_wz{3};        // right stick left/right
  int axis_pitch{4};     // right stick up/down
  int axis_height{7};    // d-pad up/down
  int button_deadman{4}; // LB  - hold to move
  int button_turbo{5};   // RB  - hold for full speed
  int button_stand{0};   // A
  int button_lie{1};     // B
  int button_greet{3};   // Y - greeting (sit, paws up, wave)
  int button_estop{6};   // Back
  int button_release{7}; // Start
  double deadzone{0.08};
  double normal_scale{0.5};  // fraction of max speed without turbo
  double height_step{0.01};  // [m] per d-pad press
};

class JoyMapper
{
public:
  JoyMapper(JoyProfile profile, Limits limits);
  TeleopOutput process(const std::vector<float> & axes, const std::vector<int> & buttons);
  /// Called when the joystick stream goes stale: stop if we were driving.
  TeleopOutput timeout();

private:
  double axis(const std::vector<float> & axes, int idx) const;
  bool button(const std::vector<int> & buttons, int idx) const;
  bool pressed(const std::vector<int> & buttons, int idx) const;  // rising edge

  JoyProfile p_;
  Limits lim_;
  std::vector<int> prev_buttons_;
  double prev_height_axis_{0.0};
  bool driving_{false};
  double height_{0.0};
};

// ----------------------------------------------------------------- keyboard

enum class Key {
  NONE, UP, DOWN, LEFT, RIGHT, ESCAPE, SPACE, CHAR, CTRL_C, CTRL_D
};

struct KeyEvent
{
  Key key{Key::NONE};
  char ch{0};
};

/// Parses one key from raw terminal bytes. Returns the number of bytes
/// consumed (0 if more bytes are needed to decide).
size_t parseKey(const std::string & buf, KeyEvent & ev, bool input_idle);

struct KeyboardParams
{
  double step_v{0.03};    // [m/s] per key press
  double step_w{0.15};    // [rad/s] per key press
  double step_h{0.01};    // [m] per key press
};

class KeyboardMapper
{
public:
  KeyboardMapper(KeyboardParams params, Limits limits);
  /// Applies a key. `quit` is set for Ctrl-C / Ctrl-D.
  TeleopOutput apply(const KeyEvent & ev, bool & quit);
  const Twist2D & twist() const {return twist_;}
  double height() const {return height_;}

private:
  KeyboardParams p_;
  Limits lim_;
  Twist2D twist_;
  double height_{0.0};
};

/// Help text for the keyboard teleop.
const char * keyboardHelp();

}  // namespace dog_teleop
