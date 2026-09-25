#include "dog_teleop/mapping.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace dog_teleop
{

namespace
{
double snap(double v) {return std::abs(v) < 1e-9 ? 0.0 : v;}

double applyDeadzone(double v, double dz)
{
  const double a = std::abs(v);
  if (a <= dz) {return 0.0;}
  return std::copysign(std::min((a - dz) / (1.0 - dz), 1.0), v);
}
}  // namespace

// ------------------------------------------------------------------ gamepad

JoyMapper::JoyMapper(JoyProfile profile, Limits limits)
: p_(profile), lim_(limits) {}

double JoyMapper::axis(const std::vector<float> & axes, int idx) const
{
  if (idx < 0 || idx >= static_cast<int>(axes.size())) {return 0.0;}
  return std::clamp(static_cast<double>(axes[idx]), -1.0, 1.0);
}

bool JoyMapper::button(const std::vector<int> & buttons, int idx) const
{
  return idx >= 0 && idx < static_cast<int>(buttons.size()) && buttons[idx] != 0;
}

bool JoyMapper::pressed(const std::vector<int> & buttons, int idx) const
{
  return button(buttons, idx) && !button(prev_buttons_, idx);
}

TeleopOutput JoyMapper::process(const std::vector<float> & axes, const std::vector<int> & buttons)
{
  TeleopOutput out;

  if (pressed(buttons, p_.button_estop)) {
    out.estop = true;
  } else if (pressed(buttons, p_.button_release)) {
    out.estop = false;
  }
  if (pressed(buttons, p_.button_stand)) {
    out.command = "stand";
  } else if (pressed(buttons, p_.button_lie)) {
    out.command = "lie";
  }

  // D-pad: one height step per press.
  const double h = axis(axes, p_.axis_height);
  if (h > 0.5 && prev_height_axis_ <= 0.5) {
    height_ = std::min(height_ + p_.height_step, lim_.max_height);
    out.height = height_;
  } else if (h < -0.5 && prev_height_axis_ >= -0.5) {
    height_ = std::max(height_ - p_.height_step, lim_.min_height);
    out.height = height_;
  }
  prev_height_axis_ = h;

  const bool deadman = p_.button_deadman < 0 || button(buttons, p_.button_deadman);
  if (deadman && !out.estop.value_or(false)) {
    const double scale = button(buttons, p_.button_turbo) ? 1.0 : p_.normal_scale;
    Twist2D t;
    t.vx = snap(applyDeadzone(axis(axes, p_.axis_vx), p_.deadzone) * lim_.max_vx * scale);
    t.vy = snap(applyDeadzone(axis(axes, p_.axis_vy), p_.deadzone) * lim_.max_vy * scale);
    t.wz = snap(applyDeadzone(axis(axes, p_.axis_wz), p_.deadzone) * lim_.max_wz * scale);
    out.twist = t;
    // Stick up = nose up = negative pitch (REP-103).
    out.pitch = snap(-applyDeadzone(axis(axes, p_.axis_pitch), p_.deadzone) * lim_.max_pitch);
    driving_ = true;
  } else if (driving_) {
    out.twist = Twist2D{};
    out.pitch = 0.0;
    driving_ = false;
  }

  prev_buttons_ = buttons;
  return out;
}

TeleopOutput JoyMapper::timeout()
{
  TeleopOutput out;
  if (driving_) {
    out.twist = Twist2D{};
    out.pitch = 0.0;
    driving_ = false;
  }
  prev_buttons_.clear();
  prev_height_axis_ = 0.0;
  return out;
}

// ----------------------------------------------------------------- keyboard

size_t parseKey(const std::string & buf, KeyEvent & ev, bool input_idle)
{
  ev = KeyEvent{};
  if (buf.empty()) {return 0;}
  const unsigned char c = static_cast<unsigned char>(buf[0]);
  if (c == 0x03) {ev.key = Key::CTRL_C; return 1;}
  if (c == 0x04) {ev.key = Key::CTRL_D; return 1;}
  if (c == 0x1b) {
    if (buf.size() == 1) {
      if (!input_idle) {return 0;}  // maybe the start of an arrow sequence
      ev.key = Key::ESCAPE;
      return 1;
    }
    if (buf[1] != '[' && buf[1] != 'O') {
      ev.key = Key::ESCAPE;
      return 1;
    }
    if (buf.size() == 2) {
      if (!input_idle) {return 0;}
      return 2;  // incomplete sequence, drop it
    }
    switch (buf[2]) {
      case 'A': ev.key = Key::UP; return 3;
      case 'B': ev.key = Key::DOWN; return 3;
      case 'C': ev.key = Key::RIGHT; return 3;
      case 'D': ev.key = Key::LEFT; return 3;
      default: break;
    }
    // Other CSI sequence (F-keys, Home...): skip up to its final byte.
    for (size_t i = 2; i < buf.size(); ++i) {
      const char f = buf[i];
      if (std::isalpha(static_cast<unsigned char>(f)) || f == '~') {return i + 1;}
    }
    return input_idle ? buf.size() : 0;
  }
  if (c == ' ') {ev.key = Key::SPACE; return 1;}
  ev.key = Key::CHAR;
  ev.ch = static_cast<char>(std::tolower(c));
  return 1;
}

KeyboardMapper::KeyboardMapper(KeyboardParams params, Limits limits)
: p_(params), lim_(limits) {}

TeleopOutput KeyboardMapper::apply(const KeyEvent & ev, bool & quit)
{
  TeleopOutput out;
  quit = false;
  Twist2D t = twist_;
  const char ch = ev.key == Key::CHAR ? ev.ch : 0;

  if (ev.key == Key::UP || ch == 'w') {t.vx += p_.step_v;}
  else if (ev.key == Key::DOWN || ch == 's') {t.vx -= p_.step_v;}
  else if (ch == 'a') {t.vy += p_.step_v;}
  else if (ch == 'd') {t.vy -= p_.step_v;}
  else if (ev.key == Key::LEFT || ch == 'q') {t.wz += p_.step_w;}
  else if (ev.key == Key::RIGHT || ch == 'e') {t.wz -= p_.step_w;}
  else if (ev.key == Key::SPACE || ch == 'k') {t = Twist2D{};}
  else if (ch == '1') {out.command = "stand"; t = Twist2D{};}
  else if (ch == '2') {out.command = "lie"; t = Twist2D{};}
  else if (ev.key == Key::ESCAPE || ch == 'x') {out.estop = true; t = Twist2D{};}
  else if (ch == 'r') {out.estop = false;}
  else if (ch == '+' || ch == '=') {
    height_ = std::min(height_ + p_.step_h, lim_.max_height);
    out.height = height_;
  } else if (ch == '-' || ch == '_') {
    height_ = std::max(height_ - p_.step_h, lim_.min_height);
    out.height = height_;
  } else if (ev.key == Key::CTRL_C || ev.key == Key::CTRL_D) {
    quit = true;
    t = Twist2D{};
  }

  t.vx = snap(std::clamp(t.vx, -lim_.max_vx, lim_.max_vx));
  t.vy = snap(std::clamp(t.vy, -lim_.max_vy, lim_.max_vy));
  t.wz = snap(std::clamp(t.wz, -lim_.max_wz, lim_.max_wz));
  if (t.vx != twist_.vx || t.vy != twist_.vy || t.wz != twist_.wz || quit) {
    twist_ = t;
    out.twist = t;
  }
  return out;
}

const char * keyboardHelp()
{
  return
    "Robot dog keyboard teleop\n"
    "  w / Up      faster forward     s / Down   faster backward\n"
    "  a           strafe left        d          strafe right\n"
    "  q / Left    turn left          e / Right  turn right\n"
    "  Space / k   stop moving\n"
    "  1           stand up           2          lie down\n"
    "  + / -       body height up / down\n"
    "  Esc / x     EMERGENCY STOP     r          release e-stop\n"
    "  h           this help          Ctrl-C     quit (sends stop)\n";
}

}  // namespace dog_teleop
