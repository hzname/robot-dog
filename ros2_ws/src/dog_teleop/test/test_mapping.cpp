#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "dog_teleop/mapping.hpp"

using namespace dog_teleop;

namespace
{
std::vector<float> axes(float lx = 0, float ly = 0, float rx = 0, float ry = 0, float dpy = 0)
{
  // xpad layout: LX LY LT RX RY RT DX DY
  return {lx, ly, 1.0f, rx, ry, 1.0f, 0.0f, dpy};
}
std::vector<int> buttons(std::initializer_list<int> pressed)
{
  std::vector<int> b(11, 0);
  for (int i : pressed) {b[i] = 1;}
  return b;
}
constexpr int A = 0, B = 1, Y = 3, LB = 4, RB = 5, BACK = 6, START = 7;
}  // namespace

TEST(JoyMapper, NoMotionWithoutDeadman)
{
  JoyMapper m(JoyProfile{}, Limits{});
  const auto out = m.process(axes(0, 1.0f), buttons({}));
  EXPECT_FALSE(out.twist.has_value());
}

TEST(JoyMapper, DeadmanScalesAndTurbo)
{
  Limits lim;
  JoyMapper m(JoyProfile{}, lim);
  auto out = m.process(axes(0.0f, 1.0f, 1.0f), buttons({LB}));
  ASSERT_TRUE(out.twist);
  EXPECT_NEAR(out.twist->vx, 0.5 * lim.max_vx, 1e-9);
  EXPECT_NEAR(out.twist->wz, 0.5 * lim.max_wz, 1e-9);  // right stick left = turn left
  out = m.process(axes(-1.0f, -1.0f), buttons({LB, RB}));
  EXPECT_NEAR(out.twist->vx, -lim.max_vx, 1e-9);
  EXPECT_NEAR(out.twist->vy, -lim.max_vy, 1e-9);  // stick right = strafe right
}

TEST(JoyMapper, DeadzoneAndReleaseStops)
{
  JoyMapper m(JoyProfile{}, Limits{});
  auto out = m.process(axes(0.05f, 0.05f), buttons({LB}));
  ASSERT_TRUE(out.twist);
  EXPECT_TRUE(out.twist->isZero());
  m.process(axes(0, 1.0f), buttons({LB}));
  out = m.process(axes(0, 1.0f), buttons({}));
  ASSERT_TRUE(out.twist);
  EXPECT_TRUE(out.twist->isZero());
  out = m.process(axes(0, 1.0f), buttons({}));
  EXPECT_FALSE(out.twist.has_value());  // stop is sent once
}

TEST(JoyMapper, ButtonsAreEdgeTriggered)
{
  JoyMapper m(JoyProfile{}, Limits{});
  EXPECT_EQ(m.process(axes(), buttons({A})).command.value_or(""), "stand");
  EXPECT_FALSE(m.process(axes(), buttons({A})).command.has_value());
  EXPECT_EQ(m.process(axes(), buttons({B})).command.value_or(""), "lie");
  EXPECT_EQ(m.process(axes(), buttons({Y})).command.value_or(""), "greet");
  EXPECT_TRUE(m.process(axes(), buttons({BACK})).estop.value_or(false));
  EXPECT_FALSE(m.process(axes(), buttons({START})).estop.value_or(true));
}

TEST(JoyMapper, EstopOverridesDeadman)
{
  JoyMapper m(JoyProfile{}, Limits{});
  m.process(axes(0, 1.0f), buttons({LB}));
  const auto out = m.process(axes(0, 1.0f), buttons({LB, BACK}));
  EXPECT_TRUE(out.estop.value_or(false));
  ASSERT_TRUE(out.twist);
  EXPECT_TRUE(out.twist->isZero());
}

TEST(JoyMapper, DpadStepsHeightWithinLimits)
{
  Limits lim;
  JoyMapper m(JoyProfile{}, lim);
  EXPECT_NEAR(m.process(axes(0, 0, 0, 0, 1.0f), buttons({})).height.value_or(-1), 0.01, 1e-9);
  EXPECT_FALSE(m.process(axes(0, 0, 0, 0, 1.0f), buttons({})).height.has_value());  // held
  m.process(axes(), buttons({}));
  for (int i = 0; i < 20; ++i) {
    m.process(axes(0, 0, 0, 0, -1.0f), buttons({}));
    m.process(axes(), buttons({}));
  }
  EXPECT_NEAR(m.process(axes(0, 0, 0, 0, -1.0f), buttons({})).height.value_or(0), lim.min_height, 1e-9);
}

TEST(JoyMapper, TimeoutStopsOnlyWhenDriving)
{
  JoyMapper m(JoyProfile{}, Limits{});
  EXPECT_FALSE(m.timeout().twist.has_value());
  m.process(axes(0, 1.0f), buttons({LB}));
  const auto out = m.timeout();
  ASSERT_TRUE(out.twist);
  EXPECT_TRUE(out.twist->isZero());
}

TEST(JoyMapper, ShortAxesArraysAreSafe)
{
  JoyMapper m(JoyProfile{}, Limits{});
  const auto out = m.process({0.5f}, {0, 0, 0, 0, 1});  // only axis 0 (strafe) present
  ASSERT_TRUE(out.twist);
  EXPECT_GT(out.twist->vy, 0.0);
  EXPECT_EQ(out.twist->vx, 0.0);
  EXPECT_EQ(out.twist->wz, 0.0);
}

TEST(KeyParser, ArrowsEscapeAndChars)
{
  KeyEvent ev;
  EXPECT_EQ(parseKey("\x1b[A", ev, false), 3u);
  EXPECT_EQ(ev.key, Key::UP);
  EXPECT_EQ(parseKey("\x1b[D", ev, false), 3u);
  EXPECT_EQ(ev.key, Key::LEFT);
  EXPECT_EQ(parseKey("\x1b", ev, false), 0u);  // wait for more bytes
  EXPECT_EQ(parseKey("\x1b", ev, true), 1u);
  EXPECT_EQ(ev.key, Key::ESCAPE);
  EXPECT_EQ(parseKey("\x1b[15~", ev, false), 5u);  // F5: ignored
  EXPECT_EQ(ev.key, Key::NONE);
  EXPECT_EQ(parseKey("W", ev, false), 1u);
  EXPECT_EQ(ev.key, Key::CHAR);
  EXPECT_EQ(ev.ch, 'w');
  EXPECT_EQ(parseKey(" ", ev, false), 1u);
  EXPECT_EQ(ev.key, Key::SPACE);
  EXPECT_EQ(parseKey("\x03", ev, false), 1u);
  EXPECT_EQ(ev.key, Key::CTRL_C);
}

TEST(KeyboardMapper, IncrementalSpeedAndStop)
{
  Limits lim;
  KeyboardMapper m(KeyboardParams{}, lim);
  bool quit = false;
  KeyEvent w{Key::CHAR, 'w'};
  auto out = m.apply(w, quit);
  ASSERT_TRUE(out.twist);
  EXPECT_NEAR(out.twist->vx, 0.03, 1e-9);
  for (int i = 0; i < 20; ++i) {m.apply(w, quit);}
  EXPECT_NEAR(m.twist().vx, lim.max_vx, 1e-9);
  EXPECT_FALSE(m.apply(w, quit).twist.has_value());  // saturated, no change
  out = m.apply({Key::LEFT, 0}, quit);
  EXPECT_GT(out.twist->wz, 0.0);
  out = m.apply({Key::SPACE, 0}, quit);
  EXPECT_TRUE(out.twist->isZero());
}

TEST(KeyboardMapper, CommandsEstopAndQuit)
{
  KeyboardMapper m(KeyboardParams{}, Limits{});
  bool quit = false;
  m.apply({Key::CHAR, 'w'}, quit);
  auto out = m.apply({Key::ESCAPE, 0}, quit);
  EXPECT_TRUE(out.estop.value_or(false));
  EXPECT_TRUE(out.twist->isZero());
  EXPECT_FALSE(m.apply({Key::CHAR, 'r'}, quit).estop.value_or(true));
  EXPECT_EQ(m.apply({Key::CHAR, '1'}, quit).command.value_or(""), "stand");
  EXPECT_EQ(m.apply({Key::CHAR, '2'}, quit).command.value_or(""), "lie");
  EXPECT_EQ(m.apply({Key::CHAR, '3'}, quit).command.value_or(""), "greet");
  EXPECT_EQ(m.apply({Key::CHAR, '4'}, quit).command.value_or(""), "survey");
  EXPECT_NEAR(m.apply({Key::CHAR, '+'}, quit).height.value_or(0), 0.01, 1e-9);
  out = m.apply({Key::CTRL_C, 0}, quit);
  EXPECT_TRUE(quit);
  ASSERT_TRUE(out.twist);
  EXPECT_TRUE(out.twist->isZero());
}
