// keyboard_teleop: drive the dog from a terminal (locally or over SSH).
//
// Terminals only report key presses (no releases), so speed keys are
// incremental: each press adds a step, Space stops. While moving, cmd_vel is
// re-sent at 10 Hz so the locomotion timeout never fires; quitting (Ctrl-C)
// or losing the SSH session sends a final stop.
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "dog_teleop/mapping.hpp"
#include "dog_teleop/teleop_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dog_teleop
{

class RawTerminal
{
public:
  RawTerminal()
  {
    ok_ = ::isatty(STDIN_FILENO) && ::tcgetattr(STDIN_FILENO, &saved_) == 0;
    if (ok_) {
      termios raw = saved_;
      raw.c_lflag &= ~(ICANON | ECHO | ISIG);  // ISIG off: Ctrl-C arrives as a key
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
  }
  ~RawTerminal()
  {
    if (ok_) {::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);}
  }
  bool ok() const {return ok_;}

private:
  termios saved_{};
  bool ok_{false};
};

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : rclcpp::Node("keyboard_teleop"), publisher_(*this)
  {
    KeyboardParams kp;
    kp.step_v = declare_parameter("step_v", kp.step_v);
    kp.step_w = declare_parameter("step_w", kp.step_w);
    kp.step_h = declare_parameter("step_h", kp.step_h);
    mapper_ = std::make_unique<KeyboardMapper>(kp, declareLimits(*this));
    state_sub_ = create_subscription<std_msgs::msg::String>(
      "state", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = msg->data;
        dirty_ = true;
      });
    repeat_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!mapper_->twist().isZero()) {
          TeleopOutput out;
          out.twist = mapper_->twist();
          publisher_.publish(out);
        }
      });
  }

  /// Blocking key loop; returns when the user quits or ROS shuts down.
  void run()
  {
    RawTerminal term;
    if (!term.ok()) {
      RCLCPP_ERROR(get_logger(),
        "stdin is not a terminal. Run in a real TTY: `ros2 run dog_teleop keyboard_teleop` "
        "(not through `ros2 launch`).");
      return;
    }
    std::fputs(keyboardHelp(), stdout);
    std::string buf;
    bool quit = false;
    while (rclcpp::ok() && !quit) {
      pollfd pfd{STDIN_FILENO, POLLIN, 0};
      const int ready = ::poll(&pfd, 1, 50);
      if (ready > 0 && (pfd.revents & (POLLHUP | POLLERR))) {
        break;  // SSH session gone
      }
      if (ready > 0) {
        char tmp[64];
        const ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
        if (n <= 0) {break;}
        buf.append(tmp, static_cast<size_t>(n));
      }
      const bool idle = ready == 0;
      KeyEvent ev;
      size_t used = 0;
      while (!quit && (used = parseKey(buf, ev, idle)) > 0) {
        buf.erase(0, used);
        if (ev.key == Key::CHAR && ev.ch == 'h') {
          std::fputs("\n", stdout);
          std::fputs(keyboardHelp(), stdout);
          continue;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto out = mapper_->apply(ev, quit);
        publisher_.publish(out);
        if (out.estop) {last_event_ = *out.estop ? "E-STOP ENGAGED" : "e-stop released";}
        if (out.command) {last_event_ = *out.command;}
        dirty_ = true;
      }
      drawStatus();
    }
    // Always leave the robot stopped.
    std::lock_guard<std::mutex> lock(mutex_);
    TeleopOutput stop;
    stop.twist = Twist2D{};
    publisher_.publish(stop);
    std::fputs("\nkeyboard teleop: stopped\n", stdout);
  }

private:
  void drawStatus()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dirty_) {return;}
    dirty_ = false;
    const auto & t = mapper_->twist();
    std::printf("\r\033[K vx %+.2f m/s  vy %+.2f m/s  wz %+.2f rad/s  height %+.0f mm | mode: %s | %s",
      t.vx, t.vy, t.wz, mapper_->height() * 1000.0, mode_.c_str(), last_event_.c_str());
    std::fflush(stdout);
  }

  TeleopPublisher publisher_;
  std::unique_ptr<KeyboardMapper> mapper_;
  std::mutex mutex_;
  std::string mode_{"unknown"};
  std::string last_event_;
  bool dirty_{true};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr repeat_timer_;
};

}  // namespace dog_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_teleop::KeyboardTeleopNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});
  node->run();
  // Give the final stop message a moment to leave before shutting down.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
