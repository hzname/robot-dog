#include "dog_teleop_cpp/keyboard_teleop.hpp"

#include <rclcpp/qos.hpp>
#include <algorithm>

namespace dog_teleop_cpp
{

KeyboardTeleop::KeyboardTeleop(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("keyboard_teleop", options),
  max_linear_speed_(0.5),
  max_angular_speed_(2.0),
  max_lateral_speed_(0.3),
  publish_rate_(50.0),
  watchdog_timeout_(0.5),
  target_height_(0.25),
  emergency_stop_(false),
  running_(false),
  active_(false),
  terminal_configured_(false)
{
  declareParameters();
}

KeyboardTeleop::~KeyboardTeleop()
{
  on_shutdown(get_current_state());
}

void KeyboardTeleop::declareParameters()
{
  declare_parameter("max_linear_speed", max_linear_speed_);
  declare_parameter("max_angular_speed", max_angular_speed_);
  declare_parameter("max_lateral_speed", max_lateral_speed_);
  declare_parameter("publish_rate", publish_rate_);
  declare_parameter("watchdog_timeout", watchdog_timeout_);
}

LifecycleCallbackReturn KeyboardTeleop::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Keyboard Teleop...");

  // Get parameters
  max_linear_speed_ = get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_parameter("max_angular_speed").as_double();
  max_lateral_speed_ = get_parameter("max_lateral_speed").as_double();
  publish_rate_ = get_parameter("publish_rate").as_double();
  watchdog_timeout_ = get_parameter("watchdog_timeout").as_double();

  // Save terminal settings
  tcgetattr(STDIN_FILENO, &original_termios_);

  // QoS
  rclcpp::QoS cmd_qos(5);
  cmd_qos.reliable();

  // Create publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);
  cmd_height_pub_ = create_publisher<std_msgs::msg::Float64>("/cmd_height", 10);
  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop_trigger", 10);

  RCLCPP_INFO(get_logger(), "Keyboard Teleop configured");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn KeyboardTeleop::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Keyboard Teleop...");

  // Activate publishers
  cmd_vel_pub_->on_activate();
  cmd_height_pub_->on_activate();
  emergency_stop_pub_->on_activate();

  // Start timers
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publishLoop(); });

  auto watchdog_period = std::chrono::duration<double>(watchdog_timeout_ / 2.0);
  watchdog_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(watchdog_period),
    [this]() { watchdogLoop(); });

  // Start input thread
  running_ = true;
  active_ = true;
  last_key_time_ = now();
  input_thread_ = std::thread(&KeyboardTeleop::inputLoop, this);

  printInstructions();
  RCLCPP_INFO(get_logger(), "Keyboard Teleop activated");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn KeyboardTeleop::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Keyboard Teleop...");

  active_ = false;
  running_ = false;

  if (input_thread_.joinable()) {
    input_thread_.join();
  }

  // Restore terminal
  if (terminal_configured_) {
    tcsetattr(STDIN_FILENO, TCSADRAIN, &original_termios_);
    terminal_configured_ = false;
  }

  // Cancel timers
  publish_timer_.reset();
  watchdog_timer_.reset();

  // Deactivate publishers
  cmd_vel_pub_->on_deactivate();
  cmd_height_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Keyboard Teleop deactivated");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn KeyboardTeleop::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Keyboard Teleop...");

  cmd_vel_pub_.reset();
  cmd_height_pub_.reset();
  emergency_stop_pub_.reset();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn KeyboardTeleop::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Keyboard Teleop...");

  running_ = false;
  active_ = false;

  if (input_thread_.joinable()) {
    input_thread_.join();
  }

  if (terminal_configured_) {
    tcsetattr(STDIN_FILENO, TCSADRAIN, &original_termios_);
    terminal_configured_ = false;
  }

  return LifecycleCallbackReturn::SUCCESS;
}

void KeyboardTeleop::printInstructions()
{
  std::cout << R"(
========================================
  Keyboard Teleop - Robot Dog Control
========================================
Movement:
    w - Forward        s - Backward
    a - Left           d - Right
    q - Rotate left    e - Rotate right
    
    Space - Stop (zero velocity)

Height Control:
    + / = - Increase height
    - / _ - Decrease height

Safety:
    ESC   - Emergency stop
    r     - Reset emergency stop

Other:
    CTRL-C - Quit

========================================
)" << std::endl;
}

char KeyboardTeleop::getKey()
{
  struct termios raw;
  tcgetattr(STDIN_FILENO, &raw);
  raw.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  terminal_configured_ = true;

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  struct timeval timeout{0, 50000};  // 50ms timeout
  select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

  char key = 0;
  if (FD_ISSET(STDIN_FILENO, &readfds)) {
    read(STDIN_FILENO, &key, 1);
  }

  tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios_);
  terminal_configured_ = false;
  return key;
}

void KeyboardTeleop::inputLoop()
{
  while (running_ && rclcpp::ok()) {
    char key = getKey();

    if (key == 0) {
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      last_key_time_ = now();
    }

    if (key == 3) {  // CTRL-C
      running_ = false;
      rclcpp::shutdown();
      break;
    }

    switch (key) {
      case 27:  // ESC
        emergency_stop_ = true;
        publishEmergencyStop(true);
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP ACTIVATED!");
        break;

      case 'r':
      case 'R':
        emergency_stop_ = false;
        publishEmergencyStop(false);
        RCLCPP_INFO(get_logger(), "Emergency stop reset");
        break;

      case ' ':  // Space
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_ = geometry_msgs::msg::Twist();
      }
        RCLCPP_INFO(get_logger(), "Stop command");
        break;

      case 'w':
      case 'W':
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_.linear.x = max_linear_speed_;
        current_twist_.linear.y = 0.0;
      }
        break;

      case 's':
      case 'S':
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_.linear.x = -max_linear_speed_;
        current_twist_.linear.y = 0.0;
      }
        break;

      case 'a':
      case 'A':
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_.linear.y = max_lateral_speed_;
        current_twist_.linear.x = 0.0;
      }
        break;

      case 'd':
      case 'D':
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_.linear.y = -max_lateral_speed_;
        current_twist_.linear.x = 0.0;
      }
        break;

      case 'q':
      case 'Q':
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_.angular.z = max_angular_speed_;
      }
        break;

      case 'e':
      case 'E':
      {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        current_twist_.angular.z = -max_angular_speed_;
      }
        break;

      case '+':
      case '=':
        target_height_ = std::min(0.35, target_height_ + 0.01);
        publishHeight();
        RCLCPP_INFO(get_logger(), "Height: %.2f m", target_height_);
        break;

      case '-':
      case '_':
        target_height_ = std::max(0.15, target_height_ - 0.01);
        publishHeight();
        RCLCPP_INFO(get_logger(), "Height: %.2f m", target_height_);
        break;

      default:
        break;
    }
  }
}

void KeyboardTeleop::publishLoop()
{
  if (!active_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  if (emergency_stop_) {
    publishZeroVelocity();
    return;
  }

  geometry_msgs::msg::Twist twist;
  {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    twist = current_twist_;
  }

  auto msg = std::make_unique<geometry_msgs::msg::Twist>(twist);
  cmd_vel_pub_->publish(std::move(msg));
}

void KeyboardTeleop::watchdogLoop()
{
  if (!active_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  rclcpp::Time last_time;
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    last_time = last_key_time_;
  }

  double elapsed = (now() - last_time).seconds();

  if (elapsed > watchdog_timeout_) {
    // No key pressed for too long - stop the robot
    {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      current_twist_ = geometry_msgs::msg::Twist();
    }
    publishZeroVelocity();

    static rclcpp::Time last_warn = now();
    if ((now() - last_warn).seconds() > 2.0) {
      RCLCPP_WARN(get_logger(), "Watchdog: No input for %.2f seconds, stopping", elapsed);
      last_warn = now();
    }
  }
}

void KeyboardTeleop::publishHeight()
{
  if (!active_ || !cmd_height_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<std_msgs::msg::Float64>();
  msg->data = target_height_;
  cmd_height_pub_->publish(std::move(msg));
}

void KeyboardTeleop::publishEmergencyStop(bool stop)
{
  if (!active_ || !emergency_stop_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = stop;
  emergency_stop_pub_->publish(std::move(msg));
}

void KeyboardTeleop::publishZeroVelocity()
{
  if (!cmd_vel_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_pub_->publish(std::move(msg));
}

}  // namespace dog_teleop_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<dog_teleop_cpp::KeyboardTeleop>();
  exec.add_node(node->get_node_base_interface());

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
