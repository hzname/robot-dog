#include "dog_teleop_cpp/udp_teleop.hpp"

#include <rclcpp/qos.hpp>
#include <rcutils/allocator.h>

#include <cmath>
#include <algorithm>
#include <sstream>

namespace dog_teleop_cpp
{

UdpTeleop::UdpTeleop(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("udp_teleop", options),
  max_linear_speed_(0.5),
  max_angular_speed_(2.0),
  max_lateral_speed_(0.3),
  publish_rate_(50.0),
  watchdog_timeout_(0.5),
  udp_port_(8888),
  max_packet_size_(256),
  running_(false),
  socket_ready_(false),
  emergency_stop_(false),
  socket_fd_(-1)
{
  declareParameters();
}

UdpTeleop::~UdpTeleop()
{
  on_shutdown(get_current_state());
}

void UdpTeleop::declareParameters()
{
  declare_parameter("max_linear_speed", max_linear_speed_);
  declare_parameter("max_angular_speed", max_angular_speed_);
  declare_parameter("max_lateral_speed", max_lateral_speed_);
  declare_parameter("publish_rate", publish_rate_);
  declare_parameter("watchdog_timeout", watchdog_timeout_);
  declare_parameter("udp_port", udp_port_);
  declare_parameter("max_packet_size", static_cast<int>(max_packet_size_));
}

LifecycleCallbackReturn UdpTeleop::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring UDP Teleop...");

  // Get parameters
  max_linear_speed_ = get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_parameter("max_angular_speed").as_double();
  max_lateral_speed_ = get_parameter("max_lateral_speed").as_double();
  publish_rate_ = get_parameter("publish_rate").as_double();
  watchdog_timeout_ = get_parameter("watchdog_timeout").as_double();
  udp_port_ = get_parameter("udp_port").as_int();
  max_packet_size_ = static_cast<size_t>(get_parameter("max_packet_size").as_int());

  // QoS
  rclcpp::QoS cmd_qos(5);
  cmd_qos.reliable();

  // Create publishers (not activated yet)
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);
  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop_trigger", 10);

  // Allocate receive buffer
  receive_buffer_.resize(max_packet_size_);

  RCLCPP_INFO(get_logger(), "UDP Teleop configured (port: %d)", udp_port_);
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn UdpTeleop::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating UDP Teleop...");

  // Setup socket
  if (!setupSocket()) {
    RCLCPP_ERROR(get_logger(), "Failed to setup UDP socket");
    return LifecycleCallbackReturn::ERROR;
  }

  // Activate publishers
  cmd_vel_pub_->on_activate();
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

  // Start receive thread
  running_ = true;
  receive_thread_ = std::thread(&UdpTeleop::receiveLoop, this);

  last_command_time_ = now();

  RCLCPP_INFO(get_logger(), "UDP Teleop activated on port %d", udp_port_);
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn UdpTeleop::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating UDP Teleop...");

  running_ = false;
  socket_ready_ = false;

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  closeSocket();

  // Cancel timers
  publish_timer_.reset();
  watchdog_timer_.reset();

  // Deactivate publishers
  cmd_vel_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "UDP Teleop deactivated");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn UdpTeleop::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up UDP Teleop...");

  cmd_vel_pub_.reset();
  emergency_stop_pub_.reset();
  receive_buffer_.clear();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn UdpTeleop::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down UDP Teleop...");

  running_ = false;
  socket_ready_ = false;

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  closeSocket();

  return LifecycleCallbackReturn::SUCCESS;
}

bool UdpTeleop::setupSocket()
{
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to create socket: %s", strerror(errno));
    return false;
  }

  // Set non-blocking
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to set non-blocking: %s", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Allow address reuse
  int opt = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    RCLCPP_WARN(get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
  }

  // Bind
  std::memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_addr.s_addr = INADDR_ANY;
  server_addr_.sin_port = htons(udp_port_);

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&server_addr_), sizeof(server_addr_)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind to port %d: %s", udp_port_, strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  socket_ready_ = true;
  return true;
}

void UdpTeleop::closeSocket()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  socket_ready_ = false;
}

void UdpTeleop::receiveLoop()
{
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  while (running_ && rclcpp::ok()) {
    if (!socket_ready_ || socket_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    ssize_t len = recvfrom(socket_fd_, receive_buffer_.data(), receive_buffer_.size() - 1, 0,
                           reinterpret_cast<struct sockaddr*>(&client_addr), &addr_len);

    if (len > 0) {
      receive_buffer_[len] = '\0';
      parseAndExecute(receive_buffer_.data(), len);
      
      {
        std::lock_guard<std::mutex> lock(time_mutex_);
        last_command_time_ = now();
      }
    } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "recvfrom error: %s", strerror(errno));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void UdpTeleop::parseAndExecute(const char* buffer, ssize_t len)
{
  // Simple protocol: "vx,vy,wz,stop" or JSON-like
  double vx = 0.0, vy = 0.0, wz = 0.0;
  bool stop = false;

  // Try parsing as comma-separated: "vx,vy,wz,stop"
  int parsed = sscanf(buffer, "%lf,%lf,%lf,%d", &vx, &vy, &wz, &stop);
  
  if (parsed >= 3) {
    if (stop || emergency_stop_) {
      resetVelocity();
      return;
    }

    // Clamp velocities
    vx = std::max(-max_linear_speed_, std::min(max_linear_speed_, vx));
    vy = std::max(-max_lateral_speed_, std::min(max_lateral_speed_, vy));
    wz = std::max(-max_angular_speed_, std::min(max_angular_speed_, wz));

    {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      current_twist_.linear.x = vx;
      current_twist_.linear.y = vy;
      current_twist_.angular.z = wz;
    }
    return;
  }

  // Try simple single character commands
  if (len > 0) {
    switch (buffer[0]) {
      case 'S':  // Stop
      case 's':
        resetVelocity();
        break;
      case 'E':  // Emergency stop
      case 'e':
        emergency_stop_ = true;
        {
          auto msg = std::make_unique<std_msgs::msg::Bool>();
          msg->data = true;
          emergency_stop_pub_->publish(std::move(msg));
        }
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP via UDP!");
        break;
      case 'R':  // Reset emergency stop
      case 'r':
        emergency_stop_ = false;
        {
          auto msg = std::make_unique<std_msgs::msg::Bool>();
          msg->data = false;
          emergency_stop_pub_->publish(std::move(msg));
        }
        RCLCPP_INFO(get_logger(), "Emergency stop reset via UDP");
        break;
    }
  }
}

void UdpTeleop::resetVelocity()
{
  std::lock_guard<std::mutex> lock(twist_mutex_);
  current_twist_ = geometry_msgs::msg::Twist();
}

void UdpTeleop::publishZeroVelocity()
{
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_pub_->publish(std::move(msg));
}

void UdpTeleop::publishLoop()
{
  if (!cmd_vel_pub_->is_activated()) {
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

void UdpTeleop::watchdogLoop()
{
  if (!cmd_vel_pub_->is_activated()) {
    return;
  }

  rclcpp::Time last_time;
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    last_time = last_command_time_;
  }

  double elapsed = (now() - last_time).seconds();
  
  if (elapsed > watchdog_timeout_) {
    // No command received for too long - stop the robot
    resetVelocity();
    publishZeroVelocity();
    
    static rclcpp::Time last_warn = now();
    if ((now() - last_warn).seconds() > 2.0) {
      RCLCPP_WARN(get_logger(), "Watchdog: No command for %.2f seconds, stopping", elapsed);
      last_warn = now();
    }
  }
}

}  // namespace dog_teleop_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<dog_teleop_cpp::UdpTeleop>();
  exec.add_node(node->get_node_base_interface());
  
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
