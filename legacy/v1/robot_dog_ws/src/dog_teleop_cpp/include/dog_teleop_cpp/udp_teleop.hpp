#ifndef DOG_TELEOP_CPP__UDP_TELEOP_HPP_
#define DOG_TELEOP_CPP__UDP_TELEOP_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>

namespace dog_teleop_cpp
{

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief UDP teleoperation node (LifecycleNode)
 * 
 * Receives teleop commands via UDP packets.
 * Protocol: JSON or binary format
 * - {"vx": 0.5, "vy": 0.0, "wz": 0.0, "stop": false}
 * 
 * Port: 8888 (configurable)
 */
class UdpTeleop : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UdpTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UdpTeleop() override;

  // Lifecycle transitions
  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  bool setupSocket();
  void closeSocket();
  void receiveLoop();
  void publishLoop();
  void watchdogLoop();
  void parseAndExecute(const char* buffer, ssize_t len);
  void resetVelocity();
  void publishZeroVelocity();

  // Parameters
  double max_linear_speed_;
  double max_angular_speed_;
  double max_lateral_speed_;
  double publish_rate_;
  double watchdog_timeout_;
  int udp_port_;
  size_t max_packet_size_;

  // State
  geometry_msgs::msg::Twist current_twist_;
  std::mutex twist_mutex_;
  std::atomic<bool> running_;
  std::atomic<bool> socket_ready_;
  std::atomic<bool> emergency_stop_;
  rclcpp::Time last_command_time_;
  std::mutex time_mutex_;

  // Socket
  int socket_fd_;
  struct sockaddr_in server_addr_;

  // Publishers (lifecycle)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Thread
  std::thread receive_thread_;
  std::vector<char> receive_buffer_;
};

}  // namespace dog_teleop_cpp

#endif  // DOG_TELEOP_CPP__UDP_TELEOP_HPP_
