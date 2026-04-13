#ifndef DOG_MONITOR_CPP__HEALTH_MONITOR_HPP_
#define DOG_MONITOR_CPP__HEALTH_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

namespace dog_monitor_cpp
{

struct MonitoredNode
{
  std::string name;
  std::string lifecycle_service;    // e.g. "/gait_controller/change_state"
  rclcpp::Time last_seen;           // Last time node was detected alive
  bool is_healthy;
  int consecutive_failures;
  int max_failures_before_restart;  // How many misses before attempting restart
};

class HealthMonitor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HealthMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Lifecycle callbacks
  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Main check loop
  void checkLoop();

  // Check if nodes are alive using ros2 node list (via rclcpp API)
  void checkNodeLiveness();

  // Attempt lifecycle restart on a failed node
  bool restartNode(const std::string & node_name);

  // Publish aggregated health
  void publishHealth();

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr system_health_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr check_timer_;

  // Configuration
  double check_interval_s_{1.0};       // How often to check nodes
  double node_timeout_s_{3.0};         // How long before a node is considered dead
  int max_restarts_{3};                // Max restart attempts before giving up

  // State
  std::vector<MonitoredNode> nodes_;
  int total_restart_attempts_{0};
  bool system_healthy_{true};
};

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

}  // namespace dog_monitor_cpp

#endif  // DOG_MONITOR_CPP__HEALTH_MONITOR_HPP_
