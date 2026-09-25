#include "dog_monitor_cpp/health_monitor.hpp"

#include <rclcpp/client.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <sstream>
#include <unordered_set>

namespace dog_monitor_cpp
{

HealthMonitor::HealthMonitor(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("health_monitor", options)
{
  declare_parameter("check_interval_s", check_interval_s_);
  declare_parameter("node_timeout_s", node_timeout_s_);
  declare_parameter("max_restarts", max_restarts_);

  // List of nodes to monitor
  declare_parameter("monitored_nodes", std::vector<std::string>{
    "/gait_controller",
    "/servo_driver_node",
    "/imu_node",
    "/balance_controller"
  });
}

LifecycleCallbackReturn HealthMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Health Monitor...");

  check_interval_s_ = get_parameter("check_interval_s").as_double();
  node_timeout_s_ = get_parameter("node_timeout_s").as_double();
  max_restarts_ = get_parameter("max_restarts").as_int();

  auto node_names = get_parameter("monitored_nodes").as_string_array();

  // Initialize monitored nodes list
  for (const auto & name : node_names) {
    MonitoredNode node;
    node.name = name;
    node.lifecycle_service = name + "/change_state";
    node.last_seen = now();
    node.is_healthy = true;
    node.consecutive_failures = 0;
    node.max_failures_before_restart = static_cast<int>(node_timeout_s_ / check_interval_s_);
    nodes_.push_back(node);
  }

  RCLCPP_INFO(get_logger(), "Monitoring %zu nodes (timeout: %.1f s, interval: %.1f s)",
              nodes_.size(), node_timeout_s_, check_interval_s_);
  for (const auto & n : nodes_) {
    RCLCPP_INFO(get_logger(), "  - %s (restart after %d misses)", n.name.c_str(), n.max_failures_before_restart);
  }

  // Publishers
  system_health_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/health", rclcpp::QoS(1).reliable());

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(5).reliable());

  RCLCPP_INFO(get_logger(), "Health Monitor configured");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn HealthMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Health Monitor...");

  system_health_pub_->on_activate();
  diag_pub_->on_activate();

  // Give other nodes time to start up — initialize last_seen to now
  auto now_time = now();
  for (auto & node : nodes_) {
    node.last_seen = now_time;
  }

  // Start check loop
  check_timer_ = create_wall_timer(
    std::chrono::duration<double>(check_interval_s_),
    [this]() { checkLoop(); });

  RCLCPP_INFO(get_logger(), "Health Monitor active (check every %.1f s)", check_interval_s_);
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn HealthMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Health Monitor...");

  if (check_timer_) {
    check_timer_->cancel();
  }

  system_health_pub_->on_deactivate();
  diag_pub_->on_deactivate();

  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn HealthMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Health Monitor...");
  if (check_timer_) {
    check_timer_->cancel();
  }
  return LifecycleCallbackReturn::SUCCESS;
}

void HealthMonitor::checkLoop()
{
  checkNodeLiveness();
  publishHealth();
}

void HealthMonitor::checkNodeLiveness()
{
  // Get all known node names from the ROS graph
  auto node_names = get_node_names();  // vector<string>
  std::unordered_set<std::string> alive_nodes(node_names.begin(), node_names.end());

  bool all_healthy = true;

  for (auto & node : nodes_) {
    bool is_alive = alive_nodes.count(node.name) > 0;

    if (is_alive) {
      node.last_seen = now();
      node.consecutive_failures = 0;
      if (!node.is_healthy) {
        RCLCPP_INFO(get_logger(), "Node %s recovered", node.name.c_str());
      }
      node.is_healthy = true;
    } else {
      auto elapsed = (now() - node.last_seen).seconds();
      node.consecutive_failures++;

      if (elapsed > node_timeout_s_) {
        node.is_healthy = false;
        all_healthy = false;

        if (node.consecutive_failures >= node.max_failures_before_restart &&
            total_restart_attempts_ < max_restarts_) {
          RCLCPP_WARN(get_logger(), "Node %s dead for %.1f s — attempting restart (attempt %d/%d)",
                      node.name.c_str(), elapsed, total_restart_attempts_ + 1, max_restarts_);
          if (restartNode(node.name)) {
            RCLCPP_INFO(get_logger(), "Restart initiated for %s", node.name.c_str());
            total_restart_attempts_++;
            node.consecutive_failures = 0;  // Reset to give it a fresh window
          } else {
            RCLCPP_ERROR(get_logger(), "Failed to initiate restart for %s", node.name.c_str());
          }
        } else if (node.consecutive_failures >= node.max_failures_before_restart) {
          RCLCPP_ERROR(get_logger(), "Node %s dead — max restart attempts reached (%d)",
                       node.name.c_str(), max_restarts_);
        }
      }
    }
  }

  system_healthy_ = all_healthy;
}

bool HealthMonitor::restartNode(const std::string & node_name)
{
  // Create a client to the lifecycle change_state service
  auto client = create_client<lifecycle_msgs::srv::ChangeState>(
    node_name + "/change_state");

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "Lifecycle service not available for %s", node_name.c_str());
    return false;
  }

  // Transition: unconfigured → inactive (configure) → active (activate)
  // If node is already active, configure will fail gracefully
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  auto result = client->async_send_request(request);

  // Wait for result (blocking, but this is a rare event — node restart)
  auto future = result.get();
  if (!future->success) {
    RCLCPP_WARN(get_logger(), "Configure failed for %s, trying deactivate first", node_name.c_str());
    // Try deactivating first
    client = create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      return false;
    }
    request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    client->async_send_request(request);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Then configure
    client = create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      return false;
    }
    request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    client->async_send_request(request);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Finally activate
  client = create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    return false;
  }
  request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  client->async_send_request(request);

  return true;
}

void HealthMonitor::publishHealth()
{
  // Publish aggregate health
  auto health_msg = std::make_unique<std_msgs::msg::Bool>();
  health_msg->data = system_healthy_;
  system_health_pub_->publish(std::move(health_msg));

  // Publish detailed diagnostics
  auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
  diag_msg.header.stamp = now();

  for (const auto & node : nodes_) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "robot_dog/node/" + node.name.substr(1);  // Remove leading /
    status.hardware_id = "robot_dog";

    if (node.is_healthy) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Node is alive and responding";
    } else {
      auto elapsed = (now() - node.last_seen).seconds();
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Node unresponsive for " + std::to_string(elapsed) + " seconds";
    }

    // Add key-value details
    diagnostic_msgs::msg::KeyValue kv_last_seen;
    kv_last_seen.key = "last_seen";
    kv_last_seen.value = std::to_string(node.last_seen.seconds()) + " s";
    status.values.push_back(kv_last_seen);

    diagnostic_msgs::msg::KeyValue kv_failures;
    kv_failures.key = "consecutive_failures";
    kv_failures.value = std::to_string(node.consecutive_failures);
    status.values.push_back(kv_failures);

    diag_msg.status.push_back(status);
  }

  // Add system-level status
  {
    diagnostic_msgs::msg::DiagnosticStatus sys_status;
    sys_status.name = "robot_dog/system";
    sys_status.hardware_id = "robot_dog";
    sys_status.level = system_healthy_
      ? diagnostic_msgs::msg::DiagnosticStatus::OK
      : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    sys_status.message = system_healthy_ ? "All systems nominal" : "Degraded — some nodes unresponsive";
    diag_msg.status.push_back(sys_status);
  }

  diag_pub_->publish(diag_msg);
}

}  // namespace dog_monitor_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<dog_monitor_cpp::HealthMonitor>();

  // Auto-transition to ACTIVE
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
