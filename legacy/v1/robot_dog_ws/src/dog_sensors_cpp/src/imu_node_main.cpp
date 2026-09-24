/**
 * @file imu_node_main.cpp
 * @brief Main entry point for IMU node
 */

#include <memory>

#include "dog_sensors_cpp/imu_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

static bool transition(rclcpp_lifecycle::LifecycleNode::SharedPtr node, uint8_t id)
{
  auto client = node->create_client<lifecycle_msgs::srv::ChangeState>(
    node->get_name() + std::string("/change_state"));
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(node->get_logger(), "Lifecycle service not available");
    return false;
  }
  auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  req->transition.id = id;
  auto result = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return result.get()->success;
  }
  return false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<dog_sensors_cpp::ImuNode>();

  // Automatically transition to ACTIVE state
  transition(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  transition(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Use SingleThreadedExecutor (sufficient for sensor publishing)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(), "IMU node starting (ACTIVE)...");

  // Spin until shutdown
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
