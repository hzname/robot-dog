/**
 * @file imu_node_main.cpp
 * @brief Main entry point for IMU node
 */

#include <memory>

#include "dog_sensors_cpp/imu_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/transitions.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<dog_sensors_cpp::ImuNode>();

  // Automatically transition to ACTIVE state
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Use SingleThreadedExecutor (sufficient for sensor publishing)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(), "IMU node starting (ACTIVE)...");

  // Spin until shutdown
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
