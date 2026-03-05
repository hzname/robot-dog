/**
 * @file imu_node_main.cpp
 * @brief Main entry point for IMU node
 */

#include <memory>

#include "dog_sensors_cpp/imu_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<dog_sensors_cpp::ImuNode>();

  // Use SingleThreadedExecutor (sufficient for sensor publishing)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(), "IMU node starting...");
  
  // Spin until shutdown
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
