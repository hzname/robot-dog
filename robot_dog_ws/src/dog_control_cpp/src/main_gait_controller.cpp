/**
 * @file main_gait_controller.cpp
 * @brief Main entry point for the gait controller node
 */

#include "dog_control_cpp/gait_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<dog_control_cpp::GaitController>(options);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}