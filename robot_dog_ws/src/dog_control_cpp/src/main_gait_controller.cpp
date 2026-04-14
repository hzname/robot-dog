#include "dog_control_cpp/gait_controller.hpp"
#include "dog_control_cpp/lifecycle_helper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_control_cpp::GaitController>();

  // Automatically transition to ACTIVE state
  dog_monitor_cpp::lifecycle_activate(node);

  // MultiThreadedExecutor: timer callback on one thread, subscribers on another
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
