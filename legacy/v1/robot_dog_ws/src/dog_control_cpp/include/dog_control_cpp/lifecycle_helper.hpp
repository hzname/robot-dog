#ifndef DOG_CONTROL_CPP__LIFECYCLE_HELPER_HPP_
#define DOG_CONTROL_CPP__LIFECYCLE_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace dog_monitor_cpp
{

inline bool lifecycle_activate(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  uint8_t transitions[] = {
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE
  };

  for (uint8_t tid : transitions) {
    auto client = node->create_client<lifecycle_msgs::srv::ChangeState>(
      node->get_name() + std::string("/change_state"));
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(node->get_logger(), "Lifecycle service unavailable");
      return false;
    }
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = tid;
    auto result = client->async_send_request(req);
    rclcpp::spin_until_future_complete(node->get_node_base_interface(), result);
  }
  return true;
}

}  // namespace dog_monitor_cpp

#endif  // DOG_CONTROL_CPP__LIFECYCLE_HELPER_HPP_
