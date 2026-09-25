// Publishes a TeleopOutput onto the robot's command topics.
#pragma once

#include "dog_teleop/mapping.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace dog_teleop
{

inline Limits declareLimits(rclcpp::Node & node)
{
  Limits l;
  l.max_vx = node.declare_parameter("max_vx", l.max_vx);
  l.max_vy = node.declare_parameter("max_vy", l.max_vy);
  l.max_wz = node.declare_parameter("max_wz", l.max_wz);
  l.max_pitch = node.declare_parameter("max_pitch", l.max_pitch);
  l.min_height = node.declare_parameter("min_height", l.min_height);
  l.max_height = node.declare_parameter("max_height", l.max_height);
  return l;
}

class TeleopPublisher
{
public:
  explicit TeleopPublisher(rclcpp::Node & node)
  {
    twist_pub_ = node.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    command_pub_ = node.create_publisher<std_msgs::msg::String>("command", 10);
    estop_pub_ = node.create_publisher<std_msgs::msg::Bool>("estop", rclcpp::QoS(10).reliable());
    pose_pub_ = node.create_publisher<geometry_msgs::msg::Vector3>("body_pose", 10);
  }

  void publish(const TeleopOutput & out)
  {
    if (out.estop) {
      std_msgs::msg::Bool m;
      m.data = *out.estop;
      estop_pub_->publish(m);
    }
    if (out.command) {
      std_msgs::msg::String m;
      m.data = *out.command;
      command_pub_->publish(m);
    }
    if (out.twist) {
      geometry_msgs::msg::Twist m;
      m.linear.x = out.twist->vx;
      m.linear.y = out.twist->vy;
      m.angular.z = out.twist->wz;
      twist_pub_->publish(m);
    }
    if (out.pitch || out.height) {
      if (out.pitch) {pitch_ = *out.pitch;}
      if (out.height) {height_ = *out.height;}
      geometry_msgs::msg::Vector3 m;
      m.y = pitch_;
      m.z = height_;
      pose_pub_->publish(m);
    }
  }

private:
  double pitch_{0.0};
  double height_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_pub_;
};

}  // namespace dog_teleop
