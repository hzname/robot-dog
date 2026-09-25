// locomotion_node: operator commands -> joint targets.
//
// Subscribes (relative names, launched in the /dog namespace):
//   cmd_vel        geometry_msgs/Twist    linear.x/y [m/s], angular.z [rad/s]
//   command        std_msgs/String        "stand" | "lie"
//   body_pose      geometry_msgs/Vector3  x=roll [rad], y=pitch [rad], z=height offset [m]
//   estop          std_msgs/Bool          true = limp, requires "stand" after release
//   imu/data       sensor_msgs/Imu        optional: slope compensation and heading hold
// Publishes:
//   joint_commands sensor_msgs/JointState 12 joint positions [rad]
//   state          std_msgs/String        current mode, or "estop" (latched)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "dog_control/locomotion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace dog_control
{

const std::vector<std::string> kJointNames = {
  "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
  "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
  "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
  "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
};

class LocomotionNode : public rclcpp::Node
{
public:
  explicit LocomotionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("locomotion", options)
  {
    const LocomotionParams p = loadParams();
    controller_ = std::make_unique<LocomotionController>(p);
    rate_ = declare_parameter("control_rate", 50.0);
    cmd_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("state", latched);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        controller_->setVelocity({msg->linear.x, msg->linear.y, msg->angular.z});
        last_cmd_vel_ = now();
        cmd_vel_active_ = true;
      });
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (controller_->request(msg->data)) {
          RCLCPP_INFO(get_logger(), "command '%s' accepted", msg->data.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "command '%s' rejected (mode %s%s)", msg->data.c_str(),
            modeName(controller_->mode()), controller_->estopActive() ? ", e-stop active" : "");
        }
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "body_pose", 10, [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
        controller_->setBodyPose({msg->x, msg->y, msg->z});
      });
    // E-stop is volatile on purpose: with several latched publishers a late
    // subscriber would get their last values in undefined order. Nodes start
    // in a safe state (PASSIVE / servos off) instead.
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "estop", rclcpp::QoS(10).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != controller_->estopActive()) {
          RCLCPP_WARN(get_logger(), "E-STOP %s", msg->data ? "ENGAGED" : "released");
        }
        controller_->setEstop(msg->data);
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        controller_->setYawRate(msg->angular_velocity.z);
        last_gyro_ = now();
        const auto & q = msg->orientation;
        if (msg->orientation_covariance[0] < 0.0) {return;}  // no orientation in this message
        const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        const double pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
        const auto t = now();
        const double dt = imu_seen_ ? std::clamp((t - last_imu_).seconds(), 0.0, 0.2) : 0.0;
        if (!imu_seen_) {
          RCLCPP_INFO(get_logger(), "IMU data received - slope compensation and heading hold active");
        }
        imu_seen_ = true;
        last_imu_ = t;
        controller_->setImuAttitude(roll, pitch, dt);
      });

    joint_msg_.name = kJointNames;
    joint_msg_.position.resize(kNumJoints);
    last_tick_ = now();
    last_cmd_vel_ = now();
    // Node clock: follows /clock when use_sim_time is set (Gazebo).
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this]() {tick();});
    publishState();
    RCLCPP_INFO(get_logger(), "locomotion ready at %.0f Hz (send \"stand\" on ~/command)", rate_);
  }

private:
  LocomotionParams loadParams()
  {
    LocomotionParams p;
    p.leg.hip = declare_parameter("geometry.hip_offset", p.leg.hip);
    p.leg.thigh = declare_parameter("geometry.thigh", p.leg.thigh);
    p.leg.calf = declare_parameter("geometry.calf", p.leg.calf);
    p.hip_x = declare_parameter("geometry.hip_x", p.hip_x);
    p.hip_y = declare_parameter("geometry.hip_y", p.hip_y);
    p.knee_direction = static_cast<int>(declare_parameter("geometry.knee_direction", -1));

    p.foot_offset_x = declare_parameter("stance.foot_offset_x", p.foot_offset_x);
    p.foot_offset_y = declare_parameter("stance.foot_offset_y", p.foot_offset_y);
    p.stand_height = declare_parameter("stance.stand_height", p.stand_height);
    p.lie_height = declare_parameter("stance.lie_height", p.lie_height);
    p.min_height = declare_parameter("stance.min_height", p.min_height);
    p.max_height = declare_parameter("stance.max_height", p.max_height);
    p.transition_time = declare_parameter("stance.transition_time", p.transition_time);
    p.max_roll = declare_parameter("stance.max_roll", p.max_roll);
    p.max_pitch = declare_parameter("stance.max_pitch", p.max_pitch);

    p.gait.period = declare_parameter("gait.period", p.gait.period);
    p.gait.duty = declare_parameter("gait.duty", p.gait.duty);
    p.gait.step_height = declare_parameter("gait.step_height", p.gait.step_height);
    p.gait.max_step = declare_parameter("gait.max_step", p.gait.max_step);

    p.slope_compensation = declare_parameter("slope.compensation", p.slope_compensation);
    p.slope_gain = declare_parameter("slope.gain", p.slope_gain);
    p.slope_filter_tau = declare_parameter("slope.filter_tau", p.slope_filter_tau);
    p.slope_max_shift = declare_parameter("slope.max_shift", p.slope_max_shift);
    p.slope_max_deg = declare_parameter("slope.max_deg", p.slope_max_deg);
    p.heading_hold = declare_parameter("heading.hold", p.heading_hold);
    p.heading_kp = declare_parameter("heading.kp", p.heading_kp);
    p.heading_ki = declare_parameter("heading.ki", p.heading_ki);
    p.heading_max_rate = declare_parameter("heading.max_rate", p.heading_max_rate);
    p.heading_max_error = declare_parameter("heading.max_error", p.heading_max_error);

    p.max_velocity.vx = declare_parameter("limits.max_vx", p.max_velocity.vx);
    p.max_velocity.vy = declare_parameter("limits.max_vy", p.max_velocity.vy);
    p.max_velocity.wz = declare_parameter("limits.max_wz", p.max_velocity.wz);
    p.max_accel.vx = declare_parameter("limits.accel_vx", p.max_accel.vx);
    p.max_accel.vy = declare_parameter("limits.accel_vy", p.max_accel.vy);
    p.max_accel.wz = declare_parameter("limits.accel_wz", p.max_accel.wz);
    return p;
  }

  void tick()
  {
    const auto t = now();
    const double dt = std::clamp((t - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = t;

    if (cmd_vel_active_ && (t - last_cmd_vel_).seconds() > cmd_timeout_) {
      controller_->setVelocity({});
      cmd_vel_active_ = false;
      RCLCPP_WARN(get_logger(), "cmd_vel timeout (%.2fs) - stopping", cmd_timeout_);
    }

    if (last_gyro_.nanoseconds() != 0 && (t - last_gyro_).seconds() > 0.2) {
      controller_->clearYawRate();  // IMU silent: no heading hold on stale data
    }
    const bool out = controller_->update(dt);
    if (controller_->mode() != last_mode_ || controller_->estopActive() != last_estop_) {
      publishState();
    }
    if (controller_->unreachableCount() > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%d foot target(s) outside the leg workspace - clamped", controller_->unreachableCount());
    }
    if (!out) {
      return;
    }
    joint_msg_.header.stamp = t;
    const auto & q = controller_->joints();
    std::copy(q.begin(), q.end(), joint_msg_.position.begin());
    joint_pub_->publish(joint_msg_);
  }

  void publishState()
  {
    last_mode_ = controller_->mode();
    last_estop_ = controller_->estopActive();
    std_msgs::msg::String msg;
    msg.data = last_estop_ ? "estop" : modeName(last_mode_);
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "mode -> %s", msg.data.c_str());
  }

  std::unique_ptr<LocomotionController> controller_;
  double rate_{50.0};
  double cmd_timeout_{0.5};
  bool cmd_vel_active_{false};
  Mode last_mode_{Mode::PASSIVE};
  bool last_estop_{false};
  bool imu_seen_{false};
  rclcpp::Time last_imu_;
  rclcpp::Time last_gyro_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_;
  rclcpp::Time last_cmd_vel_;
  sensor_msgs::msg::JointState joint_msg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_control::LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
