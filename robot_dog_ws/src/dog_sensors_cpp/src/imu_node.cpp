/**
 * @file imu_node.cpp
 * @brief IMU node implementation with MPU6050 driver and simulator support
 */

#include "dog_sensors_cpp/imu_node.hpp"
#include "dog_sensors_cpp/mpu6050_driver.hpp"
#include "dog_sensors_cpp/mpu6050_simulator.hpp"

#include <math>
#include <string>

#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace dog_sensors_cpp
{

// QoS profile for IMU data - best effort for low latency
static constexpr rmw_qos_profile_t imu_qos_profile = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,  // depth
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("imu_node", options)
{
  // Declare parameters
  declare_parameter("publish_rate_hz", publish_rate_hz_);
  declare_parameter("frame_id", frame_id_);
  declare_parameter("simulate", simulate_);
  declare_parameter("driver_type", driver_type_);
  declare_parameter("publish_pose", publish_pose_);
  declare_parameter("linear_accel_bias", linear_accel_bias_);
  declare_parameter("angular_vel_bias", angular_vel_bias_);
  
  // MPU6050 parameters
  declare_parameter("i2c_bus", i2c_bus_);
  declare_parameter("i2c_address", i2c_address_);
  
  // Simulator parameters
  declare_parameter("sim_motion_mode", sim_motion_mode_);
  declare_parameter("sim_walk_frequency", sim_walk_frequency_);
  declare_parameter("sim_walk_amplitude", sim_walk_amplitude_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ImuNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring IMU node...");

  // Get parameters
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  frame_id_ = get_parameter("frame_id").as_string();
  simulate_ = get_parameter("simulate").as_bool();
  driver_type_ = get_parameter("driver_type").as_string();
  publish_pose_ = get_parameter("publish_pose").as_bool();
  linear_accel_bias_ = get_parameter("linear_accel_bias").as_double_array();
  angular_vel_bias_ = get_parameter("angular_vel_bias").as_double_array();
  
  i2c_bus_ = get_parameter("i2c_bus").as_string();
  i2c_address_ = get_parameter("i2c_address").as_int();
  
  sim_motion_mode_ = get_parameter("sim_motion_mode").as_string();
  sim_walk_frequency_ = get_parameter("sim_walk_frequency").as_double();
  sim_walk_amplitude_ = get_parameter("sim_walk_amplitude").as_double();

  RCLCPP_INFO(get_logger(), "Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "Publish rate: %.1f Hz", publish_rate_hz_);
  RCLCPP_INFO(get_logger(), "Simulation mode: %s", simulate_ ? "enabled" : "disabled");
  RCLCPP_INFO(get_logger(), "Driver type: %s", driver_type_.c_str());

  // Determine driver type
  if (driver_type_ == "mpu6050") {
    current_driver_ = ImuDriverType::MPU6050;
  } else if (driver_type_ == "simulator" || simulate_) {
    current_driver_ = ImuDriverType::SIMULATOR;
  } else if (driver_type_ == "none") {
    current_driver_ = ImuDriverType::NONE;
  } else {
    // Auto-detect: try hardware, fall back to simulator
    current_driver_ = ImuDriverType::NONE;  // Will try to initialize in on_activate
  }

  // Create publishers with best-effort QoS for IMU (low latency priority)
  rclcpp::QoS imu_qos(rclcpp::QoSInitialization::from_rmw(imu_qos_profile));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", imu_qos);

  // Pose publisher (reliable for control algorithms)
  if (publish_pose_) {
    pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(
      "/body_pose", rclcpp::QoS(10).reliable());
  }

  // Status publisher
  status_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/imu/status", rclcpp::QoS(1).reliable());

  // Create velocity command subscriber for simulator
  if (current_driver_ == ImuDriverType::SIMULATOR || 
      (current_driver_ == ImuDriverType::NONE && simulate_)) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ImuNode::velocity_callback, this, std::placeholders::_1));
  }

  // Create timer (but don't start yet)
  std::chrono::milliseconds period_ms(
    static_cast<int>(1000.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period_ms, [this]() { timer_callback(); });
  timer_->cancel();  // Will start on activate

  // Initialize pose
  current_pose_.orientation.w = 1.0;
  current_pose_.orientation.x = 0.0;
  current_pose_.orientation.y = 0.0;
  current_pose_.orientation.z = 0.0;

  RCLCPP_INFO(get_logger(), "Configuration complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ImuNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating IMU node...");

  // Initialize driver based on type
  if (current_driver_ == ImuDriverType::MPU6050 || 
      (current_driver_ == ImuDriverType::NONE && !simulate_)) {
    // Try to initialize MPU6050
    mpu6050_driver_ = std::make_unique<Mpu6050Driver>(
      i2c_bus_, static_cast<uint8_t>(i2c_address_));
    
    if (mpu6050_driver_->initialize()) {
      RCLCPP_INFO(get_logger(), "MPU6050 initialized successfully on %s (0x%02X)",
                  i2c_bus_.c_str(), i2c_address_);
      current_driver_ = ImuDriverType::MPU6050;
    } else {
      RCLCPP_WARN(get_logger(), "Failed to initialize MPU6050, falling back to simulator");
      mpu6050_driver_.reset();
      current_driver_ = ImuDriverType::SIMULATOR;
    }
  }
  
  if (current_driver_ == ImuDriverType::SIMULATOR || simulate_) {
    // Initialize simulator
    SimulatorConfig config;
    config.walk_frequency = sim_walk_frequency_;
    config.walk_amplitude = sim_walk_amplitude_;
    
    simulator_ = std::make_unique<Mpu6050Simulator>(config);
    simulator_->initialize();
    
    // Set motion mode
    if (sim_motion_mode_ == "walking") {
      simulator_->set_motion_mode(MotionMode::WALKING);
    } else if (sim_motion_mode_ == "trotting") {
      simulator_->set_motion_mode(MotionMode::TROTTING);
    } else if (sim_motion_mode_ == "turning") {
      simulator_->set_motion_mode(MotionMode::TURNING);
    } else if (sim_motion_mode_ == "pacing") {
      simulator_->set_motion_mode(MotionMode::PACING);
    } else {
      simulator_->set_motion_mode(MotionMode::IDLE);
    }
    
    RCLCPP_INFO(get_logger(), "Simulator initialized in '%s' mode", sim_motion_mode_.c_str());
    current_driver_ = ImuDriverType::SIMULATOR;
  }

  // Activate publishers
  imu_pub_->on_activate();
  if (status_pub_) {
    status_pub_->on_activate();
  }
  if (pose_pub_) {
    pose_pub_->on_activate();
  }

  // Calibrate if needed
  if (current_driver_ == ImuDriverType::MPU6050 && !is_calibrated_) {
    calibrate();
  }

  // Start timer
  timer_->reset();
  last_publish_time_ = now();

  // Publish initial status
  std_msgs::msg::Bool status_msg;
  status_msg.data = true;
  status_pub_->publish(status_msg);

  RCLCPP_INFO(get_logger(), "IMU node activated with %s driver",
              current_driver_ == ImuDriverType::MPU6050 ? "MPU6050" :
              current_driver_ == ImuDriverType::SIMULATOR ? "SIMULATOR" : "NONE");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ImuNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating IMU node...");

  // Stop timer
  timer_->cancel();

  // Deactivate publishers
  imu_pub_->on_deactivate();
  if (status_pub_) {
    status_pub_->on_deactivate();
  }
  if (pose_pub_) {
    pose_pub_->on_deactivate();
  }

  // Release drivers
  mpu6050_driver_.reset();
  simulator_.reset();

  RCLCPP_INFO(get_logger(), "IMU node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ImuNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up IMU node...");

  // Release resources
  timer_.reset();
  imu_pub_.reset();
  pose_pub_.reset();
  status_pub_.reset();
  cmd_vel_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ImuNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down IMU node...");

  // Ensure timer is stopped
  if (timer_) {
    timer_->cancel();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ImuNode::timer_callback()
{
  // Read data based on current driver
  sensor_msgs::msg::Imu imu_msg;
  
  switch (current_driver_) {
    case ImuDriverType::MPU6050:
      imu_msg = read_imu_data();
      break;
    case ImuDriverType::SIMULATOR:
      imu_msg = simulate_imu_data();
      break;
    default:
      // Dummy data
      imu_msg = sensor_msgs::msg::Imu();
      imu_msg.linear_acceleration.x = 0.0;
      imu_msg.linear_acceleration.y = 0.0;
      imu_msg.linear_acceleration.z = 9.81;
      imu_msg.angular_velocity.x = 0.0;
      imu_msg.angular_velocity.y = 0.0;
      imu_msg.angular_velocity.z = 0.0;
      imu_msg.orientation.w = 1.0;
      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      break;
  }

  // Update timestamp
  imu_msg.header.stamp = now();
  imu_msg.header.frame_id = frame_id_;

  // Publish IMU data
  imu_pub_->publish(imu_msg);

  // Update and publish pose estimate if enabled
  if (publish_pose_ && pose_pub_->is_activated()) {
    update_pose_estimate(imu_msg);
    pose_pub_->publish(current_pose_);
  }

  last_publish_time_ = now();
}

sensor_msgs::msg::Imu ImuNode::read_imu_data()
{
  sensor_msgs::msg::Imu msg;
  
  if (!mpu6050_driver_) {
    return msg;
  }
  
  // Read from MPU6050 with Mahony filter
  double dt = 1.0 / publish_rate_hz_;
  Mpu6050Data data = mpu6050_driver_->read(dt);
  
  // Fill IMU message
  msg.linear_acceleration.x = data.accel_x;
  msg.linear_acceleration.y = data.accel_y;
  msg.linear_acceleration.z = data.accel_z;
  
  msg.angular_velocity.x = data.gyro_x;
  msg.angular_velocity.y = data.gyro_y;
  msg.angular_velocity.z = data.gyro_z;
  
  msg.orientation.w = data.q_w;
  msg.orientation.x = data.q_x;
  msg.orientation.y = data.q_y;
  msg.orientation.z = data.q_z;
  
  // Set covariance matrices
  for (int i = 0; i < 9; ++i) {
    msg.orientation_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0;
    msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.0001 : 0.0;
    msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.0001 : 0.0;
  }

  return msg;
}

sensor_msgs::msg::Imu ImuNode::simulate_imu_data()
{
  sensor_msgs::msg::Imu msg;
  
  if (!simulator_) {
    return msg;
  }
  
  // Read from simulator
  double dt = 1.0 / publish_rate_hz_;
  Mpu6050Data data = simulator_->read(dt);
  
  // Fill IMU message
  msg.linear_acceleration.x = data.accel_x;
  msg.linear_acceleration.y = data.accel_y;
  msg.linear_acceleration.z = data.accel_z;
  
  msg.angular_velocity.x = data.gyro_x;
  msg.angular_velocity.y = data.gyro_y;
  msg.angular_velocity.z = data.gyro_z;
  
  msg.orientation.w = data.q_w;
  msg.orientation.x = data.q_x;
  msg.orientation.y = data.q_y;
  msg.orientation.z = data.q_z;
  
  // Set covariance matrices (simulator has less noise)
  for (int i = 0; i < 9; ++i) {
    msg.orientation_covariance[i] = (i % 4 == 0) ? 0.0001 : 0.0;
    msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.00001 : 0.0;
    msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.00001 : 0.0;
  }

  return msg;
}

void ImuNode::calibrate()
{
  RCLCPP_INFO(get_logger(), "Calibrating IMU...");
  
  if (mpu6050_driver_) {
    if (mpu6050_driver_->calibrate_gyro(500)) {
      RCLCPP_INFO(get_logger(), "Gyroscope calibration complete");
    } else {
      RCLCPP_WARN(get_logger(), "Gyroscope calibration failed");
    }
  }
  
  is_calibrated_ = true;
  RCLCPP_INFO(get_logger(), "IMU calibration complete");
}

void ImuNode::update_pose_estimate(const sensor_msgs::msg::Imu & imu_msg)
{
  // Simple pose integration from IMU data
  // In production, use a proper state estimator (EKF, Madgwick, etc.)

  // Get current orientation as tf2 quaternion
  tf2::Quaternion q_current(
    current_pose_.orientation.x,
    current_pose_.orientation.y,
    current_pose_.orientation.z,
    current_pose_.orientation.w);

  // Create rotation from angular velocity
  double dt = 1.0 / publish_rate_hz_;
  tf2::Quaternion q_delta;
  q_delta.setRPY(
    imu_msg.angular_velocity.x * dt,
    imu_msg.angular_velocity.y * dt,
    imu_msg.angular_velocity.z * dt);

  // Update orientation
  q_current = q_current * q_delta;
  q_current.normalize();

  current_pose_.orientation.w = q_current.w();
  current_pose_.orientation.x = q_current.x();
  current_pose_.orientation.y = q_current.y();
  current_pose_.orientation.z = q_current.z();

  // Position integration (simple, will drift)
  // Remove gravity from acceleration
  tf2::Matrix3x3 rot(q_current);
  tf2::Vector3 accel_body(
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z - 9.81);
  tf2::Vector3 accel_world = rot * accel_body;

  // Integrate to get velocity and position
  for (size_t i = 0; i < 3; ++i) {
    linear_velocity_[i] += accel_world[i] * dt;
    // Add heavy damping to prevent runaway drift
    linear_velocity_[i] *= 0.99;
  }

  current_pose_.position.x += linear_velocity_[0] * dt;
  current_pose_.position.y += linear_velocity_[1] * dt;
  current_pose_.position.z += linear_velocity_[2] * dt;
}

void ImuNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (simulator_) {
    simulator_->set_velocity_command(
      msg->linear.x,
      msg->linear.y,
      msg->angular.z);
  }
}

}  // namespace dog_sensors_cpp
