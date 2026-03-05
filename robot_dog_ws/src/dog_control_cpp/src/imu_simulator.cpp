#include "dog_control_cpp/imu_simulator.hpp"

#include <random>
#include <math>

namespace dog_control_cpp
{

ImuSimulator::ImuSimulator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("imu_simulator", options),
      publish_rate_(100.0),
      static_roll_(0.0),
      static_pitch_(0.0),
      osc_amplitude_roll_(0.02),
      osc_amplitude_pitch_(0.015),
      osc_frequency_(2.0),
      noise_stddev_(0.005),
      current_time_(0.0),
      disturbance_roll_(0.0),
      disturbance_pitch_(0.0),
      disturbance_decay_(0.95)
{
  // Declare parameters
  declare_parameter("publish_rate", publish_rate_);
  declare_parameter("static_roll", static_roll_);
  declare_parameter("static_pitch", static_pitch_);
  declare_parameter("osc_amplitude_roll", osc_amplitude_roll_);
  declare_parameter("osc_amplitude_pitch", osc_amplitude_pitch_);
  declare_parameter("osc_frequency", osc_frequency_);
  declare_parameter("noise_stddev", noise_stddev_);
  declare_parameter("disturbance_decay", disturbance_decay_);

  // Get parameters
  publish_rate_ = get_parameter("publish_rate").as_double();
  static_roll_ = get_parameter("static_roll").as_double();
  static_pitch_ = get_parameter("static_pitch").as_double();
  osc_amplitude_roll_ = get_parameter("osc_amplitude_roll").as_double();
  osc_amplitude_pitch_ = get_parameter("osc_amplitude_pitch").as_double();
  osc_frequency_ = get_parameter("osc_frequency").as_double();
  noise_stddev_ = get_parameter("noise_stddev").as_double();
  disturbance_decay_ = get_parameter("disturbance_decay").as_double();

  RCLCPP_INFO(get_logger(), "IMU Simulator initialized:");
  RCLCPP_INFO(get_logger(), "  Static offset: roll=%.3f rad, pitch=%.3f rad", 
              static_roll_, static_pitch_);
  RCLCPP_INFO(get_logger(), "  Oscillation: roll=%.3f, pitch=%.3f, freq=%.1f Hz",
              osc_amplitude_roll_, osc_amplitude_pitch_, osc_frequency_);
  RCLCPP_INFO(get_logger(), "  Noise stddev: %.4f", noise_stddev_);

  // QoS for sensor data
  rclcpp::QoS sensor_qos(10);
  sensor_qos.best_effort();

  // Create publisher
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", sensor_qos);

  // Create subscriber for disturbances
  disturbance_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/imu/disturbance", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        disturbanceCallback(msg);
      });

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { timerCallback(); });
}

void ImuSimulator::timerCallback()
{
  auto msg = sensor_msgs::msg::Imu();
  generateImuData(msg);
  imu_pub_->publish(msg);

  // Decay disturbances
  disturbance_roll_ *= disturbance_decay_;
  disturbance_pitch_ *= disturbance_decay_;
  if (std::abs(disturbance_roll_) < 0.001) disturbance_roll_ = 0.0;
  if (std::abs(disturbance_pitch_) < 0.001) disturbance_pitch_ = 0.0;

  // Update time
  current_time_ += 1.0 / publish_rate_;
}

void ImuSimulator::disturbanceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  disturbance_roll_ += msg->x;
  disturbance_pitch_ += msg->y;
  RCLCPP_INFO(get_logger(), "Applied disturbance: roll=%.3f, pitch=%.3f", 
              msg->x, msg->y);
}

void ImuSimulator::generateImuData(sensor_msgs::msg::Imu &msg)
{
  // Generate orientation
  double roll = static_roll_ + 
                osc_amplitude_roll_ * std::sin(2.0 * M_PI * osc_frequency_ * current_time_) +
                disturbance_roll_;
  
  double pitch = static_pitch_ + 
                 osc_amplitude_pitch_ * std::cos(2.0 * M_PI * osc_frequency_ * current_time_) +
                 disturbance_pitch_;
  
  double yaw = 0.0;  // No yaw in simulation

  // Add noise
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0.0, noise_stddev_);
  
  roll += distribution(generator);
  pitch += distribution(generator);

  // Convert to quaternion
  double w, x, y, z;
  eulerToQuaternion(roll, pitch, yaw, w, x, y, z);

  // Fill message
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "base_link";

  msg.orientation.w = w;
  msg.orientation.x = x;
  msg.orientation.y = y;
  msg.orientation.z = z;

  // Angular velocity (simulated from oscillation)
  msg.angular_velocity.x = 2.0 * M_PI * osc_frequency_ * osc_amplitude_roll_ * 
                           std::cos(2.0 * M_PI * osc_frequency_ * current_time_);
  msg.angular_velocity.y = -2.0 * M_PI * osc_frequency_ * osc_amplitude_pitch_ * 
                            std::sin(2.0 * M_PI * osc_frequency_ * current_time_);
  msg.angular_velocity.z = 0.0;

  // Linear acceleration (gravity + small vibrations)
  msg.linear_acceleration.x = distribution(generator) * 0.1;
  msg.linear_acceleration.y = distribution(generator) * 0.1;
  msg.linear_acceleration.z = 9.81 + distribution(generator) * 0.1;

  // Covariance matrices (identity * variance)
  for (int i = 0; i < 9; ++i) {
    msg.orientation_covariance[i] = 0.0;
    msg.angular_velocity_covariance[i] = 0.0;
    msg.linear_acceleration_covariance[i] = 0.0;
  }
  msg.orientation_covariance[0] = noise_stddev_ * noise_stddev_;
  msg.orientation_covariance[4] = noise_stddev_ * noise_stddev_;
  msg.orientation_covariance[8] = noise_stddev_ * noise_stddev_;
}

void ImuSimulator::eulerToQuaternion(double roll, double pitch, double yaw,
                                      double &w, double &x, double &y, double &z)
{
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;
}

} // namespace dog_control_cpp

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_control_cpp::ImuSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
