#ifndef DOG_CONTROL_CPP__IMU_SIMULATOR_HPP_
#define DOG_CONTROL_CPP__IMU_SIMULATOR_HPP_

#include <memory>
#include <math>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace dog_control_cpp
{

/**
 * @brief IMU Simulator for testing balance controller in simulation
 * 
 * Publishes simulated IMU data with configurable:
 * - Static tilt (roll/pitch offsets)
 * - Sinusoidal oscillation (simulating walking vibration)
 * - Random noise
 * - External disturbances (can be triggered via topic)
 */
class ImuSimulator : public rclcpp::Node
{
public:
  explicit ImuSimulator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void timerCallback();
  void disturbanceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  
  // Generate simulated IMU data
  void generateImuData(sensor_msgs::msg::Imu &msg);
  
  // Convert Euler to Quaternion
  void eulerToQuaternion(double roll, double pitch, double yaw,
                         double &w, double &x, double &y, double &z);

  // Parameters
  double publish_rate_;
  
  // Static offsets (simulating permanent tilt)
  double static_roll_;
  double static_pitch_;
  
  // Oscillation parameters (simulating walking vibration)
  double osc_amplitude_roll_;
  double osc_amplitude_pitch_;
  double osc_frequency_;
  
  // Noise parameters
  double noise_stddev_;
  
  // Current state
  double current_time_;
  
  // External disturbance
  double disturbance_roll_;
  double disturbance_pitch_;
  double disturbance_decay_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
  // Subscriber for external disturbances
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr disturbance_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dog_control_cpp

#endif // DOG_CONTROL_CPP__IMU_SIMULATOR_HPP_
