/**
 * @file imu_node.hpp
 * @brief IMU sensor node with lifecycle management and MPU6050 support
 * 
 * Publishes sensor_msgs/Imu at configurable rate with best-effort QoS.
 * Supports:
 * - Hardware MPU6050 via I2C
 * - Simulated data for testing without hardware
 * - Mahony sensor fusion for orientation
 */

#ifndef DOG_SENSORS_CPP__IMU_NODE_HPP_
#define DOG_SENSORS_CPP__IMU_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

// Include driver types for unique_ptr
#include "dog_sensors_cpp/mpu6050_driver.hpp"
#include "dog_sensors_cpp/mpu6050_simulator.hpp"

namespace dog_sensors_cpp {

/**
 * @brief Driver type selection
 */
enum class ImuDriverType
{
  NONE,       ///< No driver (dummy data)
  MPU6050,    ///< Hardware MPU6050 via I2C
  SIMULATOR   ///< Software simulator
};

/**
 * @brief Lifecycle node for IMU sensor processing
 * 
 * States:
 * - UNCONFIGURED: Node created, not initialized
 * - INACTIVE: Configured, ready to activate
 * - ACTIVE: Publishing IMU data
 * - FINALIZED: Cleanup complete
 */
class ImuNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ImuNode() override;

  // Lifecycle transitions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Main timer callback for IMU data publishing
   */
  void timer_callback();

  /**
   * @brief Read IMU data from hardware driver
   * @return Populated IMU message
   */
  sensor_msgs::msg::Imu read_imu_data();

  /**
   * @brief Read IMU data from simulator
   * @return Simulated IMU message
   */
  sensor_msgs::msg::Imu simulate_imu_data();

  /**
   * @brief Calibrate IMU offsets
   */
  void calibrate();

  /**
   * @brief Update integrated pose estimate from IMU data
   * @param imu_msg Latest IMU reading
   */
  void update_pose_estimate(const sensor_msgs::msg::Imu & imu_msg);

  /**
   * @brief Velocity command callback for simulator
   * @param msg Twist message with velocity commands
   */
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double publish_rate_hz_{100.0};           // Publication rate
  std::string frame_id_{"imu_link"};        // TF frame ID
  bool simulate_{false};                     // Use simulation mode
  std::string driver_type_{"auto"};          // Driver type: "auto", "mpu6050", "simulator", "none"
  bool publish_pose_{true};                  // Publish integrated pose
  std::vector<double> linear_accel_bias_{0.0, 0.0, 0.0};
  std::vector<double> angular_vel_bias_{0.0, 0.0, 0.0};
  
  // MPU6050 specific parameters
  std::string i2c_bus_{"/dev/i2c-1"};       // I2C bus path
  int i2c_address_{0x68};                   // I2C address (0x68 or 0x69)
  
  // Simulator specific parameters
  std::string sim_motion_mode_{"idle"};     // Simulation mode: idle, walking, trotting, turning
  double sim_walk_frequency_{1.0};          // Walking gait frequency (Hz)
  double sim_walk_amplitude_{0.15};         // Walking amplitude (rad)

  // State
  bool is_calibrated_{false};
  rclcpp::Time last_publish_time_;
  
  // Pose estimation (simple integration)
  geometry_msgs::msg::Pose current_pose_;
  std::vector<double> linear_velocity_{0.0, 0.0, 0.0};
  
  // Simulated data state (legacy)
  double sim_time_{0.0};
  
  // Driver instances
  std::unique_ptr<Mpu6050Driver> mpu6050_driver_;
  std::unique_ptr<Mpu6050Simulator> simulator_;
  
  // Current driver type
  ImuDriverType current_driver_{ImuDriverType::NONE};
};

}  // namespace dog_sensors_cpp

#endif  // DOG_SENSORS_CPP__IMU_NODE_HPP_
