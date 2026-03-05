/**
 * @file mpu6050_simulator.hpp
 * @brief MPU6050 simulator with realistic robot motion patterns
 * 
 * Generates synthetic IMU data simulating:
 * - Standing idle (small vibrations)
 * - Walking gait (periodic acceleration/rotation)
 * - Turning (yaw rotation)
 * - Transient responses
 */

#ifndef DOG_SENSORS_CPP__MPU6050_SIMULATOR_HPP_
#define DOG_SENSORS_CPP__MPU6050_SIMULATOR_HPP_

#include <string>
#include <cmath>
#include <array>
#include <random>

#include "dog_sensors_cpp/mpu6050_driver.hpp"

namespace dog_sensors_cpp
{

/**
 * @brief Motion mode for simulation
 */
enum class MotionMode
{
  IDLE,       ///< Standing still with small vibrations
  WALKING,    ///< Walking gait
  TROTTING,   ///< Trotting gait (faster walking)
  TURNING,    ///< Turning in place
  PACING,     ///< Pacing gait
  CUSTOM      ///< Custom motion pattern
};

/**
 * @brief Configuration for simulation
 */
struct SimulatorConfig
{
  // Noise levels
  double accel_noise_std{0.05};    ///< Accelerometer noise (m/s²)
  double gyro_noise_std{0.02};     ///< Gyroscope noise (rad/s)
  
  // Motion parameters
  double walk_frequency{1.0};      ///< Walking cycle frequency (Hz)
  double walk_amplitude{0.15};     ///< Walking amplitude (rad)
  double turn_rate{0.5};           ///< Turning rate (rad/s)
  
  // Gait parameters
  double step_height{0.02};        ///< Vertical step height (m)
  double stride_length{0.1};       ///< Stride length (m)
  
  // Drift simulation
  bool simulate_drift{true};       ///< Add gyro drift
  double drift_rate{0.0001};       ///< Drift rate (rad/s per second)
  
  // Initial orientation
  double initial_roll{0.0};
  double initial_pitch{0.0};
  double initial_yaw{0.0};
};

/**
 * @brief MPU6050 simulator class
 */
class Mpu6050Simulator
{
public:
  /**
   * @brief Constructor
   * @param config Simulator configuration
   */
  explicit Mpu6050Simulator(const SimulatorConfig & config = SimulatorConfig{});

  /**
   * @brief Initialize simulator
   * @return Always returns true
   */
  bool initialize();

  /**
   * @brief Check if simulator is "connected" (always true)
   * @return true
   */
  bool is_connected() const { return true; }

  /**
   * @brief Read simulated sensor data
   * @param dt Time step (seconds)
   * @return Simulated sensor data
   */
  Mpu6050Data read(double dt);

  /**
   * @brief Get latest data
   * @return Last generated data
   */
  Mpu6050Data get_data() const { return data_; }

  /**
   * @brief Set motion mode
   * @param mode Motion mode
   */
  void set_motion_mode(MotionMode mode);

  /**
   * @brief Get current motion mode
   * @return Current mode
   */
  MotionMode get_motion_mode() const { return mode_; }

  /**
   * @brief Set custom velocity commands
   * @param linear_x Forward velocity (m/s)
   * @param linear_y Lateral velocity (m/s)
   * @param angular_z Yaw rate (rad/s)
   */
  void set_velocity_command(double linear_x, double linear_y, double angular_z);

  /**
   * @brief Reset simulation state
   */
  void reset();

  /**
   * @brief Set simulator configuration
   * @param config New configuration
   */
  void set_config(const SimulatorConfig & config) { config_ = config; }

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  SimulatorConfig get_config() const { return config_; }

private:
  // Generate noise
  double generate_noise(double std_dev);
  
  // Update motion based on mode
  void update_motion(double dt);
  
  // Compute IMU readings from motion state
  void compute_imu_readings(double dt);
  
  // Mahony filter update (same as hardware driver)
  void mahony_update(double ax, double ay, double az,
                     double gx, double gy, double gz, double dt);
  
  // Configuration
  SimulatorConfig config_;
  
  // Motion state
  MotionMode mode_{MotionMode::IDLE};
  double sim_time_{0.0};
  
  // Velocity commands
  double cmd_linear_x_{0.0};
  double cmd_linear_y_{0.0};
  double cmd_angular_z_{0.0};
  
  // Position and orientation
  double pos_x_{0.0}, pos_y_{0.0}, pos_z_{0.0};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  
  // Body motion (for gait simulation)
  double body_roll_{0.0};
  double body_pitch_{0.0};
  double body_height_{0.0};
  
  // Gyro drift
  double drift_x_{0.0}, drift_y_{0.0}, drift_z_{0.0};
  
  // Random number generator
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_{0.0, 1.0};
  
  // Current data
  Mpu6050Data data_;
  
  // Mahony filter state
  double mahony_q0_{1.0}, mahony_q1_{0.0}, mahony_q2_{0.0}, mahony_q3_{0.0};
  double mahony_integral_x_{0.0}, mahony_integral_y_{0.0}, mahony_integral_z_{0.0};
  static constexpr double MAHONY_KP_{2.0};
  static constexpr double MAHONY_KI_{0.005};
  
  // Previous values for differentiation
  double prev_roll_{0.0}, prev_pitch_{0.0}, prev_yaw_{0.0};
  double prev_vel_x_{0.0}, prev_vel_y_{0.0}, prev_vel_z_{0.0};
};

}  // namespace dog_sensors_cpp

#endif  // DOG_SENSORS_CPP__MPU6050_SIMULATOR_HPP_
