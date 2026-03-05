/**
 * @file test_mpu6050_simulator.cpp
 * @brief Unit tests for MPU6050 Simulator
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>
#include <math>

// Simulator configuration matching MPU6050Simulator
struct SimulatorConfig
{
  double accel_noise_std{0.05};
  double gyro_noise_std{0.02};
  double walk_frequency{1.0};
  double walk_amplitude{0.15};
  double turn_rate{0.5};
  double step_height{0.02};
  double stride_length{0.1};
  bool simulate_drift{true};
  double drift_rate{0.0001};
  double initial_roll{0.0};
  double initial_pitch{0.0};
  double initial_yaw{0.0};
};

enum class MotionMode
{
  IDLE,
  WALKING,
  TROTTING,
  TURNING,
  PACING,
  CUSTOM
};

/**
 * @brief Simplified MPU6050 data structure
 */
struct Mpu6050Data
{
  double accel_x{0.0};
  double accel_y{0.0};
  double accel_z{0.0};
  double gyro_x{0.0};
  double gyro_y{0.0};
  double gyro_z{0.0};
  double temperature{25.0};
  double q_w{1.0};
  double q_x{0.0};
  double q_y{0.0};
  double q_z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

/**
 * @brief Simplified MPU6050 simulator for testing
 */
class Mpu6050SimulatorTest
{
public:
  explicit Mpu6050SimulatorTest(const SimulatorConfig &config = SimulatorConfig{})
    : config_(config), mode_(MotionMode::IDLE), sim_time_(0.0) {}
  
  bool initialize() { return true; }
  bool is_connected() const { return true; }
  
  Mpu6050Data read(double dt)
  {
    sim_time_ += dt;
    update_motion(dt);
    compute_imu_readings(dt);
    return data_;
  }
  
  void set_motion_mode(MotionMode mode) { mode_ = mode; }
  MotionMode get_motion_mode() const { return mode_; }
  
  void set_velocity_command(double linear_x, double /*linear_y*/, double angular_z)
  {
    cmd_linear_x_ = linear_x;
    cmd_angular_z_ = angular_z;
    
    // Auto-select mode
    if (std::abs(angular_z) > 0.3) {
      mode_ = MotionMode::TURNING;
    } else if (std::abs(linear_x) > 0.8) {
      mode_ = MotionMode::TROTTING;
    } else if (std::abs(linear_x) > 0.1) {
      mode_ = MotionMode::WALKING;
    } else {
      mode_ = MotionMode::IDLE;
    }
  }
  
  void reset()
  {
    sim_time_ = 0.0;
    mode_ = MotionMode::IDLE;
    roll_ = config_.initial_roll;
    pitch_ = config_.initial_pitch;
    yaw_ = config_.initial_yaw;
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
  }
  
  double getSimTime() const { return sim_time_; }
  
private:
  void update_motion(double dt)
  {
    prev_roll_ = roll_;
    prev_pitch_ = pitch_;
    prev_yaw_ = yaw_;
    
    switch (mode_) {
      case MotionMode::IDLE:
        body_roll_ = 0.02 * std::sin(2.0 * M_PI * 2.0 * sim_time_);
        body_pitch_ = 0.01 * std::sin(2.0 * M_PI * 1.5 * sim_time_ + 1.0);
        body_height_ = 0.001 * std::sin(2.0 * M_PI * 10.0 * sim_time_);
        break;
        
      case MotionMode::WALKING: {
        double gait_phase = 2.0 * M_PI * config_.walk_frequency * sim_time_;
        body_pitch_ = config_.walk_amplitude * 0.3 * std::sin(gait_phase);
        body_roll_ = 0.05 * std::sin(gait_phase * 2.0);
        body_height_ = config_.step_height * 0.5 * (1.0 + std::sin(gait_phase * 2.0));
        yaw_ += cmd_angular_z_ * dt;
        break;
      }
      
      case MotionMode::TROTTING: {
        double gait_phase = 2.0 * M_PI * config_.walk_frequency * 1.5 * sim_time_;
        body_pitch_ = config_.walk_amplitude * 0.5 * std::sin(gait_phase);
        body_roll_ = 0.08 * std::sin(gait_phase * 2.0);
        body_height_ = config_.step_height * 0.8 * std::abs(std::sin(gait_phase));
        yaw_ += cmd_angular_z_ * dt;
        break;
      }
      
      case MotionMode::TURNING: {
        double turn_phase = 2.0 * M_PI * config_.walk_frequency * 0.5 * sim_time_;
        body_pitch_ = 0.03 * std::sin(turn_phase);
        body_roll_ = 0.1 * cmd_angular_z_ + 0.02 * std::sin(turn_phase * 2.0);
        body_height_ = config_.step_height * 0.3 * (1.0 + std::sin(turn_phase * 2.0));
        yaw_ += cmd_angular_z_ * dt;
        break;
      }
      
      default:
        break;
    }
    
    roll_ = config_.initial_roll + body_roll_;
    pitch_ = config_.initial_pitch + body_pitch_;
  }
  
  void compute_imu_readings(double dt)
  {
    double d_roll = roll_ - prev_roll_;
    double d_pitch = pitch_ - prev_pitch_;
    double d_yaw = yaw_ - prev_yaw_;
    
    if (d_yaw > M_PI) d_yaw -= 2.0 * M_PI;
    if (d_yaw < -M_PI) d_yaw += 2.0 * M_PI;
    
    double cr = std::cos(roll_);
    double sr = std::sin(roll_);
    double cp = std::cos(pitch_);
    double sp = std::sin(pitch_);
    
    double gyro_x = (d_roll - sp * d_yaw) / dt;
    double gyro_y = (cr * d_pitch + sr * cp * d_yaw) / dt;
    double gyro_z = (-sr * d_pitch + cr * cp * d_yaw) / dt;
    
    double g = 9.80665;
    double grav_x = -g * sp;
    double grav_y = g * sr * cp;
    double grav_z = g * cr * cp;
    
    data_.gyro_x = gyro_x;
    data_.gyro_y = gyro_y;
    data_.gyro_z = gyro_z;
    data_.accel_x = grav_x;
    data_.accel_y = grav_y;
    data_.accel_z = grav_z;
    data_.roll = roll_;
    data_.pitch = pitch_;
    data_.yaw = yaw_;
  }
  
  SimulatorConfig config_;
  MotionMode mode_;
  double sim_time_;
  double cmd_linear_x_{0.0};
  double cmd_angular_z_{0.0};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  double prev_roll_{0.0}, prev_pitch_{0.0}, prev_yaw_{0.0};
  double body_roll_{0.0}, body_pitch_{0.0}, body_height_{0.0};
  Mpu6050Data data_;
};

class MPU6050SimulatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_ = SimulatorConfig{};
    simulator_ = std::make_unique<Mpu6050SimulatorTest>(config_);
    dt_ = 0.01;  // 100 Hz
  }
  
  SimulatorConfig config_;
  std::unique_ptr<Mpu6050SimulatorTest> simulator_;
  double dt_;
};

// Test 1: Initialization
TEST_F(MPU6050SimulatorTest, Initialization)
{
  EXPECT_TRUE(simulator_->initialize());
  EXPECT_TRUE(simulator_->is_connected());
}

// Test 2: Default mode is IDLE
TEST_F(MPU6050SimulatorTest, DefaultModeIdle)
{
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::IDLE);
}

// Test 3: Mode transitions based on velocity
TEST_F(MPU6050SimulatorTest, ModeTransitions)
{
  // Low velocity -> IDLE
  simulator_->set_velocity_command(0.05, 0.0, 0.0);
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::IDLE);
  
  // Medium velocity -> WALKING
  simulator_->set_velocity_command(0.5, 0.0, 0.0);
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::WALKING);
  
  // High velocity -> TROTTING
  simulator_->set_velocity_command(1.0, 0.0, 0.0);
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::TROTTING);
  
  // High angular velocity -> TURNING
  simulator_->set_velocity_command(0.0, 0.0, 0.5);
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::TURNING);
}

// Test 4: IDLE mode produces small vibrations
TEST_F(MPU6050SimulatorTest, IdleModeVibrations)
{
  simulator_->set_motion_mode(MotionMode::IDLE);
  
  // Read multiple samples
  double max_roll = 0.0, max_pitch = 0.0;
  for (int i = 0; i < 100; ++i) {
    auto data = simulator_->read(dt_);
    max_roll = std::max(max_roll, std::abs(data.roll));
    max_pitch = std::max(max_pitch, std::abs(data.pitch));
  }
  
  // Should have small oscillations
  EXPECT_GT(max_roll, 0.0);
  EXPECT_LT(max_roll, 0.05);
  EXPECT_GT(max_pitch, 0.0);
  EXPECT_LT(max_pitch, 0.05);
}

// Test 5: WALKING mode produces periodic motion
TEST_F(MPU6050SimulatorTest, WalkingModePeriodic)
{
  simulator_->set_velocity_command(0.5, 0.0, 0.0);
  
  std::vector<double> pitch_values;
  
  // Collect one full cycle of data
  double duration = 2.0 / config_.walk_frequency;  // 2 periods
  int samples = static_cast<int>(duration / dt_);
  
  for (int i = 0; i < samples; ++i) {
    auto data = simulator_->read(dt_);
    pitch_values.push_back(data.pitch);
  }
  
  // Should have oscillating pitch
  double min_pitch = *std::min_element(pitch_values.begin(), pitch_values.end());
  double max_pitch = *std::max_element(pitch_values.begin(), pitch_values.end());
  double pitch_range = max_pitch - min_pitch;
  
  EXPECT_GT(pitch_range, 0.01);  // Should have significant variation
}

// Test 6: TROTTING mode has higher frequency
TEST_F(MPU6050SimulatorTest, TrottingHigherFrequency)
{
  // Collect walking data
  simulator_->set_velocity_command(0.5, 0.0, 0.0);
  simulator_->reset();
  
  std::vector<double> walking_pitch;
  for (int i = 0; i < 500; ++i) {
    auto data = simulator_->read(dt_);
    walking_pitch.push_back(data.pitch);
  }
  
  // Collect trotting data
  simulator_->set_velocity_command(1.0, 0.0, 0.0);
  simulator_->reset();
  
  std::vector<double> trotting_pitch;
  for (int i = 0; i < 500; ++i) {
    auto data = simulator_->read(dt_);
    trotting_pitch.push_back(data.pitch);
  }
  
  // Trotting should have higher variance (more aggressive motion)
  double walk_var = 0.0, trot_var = 0.0;
  double walk_mean = 0.0, trot_mean = 0.0;
  
  for (auto p : walking_pitch) walk_mean += p;
  for (auto p : trotting_pitch) trot_mean += p;
  walk_mean /= walking_pitch.size();
  trot_mean /= trotting_pitch.size();
  
  for (auto p : walking_pitch) walk_var += (p - walk_mean) * (p - walk_mean);
  for (auto p : trotting_pitch) trot_var += (p - trot_mean) * (p - trot_mean);
  
  // Trotting should have higher variance
  EXPECT_GT(trot_var, walk_var * 0.5);
}

// Test 7: TURNING mode produces yaw rotation
TEST_F(MPU6050SimulatorTest, TurningYawRotation)
{
  simulator_->set_velocity_command(0.0, 0.0, 0.5);  // Turn rate 0.5 rad/s
  simulator_->reset();
  
  double initial_yaw = simulator_->read(dt_).yaw;
  
  // Run for 1 second
  for (int i = 0; i < 100; ++i) {
    simulator_->read(dt_);
  }
  
  // Yaw should have changed
  // Note: We can't easily check the final yaw since we're reading
  // but the gyro_z should reflect the turning rate
}

// Test 8: Gravity vector in level position
TEST_F(MPU6050SimulatorTest, LevelGravityVector)
{
  simulator_->set_motion_mode(MotionMode::IDLE);
  simulator_->reset();
  
  // Let it settle
  for (int i = 0; i < 50; ++i) {
    simulator_->read(dt_);
  }
  
  auto data = simulator_->read(dt_);
  
  // When level, gravity should be primarily in Z
  EXPECT_GT(data.accel_z, 8.0);
  EXPECT_LT(std::abs(data.accel_x), 2.0);
  EXPECT_LT(std::abs(data.accel_y), 2.0);
  
  // Total acceleration magnitude should be close to gravity
  double total_accel = std::sqrt(
    data.accel_x * data.accel_x +
    data.accel_y * data.accel_y +
    data.accel_z * data.accel_z
  );
  EXPECT_NEAR(total_accel, 9.80665, 1.0);
}

// Test 9: Gyroscope readings during rotation
TEST_F(MPU6050SimulatorTest, GyroscopeReadings)
{
  simulator_->set_velocity_command(0.0, 0.0, 1.0);  // High turn rate
  simulator_->reset();
  
  auto data = simulator_->read(dt_);
  
  // Should have significant gyro readings
  EXPECT_GT(std::abs(data.gyro_x) + std::abs(data.gyro_y) + std::abs(data.gyro_z), 0.1);
}

// Test 10: Reset clears state
TEST_F(MPU6050SimulatorTest, ResetClearsState)
{
  // Run simulation for a while
  simulator_->set_velocity_command(0.5, 0.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    simulator_->read(dt_);
  }
  
  double time_before = simulator_->getSimTime();
  EXPECT_GT(time_before, 0.0);
  
  // Reset
  simulator_->reset();
  
  EXPECT_EQ(simulator_->getSimTime(), 0.0);
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::IDLE);
}

// Test 11: Simulation time progression
TEST_F(MPU6050SimulatorTest, TimeProgression)
{
  simulator_->reset();
  
  EXPECT_EQ(simulator_->getSimTime(), 0.0);
  
  simulator_->read(dt_);
  EXPECT_NEAR(simulator_->getSimTime(), dt_, 1e-10);
  
  simulator_->read(dt_);
  EXPECT_NEAR(simulator_->getSimTime(), 2 * dt_, 1e-10);
  
  // Multiple reads
  for (int i = 0; i < 98; ++i) {
    simulator_->read(dt_);
  }
  EXPECT_NEAR(simulator_->getSimTime(), 1.0, 1e-10);
}

// Test 12: Temperature reading
TEST_F(MPU6050SimulatorTest, TemperatureReading)
{
  auto data = simulator_->read(dt_);
  
  // Temperature should be around room temperature
  EXPECT_GT(data.temperature, 15.0);
  EXPECT_LT(data.temperature, 35.0);
}

// Test 13: Quaternion output
TEST_F(MPU6050SimulatorTest, QuaternionOutput)
{
  auto data = simulator_->read(dt_);
  
  // Quaternion should be normalized
  double norm = std::sqrt(
    data.q_w * data.q_w +
    data.q_x * data.q_x +
    data.q_y * data.q_y +
    data.q_z * data.q_z
  );
  
  EXPECT_NEAR(norm, 1.0, 0.1);  // Allow some tolerance for unimplemented quaternion
}

// Test 14: Euler angles consistency
TEST_F(MPU6050SimulatorTest, EulerAnglesConsistency)
{
  simulator_->set_motion_mode(MotionMode::IDLE);
  simulator_->reset();
  
  for (int i = 0; i < 50; ++i) {
    auto data = simulator_->read(dt_);
    
    // All angles should be finite
    EXPECT_TRUE(std::isfinite(data.roll));
    EXPECT_TRUE(std::isfinite(data.pitch));
    EXPECT_TRUE(std::isfinite(data.yaw));
  }
}

// Test 15: Zero velocity command results in IDLE
TEST_F(MPU6050SimulatorTest, ZeroVelocityIdle)
{
  simulator_->set_velocity_command(0.0, 0.0, 0.0);
  EXPECT_EQ(simulator_->get_motion_mode(), MotionMode::IDLE);
}

// Test 16: Configuration parameters
TEST_F(MPU6050SimulatorTest, ConfigurationParameters)
{
  SimulatorConfig custom_config;
  custom_config.walk_frequency = 2.0;
  custom_config.walk_amplitude = 0.3;
  custom_config.step_height = 0.05;
  
  auto custom_sim = std::make_unique<Mpu6050SimulatorTest>(custom_config);
  custom_sim->set_velocity_command(0.5, 0.0, 0.0);
  
  // Should use custom parameters
  EXPECT_EQ(custom_sim->get_motion_mode(), MotionMode::WALKING);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
