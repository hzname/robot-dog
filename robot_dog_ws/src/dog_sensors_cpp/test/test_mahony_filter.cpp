/**
 * @file test_mahony_filter.cpp
 * @brief Unit tests for Mahony sensor fusion filter
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

/**
 * @brief Mahony filter implementation (mirrors MPU6050 driver/simulator)
 * 
 * The Mahony filter fuses accelerometer and gyroscope data to estimate
 * orientation in the form of a quaternion.
 */
class MahonyFilter
{
public:
  MahonyFilter(double kp = 2.0, double ki = 0.005)
    : kp_(kp), ki_(ki),
      q0_(1.0), q1_(0.0), q2_(0.0), q3_(0.0),
      integral_x_(0.0), integral_y_(0.0), integral_z_(0.0) {}
  
  void reset()
  {
    q0_ = 1.0;
    q1_ = q2_ = q3_ = 0.0;
    integral_x_ = integral_y_ = integral_z_ = 0.0;
  }
  
  void update(double ax, double ay, double az,
              double gx, double gy, double gz, double dt)
  {
    // Normalize accelerometer measurement
    double norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0) {
      norm = 1.0 / norm;
      ax *= norm;
      ay *= norm;
      az *= norm;
    }
    
    // Estimated direction of gravity in body frame
    double vx = 2.0 * (q1_ * q3_ - q0_ * q2_);
    double vy = 2.0 * (q0_ * q1_ + q2_ * q3_);
    double vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;
    
    // Error is cross product between estimated and measured direction of gravity
    double ex = ay * vz - az * vy;
    double ey = az * vx - ax * vz;
    double ez = ax * vy - ay * vx;
    
    // Accumulate integral error
    integral_x_ += ex * ki_ * dt;
    integral_y_ += ey * ki_ * dt;
    integral_z_ += ez * ki_ * dt;
    
    // Apply feedback terms
    gx += kp_ * ex + integral_x_;
    gy += kp_ * ey + integral_y_;
    gz += kp_ * ez + integral_z_;
    
    // Integrate rate of change of quaternion
    double q0_dot = 0.5 * (-q1_ * gx - q2_ * gy - q3_ * gz);
    double q1_dot = 0.5 * (q0_ * gx + q2_ * gz - q3_ * gy);
    double q2_dot = 0.5 * (q0_ * gy - q1_ * gz + q3_ * gx);
    double q3_dot = 0.5 * (q0_ * gz + q1_ * gy - q2_ * gx);
    
    q0_ += q0_dot * dt;
    q1_ += q1_dot * dt;
    q2_ += q2_dot * dt;
    q3_ += q3_dot * dt;
    
    // Normalize quaternion
    norm = std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (norm > 0.0) {
      norm = 1.0 / norm;
      q0_ *= norm;
      q1_ *= norm;
      q2_ *= norm;
      q3_ *= norm;
    }
  }
  
  // Get quaternion
  std::array<double, 4> getQuaternion() const {
    return {{q0_, q1_, q2_, q3_}};
  }
  
  // Compute and get Euler angles
  std::array<double, 3> getEulerAngles() const {
    std::array<double, 3> angles;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q0_ * q1_ + q2_ * q3_);
    double cosr_cosp = 1.0 - 2.0 * (q1_ * q1_ + q2_ * q2_);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q0_ * q2_ - q3_ * q1_);
    if (std::abs(sinp) >= 1.0) {
      angles[1] = std::copysign(M_PI / 2.0, sinp);
    } else {
      angles[1] = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q0_ * q3_ + q1_ * q2_);
    double cosy_cosp = 1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    
    return angles;
  }
  
private:
  double kp_, ki_;
  double q0_, q1_, q2_, q3_;
  double integral_x_, integral_y_, integral_z_;
};

class MahonyFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    filter_ = std::make_unique<MahonyFilter>(2.0, 0.005);
    dt_ = 0.01;  // 100 Hz
  }
  
  std::unique_ptr<MahonyFilter> filter_;
  double dt_;
};

// Test 1: Initial quaternion should be identity
TEST_F(MahonyFilterTest, InitialQuaternionIdentity)
{
  auto q = filter_->getQuaternion();
  
  EXPECT_NEAR(q[0], 1.0, 1e-10);  // w
  EXPECT_NEAR(q[1], 0.0, 1e-10);  // x
  EXPECT_NEAR(q[2], 0.0, 1e-10);  // y
  EXPECT_NEAR(q[3], 0.0, 1e-10);  // z
}

// Test 2: Quaternion normalization
TEST_F(MahonyFilterTest, QuaternionNormalization)
{
  // Apply several updates
  for (int i = 0; i < 100; ++i) {
    filter_->update(0.0, 0.0, 9.81, 0.01, 0.02, 0.03, dt_);
  }
  
  auto q = filter_->getQuaternion();
  double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  
  EXPECT_NEAR(norm, 1.0, 1e-10);
}

// Test 3: Level position (gravity along Z)
TEST_F(MahonyFilterTest, LevelPosition)
{
  // Simulate IMU at level position
  // Accelerometer: only gravity in Z
  // Gyroscope: no rotation
  for (int i = 0; i < 500; ++i) {
    filter_->update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Should be close to zero roll, pitch
  EXPECT_NEAR(angles[0], 0.0, 0.05);  // roll
  EXPECT_NEAR(angles[1], 0.0, 0.05);  // pitch
}

// Test 4: Roll 90 degrees
TEST_F(MahonyFilterTest, Roll90Degrees)
{
  // Simulate IMU rolled 90 degrees
  // Accelerometer: gravity in Y (if rolled 90° around X)
  for (int i = 0; i < 500; ++i) {
    filter_->update(0.0, 9.81, 0.0, 0.0, 0.0, 0.0, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Should show approximately 90 degree roll
  EXPECT_NEAR(std::abs(angles[0]), M_PI / 2.0, 0.1);
}

// Test 5: Pitch 45 degrees
TEST_F(MahonyFilterTest, Pitch45Degrees)
{
  filter_->reset();
  
  // Simulate IMU pitched 45 degrees
  // Accelerometer: gravity has X and Z components
  double pitch_angle = M_PI / 4.0;
  double ax = -9.81 * std::sin(pitch_angle);
  double az = 9.81 * std::cos(pitch_angle);
  
  for (int i = 0; i < 500; ++i) {
    filter_->update(ax, 0.0, az, 0.0, 0.0, 0.0, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Should show approximately 45 degree pitch
  EXPECT_NEAR(angles[1], pitch_angle, 0.1);
}

// Test 6: Gyroscope integration only
TEST_F(MahonyFilterTest, GyroIntegration)
{
  filter_->reset();
  
  // Pure rotation around Z axis (yaw)
  double yaw_rate = 0.1;  // rad/s
  
  for (int i = 0; i < 100; ++i) {
    // No accelerometer (zero gravity - unrealistic but tests gyro integration)
    // Small accelerometer input to prevent drift issues
    filter_->update(0.0, 0.0, 0.001, 0.0, 0.0, yaw_rate, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Yaw should have accumulated
  double expected_yaw = yaw_rate * 100 * dt_;  // ~0.1 rad
  EXPECT_GT(std::abs(angles[2]), 0.01);
}

// Test 7: Accelerometer noise handling
TEST_F(MahonyFilterTest, AccelerometerNoise)
{
  filter_->reset();
  
  // Simulate noisy accelerometer at level position
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> noise(0.0, 0.5);  // 0.5 m/s² noise
  
  for (int i = 0; i < 500; ++i) {
    double ax = noise(gen);
    double ay = noise(gen);
    double az = 9.81 + noise(gen);
    filter_->update(ax, ay, az, 0.0, 0.0, 0.0, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Should still be reasonably close to level despite noise
  EXPECT_LT(std::abs(angles[0]), 0.3);  // roll
  EXPECT_LT(std::abs(angles[1]), 0.3);  // pitch
}

// Test 8: Gyroscope bias rejection
TEST_F(MahonyFilterTest, GyroBiasRejection)
{
  filter_->reset();
  
  // Simulate constant gyro bias with gravity reference
  double gyro_bias = 0.1;  // rad/s bias
  
  for (int i = 0; i < 1000; ++i) {
    filter_->update(0.0, 0.0, 9.81, gyro_bias, 0.0, 0.0, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Filter should converge to level despite gyro bias
  EXPECT_LT(std::abs(angles[0]), 0.2);
  EXPECT_LT(std::abs(angles[1]), 0.2);
}

// Test 9: Rapid rotation handling
TEST_F(MahonyFilterTest, RapidRotation)
{
  filter_->reset();
  
  // Simulate rapid rotation
  double high_gyro = 5.0;  // rad/s (about 286 deg/s)
  
  for (int i = 0; i < 50; ++i) {
    filter_->update(0.0, 0.0, 9.81, 0.0, 0.0, high_gyro, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Should have rotated significantly
  EXPECT_GT(std::abs(angles[2]), 0.1);
  
  // Quaternion should still be normalized
  auto q = filter_->getQuaternion();
  double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  EXPECT_NEAR(norm, 1.0, 1e-10);
}

// Test 10: Reset functionality
TEST_F(MahonyFilterTest, ResetFunctionality)
{
  // Apply some updates to change the state
  for (int i = 0; i < 100; ++i) {
    filter_->update(0.0, 9.81, 0.0, 0.0, 0.0, 0.0, dt_);
  }
  
  // State should have changed
  auto angles_before = filter_->getEulerAngles();
  EXPECT_GT(std::abs(angles_before[0]), 0.1);
  
  // Reset
  filter_->reset();
  
  // Should be back to identity
  auto q = filter_->getQuaternion();
  EXPECT_NEAR(q[0], 1.0, 1e-10);
  EXPECT_NEAR(q[1], 0.0, 1e-10);
  EXPECT_NEAR(q[2], 0.0, 1e-10);
  EXPECT_NEAR(q[3], 0.0, 1e-10);
}

// Test 11: Different filter gains
TEST_F(MahonyFilterTest, DifferentGains)
{
  // Create filter with different gains
  MahonyFilter fast_filter(5.0, 0.01);  // Higher Kp = faster response
  MahonyFilter slow_filter(0.5, 0.001);  // Lower Kp = slower response
  
  // Simulate roll position
  for (int i = 0; i < 100; ++i) {
    fast_filter.update(0.0, 9.81, 0.0, 0.0, 0.0, 0.0, dt_);
    slow_filter.update(0.0, 9.81, 0.0, 0.0, 0.0, 0.0, dt_);
  }
  
  auto fast_angles = fast_filter.getEulerAngles();
  auto slow_angles = slow_filter.getEulerAngles();
  
  // Fast filter should be closer to true value
  EXPECT_GT(std::abs(fast_angles[0]), std::abs(slow_angles[0]));
}

// Test 12: Quaternion to Euler conversion
TEST_F(MahonyFilterTest, QuaternionToEuler)
{
  // Test specific quaternion values
  filter_->reset();
  
  // After reset, should be at zero orientation
  auto angles = filter_->getEulerAngles();
  EXPECT_NEAR(angles[0], 0.0, 1e-10);
  EXPECT_NEAR(angles[1], 0.0, 1e-10);
  EXPECT_NEAR(angles[2], 0.0, 1e-10);
}

// Test 13: Combined rotation tracking
TEST_F(MahonyFilterTest, CombinedRotation)
{
  filter_->reset();
  
  // Simulate complex motion
  for (int i = 0; i < 300; ++i) {
    double t = i * dt_;
    // Time-varying accelerometer input (circular motion)
    double ax = 3.0 * std::sin(2.0 * M_PI * 0.5 * t);
    double ay = 3.0 * std::cos(2.0 * M_PI * 0.5 * t);
    double az = 9.0;  // Reduced gravity due to tilt
    
    // Corresponding gyroscope input
    double gx = 0.0;
    double gy = 0.0;
    double gz = 2.0 * M_PI * 0.5;  // Yaw rotation
    
    filter_->update(ax, ay, az, gx, gy, gz, dt_);
  }
  
  auto angles = filter_->getEulerAngles();
  
  // Should have valid angles
  EXPECT_TRUE(std::isfinite(angles[0]));
  EXPECT_TRUE(std::isfinite(angles[1]));
  EXPECT_TRUE(std::isfinite(angles[2]));
  
  // Quaternion should be normalized
  auto q = filter_->getQuaternion();
  double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  EXPECT_NEAR(norm, 1.0, 1e-10);
}

// Test 14: Zero gravity edge case (free fall)
TEST_F(MahonyFilterTest, ZeroGravityEdgeCase)
{
  filter_->reset();
  
  // Zero gravity (free fall) - accelerometer reads zero
  // Filter should still work with gyro only, but drift may occur
  for (int i = 0; i < 50; ++i) {
    filter_->update(0.0, 0.0, 0.0, 0.0, 0.0, 0.1, dt_);
  }
  
  auto q = filter_->getQuaternion();
  
  // Quaternion should still be valid (normalized)
  double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  EXPECT_NEAR(norm, 1.0, 1e-10);
}

// Test 15: Very large angular velocity
TEST_F(MahonyFilterTest, LargeAngularVelocity)
{
  filter_->reset();
  
  // Very high rotation rate (1000 deg/s)
  double extreme_gyro = 17.45;  // rad/s
  
  for (int i = 0; i < 10; ++i) {
    filter_->update(0.0, 0.0, 9.81, extreme_gyro, 0.0, 0.0, dt_);
  }
  
  auto q = filter_->getQuaternion();
  
  // Should still maintain normalized quaternion
  double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  EXPECT_NEAR(norm, 1.0, 1e-10);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
