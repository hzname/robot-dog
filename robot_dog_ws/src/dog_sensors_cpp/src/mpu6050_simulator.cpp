/**
 * @file mpu6050_simulator.cpp
 * @brief MPU6050 simulator implementation
 */

#include "dog_sensors_cpp/mpu6050_simulator.hpp"

#include <math>

namespace dog_sensors_cpp
{

Mpu6050Simulator::Mpu6050Simulator(const SimulatorConfig & config)
: config_(config)
{
  // Seed RNG with time-based seed
  std::random_device rd;
  rng_.seed(rd());
  
  // Initialize orientation from config
  roll_ = config.initial_roll;
  pitch_ = config.initial_pitch;
  yaw_ = config.initial_yaw;
  prev_roll_ = roll_;
  prev_pitch_ = pitch_;
  prev_yaw_ = yaw_;
}

bool Mpu6050Simulator::initialize()
{
  reset();
  return true;
}

void Mpu6050Simulator::reset()
{
  sim_time_ = 0.0;
  pos_x_ = 0.0;
  pos_y_ = 0.0;
  pos_z_ = 0.0;
  roll_ = config_.initial_roll;
  pitch_ = config_.initial_pitch;
  yaw_ = config_.initial_yaw;
  body_roll_ = 0.0;
  body_pitch_ = 0.0;
  body_height_ = 0.0;
  drift_x_ = 0.0;
  drift_y_ = 0.0;
  drift_z_ = 0.0;
  prev_roll_ = roll_;
  prev_pitch_ = pitch_;
  prev_yaw_ = yaw_;
  prev_vel_x_ = 0.0;
  prev_vel_y_ = 0.0;
  prev_vel_z_ = 0.0;
  
  // Reset Mahony filter
  mahony_q0_ = 1.0;
  mahony_q1_ = 0.0;
  mahony_q2_ = 0.0;
  mahony_q3_ = 0.0;
  mahony_integral_x_ = 0.0;
  mahony_integral_y_ = 0.0;
  mahony_integral_z_ = 0.0;
  
  // Initialize data
  data_ = Mpu6050Data{};
}

Mpu6050Data Mpu6050Simulator::read(double dt)
{
  // Update simulation time
  sim_time_ += dt;
  
  // Update motion based on current mode
  update_motion(dt);
  
  // Compute IMU readings
  compute_imu_readings(dt);
  
  return data_;
}

void Mpu6050Simulator::set_motion_mode(MotionMode mode)
{
  if (mode_ != mode) {
    mode_ = mode;
    // Reset body motion when changing modes
    body_roll_ = 0.0;
    body_pitch_ = 0.0;
    body_height_ = 0.0;
  }
}

void Mpu6050Simulator::set_velocity_command(double linear_x, double linear_y, double angular_z)
{
  cmd_linear_x_ = linear_x;
  cmd_linear_y_ = linear_y;
  cmd_angular_z_ = angular_z;
  
  // Auto-select mode based on velocity
  if (std::abs(angular_z) > 0.3) {
    set_motion_mode(MotionMode::TURNING);
  } else if (std::abs(linear_x) > 0.8) {
    set_motion_mode(MotionMode::TROTTING);
  } else if (std::abs(linear_x) > 0.1) {
    set_motion_mode(MotionMode::WALKING);
  } else if (std::abs(linear_x) > 0.01 || std::abs(linear_y) > 0.01) {
    set_motion_mode(MotionMode::PACING);
  } else {
    set_motion_mode(MotionMode::IDLE);
  }
}

double Mpu6050Simulator::generate_noise(double std_dev)
{
  return noise_dist_(rng_) * std_dev;
}

void Mpu6050Simulator::update_motion(double dt)
{
  // Update drift
  if (config_.simulate_drift) {
    drift_x_ += generate_noise(config_.drift_rate * dt);
    drift_y_ += generate_noise(config_.drift_rate * dt);
    drift_z_ += generate_noise(config_.drift_rate * dt);
  }
  
  // Save previous orientation for angular velocity calculation
  prev_roll_ = roll_;
  prev_pitch_ = pitch_;
  prev_yaw_ = yaw_;
  
  // Update based on motion mode
  switch (mode_) {
    case MotionMode::IDLE:
      // Small random vibrations
      body_roll_ = 0.02 * std::sin(2.0 * M_PI * 2.0 * sim_time_) + 
                   generate_noise(0.005);
      body_pitch_ = 0.01 * std::sin(2.0 * M_PI * 1.5 * sim_time_ + 1.0) + 
                    generate_noise(0.005);
      body_height_ = 0.001 * std::sin(2.0 * M_PI * 10.0 * sim_time_);
      break;
      
    case MotionMode::WALKING:
    case MotionMode::PACING: {
      // Walking gait: sinusoidal body motion
      double gait_phase = 2.0 * M_PI * config_.walk_frequency * sim_time_;
      
      // Body pitch oscillates with gait
      body_pitch_ = config_.walk_amplitude * 0.3 * std::sin(gait_phase);
      
      // Body roll for lateral balance
      body_roll_ = 0.05 * std::sin(gait_phase * 2.0) + generate_noise(0.01);
      
      // Vertical oscillation
      body_height_ = config_.step_height * 0.5 * (1.0 + std::sin(gait_phase * 2.0));
      
      // Update yaw from commanded angular velocity
      yaw_ += cmd_angular_z_ * dt;
      
      // Update position from commanded velocity
      pos_x_ += (cmd_linear_x_ * std::cos(yaw_) - cmd_linear_y_ * std::sin(yaw_)) * dt;
      pos_y_ += (cmd_linear_x_ * std::sin(yaw_) + cmd_linear_y_ * std::cos(yaw_)) * dt;
      break;
    }
    
    case MotionMode::TROTTING: {
      // Faster gait with higher frequency
      double gait_phase = 2.0 * M_PI * config_.walk_frequency * 1.5 * sim_time_;
      
      body_pitch_ = config_.walk_amplitude * 0.5 * std::sin(gait_phase);
      body_roll_ = 0.08 * std::sin(gait_phase * 2.0) + generate_noise(0.015);
      body_height_ = config_.step_height * 0.8 * std::abs(std::sin(gait_phase));
      
      yaw_ += cmd_angular_z_ * dt;
      pos_x_ += (cmd_linear_x_ * std::cos(yaw_) - cmd_linear_y_ * std::sin(yaw_)) * dt;
      pos_y_ += (cmd_linear_x_ * std::sin(yaw_) + cmd_linear_y_ * std::cos(yaw_)) * dt;
      break;
    }
    
    case MotionMode::TURNING: {
      // Turning in place
      double turn_phase = 2.0 * M_PI * config_.walk_frequency * 0.5 * sim_time_;
      
      body_pitch_ = 0.03 * std::sin(turn_phase);
      body_roll_ = 0.1 * cmd_angular_z_ + 0.02 * std::sin(turn_phase * 2.0);
      body_height_ = config_.step_height * 0.3 * (1.0 + std::sin(turn_phase * 2.0));
      
      yaw_ += cmd_angular_z_ * dt;
      break;
    }
    
    case MotionMode::CUSTOM:
      // Use commanded velocities directly
      yaw_ += cmd_angular_z_ * dt;
      pos_x_ += (cmd_linear_x_ * std::cos(yaw_) - cmd_linear_y_ * std::sin(yaw_)) * dt;
      pos_y_ += (cmd_linear_x_ * std::sin(yaw_) + cmd_linear_y_ * std::cos(yaw_)) * dt;
      break;
  }
  
  // Combine base orientation with body motion
  roll_ = config_.initial_roll + body_roll_;
  pitch_ = config_.initial_pitch + body_pitch_;
  // yaw is already updated above
}

void Mpu6050Simulator::compute_imu_readings(double dt)
{
  // Compute angular velocity from orientation change
  double d_roll = roll_ - prev_roll_;
  double d_pitch = pitch_ - prev_pitch_;
  double d_yaw = yaw_ - prev_yaw_;
  
  // Handle wraparound for yaw
  if (d_yaw > M_PI) d_yaw -= 2.0 * M_PI;
  if (d_yaw < -M_PI) d_yaw += 2.0 * M_PI;
  
  // Convert to body frame angular velocity
  // ω_x = d_roll - sin(pitch) * d_yaw
  // ω_y = cos(roll) * d_pitch + sin(roll) * cos(pitch) * d_yaw
  // ω_z = -sin(roll) * d_pitch + cos(roll) * cos(pitch) * d_yaw
  double cr = std::cos(roll_);
  double sr = std::sin(roll_);
  double cp = std::cos(pitch_);
  double sp = std::sin(pitch_);
  
  double gyro_x = (d_roll - sp * d_yaw) / dt;
  double gyro_y = (cr * d_pitch + sr * cp * d_yaw) / dt;
  double gyro_z = (-sr * d_pitch + cr * cp * d_yaw) / dt;
  
  // Add drift
  gyro_x += drift_x_;
  gyro_y += drift_y_;
  gyro_z += drift_z_;
  
  // Add noise
  gyro_x += generate_noise(config_.gyro_noise_std);
  gyro_y += generate_noise(config_.gyro_noise_std);
  gyro_z += generate_noise(config_.gyro_noise_std);
  
  // Compute accelerations
  // Gravity in body frame (rotated by roll, pitch, yaw)
  double g = 9.80665;
  double grav_x = -g * sp;
  double grav_y = g * sr * cp;
  double grav_z = g * cr * cp;
  
  // Linear accelerations from body motion
  // Add some realistic accelerations based on gait
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;
  
  if (mode_ != MotionMode::IDLE) {
    double gait_phase = 2.0 * M_PI * config_.walk_frequency * sim_time_;
    
    // Forward acceleration oscillation
    accel_x = 0.5 * config_.walk_amplitude * std::cos(gait_phase);
    
    // Lateral acceleration during gait
    accel_y = 0.1 * std::sin(gait_phase * 2.0);
    
    // Vertical acceleration from stepping
    accel_z = 2.0 * config_.step_height * 
              std::pow(2.0 * M_PI * config_.walk_frequency, 2) *
              std::sin(gait_phase * 2.0);
  }
  
  // Combine gravity and linear acceleration
  double accel_x_total = grav_x + accel_x + generate_noise(config_.accel_noise_std);
  double accel_y_total = grav_y + accel_y + generate_noise(config_.accel_noise_std);
  double accel_z_total = grav_z + accel_z + generate_noise(config_.accel_noise_std);
  
  // Store raw readings
  data_.gyro_x = gyro_x;
  data_.gyro_y = gyro_y;
  data_.gyro_z = gyro_z;
  data_.accel_x = accel_x_total;
  data_.accel_y = accel_y_total;
  data_.accel_z = accel_z_total;
  data_.temperature = 25.0 + generate_noise(0.5);  // Room temperature + noise
  
  // Update Mahony filter for orientation estimation
  mahony_update(accel_x_total, accel_y_total, accel_z_total,
                gyro_x, gyro_y, gyro_z, dt);
  
  // Store quaternion
  data_.q_w = mahony_q0_;
  data_.q_x = mahony_q1_;
  data_.q_y = mahony_q2_;
  data_.q_z = mahony_q3_;
  
  // Compute and store Euler angles
  double sinr_cosp = 2.0 * (mahony_q0_ * mahony_q1_ + mahony_q2_ * mahony_q3_);
  double cosr_cosp = 1.0 - 2.0 * (mahony_q1_ * mahony_q1_ + mahony_q2_ * mahony_q2_);
  data_.roll = std::atan2(sinr_cosp, cosr_cosp);
  
  double sinp = 2.0 * (mahony_q0_ * mahony_q2_ - mahony_q3_ * mahony_q1_);
  if (std::abs(sinp) >= 1.0) {
    data_.pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    data_.pitch = std::asin(sinp);
  }
  
  double siny_cosp = 2.0 * (mahony_q0_ * mahony_q3_ + mahony_q1_ * mahony_q2_);
  double cosy_cosp = 1.0 - 2.0 * (mahony_q2_ * mahony_q2_ + mahony_q3_ * mahony_q3_);
  data_.yaw = std::atan2(siny_cosp, cosy_cosp);
}

void Mpu6050Simulator::mahony_update(double ax, double ay, double az,
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
  double vx = 2.0 * (mahony_q1_ * mahony_q3_ - mahony_q0_ * mahony_q2_);
  double vy = 2.0 * (mahony_q0_ * mahony_q1_ + mahony_q2_ * mahony_q3_);
  double vz = mahony_q0_ * mahony_q0_ - mahony_q1_ * mahony_q1_ -
              mahony_q2_ * mahony_q2_ + mahony_q3_ * mahony_q3_;
  
  // Error is cross product between estimated and measured direction of gravity
  double ex = ay * vz - az * vy;
  double ey = az * vx - ax * vz;
  double ez = ax * vy - ay * vx;
  
  // Accumulate integral error
  mahony_integral_x_ += ex * MAHONY_KI_ * dt;
  mahony_integral_y_ += ey * MAHONY_KI_ * dt;
  mahony_integral_z_ += ez * MAHONY_KI_ * dt;
  
  // Apply feedback terms
  gx += MAHONY_KP_ * ex + mahony_integral_x_;
  gy += MAHONY_KP_ * ey + mahony_integral_y_;
  gz += MAHONY_KP_ * ez + mahony_integral_z_;
  
  // Integrate rate of change of quaternion
  double q0_dot = 0.5 * (-mahony_q1_ * gx - mahony_q2_ * gy - mahony_q3_ * gz);
  double q1_dot = 0.5 * (mahony_q0_ * gx + mahony_q2_ * gz - mahony_q3_ * gy);
  double q2_dot = 0.5 * (mahony_q0_ * gy - mahony_q1_ * gz + mahony_q3_ * gx);
  double q3_dot = 0.5 * (mahony_q0_ * gz + mahony_q1_ * gy - mahony_q2_ * gx);
  
  q0_dot *= dt;
  q1_dot *= dt;
  q2_dot *= dt;
  q3_dot *= dt;
  
  mahony_q0_ += q0_dot;
  mahony_q1_ += q1_dot;
  mahony_q2_ += q2_dot;
  mahony_q3_ += q3_dot;
  
  // Normalize quaternion
  norm = std::sqrt(mahony_q0_ * mahony_q0_ + mahony_q1_ * mahony_q1_ +
                   mahony_q2_ * mahony_q2_ + mahony_q3_ * mahony_q3_);
  if (norm > 0.0) {
    norm = 1.0 / norm;
    mahony_q0_ *= norm;
    mahony_q1_ *= norm;
    mahony_q2_ *= norm;
    mahony_q3_ *= norm;
  }
}

}  // namespace dog_sensors_cpp
