/**
 * @file mpu6050_driver.hpp
 * @brief MPU6050 I2C driver with Mahony sensor fusion
 * 
 * Supports:
 * - I2C communication (/dev/i2c-1)
 * - Accelerometer (±2g default)
 * - Gyroscope (±250°/s default)
 * - Digital Low Pass Filter (DLPF)
 * - Mahony AHRS for orientation estimation
 */

#ifndef DOG_SENSORS_CPP__MPU6050_DRIVER_HPP_
#define DOG_SENSORS_CPP__MPU6050_DRIVER_HPP_

#include <string>
#include <array>
#include <memory>

namespace dog_sensors_cpp
{

/**
 * @brief Raw sensor data from MPU6050
 */
struct Mpu6050RawData
{
  int16_t accel_x{0};  ///< Raw accelerometer X
  int16_t accel_y{0};  ///< Raw accelerometer Y
  int16_t accel_z{0};  ///< Raw accelerometer Z
  int16_t gyro_x{0};   ///< Raw gyroscope X
  int16_t gyro_y{0};   ///< Raw gyroscope Y
  int16_t gyro_z{0};   ///< Raw gyroscope Z
  int16_t temperature{0};  ///< Raw temperature
};

/**
 * @brief Processed sensor data in SI units
 */
struct Mpu6050Data
{
  double accel_x{0.0};    ///< Acceleration X (m/s²)
  double accel_y{0.0};    ///< Acceleration Y (m/s²)
  double accel_z{0.0};    ///< Acceleration Z (m/s²)
  double gyro_x{0.0};     ///< Angular velocity X (rad/s)
  double gyro_y{0.0};     ///< Angular velocity Y (rad/s)
  double gyro_z{0.0};     ///< Angular velocity Z (rad/s)
  double temperature{0.0};  ///< Temperature (°C)
  
  // Orientation quaternion (from Mahony filter)
  double q_w{1.0};
  double q_x{0.0};
  double q_y{0.0};
  double q_z{0.0};
  
  // Euler angles (computed from quaternion)
  double roll{0.0};   ///< Roll angle (rad)
  double pitch{0.0};  ///< Pitch angle (rad)
  double yaw{0.0};    ///< Yaw angle (rad)
};

/**
 * @brief MPU6050 full-scale ranges
 */
enum class Mpu6050AccelRange : uint8_t
{
  RANGE_2G = 0,   ///< ±2g
  RANGE_4G = 1,   ///< ±4g
  RANGE_8G = 2,   ///< ±8g
  RANGE_16G = 3   ///< ±16g
};

enum class Mpu6050GyroRange : uint8_t
{
  RANGE_250DPS = 0,   ///< ±250°/s
  RANGE_500DPS = 1,   ///< ±500°/s
  RANGE_1000DPS = 2,  ///< ±1000°/s
  RANGE_2000DPS = 3   ///< ±2000°/s
};

/**
 * @brief DLPF (Digital Low Pass Filter) bandwidth settings
 */
enum class Mpu6050DlpfBandwidth : uint8_t
{
  BW_260HZ = 0,   ///< 260 Hz (Fs=8kHz)
  BW_184HZ = 1,   ///< 184 Hz
  BW_94HZ = 2,    ///< 94 Hz
  BW_44HZ = 3,    ///< 44 Hz
  BW_21HZ = 4,    ///< 21 Hz
  BW_10HZ = 5,    ///< 10 Hz
  BW_5HZ = 6      ///< 5 Hz
};

/**
 * @brief MPU6050 I2C driver class
 */
class Mpu6050Driver
{
public:
  /**
   * @brief Constructor
   * @param i2c_bus I2C bus path (e.g., "/dev/i2c-1")
   * @param address I2C address (0x68 for AD0=GND, 0x69 for AD0=VCC)
   */
  Mpu6050Driver(const std::string & i2c_bus = "/dev/i2c-1", uint8_t address = 0x68);
  
  /**
   * @brief Destructor
   */
  ~Mpu6050Driver();

  // Non-copyable
  Mpu6050Driver(const Mpu6050Driver &) = delete;
  Mpu6050Driver & operator=(const Mpu6050Driver &) = delete;

  /**
   * @brief Initialize the MPU6050
   * @return true if successful
   */
  bool initialize();

  /**
   * @brief Check if device is connected and responding
   * @return true if device found
   */
  bool is_connected() const;

  /**
   * @brief Configure accelerometer full-scale range
   * @param range Full-scale range
   * @return true if successful
   */
  bool set_accel_range(Mpu6050AccelRange range);

  /**
   * @brief Configure gyroscope full-scale range
   * @param range Full-scale range
   * @return true if successful
   */
  bool set_gyro_range(Mpu6050GyroRange range);

  /**
   * @brief Configure DLPF bandwidth
   * @param bandwidth DLPF bandwidth
   * @return true if successful
   */
  bool set_dlpf_bandwidth(Mpu6050DlpfBandwidth bandwidth);

  /**
   * @brief Set sample rate divider
   * @param divider Sample rate = gyro_rate / (1 + divider)
   * @return true if successful
   */
  bool set_sample_rate_divider(uint8_t divider);

  /**
   * @brief Read raw sensor data
   * @return Raw data structure
   */
  Mpu6050RawData read_raw();

  /**
   * @brief Read and process sensor data with orientation
   * @param dt Time step for filter update (seconds)
   * @return Processed data with orientation
   */
  Mpu6050Data read(double dt);

  /**
   * @brief Get latest processed data
   * @return Last processed data
   */
  Mpu6050Data get_data() const { return data_; }

  /**
   * @brief Calibrate gyroscope biases (call when stationary)
   * @param samples Number of samples to average
   * @return true if calibration successful
   */
  bool calibrate_gyro(size_t samples = 500);

  /**
   * @brief Set accelerometer bias offsets
   * @param x X-axis bias (m/s²)
   * @param y Y-axis bias (m/s²)
   * @param z Z-axis bias (m/s²)
   */
  void set_accel_bias(double x, double y, double z);

  /**
   * @brief Set gyroscope bias offsets
   * @param x X-axis bias (rad/s)
   * @param y Y-axis bias (rad/s)
   * @param z Z-axis bias (rad/s)
   */
  void set_gyro_bias(double x, double y, double z);

  /**
   * @brief Reset the Mahony filter
   */
  void reset_filter();

  /**
   * @brief Get accelerometer scale factor (m/s² per LSB)
   * @return Scale factor
   */
  double get_accel_scale() const { return accel_scale_; }

  /**
   * @brief Get gyroscope scale factor (rad/s per LSB)
   * @return Scale factor
   */
  double get_gyro_scale() const { return gyro_scale_; }

private:
  // I2C communication
  bool i2c_write_byte(uint8_t reg, uint8_t data);
  bool i2c_read_bytes(uint8_t reg, uint8_t * buffer, size_t length);
  bool i2c_read_byte(uint8_t reg, uint8_t & data);

  // Scale raw data to SI units
  void scale_data(const Mpu6050RawData & raw);

  // Mahony AHRS filter
  void mahony_update(double ax, double ay, double az, double gx, double gy, double gz, double dt);
  void compute_euler_angles();

  // I2C
  std::string i2c_bus_;
  uint8_t address_;
  int fd_{-1};

  // Configuration
  Mpu6050AccelRange accel_range_{Mpu6050AccelRange::RANGE_2G};
  Mpu6050GyroRange gyro_range_{Mpu6050GyroRange::RANGE_250DPS};
  double accel_scale_{9.80665 / 16384.0};  // Default for ±2g
  double gyro_scale_{0.0174533 / 131.0};   // Default for ±250°/s (rad/s per LSB)

  // Calibration offsets
  double gyro_bias_x_{0.0};
  double gyro_bias_y_{0.0};
  double gyro_bias_z_{0.0};
  double accel_bias_x_{0.0};
  double accel_bias_y_{0.0};
  double accel_bias_z_{0.0};

  // Filter state
  Mpu6050Data data_;
  double mahony_q0_{1.0}, mahony_q1_{0.0}, mahony_q2_{0.0}, mahony_q3_{0.0};  // Quaternion
  double mahony_integral_x_{0.0}, mahony_integral_y_{0.0}, mahony_integral_z_{0.0};  // Integral error
  static constexpr double MAHONY_KP_{2.0};   // Proportional gain
  static constexpr double MAHONY_KI_{0.005}; // Integral gain
};

}  // namespace dog_sensors_cpp

#endif  // DOG_SENSORS_CPP__MPU6050_DRIVER_HPP_
