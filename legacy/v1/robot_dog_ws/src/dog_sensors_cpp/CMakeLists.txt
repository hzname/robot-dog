cmake_minimum_required(VERSION 3.8)
project(dog_sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for modern ROS2 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# IMU Node executable with MPU6050 driver and simulator
add_executable(imu_node
  src/imu_node.cpp
  src/imu_node_main.cpp
  src/mpu6050_driver.cpp
  src/mpu6050_simulator.cpp
)

target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Install targets
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Find gtest
  find_package(ament_cmake_gtest REQUIRED)
  
  # Mahony Filter tests
  ament_add_gtest(test_mahony_filter test/test_mahony_filter.cpp)
  target_include_directories(test_mahony_filter PRIVATE include)
  
  # MPU6050 Simulator tests
  ament_add_gtest(test_mpu6050_simulator test/test_mpu6050_simulator.cpp)
  target_include_directories(test_mpu6050_simulator PRIVATE include)
endif()

ament_package()
