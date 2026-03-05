cmake_minimum_required(VERSION 3.8)
project(dog_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# Gait Controller
add_executable(gait_controller
  src/gait_controller.cpp
)

ament_target_dependencies(gait_controller
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_msgs
  tf2
)

# Balance Controller
add_executable(balance_controller
  src/balance_controller.cpp
)

ament_target_dependencies(balance_controller
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
)

# IMU Simulator (for testing)
add_executable(imu_simulator
  src/imu_simulator.cpp
)

ament_target_dependencies(imu_simulator
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS 
  gait_controller 
  balance_controller
  imu_simulator
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
endif()

ament_package()
