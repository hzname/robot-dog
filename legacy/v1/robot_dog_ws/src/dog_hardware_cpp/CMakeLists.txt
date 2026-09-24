cmake_minimum_required(VERSION 3.8)
project(dog_hardware_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Servo driver node
add_executable(servo_driver_node
  src/servo_driver_node.cpp
  src/servo_controller.cpp
  src/pca9685_driver.cpp
  src/simulation_driver.cpp
)

ament_target_dependencies(servo_driver_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  trajectory_msgs
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS servo_driver_node
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
  
  # Servo controller tests
  ament_add_gtest(test_servo_controller test/test_servo_controller.cpp)
  target_include_directories(test_servo_controller PRIVATE include)
  
  # PCA9685 driver tests
  ament_add_gtest(test_pca9685_driver test/test_pca9685_driver.cpp)
  target_include_directories(test_pca9685_driver PRIVATE include)
endif()

ament_package()
