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

# Gait Controller Library
add_library(gait_controller_lib SHARED
  src/gait_controller.cpp
)
target_include_directories(gait_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(gait_controller_lib PUBLIC cxx_std_17)
ament_target_dependencies(gait_controller_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_msgs
  tf2
)

# Gait Controller Executable
add_executable(gait_controller src/main_gait_controller.cpp)
target_link_libraries(gait_controller PRIVATE gait_controller_lib)

# Balance Controller
add_executable(balance_controller
  src/balance_controller.cpp
)
target_include_directories(balance_controller PRIVATE include)
ament_target_dependencies(balance_controller
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
)

# IMU Simulator
add_executable(imu_simulator
  src/imu_simulator.cpp
)
target_include_directories(imu_simulator PRIVATE include)
ament_target_dependencies(imu_simulator
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install
install(TARGETS gait_controller_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gait_controller balance_controller imu_simulator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
