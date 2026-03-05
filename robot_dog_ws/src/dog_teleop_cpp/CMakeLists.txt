cmake_minimum_required(VERSION 3.8)
project(dog_teleop_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Keyboard teleop node (LifecycleNode)
add_executable(keyboard_teleop
  src/keyboard_teleop.cpp
)

ament_target_dependencies(keyboard_teleop
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Gamepad teleop node (LifecycleNode)
add_executable(gamepad_teleop
  src/gamepad_teleop.cpp
)

ament_target_dependencies(gamepad_teleop
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
)

# UDP teleop node (LifecycleNode)
add_executable(udp_teleop
  src/udp_teleop.cpp
)

ament_target_dependencies(udp_teleop
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS keyboard_teleop gamepad_teleop udp_teleop
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
