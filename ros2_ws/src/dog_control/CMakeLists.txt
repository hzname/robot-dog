cmake_minimum_required(VERSION 3.16)
project(dog_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dog_diagnostics REQUIRED)

# ROS-independent core: kinematics, gait, state machine.
add_library(dog_control_core
  src/kinematics.cpp
  src/gait.cpp
  src/crawl.cpp
  src/greet.cpp
  src/survey.cpp
  src/locomotion.cpp
  src/servo_limits.cpp)
target_include_directories(dog_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(locomotion_node
  dog_control_core
  dog_diagnostics::dog_diagnostics
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})

install(TARGETS dog_control_core EXPORT export_dog_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS locomotion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  foreach(t kinematics gait crawl greet locomotion servo_limits safety)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} dog_control_core)
  endforeach()
  # The fall chain with the real binaries: a mock IMU, the driver and the
  # controller, plus the diagnostics the page reads (SAF-05, DIA-02).
  add_launch_test(test/test_safety_launch.py TIMEOUT 90 ENV ROS_DOMAIN_ID=44)
endif()

ament_export_targets(export_dog_control HAS_LIBRARY_TARGET)
ament_package()
