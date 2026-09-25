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

# ROS-independent core: kinematics, gait, state machine.
add_library(dog_control_core
  src/kinematics.cpp
  src/gait.cpp
  src/locomotion.cpp)
target_include_directories(dog_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(locomotion_node
  dog_control_core
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS dog_control_core EXPORT export_dog_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS locomotion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t kinematics gait locomotion)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} dog_control_core)
  endforeach()
endif()

ament_export_targets(export_dog_control HAS_LIBRARY_TARGET)
ament_package()
