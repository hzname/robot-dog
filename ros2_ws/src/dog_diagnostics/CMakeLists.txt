# dog_diagnostics — one way for every node to report itself
#
# Header-only on purpose: the reporting rule is a few lines, but it must be the
# same few lines in the driver, the IMU, the power monitor and the controller,
# otherwise the page shows four different vocabularies for the same fault.

cmake_minimum_required(VERSION 3.8)
project(dog_diagnostics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(dog_diagnostics INTERFACE)
target_include_directories(dog_diagnostics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dog_diagnostics INTERFACE
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dog_diagnostics EXPORT export_dog_diagnostics)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp diagnostic_msgs)
ament_export_targets(export_dog_diagnostics HAS_LIBRARY_TARGET)
ament_package()
