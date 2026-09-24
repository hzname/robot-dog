cmake_minimum_required(VERSION 3.16)
project(dog_teleop LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(dog_teleop_mapping src/mapping.cpp)
target_include_directories(dog_teleop_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_teleop_mapping PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(msg_libs rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(gamepad_node src/gamepad_node.cpp)
target_link_libraries(gamepad_node ${msg_libs})

add_executable(joy_teleop_node src/joy_teleop_node.cpp)
target_link_libraries(joy_teleop_node dog_teleop_mapping ${msg_libs})

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop dog_teleop_mapping ${msg_libs})

install(TARGETS dog_teleop_mapping EXPORT export_dog_teleop
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS gamepad_node joy_teleop_node keyboard_teleop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mapping test/test_mapping.cpp)
  target_link_libraries(test_mapping dog_teleop_mapping)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_gamepad_fifo.py TIMEOUT 60)
endif()

ament_export_targets(export_dog_teleop HAS_LIBRARY_TARGET)
ament_package()
