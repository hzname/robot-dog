cmake_minimum_required(VERSION 3.16)
project(dog_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(dog_hardware_core
  src/servo_bus.cpp
  src/servo_driver.cpp
  src/power_sensor.cpp
  src/imu_sensor.cpp)
target_include_directories(dog_hardware_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_hardware_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(servo_driver_node src/servo_driver_node.cpp)
target_link_libraries(servo_driver_node
  dog_hardware_core rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(power_monitor_node src/power_monitor_node.cpp)
target_link_libraries(power_monitor_node
  dog_hardware_core rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node
  dog_hardware_core rclcpp::rclcpp ${sensor_msgs_TARGETS})

add_executable(pca9685_probe src/pca9685_probe.cpp)
target_link_libraries(pca9685_probe dog_hardware_core)

install(TARGETS dog_hardware_core EXPORT export_dog_hardware
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS servo_driver_node power_monitor_node imu_node pca9685_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_servo_driver test/test_servo_driver.cpp)
  target_link_libraries(test_servo_driver dog_hardware_core)
  ament_add_gtest(test_power test/test_power.cpp)
  target_link_libraries(test_power dog_hardware_core)
  ament_add_gtest(test_imu test/test_imu.cpp)
  target_link_libraries(test_imu dog_hardware_core)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Own DDS domain: colcon runs packages in parallel and every test uses /dog.
  add_launch_test(test/test_power_monitor.py TIMEOUT 60 ENV ROS_DOMAIN_ID=41)
endif()

ament_export_targets(export_dog_hardware HAS_LIBRARY_TARGET)
ament_package()
