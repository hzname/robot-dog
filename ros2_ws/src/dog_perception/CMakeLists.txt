cmake_minimum_required(VERSION 3.16)
project(dog_perception LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-independent core: geometry, planes, detectors, guard (no Eigen: the
# robot image is ros-base without extra packages).
add_library(dog_perception_core src/core.cpp src/localization.cpp)
target_include_directories(dog_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node
  dog_perception_core
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

# map of the walls from the lidars, and where the robot is in it
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node
  dog_perception_core
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS dog_perception_core EXPORT export_dog_perception
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS perception_node localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Python twin of the core (numpy): used offline by dog_gazebo's
# perception_check and tools/sim_video, and as the reference in tests.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core dog_perception_core)
  ament_add_gtest(test_localization test/test_localization.cpp)
  target_link_libraries(test_localization dog_perception_core)
  ament_add_pytest_test(test_core_py test/test_core.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV OPENBLAS_NUM_THREADS=1)
endif()

ament_export_targets(export_dog_perception HAS_LIBRARY_TARGET)
ament_package()
