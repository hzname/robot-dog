cmake_minimum_required(VERSION 3.16)
project(dog_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/calib_pose DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Own DDS domain: colcon runs packages in parallel and every test uses /dog.
  add_launch_test(test/test_mock_bringup.py TIMEOUT 120 ENV ROS_DOMAIN_ID=43)
endif()

ament_package()
