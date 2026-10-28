cmake_minimum_required(VERSION 3.16)
project(dog_bringup NONE)

find_package(ament_cmake REQUIRED)

# profile.py sits next to robot.launch.py and is imported from there by path
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/calib_pose DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # Own DDS domain: colcon runs packages in parallel and every test uses /dog.
  add_launch_test(test/test_mock_bringup.py TIMEOUT 120 ENV ROS_DOMAIN_ID=43)
  # No ROS in this one: the floor profile is a table of numbers.
  ament_add_pytest_test(test_profile test/test_profile.py)
endif()

ament_package()
