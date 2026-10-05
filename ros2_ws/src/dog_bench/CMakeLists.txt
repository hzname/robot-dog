cmake_minimum_required(VERSION 3.16)
project(dog_bench LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ROS-independent core: I2C transactions and the fast INA219 polling layer.
add_library(dog_bench_core
  src/i2c_bus.cpp
  src/ina219_fast.cpp
  src/selftest.cpp
  src/ramp.cpp
  src/safety.cpp
  src/pwm_out.cpp
  src/session.cpp)
target_include_directories(dog_bench_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_bench_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(servo_speed_test src/servo_speed_test_main.cpp)
target_link_libraries(servo_speed_test dog_bench_core)

install(TARGETS dog_bench_core EXPORT export_dog_bench
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS servo_speed_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t ina219_fast selftest ramp safety pwm_out session)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} dog_bench_core)
  endforeach()
endif()

ament_export_targets(export_dog_bench HAS_LIBRARY_TARGET)
ament_package()
