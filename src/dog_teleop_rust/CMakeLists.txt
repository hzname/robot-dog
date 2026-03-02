cmake_minimum_required(VERSION 3.8)
project(dog_teleop_rust)

find_package(ament_cmake REQUIRED)

# Build Rust project using cargo
add_custom_target(${PROJECT_NAME}_build ALL
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building ${PROJECT_NAME} with cargo"
)

# Install the binary
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/target/release/teleop_rust
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
