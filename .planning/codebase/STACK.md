---
last_mapped_commit: 104fd262ee2afbd8c9ca25310c9647ab91815a75
last_mapped_at: 2026-09-29
---
# Technology Stack

**Analysis Date:** 2026-09-29

## Languages

**Primary:**

- **C++ (17)** — Locomotion control, hardware drivers, perception core, teleoperation
  - Used in: `ros2_ws/src/dog_hardware/`, `ros2_ws/src/dog_control/`, `ros2_ws/src/dog_teleop/`, `ros2_ws/src/dog_perception/`
  - Compilation flags: `-Wall -Wextra -Wpedantic` with `-DCMAKE_BUILD_TYPE=Release` for robot
  
- **Python (3.x)** — Web interface, simulation utilities, robot setup, calibration tools
  - Used in: `ros2_ws/src/dog_web/`, `ros2_ws/src/dog_gazebo/`, `ros2_ws/src/dog_description/`, `tools/`
  - Tested with Python 3.12 in CI

**Secondary:**

- **Shell (Bash)** — Build and deployment scripts, CI workflows, testing harnesses
  - Used in: `docker/entrypoint.sh`, `test_servo_config_reader.sh`, `.github/workflows/ci.yml`

- **XML** — ROS package descriptors and configuration
  - All packages use `package.xml` format 3

## Runtime

**Environment:**

- **ROS 2 (Jazzy & Lyrical)** — Middleware for inter-process communication, message passing, node coordination
  - Base image: `ros:${ROS_DISTRO}-ros-base` (lightweight, no Gazebo in production Docker image)
  - Simulation image: `osrf/ros:${ROS_DISTRO}-simulation` (includes Gazebo)
  - Used across all packages; message types: `geometry_msgs`, `sensor_msgs`, `std_msgs`, `nav_msgs`

- **Gazebo (Harmonic & Jetty)** — Physics simulation for gait validation, terrain testing, perception validation
  - Only loaded in simulation environment; not in production robot Docker image
  - Used for walk_check validation and terrain sweep testing in CI

**Package Manager:**

- **colcon** — ROS 2 build system orchestrator
  - Manages workspace builds, testing, and artifact collection
  - Configured with parallel workers (limited to 1 for Pi: 2 GB RAM constraint)
  - Build flags: `MAKEFLAGS="-j${BUILD_JOBS}"`, `CMAKE_BUILD_TYPE=Release`

- **pip** — Python package management
  - Used in CI for test dependencies and tools
  - No `requirements.txt` for main workspace; ROS dependencies declared in `package.xml`

## Frameworks

**Core:**

- **ROS 2 rclcpp** (2.x, via Jazzy/Lyrical) — C++ ROS client library for nodes, publishers, subscribers, services
  - Hardware drivers, control, teleoperation, perception all use `rclcpp::Node`
  - Example: `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`

- **ROS 2 rclpy** (2.x, via Jazzy/Lyrical) — Python ROS client library
  - Web interface, description generation, Gazebo simulation use `rclpy.node.Node`
  - Example: `ros2_ws/src/dog_web/dog_web/web_teleop.py`

- **launch & launch_ros** — ROS 2 launch system for multi-node orchestration
  - Configured in: `ros2_ws/src/dog_bringup/launch/robot.launch.py`
  - Parameters passed at runtime (e.g., `backend:=pca9685`, `web:=true`, `gamepad:=true`)

**Testing:**

- **Gtest (ament_cmake_gtest)** — C++ unit tests for kinematics, gait, hardware, perception
  - Test command: `colcon test --packages-skip dog_gazebo`
  - 109 tests across hardware, control, teleop, web, perception

- **pytest (Python)** — Python unit tests for web, robot setup, autocal, perception
  - Integrated via `ament_cmake_pytest` in ROS packages
  - CI command: `pip install pytest` + `python -m pytest -q`

- **launch_testing_ament_cmake / launch_testing_ros** — Integration tests for multi-node scenarios
  - Mock bringup test: `ros2_ws/src/dog_bringup/test/test_mock_bringup.py`
  - Separate DDS domain (ROS_DOMAIN_ID=43) to avoid colcon parallel test conflicts

**Build/Dev:**

- **CMake 3.16+** — C++ project build configuration
  - Declarative dependency resolution via `find_package()` (ament_cmake, rclcpp, message types)
  - Per-package CMakeLists.txt in `ros2_ws/src/dog_*/`

- **ament_cmake_python** — Dual C++/Python builds (e.g., dog_perception has core.cpp + Python numpy twin)
  - Allows data processing tools to share logic with the C++ perception node

## Key Dependencies

**ROS 2 Infrastructure (Jazzy/Lyrical, included in base image):**

- `rclcpp`, `rclpy` — Pub/sub, services, parameters, logging
- `geometry_msgs` — Twist (velocity commands), Pose, Transform
- `sensor_msgs` — JointState (servo positions), Imu (MPU6050), LaserScan (lidar), PointCloud2 (perception)
- `std_msgs` — Bool (e-stop), Int32, Float32
- `nav_msgs` — Odometry
- `tf2_ros` — Transform broadcasts for perception → locomotion feedback
- `robot_state_publisher` — Publishes URDF joint state transforms for visualization
- `ros_gz_sim`, `ros_gz_bridge` — Gazebo integration (simulation only)

**System Libraries (Linux, included in base image):**

- `linux/i2c-dev.h`, `sys/ioctl.h` — I2C bus communication for PCA9685 servo driver and MPU6050 IMU
- Standard C++ stdlib (string, vector, memory, chrono, etc.)

**Python (declared in tools, CI only):**

- `numpy >= 1.24` — Numerical arrays for perception geometry, gait math (dual C++/Python in dog_perception)
- `opencv-contrib-python >= 4.6` — ArUco marker detection for servo calibration (`tools/autocal/requirements.txt`)
  - Alternative: `opencv-contrib-python-headless` on headless systems
- `pyyaml` — Configuration parsing for robot geometry and servo calibration (`tools/robot_setup/robot_setup.py`)

**Hardware Driver Support (no explicit packages; Linux kernel provides):**

- `/dev/i2c-0` — I2C device node for PCA9685 (servo controller, address 0x40) and MPU6050 (IMU, addresses 0x68/0x69)
- `/dev/input/js*` — Linux joystick API for gamepad input (dog_teleop reads this via raw device file)
- `/dev/input/event*` — Keyboard input in terminal or web server

## Configuration

**Environment:**

- **ROS_DOMAIN_ID** — Separates ROS 2 DDS networks
  - Default: `0` for production robot
  - CI tests: `43` (isolated to avoid parallel test conflicts)
  - Declared in `docker-compose.yml` and test launch files

- **ROS_DISTRO** — Runtime distro selection
  - Docker build arg: `ARG ROS_DISTRO=jazzy`
  - Supports: Jazzy (default), Lyrical (alternative)
  - Source path: `/opt/ros/${ROS_DISTRO}/setup.bash`

**Build:**

- `docker/Dockerfile` — Multi-stage runtime image for Banana Pi (arm64) or PC (amd64)
  - Base: `ros:${ROS_DISTRO}-ros-base` (minimized, no extra packages like xacro/joy/ros2_control)
  - Build jobs limited: `ARG BUILD_JOBS=2` (2 GB RAM on Pi)
  - Skips dog_gazebo during build: `--packages-skip dog_gazebo`

- `docker/Dockerfile.sim` — Optional simulation image for PC development
  - Includes full Gazebo stack

- `.github/workflows/ci.yml` — GitHub Actions CI/CD
  - Matrix test: Ubuntu 24.04 containers with Jazzy and Lyrical
  - Builds, unit tests, integration tests, Gazebo walk_check, terrain sweep

**Runtime Parameters (robot.launch.py):**

- `backend:=pca9685` — Hardware driver (PCA9685 for real robot, `mock` for simulation, `auto` to probe)
- `gamepad:=true` — Enable gamepad teleoperation node
- `web:=true` — Enable web server on port 8080
- `rviz:=true` — Launch RViz visualization (dev/debugging only)
- Servo calibration and geometry parameters loaded from `ros2_ws/src/dog_bringup/config/`

**Device Access:**

- I2C: `/dev/i2c-0` mounted as `ro` in docker-compose (servo controller + IMU)
- Input devices: `/dev/input/*` mounted, with `device_cgroup_rules` to permit dynamic plugging
- No network exposure in production (host network mode for ROS 2 DDS discovery)

## Platform Requirements

**Development:**

- **Ubuntu 24.04** (CI baseline; WSL2 supported for local development)
- **ROS 2 Jazzy or Lyrical** installed or docker image pulled
- **colcon** + **CMake 3.16+**
- **Python 3.12** recommended (CI tested version)
- **Gazebo Harmonic or Jetty** (optional, for simulation)
- **Git** for version control

**Production (Banana Pi BPI-M4 Zero):**

- **Armbian** Linux (stripped, minimal overhead)
- **Docker** runtime
- **I2C interface** enabled on `/dev/i2c-0` (PCA9685 at 0x40, MPU6050 at 0x68)
- **USB or Bluetooth gamepad** (optional; keyboard/web always available)
- **Network connectivity** for web UI (`--net=host` in docker-compose)
- **2 GB RAM** (build-time constraint; runtime is lighter)

**Hardware Sensors/Actuators:**

- **12 × MG996R servos** (PCA9685 PWM driver at 50 Hz)
- **MPU6050 IMU** (accelerometer + gyroscope on I2C)
- **Optional:** Cross-mounted single-beam lidars, GS2 line lidar, VL53L1X ToF sensors (all via ROS topics, not direct hardware)
- **Optional:** USB camera (for autocal tool; not used in main control loop)

---

*Stack analysis: 2026-09-29*
