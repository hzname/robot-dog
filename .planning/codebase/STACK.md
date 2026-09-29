---
last_mapped_commit: d269bd16b0d0f69f3ab623b9cecc4ba545b70517
last_mapped_at: 2026-09-29
---
# Technology Stack

**Analysis Date:** 2026-09-29

## Languages

**Primary:**

- C++17 - Robot runtime: kinematics, gait, locomotion state machine, hardware drivers (I2C), perception, localization, teleop. Standard set per package with `set(CMAKE_CXX_STANDARD 17)` in `ros2_ws/src/*/CMakeLists.txt`; warnings `-Wall -Wextra -Wpedantic`.
  - `ros2_ws/src/dog_control/`, `ros2_ws/src/dog_hardware/`, `ros2_ws/src/dog_perception/`, `ros2_ws/src/dog_teleop/`
- Python 3.12 (Jazzy / Ubuntu 24.04) and 3.14 (Lyrical / Ubuntu 26.04) - ROS packages for URDF generation, Gazebo glue and checks, the web teleop server, launch files (`ros2_ws/src/dog_description/`, `ros2_ws/src/dog_gazebo/`, `ros2_ws/src/dog_web/`, `ros2_ws/src/dog_bringup/launch/robot.launch.py`), plus the offline tools in `tools/`.

**Secondary:**

- JavaScript (vanilla, no build step, no framework) - Browser teleop page: `ros2_ws/src/dog_web/static/app.js`, `ros2_ws/src/dog_web/static/index.html`, `ros2_ws/src/dog_web/static/style.css`.
- YAML - ROS parameter files and robot geometry: `ros2_ws/src/dog_bringup/config/*.yaml`.
- SDF / URDF (generated) - Gazebo world `ros2_ws/src/dog_gazebo/worlds/flat.sdf`; URDF generated at launch by `ros2_ws/src/dog_description/dog_description/urdf.py` (no xacro).
- Bash - `docker/entrypoint.sh`, `test_servo_config_reader.sh` (root, ad-hoc on-robot test script for the v1 workspace).
- Legacy (ignored by colcon via `legacy/COLCON_IGNORE`): v1 C++/Python/Rust-stub ROS 2 workspace, Flask-style web tools, in `legacy/v1/`. Do not add code there.

## Runtime

**Environment:**

- ROS 2 **Jazzy Jalisco** (default, LTS to 2029) and **Lyrical Luth** (supported, tested in CI). Selected by Docker build arg `ROS_DISTRO` (`docker/Dockerfile`, `docker-compose.yml`).
- Robot: Banana Pi BPI-M4 Zero (Allwinner H618, 4x Cortex-A53, arm64, 2-4 GB RAM) running Armbian (Debian 12); ROS runs inside Docker because ROS binaries target Ubuntu (`docs/PLATFORM.md`).
- Simulation: PC amd64, Gazebo Sim 8 (Harmonic) with Jazzy, Gazebo Sim 10 (Jetty) with Lyrical, via `osrf/ros:<distro>-simulation`.
- Middleware: DDS via ROS 2 defaults; `ROS_DOMAIN_ID=0` on the robot (`docker-compose.yml`), per-check domains and `GZ_PARTITION` values in simulation (`.github/workflows/ci.yml`, `ros2_ws/src/dog_gazebo/dog_gazebo/terrain_sweep.py`).

**Package Manager:**

- ROS: `colcon` + `ament_cmake` (C++) / `ament_python` (Python); dependencies declared in each `ros2_ws/src/*/package.xml` and resolved from the base ROS image (no `rosdep install` step).
- Python tools: `pip` (`tools/autocal/requirements.txt`).
- Lockfile: none (no `requirements.lock`, no `poetry.lock`; versions come from the pinned base Docker image tags).

## Frameworks

**Core:**

- ROS 2 `rclcpp` (Jazzy 28.1.x, Lyrical 32.x) - node framework for C++ packages.
- ROS 2 `rclpy` - Python nodes (`dog_web`, `dog_gazebo`).
- `tf2_ros` - transforms in `ros2_ws/src/dog_perception/`.
- `robot_state_publisher` - consumes generated URDF (`ros2_ws/src/dog_bringup/launch/robot.launch.py`).
- Message packages used: `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `rcl_interfaces`.
- Custom web stack: standard-library `asyncio` HTTP + RFC 6455 WebSocket server in `ros2_ws/src/dog_web/dog_web/wsserver.py` (deliberately no aiohttp/websockets/Flask dependency).

**Testing:**

- GoogleTest via `ament_cmake_gtest` - C++ unit tests in `ros2_ws/src/*/test/test_*.cpp`.
- `launch_testing` (`launch_testing_ament_cmake`, `launch_testing_ros`) - integration tests such as `ros2_ws/src/dog_hardware/test/test_power_monitor.py`, `ros2_ws/src/dog_bringup/test/test_mock_bringup.py`.
- `pytest` - Python tests (`ros2_ws/src/dog_web/test/`, `ros2_ws/src/dog_description/test/`, `tools/autocal/tests/`, `tools/robot_setup/test/`); registered via `extras_require={'test': ['pytest']}` in `setup.py` (needed on Python 3.14).
- Simulation acceptance checks (not unit tests): `walk_check`, `terrain_sweep`, `perception_check`, `localization_check` in `ros2_ws/src/dog_gazebo/dog_gazebo/`.

**Build/Dev:**

- CMake >= 3.16 (`cmake_minimum_required(VERSION 3.16)`), GCC 13.3 (Jazzy image) / 15.2 (Lyrical image).
- Docker + Docker Compose - `docker/Dockerfile` (robot, `ros:<distro>-ros-base`), `docker/Dockerfile.sim` (`osrf/ros:<distro>-simulation`), `docker-compose.yml`.
- Gazebo plugins: `gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system`, `gz-sim-imu-system`, `gz-sim-sensors-system` (`ros2_ws/src/dog_gazebo/worlds/flat.sdf`); `ros_gz_sim` and `ros_gz_bridge` (`parameter_bridge`) in `ros2_ws/src/dog_gazebo/launch/sim.launch.py`.
- RViz config: `ros2_ws/src/dog_bringup/config/dog.rviz`.

## Key Dependencies

**Critical (ROS, from base image):**

- `rclcpp` / `rclpy` - all nodes.
- `ros_gz_sim`, `ros_gz_bridge` - simulation only (`dog_gazebo` is skipped in the robot image: `--packages-skip dog_gazebo`).
- `python3-yaml` - `dog_description` URDF generator reads `robot.yaml`.
- `python3-numpy` - `dog_gazebo`, tests in `dog_perception` and `dog_description`.

**Critical (Linux kernel APIs, no third-party libs):**

- `linux/i2c-dev.h` + `ioctl(I2C_SLAVE)` in `ros2_ws/src/dog_hardware/src/servo_bus.cpp` - PCA9685, MPU6050, INA226/INA219 access.
- Linux joystick API (`/dev/input/js0`) in `ros2_ws/src/dog_teleop/src/gamepad_node.cpp`.

**Python tools (not ROS):**

- `numpy>=1.24`, `opencv-contrib-python>=4.6` (ArUco; `-headless` variant on servers) - `tools/autocal/requirements.txt`.
- `pyyaml` - `tools/robot_setup/robot_setup.py`.
- `matplotlib`, `imageio-ffmpeg`, `opencv-contrib-python`, `numpy` - `tools/sim_video/` (versions not pinned; see `tools/sim_video/README.md`).
- `pytest` - tool tests.

**Infrastructure:**

- Docker BuildKit + QEMU (`docker/setup-qemu-action@v3`, `docker/setup-buildx-action@v3`, `docker/build-push-action@v6`) - arm64 image build in CI.

## Configuration

**Environment:**

- Runtime tunables are ROS parameters in YAML, not env vars: `ros2_ws/src/dog_bringup/config/robot.yaml` (geometry, gait, limits, sensors), `servos.yaml` (PCA9685 channels, pulse ranges, offsets), `imu.yaml`, `power.yaml`, `teleop.yaml`, `teleop_ps.yaml`.
- `docker-compose.yml` bind-mounts `ros2_ws/src/dog_bringup/config` read-only over the installed config so edits apply without rebuilding.
- Launch arguments (`backend:=pca9685|mock`, `gamepad`, `gamepad_profile:=xbox|ps`, `web`, `rviz`, `imu`, `power`) in `ros2_ws/src/dog_bringup/launch/robot.launch.py`; sim arguments (`headless`, `terrain`, `level`, `perception`, `localization`, `dead_reckoning`, `gyro_bias`, `seed`, `map`) in `ros2_ws/src/dog_gazebo/launch/sim.launch.py`.
- Env vars used: `ROS_DOMAIN_ID`, `ROS_DISTRO`, `GZ_PARTITION`. No `.env` files detected; no secrets required.
- ROS namespace is `/dog` (`NS = 'dog'` in `ros2_ws/src/dog_bringup/launch/robot.launch.py`).

**Build:**

- `ros2_ws/src/*/CMakeLists.txt`, `ros2_ws/src/*/setup.py`, `ros2_ws/src/*/setup.cfg`, `ros2_ws/src/*/package.xml`.
- Docker build args: `ROS_DISTRO` (default `jazzy`), `BUILD_JOBS` (default 2 for the 2 GB Pi). Release builds use `-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`.
- No linter/formatter config detected (no `.clang-format`, `.flake8`, `pyproject.toml`, `.prettierrc`).
- Build outputs ignored by git: `ros2_ws/build/`, `install/`, `log/` (`.gitignore`).

## Platform Requirements

**Development:**

- Linux (WSL2 works) with either Docker or a native ROS 2 Jazzy/Lyrical install with `colcon`.
- Simulation needs the `ros-<distro>-ros-gz` stack / `osrf/ros:<distro>-simulation` image (amd64 only).
- Tools only: Python 3.8+ (`tools/robot_setup/robot_setup.py`), Python 3.12 in CI for `tools/autocal`.

**Production:**

- Banana Pi BPI-M4 Zero, Docker, `network_mode: host`, `/dev/i2c-0` passed through, `/dev/input` mounted for the gamepad (`docker-compose.yml`).
- Hardware: PCA9685 (I2C 0x40) driving 12x MG996R servos; MPU6050 IMU (0x68/0x69); optional INA226/INA219 current sensor (0x41/0x44/0x45); optional LD19-class lidars, GS2 line lidar, VL53L1X ToF (perception sensors are simulated only; no hardware drivers detected) - see `docs/HARDWARE.md`, `docs/HEAD.md`.
- Web teleop served on port 8080 (`web_teleop` params in `ros2_ws/src/dog_bringup/config/teleop.yaml`).

## Stray / Untracked Files at Repo Root

- `robot_configurator.py` - standalone v1-era validator for `servo_config.json` (FK, limits, inversions); untracked, not part of the ROS workspace.
- `robot_dog_ws/` - untracked partial v1 workspace (`dog_hardware`, `dog_hardware_cpp`, `dog_web` servo-config reader files); not built by colcon (`ros2_ws` is the only workspace).
- `test_servo_config_reader.sh` - untracked on-robot test script targeting `~/robot_dog_ws` (v1 layout).
- `__pycache__/` - compiled bytecode from a Python 3.13 run of a legacy script.

---

*Stack analysis: 2026-09-29*
