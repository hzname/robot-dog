---
last_mapped_commit: 104fd262ee2afbd8c9ca25310c9647ab91815a75
last_mapped_at: 2026-09-29
---
# External Integrations

**Analysis Date:** 2026-09-29

## APIs & External Services

**None.** This codebase is a self-contained robotics system with no external API integrations.

- No HTTP/REST client imports detected across the workspace
- All inter-process communication uses ROS 2 middleware (local DDS over network or localhost)
- Web interface is internal only (serves static HTML/CSS/JS + WebSocket server on `:8080`)

## Data Storage

**Databases:**

- **Not applicable.** No traditional database system is used.

**Configuration Storage:**

- **YAML files** — Robot geometry, servo calibration, teleop mappings
  - Location: `ros2_ws/src/dog_bringup/config/`
  - Read by: ROS 2 parameter server at launch time
  - Parser: `pyyaml` (Python) in `tools/robot_setup/robot_setup.py`

- **JSON** (runtime output only)
  - Terrain sweep results saved to `.json` in CI tests (e.g., `slope.json`, `waves.json`)
  - Not persisted to robot; used for validation and reporting

**File Storage:**

- **Local filesystem only** on the robot
  - No cloud storage, S3, or external file services
  - Logs written to stdout/stderr, captured by Docker

**Caching:**

- **None.** All data is either transient (ROS message bus) or configuration-driven.

## Authentication & Identity

**Auth Provider:**

- **None.** This is a local hardware system.

**Implementation:**

- No user authentication required
- No API keys or tokens used in codebase (except CI secrets in `.github/workflows/ci.yml`, not visible)
- Access control via Linux file permissions on `/dev/i2c-0` and `/dev/input/*` device nodes
- Web interface accessible to any client on the robot's network (no password, designed for LAN only)

## Hardware Integrations

**I2C Devices (Hardware Communication):**

- **PCA9685 PWM Servo Driver** — I2C address 0x40
  - Interface: `/dev/i2c-0` via Linux I2C ioctl (fcntl, I2C_SLAVE)
  - Controlled by: `ros2_ws/src/dog_hardware/src/servo_driver.cpp`
  - Mounted in Docker: `- /dev/i2c-0:/dev/i2c-0`
  - Protocol: I2C (400 kHz standard)

- **MPU6050 IMU (Accelerometer + Gyroscope)** — I2C addresses 0x68, 0x69 (optional dual)
  - Interface: `/dev/i2c-0` via Linux I2C ioctl
  - Controlled by: `ros2_ws/src/dog_hardware/src/imu_sensor.cpp`
  - Reads: 3-axis acceleration, 3-axis angular velocity
  - Output: `ros2_ws/src/dog_hardware/src/imu_node.cpp` publishes `sensor_msgs/Imu`

- **Power Sensor (INA series)** — I2C addresses 0x41, 0x44, 0x45 (optional, probed at startup)
  - Interface: `/dev/i2c-0` via Linux I2C ioctl
  - Controlled by: `ros2_ws/src/dog_hardware/src/power_sensor.cpp`
  - Reads: Input voltage, current draw
  - Monitored by: `ros2_ws/src/dog_hardware/src/power_monitor_node.cpp`

**Input Devices (Linux User Input):**

- **Gamepad (Joystick API)** — USB or Bluetooth
  - Interface: `/dev/input/js*` (Linux joystick device nodes)
  - Mounted in Docker: `- /dev/input:/dev/input`
  - Controlled by: `ros2_ws/src/dog_teleop/src/gamepad_node.cpp`
  - Supported: Xbox and PlayStation controllers (Linux joystick standard)
  - Dynamic plugging: `device_cgroup_rules: ['c 13:* rmw']` allows input devices plugged after container start

- **Keyboard** — Terminal or web browser
  - Terminal: Direct TTY input to `keyboard_teleop` process
  - Web: JavaScript captures keyboard events in browser, sends via WebSocket

**Simulation (Optional):**

- **Gazebo** — Physics and sensor simulation
  - Interface: Native ROS 2 integration via `ros_gz_sim` and `ros_gz_bridge`
  - Container image: `osrf/ros:jazzy-simulation` (includes Gazebo Harmonic/Jetty)
  - Simulated sensors: Joint encoders (kinematics), accelerometer/gyro (IMU), lidars, ToF

## Monitoring & Observability

**Error Tracking:**

- Not applicable. No centralized error tracking service.

**Logs:**

- **stdout/stderr** — All node logs printed to console
  - Captured by Docker: `docker compose logs -f`
  - CI captures on failure: `robot test-result --verbose` and uploads `log/` artifact
  - Example: `ros2_ws/log/` directory in CI uploads

**Node Introspection:**

- **ROS 2 CLI tools** — For debugging
  - `ros2 node list` — List active nodes
  - `ros2 topic list` — List published topics
  - `ros2 param list` — List parameters
  - Example commands in Docker: `docker compose exec robot ros2 topic echo /dog/joint_states`

## CI/CD & Deployment

**Hosting:**

- **Self-hosted hardware**: Banana Pi BPI-M4 Zero (arm64)
- **Local docker-compose** orchestration (no cloud platform)

**CI Pipeline:**

- **GitHub Actions** (`.github/workflows/ci.yml`)
  - Triggers: push to main/master, pull requests, manual workflow_dispatch
  - Runners: ubuntu-24.04
  - Matrix test: Jazzy and Lyrical ROS 2 distros

- **Build jobs:**
  - `build-test`: Compile all packages, run 109 unit/integration tests
    - Skip dog_gazebo (simulation only)
    - Command: `colcon build --packages-skip dog_gazebo && colcon test --packages-skip dog_gazebo`
  
  - `simulation`: Gazebo walk validation on different distros
    - Walk check: `ros2 run dog_gazebo walk_check --backward-ratio 0.2`
    - Terrain sweep: `ros2 run dog_gazebo terrain_sweep --terrain slope --levels 10`
  
  - `autocal`: Camera calibration tool tests
    - Python 3.12 + numpy + opencv-contrib-python-headless
    - Command: `python -m robotdog_autocal demo`
  
  - `robot-setup`: Parameter validation tool
    - Checks: `python tools/robot_setup/robot_setup.py --check`

- **Artifacts on failure:**
  - `test-logs-{distro}`: ROS 2 build/test logs
  - `sim-log-{distro}`: Gazebo simulation output

**Deployment (Robot):**

- **Docker image build**: `docker build -f docker/Dockerfile -t robot-dog:2.0 .`
  - Multi-platform: arm64 (Banana Pi), amd64 (PC)
  - Build args: `ROS_DISTRO`, `BUILD_JOBS` (limited to 2 for Pi)

- **Docker container runtime**: `docker compose up -d --build`
  - Orchestrated by: `docker-compose.yml`
  - Restart policy: `unless-stopped`
  - Entrypoint: `/entrypoint.sh` → `ros2 launch dog_bringup robot.launch.py`
  - Network: `host` mode (enables ROS 2 DDS discovery)

## Environment Configuration

**Required Environment Variables:**

- **ROS_DOMAIN_ID** — DDS domain for ROS 2
  - Default: `0` (production)
  - CI test: `43` (isolated)
  - Set in: `docker-compose.yml` (`environment:`) or shell before `colcon build`

- **ROS_DISTRO** — ROS 2 version
  - Values: `jazzy`, `lyrical`
  - Set in: Docker Dockerfile `ARG ROS_DISTRO=jazzy`, or shell `export ROS_DISTRO=jazzy`

- **PATH** — Must include `/opt/ros/${ROS_DISTRO}/bin`
  - Automatically set by: `. /opt/ros/${ROS_DISTRO}/setup.bash`

**Optional Launch Parameters:**

- `backend:=pca9685` — Hardware backend (pca9685 for real, mock for simulation, auto to probe)
- `gamepad:=true` — Enable gamepad teleop node
- `web:=true` — Enable web server on `:8080`
- `rviz:=true` — Launch RViz visualization (dev only)

**Build Environment:**

- **MAKEFLAGS** — Parallel compilation jobs
  - Docker Dockerfile: `ARG BUILD_JOBS=2` → `MAKEFLAGS="-j${BUILD_JOBS}"`
  - Limited on Pi due to 2 GB RAM

- **CMAKE_BUILD_TYPE** — Optimization level
  - Production: `Release`
  - Dockerfile: `--cmake-args -DCMAKE_BUILD_TYPE=Release`
  - CI: Default (Debug build, slower but better for testing)

**Secrets/Configuration Location:**

- No .env files in repo
- Configuration stored in YAML: `ros2_ws/src/dog_bringup/config/`
- Robot parameters (servo calibration, geometry) validated by: `tools/robot_setup/robot_setup.py`
- Servo calibration workflow documented in: `docs/CALIBRATION.md`

## Webhooks & Callbacks

**Incoming:**

- Not applicable. No incoming webhooks.

**Outgoing:**

- Not applicable. No outgoing webhooks.

**Internal Message Bus (ROS 2 Topics):**

- Multi-node communication via ROS 2 pub/sub (not HTTP webhooks)
- Topics include:
  - `/dog/cmd_vel` — Velocity commands (Twist)
  - `/dog/joint_states` — Servo positions/velocities (JointState)
  - `/dog/imu` — Accelerometer/gyro data (Imu)
  - `/dog/perception/*` — Lidar scans, obstacle detection, elevation maps
  - Full list visible via: `ros2 topic list` on running robot

---

*Integration audit: 2026-09-29*
