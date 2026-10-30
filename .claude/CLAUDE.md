<!-- GSD:project-start source:PROJECT.md -->

## Project

**RobotDog 2.0**

Четвероногий робот на Banana Pi BPI-M4 Zero (PCA9685, 12 × MG996R, IMU MPU6050) под ROS 2 Jazzy и Lyrical. Управляется геймпадом, клавиатурой и веб-пультом. Полностью написан и проверен в симуляции Gazebo; на настоящем роботе (сейчас есть корпус, сервы, IMU и датчик тока, без лидаров, GS2 и VL53L1X) код ещё ни разу не запускался. Проект владельца репозитория `hzname/robot-dog`, версия 2.0 написана с нуля, v1 лежит в `legacy/v1/`.

**Core Value:** Робот надёжно и безопасно ходит по полу с геймпада, не падая и не убивая свои сервы.

### Constraints

- **Tech stack**: C++17 и ROS 2 Jazzy вместе с Lyrical должны собираться без предупреждений (`-Wall -Wextra -Wpedantic`) — так проходит CI
- **Tech stack**: рантайм робота требует только `ros-base`, без xacro, joy и ros2_control — образ для Pi должен оставаться лёгким
- **Compatibility**: все ноды и топики в namespace `/dog`; параметры только через YAML (`robot.yaml`, `servos.yaml`) — это единый источник правды
- **Hardware**: Banana Pi с 2 ГБ ОЗУ, Docker-сборка идёт с `BUILD_JOBS=2`, одна шина I2C на PCA9685, IMU и датчик питания
- **Dependencies**: восприятие и локализация работают только в симуляции, пока на роботе нет датчиков
- **Safety**: на пол ставить робота только после защиты серв (см. решения ниже)

<!-- GSD:project-end -->

<!-- GSD:stack-start source:codebase/STACK.md -->

## Technology Stack

## Languages

- C++17 - Robot runtime: kinematics, gait, locomotion state machine, hardware drivers (I2C), perception, localization, teleop. Standard set per package with `set(CMAKE_CXX_STANDARD 17)` in `ros2_ws/src/*/CMakeLists.txt`; warnings `-Wall -Wextra -Wpedantic`.
- Python 3.12 (Jazzy / Ubuntu 24.04) and 3.14 (Lyrical / Ubuntu 26.04) - ROS packages for URDF generation, Gazebo glue and checks, the web teleop server, launch files (`ros2_ws/src/dog_description/`, `ros2_ws/src/dog_gazebo/`, `ros2_ws/src/dog_web/`, `ros2_ws/src/dog_bringup/launch/robot.launch.py`), plus the offline tools in `tools/`.
- JavaScript (vanilla, no build step, no framework) - Browser teleop page: `ros2_ws/src/dog_web/static/app.js`, `ros2_ws/src/dog_web/static/index.html`, `ros2_ws/src/dog_web/static/style.css`.
- YAML - ROS parameter files and robot geometry: `ros2_ws/src/dog_bringup/config/*.yaml`.
- SDF / URDF (generated) - Gazebo world `ros2_ws/src/dog_gazebo/worlds/flat.sdf`; URDF generated at launch by `ros2_ws/src/dog_description/dog_description/urdf.py` (no xacro).
- Bash - `docker/entrypoint.sh`, `test_servo_config_reader.sh` (root, ad-hoc on-robot test script for the v1 workspace).
- Legacy (ignored by colcon via `legacy/COLCON_IGNORE`): v1 C++/Python/Rust-stub ROS 2 workspace, Flask-style web tools, in `legacy/v1/`. Do not add code there.

## Runtime

- ROS 2 **Jazzy Jalisco** (default, LTS to 2029) and **Lyrical Luth** (supported, tested in CI). Selected by Docker build arg `ROS_DISTRO` (`docker/Dockerfile`, `docker-compose.yml`).
- Robot: Banana Pi BPI-M4 Zero (Allwinner H618, 4x Cortex-A53, arm64, 2-4 GB RAM) running Armbian (Debian 12); ROS runs inside Docker because ROS binaries target Ubuntu (`docs/PLATFORM.md`).
- Simulation: PC amd64, Gazebo Sim 8 (Harmonic) with Jazzy, Gazebo Sim 10 (Jetty) with Lyrical, via `osrf/ros:<distro>-simulation`.
- Middleware: DDS via ROS 2 defaults; `ROS_DOMAIN_ID=0` on the robot (`docker-compose.yml`), per-check domains and `GZ_PARTITION` values in simulation (`.github/workflows/ci.yml`, `ros2_ws/src/dog_gazebo/dog_gazebo/terrain_sweep.py`).
- ROS: `colcon` + `ament_cmake` (C++) / `ament_python` (Python); dependencies declared in each `ros2_ws/src/*/package.xml` and resolved from the base ROS image (no `rosdep install` step).
- Python tools: `pip` (`tools/autocal/requirements.txt`).
- Lockfile: none (no `requirements.lock`, no `poetry.lock`; versions come from the pinned base Docker image tags).

## Frameworks

- ROS 2 `rclcpp` (Jazzy 28.1.x, Lyrical 32.x) - node framework for C++ packages.
- ROS 2 `rclpy` - Python nodes (`dog_web`, `dog_gazebo`).
- `tf2_ros` - transforms in `ros2_ws/src/dog_perception/`.
- `robot_state_publisher` - consumes generated URDF (`ros2_ws/src/dog_bringup/launch/robot.launch.py`).
- Message packages used: `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `rcl_interfaces`.
- Custom web stack: standard-library `asyncio` HTTP + RFC 6455 WebSocket server in `ros2_ws/src/dog_web/dog_web/wsserver.py` (deliberately no aiohttp/websockets/Flask dependency).
- GoogleTest via `ament_cmake_gtest` - C++ unit tests in `ros2_ws/src/*/test/test_*.cpp`.
- `launch_testing` (`launch_testing_ament_cmake`, `launch_testing_ros`) - integration tests such as `ros2_ws/src/dog_hardware/test/test_power_monitor.py`, `ros2_ws/src/dog_bringup/test/test_mock_bringup.py`.
- `pytest` - Python tests (`ros2_ws/src/dog_web/test/`, `ros2_ws/src/dog_description/test/`, `tools/autocal/tests/`, `tools/robot_setup/test/`); registered via `extras_require={'test': ['pytest']}` in `setup.py` (needed on Python 3.14).
- Simulation acceptance checks (not unit tests): `walk_check`, `terrain_sweep`, `perception_check`, `localization_check` in `ros2_ws/src/dog_gazebo/dog_gazebo/`.
- CMake >= 3.16 (`cmake_minimum_required(VERSION 3.16)`), GCC 13.3 (Jazzy image) / 15.2 (Lyrical image).
- Docker + Docker Compose - `docker/Dockerfile` (robot, `ros:<distro>-ros-base`), `docker/Dockerfile.sim` (`osrf/ros:<distro>-simulation`), `docker-compose.yml`.
- Gazebo plugins: `gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system`, `gz-sim-imu-system`, `gz-sim-sensors-system` (`ros2_ws/src/dog_gazebo/worlds/flat.sdf`); `ros_gz_sim` and `ros_gz_bridge` (`parameter_bridge`) in `ros2_ws/src/dog_gazebo/launch/sim.launch.py`.
- RViz config: `ros2_ws/src/dog_bringup/config/dog.rviz`.

## Key Dependencies

- `rclcpp` / `rclpy` - all nodes.
- `ros_gz_sim`, `ros_gz_bridge` - simulation only (`dog_gazebo` is skipped in the robot image: `--packages-skip dog_gazebo`).
- `python3-yaml` - `dog_description` URDF generator reads `robot.yaml`.
- `python3-numpy` - `dog_gazebo`, tests in `dog_perception` and `dog_description`.
- `linux/i2c-dev.h` + `ioctl(I2C_SLAVE)` in `ros2_ws/src/dog_hardware/src/servo_bus.cpp` - PCA9685, MPU6050, INA226/INA219 access.
- Linux joystick API (`/dev/input/js0`) in `ros2_ws/src/dog_teleop/src/gamepad_node.cpp`.
- `numpy>=1.24`, `opencv-contrib-python>=4.6` (ArUco; `-headless` variant on servers) - `tools/autocal/requirements.txt`.
- `pyyaml` - `tools/robot_setup/robot_setup.py`.
- `matplotlib`, `imageio-ffmpeg`, `opencv-contrib-python`, `numpy` - `tools/sim_video/` (versions not pinned; see `tools/sim_video/README.md`).
- `pytest` - tool tests.
- Docker BuildKit + QEMU (`docker/setup-qemu-action@v3`, `docker/setup-buildx-action@v3`, `docker/build-push-action@v6`) - arm64 image build in CI.

## Configuration

- Runtime tunables are ROS parameters in YAML, not env vars: `ros2_ws/src/dog_bringup/config/robot.yaml` (geometry, gait, limits, sensors), `servos.yaml` (PCA9685 channels, pulse ranges, offsets), `imu.yaml`, `power.yaml`, `teleop.yaml`, `teleop_ps.yaml`.
- `docker-compose.yml` bind-mounts `ros2_ws/src/dog_bringup/config` read-only over the installed config so edits apply without rebuilding.
- Launch arguments (`backend:=pca9685|mock`, `gamepad`, `gamepad_profile:=xbox|ps`, `web`, `rviz`, `imu`, `power`) in `ros2_ws/src/dog_bringup/launch/robot.launch.py`; sim arguments (`headless`, `terrain`, `level`, `perception`, `localization`, `dead_reckoning`, `gyro_bias`, `seed`, `map`) in `ros2_ws/src/dog_gazebo/launch/sim.launch.py`.
- Env vars used: `ROS_DOMAIN_ID`, `ROS_DISTRO`, `GZ_PARTITION`. No `.env` files detected; no secrets required.
- ROS namespace is `/dog` (`NS = 'dog'` in `ros2_ws/src/dog_bringup/launch/robot.launch.py`).
- `ros2_ws/src/*/CMakeLists.txt`, `ros2_ws/src/*/setup.py`, `ros2_ws/src/*/setup.cfg`, `ros2_ws/src/*/package.xml`.
- Docker build args: `ROS_DISTRO` (default `jazzy`), `BUILD_JOBS` (default 2 for the 2 GB Pi). Release builds use `-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`.
- No linter/formatter config detected (no `.clang-format`, `.flake8`, `pyproject.toml`, `.prettierrc`).
- Build outputs ignored by git: `ros2_ws/build/`, `install/`, `log/` (`.gitignore`).

## Platform Requirements

- Linux (WSL2 works) with either Docker or a native ROS 2 Jazzy/Lyrical install with `colcon`.
- Simulation needs the `ros-<distro>-ros-gz` stack / `osrf/ros:<distro>-simulation` image (amd64 only).
- Tools only: Python 3.8+ (`tools/robot_setup/robot_setup.py`), Python 3.12 in CI for `tools/autocal`.
- Banana Pi BPI-M4 Zero, Docker, `network_mode: host`, `/dev/i2c-0` passed through, `/dev/input` mounted for the gamepad (`docker-compose.yml`).
- Hardware: PCA9685 (I2C 0x40) driving 12x MG996R servos; MPU6050 IMU (0x68/0x69); optional INA226/INA219 current sensor (0x41/0x44/0x45); optional LD19-class lidars, GS2 line lidar, VL53L1X ToF (perception sensors are simulated only; no hardware drivers detected) - see `docs/HARDWARE.md`, `docs/HEAD.md`.
- Web teleop served on port 8080 (`web_teleop` params in `ros2_ws/src/dog_bringup/config/teleop.yaml`).

## Stray / Untracked Files at Repo Root

- `robot_configurator.py` - standalone v1-era validator for `servo_config.json` (FK, limits, inversions); untracked, not part of the ROS workspace.
- `robot_dog_ws/` - untracked partial v1 workspace (`dog_hardware`, `dog_hardware_cpp`, `dog_web` servo-config reader files); not built by colcon (`ros2_ws` is the only workspace).
- `test_servo_config_reader.sh` - untracked on-robot test script targeting `~/robot_dog_ws` (v1 layout).
- `__pycache__/` - compiled bytecode from a Python 3.13 run of a legacy script.

<!-- GSD:stack-end -->

<!-- GSD:conventions-start source:CONVENTIONS.md -->

## Conventions

## Naming Patterns

- C++ sources/headers: `snake_case.cpp` / `snake_case.hpp`, headers under `include/<package>/` (`ros2_ws/src/dog_control/include/dog_control/kinematics.hpp`). ROS node entry points end in `_node.cpp` (`locomotion_node.cpp`, `servo_driver_node.cpp`); library code has no suffix (`locomotion.cpp`, `servo_driver.cpp`).
- Python modules: `snake_case.py` (`ros2_ws/src/dog_web/dog_web/protocol.py`, `wsserver.py`). Simulation checkers end in `_check.py` (`walk_check.py`, `perception_check.py`, `localization_check.py`).
- Tests: `test_<unit>.cpp` / `test_<unit>.py`, named after the unit (`test_kinematics.cpp`, `test_protocol.py`). Launch/integration tests are named after the scenario (`test_mock_bringup.py`, `test_gamepad_fifo.py`, `test_power_monitor.py`).
- ROS packages: `dog_<domain>` (`dog_control`, `dog_hardware`, `dog_perception`, `dog_teleop`, `dog_web`, `dog_gazebo`, `dog_description`, `dog_bringup`).
- C++: `camelCase` for free functions and methods (`forwardKinematics`, `inverseKinematics`, `jointToPulseUs`, `setTargets`, `legSide`). Trivial accessors are named for the value, no `get` prefix (`mode()`, `joints()`, `estopActive()`, `enabled(i)`); setters use `set` (`setVelocity`, `setEstop`).
- Python: `snake_case` (`handle_message`, `fit_joint`, `find_limit`). Module-private helpers get a leading underscore (`_num`, `_rot`, `_fk`, `_wrap_deg`).
- C++ locals and struct fields: `snake_case` (`stand_height`, `cmd_timeout`). Private class data members: `snake_case_` with trailing underscore (`controller_`, `cmd_vel_active_`, `last_guard_vx_` in `ros2_ws/src/dog_control/src/locomotion_node.cpp`). Plain aggregate structs (`LocomotionParams`, `ServoCalibration`) use fields without underscore and in-class default initializers `double hip{0.055};`.
- Put units in the name or a trailing bracketed comment: `offset_deg`, `pulse_min_us`, `servo_arm_mm`, `max_joint_speed{6.0};  // [rad/s] slew limit`. Internal angles are radians; degrees appear only at config/calibration boundaries and are named `_deg`.
- Compile-time constants: `kCamelCase` (`kNumLegs`, `kEps`, `kDt`, `kFloor`, `kJointNames`). Python module constants: `UPPER_SNAKE` (`COMMANDS`, `GUARD_STATES`, `CAL_FIELDS`, `MIN_BODY_HEIGHT`).
- C++ classes/structs/enums: `PascalCase` (`LocomotionController`, `ServoCalibration`, `IkResult`); `enum class` values are `UPPER_SNAKE` (`Mode::STANDING_UP`, `GaitType::CRAWL`). Legacy unscoped enum for leg indices: `LF, RF, LR, RR` (`kinematics.hpp`).
- Python classes: `PascalCase`; prefer `@dataclass` for value objects (`Limits`, `Actions` in `protocol.py`; `FitResult` in `tools/autocal/robotdog_autocal/fit.py`).
- Namespaces: `dog_<domain>` matching the package (`namespace dog_control { ... }  // namespace dog_control`).
- Topics/parameters are relative and live under the `dog` namespace (`NS = 'dog'` in `ros2_ws/src/dog_bringup/launch/robot.launch.py`). Parameters are dotted `group.name` (`geometry.thigh`, `stance.stand_height`, `odom.publish`, `<joint>.offset_deg`).
- Joint names: `<leg>_<joint>_joint` with legs `lf, rf, lr, rr` and joints `hip, thigh, calf`.

## Code Style

- 2-space indent, no tabs, `#pragma once` in every header (no include guards).
- Function, class, struct and namespace braces on their own line; control-flow braces on the same line: `if (x) {`, `} else {`.
- Single-statement bodies use compact braces on one line: `if (h2 < 0) {return;}`, `double x() const {return x_;}`.
- Reference/pointer spacing: `const Vec3 & foot`, `const std::string & cmd` (space on both sides of `&`), but `char ** argv`.
- Constructor initializer lists start with a leading colon on the next line: `: rclcpp::Node("locomotion", options)`.
- Line length: soft ~100, hard ~120 (only 8 lines exceed 120). Continuation lines indent by 2 (function args) or align under the opening arg.
- `///` for API doc comments in headers, `//` for everything else.
- 4-space indent, single quotes for strings (`'stand'`), f-strings for formatting (`f'unknown message type {kind!r}'`); older `%`-formatting is acceptable in logging calls (`self.get_logger().info('web: %s' % actions.command)`).
- Lines run to ~120 characters (over 300 lines exceed that; do not add many more). Two blank lines between top-level definitions; section dividers use `# ---------- name` comments (`ros2_ws/src/dog_perception/dog_perception/core.py`, `# ---- ROS side` in `web_teleop.py`).
- Type hints are used sparingly, on public boundaries only (`def handle_message(text: str, limits: Limits) -> Actions:`); not required everywhere.
- Lambda assigned to a name is flagged with `# noqa: E731` (`robot.launch.py`); late imports after `sys.path` tweaks use `# noqa: E402` (`tools/robot_setup/test/test_robot_setup.py`). This shows flake8 rules are followed by hand.
- None automated. `-Wall -Wextra -Wpedantic` must stay clean for C++. Fix warnings in code (explicit `static_cast<int>(...)`, unused parameters removed) rather than disabling them.

## Import / Include Organization

## Error Handling

- Validation returns a value instead of throwing: `std::string validate() const` returns an empty string when OK and an error message otherwise (`ServoCalibration::validate` in `ros2_ws/src/dog_hardware/include/dog_hardware/servo_driver.hpp`); bool-returning mutators for rejected requests (`bool request(const std::string & cmd)`, `bool setCalibration(...)`).
- Out-of-range math is clamped and flagged, never NaN: `IkResult{q, reachable}` in `kinematics.hpp`; `jointToPulseUs(..., bool * clamped)`.
- Constructors of parsers throw `std::invalid_argument` with the offending spec in the message (`ros2_ws/src/dog_hardware/src/imu_sensor.cpp`, `ros2_ws/src/dog_perception/src/core.cpp`).
- Fail fast at startup with `throw std::runtime_error("<what> '<value>' (use a or b)")` for bad config (unknown backend, invalid calibration). `main()` wraps `rclcpp::spin` in `try/catch (const std::exception &)`, logs with `RCLCPP_FATAL`, and returns exit code 1 (`ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`, `power_monitor_node.cpp`).
- Runtime parameter callbacks return `rcl_interfaces::msg::SetParametersResult` with `successful=false` and a `reason` instead of throwing.
- Bad/stale input at runtime is dropped and logged, not fatal: undersized arrays are ignored (`if (msg->data.size() < 2) {return;}`), stale command/guard/terrain inputs are cleared after a timeout (`cmd_vel_timeout`, `guard_timeout` in `locomotion_node.cpp`). Every safety input has a watchdog: new inputs must too.
- Numbers from YAML may be int or double: read parameters through a tolerant helper (`asNumber`, `declareNumber` in `servo_driver_node.cpp`).
- Message handlers never raise to the caller: `protocol.handle_message` catches `(ValueError, json.JSONDecodeError)` and returns `Actions(errors=[...])`; the server answers `{"type": "error", ...}` (`ros2_ws/src/dog_web/dog_web/protocol.py`, `web_teleop.py`).
- Input validation raises `ValueError` with a message naming the field: `raise ValueError(f'"{key}" must be a finite number')`. Reject `bool` where a number is expected (`isinstance(v, bool)` check in `_num`).
- Async timeouts raise `TimeoutError('no answer from the robot')` (`dog_web/calibration.py`).
- CLI checkers (`walk_check.py`, `terrain_sweep.py`, `perception_check.py`) report `PASS`/`FAIL` lines with `print(..., flush=True)` and end with `sys.exit(0 if ok else 1)`; CI relies on the exit code.

## Logging

- C++: `RCLCPP_INFO/WARN/FATAL(get_logger(), "fmt %s", x.c_str())`; use `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, ...)` for anything in the control loop.
- Python nodes: `self.get_logger().info/warning/error(...)`.
- Log state transitions and rejected commands (`"command '%s' rejected (mode %s)"`), not every tick. Safety events are `WARN` (`"E-STOP ENGAGED"`, `"guard: hazard ahead - forward motion stopped"`).

## Comments

- Comments explain WHY and record measured/derived numbers, units and frames, not what the line does: `// Tuned in Gazebo with servo speed capped at 6 rad/s (see docs/SIMULATION.md)` in `ros2_ws/src/dog_bringup/config/robot.yaml`.
- Every header/module starts with a block comment describing the frame conventions, state machine or topic interface (`kinematics.hpp` joint sign conventions; `locomotion_node.cpp` subscribes/publishes list; `protocol.py` message grammar). Keep node topic lists in that header comment up to date when adding a topic.
- Reference docs by path: `(see docs/CALIBRATION.md)`, `TERRAIN.md`, `DEPLOYMENT.md`.
- Language: code comments, identifiers, log messages and docstrings are English. Long-form docs under `docs/` and the operator-facing UI text of `tools/robot_setup/robot_setup.py` (form labels, validation messages) are Russian; assert on Russian substrings in `tools/robot_setup/test/test_robot_setup.py` accordingly.
- No TODO/FIXME/HACK markers exist in `ros2_ws/` or `tools/`; open issues are tracked in `docs/REVIEW.md` instead. Follow that: write the issue there, not as a TODO.
- Python modules and public classes/functions have triple-double-quote docstrings, first line a summary (`"""Fitting servo calibration from (pulse, measured joint angle) samples."""`). Test modules have a one-line docstring stating what is verified end to end.
- C++ `///` on public API in headers, with units and failure behaviour (`/// Validates the fields; returns an empty string when OK.`).

## Function Design

## Module Design

- Pattern per C++ package: a `<package>_core` static/shared library (no rclcpp) + thin `*_node.cpp` executables + gtests linking the core. Add new logic to the core, keep nodes to wiring, parameters and timers (`ros2_ws/src/dog_control/CMakeLists.txt`).
- Node parameters are declared with their default taken from the params struct: `p.leg.thigh = declare_parameter("geometry.thigh", p.leg.thigh);`. Defaults in code and in `ros2_ws/src/dog_bringup/config/robot.yaml` must agree; `robot.yaml` is the single source of truth for geometry (also read by `dog_description.urdf.load_config` and `dog_perception.core.robot_config`).
- Python packages: `__init__.py` is empty; import modules explicitly (`from dog_web import protocol`).

## Configuration Files

- ROS parameter YAML uses the wildcard node key and `ros__parameters` (`/**:` in `robot.yaml`, `/**/servo_driver:` in `servos.yaml`), inline-flow maps for per-joint rows aligned in columns, and `# [unit]` trailing comments. Explain non-obvious values in a comment above the key.
- `tools/robot_setup/robot_setup.py` rewrites values in place and keeps comments/layout: keep YAML edits regex-friendly (`key: value  # comment`, one key per line).
- Launch files: `ros2_ws/src/dog_bringup/launch/robot.launch.py` builds actions in an `OpaqueFunction` `_setup(context)`, using small local helpers `cfg(name)` / `on(name)` for launch arguments; each ROS node is given `namespace=NS`.
- `package.xml` is format 3; maintainer `hzname`, license MIT, version `2.0.0` across packages.

<!-- GSD:conventions-end -->

<!-- GSD:architecture-start source:ARCHITECTURE.md -->

## Architecture

## System Overview

```text

```

## Component Responsibilities

| Component | Responsibility | File |
|-----------|----------------|------|
| `LocomotionController` | Mode state machine (PASSIVE/STANDING_UP/STAND/WALK/LYING_DOWN/LYING/GREETING/SURVEY), velocity/pose limiting, slope compensation, heading hold, guard handling; ROS-free | `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp`, `ros2_ws/src/dog_control/src/locomotion.cpp` |
| `locomotion_node` | ROS shell around the controller: subscriptions, 50 Hz timer, cmd_vel timeout, dead-reckoned odom, parameter loading | `ros2_ws/src/dog_control/src/locomotion_node.cpp` |
| `TrotGait` | Diagonal-pair trot with per-leg step height | `ros2_ws/src/dog_control/src/gait.cpp` |
| `CrawlGait` | Three-support crawl following terrain profile | `ros2_ws/src/dog_control/src/crawl.cpp` |
| `GreetSequence`, `SurveySequence` | Scripted motions (sit+wave, body sweep for lidar mapping) | `ros2_ws/src/dog_control/src/greet.cpp`, `ros2_ws/src/dog_control/src/survey.cpp` |
| Kinematics | Forward/inverse kinematics, REP-103 frames | `ros2_ws/src/dog_control/src/kinematics.cpp` |
| `ServoDriver` | Calibration (angle→pulse), joint speed limiting, staggered leg enable, E-STOP; ROS-free | `ros2_ws/src/dog_hardware/src/servo_driver.cpp` |
| `ServoBus` (`Pca9685Bus`, `MockBus`) | Hardware abstraction for PWM output | `ros2_ws/src/dog_hardware/src/servo_bus.cpp` |
| `servo_driver_node` | Declares per-joint calibration params, publishes `joint_states`/`servo_pulses`, live parameter tuning | `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp` |
| `imu_node`, `power_monitor_node` | Auto-probe I2C chip, exit if absent; IMU attitude filter; current/voltage guard | `ros2_ws/src/dog_hardware/src/imu_node.cpp`, `ros2_ws/src/dog_hardware/src/power_monitor_node.cpp` |
| `JoyMapper`, `KeyboardMapper` | Button/key → twist/command logic; ROS-free | `ros2_ws/src/dog_teleop/src/mapping.cpp` |
| `TeleopPublisher` | Shared publisher set (`cmd_vel`, `command`, `estop`, `body_pose`) for all teleop nodes | `ros2_ws/src/dog_teleop/include/dog_teleop/teleop_publisher.hpp` |
| `web_teleop` | HTTP+WebSocket teleop, calibration channel | `ros2_ws/src/dog_web/dog_web/web_teleop.py` |
| `protocol`, `wsserver`, `calibration` | ROS-free message validation; RFC 6455 server; calibration bridge to `servo_driver` parameters | `ros2_ws/src/dog_web/dog_web/protocol.py`, `wsserver.py`, `calibration.py` |
| `perception_node` | Floor plane, elevation map, hazards, `HazardGuard`, `Avoider`; publishes `guard` and `terrain/profile` | `ros2_ws/src/dog_perception/src/perception_node.cpp`, `core.cpp` |
| `localization_node` | Wall-grid map from lidars, scan matching, submaps, loop closure, pose | `ros2_ws/src/dog_perception/src/localization_node.cpp`, `localization.cpp`, `submaps.cpp` |
| `dog_perception.core` | numpy port of the C++ perception algorithm for checks/videos | `ros2_ws/src/dog_perception/dog_perception/core.py` |
| URDF generator | `robot.yaml` → URDF (plain/Gazebo variant), joint and sim topic names | `ros2_ws/src/dog_description/dog_description/urdf.py` |
| Bringup launch | Robot-side composition of nodes by launch arguments | `ros2_ws/src/dog_bringup/launch/robot.launch.py` |
| Sim launch | Gazebo + bridges + same control nodes | `ros2_ws/src/dog_gazebo/launch/sim.launch.py` |
| Terrain/world generator | SDF worlds by kind (slope, waves, rough, steps, bar, block, wall, room, house) | `ros2_ws/src/dog_gazebo/dog_gazebo/terrain.py` |
| Sim checks | Automated physical verification | `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py`, `terrain_sweep.py`, `perception_check.py`, `localization_check.py` |
| `robot_setup` | Web/CLI form editing `robot.yaml` / `servos.yaml` in place | `tools/robot_setup/robot_setup.py` |
| `robotdog_autocal` | Laptop-side camera auto-calibration over the web WebSocket (no ROS) | `tools/autocal/robotdog_autocal/` |
| Servo JSON config layer (uncommitted, v1-style) | `ServoConfigReader` (C++/Python) reading `servo_config.json`; `robot_configurator.py` computes derived values | `robot_dog_ws/src/dog_hardware_cpp/src/servo_config_reader.cpp`, `robot_dog_ws/src/dog_hardware/dog_hardware/servo_config_reader.py`, `robot_configurator.py` |

## Pattern Overview

- Each C++ package splits into a ROS-independent core library (`dog_control_core`, `dog_hardware_core`, `dog_teleop_mapping`, `dog_perception_core`) and thin `rclcpp` node executables; unit tests link only the core.
- Robot and simulation run the same `locomotion_node`; only the actuation side differs (`servo_driver_node` vs. `joint_command_bridge` + Gazebo). Hardware differences hide behind `ServoBus` (`pca9685` | `mock`) selected by the `backend` parameter.
- Configuration by YAML parameters: `robot.yaml` feeds both `locomotion_node` and the URDF generator so geometry cannot diverge; `servos.yaml` feeds `servo_driver`.
- Optional subsystems attach by topic only (IMU, power, perception, localization); missing sensors degrade gracefully (nodes exit or controller behaves as on flat ground).
- Safety is layered in the chain: deadman/cmd_vel timeout (0.4–0.5 s) in teleop and `locomotion_node`, E-STOP topic honored by both `locomotion_node` and `servo_driver_node`, joint-speed limiting and staggered leg enable in `ServoDriver`.

## Layers

- Purpose: Convert operator input into `cmd_vel`, `command`, `body_pose`, `estop`.
- Location: `ros2_ws/src/dog_teleop/`, `ros2_ws/src/dog_web/`
- Contains: gamepad reader (Linux joystick API), joy mapper, keyboard raw-terminal node, web server + static page (`ros2_ws/src/dog_web/static/`).
- Depends on: ROS message types only.
- Used by: `locomotion_node`.
- Purpose: Mode logic, gait generation, IK, joint target output at 50 Hz.
- Location: `ros2_ws/src/dog_control/`
- Depends on: `rclcpp`, std/sensor/nav/geometry messages; core has no ROS dependency.
- Used by: actuation layer via `joint_commands`; perception via `state`/`odom`.
- Purpose: Turn joint angles into servo pulses (robot) or Gazebo joint commands (sim).
- Location: `ros2_ws/src/dog_hardware/`, `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py`
- Depends on: I2C `/dev/i2c-*` on hardware; `ros_gz_bridge` in sim.
- Purpose: Terrain hazard guard, terrain profile, wall map and localization, IMU, power.
- Location: `ros2_ws/src/dog_perception/`, `ros2_ws/src/dog_hardware/src/imu_node.cpp`, `power_monitor_node.cpp`
- Feeds back into control via `guard` (Float64MultiArray) and `terrain/profile` (Float32MultiArray).
- Purpose: Robot model and parameters.
- Location: `ros2_ws/src/dog_description/`, `ros2_ws/src/dog_bringup/config/`
- Purpose: Gazebo worlds, bridges, automated checks.
- Location: `ros2_ws/src/dog_gazebo/`, CI in `.github/workflows/ci.yml`
- Location: `tools/robot_setup/`, `tools/autocal/`, `tools/sim_video/` (renders videos and `report/` HTML from recorded runs).

## Data Flow

### Primary Request Path (operator → servo)

### Perception guard loop

### Localization flow

### Calibration flow

- Controller state lives in `LocomotionController` members; mode published as latched String on `state`. E-STOP is deliberately a volatile (non-latched) reliable topic; nodes start safe (PASSIVE / servos off).

## Key Abstractions

- Purpose: Testable robot motion brain independent of ROS.
- Examples: `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp`, `ros2_ws/src/dog_control/test/test_locomotion.cpp`
- Pattern: Pull-style `update(dt)` returning whether joints should be sent; setters for external inputs (`setVelocity`, `setImuAttitude`, `addYawRate`, `setGuard`, `setTerrain`).
- Purpose: Swap real PCA9685 for a mock.
- Examples: `ros2_ws/src/dog_hardware/include/dog_hardware/servo_bus.hpp`
- Pattern: Abstract base class + `Pca9685Bus` (Linux i2c-dev) + `MockBus`.
- `ImuSensor`, `PowerSensor` (`imu_sensor.hpp`, `power_sensor.hpp`): backend `auto|mock|off`; node shuts down if chip not on bus.
- `build_urdf(geometry, description, gazebo, namespace, initial)` in `ros2_ws/src/dog_description/dog_description/urdf.py`; also used by `sim.launch.py` and `robot.launch.py`.
- Arguments like `perception`, `localization`, `gamepad`, `web` toggle nodes in `_setup(context)` in both launch files.

## Entry Points

- Location: `ros2_ws/src/dog_bringup/launch/robot.launch.py`
- Triggers: `ros2 launch dog_bringup robot.launch.py [backend:=mock rviz:=true perception:=true localization:=true]`; Docker CMD in `docker/Dockerfile`.
- Responsibilities: robot_state_publisher, locomotion, servo_driver, optional power/imu/perception/localization/gamepad/web/rviz.
- Location: `ros2_ws/src/dog_gazebo/launch/sim.launch.py`
- Triggers: `ros2 launch dog_gazebo sim.launch.py [terrain:=slope level:=10 headless:=true ...]`.

## Architectural Constraints

- **Threading:** Nodes use rclcpp single-threaded executors with wall/steady timers (locomotion 50 Hz, servo_driver timer, gamepad 5 ms poll). `web_teleop` runs an asyncio loop in a separate thread beside a `SingleThreadedExecutor`, crossing via `call_soon_threadsafe` (`web_teleop.py`).
- **Global state:** No module-level singletons in v2 C++/Python nodes. In the uncommitted `robot_dog_ws/`, `ServoConfigReader` is a process-wide singleton (`robot_dog_ws/src/dog_hardware/dog_hardware/servo_config_reader.py`).
- **Circular imports:** None detected; core libraries are leaf dependencies of their nodes.
- **Namespace:** All runtime nodes and topics are relative names under `/dog`; launch files set `namespace=NS` (`'dog'`). Add new nodes with the same namespace.
- **Dependencies:** Runtime only needs `ros-base`; no xacro/joy/ros2_control (`docker/Dockerfile` comment). Keep it that way. `dog_gazebo` is skipped in robot builds and CI unit build (`--packages-skip dog_gazebo`).
- **Build targets:** ROS 2 Jazzy and Lyrical must both build without warnings (`-Wall -Wextra -Wpedantic`, C++17).
- **QoS:** `state` and `localization/*` status/map are transient_local; `estop` is reliable volatile; sensors use `SensorDataQoS`.

## Anti-Patterns

### Hard-coded absolute config path

### Parallel workspace outside `ros2_ws`

### Duplicated perception algorithm

## Error Handling

- Stale input timeouts: `cmd_vel_timeout` (0.5 s), `guard_timeout` (1.0 s), web `drive_timeout` (0.4 s) zero the command.
- Commands return accept/reject (`LocomotionController::request` → bool; node logs warning with mode).
- `PowerGuard` events (OVERCURRENT, UNDERVOLTAGE) drive `estop` (`power_sensor.hpp`).
- Unreachable IK targets are clamped and counted (`unreachableCount()`).

## Cross-Cutting Concerns

<!-- GSD:architecture-end -->

<!-- GSD:skills-start source:skills/ -->

## Project Skills

No project skills found. Add skills to any of: `.claude/skills/`, `.agents/skills/`, `.cursor/skills/`, `.github/skills/`, or `.codex/skills/` with a `SKILL.md` index file.
<!-- GSD:skills-end -->

<!-- GSD:workflow-start source:GSD defaults -->

## GSD Workflow Enforcement

Before using Edit, Write, or other file-changing tools, start work through a GSD command so planning artifacts and execution context stay in sync.

Use these entry points:

- `/gsd-quick` for small fixes, doc updates, and ad-hoc tasks
- `/gsd-debug` for investigation and bug fixing
- `/gsd-execute-phase` for planned phase work

Do not make direct repo edits outside a GSD workflow unless the user explicitly asks to bypass it.
<!-- GSD:workflow-end -->

<!-- GSD:profile-start -->

## Developer Profile

> Profile not yet configured. Run `/gsd-profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->
