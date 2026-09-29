---
last_mapped_commit: d269bd16b0d0f69f3ab623b9cecc4ba545b70517
last_mapped_at: 2026-09-29
---
<!-- refreshed: 2026-09-29 -->

# Architecture

**Analysis Date:** 2026-09-29

## System Overview

```text
┌──────────────────────────────────────────────────────────────────────────┐
│                       Operator input (teleop layer)                       │
├────────────────────┬─────────────────────┬───────────────────────────────┤
│ gamepad_node ─joy─▶│ keyboard_teleop     │ web_teleop (HTTP+WS :8080)    │
│ joy_teleop_node    │ (TTY / SSH)         │ Python, stdlib-only server    │
│ `dog_teleop/src/`  │ `dog_teleop/src/`   │ `dog_web/dog_web/`            │
└─────────┬──────────┴──────────┬──────────┴───────────────┬───────────────┘
          │ cmd_vel · command · body_pose · estop (namespace /dog)
          ▼                                                 ▲ state, power, perception/guard
┌──────────────────────────────────────────────────────────────────────────┐
│  locomotion_node  (50 Hz)  `dog_control/src/locomotion_node.cpp`          │
│  LocomotionController state machine + gaits + IK                          │
│  `dog_control/src/locomotion.cpp`, `gait.cpp`, `crawl.cpp`, `kinematics.cpp`│
└───────────┬──────────────────────────────────────────────┬───────────────┘
   joint_commands (12 angles)                       state (latched), odom
            │                                              │
   ┌────────┴───────────── robot ────────┐     ┌──────────▼──────────────────┐
   │ servo_driver_node                   │     │ perception_node (guard,     │
   │ `dog_hardware/src/servo_driver*.cpp`│     │  terrain/profile) and        │
   │ ServoBus: Pca9685Bus | MockBus      │     │ localization_node (map/pose) │
   └────────┬────────────────────────────┘     │ `dog_perception/src/`        │
            │ I2C /dev/i2c-0 (PCA9685)         └──────────┬──────────────────┘
            ▼                                    guard, terrain/profile ─▶ locomotion
   12 × MG996R servos            ── or, in simulation ──
                                 joint_command_bridge ─▶ Gazebo joint controllers
                                 `dog_gazebo/dog_gazebo/joint_command_bridge.py`
   Optional sensors: imu_node (MPU6050) → imu/data; power_monitor_node (INA226/219) → power, estop
   `dog_hardware/src/imu_node.cpp`, `dog_hardware/src/power_monitor_node.cpp`

   Single source of truth for geometry/gait/stance: `ros2_ws/src/dog_bringup/config/robot.yaml`
   Servo calibration: `ros2_ws/src/dog_bringup/config/servos.yaml`
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

**Overall:** Distributed ROS 2 node graph (all under namespace `/dog`) with a ROS-free core library per package, wired together by launch files and one YAML parameter source of truth.

**Key Characteristics:**

- Each C++ package splits into a ROS-independent core library (`dog_control_core`, `dog_hardware_core`, `dog_teleop_mapping`, `dog_perception_core`) and thin `rclcpp` node executables; unit tests link only the core.
- Robot and simulation run the same `locomotion_node`; only the actuation side differs (`servo_driver_node` vs. `joint_command_bridge` + Gazebo). Hardware differences hide behind `ServoBus` (`pca9685` | `mock`) selected by the `backend` parameter.
- Configuration by YAML parameters: `robot.yaml` feeds both `locomotion_node` and the URDF generator so geometry cannot diverge; `servos.yaml` feeds `servo_driver`.
- Optional subsystems attach by topic only (IMU, power, perception, localization); missing sensors degrade gracefully (nodes exit or controller behaves as on flat ground).
- Safety is layered in the chain: deadman/cmd_vel timeout (0.4–0.5 s) in teleop and `locomotion_node`, E-STOP topic honored by both `locomotion_node` and `servo_driver_node`, joint-speed limiting and staggered leg enable in `ServoDriver`.

## Layers

**Teleop (input) layer:**

- Purpose: Convert operator input into `cmd_vel`, `command`, `body_pose`, `estop`.
- Location: `ros2_ws/src/dog_teleop/`, `ros2_ws/src/dog_web/`
- Contains: gamepad reader (Linux joystick API), joy mapper, keyboard raw-terminal node, web server + static page (`ros2_ws/src/dog_web/static/`).
- Depends on: ROS message types only.
- Used by: `locomotion_node`.

**Control layer:**

- Purpose: Mode logic, gait generation, IK, joint target output at 50 Hz.
- Location: `ros2_ws/src/dog_control/`
- Depends on: `rclcpp`, std/sensor/nav/geometry messages; core has no ROS dependency.
- Used by: actuation layer via `joint_commands`; perception via `state`/`odom`.

**Actuation layer:**

- Purpose: Turn joint angles into servo pulses (robot) or Gazebo joint commands (sim).
- Location: `ros2_ws/src/dog_hardware/`, `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py`
- Depends on: I2C `/dev/i2c-*` on hardware; `ros_gz_bridge` in sim.

**Sensing/perception layer:**

- Purpose: Terrain hazard guard, terrain profile, wall map and localization, IMU, power.
- Location: `ros2_ws/src/dog_perception/`, `ros2_ws/src/dog_hardware/src/imu_node.cpp`, `power_monitor_node.cpp`
- Feeds back into control via `guard` (Float64MultiArray) and `terrain/profile` (Float32MultiArray).

**Description/config layer:**

- Purpose: Robot model and parameters.
- Location: `ros2_ws/src/dog_description/`, `ros2_ws/src/dog_bringup/config/`

**Simulation/verification layer:**

- Purpose: Gazebo worlds, bridges, automated checks.
- Location: `ros2_ws/src/dog_gazebo/`, CI in `.github/workflows/ci.yml`

**Offline tooling layer (no ROS):**

- Location: `tools/robot_setup/`, `tools/autocal/`, `tools/sim_video/` (renders videos and `report/` HTML from recorded runs).

## Data Flow

### Primary Request Path (operator → servo)

1. Operator input arrives at `gamepad_node`/`joy_teleop_node`, `keyboard_teleop`, or `web_teleop` (`ros2_ws/src/dog_web/dog_web/web_teleop.py`, WebSocket JSON validated by `protocol.py`).
2. Teleop publishes `cmd_vel`, `command`, `body_pose`, `estop` via `TeleopPublisher` (`ros2_ws/src/dog_teleop/include/dog_teleop/teleop_publisher.hpp`).
3. `locomotion_node` callbacks (`ros2_ws/src/dog_control/src/locomotion_node.cpp:87-120`) feed `LocomotionController`; the 50 Hz `tick()` (`locomotion_node.cpp:291`) runs `update(dt)` and publishes `joint_commands` (`:333`) and latched `state` (`:372`).
4. `servo_driver_node` receives `joint_commands` (`servo_driver_node.cpp:146`), `ServoDriver::setTargets` applies calibration/limits, and `ServoBus` writes PCA9685 pulses; publishes `joint_states`, `servo_pulses`.
5. In simulation `joint_command_bridge` republishes each joint as `std_msgs/Float64` on per-joint gz topics defined by `sim_command_topic()` in `urdf.py`.

### Perception guard loop

1. Sensor topics (`lidar_left/scan`, `lidar_right/scan`, `gs2/scan`, `tof/<n>`, `imu/data`, `joint_states`, `odom`) reach `perception_node` (`perception_node.cpp:210-234`).
2. `HazardGuard` computes limits; `perception_node` publishes `guard` at 10 Hz and `terrain/profile` (`perception_node.cpp:244-247`).
3. `locomotion_node` subscribes (`locomotion_node.cpp:165`, `:193`) → `setGuard`, `setGuardGait`, `setTerrain` (slower, higher step, crawl, stop, sidestep). `guard_timeout` (1.0 s) clears stale guards.
4. `perception/guard` string is also consumed by `web_teleop` for the UI.

### Localization flow

1. `localization_node` subscribes `state`, `imu/data`, `odom`, `<lidar>/scan`, `localization/command` (`localization_node.cpp:159-173`).
2. Builds `WallGrid`/`SubmapMap`, matches scans (`localization.cpp`, `submaps.cpp`), publishes `localization/pose`, `localization/status`, latched `localization/map`. Map persisted to file path given by `localization.map`.

### Calibration flow

1. Laptop tool (`tools/autocal/robotdog_autocal/client.py`) connects to the web WebSocket.
2. `CalibrationBridge` (`ros2_ws/src/dog_web/dog_web/calibration.py`) forwards `cal_pose`/`cal_set` to `servo_driver` through ROS parameter services (`SetParameters`), only while mode is `passive`/`unknown`.
3. `ServoDriverNode::onParams` applies changes live (`add_on_set_parameters_callback`).

**State Management:**

- Controller state lives in `LocomotionController` members; mode published as latched String on `state`. E-STOP is deliberately a volatile (non-latched) reliable topic; nodes start safe (PASSIVE / servos off).

## Key Abstractions

**LocomotionController:**

- Purpose: Testable robot motion brain independent of ROS.
- Examples: `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp`, `ros2_ws/src/dog_control/test/test_locomotion.cpp`
- Pattern: Pull-style `update(dt)` returning whether joints should be sent; setters for external inputs (`setVelocity`, `setImuAttitude`, `addYawRate`, `setGuard`, `setTerrain`).

**ServoBus:**

- Purpose: Swap real PCA9685 for a mock.
- Examples: `ros2_ws/src/dog_hardware/include/dog_hardware/servo_bus.hpp`
- Pattern: Abstract base class + `Pca9685Bus` (Linux i2c-dev) + `MockBus`.

**Sensor auto-probe:**

- `ImuSensor`, `PowerSensor` (`imu_sensor.hpp`, `power_sensor.hpp`): backend `auto|mock|off`; node shuts down if chip not on bus.

**Config-as-code URDF:**

- `build_urdf(geometry, description, gazebo, namespace, initial)` in `ros2_ws/src/dog_description/dog_description/urdf.py`; also used by `sim.launch.py` and `robot.launch.py`.

**Launch composition via `OpaqueFunction`:**

- Arguments like `perception`, `localization`, `gamepad`, `web` toggle nodes in `_setup(context)` in both launch files.

## Entry Points

**Robot launch:**

- Location: `ros2_ws/src/dog_bringup/launch/robot.launch.py`
- Triggers: `ros2 launch dog_bringup robot.launch.py [backend:=mock rviz:=true perception:=true localization:=true]`; Docker CMD in `docker/Dockerfile`.
- Responsibilities: robot_state_publisher, locomotion, servo_driver, optional power/imu/perception/localization/gamepad/web/rviz.

**Simulation launch:**

- Location: `ros2_ws/src/dog_gazebo/launch/sim.launch.py`
- Triggers: `ros2 launch dog_gazebo sim.launch.py [terrain:=slope level:=10 headless:=true ...]`.

**Node executables:** `locomotion_node`, `servo_driver_node`, `power_monitor_node`, `imu_node`, `pca9685_probe`, `gamepad_node`, `joy_teleop_node`, `keyboard_teleop`, `perception_node`, `localization_node` (C++ CMake targets); `web_teleop`, `joint_command_bridge`, `tof_bridge`, `walk_check`, `terrain_sweep`, `perception_check`, `localization_check` (Python console_scripts in `setup.py`); `calib_pose` (`ros2_ws/src/dog_bringup/scripts/calib_pose`).

**Docker:** `docker/entrypoint.sh` sources ROS + `/ws/install`; `docker-compose.yml` runs the `robot` service; `docker/Dockerfile.sim` builds the sim image.

**Tools:** `python3 tools/robot_setup/robot_setup.py`, `python -m robotdog_autocal` (`tools/autocal/robotdog_autocal/__main__.py`), `tools/sim_video/render.py`, `tools/sim_video/report/build_*.py`, `python3 robot_configurator.py`.

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

**What happens:** `ServoConfigReader` defaults to `/home/sg/robot-dog/servo_config.json` (`robot_dog_ws/src/dog_hardware_cpp/include/dog_hardware_cpp/servo_config_reader.hpp`, `robot_dog_ws/src/dog_hardware/dog_hardware/servo_config_reader.py`), and that file does not exist at the repo root (only `legacy/v1/servo_config.json`).
**Why it's wrong:** Breaks in Docker/CI and on any other machine; competes with the YAML source of truth.
**Do this instead:** Pass config via ROS parameters as `servo_driver` does (`ros2_ws/src/dog_bringup/config/servos.yaml`, `robot.yaml`).

### Parallel workspace outside `ros2_ws`

**What happens:** `robot_dog_ws/` (untracked, has no `package.xml`/`CMakeLists.txt`, empty `api/`, `css/`, `js/`) duplicates package names `dog_hardware` and `dog_web` used by `ros2_ws/src`.
**Why it's wrong:** Name collisions with v2 packages; colcon cannot build it as-is; the `legacy/v1/` layout is not v2.
**Do this instead:** Put new packages/code in `ros2_ws/src/<pkg>/` and register them in that package's `CMakeLists.txt`/`setup.py`.

### Duplicated perception algorithm

**What happens:** The core algorithm exists in C++ (`ros2_ws/src/dog_perception/src/core.cpp`) and numpy (`ros2_ws/src/dog_perception/dog_perception/core.py`).
**Why it's wrong:** Changes must be mirrored; tests in `test/test_core.cpp` and `test/test_core.py` guard drift.
**Do this instead:** Change both together and run both tests.

## Error Handling

**Strategy:** Fail safe. Invalid config throws at startup (`servo_driver_node.cpp` throws on invalid calibration or unknown backend); runtime faults degrade to passive/stop.

**Patterns:**

- Stale input timeouts: `cmd_vel_timeout` (0.5 s), `guard_timeout` (1.0 s), web `drive_timeout` (0.4 s) zero the command.
- Commands return accept/reject (`LocomotionController::request` → bool; node logs warning with mode).
- `PowerGuard` events (OVERCURRENT, UNDERVOLTAGE) drive `estop` (`power_sensor.hpp`).
- Unreachable IK targets are clamped and counted (`unreachableCount()`).

## Cross-Cutting Concerns

**Logging:** `RCLCPP_INFO/WARN` via node loggers; Python nodes use `self.get_logger()`.
**Validation:** Web messages validated in `ros2_ws/src/dog_web/dog_web/protocol.py`; calibration checked by `ServoCalibration::validate()` (`servo_driver.hpp`); robot parameters checked by `tools/robot_setup/robot_setup.py --check` in CI.
**Authentication:** None; the web teleop and calibration channel accept any client on the network (`host` defaults to `0.0.0.0`, `allow_calibration` true).

---

*Architecture analysis: 2026-09-29*
