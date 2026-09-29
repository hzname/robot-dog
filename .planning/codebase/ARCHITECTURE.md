---
last_mapped_commit: 104fd262ee2afbd8c9ca25310c9647ab91815a75
last_mapped_at: 2026-09-29
---
<!-- refreshed: 2026-09-29 -->

# Architecture

**Analysis Date:** 2026-09-29

## System Overview

ROS 2 quadruped robot (Robot Dog 2.0) controlled via gamepad, keyboard, or web interface. The architecture follows a publish-subscribe message bus (ROS 2) with all nodes operating in the `/dog` namespace. Central component is the `locomotion_node` which translates operator commands into joint targets. Hardware drivers handle servo control via I2C PCA9685, optional IMU (MPU6050) for heading/slope compensation, and power monitoring. Simulation in Gazebo uses the same control code for validation.

```text
┌──────────────── Control Inputs ─────────────────┐
│ gamepad_node ──joy──▶ joy_teleop_node          │
│ keyboard_teleop (terminal/SSH)                 │
│ web_teleop (HTTP + WebSocket :8080)            │
└──────────────────────┬──────────────────────────┘
                       │ cmd_vel, command, body_pose, estop
                       ▼
┌──────────────────────────────────────────────────────────┐
│ ┌────────────────────────────────────────────────────┐  │
│ │ locomotion_node (C++ ROS 2, 50 Hz control loop)    │  │
│ │ • State machine: PASSIVE → STANDING_UP → STAND     │  │
│ │                           ↔ WALK ↔ LYING          │  │
│ │                           ↔ GREETING/SURVEY        │  │
│ │ • Gaits: trot (normal), crawl (terrain adapt)     │  │
│ │ • Kinematics: IK for 3-dof legs (hip/thigh/calf)  │  │
│ │ • IMU compensation: slope/heading hold            │  │
│ │ Location: ros2_ws/src/dog_control                 │  │
│ └────────────────────────────────────────────────────┘  │
│                       │ joint_commands (12 angles)
│                       │ state (mode string)
│                       │ odom (dead reckoning pose)
└───────────┬───────────────────────────────────────────┘
            │
            ├─────────────────────┬────────────────────────────┐
            │                     │                            │
            ▼                     ▼                            ▼
┌─────────────────────┐ ┌────────────────────┐  ┌──────────────────────┐
│ servo_driver_node   │ │ robot_state_pub    │  │ [optional sensors]   │
│ (C++ ROS 2)         │ │ (ROS 2 standard)   │  │ • imu_node (MPU6050) │
│ • I2C PCA9685 drv   │ │ • Publishes /tf    │  │ • power_monitor      │
│ • Servo calibration │ │   frames           │  │ • perception_node    │
│ • Speed limiting    │ │                    │  │ • localization_node  │
│ • E-STOP logic      │ │ ros2_ws/src/       │  │                      │
│ ros2_ws/src/        │ │   dog_bringup      │  │ ros2_ws/src/         │
│   dog_hardware      │ │                    │  │   dog_hardware,      │
│                     │ │                    │  │   dog_perception     │
└─────────────────────┘ └────────────────────┘  └──────────────────────┘
            │
            ▼
┌─────────────────────────────┐
│ Physical Robot / Gazebo     │
│ • 12 × MG996R servo motors  │
│ • Banana Pi BPI-M4 Zero     │
│ • I2C bus (PCA9685)         │
└─────────────────────────────┘
```

## Component Responsibilities

| Component | Responsibility | Package | File |
|-----------|----------------|---------|------|
| `locomotion_node` | State machine, gaits, IK, command fusion, compensation | `dog_control` | `ros2_ws/src/dog_control/src/locomotion_node.cpp` |
| `LocomotionController` (library) | Core logic independent of ROS | `dog_control` | `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp` |
| `kinematics` (library) | Forward/inverse kinematics, leg geometry | `dog_control` | `ros2_ws/src/dog_control/include/dog_control/kinematics.hpp` |
| `gait` (library) | Trot gait: timing, foot placement, swing trajectory | `dog_control` | `ros2_ws/src/dog_control/include/dog_control/gait.hpp` |
| `crawl` (library) | Crawl gait: terrain-adaptive stepping for stairs | `dog_control` | `ros2_ws/src/dog_control/include/dog_control/crawl.hpp` |
| `servo_driver_node` | I2C servo bus control, safety limits | `dog_hardware` | `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp` |
| `ServoDriver` (library) | PCA9685 protocol, angle→pulse timing | `dog_hardware` | `ros2_ws/src/dog_hardware/include/dog_hardware/servo_driver.hpp` |
| `joy_teleop_node` | ROS joy → cmd_vel/command conversion | `dog_teleop` | `ros2_ws/src/dog_teleop/src/joy_teleop_node.cpp` |
| `gamepad_node` | Linux joystick API → joy messages | `dog_teleop` | `ros2_ws/src/dog_teleop/src/gamepad_node.cpp` |
| `keyboard_teleop` | Terminal stdin → cmd_vel (TTY only) | `dog_teleop` | `ros2_ws/src/dog_teleop/src/keyboard_teleop.cpp` |
| `web_teleop` (Python) | HTTP + WebSocket server, virtual joystick | `dog_web` | `ros2_ws/src/dog_web/dog_web/web_teleop.py` |
| `robot.launch.py` | Node orchestration, parameter loading | `dog_bringup` | `ros2_ws/src/dog_bringup/launch/robot.launch.py` |
| `URDF generator` | Dynamic URDF from robot.yaml | `dog_description` | `ros2_ws/src/dog_description/dog_description/urdf.py` |
| `perception_node` | Terrain height map, obstacle detection | `dog_perception` | `ros2_ws/src/dog_perception/src/perception_node.cpp` |
| `localization_node` | Map building and pose tracking | `dog_perception` | `ros2_ws/src/dog_perception/src/localization_node.cpp` |

## Pattern Overview

**Overall:** ROS 2 pub/sub message-passing architecture with centralised control state machine. Core algorithms (kinematics, gaits, controllers) are ROS-independent C++ libraries used by ROS node wrappers. Configuration-driven via `robot.yaml` which both robot_state_publisher (URDF generation) and locomotion_node (parameters) consume.

**Key Characteristics:**

- **Message-based:** All inter-node communication via ROS 2 topics (no direct calls)
- **50 Hz control loop:** locomotion_node runs deterministically at stated rate
- **Namespace isolation:** All nodes in `/dog` namespace for multi-robot readiness
- **Hardware abstraction:** Servo driver supports `pca9685` (real I2C) and `mock` (testing) backends
- **ROS library pattern:** Non-ROS C++ libraries (kinematics, gait, crawl) imported by ROS nodes
- **Configuration-driven:** Single source of truth (`robot.yaml`) for geometry, gains, gait parameters

## Layers

**Operator Interface Layer:**

- Purpose: Capture and forward human commands (gamepad buttons/sticks, keyboard, web)
- Location: `ros2_ws/src/dog_teleop`, `ros2_ws/src/dog_web`
- Contains: gamepad_node, joy_teleop_node, keyboard_teleop, web_teleop (Python)
- Depends on: Linux joystick API, Python asyncio/WebSocket, ROS 2 publishers
- Used by: locomotion_node subscribes to their output topics

**Control Layer:**

- Purpose: Translate continuous commands (twist) and discrete requests into joint targets
- Location: `ros2_ws/src/dog_control`
- Contains: locomotion_node (ROS wrapper) + libraries (locomotion, kinematics, gait, crawl, greet, survey)
- Depends on: robot.yaml parameters, optional IMU/terrain data
- Used by: servo_driver_node (subscribes joint_commands)

**Hardware Abstraction Layer:**

- Purpose: Drive servo motors and read sensors (power, IMU)
- Location: `ros2_ws/src/dog_hardware`
- Contains: servo_driver_node, power_monitor_node, imu_node (C++ ROS wrappers) + libraries
- Depends on: Linux i2c-dev, hardware (real or mocked for testing)
- Used by: Gazebo simulator or physical hardware

**Perception Layer (Optional):**

- Purpose: Terrain analysis and localization for autonomous obstacle avoidance
- Location: `ros2_ws/src/dog_perception`
- Contains: perception_node (C++), localization_node (C++), core.py (offline numpy)
- Depends on: lidar/ToF sensor data, IMU, odometry
- Used by: locomotion_node (subscribes guard, terrain/profile)

**Orchestration/Config Layer:**

- Purpose: Launch nodes, load shared parameters, generate URDF
- Location: `ros2_ws/src/dog_bringup`, `ros2_ws/src/dog_description`
- Contains: robot.launch.py, robot.yaml, urdf.py
- Depends on: ROS 2 launch framework, YAML parser
- Used by: All nodes (parameters), robot_state_publisher (URDF)

**Simulation Layer:**

- Purpose: Gazebo physics simulation and test automation
- Location: `ros2_ws/src/dog_gazebo`
- Contains: sim.launch.py, world files, walk_check, terrain_sweep, perception_check
- Depends on: ros_gz bridge, Gazebo simulator
- Used by: CI/CD validation, development

## Data Flow

### Primary Request Path: "Stand and Walk Forward"

1. **Operator input** (`web_teleop` or `gamepad_node`) publishes:
   - `cmd_vel` with `linear.x = 0.1` m/s (forward)
   - `command = "stand"` (once, or already standing)

2. **locomotion_node** (50 Hz):
   - Subscribes to `cmd_vel`, `command` topics
   - State machine: PASSIVE → STANDING_UP (if "stand" received) → STAND → WALK
   - Gait engine (trot or crawl) computes foot positions for current cycle
   - Kinematics IK: body frame foot targets → joint angles (hip/thigh/calf)
   - Publishes `joint_commands` (12 float angles [rad])
   - Publishes `state` = "walk"
   - Publishes `odom` (dead-reckoned pose)

3. **servo_driver_node** (subscribes `joint_commands`):
   - Reads 12 angles from message
   - Applies calibration offsets (per servo)
   - Clamps to joint limits
   - Rate-limits (max joint speed 5.5 rad/s)
   - Converts angles → PCA9685 pulse widths
   - Writes I2C to PCA9685 PWM controller
   - Real hardware: drives 12 × MG996R servos
   - Mock mode: stores values, no I2C

4. **Physical result:**
   - Legs move, robot walks forward
   - Odometry updated on next cycle (dead reckoning + IMU heading)

### Secondary Flow: IMU Compensation (Optional)

When `imu_node` (MPU6050) is enabled:

- Publishes `imu/data` (sensor_msgs/Imu) at ~10 Hz
- `locomotion_node` subscribes, extracts roll/pitch/yaw
- **Slope compensation:** Downhill feet shifted by `gain × height × tan(slope)`
- **Heading hold:** Gyro yaw error fed back (PI regulator) into trot yaw rate

### Tertiary Flow: Perception Guard (Optional, terrain detection)

When `perception_node` enabled:

- Subscribes: lidar_left/scan, lidar_right/scan, gs2/scan, tof/*, IMU, odometry
- Computes: ground plane, elevation map, obstacles
- Publishes `perception/guard` with: max forward speed, leg swing heights, gait choice, obstacle avoidance sideways velocity
- `locomotion_node` subscribes, applies guard limits (slows or stops gait)

### State Management

**locomotion_node internal state:**

- **Mode:** Current operational state (PASSIVE, STANDING_UP, STAND, WALK, LYING, GREETING, SURVEY, LYING_DOWN)
- **Gait state:** Trot/crawl cycle phase (0..1), foot positions
- **Body pose:** roll/pitch/height [rad/rad/m]
- **Velocity command:** vx, vy, angular.z [m/s, m/s, rad/s]
- **E-stop flag:** latches on estop topic, clears only on explicit "stand" command

## Key Abstractions

**Locomotion State Machine:**

- Purpose: Ensure safe transitions between standing, walking, lying, and special poses
- Examples: `ros2_ws/src/dog_control/src/locomotion.cpp`
- Pattern: Enum-based (Mode enum) with transition table; each state has entry/exit/tick logic
- Prevents invalid commands (e.g., walk from PASSIVE) and intermediate states (STANDING_UP)

**Gait Generators (Trot & Crawl):**

- Purpose: Compute foot positions in body frame given desired velocity and cycle phase
- Examples: `ros2_ws/src/dog_control/include/dog_control/gait.hpp`, `crawl.hpp`
- Pattern: Stateless functions; caller manages phase accumulation
- Trot: two diagonal pairs (LF+RR, RF+LR), 0.65 duty cycle, 0.55 s period
- Crawl: three feet down, one leg swinging, terrain-aware (terrain/profile topic)

**Inverse Kinematics (IK):**

- Purpose: Convert body-frame foot targets → joint angles for 3-dof leg
- Examples: `ros2_ws/src/dog_control/include/dog_control/kinematics.hpp`
- Pattern: Geometric solver; knee-backward constraint (q2 < 0)
- Verified by tests against forward kinematics on 1000+ random points

**Servo Pulse Timing:**

- Purpose: Map joint angles [rad] → PCA9685 pulse widths for analog servo control
- Examples: `ros2_ws/src/dog_hardware/src/servo_driver.cpp`
- Pattern: Calibration (zero offset per servo) + linear scaling (angle → microseconds)
- Angle range clamped to joint limits (hip: ±40°, thigh: -45°…135°, calf: -165°…-15°)

**ROS Message Protocol:**

- Purpose: Async, type-safe inter-node communication
- Examples: `geometry_msgs/Twist`, `std_msgs/String`, `sensor_msgs/JointState`
- Pattern: Topics (pub/sub, fire-and-forget) for data streams; no RPC calls
- QoS: Most topics latched (retain last), some best-effort (lidar), some reliable

## Entry Points

**`robot.launch.py`:**

- Location: `ros2_ws/src/dog_bringup/launch/robot.launch.py`
- Triggers: `ros2 launch dog_bringup robot.launch.py [args]`
- Responsibilities: 
  - Parse launch arguments (backend, gamepad, web, perception, localization, etc.)
  - Load robot.yaml and servos.yaml
  - Generate URDF via `dog_description.urdf.build_urdf()`
  - Instantiate all enabled nodes with their parameters
  - Handle optional sensors (IMU, power monitor) with backend probing

**`locomotion_node`:**

- Location: `ros2_ws/src/dog_control/src/locomotion_node.cpp`
- Triggers: Launched by robot.launch.py or standalone
- Responsibilities:
  - Run 50 Hz control loop
  - Fuse cmd_vel, command, body_pose, estop, imu/data, terrain/profile
  - Compute joint targets and publish joint_commands
  - Manage state machine, gait switching, mode transitions

**`servo_driver_node`:**

- Location: `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`
- Triggers: Launched by robot.launch.py
- Responsibilities:
  - Subscribe joint_commands at 50 Hz
  - Apply servo calibration and limits
  - Rate-limit joint velocities
  - Write to I2C (real) or mock buffer
  - Publish joint_states (feedback, optional)

**`web_teleop` (Python):**

- Location: `ros2_ws/src/dog_web/dog_web/web_teleop.py`
- Triggers: `ros2 run dog_web web_teleop --ros-args -r __ns:=/dog --params-file <yaml>`
- Responsibilities:
  - Serve HTTP page on :8080 (virtual joystick, keyboard, gamepad API)
  - Upgrade HTTP → WebSocket for real-time client ↔ server communication
  - Parse incoming JSON commands (stick positions, key presses, gamepad events)
  - Publish cmd_vel, command, estop, body_pose to ROS topics
  - Subscribe state, power, perception/guard and broadcast to connected clients

**`walk_check` (Simulation test):**

- Location: `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py`
- Triggers: `ros2 run dog_gazebo walk_check [options]`
- Responsibilities:
  - Launch Gazebo with sim.launch.py
  - Send walk commands in eight directions
  - Verify odometry matches expected distance (±tolerance)
  - Check gait stability (e.g., no rollover)
  - Report pass/fail per direction

## Architectural Constraints

- **Threading:** Single-threaded ROS executors; `locomotion_node` runs callbacks sequentially on 50 Hz timer
- **Global state:** None module-level singletons; state is encapsulated in ROS node objects and topic subscribers
- **Circular imports:** None detected; library dependencies are acyclic (kinematics → gait / crawl / locomotion)
- **Real-time:** No hard RT guarantees; ROS 2 single-threaded executor is best-effort scheduling
- **Message latency:** ~20 ms (50 Hz tick) between input command and joint output; I2C servo response ~5–10 ms additional
- **I2C bus contention:** All 12 servos on single PCA9685 (one I2C address); writes are atomic per servo (one PWM channel at a time)
- **Namespace isolation:** All nodes explicitly in `/dog` namespace; allows multi-dog or multi-arm extensions
- **Backend abstraction:** Hardware backend (`pca9685` vs `mock`) determined at launch time; no runtime switching

## Anti-Patterns

### Missing ROS 2 Parameter Overrides

**What happens:** Launch file hardcodes paths to robot.yaml and servos.yaml; difficult to deploy different robot configurations without modifying launch file.

**Why it's wrong:** Each robot or test variant needs unique geometry/calibration; no way to hot-swap configs without rebuilding/relaunching.

**Do this instead:** Use launch arguments (`robot_config`, `servo_config`) in `robot.launch.py` (line 30–31) to accept custom paths: `ros2 launch dog_bringup robot.launch.py robot_config:=/path/to/custom.yaml`

### Direct Joint Angle Publishing

**What happens:** Some test code writes directly to `/dog/joint_commands` instead of going through command translation (cmd_vel → locomotion → joint_commands).

**Why it's wrong:** Bypasses safety checks (rate limiting, E-STOP, collision avoidance), state machine validation, and odometry tracking.

**Do this instead:** Publish `cmd_vel` (Twist), `command` (String), or `body_pose` (Vector3); let `locomotion_node` translate to safe joint targets.

### Hardcoded Magic Numbers in Servo Calibration

**What happens:** Joint angle offsets stored in source code comments or scattered across test files instead of centralized.

**Why it's wrong:** Calibration values must be measured per robot and updated without recompilation; source-code values become stale.

**Do this instead:** Store calibration in `servos.yaml` under `ros__parameters.servos.[servo_name].zero_offset`; `servo_driver_node` loads at startup.

## Error Handling

**Strategy:** Defensive checks with warnings/errors logged to ROS 2 logger; graceful degradation (disable optional features, remain operational).

**Patterns:**

- **Out-of-range angles:** Locomotion clamps joint targets to limits before publishing; servo_driver clamps again before I2C write
- **Timeout handling:** `cmd_vel_timeout` (0.5 s default) causes `locomotion_node` to zero velocity if no new command received (failsafe)
- **Sensor failures:** Optional sensors (IMU, power monitor, perception) fail silently if not found on I2C; robot operates with basic control only
- **E-STOP latch:** Once E-STOP triggered, robot goes limp (PASSIVE mode); only "stand" command clears it (requires operator re-engagement)
- **Missing topics:** Subscriptions default to sensible values (e.g., no IMU → no slope compensation)

## Cross-Cutting Concerns

**Logging:** Uses ROS 2 RCLCPP_INFO/WARN/ERROR to stdout; level controllable via ROS_LOG_LEVEL env var.

**Validation:** Config validation in `robot_setup.py` (checks robot.yaml for required keys, reasonable ranges); URDF generated from same config ensures consistency.

**Authentication:** None; assumes trusted local network. Web teleop has optional calibration mode (allow_calibration param) disabled by default in production.

**Synchronization:** No explicit mutex/locks; ROS executor serializes callbacks. IMU/odometry update at different rates (10 Hz IMU, 50 Hz control) but safe due to read-only subscription model.

---

*Architecture analysis: 2026-09-29*
