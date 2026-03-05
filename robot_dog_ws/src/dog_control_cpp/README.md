# dog_control_cpp

C++ control algorithms for quadruped robot dog with lifecycle management, featuring balance control integrated with gait generation.

## Overview

This package provides ROS2 control nodes for a quadruped robot dog:

- **Gait Controller**: Implements trot gait with 3-DOF inverse kinematics
- **Balance Controller**: PID-based stabilization using IMU feedback
- **IMU Simulator**: For testing balance control without hardware

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Control Architecture                      │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐      ┌──────────────────┐                 │
│  │   IMU Sensor │──────▶ Balance Controller│                 │
│  │  (or Sim)    │      │   (PID control)  │                 │
│  └──────────────┘      └────────┬─────────┘                 │
│                                 │                            │
│                                 ▼                            │
│                        /balance_adjustment                   │
│                    (roll, pitch, height)                     │
│                                 │                            │
│                                 ▼                            │
│  ┌──────────────┐      ┌──────────────────┐                 │
│  │   cmd_vel    │──────▶  Gait Controller │                 │
│  │   (Twist)    │      │   (Trot + IK)    │                 │
│  └──────────────┘      └────────┬─────────┘                 │
│                                 │                            │
│                                 ▼                            │
│                        /joint_trajectory                     │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Nodes

### Balance Controller

Lifecycle node that stabilizes the robot body using PID control.

**Subscriptions:**
- `/imu/data` (sensor_msgs/Imu): IMU orientation data
- `/cmd_height` (std_msgs/Float64): Target body height
- `/cmd_body_pose` (geometry_msgs/Pose): Target body orientation

**Publications:**
- `/balance_adjustment` (geometry_msgs/Vector3):
  - `x`: Roll correction (rad)
  - `y`: Pitch correction (rad)
  - `z`: Height adjustment (m)
- `/balance_state` (std_msgs/Float64MultiArray): Debug state

**Parameters:**
- `kp_roll`, `ki_roll`, `kd_roll`: PID gains for roll
- `kp_pitch`, `ki_pitch`, `kd_pitch`: PID gains for pitch
- `max_roll_correction`, `max_pitch_correction`: Output limits
- `control_rate`: Control loop frequency (Hz)

### Gait Controller

Lifecycle node that generates trot gait with balance compensation.

**Subscriptions:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/body_pose` (geometry_msgs/Pose): Body pose adjustments
- `/balance_adjustment` (geometry_msgs/Vector3): Balance corrections

**Publications:**
- `/joint_trajectory` (trajectory_msgs/JointTrajectory): Joint commands
- `/foot_positions` (std_msgs/Float64MultiArray): Debug foot positions
- `/gait_state` (std_msgs/Float64MultiArray): Debug gait state

**Parameters:**
- `gait_type`: Type of gait ("trot", "walk", "bound")
- `stance_height`: Standing height (m)
- `step_height`: Swing foot clearance (m)
- `gait_period`: Duration of one gait cycle (s)
- `balance_enabled`: Enable balance compensation
- `balance_response_factor`: Scaling for balance corrections

### IMU Simulator

Regular node that simulates IMU data for testing.

**Subscriptions:**
- `/imu/disturbance` (geometry_msgs/Vector3): Apply disturbances

**Publications:**
- `/imu/data` (sensor_msgs/Imu): Simulated IMU data

**Parameters:**
- `static_roll`, `static_pitch`: Static offsets (rad)
- `osc_amplitude_roll`, `osc_amplitude_pitch`: Vibration amplitude
- `osc_frequency`: Vibration frequency (Hz)
- `noise_stddev`: Sensor noise level

## Launch Files

### Simulation Mode

```bash
ros2 launch dog_control_cpp control_simulation.launch.py
```

Launches: gait controller + balance controller + IMU simulator

### Hardware Mode

```bash
ros2 launch dog_control_cpp control_launch.py
```

Launches: gait controller + balance controller (expects real IMU on `/imu/data`)

## PID Tuning Guide

### Starting Values

Use the default `balance_params.yaml` as a starting point:

```yaml
kp_roll: 0.15
ki_roll: 0.01
kd_roll: 0.05
```

### Tuning Procedure

1. **Set Ki and Kd to 0**, tune Kp first
   - Increase Kp until the robot responds quickly but doesn't oscillate
   - If it oscillates, reduce Kp

2. **Add Kd** (damping)
   - Increase Kd to reduce overshoot
   - Too much Kd causes noise amplification

3. **Add Ki** (steady-state error correction)
   - Increase Ki slowly to eliminate steady-state error
   - Watch for integral windup (use `max_integral_*` limits)

### Troubleshooting

| Symptom | Solution |
|---------|----------|
| Slow response | Increase Kp |
| Oscillations | Decrease Kp, increase Kd |
| Overshoot | Increase Kd |
| Steady-state error | Increase Ki |
| Integral windup | Decrease max_integral limits |

### Aggressive Tuning

For faster response (may be less stable), use `balance_params_aggressive.yaml`:

```bash
ros2 launch dog_control_cpp control_simulation.launch.py \
  balance_config:=install/dog_control_cpp/share/dog_control_cpp/config/balance_params_aggressive.yaml
```

## Testing in Simulation

### 1. Launch the simulation

```bash
ros2 launch dog_control_cpp control_simulation.launch.py
```

### 2. Monitor topics

```bash
# View balance corrections
ros2 topic echo /balance_adjustment

# View joint commands
ros2 topic echo /joint_trajectory

# View balance state
ros2 topic echo /balance_state
```

### 3. Send velocity commands

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

### 4. Apply disturbances

Test balance response by applying disturbances:

```bash
# Roll disturbance (tilt left/right)
ros2 topic pub /imu/disturbance geometry_msgs/msg/Vector3 '{x: 0.1, y: 0.0, z: 0.0}'

# Pitch disturbance (tilt forward/back)
ros2 topic pub /imu/disturbance geometry_msgs/msg/Vector3 '{x: 0.0, y: 0.1, z: 0.0}'
```

### 5. Change height

```bash
ros2 topic pub /cmd_height std_msgs/msg/Float64 '{data: 0.20}'
```

### 6. Visualize

```bash
# Plot balance adjustments
ros2 run rqt_plot rqt_plot /balance_adjustment/x /balance_adjustment/y /balance_adjustment/z

# Plot IMU data
ros2 run rqt_plot rqt_plot /imu/data/orientation/x /imu/data/orientation/y
```

## Balance Integration

The gait controller applies balance corrections to foot positions:

1. **Height correction**: Applied to all legs equally
2. **Roll correction**: Left legs move opposite to right legs
3. **Pitch correction**: Front legs move opposite to rear legs

Formula:
```
foot_z += height_correction
foot_z += roll_correction * leg_sign_y * (body_width / 2)
foot_z -= pitch_correction * leg_sign_x * (body_length / 2)
```

## Coordinate Frames

- **X**: Forward
- **Y**: Left
- **Z**: Up
- **Roll**: Rotation around X (tilt left/right)
- **Pitch**: Rotation around Y (tilt forward/back)
- **Yaw**: Rotation around Z (turn)

## Hardware Integration

To use with real hardware:

1. Ensure your IMU publishes to `/imu/data`
2. Launch with hardware mode: `ros2 launch dog_control_cpp control_launch.py`
3. Connect joint trajectory output to your motor driver
4. Tune PID parameters for your specific robot

## Development

### Building

```bash
cd ~/robot-dog/robot_dog_ws
colcon build --packages-select dog_control_cpp
```

### Testing

```bash
# Run tests
colcon test --packages-select dog_control_cpp
```

### Code Style

```bash
# Format code
ament_clang_format --reformat src/

# Check code
ament_clang_tidy src/
```

## License

MIT
