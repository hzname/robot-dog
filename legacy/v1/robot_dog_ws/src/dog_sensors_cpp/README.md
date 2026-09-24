# Dog Sensors C++ - MPU6050 Driver

ROS2 package with MPU6050 IMU driver and simulator for robot dog project.

## Features

- **MPU6050 Hardware Driver**: I2C communication with real MPU6050 sensor
- **Mahony AHRS Filter**: Sensor fusion for orientation estimation
- **Software Simulator**: Test without hardware with realistic motion patterns
- **Lifecycle Management**: Proper ROS2 lifecycle node implementation
- **Multiple Motion Modes**: Idle, walking, trotting, turning, pacing

## Quick Start

### Hardware Mode (with MPU6050)

```bash
# Default auto-detect mode (tries hardware, falls back to simulator)
ros2 launch dog_sensors_cpp imu_launch.py

# Force hardware mode
ros2 launch dog_sensors_cpp imu_launch.py driver_type:=mpu6050

# Custom I2C settings
ros2 launch dog_sensors_cpp imu_launch.py i2c_bus:='/dev/i2c-1' i2c_address:='0x68'
```

### Simulation Mode (no hardware required)

```bash
# Launch with simulator
ros2 launch dog_sensors_cpp imu_simulation.launch.py

# Different motion modes
ros2 launch dog_sensors_cpp imu_simulation.launch.py motion_mode:=walking
ros2 launch dog_sensors_cpp imu_simulation.launch.py motion_mode:=trotting
ros2 launch dog_sensors_cpp imu_simulation.launch.py motion_mode:=turning

# Control with velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Filtered IMU data with orientation |
| `/body_pose` | `geometry_msgs/Pose` | Integrated pose estimate |
| `/imu/status` | `std_msgs/Bool` | IMU status (true = active) |

## Parameters

### General Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | "imu_link" | TF frame ID |
| `publish_rate_hz` | double | 100.0 | Publication rate |
| `driver_type` | string | "auto" | Driver: auto, mpu6050, simulator, none |
| `simulate` | bool | false | Enable simulation mode |
| `publish_pose` | bool | true | Publish integrated pose |

### MPU6050 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_bus` | string | "/dev/i2c-1" | I2C bus device path |
| `i2c_address` | int | 0x68 | I2C address (0x68 or 0x69) |

### Simulator Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sim_motion_mode` | string | "idle" | Motion: idle, walking, trotting, turning, pacing |
| `sim_walk_frequency` | double | 1.0 | Gait frequency (Hz) |
| `sim_walk_amplitude` | double | 0.15 | Motion amplitude (rad) |

## Hardware Setup

### Wiring (Raspberry Pi)

```
MPU6050    Raspberry Pi
-------    ------------
VCC    →   3.3V (Pin 1)
GND    →   GND (Pin 6)
SCL    →   SCL (Pin 5, GPIO 3)
SDA    →   SDA (Pin 3, GPIO 2)
AD0    →   GND (for 0x68) or 3.3V (for 0x69)
```

### Enable I2C

```bash
sudo raspi-config
# Interface Options → I2C → Enable

# Verify
sudo i2cdetect -y 1
# Should show 68 or 69
```

### Permissions

```bash
# Add user to i2c group (no sudo needed)
sudo usermod -a -G i2c $USER
# Logout and login again
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         IMU Node                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │  MPU6050    │    │  Simulator  │    │    None     │     │
│  │   Driver    │    │             │    │  (dummy)    │     │
│  │             │    │             │    │             │     │
│  │ • I2C comm  │    │ • Synthetic │    │ • Zeros     │     │
│  │ • Mahony    │    │ • Gait sim  │    │             │     │
│  │   filter    │    │ • Velocity  │    │             │     │
│  │             │    │   control   │    │             │     │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘     │
│         └────────────────────┼──────────────────┘            │
│                              ▼                              │
│                    ┌──────────────────┐                     │
│                    │  Pose Integration │                     │
│                    └────────┬─────────┘                     │
│                             ▼                               │
│         ┌──────────────────────────────────────┐            │
│         │  Publishers: /imu/data, /body_pose   │            │
│         └──────────────────────────────────────┘            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Files

```
dog_sensors_cpp/
├── include/dog_sensors_cpp/
│   ├── imu_node.hpp           # Lifecycle node header
│   ├── mpu6050_driver.hpp     # MPU6050 driver with Mahony filter
│   └── mpu6050_simulator.hpp  # Software simulator
├── src/
│   ├── imu_node.cpp           # Node implementation
│   ├── imu_node_main.cpp      # Entry point
│   ├── mpu6050_driver.cpp     # MPU6050 I2C implementation
│   └── mpu6050_simulator.cpp  # Simulator implementation
├── launch/
│   ├── imu_launch.py          # Generic launch file
│   └── imu_simulation.launch.py  # Simulation-only launch
├── config/
│   └── imu_params.yaml        # Configuration parameters
└── test/
    └── test_imu_simulation.sh # Test script
```

## Testing

```bash
# Build
cd ~/robot-dog/robot_dog_ws
colcon build --packages-select dog_sensors_cpp --symlink-install

# Source
source install/setup.bash

# Test simulation (no hardware needed)
ros2 launch dog_sensors_cpp imu_simulation.launch.py

# View data
ros2 topic echo /imu/data
ros2 topic hz /imu/data
```

## License

MIT License
