#!/bin/bash
# RobotDogQwen startup script

set -e

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

echo '=== SYSTEM READY ==='

# Start servo driver (auto-configures and activates)
echo 'Starting servo driver...'
ros2 run dog_hardware_cpp servo_driver_node \
  --ros-args \
  -p bus_type:=i2c \
  -p device_port:=/dev/i2c-0 &
sleep 3

# Start gait controller (auto-activates)
echo 'Starting gait controller...'
ros2 run dog_control_cpp gait_controller &
sleep 2

# Start ROS bridge
echo 'Starting ROS bridge...'
rm -f /tmp/robot_dog/ros_bridge.sock
mkdir -p /tmp/robot_dog
python3 -u /workspace/ros_bridge.py &
sleep 1

# Start health monitor
echo 'Starting health monitor...'
ros2 run dog_monitor_cpp health_monitor &

echo '=== ALL SYSTEMS ONLINE ==='
# Keep container alive
exec tail -f /dev/null
