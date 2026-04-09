#!/bin/bash
# Entrypoint for robot_dog container
set -e

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

echo "=== SYSTEM READY ==="

# Start gait controller
echo "Starting gait controller..."
ros2 run dog_control_cpp gait_controller &
GAIT_PID=$!
sleep 3

echo "Configuring gait controller..."
ros2 lifecycle set /gait_controller configure
ros2 lifecycle set /gait_controller activate

# Start servo driver
echo "Starting servo driver..."
sleep 1
ros2 run dog_hardware_cpp servo_driver_node --ros-args \
  -p bus_type:=i2c \
  -p device_port:=/dev/i2c-0 \
  -p watchdog_timeout_s:=10.0 &
SERVO_PID=$!
sleep 2

echo "Configuring servo driver..."
ros2 lifecycle set /servo_driver_node configure
ros2 lifecycle set /servo_driver activate

echo "=== ALL SYSTEMS ONLINE ==="

# Wait for any process to exit
wait -n $GAIT_PID $SERVO_PID 2>/dev/null || wait $GAIT_PID
