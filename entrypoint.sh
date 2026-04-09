#!/bin/bash
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
ros2 run dog_hardware_cpp servo_driver_node --ros-args \
  -p bus_type:=i2c \
  -p device_port:=/dev/i2c-0 \
  -p watchdog_timeout_s:=10.0 &
SERVO_PID=$!
sleep 3

# Retry lifecycle config
echo "Configuring servo driver..."
for i in 1 2 3 4 5; do
  ros2 lifecycle set /servo_driver_node configure && break
  echo "Retry $i..."
  sleep 2
done
ros2 lifecycle set /servo_driver_node activate

echo "=== ALL SYSTEMS ONLINE ==="

# Keep alive - wait for background processes
wait $GAIT_PID $SERVO_PID 2>/dev/null
