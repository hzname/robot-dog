#!/bin/bash
# Сборка и тестирование в Docker с правильными путями

set -e

echo "🔧 Сборка и тестирование robot-dog в Docker"

# Сборка
echo "📦 Сборка..."
docker run --rm \
  -v $(pwd)/robot_dog_ws:/workspace \
  -w /workspace \
  ros:jazzy \
  bash -c "
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-select dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp
  "

# Тестирование
echo "🧪 Тестирование..."
docker run --rm \
  -v $(pwd)/robot_dog_ws:/workspace \
  -w /workspace \
  ros:jazzy \
  bash -c "
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    colcon test --packages-select dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp
    colcon test-result --verbose
  "

echo "✅ Готово!"
