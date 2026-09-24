#!/bin/bash
# Скрипт для сборки robot-dog в Docker

set -e

echo "🔧 Сборка robot-dog в Docker (ROS2 Jazzy)"

# Проверка Docker
if ! command -v docker &> /dev/null; then
    echo "❌ Docker не установлен. Установи Docker first."
    exit 1
fi

# Сборка образа
echo "📦 Сборка Docker образа..."
docker build -t robot-dog-builder .

# Запуск сборки
echo "🔨 Сборка C++ пакетов..."
docker run --rm -v $(pwd)/robot_dog_ws:/workspace robot-dog-builder -c "
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-select dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp
    echo '✅ Сборка завершена!'
"

# Копирование build artifacts
echo "📋 Копирование результатов сборки..."
docker run --rm -v $(pwd)/robot_dog_ws:/workspace robot-dog-builder -c "
    cp -r /workspace/build /workspace/install /workspace/log /workspace/src/*_cpp/
" 2>/dev/null || true

echo "✅ Готово! Проверь robot_dog_ws/build и robot_dog_ws/install"
