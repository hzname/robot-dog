#!/bin/bash
# Запуск тестов в Docker

docker run --rm -v $(pwd)/robot_dog_ws:/workspace robot-dog-builder -c "
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    cd /workspace
    colcon test --packages-select dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp
    colcon test-result --verbose
"
