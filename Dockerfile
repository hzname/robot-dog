# Dockerfile для сборки robot-dog C++ пакетов на ROS2 Jazzy
FROM ros:jazzy

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    clang-format \
    clang-tidy \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Создание workspace
WORKDIR /workspace

# Копирование исходников
COPY robot_dog_ws/src ./src

# Сборка
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select \
    dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp"

# Настройка entrypoint
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'source /workspace/install/setup.bash' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
