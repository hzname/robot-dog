# ROS2 Jazzy for Banana Pi (ARM64)
FROM arm64v8/ros:jazzy-ros-base

# Build arguments
ARG USERNAME=banana
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install dependencies
RUN apt-get update && apt-get install -y \
    # ROS2 packages
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2 \
    ros-jazzy-rviz2 \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-rclcpp-components \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    # Python
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Tools
    git \
    vim \
    htop \
    i2c-tools \
    # Build tools
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Создание пользователя
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Установка Python библиотек
RUN pip3 install --no-cache-dir \
    adafruit-circuitpython-servokit \
    numpy \
    transforms3d

# Инициализация rosdep
RUN rosdep init || true && rosdep update

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

# Рабочая директория
WORKDIR /workspace

# Переключение на пользователя
USER $USERNAME

# Entrypoint
CMD ["/bin/bash"]
