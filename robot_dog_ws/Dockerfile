# ROS2 Humble для Banana Pi (ARM64)
FROM arm64v8/ros:humble-ros-base

# Аргументы сборки
ARG USERNAME=banana
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    # ROS2 пакеты
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    # Python
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Инструменты
    git \
    vim \
    htop \
    i2c-tools \
    # Сборка
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

# Настройка окружения
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Рабочая директория
WORKDIR /workspace

# Переключение на пользователя
USER $USERNAME

# Entrypoint
CMD ["/bin/bash"]
