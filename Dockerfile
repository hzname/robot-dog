# ROS2 Humble + Rust для Banana Pi (ARM64)
FROM arm64v8/ros:humble-ros-base

# Аргументы сборки
ARG USERNAME=banana
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git vim htop i2c-tools \
    build-essential cmake \
    curl pkg-config libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Установка Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN rustup target add aarch64-unknown-linux-gnu

# Установка rclrs (ROS2 Rust)
RUN cargo install cargo-ros2

# Python библиотеки
RUN pip3 install --no-cache-dir \
    adafruit-circuitpython-servokit \
    numpy \
    transforms3d

# rosdep
RUN rosdep init || true && rosdep update

# Пользователь
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME

# Копируем Rust toolchain для пользователя
RUN cp -r /root/.cargo /home/$USERNAME/ && chown -R $USERNAME:$USERNAME /home/$USERNAME/.cargo

# Окружение
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> /home/$USERNAME/.bashrc

WORKDIR /workspace
USER $USERNAME
ENV PATH="/home/banana/.cargo/bin:${PATH}"

CMD ["/bin/bash"]
