# Robot Dog ROS2 Workspace

ROS2 workspace для квадрупедного робота-собаки с Python и Rust нодами.

## 📁 Структура

```
robot_dog_ws/
├── src/
│   ├── dog_bringup/          # Launch файлы и конфигурация
│   ├── dog_description/      # URDF, меши, описание робота
│   ├── dog_control/          # Python: контроллеры походки и баланса
│   ├── dog_hardware/         # Python: интерфейс сервоприводов
│   ├── dog_sensors/          # Python: драйверы сенсоров (IMU)
│   ├── dog_teleop/           # Python: телеуправление
│   ├── dog_control_rust/     # Rust: контроллер походки
│   ├── dog_sensors_rust/     # Rust: IMU нода
│   ├── dog_hardware_rust/    # Rust: драйвер сервоприводов
│   ├── dog_balance_rust/     # Rust: контроллер баланса
│   └── dog_teleop_rust/      # Rust: телеуправление
```

## 🚀 Быстрый старт

### Требования

- ROS2 Humble или Jazzy
- Python 3.8+
- Rust 1.70+ (для Rust нод)
- colcon

### Сборка

```bash
cd ~/robot_dog_ws

# Установка зависимостей
rosdep install --from-paths src --ignore-src -y

# Сборка всего workspace
colcon build --symlink-install

# Source окружения
source install/setup.bash
```

### Запуск

```bash
# Полный запуск (Python ноды + bridge)
ros2 launch dog_bringup robot.launch.py

# Запуск с Rust нодами
ros2 launch dog_bringup rust_robot.launch.py use_rust:=true

# Отдельные компоненты
ros2 run dog_control gait_controller
ros2 run dog_sensors imu_node
ros2 run dog_teleop teleop_keyboard

# Rust ноды напрямую
ros2 run dog_control_rust gait_controller_rust
ros2 run dog_sensors_rust imu_node_rust
```

## 🔧 Деплой на Banana Pi

### 1. Подготовка

```bash
# На Banana Pi
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions

# Установка ROS2 (если не установлен)
# См. официальную документацию ROS2 для ARM64
```

### 2. Клонирование и сборка

```bash
# На Banana Pi
git clone https://github.com/hzname/dev-workspace.git ~/dev-workspace
cd ~/dev-workspace/robot_dog_ws

# Установка Python зависимостей
pip3 install -r requirements.txt

# Сборка (только Python пакеты)
colcon build --packages-select dog_bringup dog_description dog_control dog_hardware dog_sensors dog_teleop

# Source
source install/setup.bash
```

### 3. Установка Rust (опционально)

```bash
# Если нужны Rust ноды на Banana Pi
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

# Пересборка с Rust
colcon build --symlink-install
```

### 4. Запуск сервиса

```bash
# Создание systemd сервиса
sudo tee /etc/systemd/system/robot-dog.service > /dev/null <<EOF
[Unit]
Description=Robot Dog ROS2
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/dev-workspace/robot_dog_ws
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch dog_bringup robot.launch.py'
Restart=always

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable robot-dog
sudo systemctl start robot-dog
```

## 🧪 Тестирование

```bash
# Проверка всех нод
ros2 node list

# Просмотр топиков
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /imu/data

# Тест телеуправления
ros2 run dog_teleop teleop_keyboard
# Управление: WASD для движения, Q/E для поворота
```

## 🐛 Отладка

```bash
# Логи ROS2
ros2 run rqt_console rqt_console

# Проверка графа
ros2 run rqt_graph rqt_graph

# RViz
ros2 launch dog_bringup robot.launch.py use_rviz:=true
```

## 🔒 Исправленные проблемы

- ✅ shell=True заменён на shell=False
- ✅ Исправлены unwrap() в Rust коде
- ✅ Исправлены condition в launch файлах
- ✅ Добавлены недостающие зависимости
- ✅ Полная интеграция Rust пакетов

## 📄 Лицензия

MIT
