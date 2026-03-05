# Robot Dog ROS2 Workspace 🤖🐕

Квадрупедальный робот на базе ROS2 с C++ нодами для реального времени управления.

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Build Status](https://github.com/hzname/robot-dog/actions/workflows/ci-cpp.yml/badge.svg)](https://github.com/hzname/robot-dog/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## 🎯 Возможности

- **C++ ноды** — высокая производительность и детерминизм
- **Lifecycle management** — безопасный запуск/остановка
- **Симуляция** — тестирование без железа
- **Safety first** — watchdog, emergency stop, position limits
- **Teleoperation** — клавиатура, джойстик, UDP
- **Web interface** — в разработке (см. ROADMAP)

## 📁 Структура

```
robot_dog_ws/
├── src/
│   ├── dog_sensors_cpp/      # C++: IMU node (MPU6050 + simulator)
│   ├── dog_control_cpp/       # C++: Gait + Balance controllers
│   ├── dog_teleop_cpp/        # C++: Keyboard/Joystick/UDP teleop
│   ├── dog_hardware_cpp/      # C++: Servo driver (PCA9685)
│   ├── dog_description/       # URDF модель робота
│   ├── dog_bringup/           # Launch файлы
│   └── [deprecated]/          # Python и Rust пакеты (устарели)
├── Dockerfile                 # Docker образ для сборки
├── build-and-test.sh          # Скрипт сборки и тестирования
└── ROADMAP.md                 # План разработки
```

## 🚀 Быстрый старт

### Требования

- Docker (рекомендуется)
- или ROS2 Jazzy (Ubuntu 24.04)
- colcon

### Сборка через Docker (рекомендуется)

```bash
git clone https://github.com/hzname/robot-dog.git
cd robot-dog

# Полная сборка и тесты
./build-and-test.sh

# Или только сборка
docker build -t robot-dog-builder .
docker run --rm -v $(pwd)/robot_dog_ws:/workspace robot-dog-builder
```

### Локальная сборка (без Docker)

```bash
cd robot-dog/robot_dog_ws
source /opt/ros/jazzy/setup.bash

# Сборка C++ пакетов
colcon build --symlink-install --packages-select \
  dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp

# Source окружения
source install/setup.bash
```

## 🎮 Запуск

### Симуляция (без железа)

```bash
# Все ноды
ros2 launch dog_bringup robot_dog.launch.py use_hardware:=false

# Или отдельно
ros2 run dog_sensors_cpp imu_node --ros-args -p simulate:=true
ros2 run dog_control_cpp gait_controller
ros2 run dog_teleop_cpp keyboard_teleop
```

### На реальном железе (Banana Pi)

```bash
ros2 launch dog_bringup robot_dog.launch.py use_hardware:=true
```

## 🕹️ Управление

### Keyboard Teleop

Запустите `keyboard_teleop` и используйте:

```
w/s — вперед/назад
a/d — влево/вправо
q/e — поворот
Space — стоп
+/- — высота корпуса
ESC — emergency stop
r — сброс emergency stop
```

### Топики

- `/cmd_vel` — команды скорости (geometry_msgs/Twist)
- `/imu/data` — данные IMU (sensor_msgs/Imu)
- `/joint_trajectory` — траектории суставов
- `/gait_state` — состояние походки
- `/emergency_stop` — emergency stop trigger

## 🧪 Тестирование

```bash
# В Docker
./test-docker.sh

# Локально
colcon test --packages-select dog_sensors_cpp dog_control_cpp
colcon test-result --verbose
```

## 🏗️ Архитектура

```
┌─────────────────────────────────────────┐
│           Teleop (Keyboard/UDP)         │
└─────────────┬───────────────────────────┘
              │ /cmd_vel
┌─────────────▼───────────────────────────┐
│        Gait Controller (C++)            │
│    - Trot gait with phase offsets       │
│    - Balance compensation (PID)         │
└─────────────┬───────────────────────────┘
              │ /joint_trajectory
┌─────────────▼───────────────────────────┐
│       Servo Driver (C++)                │
│    - PCA9685 I2C PWM                    │
│    - Safety: limits, watchdog, e-stop   │
└─────────────┬───────────────────────────┘
              │ I2C/UART
┌─────────────▼───────────────────────────┐
│         Hardware (12 servos)            │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│       IMU Node (C++)                    │
│    - MPU6050 driver                     │
│    - Mahony sensor fusion               │
│    - Simulation mode                    │
└─────────────────────────────────────────┘
```

## 🔧 Разработка

### Добавление нового пакета

```bash
cd robot_dog_ws/src
ros2 pkg create --build-type ament_cmake my_package_cpp --dependencies rclcpp
```

### Code style

Проект использует:
- `clang-format` — форматирование C++
- `clang-tidy` — статический анализ
- `cpplint` — проверка стиля Google

```bash
# Проверка форматирования
find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format --dry-run --Werror
```

## 📋 Статус разработки

| Компонент | Статус |
|-----------|--------|
| C++ пакеты | ✅ Готово |
| Сборка (Docker) | ✅ Готово |
| Unit tests | ✅ 56/66 passed |
| Симуляция | ✅ Работает |
| Веб-интерфейс | 🚧 В разработке |
| Тесты на железе | ⏳ Запланировано |

См. [ROADMAP.md](ROADMAP.md) для подробного плана.

## 🆘 Deprecated пакеты

Следующие пакеты устарели и будут удалены:
- `dog_control/` — Python контроллер
- `dog_sensors/` — Python драйверы
- `dog_hardware/` — Python интерфейс
- `dog_teleop/` — Python телеоперация
- `*_rust/` — Rust пакеты (заменены на C++)

Используйте `*_cpp` пакеты вместо них.

## 📄 Лицензия

MIT License — см. [LICENSE](LICENSE)

## 🤝 Contributing

См. [CONTRIBUTING.md](CONTRIBUTING.md)

---

**Вопросы?** Создайте issue на GitHub или пишите в Telegram.
