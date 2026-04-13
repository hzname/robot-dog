# RobotDogQwen — План запуска и настройки

**Дата:** 2026-04-07  
**Цель:** Запустить робота, установить RViz, настроить связь

---

## 📊 Текущий статус

### ✅ Выполнено
- [x] Миграция Python → C++ (rclcpp)
- [x] PCA9685 driver: исправлен порядок байт (MODE1=0x21, ON_L,ON_H,OFF_L,OFF_H)
- [x] Docker образ собран (ros:jazzy base)
- [x] Тестовые скрипты: walk_final.cpp, test_all_servos_final.py
- [x] Git push с тестами и фиксами

### ❌ Текущие проблемы
1. **Часы системы:** 2026 год → apt reject'ит репозитории
2. **ROS версия:** Workspace собран под Humble, запускаем под Jazzy — несовместимость
3. **Gait controller:** `Package 'dog_control_cpp' not found` — нужен пересбор

---

## 🎯 План действий

### Шаг 1: Исправить системное время
**Приоритет:** 🔴 P0 (блокирует apt, docker build)

```bash
# На robot-dog (192.168.31.165)
sudo timedatectl set-ntp true
# Или вручную:
sudo date -s "2025-04-07 21:00:00"
```

**Проверка:**
```bash
date
timedatectl status
```

---

### Шаг 2: Пересобрать workspace под ROS2 Jazzy
**Приоритет:** 🔴 P0 (блокирует gait_controller)

```bash
# На robot-dog
cd ~/robot-dog

# Очистить старую сборку (Humble)
rm -rf robot_dog_ws/build robot_dog_ws/install robot_dog_ws/log

# Собрать в Docker (ros:jazzy уже скачан)
docker run --rm -v ~/robot-dog/robot_dog_ws:/workspace ros:jazzy bash -c "\
  source /opt/ros/jazzy/setup.bash && \
  cd /workspace && \
  colcon build --symlink-install --packages-select \
    dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp \
  2>&1 | tee build.log"
```

**Проверка:**
```bash
ls ~/robot-dog/robot_dog_ws/install/setup.bash
# Должен содержать: COLCON_CURRENT_PREFIX="/opt/ros/jazzy"
```

---

### Шаг 3: Запустить Docker контейнер с роботом
**Приоритет:** 🔴 P0

```bash
# Остановить старый контейнер (если есть)
docker stop robot_dog 2>/dev/null; docker rm robot_dog 2>/dev/null

# Запустить с монтированием workspace
docker run -d --name robot_dog \
  --privileged \
  --device /dev/i2c-0 \
  -v /dev/i2c-0:/dev/i2c-0 \
  -v /home/sg/robot-dog/robot_dog_ws:/workspace \
  --network host \
  ros:jazzy \
  /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source /workspace/install/setup.bash && \
    echo '=== STARTING SERVO DRIVER ===' && \
    ros2 run dog_hardware_cpp servo_driver_node --ros-args -p bus_type:=i2c -p device_port:=/dev/i2c-0 & \
    sleep 3 && \
    ros2 lifecycle set /servo_driver_node configure && \
    ros2 lifecycle set /servo_driver_node activate && \
    sleep 2 && \
    echo '=== STARTING GAIT CONTROLLER ===' && \
    ros2 run dog_control_cpp gait_controller & \
    echo '=== SYSTEM READY ===' && \
    tail -f /dev/null"
```

**Проверка:**
```bash
docker logs robot_dog --tail 30
# Должно быть "=== SYSTEM READY ===" без ошибок
```

---

### Шаг 4: Установить RViz2 на robot-dog
**Приоритет:** 🟡 P1

```bash
# На robot-dog (не в Docker, на хосте)
sudo apt update
sudo apt install -y ros-jazzy-rviz2

# Или если нужен полный desktop:
# sudo apt install -y ros-jazzy-desktop
```

**Проверка:**
```bash
rviz2 --version
```

---

### Шаг 5: Настроить связь с RViz
**Приоритет:** 🟡 P1

#### Вариант A: RViz на robot-dog (если есть дисплей/X11)

```bash
# На robot-dog
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
rviz2
```

#### Вариант B: RViz на удалённой машине (рекомендуется)

**На robot-dog (контейнер):**
```bash
# ROS_DOMAIN_ID должен совпадать
export ROS_DOMAIN_ID=0
```

**На рабочей станции (где RViz):**
```bash
# Установить ROS2 Jazzy (или Humble)
# Настроить multicast/DDS discovery

# Вариант 1: Прямое подключение по IP
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Добавить robot-dog в known hosts
export CYCLONEDDS_URI='<CycloneDDS><Discovery><Peers><Peer address="192.168.31.165"/></Peers></Discovery></CycloneDDS>'

rviz2
```

#### Вариант C: SSH X11 Forwarding (простой)

**На рабочей станции:**
```bash
ssh -X sg@192.168.31.165
source /opt/ros/jazzy/setup.bash
rviz2
```

---

### Шаг 6: Настроить RViz для robot-dog
**Приоритет:** 🟡 P1

**Добавить displays:**
1. **RobotModel** — URDF визуализация
   - Description topic: `/robot_description`
   - TF prefix: (пусто)

2. **TF** — система координат
   - Show Arrows: true
   - Show Axes: true

3. **Grid** — референсная плоскость
   - Reference Frame: `base_link`

**Добавить темы:**
- `/joint_states` — JointState (состояние суставов)
- `/imu/data` — IMU данные

**Сохранить конфиг:**
```bash
# В RViz: File → Save Config As
# ~/robot-dog/robot_dog_ws/src/dog_bringup/rviz/robot_dog.rviz
```

---

### Шаг 7: Создать launch файл для полного запуска
**Приоритет:** 🟢 P2

```python
# ~/robot-dog/robot_dog_ws/src/dog_bringup/launch/robot_dog_full.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Servo driver
        Node(
            package='dog_hardware_cpp',
            executable='servo_driver_node',
            name='servo_driver_node',
            parameters=[{
                'bus_type': 'i2c',
                'device_port': '/dev/i2c-0'
            }],
            output='screen'
        ),
        
        # Gait controller
        Node(
            package='dog_control_cpp',
            executable='gait_controller',
            name='gait_controller',
            output='screen'
        ),
        
        # Robot state publisher (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(os.path.join(
                    get_package_share_directory('dog_description'),
                    'urdf', 'robot_dog.urdf'
                )).read()
            }]
        ),
    ])
```

---

## 🔧 Полезные команды

### Диагностика ROS2

```bash
# Список топиков
ros2 topic list

# Частота топика
ros2 topic hz /joint_states

# Содержимое топика
ros2 topic echo /joint_states

# Список нод
ros2 node list

# Информация о ноде
ros2 node info /servo_driver_node

# TF дерево
ros2 run tf2_tools view_frames
```

### Диагностика Docker

```bash
# Логи
docker logs robot_dog --tail 50 -f

# Зайти в контейнер
docker exec -it robot_dog bash

# Статус контейнера
docker inspect robot_dog --format '{{.State.Status}}'
```

### Тестирование серв

```bash
# Python тест
python3 ~/robot-dog/robot_dog_ws/scripts/test_all_servos_final.py

# C++ тест
~/robot-dog/robot_dog_ws/scripts/walk_final
```

---

## 📋 Чек-лист готовности

- [ ] Системное время корректно
- [ ] Workspace пересобран под Jazzy
- [ ] Docker контейнер запущен без ошибок
- [ ] `ros2 topic list` показывает `/joint_states`
- [ ] `ros2 node list` показывает `/servo_driver_node`, `/gait_controller`
- [ ] RViz установлен и запускается
- [ ] RobotModel отображается в RViz
- [ ] TF дерево корректно (base_link → leg_link)

---

## ⚠️ Известные проблемы

1. **Watchdog timeout** — нормально при первом запуске, gait_controller должен начать отправлять команды
2. **Failed to enable servo** — проверить питание серв (должно быть 5-6V отдельно от логики)
3. **RViz не видит топики** — проверить `ROS_DOMAIN_ID` и сеть (multicast)

---

*План составлен: 2026-04-07*
