# RViz Integration for Rust Nodes

## Запуск на Banana Pi

```bash
# 1. Запуск Rust нод (если еще не запущены)
~/start_rust_nodes.sh

# 2. В Docker контейнере - ROS2 bridge
source /opt/ros/humble/setup.bash
source /robot_dog_ws/install/setup.bash
ros2 run dog_bringup rust_bridge.py

# Или полный launch
ros2 launch dog_bringup rust_robot.launch.py
```

## Запуск RViz на ПК

```bash
# Установить ROS_DOMAIN_ID как на Пи (обычно 0)
export ROS_DOMAIN_ID=0

# Запустить RViz
rviz2

# Или с конфигурацией
rviz2 -d ~/robot.rviz
```

## Настройка RViz

1. **Add → RobotModel**
   - Topic: `/robot_description`

2. **Add → TF**
   - Покажет все фреймы

3. **Add → By Topic → /joint_states**
   - JointState

4. **Add → By Topic → /imu/data**
   - IMU

## Топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/joint_states` | sensor_msgs/JointState | Положения 12 суставов |
| `/imu/data` | sensor_msgs/Imu | Ориентация корпуса |
| `/robot_description` | std_msgs/String | URDF модель |
| `/tf` | tf2_msgs/TFMessage | Трансформации |

## Архитектура

```
Banana Pi                    ПК (RViz)
┌─────────────────┐          ┌─────────────┐
│ Rust ноды       │          │ RViz2       │
│ ├─ gait         │          │ ├─ RobotModel
│ ├─ imu          │          │ ├─ TF       │
│ ├─ servo        │          │ └─ JointState│
│ └─ balance      │          └─────────────┘
│      │          │                ↑
│      ↓          │         (ros2 topic echo)
│ rust_bridge.py  │         ROS2 network
│ (ROS2 node)     │                ↑
│      │          │         ┌─────────────┐
└──────┼──────────┘         │ ros2_bridge │
       │                     │ (converts)  │
       └────────────────────→│ YAML → ROS2 │
              /joint_states  └─────────────┘
              /imu/data
```

## Проверка соединения

На Banana Pi:
```bash
ros2 topic list
ros2 topic echo /joint_states
```

На ПК:
```bash
ros2 topic list  # должен показывать топики с Пи
ros2 topic hz /joint_states  # частота публикации
```
