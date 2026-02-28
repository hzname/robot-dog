# Структура пакетов ROS2 Humble для робособаки

## Созданные пакеты

### 1. dog_description/ (ament_cmake)
Описание робота в URDF/XACRO:
- ✅ package.xml
- ✅ CMakeLists.txt
- ✅ urdf/dog.urdf.xacro - главный URDF с 4 ногами (FL, FR, BL, BR)
- ✅ urdf/leg.xacro - макрос ноги (hip, thigh, calf)
- ✅ urdf/materials.xacro - цвета материалов
- ✅ launch/display.launch.py - запуск RViz
- ✅ meshes/ (папка)

### 2. dog_control/ (ament_python)
Контроллер походок:
- ✅ package.xml
- ✅ setup.py
- ✅ dog_control/__init__.py
- ✅ dog_control/gait_controller.py - реализует trot, walk, pace, bound

### 3. dog_teleop/ (ament_python)
Управление с клавиатуры:
- ✅ package.xml
- ✅ setup.py
- ✅ dog_teleop/__init__.py
- ✅ dog_teleop/teleop_keyboard.py - управление с клавиатуры

### 4. dog_bringup/ (ament_cmake)
Запуск системы:
- ✅ package.xml
- ✅ CMakeLists.txt
- ✅ launch/robot.launch.py
- ✅ config/robot_config.yaml

## Суставы робота

Каждая нога имеет 3 сустава:
```
{FL|FR|BL|BR}_hip_joint    - вращение бедра (ось X)
{FL|FR|BL|BR}_thigh_joint  - бедро (ось Y)  
{FL|FR|BL|BR}_calf_joint   - голень (ось Y)
```

Всего: 12 сервоприводов

## Команды для запуска

```bash
# Сборка
cd ~/robot_dog_ws
colcon build --symlink-install
source install/setup.bash

# Запуск RViz
ros2 launch dog_description display.launch.py

# Запуск контроллера
ros2 run dog_control gait_controller

# Запуск телеопа
ros2 run dog_teleop teleop_keyboard

# Полный запуск
ros2 launch dog_bringup robot.launch.py
```
