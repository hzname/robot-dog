# Robot Dog Workspace

Этот репозиторий содержит проект квадрупедального робота на базе ROS2 Humble.

## Быстрый старт

```bash
# Сборка и запуск в Docker
docker-compose up -d
docker exec -it robot_dog bash

# Сборка проекта
colcon build --symlink-install
source install/setup.bash

# Запуск
ros2 launch dog_bringup robot_dog.launch.py
```

## Структура

- `src/dog_description/` - URDF модель робота
- `src/dog_control/` - Контроллеры движения
- `src/dog_teleop/` - Телеоперация
- `src/dog_bringup/` - Launch файлы

## Документация

См. папку `docs/` для подробной документации.
