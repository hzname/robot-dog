# dog_teleop

Пакет телеоперации для робособаки. Поддерживает управление с клавиатуры, геймпада и по UDP.

## Установка

```bash
cd ~/robot_dog_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select dog_teleop
source install/setup.bash
```

Для работы с геймпадом установите пакет joy:
```bash
sudo apt install ros-$ROS_DISTRO-joy
```

Для UDP клиента на ПК установите pynput:
```bash
pip install pynput
```

## Использование

### Управление с клавиатуры

```bash
ros2 launch dog_teleop teleop_keyboard.launch.py
```

**Управление:**
- `W/S` — движение вперёд/назад
- `A/D` — движение влево/вправо (страф)
- `Q/E` — поворот влево/вправо
- `Пробел` — экстренная остановка
- `1/2/3` — позы: стоять/сидеть/лежать
- `G` — включить/выключить trot походку
- `+/-` — увеличить/уменьшить скорость
- `H` — помощь
- `Ctrl+C` — выход

### Управление с геймпада (Xbox/PS4)

```bash
ros2 launch dog_teleop teleop_joystick.launch.py
```

**Управление:**
- **Левый стик** — движение вперёд/назад/влево/вправо
- **Правый стик (влево/вправо)** — поворот
- **A/✕** — поза "стоять"
- **B/○** — поза "сидеть"
- **X/□** — поза "лежать"
- **LB/L1** — уменьшить скорость
- **RB/R1** — увеличить скорость
- **Start/Options** — включить/выключить походку
- **Back/Share** — экстренная остановка

### Удалённое управление по UDP

На роботе (сервер):
```bash
ros2 launch dog_teleop udp_server.launch.py port:=8888
```

На ПК (клиент):
```bash
python3 scripts/udp_client.py --host 192.168.1.100 --port 8888
```

Протокол UDP — JSON сообщения:
```json
{"type": "velocity", "linear_x": 0.5, "linear_y": 0.0, "angular_z": 0.0}
{"type": "pose", "pose": "stand"}
{"type": "gait", "active": true}
{"type": "speed", "linear": 0.8, "angular": 0.8}
{"type": "stop"}
```

### Все методы одновременно

```bash
ros2 launch dog_teleop teleop_all.launch.py
```

Запускает клавиатуру, джойстик и UDP сервер одновременно.

## Топики

### Публикуемые
- `/cmd_vel` (geometry_msgs/Twist) — команды скорости
- `/gait/start_stop` (std_msgs/Bool) — старт/стоп походки
- `/body_pose` (std_msgs/String) — команда позы

### Подписываемые
- `/joy` (sensor_msgs/Joy) — данные с геймпада

## Параметры

### teleop_keyboard
- `linear_speed` (float, 0.5) — линейная скорость (0.1-1.0)
- `angular_speed` (float, 0.8) — угловая скорость (0.1-1.0)
- `smoothing_factor` (float, 0.2) — плавность изменения скорости
- `update_rate` (float, 20.0) — частота публикации (Hz)

### teleop_joystick
- `controller_type` (string, "auto") — тип контроллера: auto, xbox, ps4, generic
- `linear_speed` (float, 0.5) — линейная скорость
- `angular_speed` (float, 0.8) — угловая скорость
- `deadzone` (float, 0.15) — мёртвая зона стиков
- `axis_linear_x` (int, 1) — ось движения вперёд/назад
- `axis_linear_y` (int, 0) — ось стрейфа
- `axis_angular_z` (int, 3) — ось поворота
- `invert_*` (bool) — инверсия осей

### udp_server
- `port` (int, 8888) — UDP порт
- `max_clients` (int, 4) — максимум клиентов
- `timeout_sec` (float, 2.0) — таймаут клиента
- `linear_speed` (float, 0.5) — линейная скорость
- `angular_speed` (float, 0.8) — угловая скорость

## Архитектура

```
┌─────────────────┐     ┌─────────────┐     ┌─────────────┐
│  Keyboard       │     │  Joystick   │     │  UDP Client │
│  (on robot)     │     │  (joy_node) │     │  (remote PC)│
└────────┬────────┘     └──────┬──────┘     └──────┬──────┘
         │                     │                    │
         │                     │                    │ UDP
         ▼                     ▼                    ▼
┌─────────────────────────────────────────────────────────┐
│                    dog_teleop nodes                     │
│  ┌────────────────┐ ┌──────────────┐ ┌──────────────┐   │
│  │teleop_keyboard │ │teleop_joystick│ │ udp_server  │   │
│  └────────┬───────┘ └───────┬──────┘ └──────┬───────┘   │
│           │                 │               │           │
│           ▼                 ▼               ▼           │
│         ┌─────────────────────────────────────┐         │
│         │           /cmd_vel                  │         │
│         │        /gait/start_stop             │         │
│         │         /body_pose                  │         │
│         └─────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌─────────────────────┐
              │   gait_controller   │
              │   (dog_control)     │
              └─────────────────────┘
```

## Лицензия

MIT
