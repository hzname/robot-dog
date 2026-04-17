# RobotDogQwen 🤖🐕

Квадрупедальный робот на базе ROS2 Jazzy с C++ нодами реального времени, веб-интерфейсом и мониторингом здоровья.

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## 🎯 Возможности

- **C++ ноды** — 100 Hz контроль с детерминизмом
- **Lifecycle Nodes** — безопасный запуск/остановка с auto-activate
- **Health Monitor** — автоматический мониторинг и рестарт нод при падении
- **Graceful Degradation** — баланс-контроллер отключается при отказе IMU
- **Real-time** — CPU pinning + SCHED_FIFO для контрольных циклов
- **Safety** — watchdog, emergency stop, joint limits, rate limiting
- **Web Dashboard** — IMU orientation, суставы, джойстик, калибровка, E-Stop
- **IMU (MPU6050)** — Mahony sensor fusion, ориентация в реальном времени
- **Servo Driver (PCA9685)** — 12 сервоприводов через I2C
- **Топик-неймспейс** — `/dog/...` префикс для изоляции

## 📁 Структура

```
├── docker-compose.yml          # Production деплой (Banana Pi)
├── ros_bridge.py               # Unix socket bridge ROS2 ↔ Web
├── web_server_host.py          # FastAPI + WebSocket сервер (:8080)
├── calibrate_web.py            # Веб-калибровка сервоприводов (:8081)
├── servo_config.json           # Единый конфиг: zero, sign, coupling, ranges
│
└── robot_dog_ws/
    ├── Dockerfile              # ROS2 Jazzy build образ
    ├── docker-compose.yml      # Dev docker-compose
    └── src/
        ├── dog_sensors_cpp/     # IMU node: MPU6050 + Mahony filter + simulator
        ├── dog_control_cpp/     # Gait controller + Balance controller (PID)
        ├── dog_teleop_cpp/      # Keyboard + Gamepad + UDP teleop
        ├── dog_hardware_cpp/    # Servo driver: PCA9685 + Simulation
        ├── dog_monitor/         # Health monitor: node liveness + auto-restart
        ├── dog_web/             # Web UI: dashboard, control, state, calibration
        └── dog_description/     # URDF модель робота
```

## 🚀 Быстрый старт

### Требования

- **Banana Pi BPI-M4-Zero** (или любой SBC с I2C)
- **Armbian** (Debian 12 bookworm)
- **Docker** с образом `ros:jazzy`
- MPU6050 IMU + PCA9685 servo driver (подключены к `/dev/i2c-0`)

### Деплой на Banana Pi

```bash
git clone https://github.com/hzname/robot-dog.git
cd robot-dog
docker compose up -d
```

Система запустит:
1. **IMU node** — MPU6050 с калибровкой гироскопа
2. **Servo driver** — PCA9685 I2C, 12 серво, watchdog 10s
3. **Gait controller** — trot походка с баланс-коррекцией
4. **Health monitor** — мониторинг 4 нод, auto-restart
5. **ROS bridge** — Unix socket bridge для web-сервера

### Веб-интерфейс

Откройте `http://<BANANA_PI_IP>:8080`

| Страница | Что показывает |
|----------|---------------|
| **📊 Панель** | IMU (Roll/Pitch/Yaw), суставы, быстрые действия, E-Stop |
| **🎮 Управление** | Виртуальный джойстик, D-pad, скорость |
| **📈 Состояние** | Таблица всех 12 суставов, частота обновлений |
| **🔧 Калибровка** | Веб-утилита калибровки сервоприводов (порт 8081) |

Статус-бар на каждой странице:
- 🟢 WebSocket / ROS2 / Сервы — индикаторы состояния
- 🟢 E-Stop — горит когда всё ОК, 🔴 при аварийной остановке
- 🛑 Стоп / 🔓 Снять — кнопки аварийной блокировки

## 🔧 Калибровка сервоприводов

Вынесена из ROS2 в отдельный веб-сервис. Работает напрямую с I2C, останавливая ROS2 контейнер.

### Запуск

```bash
# На Banana Pi (нужен sudo для I2C)
nohup sudo python3 ~/robot-dog/calibrate_web.py >/tmp/cal_web.log 2>&1 &

# Открыть в браузере
http://<BANANA_PI_IP>:8081
```

### Конфигурация: `servo_config.json`

Единый файл конфигурации для 12 суставов + 4 aux серво:

| Поле | Описание |
|------|----------|
| `sign` | +1 или -1 — инверсия направления |
| `zero_deg` | Смещение нуля (градусы) |
| `min_deg` / `max_deg` | Пределы движения |
| `coupled_to` | Родительский сустав (для кинематики Kangal) |
| `coupling_coeff` | Коэффициент связи с родителем |
| `enabled` | false = ROS2 и калибровка игнорируют |

### Процесс калибровки

1. **▶ Start** — остановить ROS2, захватить PCA9685 (сервы не двигаются)
2. Двигать **слайдером** до нужной позиции
3. **↔ Inv** — если серва двигается в обратную сторону
4. **⊙ Zero** — текущее положение = новый ноль
5. Повторить для каждого сустава
6. **💾 Save** — записать конфиг на робота
7. **■ Stop** — запустить ROS2 обратно

### Позы (управление лапами, не суставами)

| Поза | Описание |
|------|----------|
| Reference (ноль) | Все лапы в 0° |
| На цыпочках (перед/зад/все) | Лапы вверх |
| Наклон вперёд | Перед назад, зад вперёд |
| Сесть на задние | Зад согнуты |
| Stand / Присед | Максимально высоко / низко |
| Передние вперёд / Задние назад | По парам |

### Безопасность

- Скорость: **9°/с** при плавном движении
- Первая команда к серве — телепорт (без рывка)
- Movement thread автоматически перезапускается при падении
- PCA9685 не ресетится при старте — сервы стоят как стояли

### API Endpoints

```
POST /api/cal/start         # Начать калибровку (остановить ROS2)
POST /api/cal/stop          # Закончить (запустить ROS2)
GET  /api/cal/status        # Статус + текущие позиции
GET  /api/cal/config        # Чтение servo_config.json
POST /api/cal/move          # Двинуть сустав {joint_name, angle_deg}
POST /api/cal/move_all_hips # Двинуть все hip {angle_deg}
POST /api/cal/pose/{name}   # Перейти в позу
POST /api/cal/set_zero      # Текущая позиция = ноль
POST /api/cal/reapply/{j}   # Пере-применить позицию (после Inv)
POST /api/cal/disable_all   # Отключить PWM всех серво
```

## 🎮 Управление через API

```bash
# Движение вперёд
curl -X POST http://<IP>:8080/api/control/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{"linear_x": 0.3}'

# Стоп
curl -X POST http://<IP>:8080/api/control/stop

# Включить/выключить сервы
curl -X POST http://<IP>:8080/api/control/servo_enable \
  -H "Content-Type: application/json" \
  -d '{"enable": true}'

# Emergency stop
curl -X POST http://<IP>:8080/api/control/emergency_stop \
  -H "Content-Type: application/json" \
  -d '{"stop": true}'

# Получить состояние
curl http://<IP>:8080/api/state
```

## 🕹️ Keyboard Teleop (локально)

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 run dog_teleop_cpp keyboard_teleop
```

| Клавиша | Действие |
|---------|----------|
| `w` / `s` | Вперёд / Назад |
| `a` / `d` | Влево / Вправо |
| `q` / `e` | Поворот |
| `Space` | Стоп |
| `+` / `-` | Высота корпуса |
| `ESC` | Emergency stop |
| `r` | Сброс E-Stop |

## 🏗️ Архитектура

```
┌──────────────────────────────────────────────────────┐
│  Banana Pi Host                                      │
│                                                      │
│  ┌──────────────┐   WebSocket/REST   ┌─────────────┐ │
│  │   Browser    │ ◄────────────────► │  FastAPI    │ │
│  └──────────────┘                    │  :8080      │ │
│                                      └──────┬──────┘ │
│                                             │        │
│                              Unix socket    │        │
│                              (length-prefix)│        │
│                                      ┌──────▼──────┐ │
│                                      │  ROS Bridge │ │
│                                      │  (Python)   │ │
│                                      └──────┬──────┘ │
│                                             │        │
│  ┌──────────────┐   REST    ┌─────────────┐ │        │
│  │   Browser    │ ◄────────►│ Calibrate   │ │        │
│  │  (calibrate) │           │  :8081      │ │        │
│  └──────────────┘           │  I2C direct │ │        │
│                             └─────────────┘ │        │
└─────────────────────────────────────────────┼────────┘
                                              │
              ┌───────────────────────────────┼─────────────────┐
              │  Docker: ROS2 Jazzy           │                 │
              │                               │                 │
              │  ┌───────────┐  /dog/cmd_vel  │                 │
              │  │  Gait     │ ──────────────►│                 │
              │  │  Ctrl     │                │                 │
              │  └─────┬─────┘                │                 │
              │        │ /dog/joint_trajectory│                 │
              │        ▼                      │                 │
              │  ┌───────────┐   I2C           │                 │
              │  │  Servo    │ ──────────────► /dev/i2c-0       │
              │  │  Driver   │   (PCA9685)     │                 │
              │  └─────▲─────┘                │                 │
              │        │                      │                 │
              │  ┌─────┴─────┐   I2C          │                 │
              │  │  IMU      │ ◄───────────── /dev/i2c-0       │
              │  │  (MPU6050)│                │                 │
              │  └───────────┘                │                 │
              │                               │                 │
              │  ┌───────────┐                │                 │
              │  │  Health   │ → мониторит все│                 │
              │  │  Monitor  │ → auto-restart │                 │
              │  └───────────┘                │                 │
              └───────────────────────────────┘
```

## 🔧 Топики

| Топик | Тип | Направление | Описание |
|-------|-----|-------------|----------|
| `/dog/cmd_vel` | geometry_msgs/Twist | ← | Команды скорости |
| `/dog/joint_states` | sensor_msgs/JointState | → | Позиции 12 суставов |
| `/dog/joint_trajectory` | trajectory_msgs/JointTrajectory | → | Траектории от gait ctrl |
| `/dog/imu/data` | sensor_msgs/Imu | → | Данные MPU6050 + Mahony |
| `/dog/emergency_stop` | std_msgs/Bool | ↔ | Состояние E-Stop |
| `/dog/gait_state` | std_msgs/Float64MultiArray | → | Фаза, шаг, скорость |
| `/dog/health` | std_msgs/Bool | → | Health monitor |
| `/dog/diagnostics` | diagnostic_msgs/DiagnosticArray | → | Диагностика нод |

## 🛡️ Safety

| Механизм | Описание |
|----------|----------|
| **Watchdog** | 10s таймаут → emergency stop |
| **Joint Limits** | Min/max позиция для каждого сустава |
| **Rate Limiting** | Max delta per cycle (0.15 rad) |
| **Interpolation** | 50ms сглаживание между командами |
| **E-Stop** | Мгновенная остановка из любого источника |
| **Graceful Degradation** | Balance ctrl отключается при stale IMU |
| **Health Monitor** | Auto-restart до 3 раз при падении ноды |

## 🧪 Сборка и тесты

```bash
# Внутри контейнера
docker exec robot_dog bash -c "
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --packages-select \
    dog_sensors_cpp dog_control_cpp dog_teleop_cpp dog_hardware_cpp dog_monitor_cpp
"

# Проверка нод
docker exec robot_dog bash -c "
  source /opt/ros/jazzy/setup.bash
  ros2 node list
  ros2 topic list
"
```

## 📋 Статус компонентов

| Компонент | Статус |
|-----------|--------|
| dog_sensors_cpp (IMU + Mahony) | ✅ Production |
| dog_hardware_cpp (PCA9685) | ✅ Production |
| dog_control_cpp (Gait + Balance) | ✅ Production |
| dog_teleop_cpp (Keyboard/Gamepad/UDP) | ✅ Production |
| dog_monitor_cpp (Health Monitor) | ✅ Production |
| docker-compose.yml | ✅ Production |
| ros_bridge.py + web_server_host.py | ✅ Production |
| calibrate_web.py (:8081) | ✅ Работает |
| servo_config.json | ✅ Единый конфиг |
| Web Dashboard | ✅ Production |
| Тесты на реальном железе | ✅ Работает |
| servo_controller чтение servo_config.json | 🔲 TODO |
| C++ coupling компенсация из конфига | 🔲 TODO |

## 📄 Лицензия

MIT License — см. [LICENSE](LICENSE)

## 🤝 Contributing

Pull requests welcome. Перед PR:
1. `clang-format` для всех C++ файлов
2. `colcon build` без ошибок
3. Тесты проходят

---

**Вопросы?** Создайте [issue](https://github.com/hzname/robot-dog/issues) или пишите в Telegram.
