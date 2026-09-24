# Gait Controller Documentation

## Trot Gait Implementation

Обновлённый `gait_controller.py` реализует trot походку для робота-собаки.

### Особенности реализации

#### 1. Диагональная пара (Trot Gait)
- **Пара 1**: FL (Front Left) + BR (Back Right) — фаза 0
- **Пара 2**: FR (Front Right) + BL (Back Left) — фаза π

#### 2. Параметры походки (в конфиге)
```yaml
gait:
  period: 1.0                # Период 1 сек (1 Hz)
  step_height: 0.04          # Высота подъёма 4 см (3-5 см)
  step_length: 0.06          # Шаг вперёд 6 см (5-8 см)
  trajectory_type: "triangle" # Траектория: треугольник или синусоида
```

#### 3. Компенсация высоты корпуса
- Корпус опускается при фазе swing для стабильности
- Коэффициент: `body_compensation: 0.02`

#### 4. Плавный переход
- Состояния: STOPPED → STARTING → RUNNING → STOPPING
- Длительность перехода: 0.5 сек
- `transition_factor`: 0.0 → 1.0

### Запуск

```bash
# Запуск контроллера
ros2 launch dog_bringup gait_controller.launch.py

# Старт походки
ros2 topic pub /gait/start_stop std_msgs/msg/Bool "data: true"

# Остановка походки
ros2 topic pub /gait/start_stop std_msgs/msg/Bool "data: false"

# Управление скоростью
ros2 topic pub /gait/command geometry_msgs/msg/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Тестирование

```bash
# Запуск тестового скрипта
ros2 run dog_control test_gait
```

### Топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/joint_trajectory_controller/joint_trajectory` | JointTrajectory | Команды для сервоприводов |
| `/cmd_vel` | Twist | Текущая скорость (для теста) |
| `/body_height_adjustment` | Float64 | Корректировка высоты корпуса |
| `/gait_active` | Bool | Статус походки |
| `/gait/start_stop` | Bool | Команда старт/стоп |
| `/gait/command` | Twist | Команда скорости |

### Структура кода

- `GaitConfig` — dataclass с параметрами походки
- `GaitState` — enum состояний (STOPPED, STARTING, RUNNING, STOPPING)
- `_calculate_trot_gait()` — расчёт trot походки
- `_calculate_foot_trajectory()` — траектория лапы (треугольник/синусоида)
- `_foot_to_joint_angles()` — упрощённая обратная кинематика
