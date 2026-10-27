# Phase 2 — Configuration Sync (PLAN.md) — NO CONVERSION

**Project:** RobotDogQwen
**Phase:** 2 — Configuration Sync
**Created:** 2026-05-12
**Status:** Planning
**Goal:** Все ROS2 ноды читают servo_config.json напрямую. Нет конвертации, нет YAML. Servo_config.json — единственный источник правды.

---

## 📋 Requirements Traceability

| Req | Description | Status |
|-----|-------------|--------|
| CONFIG-01 | Python ноды читают servo_config.json напрямую | Planned |
| CONFIG-02 | Интеграция с calibrate_web.py (auto-reload при save) | Planned |
| CONFIG-03 | C++ ноды читают servo_config.json напрямую | Planned |

---

## 📊 Current State Analysis

### ServoConfigReader (shared utility) — НЕТ (создать)
Новый общий модуль для парсинга servo_config.json. Используют и Python, и C++ ноды.

### Python servo_driver_node.py
- **Сейчас:** читает YAML из `config_file` параметра, fallback на хардкод 12 сервов
- **Стало:** читает servo_config.json напрямую через ServoConfigReader
- Поля: `joints.list` (servo_id, name, inverted), `joints.{type}` (limits в rad)
- Конвертация rad→deg происходит ТОЛЬКО в Python для angle_to_pulse()

### C++ gait_controller.cpp
- **Сейчас:** Phase 1 параметризовал всё через ROS2 params. Конструктор имеет хардкод-дефолты.
- **Стало:** ServoConfigReader читает JSON напрямую. ROS2 params убраны.
- Поля: `body.link_lengths_m`, `body.standing._computed`, `body.standing.foot_x/y_scale`, `gait.*`

### C++ balance_controller.cpp
- **Сейчас:** Phase 1 параметризовал PID и height limits через ROS2 params.
- **Стало:** ServoConfigReader читает JSON напрямую. ROS2 params убраны.
- Поля: `balance.pid.*`, `body.standing._computed.min/max_height_m`

### Calibrate Web
- **Сейчас:** POST /api/cal/config/save сохраняет в servo_config.json
- **Стало:** После сохранения — сигнализирует нодам об изменении файла

---

## 📐 Architecture

```
servo_config.json ← calibrate_web.py (save)
        │
        ├──► ServoConfigReader (Python) ──► servo_driver_node.py
        │
        └──► ServoConfigReader (C++) ────► gait_controller.cpp
                                           balance_controller.cpp
                                           servo_driver_node.cpp (если есть)
```

**Ключевое решение:** Каждая нода сама читает servo_config.json при старте.
При изменении файла — mtime watchdog вызывает reload() без перезапуска ноды.

**Launch файлы:** только передают `config_path` параметр (или хардкодят путь).
НИКАКИХ ROS2 params для конфига — это ломает direct-reading подход.

---

## 📝 Sub-tasks

### Task 2.1: ServoConfigReader (Python)
**Файл:** `robot_dog_ws/src/dog_hardware/dog_hardware/servo_config_reader.py`

Функции:
- `load_servo_config(path='/home/sg/robot-dog/servo_config.json')` → dict
- `get_joint_config(name)` → {channel, inverted, min_rad, max_rad, type, servo_id}
- `get_all_joints()` → list of all 12 joint configs
- `get_gait_params()` → {period_s, step_height_m, foot_x/y_scale, turn_gain, cog_pitch/y_gain}
- `get_balance_params()` → {pid: {roll: {kp,ki,kd}, ...}, max_correction: {roll, pitch, height}}
- `get_standing_params()` → {standing_height_m, min/max_height_m, hip_length, thigh_length, calf_length}
- `get_servo_driver_params()` → {i2c_bus, i2c_address, pwm_frequency, zero_offsets_file, joint_limits_file}
- `get_teleop_params()` → {keyboard_deadzone, gamepad_deadzone, step_keyboard_m, step_gamepad_m}

```python
import json
import os

_SERVO_CONFIG_PATH = '/home/sg/robot-dog/servo_config.json'
_last_mtime = 0.0

def load_servo_config(path=None):
    """Load servo_config.json and return dict."""
    if path is None:
        path = _SERVO_CONFIG_PATH
    with open(path, 'r') as f:
        return json.load(f)

def get_servo_config_mtime(path=None):
    """Get file modification time for watchdog."""
    if path is None:
        path = _SERVO_CONFIG_PATH
    return os.path.getmtime(path)

def is_config_changed(path=None):
    """Check if config file was modified since last read."""
    global _last_mtime
    current = get_servo_config_mtime(path)
    changed = current > _last_mtime
    if changed:
        _last_mtime = current
    return changed

# Helper functions that extract specific data
def get_joint_config(name):
    cfg = load_servo_config()
    for j in cfg['joints']['list']:
        if j['name'] == name:
            return j
    return None

def get_all_joints():
    cfg = load_servo_config()
    result = []
    for j in cfg['joints']['list']:
        type_cfg = cfg['joints'][j['type']]
        result.append({
            'name': j['name'],
            'leg': j['leg'],
            'type': j['type'],
            'channel': j['servo_id'],
            'inverted': j['inverted'],
            'min_rad': type_cfg['min_rad'],
            'max_rad': type_cfg['max_rad'],
            'neutral_rad': type_cfg['neutral_rad'],
            'min_pulse_ms': type_cfg['pwm_min_ms'],
            'max_pulse_ms': type_cfg['pwm_max_ms'],
        })
    return result
```

### Task 2.2: Переписать servo_driver_node.py (Python)
**Файл:** `robot_dog_ws/src/dog_hardware/dog_hardware/servo_driver_node.py`

Изменения:
1. Убрать `import yaml`
2. Добавить `from servo_config_reader import load_servo_config, get_all_joints, get_servo_config_mtime`
3. Переписать `_load_config()` на чтение JSON через `get_all_joints()`
4. Добавить watchdog timer (1s) для mtime-based auto-reload
5. При reload — обновить `self.servos` dict

```python
def _load_config(self):
    """Load servo configuration from servo_config.json."""
    self.servos.clear()
    joints = get_all_joints()
    for j in joints:
        min_deg = math.degrees(j['min_rad'])
        max_deg = math.degrees(j['max_rad'])
        servo = ServoConfig(
            name=j['name'],
            channel=j['channel'],
            min_angle=min_deg,
            max_angle=max_deg,
            inverted=j['inverted'],
            offset=0.0,  # will be loaded from calibration
        )
        self.servos[servo.name] = servo

def _config_watchdog(self):
    """Check config file mtime and reload if changed."""
    if ServoConfigReader.is_config_changed(self.config_path):
        self.get_logger().info('Config file changed, reloading...')
        self._load_config()
```

### Task 2.3: ServoConfigReader (C++)
**Файл:** `robot_dog_ws/src/dog_hardware/include/dog_hardware/servo_config_reader.hpp`

Требуется: подключить nlohmann/json в CMakeLists.txt dog_hardware.

```cpp
#pragma once
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <optional>
#include <fstream>
#include <filesystem>

namespace dog_hardware {

struct JointConfig {
    std::string name;
    std::string leg;       // "LF", "RF", "LR", "RR"
    std::string type;      // "hip", "thigh", "calf"
    int channel;           // servo_id = PCA9685 channel
    bool inverted;
    double min_rad;
    double max_rad;
    double neutral_rad;
    double min_pulse_ms;
    double max_pulse_ms;
};

struct ServoConfigReader {
    std::string config_path_;
    nlohmann::json config_;
    std::filesystem::file_time_type last_mtime_;

    ServoConfigReader(const std::string& config_path = "/home/sg/robot-dog/servo_config.json");

    void reload();
    bool is_changed();

    // Joint queries
    std::vector<JointConfig> get_all_joints() const;
    std::optional<JointConfig> get_joint(const std::string& name) const;
    std::optional<JointConfig> get_joint_by_channel(int channel) const;

    // Body/geometry
    double get_hip_length() const;
    double get_thigh_length() const;
    double get_calf_length() const;
    double get_standing_height() const;
    double get_min_height() const;
    double get_max_height() const;
    double get_foot_x_scale() const;
    double get_foot_y_scale() const;

    // Gait params
    double get_gait_period() const;
    double get_step_height() const;
    double get_turn_gain() const;
    double get_cog_pitch_gain() const;
    double get_cog_roll_gain() const;

    // Balance params (PID)
    struct PID { double kp, ki, kd; };
    PID get_pid_roll() const;
    PID get_pid_pitch() const;
    PID get_pid_height() const;
    double get_max_roll_correction() const;
    double get_max_pitch_correction() const;
    double get_max_height_correction() const;

    // Servo driver params
    int get_i2c_bus() const;
    int get_i2c_address() const;
    int get_pwm_frequency() const;
};

} // namespace dog_hardware
```

Реализация: `robot_dog_ws/src/dog_hardware/src/servo_config_reader.cpp`

### Task 2.4: Подключить nlohmann/json в dog_hardware CMakeLists.txt
**Файл:** `robot_dog_ws/src/dog_hardware/CMakeLists.txt`

Добавить:
```cmake
find_package(nlohmann_json REQUIRED)
target_link_libraries(servo_driver_node nlohmann_json::nlohmann_json)
```

### Task 2.5: Обновить gait_controller.cpp
**Файл:** `robot_dog_ws/src/dog_control_cpp/src/gait_controller.cpp`

Изменения:
1. Добавить `#include "dog_hardware/servo_config_reader.hpp"`
2. В конструкторе убрать `declare_parameter()` для geometry/gait params
3. В `on_configure()` вместо `get_parameter()` — создать `ServoConfigReader` и читать напрямую:

```cpp
// on_configure()
ServoConfigReader cfg(config_path_);
cfg.reload();

stance_height_ = cfg.get_standing_height();
foot_x_scale_ = cfg.get_foot_x_scale();
foot_y_scale_ = cfg.get_foot_y_scale();
hip_length_ = cfg.get_hip_length();
thigh_length_ = cfg.get_thigh_length();
calf_length_ = cfg.get_calf_length();
gait_period_ = cfg.get_gait_period();
// ... и т.д.
```

4. Добавить timer-based watchdog для auto-reload конфига

### Task 2.6: Обновить balance_controller.cpp
Аналогично gait_controller.cpp — использовать ServoConfigReader для PID и height limits.

```cpp
// on_configure()
ServoConfigReader cfg(config_path_);
cfg.reload();

kp_roll_ = cfg.get_pid_roll().kp;
// ...
height_min_m_ = cfg.get_min_height();
height_max_m_ = cfg.get_max_height();
```

### Task 2.7: Интеграция с calibrate_web.py
**Файл:** `calibrate_web.py`

При POST /api/cal/config/save:
1. Сохранить JSON (как сейчас)
2. Обнулить mtime cache в servo_config_reader (`_last_mtime = 0`)
3. Ноды сами обнаружат изменение через watchdog

Добавить endpoint:
```python
@app.get("/api/cal/config/status")
def config_status():
    from servo_config_reader import get_servo_config_mtime, _last_mtime
    return {
        "config_mtime": get_servo_config_mtime(),
        "last_read": _last_mtime,
        "changed": _last_mtime > 0 and get_servo_config_mtime() > _last_mtime,
    }
```

### Task 2.8: Обновить launch файлы
**Файл:** `robot_dog_ws/src/dog_control_cpp/launch/control_launch.py`
**Файл:** `robot_dog_ws/src/dog_hardware/launch/servo_driver.launch.py`

Убрать все `<param>` для конфига (Phase 1 добавил). Заменить на:
```python
# control_launch.py
control_node = Node(
    package='dog_control_cpp',
    node_executable='gait_controller',
    parameters=[{
        'config_path': '/home/sg/robot-dog/servo_config.json',
    }]
)
```

### Task 2.9: Testing
1. Unit-тесты ServoConfigReader (Python):
   - Проверка load_servo_config() — все поля на месте
   - get_all_joints() — 12 суставов, правильные лимиты
   - is_config_changed() — detects file modification
2. Unit-тесты ServoConfigReader (C++):
   - Проверка парсинга всех секций
   - Проверка типов данных
3. Integration-тест:
   - Запустить ноды
   - Изменить servo_config.json
   - Проверить что ноды перегрузили конфиг без перезапуска
   - `ros2 param list` — старых params нет

---

## ⚠️ Risks & Mitigations

| # | Risk | Mitigation |
|---|------|------------|
| 1 | nlohmann/json не подключён в CMakeLists.txt | `apt install nlohmann-json3-dev` или fetch в CMake |
| 2 | Парсинг JSON медленнее чем чтение params | reload происходит раз в секунду, не в control loop (100Hz) |
| 3 | Race condition при частых save | Файловая система гарантирует atomic write при overwrite; watchdog проверяет mtime |
| 4 | Servo_config.json повреждён | Try/catch при reload, fallback на предыдущее состояние |
| 5 | Разные пути к JSON в Docker vs host | config_path параметр в launch, по умолчанию `/home/sg/robot-dog/servo_config.json` |
| 6 | C++ ServoConfigReader в dog_hardware, но gait_controller в dog_control_cpp | dog_control_cpp зависит от dog_hardware (already the case) |

---

## 📅 Estimation

| Task | Est. | Depends On |
|------|------|-----------|
| 2.1: ServoConfigReader (Python) | 2h | — |
| 2.2: Переписать servo_driver_node.py | 2h | 2.1 |
| 2.3: ServoConfigReader (C++) | 3h | — |
| 2.4: Подключить nlohmann/json | 1h | — |
| 2.5: Обновить gait_controller.cpp | 2h | 2.3 |
| 2.6: Обновить balance_controller.cpp | 2h | 2.3 |
| 2.7: Интеграция с calibrate_web.py | 1h | 2.1 |
| 2.8: Обновить launch файлы | 1h | 2.5, 2.6 |
| 2.9: Тестирование | 3h | 2.2, 2.5, 2.6 |
| **Total** | **~17h** | |

---

## 🧪 Verification Steps

### V1: Python ServoConfigReader
```bash
cd robot_dog_ws/src/dog_hardware
python3 -c "from dog_hardware.servo_config_reader import get_all_joints; print(len(get_all_joints()))"
# Ожидается: 12
```

### V2: C++ ServoConfigReader
```bash
cd robot_dog_ws
colcon build --packages-select dog_hardware
# 0 errors
```

### V3: Ноды читают JSON
```bash
# Запустить ноды, изменить servo_config.json
# Проверить что nоды перегрузили конфиг без перезапуска
```

### V4: colcon build
```bash
colcon build --packages-select dog_hardware dog_control_cpp dog_web
# 0 errors, 0 warnings
```

---

*Plan created: 2026-05-12*
*Review: Pending*
*Approach: Direct JSON reading, NO YAML conversion*
