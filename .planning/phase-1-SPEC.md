# Phase 1 Spec: Hardware Integration — Убрать хардкод

**Дата:** 2026-05-05
**Цель:** Все C++ ноды читают параметры из servo_config.json (через ROS2 parameters) вместо хардкода.
**Требования:** HARD-01..07

---

## Архитектура

servo_config.json уже существует и валидируется робот-конфигуратором. C++ ноды получают параметры через ROS2 parameter system, где параметры загружаются при старте.

```
servo_config.json ──(робот-конфигуратор)──▶ _codegen в JSON
                                                │
                                                ▼
                                        launch файлы
                                        (python, загружают JSON
                                         в ROS2 parameters)
                                                │
                                                ▼
                                        C++ ноды
                                        (rclcpp::get_parameter())
```

**Ключевое решение:** Python launch файлы будут читать servo_config.json и передавать все параметры через ROS2 Parameter. Это значит:
- servo_controller.hpp — убрать хардкод, добавить public set* методы
- servo_driver_node.cpp — параметризовать пути к calibration файлам
- gait_controller.cpp — параметризовать все размеры и gain-коэффициенты
- balance_controller.cpp — параметризовать height clamp

---

## План реализации

### Plan 1.1: ServoController — конфиг-драйв

**Изменения:** servo_controller.hpp + servo_controller.cpp

**Что убрать:**
1. `std::array<bool, 12> inverted_ = {false, false, false, true, true, true, false, false, false, true, true, true};` (строка 214-217 servo_controller.hpp)
2. Joint limits в конструкторе: `joint_limits_[i].min_position_rad = -0.8;` и т.д. (строки 29-48 servo_controller.cpp)

**Что добавить:**
1. `void setInversionArray(const std::array<bool, 12>& inverted)` — public метод
2. `void setAllJointLimits(const std::array<JointLimits, 12>& limits)` — public метод
3. В конструкторе: инициализировать `inverted_.fill(false)` и joint_limits с дефолтными значениями (обратная совместимость)
4. В `configureJointLimits()`: уже работает — обновляет `joint_limits_[servo_id]`
5. Конструктор: НЕ инициализировать joint_limits hardcoded — оставить все значения по умолчанию (из JointLimits struct)

**Сервис для чтения JSON:**
- Создать `ConfigLoader` в dog_hardware_cpp — простой JSON парсер (только нужные поля из servo_config.json)
- ИЛИ использовать nlohmann/json (проверить, подключён ли уже в CMakeLists.txt)

### Plan 1.2: ServoDriverNode — относительные пути к calibration

**Изменения:** servo_driver_node.cpp

**Что убрать:**
1. Строка 123: `controller_->loadZeroOffsets("/home/sg/robot-dog/calibration.json")` — абсолютный путь
2. Строка 124: `controller_->loadCalibratedLimits("/home/sg/robot-dog/joint_limits.json")` — абсолютный путь

**Что добавить:**
1. В `on_configure()`:
   ```cpp
   // Сначала загрузить базовые параметры
   loadParameters();
   
   // Загрузить пути к calibration файлам из конфига
   std::string calib_dir = get_parameter("calibration_directory").as_string();
   std::string zero_offsets_file = get_parameter("zero_offsets_file").as_string();
   std::string joint_limits_file = get_parameter("joint_limits_file").as_string();
   
   // Объединить в полный путь
   std::string zero_path = calib_dir + "/" + zero_offsets_file;
   std::string limits_path = calib_dir + "/" + joint_limits_file;
   
   controller_->loadZeroOffsets(zero_path);
   controller_->loadCalibratedLimits(limits_path);
   ```
2. Новый parameter `calibration_directory` (по умолчанию `calibration/`)
3. Новый parameter `zero_offsets_file` (по умолчанию `calibration.json`)
4. Новый parameter `joint_limits_file` (по умолчанию `joint_limits.json`)

### Plan 1.3: GaitController — параметризация размеров и gain

**Изменения:** gait_controller.cpp + gait_controller.hpp

**Что убрать:**
1. `hip_length_(0.045)` → parameter
2. `joint_names_` хардкод → параметр (array of strings)
3. `default_x = leg_positions_[leg_idx][0] * 0.5` → `* foot_x_scale_`
4. `default_y = leg_positions_[leg_idx][1] * 0.8` → `* foot_y_scale_`
5. Hardcoded `0.05` (turn gain, строка 316)
6. Hardcoded `0.02` (CoG gains, строки 353-354)

**Что добавить:**
```cpp
// В конструкторе — declare_parameter:
declare_parameter("hip_length_m", hip_length_);
declare_parameter("foot_x_scale", foot_x_scale_);
declare_parameter("foot_y_scale", foot_y_scale_);
declare_parameter("turn_gain", turn_gain_);
declare_parameter("cog_pitch_gain_m_per_rad", cog_pitch_gain_);
declare_parameter("cog_roll_gain_m_per_rad", cog_roll_gain_);
declare_parameter("joint_names", joint_names_);

// В on_configure — get_parameter:
hip_length_ = get_parameter("hip_length_m").as_double();
foot_x_scale_ = get_parameter("foot_x_scale").as_double();
foot_y_scale_ = get_parameter("foot_y_scale").as_double();
turn_gain_ = get_parameter("turn_gain").as_double();
cog_pitch_gain_ = get_parameter("cog_pitch_gain_m_per_rad").as_double();
cog_roll_gain_ = get_parameter("cog_roll_gain_m_per_rad").as_double();
joint_names_ = get_parameter("joint_names").as_string_array();

// В calculateFootTrajectory:
double default_x = leg_positions_[leg_idx][0] * foot_x_scale_;
double default_y = leg_positions_[leg_idx][1] * foot_y_scale_;

// В calculateFootTrajectory (turn):
turn_offset = leg_sign * velocity_cmd.angular.z * turn_gain_ * std::sin(phase * 2.0 * M_PI);

// В applyBalanceCorrections:
x += pitch_corr * cog_pitch_gain_;
y -= roll_corr * cog_roll_gain_;
```

### Plan 1.4: BalanceController — параметризация height clamp

**Изменения:** balance_controller.cpp

**Что убрать:**
1. Строка 212: `target_height_ = std::clamp(msg->data, 0.15, 0.35);` — hardcoded clamp

**Что добавить:**
```cpp
// declare_parameter в конструкторе:
declare_parameter("height_min_m", height_min_m_);
declare_parameter("height_max_m", height_max_m_);

// get_parameter в on_configure:
height_min_m_ = get_parameter("height_min_m").as_double();
height_max_m_ = get_parameter("height_max_m").as_double();

// heightCmdCallback:
target_height_ = std::clamp(msg->data, height_min_m_, height_max_m_);
```

### Plan 1.5: Launch files — передача параметров из servo_config.json

**Изменения:**
1. dog_control_cpp/launch/ — добавить параметры gait_controller и balance_controller
2. dog_hardware_cpp/launch/ — добавить параметры servo_driver и paths к calibration

**Подход:** Создать helper Python function в launch файлах:

```python
def load_servo_config(config_path):
    """Load servo_config.json and extract ROS2 parameters."""
    import json
    with open(config_path) as f:
        cfg = json.load(f)
    
    params = {
        # Gait controller
        'hip_length_m': cfg['body']['link_lengths_m']['hip'],
        'foot_x_scale': cfg['body']['standing']['foot_x_scale'],
        'foot_y_scale': cfg['body']['standing']['foot_y_scale'],
        'cog_pitch_gain_m_per_rad': cfg['gait']['gaits']['trot']['cog_pitch_gain_m_per_rad'],
        'cog_roll_gain_m_per_rad': cfg['gait']['gaits']['trot']['cog_roll_gain_m_per_rad'],
        'turn_gain': cfg['gait']['gaits']['trot']['turn_gain'],
        'joint_names': [j['name'] for j in cfg['joints']['list']],
        # Balance controller
        'height_min_m': cfg['body']['standing']['_computed']['min_height_m'],
        'height_max_m': cfg['body']['standing']['_computed']['max_height_m'],
        # Servo controller
        'inverted_lf': not cfg['joints']['list'][0]['inverted'],
        'inverted_rf': cfg['joints']['list'][3]['inverted'],
        # ... etc
    }
    return params
```

---

## Порядок реализации

1. **Plan 1.1** — ServoController (самое критичное — inverted_ и joint limits)
2. **Plan 1.2** — ServoDriverNode (calibration paths)
3. **Plan 1.3** — GaitController (размеры и gains)
4. **Plan 1.4** — BalanceController (height clamp)
5. **Plan 1.5** — Launch files (передача параметров)
6. **Тестирование** — проверка что код собирается и параметры загружаются

## Риски

1. **nlohmann/json** — нужно проверить, подключён ли в CMakeLists.txt. Если нет — нужен альтернативный подход.
2. **ROS2 parameter arrays** — `as_string_array()` для joint_names может не работать в ROS2 Jazzy без специальной конфигурации.
3. **Обратная совместимость** — при отсутствии параметров должны использоваться defaults.

## Верификация

- `colcon build` проходит без ошибок
- Все параметры загружаются из servo_config.json (проверить через `ros2 param list` и `ros2 param get`)
- `robot_configurator.py` — валидация проходит
- Сервоприводы получают правильные лимиты и инверсию

---
*Spec created: 2026-05-05*
