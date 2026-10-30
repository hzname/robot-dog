# Phase 1 Plan: Hardware Integration — убрать хардкод

**Дата:** 2026-05-05
**Цель:** Все C++ ноды читают параметры из servo_config.json через ROS2 parameters.
**Требования:** HARD-01..07

---

## Архитектура

```
servo_config.json ──(Python launch)──▶ ROS2 parameters
                                            │
                                    C++ ноды (rclcpp::get_parameter)
```

Сервис: `get_servo_config()` — Python функция в launch файлах, читает JSON, возвращает dict параметров.

---

## HARD-01: ServoController — joint limits из конфига

**Файлы:** servo_controller.hpp, servo_controller.cpp

**Изменения в servo_controller.hpp:**
1. Добавить публичные методы:
```cpp
void setInversionArray(const std::array<bool, 12>& inverted);
void setAllJointLimits(const std::array<JointLimits, 12>& limits);
```
2. `inverted_` из private в private, но инициализировать `inverted_.fill(false)` (по умолчанию)

**Изменения в servo_controller.cpp:**
1. Убрать хардкод joint limits из конструктора (строки 29-48)
2. Заменить на:
```cpp
ServoController::ServoController(rclcpp::Logger logger)
    : logger_(logger),
      target_positions_{0},
      smoothed_positions_{0},
      current_positions_{0},
      previous_positions_{0}
{
  for (size_t i = 0; i < 12; ++i)
  {
    joint_limits_[i].name = DEFAULT_JOINT_NAMES[i];
    // Use default values from JointLimits struct
    // min/max: -M_PI to M_PI, neutral: 0.0 — already set in struct defaults
  }
}
```
3. Реализовать новые методы:
```cpp
void ServoController::setInversionArray(const std::array<bool, 12>& inverted)
{ inverted_ = inverted; }

void ServoController::setAllJointLimits(const std::array<JointLimits, 12>& limits)
{ joint_limits_ = limits; }
```

**Сервис (servo_driver.launch.py):**
```python
# Читаем servo_config.json
import json
cfg = json.load(open('/home/sg/robot-dog/servo_config.json'))

# inverted array
inverted = [j['inverted'] for j in cfg['joints']['list']]

# joint limits
joint_limits = []
for j in cfg['joints']['list']:
    type_cfg = cfg['joints'][j['type']]
    limits = {
        'name': j['name'],
        'min_position_rad': type_cfg['min_rad'],
        'max_position_rad': type_cfg['max_rad'],
        'neutral_position_rad': type_cfg['neutral_rad'],
    }
    joint_limits.append(limits)
```

---

## HARD-02: ServoController — inverted_ из конфига

**Файлы:** servo_controller.hpp

**Изменения в servo_driver.launch.py:**
```python
parameters=[
    {
        # ... другие параметры
        'inverted_lf_hip': cfg['joints']['list'][0]['inverted'],
        'inverted_lf_thigh': cfg['joints']['list'][1]['inverted'],
        # ... или лучше массив:
        'inverted_flags': [j['inverted'] for j in cfg['joints']['list']],
    }
]
```

**Изменения в servo_controller.hpp/cpp:**
1. Заменим `std::array<bool, 12> inverted_` на `std::vector<bool> inverted_` (или оставим array)
2. В `setInversionArray()` — присваиваем при configure
3. Добавить parameter `inverted_flags` для загрузки из launch

---

## HARD-03: GaitController — foot_x/y scale из конфига

**Файлы:** gait_controller.cpp, gait_controller.hpp

**Изменения в gait_controller.hpp:**
```cpp
double foot_x_scale_ = 0.5;
double foot_y_scale_ = 0.8;
```

**Изменения в gait_controller.cpp:**
```cpp
// Constructor — declare_parameter
declare_parameter("foot_x_scale", foot_x_scale_);
declare_parameter("foot_y_scale", foot_y_scale_);

// on_configure — get_parameter
foot_x_scale_ = get_parameter("foot_x_scale").as_double();
foot_y_scale_ = get_parameter("foot_y_scale").as_double();

// calculateFootTrajectory
double default_x = leg_positions_[leg_idx][0] * foot_x_scale_;
double default_y = leg_positions_[leg_idx][1] * foot_y_scale_;

// calculateStancePose
double fx = leg_positions_[leg_idx][0] * foot_x_scale_;
double fy = leg_positions_[leg_idx][1] * foot_y_scale_;
```

---

## HARD-04: GaitController — hip_length из конфига

**Файлы:** gait_controller.cpp

**Изменения:**
```cpp
// Constructor — declare_parameter
declare_parameter("hip_length_m", hip_length_);

// on_configure — get_parameter
hip_length_ = get_parameter("hip_length_m").as_double();
// В конструкторе: hip_length_ = 0.045 — оставить как default
```

---

## HARD-05: ServoController — inverted_ полностью из конфига

Это уже covered HARD-02, реализуем в Plan 1.2.

---

## HARD-06: BalanceController — height clamp из конфига

**Файлы:** balance_controller.cpp

**Изменения:**
```cpp
// Constructor
double height_min_m_ = 0.15;  // default
double height_max_m_ = 0.35;  // default
declare_parameter("height_min_m", height_min_m_);
declare_parameter("height_max_m", height_max_m_);

// on_configure
height_min_m_ = get_parameter("height_min_m").as_double();
height_max_m_ = get_parameter("height_max_m").as_double();

// heightCmdCallback
target_height_ = std::clamp(msg->data, height_min_m_, height_max_m_);
```

---

## HARD-07: ServoDriverNode — относительные пути к calibration

**Файлы:** servo_driver_node.cpp

**Изменения:**
```cpp
// on_configure — заменить хардкод путей на параметры:
std::string calib_dir = get_parameter("calibration_dir").as_string();
std::string zero_file = get_parameter("zero_offsets_file").as_string();
std::string limits_file = get_parameter("joint_limits_file").as_string();

std::string zero_path = calib_dir + "/" + zero_file;
std::string limits_path = calib_dir + "/" + limits_file;

controller_->loadZeroOffsets(zero_path);
controller_->loadCalibratedLimits(limits_path);
```

**Параметры по умолчанию:**
- `calibration_dir`: `"/home/sg/robot-dog"`
- `zero_offsets_file`: `"calibration.json"`
- `joint_limits_file`: `"joint_limits.json"`

---

## Дополнительные изменения (из servo_config.json known_issues)

### Turn gain из конфига
```cpp
// gait_controller
declare_parameter("turn_gain", 0.05);
declare_parameter("cog_pitch_gain_m_per_rad", 0.02);
declare_parameter("cog_roll_gain_m_per_rad", 0.02);

// applyBalanceCorrections
x += pitch_corr * cog_pitch_gain_m_per_rad_;
y -= roll_corr * cog_roll_gain_m_per_rad_;
```

### Joint names из конфига
```cpp
// gait_controller
declare_parameter("joint_names", joint_names_);
// get_parameter в on_configure
joint_names_ = get_parameter("joint_names").as_string_array();
```

---

## Launch файлы — новый helper

**dog_control_cpp/launch/control_launch.py** — добавить `load_servo_config()`:

```python
def load_servo_config(config_path='/home/sg/robot-dog/servo_config.json'):
    import json
    with open(config_path) as f:
        cfg = json.load(f)
    
    return {
        # Gait controller params
        'hip_length_m': cfg['body']['link_lengths_m']['hip'],
        'foot_x_scale': cfg['body']['standing']['foot_x_scale'],
        'foot_y_scale': cfg['body']['standing']['foot_y_scale'],
        'turn_gain': cfg['gait']['gaits']['trot']['turn_gain'],
        'cog_pitch_gain_m_per_rad': cfg['gait']['gaits']['trot']['cog_pitch_gain_m_per_rad'],
        'cog_roll_gain_m_per_rad': cfg['gait']['gaits']['trot']['cog_roll_gain_m_per_rad'],
        'joint_names': [j['name'] for j in cfg['joints']['list']],
        'stance_height': cfg['body']['standing']['_computed']['standing_height_m'],
        'body_length': cfg['body']['length_m'],
        'body_width': cfg['body']['width_m'],
        'thigh_length': cfg['body']['link_lengths_m']['thigh'],
        'calf_length': cfg['body']['link_lengths_m']['calf'],
        # Balance controller params
        'height_min_m': cfg['body']['standing']['_computed']['min_height_m'],
        'height_max_m': cfg['body']['standing']['_computed']['max_height_m'],
        'kp_roll': cfg['balance']['pid']['roll']['kp'],
        'ki_roll': cfg['balance']['pid']['roll']['ki'],
        'kd_roll': cfg['balance']['pid']['roll']['kd'],
        'kp_pitch': cfg['balance']['pid']['pitch']['kp'],
        'ki_pitch': cfg['balance']['pid']['pitch']['ki'],
        'kd_pitch': cfg['balance']['pid']['pitch']['kd'],
        'kp_height': cfg['balance']['pid']['height']['kp'],
        'ki_height': cfg['balance']['pid']['height']['ki'],
        'kd_height': cfg['balance']['pid']['height']['kd'],
        'max_roll_correction': cfg['balance']['max_correction']['roll_rad'],
        'max_pitch_correction': cfg['balance']['max_correction']['pitch_rad'],
        'max_height_correction': cfg['balance']['max_correction']['height_m'],
    }
```

**dog_hardware_cpp/launch/servo_driver.launch.py** — добавить:
```python
def load_servo_config(config_path='/home/sg/robot-dog/servo_config.json'):
    import json
    with open(config_path) as f:
        cfg = json.load(f)
    
    return {
        'inverted_flags': [j['inverted'] for j in cfg['joints']['list']],
        'joint_names': [j['name'] for j in cfg['joints']['list']],
        'calibration_dir': os.path.dirname(config_path),
        'zero_offsets_file': cfg['calibration']['zero_offsets_file'],
        'joint_limits_file': cfg['calibration']['joint_limits_file'],
    }
```

---

## Сервис (servo_driver.launch.py) — inverted_ из конфига

```cpp
// ServoDriverNode::on_configure — после loadParameters():
std::array<bool, 12> inverted_flags;
auto inverted_param = get_parameter("inverted_flags");
if (inverted_param.get_type() != rclcpp::PARAMETER_NOT_SET) {
    auto vals = inverted_param.as_bool_array();
    for (int i = 0; i < 12; ++i) inverted_flags[i] = vals[i];
    controller_->setInversionArray(inverted_flags);
}
```

---

## Порядок реализации

| # | Задача | Файлы | HARD |
|---|--------|-------|------|
| 1 | ServoController: setInversionArray, setAllJointLimits, убрать хардкод из конструктора | servo_controller.hpp, servo_controller.cpp | HARD-01, HARD-05 |
| 2 | ServoDriverNode: параметризовать calibration paths | servo_driver_node.cpp | HARD-07 |
| 3 | GaitController: footprint scale, hip_length, turn/gain | gait_controller.cpp, gait_controller.hpp | HARD-03, HARD-04 |
| 4 | BalanceController: height_min/height_max | balance_controller.cpp | HARD-06 |
| 5 | Launch файлы: load_servo_config() | control_launch.py, servo_driver.launch.py | Все |

---

## Риски

1. `get_parameter("inverted_flags").as_bool_array()` — проверить что работает в ROS2 Jazzy
2. `get_parameter("joint_names").as_string_array()` — аналогично
3. При отсутствии параметров — должны использоваться defaults

## Верификация

- `colcon build` проходит
- `ros2 param list` показывает все новые параметры
- `ros2 param get <node> foot_x_scale` возвращает правильное значение
- Сервоприводы получают правильные лимиты и инверсию

---
*Plan created: 2026-05-05*
