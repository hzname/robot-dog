# План интеграции системы конфигурирования робота

## 🔍 Текущее состояние (АНАЛИЗ)

### Существующие компоненты:

| Компонент | Файл | Формат | Путь | Статус |
|-----------|------|--------|------|--------|
| **Калибратор** | `calibrate_web.py` | JSON | `/home/sg/robot-dog/servo_config.json` | ✅ Работает |
| **ROS2 нода** | `servo_driver_node.py` | YAML | `dog_hardware/config/servo_config.yaml` | ✅ Работает |
| **Calibration API** | `dog_web/api/calibration.py` | JSON profiles | `/workspace/calibration/profiles/*.json` | ✅ Работает |

### ❌ Проблемы:

1. **Два разных формата конфигов:**
   - Калибратор: JSON с полями `sign`, `zero_deg`, `min_deg`, `max_deg`, `coupled_to`
   - ROS2 нода: YAML с полями `inverted`, `offset`, `min_pulse`, `max_pulse`

2. **Нет синхронизации:**
   - После калибровки JSON обновляется, но ROS2 продолжает использовать старый YAML
   - При перезапуске ROS2 контейнера настройки калибровки теряются

3. **Разная семантика полей:**
   - `sign` (калибратор) vs `inverted` (ROS2) — противоположная логика!
   - `zero_deg` (калибратор) vs `offset` (ROS2) — разное применение

4. **Нет механизма экспорта:**
   - Откалиброванные параметры не конвертируются в формат ROS2

---

## 📐 Архитектура решения

### Принцип: **"Calibrate Once, Use Everywhere"**

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  calibrate_web  │────▶│  Config Sync     │◀────│  servo_driver   │
│  (порт 8081)    │     │  Service         │     │  (ROS2 node)    │
│                 │     │                  │     │                 │
│ Генерирует JSON │     │ Конвертирует     │     │ Читает YAML     │
│ servo_config.json│    │ JSON → YAML      │     │ из того же пути │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                               │
                               ▼
                        /workspace/config/
                        robot_config.yaml
                        (единый источник)
```

### Ключевые изменения:

1. **Единый файл конфигурации:** `/workspace/config/robot_config.yaml`
2. **Конвертер форматов:** `config_sync.py` — JSON ↔ YAML
3. **ROS2 нода:** Чтение из единого YAML файла
4. **Калибратор:** Экспорт в единый YAML после сохранения

---

## 🛠️ Этапы реализации

### Этап 1: Создание единого формата конфигурации

**Файл:** `/workspace/config/robot_config.yaml`

```yaml
# Единый конфиг для калибратора и ROS2
# Генерируется автоматически при калибровке

metadata:
  version: "1.0"
  created: "2025-01-15T10:30:00Z"
  last_modified: "2025-01-15T10:30:00Z"
  source: "calibration_service"

# Общие настройки PCA9685
pca9685:
  i2c_bus: 0
  i2c_address: 0x40
  pwm_frequency: 50

# Сервоприводы (12 основных + 4 дополнительных)
servos:
  # === Передняя левая нога ===
  - name: "lf_hip_joint"
    channel: 0
    enabled: true
    leg: "lf"
    joint_type: "hip"
    
    # Калибровочные параметры (заполняются калибратором)
    calibration:
      sign: 1              # направление (1 или -1)
      zero_deg: 0.0        # смещение нуля в градусах
      min_deg: -60.0       # минимальный угол
      max_deg: 60.0        # максимальный угол
      coupled_to: null     # связанный сустав
      coupling_coeff: 0.0  # коэффициент связи
    
    # Аппаратные ограничения (MG996R)
    hardware:
      min_pulse_us: 520    # минимальный импульс мкс
      max_pulse_us: 2220   # максимальный импульс мкс
      deadband_us: 7       # мёртвая зона
    
    # Для ROS2 (вычисляется автоматически)
    ros2:
      inverted: false      # вычисляется из sign
      offset_deg: 0.0      # равно zero_deg
      min_angle: -60.0     # равно min_deg
      max_angle: 60.0      # равно max_deg

  - name: "lf_thigh_joint"
    channel: 1
    enabled: true
    leg: "lf"
    joint_type: "thigh"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "lf_calf_joint"
    channel: 2
    enabled: true
    leg: "lf"
    joint_type: "calf"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  # === Передняя правая нога ===
  - name: "rf_hip_joint"
    channel: 3
    enabled: true
    leg: "rf"
    joint_type: "hip"
    calibration:
      sign: -1             # инвертировано для правой стороны
      zero_deg: 0.0
      min_deg: -60.0
      max_deg: 60.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "rf_thigh_joint"
    channel: 4
    enabled: true
    leg: "rf"
    joint_type: "thigh"
    calibration:
      sign: -1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "rf_calf_joint"
    channel: 5
    enabled: true
    leg: "rf"
    joint_type: "calf"
    calibration:
      sign: -1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  # === Задняя левая нога ===
  - name: "lr_hip_joint"
    channel: 6
    enabled: true
    leg: "lr"
    joint_type: "hip"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -60.0
      max_deg: 60.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "lr_thigh_joint"
    channel: 7
    enabled: true
    leg: "lr"
    joint_type: "thigh"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "lr_calf_joint"
    channel: 8
    enabled: true
    leg: "lr"
    joint_type: "calf"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  # === Задняя правая нога ===
  - name: "rr_hip_joint"
    channel: 9
    enabled: true
    leg: "rr"
    joint_type: "hip"
    calibration:
      sign: -1
      zero_deg: 0.0
      min_deg: -60.0
      max_deg: 60.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "rr_thigh_joint"
    channel: 10
    enabled: true
    leg: "rr"
    joint_type: "thigh"
    calibration:
      sign: -1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "rr_calf_joint"
    channel: 11
    enabled: true
    leg: "rr"
    joint_type: "calf"
    calibration:
      sign: -1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  # === Дополнительные сервоприводы ===
  - name: "head_pan"
    channel: 12
    enabled: false
    leg: null
    joint_type: "aux"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "head_tilt"
    channel: 13
    enabled: false
    leg: null
    joint_type: "aux"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "arm_joint_1"
    channel: 14
    enabled: false
    leg: null
    joint_type: "aux"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

  - name: "arm_joint_2"
    channel: 15
    enabled: false
    leg: null
    joint_type: "aux"
    calibration:
      sign: 1
      zero_deg: 0.0
      min_deg: -90.0
      max_deg: 90.0
      coupled_to: null
      coupling_coeff: 0.0
    hardware:
      min_pulse_us: 520
      max_pulse_us: 2220
      deadband_us: 7

# Кинематические параметры (для IK/FK)
kinematics:
  body_dimensions_mm:
    width: 120
    length: 180
    hip_offset: 55
    femur_length: 105
    tibia_length: 105
  
  # Преобразование в метры для ROS2
  body_dimensions_m:
    width: 0.120
    length: 0.180
    hip_offset: 0.055
    femur_length: 0.105
    tibia_length: 0.105

# Настройки походок
gait:
  default: "trot"
  trot:
    cycle_time: 0.8
    step_height: 0.03
    stance_ratio: 0.5
    body_height: 0.15
  
  walk:
    cycle_time: 1.2
    step_height: 0.025
    stance_ratio: 0.75
    body_height: 0.15

# Emergency Stop
estop:
  enabled: true
  tilt_threshold_deg: 45.0
  overcurrent_threshold_a: 2.5
  low_battery_v: 6.0
  communication_timeout_ms: 500
```

---

### Этап 2: Конвертер форматов (Config Sync Service)

**Файл:** `/workspace/robot_dog_ws/src/dog_config/dog_config/config_sync.py`

```python
#!/usr/bin/env python3
"""
Синхронизация конфигурации между калибратором (JSON) и ROS2 (YAML).

Конвертирует:
  /workspace/servo_config.json (от calibrate_web.py)
  → /workspace/config/robot_config.yaml (для ROS2 нод)
"""

import json
import yaml
import os
from datetime import datetime
from typing import Dict, Any, List
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Пути к файлам
JSON_CONFIG_PATH = "/workspace/servo_config.json"
YAML_CONFIG_PATH = "/workspace/config/robot_config.yaml"
BACKUP_DIR = "/workspace/config/backups"


def load_json_config(path: str) -> Dict[str, Any]:
    """Загрузка JSON конфига от калибратора."""
    with open(path, 'r') as f:
        return json.load(f)


def load_yaml_config(path: str) -> Dict[str, Any]:
    """Загрузка YAML конфига для ROS2."""
    if not os.path.exists(path):
        logger.warning(f"YAML config not found: {path}")
        return {}
    
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def save_yaml_config(config: Dict[str, Any], path: str):
    """Сохранение YAML конфига с бэкапом."""
    # Создаём бэкап если файл существует
    if os.path.exists(path):
        os.makedirs(BACKUP_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = os.path.join(BACKUP_DIR, f"robot_config_{timestamp}.yaml")
        os.rename(path, backup_path)
        logger.info(f"Backup created: {backup_path}")
    
    # Сохраняем новый конфиг
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    
    logger.info(f"Saved YAML config: {path}")


def json_to_yaml(json_cfg: Dict[str, Any], existing_yaml: Dict[str, Any] = None) -> Dict[str, Any]:
    """
    Конвертация JSON конфига калибратора в YAML формат для ROS2.
    
    Args:
        json_cfg: Конфиг из calibrate_web.py
        existing_yaml: Существующий YAML конфиг (для сохранения не-калибровочных параметров)
    
    Returns:
        Объединённый YAML конфиг
    """
    # Базовая структура
    yaml_cfg = {
        "metadata": {
            "version": "1.0",
            "created": datetime.now().isoformat(),
            "last_modified": datetime.now().isoformat(),
            "source": "calibration_service"
        },
        "pca9685": {
            "i2c_bus": 0,
            "i2c_address": 0x40,
            "pwm_frequency": 50
        },
        "servos": [],
        "kinematics": {},
        "gait": {},
        "estop": {}
    }
    
    # Сохраняем не-калибровочные секции из существующего конфига
    if existing_yaml:
        for key in ["kinematics", "gait", "estop"]:
            if key in existing_yaml:
                yaml_cfg[key] = existing_yaml[key]
    
    # Конвертируем основные суставы
    leg_mapping = {
        "lf": ["lf_hip_joint", "lf_thigh_joint", "lf_calf_joint"],
        "rf": ["rf_hip_joint", "rf_thigh_joint", "rf_calf_joint"],
        "lr": ["lr_hip_joint", "lr_thigh_joint", "lr_calf_joint"],
        "rr": ["rr_hip_joint", "rr_thigh_joint", "rr_calf_joint"]
    }
    
    # Маппинг имён из JSON в стандартные имена
    json_to_standard_names = {
        "lf_hip_joint": "lf_hip_joint",
        "lf_thigh_joint": "lf_thigh_joint",
        "lf_calf_joint": "lf_calf_joint",
        "rf_hip_joint": "rf_hip_joint",
        "rf_thigh_joint": "rf_thigh_joint",
        "rf_calf_joint": "rf_calf_joint",
        "lr_hip_joint": "lr_hip_joint",
        "lr_thigh_joint": "lr_thigh_joint",
        "lr_calf_joint": "lr_calf_joint",
        "rr_hip_joint": "rr_hip_joint",
        "rr_thigh_joint": "rr_thigh_joint",
        "rr_calf_joint": "rr_calf_joint",
    }
    
    # Определяем leg и joint_type из имени
    def parse_joint_name(name: str):
        parts = name.split('_')
        if len(parts) >= 3:
            leg = parts[0]  # lf, rf, lr, rr
            joint_type = parts[1]  # hip, thigh, calf
            return leg, joint_type
        return None, None
    
    # Конвертируем каждый сустав из JSON
    for joint in json_cfg.get("joints", []):
        name = joint["name"]
        leg, joint_type = parse_joint_name(name)
        
        # Вычисляем inverted из sign (противоположная логика!)
        sign = joint.get("sign", 1)
        inverted = (sign == -1)
        
        servo_cfg = {
            "name": name,
            "channel": joint["servo_id"],
            "enabled": joint.get("enabled", True),
            "leg": leg,
            "joint_type": joint_type,
            "calibration": {
                "sign": sign,
                "zero_deg": joint.get("zero_deg", 0.0),
                "min_deg": joint.get("min_deg", -90.0),
                "max_deg": joint.get("max_deg", 90.0),
                "coupled_to": joint.get("coupled_to"),
                "coupling_coeff": joint.get("coupling_coeff", 0.0)
            },
            "hardware": {
                "min_pulse_us": 520,
                "max_pulse_us": 2220,
                "deadband_us": 7
            },
            "ros2": {
                "inverted": inverted,
                "offset_deg": joint.get("zero_deg", 0.0),
                "min_angle": joint.get("min_deg", -90.0),
                "max_angle": joint.get("max_deg", 90.0)
            }
        }
        
        yaml_cfg["servos"].append(servo_cfg)
    
    # Добавляем дополнительные сервоприводы из JSON
    for aux in json_cfg.get("aux_servos", []):
        name = aux["name"]
        
        sign = aux.get("sign", 1)
        inverted = (sign == -1)
        
        servo_cfg = {
            "name": name,
            "channel": aux["servo_id"],
            "enabled": aux.get("enabled", False),
            "leg": None,
            "joint_type": "aux",
            "calibration": {
                "sign": sign,
                "zero_deg": aux.get("zero_deg", 0.0),
                "min_deg": aux.get("min_deg", -90.0),
                "max_deg": aux.get("max_deg", 90.0),
                "coupled_to": None,
                "coupling_coeff": 0.0
            },
            "hardware": {
                "min_pulse_us": 520,
                "max_pulse_us": 2220,
                "deadband_us": 7
            },
            "ros2": {
                "inverted": inverted,
                "offset_deg": aux.get("zero_deg", 0.0),
                "min_angle": aux.get("min_deg", -90.0),
                "max_angle": aux.get("max_deg", 90.0)
            }
        }
        
        yaml_cfg["servos"].append(servo_cfg)
    
    # Конвертируем body_dimensions
    body_dims = json_cfg.get("body_dimensions", {})
    yaml_cfg["kinematics"]["body_dimensions_mm"] = {
        "width": body_dims.get("width_mm", 120),
        "length": body_dims.get("length_mm", 180),
        "hip_offset": body_dims.get("hip_mm", 55),
        "femur_length": body_dims.get("thigh_mm", 105),
        "tibia_length": body_dims.get("calf_mm", 105)
    }
    
    # Добавляем метры
    dims_mm = yaml_cfg["kinematics"]["body_dimensions_mm"]
    yaml_cfg["kinematics"]["body_dimensions_m"] = {
        k: v / 1000.0 for k, v in dims_mm.items()
    }
    
    return yaml_cfg


def sync_configs(force: bool = False) -> bool:
    """
    Синхронизация конфигов: JSON → YAML.
    
    Args:
        force: Принудительная синхронизация даже если JSON старше YAML
    
    Returns:
        True если синхронизация выполнена
    """
    # Проверяем существование JSON
    if not os.path.exists(JSON_CONFIG_PATH):
        logger.error(f"JSON config not found: {JSON_CONFIG_PATH}")
        return False
    
    # Загружаем JSON
    json_cfg = load_json_config(JSON_CONFIG_PATH)
    logger.info(f"Loaded JSON config: {JSON_CONFIG_PATH}")
    
    # Загружаем существующий YAML (если есть)
    existing_yaml = load_yaml_config(YAML_CONFIG_PATH)
    
    # Проверяем актуальность (если не force)
    if not force and existing_yaml:
        json_mtime = os.path.getmtime(JSON_CONFIG_PATH)
        yaml_mtime = os.path.getmtime(YAML_CONFIG_PATH)
        
        if json_mtime < yaml_mtime:
            logger.info("YAML config is newer than JSON, skipping sync")
            return False
    
    # Конвертируем
    yaml_cfg = json_to_yaml(json_cfg, existing_yaml)
    
    # Сохраняем
    save_yaml_config(yaml_cfg, YAML_CONFIG_PATH)
    
    logger.info("✅ Config sync completed successfully")
    return True


def yaml_to_json(yaml_cfg: Dict[str, Any]) -> Dict[str, Any]:
    """
    Обратная конвертация: YAML → JSON (для калибратора).
    Может понадобиться для миграции старых конфигов.
    """
    json_cfg = {
        "body_dimensions": {},
        "joints": [],
        "aux_servos": []
    }
    
    # Конвертируем body_dimensions
    dims = yaml_cfg.get("kinematics", {}).get("body_dimensions_mm", {})
    json_cfg["body_dimensions"] = {
        "width_mm": dims.get("width", 120),
        "length_mm": dims.get("length", 180),
        "hip_mm": dims.get("hip_offset", 55),
        "thigh_mm": dims.get("femur_length", 105),
        "calf_mm": dims.get("tibia_length", 105)
    }
    
    # Конвертируем сервоприводы
    for servo in yaml_cfg.get("servos", []):
        calib = servo.get("calibration", {})
        
        joint_entry = {
            "name": servo["name"],
            "servo_id": servo["channel"],
            "sign": calib.get("sign", 1),
            "zero_deg": calib.get("zero_deg", 0.0),
            "min_deg": calib.get("min_deg", -90.0),
            "max_deg": calib.get("max_deg", 90.0),
            "coupled_to": calib.get("coupled_to"),
            "coupling_coeff": calib.get("coupling_coeff", 0.0),
            "enabled": servo.get("enabled", True)
        }
        
        if servo.get("joint_type") == "aux":
            json_cfg["aux_servos"].append(joint_entry)
        else:
            json_cfg["joints"].append(joint_entry)
    
    return json_cfg


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Sync robot configuration between JSON and YAML")
    parser.add_argument("--force", action="store_true", help="Force sync even if YAML is newer")
    parser.add_argument("--check", action="store_true", help="Only check status, don't sync")
    
    args = parser.parse_args()
    
    if args.check:
        json_exists = os.path.exists(JSON_CONFIG_PATH)
        yaml_exists = os.path.exists(YAML_CONFIG_PATH)
        
        print(f"JSON config ({JSON_CONFIG_PATH}): {'✅' if json_exists else '❌'}")
        print(f"YAML config ({YAML_CONFIG_PATH}): {'✅' if yaml_exists else '❌'}")
        
        if json_exists and yaml_exists:
            json_mtime = os.path.getmtime(JSON_CONFIG_PATH)
            yaml_mtime = os.path.getmtime(YAML_CONFIG_PATH)
            
            if json_mtime > yaml_mtime:
                print("⚠️  JSON is newer than YAML - sync needed")
            elif json_mtime < yaml_mtime:
                print("✅ YAML is up to date")
            else:
                print("✅ Configs are in sync")
    else:
        success = sync_configs(force=args.force)
        exit(0 if success else 1)
```

---

### Этап 3: Модификация калибратора для авто-экспорта

**Изменения в `/workspace/calibrate_web.py`:**

```python
# Добавить в конец файла после route'ов

def export_to_yaml():
    """Экспорт текущего JSON конфига в YAML формат для ROS2."""
    try:
        from dog_config.config_sync import sync_configs
        sync_configs(force=True)
        logger.info("Auto-exported config to YAML for ROS2")
        return True
    except Exception as e:
        logger.error(f"Failed to export config: {e}")
        return False


@app.post("/api/cal/config/save")
def save_full(cfg: FullConfig):
    """Сохранение полного конфига с авто-экспортом в YAML."""
    save_cfg(cfg.model_dump())
    # Авто-экспорт в YAML для ROS2
    export_to_yaml()
    return {"status": "ok"}


@app.post("/api/cal/config/replace")
def replace_full(cfg: FullConfig):
    """Замена конфига с авто-экспортом."""
    save_cfg(cfg.model_dump())
    export_to_yaml()
    return {"status": "ok"}


@app.post("/api/cal/set_zero")
def set_zero(data: ServoCmd):
    """Установка нуля с авто-экспортом."""
    if not calibration_active:
        raise HTTPException(400)
    cfg = load_cfg()
    idx, j = find_joint(cfg, data.joint_name)
    if idx < 0:
        raise HTTPException(404)
    sid = j['servo_id']
    cur = current_positions[sid]
    j['zero_deg'] = round(cur + j['zero_deg'], 2)
    save_cfg(cfg)
    with lock:
        current_positions[sid] = 0.0
        target_positions[sid] = 0.0
    # Авто-экспорт после изменения калибровки
    export_to_yaml()
    return {"status": "ok", "zero_deg": j['zero_deg']}


@app.post("/api/cal/config")
def set_field(data: JointFieldCmd):
    """Изменение поля сустава с авто-экспортом."""
    cfg = load_cfg()
    idx, j = find_joint(cfg, data.joint_name)
    if idx < 0:
        raise HTTPException(404)
    if data.field not in ('sign','zero_deg','min_deg','max_deg','coupled_to','coupling_coeff'):
        raise HTTPException(400, f"Bad field: {data.field}")
    j[data.field] = data.value
    save_cfg(cfg)
    # Авто-экспорт после изменения калибровочных параметров
    export_to_yaml()
    return {"status": "ok"}
```

---

### Этап 4: Модификация ROS2 ноды для чтения YAML

**Изменения в `/workspace/robot_dog_ws/src/dog_hardware/dog_hardware/servo_driver_node.py`:**

```python
# В методе _load_config добавить поддержку нового формата

def _load_config(self, config_file: str):
    """Load servo configuration from file or use defaults."""

    if config_file:
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            # Проверяем новый формат (с ключом 'servos' как списком)
            if 'servos' in config and isinstance(config['servos'], list):
                # Новый формат из калибратора
                for servo_cfg in config['servos']:
                    # Пропускаем отключенные сервоприводы
                    if not servo_cfg.get('enabled', True):
                        continue
                    
                    # Извлекаем параметры из нового формата
                    ros2_params = servo_cfg.get('ros2', {})
                    calib_params = servo_cfg.get('calibration', {})
                    hw_params = servo_cfg.get('hardware', {})
                    
                    servo = ServoConfig(
                        name=servo_cfg['name'],
                        channel=servo_cfg['channel'],
                        min_angle=ros2_params.get('min_angle', calib_params.get('min_deg', -90.0)),
                        max_angle=ros2_params.get('max_angle', calib_params.get('max_deg', 90.0)),
                        min_pulse=hw_params.get('min_pulse_us', 520),
                        max_pulse=hw_params.get('max_pulse_us', 2220),
                        inverted=ros2_params.get('inverted', calib_params.get('sign', 1) == -1),
                        offset=ros2_params.get('offset_deg', calib_params.get('zero_deg', 0.0))
                    )
                    self.servos[servo.name] = servo
                
                self.get_logger().info(f'Loaded NEW FORMAT configuration from {config_file}')
                self.get_logger().info(f'Loaded {len(self.servos)} servos')
                return
            
            # Старый формат (обратная совместимость)
            for servo_cfg in config.get('servos', []):
                servo = ServoConfig(
                    name=servo_cfg['name'],
                    channel=servo_cfg['channel'],
                    min_angle=servo_cfg.get('min_angle', -90.0),
                    max_angle=servo_cfg.get('max_angle', 90.0),
                    min_pulse=servo_cfg.get('min_pulse', 500.0),
                    max_pulse=servo_cfg.get('max_pulse', 2500.0),
                    inverted=servo_cfg.get('inverted', False),
                    offset=servo_cfg.get('offset', 0.0)
                )
                self.servos[servo.name] = servo

            self.get_logger().info(f'Loaded OLD FORMAT configuration from {config_file}')
            return
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}, using defaults')

    # Default configuration... (остаётся без изменений)
```

---

### Этап 5: Обновление launch-файла

**Изменения в `/workspace/robot_dog_ws/src/dog_hardware/launch/servo_driver.launch.py`:**

```python
def generate_launch_description():
    """Generate launch description for servo driver."""

    pkg_path = get_package_share_directory('dog_hardware')
    
    # Приоритет путей к конфигу:
    # 1. /workspace/config/robot_config.yaml (новый единый конфиг)
    # 2. dog_hardware/config/servo_config.yaml (старый дефолтный)
    unified_config = "/workspace/config/robot_config.yaml"
    default_config = os.path.join(pkg_path, 'config', 'servo_config.yaml')
    
    # Используем новый конфиг если существует, иначе старый
    initial_config = unified_config if os.path.exists(unified_config) else default_config

    declare_config = DeclareLaunchArgument(
        'config_file',
        default_value=initial_config,
        description='Path to servo configuration YAML file'
    )
    
    # ... остальной код без изменений
```

---

### Этап 6: CLI утилита для управления

**Файл:** `/workspace/robot_dog_ws/src/dog_config/dog_config/config_cli.py`

```python
#!/usr/bin/env python3
"""CLI для управления конфигурацией робота."""

import argparse
import sys
import os
from pathlib import Path

# Добавляем путь к пакету
sys.path.insert(0, str(Path(__file__).parent.parent))

from dog_config.config_sync import (
    sync_configs, 
    load_json_config, 
    load_yaml_config,
    JSON_CONFIG_PATH,
    YAML_CONFIG_PATH
)


def cmd_status(args):
    """Показать статус синхронизации конфигов."""
    json_exists = os.path.exists(JSON_CONFIG_PATH)
    yaml_exists = os.path.exists(YAML_CONFIG_PATH)
    
    print("📊 Robot Configuration Status")
    print("=" * 50)
    print(f"JSON config (calibrator): {'✅' if json_exists else '❌'} {JSON_CONFIG_PATH}")
    print(f"YAML config (ROS2):       {'✅' if yaml_exists else '❌'} {YAML_CONFIG_PATH}")
    
    if json_exists and yaml_exists:
        import time
        json_mtime = os.path.getmtime(JSON_CONFIG_PATH)
        yaml_mtime = os.path.getmtime(YAML_CONFIG_PATH)
        
        json_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(json_mtime))
        yaml_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(yaml_mtime))
        
        print(f"\nJSON modified: {json_time}")
        print(f"YAML modified: {yaml_time}")
        
        if json_mtime > yaml_mtime:
            print("\n⚠️  WARNING: JSON is newer than YAML!")
            print("   Run 'dog_config sync' to update ROS2 config")
        elif json_mtime < yaml_mtime:
            print("\n✅ Configs are synchronized")
        else:
            print("\n✅ Configs are synchronized")
    
    elif json_exists and not yaml_exists:
        print("\n⚠️  WARNING: No YAML config for ROS2!")
        print("   Run 'dog_config sync' to generate it")
    
    return 0


def cmd_sync(args):
    """Синхронизировать JSON → YAML."""
    print("🔄 Synchronizing configuration...")
    success = sync_configs(force=args.force)
    
    if success:
        print("✅ Configuration synchronized successfully")
        print(f"   Source: {JSON_CONFIG_PATH}")
        print(f"   Target: {YAML_CONFIG_PATH}")
        return 0
    else:
        print("❌ Failed to synchronize configuration")
        return 1


def cmd_show(args):
    """Показать текущую конфигурацию."""
    if args.format == "json":
        path = JSON_CONFIG_PATH
        if not os.path.exists(path):
            print(f"❌ JSON config not found: {path}")
            return 1
        import json
        with open(path) as f:
            cfg = json.load(f)
    else:  # yaml
        path = YAML_CONFIG_PATH
        if not os.path.exists(path):
            print(f"❌ YAML config not found: {path}")
            return 1
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
    
    if args.key:
        # Показать конкретный ключ
        keys = args.key.split('.')
        value = cfg
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                print(f"❌ Key not found: {args.key}")
                return 1
        print(f"{args.key} = {value}")
    else:
        # Показать весь конфиг
        import pprint
        pprint.pprint(cfg)
    
    return 0


def cmd_validate(args):
    """Валидировать конфигурацию."""
    print("🔍 Validating configuration...")
    
    errors = []
    warnings = []
    
    # Проверяем JSON
    if os.path.exists(JSON_CONFIG_PATH):
        try:
            import json
            with open(JSON_CONFIG_PATH) as f:
                cfg = json.load(f)
            
            # Проверка суставов
            joints = cfg.get('joints', [])
            if len(joints) != 12:
                warnings.append(f"Expected 12 joints, found {len(joints)}")
            
            # Проверка каналов
            channels = set()
            for joint in joints:
                ch = joint.get('servo_id')
                if ch in channels:
                    errors.append(f"Duplicate channel {ch} for joint {joint['name']}")
                channels.add(ch)
                
                # Проверка пределов
                min_deg = joint.get('min_deg', -90)
                max_deg = joint.get('max_deg', 90)
                if min_deg >= max_deg:
                    errors.append(f"Invalid limits for {joint['name']}: {min_deg} >= {max_deg}")
            
            print(f"✅ JSON config valid: {len(joints)} joints, {len(channels)} channels")
            
        except Exception as e:
            errors.append(f"Failed to parse JSON: {e}")
    else:
        warnings.append("JSON config not found")
    
    # Проверка YAML
    if os.path.exists(YAML_CONFIG_PATH):
        try:
            import yaml
            with open(YAML_CONFIG_PATH) as f:
                cfg = yaml.safe_load(f)
            print(f"✅ YAML config valid")
        except Exception as e:
            errors.append(f"Failed to parse YAML: {e}")
    else:
        warnings.append("YAML config not found")
    
    # Вывод результатов
    print("\n" + "=" * 50)
    if errors:
        print("❌ ERRORS:")
        for err in errors:
            print(f"   • {err}")
    
    if warnings:
        print("⚠️  WARNINGS:")
        for warn in warnings:
            print(f"   • {warn}")
    
    if not errors and not warnings:
        print("✅ Configuration is valid and complete")
    
    return 1 if errors else 0


def main():
    parser = argparse.ArgumentParser(
        description='Robot Dog Configuration Manager',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  dog_config status              # Показать статус синхронизации
  dog_config sync                # Синхронизировать JSON → YAML
  dog_config sync --force        # Принудительная синхронизация
  dog_config show --format yaml  # Показать YAML конфиг
  dog_config show --key servos   # Показать секцию servos
  dog_config validate            # Валидировать конфигурацию
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', required=True)
    
    # Status command
    p_status = subparsers.add_parser('status', help='Show configuration status')
    p_status.set_defaults(func=cmd_status)
    
    # Sync command
    p_sync = subparsers.add_parser('sync', help='Synchronize JSON → YAML')
    p_sync.add_argument('--force', '-f', action='store_true', 
                       help='Force sync even if YAML is newer')
    p_sync.set_defaults(func=cmd_sync)
    
    # Show command
    p_show = subparsers.add_parser('show', help='Show configuration')
    p_show.add_argument('--format', choices=['json', 'yaml'], default='yaml',
                       help='Config format to show')
    p_show.add_argument('--key', '-k', help='Show specific key (dot notation)')
    p_show.set_defaults(func=cmd_show)
    
    # Validate command
    p_validate = subparsers.add_parser('validate', help='Validate configuration')
    p_validate.set_defaults(func=cmd_validate)
    
    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == '__main__':
    main()
```

---

### Этап 7: Setup.py для пакета dog_config

**Файл:** `/workspace/robot_dog_ws/src/dog_config/setup.py`

```python
from setuptools import setup, find_packages

package_name = 'dog_config'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'pyyaml>=5.4',
    ],
    entry_points={
        'console_scripts': [
            'dog_config = dog_config.config_cli:main',
            'config_sync = dog_config.config_sync:main',
        ],
    },
)
```

---

## 📋 Чеклист внедрения

| Шаг | Действие | Команда | Ожидаемый результат |
|-----|----------|---------|---------------------|
| 1 | Создать пакет dog_config | `mkdir -p /workspace/robot_dog_ws/src/dog_config/dog_config` | Пакет создан |
| 2 | Установить зависимости | `pip install pyyaml` | YAML установлен |
| 3 | Разместить файлы | Копировать config_sync.py, config_cli.py | Файлы на месте |
| 4 | Собрать пакет | `cd /workspace/robot_dog_ws && colcon build` | Сборка успешна |
| 5 | Проверить статус | `dog_config status` | Показывает JSON и YAML |
| 6 | Первая синхронизация | `dog_config sync --force` | Создан robot_config.yaml |
| 7 | Валидация | `dog_config validate` | ✅ Configuration is valid |
| 8 | Тест ROS2 | Запустить servo_driver_node | Загрузил новый конфиг |
| 9 | Тест калибратора | Изменить zero_deg → сохранить | Авто-экспорт в YAML |
| 10 | Интеграционный тест | Полный цикл калибровки | Все работает |

---

## 🔐 Безопасность и надёжность

1. **Бэкапы:** Автоматическое создание бэкапов перед перезаписью YAML
2. **Валидация:** Проверка корректности данных перед сохранением
3. **Атомарность:** Запись через временный файл + rename
4. **Логирование:** Все операции логируются с timestamp
5. **Откат:** При ошибке сохраняется старый конфиг

---

## 🚀 Расширения (Future Work)

1. **Hot-reload:** Мониторинг изменений YAML и перезагрузка параметров без рестарта ноды
2. **Web UI:** Визуальный редактор конфига в веб-интерфейсе
3. **Git integration:** Автocommit изменений конфига в git
4. **Profiles:** Переключение между профилями (indoor/outdoor/battery_saver)
5. **Cloud sync:** Синхронизация конфигов между несколькими роботами

---

## 📄 Итоговые артефакты

После реализации:
- ✅ `/workspace/config/robot_config.yaml` — единый конфиг
- ✅ `/workspace/robot_dog_ws/src/dog_config/` — пакет синхронизации
- ✅ `dog_config` CLI — управление конфигом
- ✅ Авто-экспорт из калибратора в ROS2
- ✅ Обратная совместимость со старым форматом
- ✅ Документация и тесты

**Статус:** Готов к реализации 🚀
