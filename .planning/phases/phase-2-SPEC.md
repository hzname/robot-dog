# Phase 2 — Configuration Sync (SPEC.md)

**Project:** RobotDogQwen  
**Phase:** 2 — Configuration Sync  
**Created:** 2026-05-12  
**Status:** Planning  
**Goal:** После калибровки в web calibrator параметры автоматически конвертируются в формат ROS2.

---

## 1. Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     HOST (Banana Pi)                        │
│                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │ calibrate_web│───▶│ servo_config │───▶│ config_sync   │  │
│  │  :8081       │    │  .json       │    │   (converter) │  │
│  │ FastAPI +    │    │  (source of  │    │               │  │
│  │ PCA9685 I2C  │    │  truth)      │    │ JSON → YAML   │  │
│  └──────┬───────┘    └──────┬───────┘    └───────┬───────┘  │
│         │                   │                     │          │
│         │           POST /api/cal/save       sync_configs() │
│         │                   │                     │          │
│         │                   │                     ▼          │
│         │                   │            ┌───────────────┐   │
│         │                   │            │ servo_config  │   │
│         │                   │            │ .yaml         │   │
│         │                   │            │ (ROS2 config) │   │
│         │                   │            └───────┬───────┘   │
│         │                   │                    │           │
│         │                   │              ROS2 launch      │
│         │                   │                    │           │
│         │                   │              ┌───▼────────┐    │
│         │                   │              │servo_driver│    │
│         │                   │              │_node.py    │    │
│         │                   │              └─────┬──────┘    │
│         │                   │                    │           │
│         │                   │              ┌─────▼──────┐    │
│         │                   │              │gait_ctrl   │    │
│         │                   │              │_node (C++) │    │
│         │                   │              └─────┬──────┘    │
│         │                   │                    │           │
│         │                   │              ┌─────▼──────┐    │
│         │                   │              │balance_ctrl│    │
│         │                   │              │_node (C++) │    │
│         │                   │              └────────────┘    │
│         │                   │                                │
│  ┌──────▼───────┐          │           DOCKER CONTAINER      │
│  │ dog_web      │          │         (robot_dog)             │
│  │  :8080       │──────────┘                                 │
│  │ FastAPI +    │                                            │
│  │ rclpy        │                                            │
│  └──────────────┘                                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 2. Data Flow

### 2.1 Normal Operation (calibration)

```
User opens calibrate_web (port 8081)
  │
  ├── GET /api/cal/status → current servo angles
  ├── GET /api/cal/config → servo_config.json content
  │
  ├── User adjusts servos (slider UI)
  │   └── POST /api/cal/move → {joint_name, angle_deg}
  │       └── movement_loop() → smooth movement to target
  │
  ├── User sets zero position
  │   └── POST /api/cal/set_zero → {joint_name, angle_deg}
  │       └── updates zero_deg in servo_config.json
  │
  ├── User saves calibration
  │   └── POST /api/cal/save → FullConfig
  │       ├── save_cfg(cfg) → writes servo_config.json
  │       └── async: config_sync.sync_configs()
  │           ├── load_servo_config()
  │           ├── json_to_yaml()
  │           ├── validate_config()
  │           └── save_yaml_config()
  │
  └── GET /api/cal/poses → list of preset poses
```

### 2.2 ROS2 Startup

```
docker compose up
  │
  ├── servo_driver.launch.py starts
  │   ├── Launch argument: config_file := servo_config.yaml path
  │   └── ServoDriverNode reads servo_config.yaml
  │       └── ServoConfig objects for 12 servos
  │
  ├── control_launch.py starts
  │   ├── gait_controller lifecycle node
  │   │   └── ROS2 params from launch file (read from servo_config.json)
  │   │       ├── stance_height = _computed.standing_height_m
  │   │       ├── foot_x_scale = body.standing.foot_x_scale
  │   │       ├── foot_y_scale = body.standing.foot_y_scale
  │   │       ├── thigh_length = body.link_lengths_m.thigh
  │   │       ├── calf_length = body.link_lengths_m.calf
  │   │       ├── hip_length = body.link_lengths_m.hip
  │   │       └── control_rate = gait.control_rate_hz
  │   │
  │   └── balance_controller (C++)
  │       └── ROS2 params from launch file
  │           ├── pid_roll_kp, pid_roll_ki, pid_roll_kd
  │           ├── pid_pitch_kp, pid_pitch_ki, pid_pitch_kd
  │           └── pid_height_kp, pid_height_ki, pid_height_kd
  │
  └── dog_web_node starts
      └── ROS2 params for calibration JSON path
```

## 3. Interface Specifications

### 3.1 config_sync.py — Public API

```python
class SyncResult:
    """Result of a sync operation."""
    yaml_path: str       # Path to written YAML
    servo_count: int     # Number of servos written
    backup_path: str     # Path to backup (if created)
    errors: list[str]    # Validation errors (empty if valid)
    warnings: list[str]  # Non-fatal warnings
    duration_s: float    # Time taken

def load_servo_config(path: str = "/home/sg/robot-dog/servo_config.json") -> dict:
    """Load and parse servo_config.json. Returns raw dict."""
    ...

def json_to_yaml(json_cfg: dict) -> dict:
    """
    Convert servo_config.json structure → servo_config.yaml structure.
    
    Transformations:
    - joints.list[i] → servos[i]
    - joints.{type}.{field} → base servo defaults
    - angle_rad → angle_deg (÷ π × 180)
    - sign → inverted (sign < 0 → inverted = True)
    - _computed → kinematics.standing_height_m
    
    Returns: dict ready for YAML serialization
    """
    ...

def yaml_to_json(yaml_cfg: dict) -> dict:
    """
    Reverse conversion: YAML → JSON structure.
    Used for validation and manual editing.
    
    Returns: dict compatible with servo_config.json format
    """
    ...

def validate_config(yaml_cfg: dict) -> SyncResult:
    """
    Validate the YAML configuration.
    
    Checks:
    - 12 main servos + 4 aux servos present
    - servo_id/channel is unique (0-15)
    - All servo names valid (match ROS joint names)
    - Angles within reasonable ranges
    - Inverted flags consistent with leg assignment
    """
    ...

def sync_configs(
    json_path: str = "/home/sg/robot-dog/servo_config.json",
    yaml_path: str = "/home/sg/robot-dog/robot_dog_ws/src/dog_hardware/config/servo_config.yaml",
    backup: bool = True
) -> SyncResult:
    """
    Full sync pipeline: load JSON → convert → validate → save YAML.
    
    Creates a timestamped backup of existing YAML before overwriting.
    Thread-safe: uses threading.Lock.
    """
    ...

def main():
    """CLI entry point."""
    ...
```

### 3.2 Calibrator Integration

Changes to `calibrate_web.py`:

```python
# NEW: Import config sync
try:
    from dog_config.config_sync import sync_configs
    HAS_CONFIG_SYNC = True
except ImportError:
    HAS_CONFIG_SYNC = False

# NEW: Track sync status on app state
app.state.last_sync_result = None
app.state.last_sync_error = None

# NEW: Background sync function
def _trigger_config_sync():
    """Run config sync in background thread."""
    if not HAS_CONFIG_SYNC:
        app.state.last_sync_error = "dog_config package not installed"
        return
    try:
        result = sync_configs(
            json_path=CONFIG_PATH,
            yaml_path=YAML_CONFIG_PATH  # new constant
        )
        app.state.last_sync_result = result
        app.logger.info(f"Config sync complete: {result.servo_count} servos, "
                       f"{len(result.errors)} errors")
    except Exception as e:
        app.logger.error(f"Config sync failed: {e}")
        app.state.last_sync_error = str(e)

# MODIFIED: POST /api/cal/save
@app.post("/api/cal/save")
def save_full(cfg: FullConfig):
    save_cfg(cfg.model_dump())
    if HAS_CONFIG_SYNC:
        threading.Thread(target=_trigger_config_sync, daemon=True).start()
    return {"status": "ok"}

# MODIFIED: POST /api/cal/config/replace
@app.post("/api/cal/config/replace")
def replace_full(cfg: FullConfig):
    save_cfg(cfg.model_dump())
    if HAS_CONFIG_SYNC:
        threading.Thread(target=_trigger_config_sync, daemon=True).start()
    return {"status": "ok"}

# MODIFIED: POST /api/cal/set_zero
@app.post("/api/cal/set_zero")
def set_zero(data: ServoCmd):
    # ... existing logic ...
    j['zero_deg'] = round(cur + j['zero_deg'], 2)
    save_cfg(cfg)
    # Trigger sync for zero update
    if HAS_CONFIG_SYNC:
        threading.Thread(target=_trigger_config_sync, daemon=True).start()
    return {"status": "ok", "zero_deg": j['zero_deg']}

# NEW: Status endpoint for sync info
@app.get("/api/cal/sync")
def sync_status():
    return {
        "configured": HAS_CONFIG_SYNC,
        "last_result": getattr(app.state, 'last_sync_result', None),
        "last_error": getattr(app.state, 'last_sync_error', None)
    }
```

New constant:
```python
YAML_CONFIG_PATH = "/home/sg/robot-dog/robot_dog_ws/src/dog_hardware/config/servo_config.yaml"
```

### 3.3 YAML Output Format

```yaml
# servo_config.yaml — ROS2 configuration
# Auto-generated by config_sync.py from servo_config.json
# Do not edit manually — use web calibrator or `dog_config sync`

_metadata:
  source: servo_config.json
  generated: "2026-05-12T15:30:00Z"
  config_version: "2.0"
  converter: dog_config.config_sync

pca9685:
  i2c_bus: 0
  i2c_address: 0x40
  pwm_frequency: 50

# 12 main servos (channels 0-11)
servos:
  - name: "lf_hip_joint"
    channel: 0
    leg: "LF"
    joint_type: "hip"
    min_angle: -45.84      # from joints.hip.min_rad (-0.8 rad)
    max_angle: 45.84       # from joints.hip.max_rad (0.8 rad)
    min_pulse: 520.0
    max_pulse: 2220.0
    inverted: false        # derived from joints.list[0].inverted
    offset: 0.0            # from joints.hip.neutral_rad (0.0)
    max_speed: 10.0
    max_torque: 10.0

  - name: "lf_thigh_joint"
    channel: 1
    leg: "LF"
    joint_type: "thigh"
    min_angle: -85.94      # from joints.thigh.min_rad (-1.5 rad)
    max_angle: 85.94       # from joints.thigh.max_rad (1.5 rad)
    min_pulse: 520.0
    max_pulse: 2220.0
    inverted: false
    offset: 0.0
    max_speed: 10.0
    max_torque: 15.0

  - name: "lf_calf_joint"
    channel: 2
    leg: "LF"
    joint_type: "calf"
    min_angle: -143.24     # from joints.calf.min_rad (-2.5 rad)
    max_angle: -28.65      # from joints.calf.max_rad (-0.5 rad)
    min_pulse: 520.0
    max_pulse: 2220.0
    inverted: false
    offset: -68.75         # from joints.calf.neutral_rad (-1.2 rad)
    max_speed: 10.0
    max_torque: 15.0

  # ... (rf_hip_joint through rr_calf_joint, same pattern)
  # Right legs (RF, RR) have inverted: true

# Kinematics
kinematics:
  body:
    length_m: 0.35
    width_m: 0.2
    mass_kg: 2.5
  link_lengths_m:
    hip: 0.045
    thigh: 0.11
    calf: 0.11
  standing:
    nominal_calf_angle_rad: -1.2
    nominal_thigh_angle_rad: 0.0
    nominal_hip_angle_rad: 0.0
    foot_x_scale: 0.5
    foot_y_scale: 0.8
    height_min_ratio: 0.7
    height_max_ratio: 1.3
    standing_height_m: 0.226574    # from _computed
    min_height_m: 0.158602
    max_height_m: 0.294546

# Gait parameters
gait:
  control_rate_hz: 100.0
  velocity_deadzone: 0.01
  balance_enabled: true
  balance_response_factor: 1.0
  default_type: "trot"
  trot:
    period_s: 0.5
    step_height_m: 0.08
    step_length_scale: 0.5
    foot_x_scale: 0.5
    foot_y_scale: 0.8
    turn_gain: 0.05
    cog_pitch_gain_m_per_rad: 0.02
    cog_roll_gain_m_per_rad: 0.02

# Balance PID
balance:
  control_rate_hz: 100.0
  max_correction_rate: 0.5
  target_pose_clamp_rad: 0.3
  pid:
    roll: {kp: 0.15, ki: 0.01, kd: 0.05}
    pitch: {kp: 0.15, ki: 0.01, kd: 0.05}
    height: {kp: 1.0, ki: 0.1, kd: 0.2}

# Servo driver
servo_driver:
  bus_type: "simulation"
  device_port: "/dev/i2c-1"
  pwm_frequency_hz: 50
  servo_count: 12
  control_rate_hz: 100.0
  watchdog_timeout_s: 0.5
  enable_watchdog: true
  safety_enabled: true
```

### 3.4 C++ Parameter Mapping

| ROS2 param name | Default (C++) | Source (servo_config.json) | Example value |
|-----------------|---------------|---------------------------|---------------|
| `stance_height` | 0.25 | body.standing._computed.standing_height_m | 0.226574 |
| `step_height` | 0.08 | gait.gaits.trot.step_height_m | 0.08 |
| `gait_period` | 0.5 | gait.gaits.trot.period_s | 0.5 |
| `body_length` | 0.35 | body.length_m | 0.35 |
| `body_width` | 0.20 | body.width_m | 0.20 |
| `hip_length` | 0.045 | body.link_lengths_m.hip | 0.045 |
| `thigh_length` | 0.11 | body.link_lengths_m.thigh | 0.11 |
| `calf_length` | 0.11 | body.link_lengths_m.calf | 0.11 |
| `control_rate` | 100.0 | gait.control_rate_hz | 100.0 |
| `foot_x_scale` | 0.5 (hardcoded) | body.standing.foot_x_scale | 0.5 |
| `foot_y_scale` | 0.8 (hardcoded) | body.standing.foot_y_scale | 0.8 |
| `turn_gain` | 0.05 (hardcoded) | gait.gaits.trot.turn_gain | 0.05 |
| `cog_pitch_gain` | 0.02 (hardcoded) | gait.gaits.trot.cog_pitch_gain_m_per_rad | 0.02 |
| `cog_roll_gain` | 0.02 (hardcoded) | gait.gaits.trot.cog_roll_gain_m_per_rad | 0.02 |
| `hip_length` | 0.045 (hardcoded) | body.link_lengths_m.hip | 0.045 |

## 4. Conversion Logic Details

### 4.1 rad ↔ deg Conversion

```python
RAD_TO_DEG = 180.0 / math.pi  # ≈ 57.2957795

def rad_to_deg(rad):
    """Convert radians to degrees."""
    return round(rad * RAD_TO_DEG, 4)

def deg_to_rad(deg):
    """Convert degrees to radians."""
    return round(deg / RAD_TO_DEG, 6)
```

### 4.2 inverted ↔ sign Mapping

```
calibrator `sign` field     ROS2 `inverted`
                    ──────────────────────
    sign = +1              →  inverted = false
    sign = -1              →  inverted = true
    inverted = true        →  sign = -1
    inverted = false       →  sign = +1
```

### 4.3 servo_id ↔ channel

Direct mapping: `servo_id` in JSON = `channel` in YAML.

### 4.4 offset Calculation

The YAML `offset` represents the neutral position deviation from mechanical zero.

```python
# From servo_config.json:
# joints.{type}.neutral_rad → the expected neutral angle in radians
# Calibrator zero_deg → the measured offset from mechanical zero

# For YAML, the offset is:
offset_deg = neutral_rad * RAD_TO_DEG
# This means at offset=0, the servo sits at neutral_rad
```

## 5. Verification Plan

### 5.1 Unit Tests

```python
# tests/test_config_sync.py

def test_rad_to_deg():
    assert rad_to_deg(0.0) == 0.0
    assert rad_to_deg(0.8) == 45.8366  # hip.max_rad
    assert rad_to_deg(-1.5) == -85.9437  # thigh.min_rad

def test_inverted_mapping():
    # sign +1 → inverted False
    assert map_sign_to_inverted(1) == False
    # sign -1 → inverted True
    assert map_sign_to_inverted(-1) == True

def test_json_to_yaml_servos():
    cfg = load_servo_config()
    yaml = json_to_yaml(cfg)
    assert len(yaml['servos']) == 12
    assert yaml['servos'][0]['name'] == 'lf_hip_joint'
    assert yaml['servos'][0]['channel'] == 0
    assert yaml['servos'][0]['inverted'] == False
    assert yaml['servos'][3]['name'] == 'rf_hip_joint'
    assert yaml['servos'][3]['inverted'] == True  # right leg
    assert yaml['servos'][2]['min_angle'] == -143.24  # calf.min_rad → deg

def test_yaml_to_json_roundtrip():
    cfg = load_servo_config()
    yaml = json_to_yaml(cfg)
    cfg_back = yaml_to_json(yaml)
    # Key fields must match
    assert cfg_back['joints']['list'][0]['name'] == 'lf_hip_joint'
    assert cfg_back['joints']['list'][11]['servo_id'] == 11

def test_validate_valid():
    cfg = load_servo_config()
    yaml = json_to_yaml(cfg)
    result = validate_config(yaml)
    assert len(result.errors) == 0

def test_validate_missing_servo():
    yaml = {"servos": []}  # empty
    result = validate_config(yaml)
    assert len(result.errors) >= 1

def test_computed_values():
    cfg = load_servo_config()
    yaml = json_to_yaml(cfg)
    assert yaml['kinematics']['standing']['standing_height_m'] == pytest.approx(0.226574, abs=0.0001)
    assert yaml['kinematics']['standing']['min_height_m'] == pytest.approx(0.158602, abs=0.0001)
    assert yaml['kinematics']['standing']['max_height_m'] == pytest.approx(0.294546, abs=0.0001)
```

### 5.2 Integration Tests

```python
# tests/test_integration.py

def test_sync_creates_yaml():
    """config_sync creates a valid YAML file."""
    result = sync_configs(backup=False)
    assert result.servo_count == 12
    assert result.errors == []
    assert os.path.exists(YAML_CONFIG_PATH)
    
    with open(YAML_CONFIG_PATH) as f:
        yaml = yaml.safe_load(f)
    assert len(yaml['servos']) == 12

def test_sync_updates_yaml():
    """After modifying JSON, YAML reflects changes."""
    # Modify a joint in JSON
    cfg = load_servo_config()
    cfg['joints']['list'][0]['zero_deg'] = 5.5
    save_servo_config(cfg)
    
    sync_configs(backup=False)
    
    with open(YAML_CONFIG_PATH) as f:
        yaml = yaml.safe_load(f)
    assert yaml['servos'][0]['offset'] == pytest.approx(0.0, abs=0.1)  # depends on mapping

def test_calibrator_trigger_sync():
    """POST /api/cal/save triggers config sync."""
    # Run calibrate_web in test mode
    # POST /api/cal/save
    # Check that YAML file is updated within 1 second
    ...

def test_ros2_read_yaml():
    """ROS2 servo_driver reads correct values from YAML."""
    # Start servo_driver with YAML path
    # Check that ServoConfig objects have correct values
    ...
```

### 5.3 Quality Gates

| Gate | Command | Pass Criteria |
|------|---------|---------------|
| colcon build | `colcon build --packages-select dog_config dog_hardware` | 0 errors |
| Unit tests | `python3 -m pytest tests/test_config_sync.py -v` | All pass |
| Import check | `python3 -c "from dog_config.config_sync import sync_configs"` | No ImportError |
| YAML valid | `python3 -c "import yaml; yaml.safe_load(open(YAML_PATH))"` | No YAML error |
| 12 servos | `python3 -c "import yaml; s=yaml.safe_load(open(YAML_PATH)); assert len(s['servos'])==12"` | No AssertionError |
| Inverted check | Right legs have inverted=True | Assertion passes |
| FK check | standing_height matches _computed | diff < 0.001 |

## 6. Error Handling

### 6.1 config_sync.py

| Error | Severity | Action |
|-------|----------|--------|
| JSON file not found | Fatal | Log error, return SyncResult with error message |
| JSON parse error | Fatal | Log error with details, return SyncResult |
| Missing joint in list | Validation error | Log warning, include in SyncResult.errors |
| Duplicate servo_id | Fatal | Log error, abort sync |
| Invalid angle range | Validation warning | Log warning, still write |
| YAML write failure | Fatal | Log error, keep old YAML (backup intact) |
| Backup creation failure | Warning | Continue, log warning |

### 6.2 Calibrator Integration

| Error | Handling |
|-------|----------|
| dog_config not installed | `HAS_CONFIG_SYNC = False`, sync skipped silently |
| Sync thread crashes | Logged to stderr, does NOT affect calibrator functionality |
| Long sync (>5s) | Logged as warning, calibrator still responds |

## 7. File Changes Summary

| File | Change | Type |
|------|--------|------|
| `robot_dog_ws/src/dog_config/package.xml` | **NEW** — ROS2 package manifest | New |
| `robot_dog_ws/src/dog_config/setup.py` | **NEW** — Python package setup | New |
| `robot_dog_ws/src/dog_config/dog_config/__init__.py` | **NEW** — Package init | New |
| `robot_dog_ws/src/dog_config/dog_config/config_sync.py` | **NEW** — Main converter logic | New |
| `robot_dog_ws/src/dog_config/tests/test_config_sync.py` | **NEW** — Unit tests | New |
| `calibrate_web.py` | Add config_sync import, sync trigger, /api/cal/sync endpoint | Modified |
| `robot_dog_ws/src/dog_hardware/config/servo_config.yaml` | **REGENERATED** by config_sync.py | Regenerated |
| `robot_dog_ws/src/dog_hardware/launch/servo_driver.launch.py` | Update config_file path if needed | Modified |
| `robot_dog_ws/src/dog_control_cpp/launch/control_launch.py` | Update params from servo_config.json | Modified |

## 8. Migration Notes

### Backward Compatibility

1. **servo_config.json is NOT modified** — it remains the single source of truth
2. **servo_config.yaml is auto-generated** — can be deleted and regenerated at any time
3. **Calibrator keeps working** — if dog_config is not installed, sync is silently skipped
4. **ROS2 nodes fall back** — if YAML doesn't exist, servo_driver uses defaults (as now)

### Rollback Plan

If Phase 2 introduces issues:
1. Revert `calibrate_web.py` changes
2. Delete `dog_config` package (COLCON_IGNORE)
3. Restore `servo_config.yaml` from git
4. No data loss — servo_config.json unchanged

---

*Spec created: 2026-05-12*
*Reviewed: Pending*
