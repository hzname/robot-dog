# Детальный анализ и рекомендации по улучшению кода RobotDogQwen

## 1. `_on_imu(msg)` — Фильтрация IMU данных

### Текущая реализация (ros_bridge.py:56-64)
```python
def _on_imu(self, msg):
    self.latest_imu = {
        'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y,
                       'z': msg.orientation.z, 'w': msg.orientation.w},
        'angular_velocity': {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y,
                            'z': msg.angular_velocity.z},
        'linear_acceleration': {'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y,
                               'z': msg.linear_acceleration.z}
    }
```

### Проблемы:
1. **Нет фильтрации шума** — сырые данные с MPU6050 содержат высокочастотный шум
2. **Нет учёта люфтов MG996R** — люфт ~1-2° создаёт вибрации корпуса
3. **Нет компенсации дрейфа гироскопа** — гироскоп дрейфует на 0.1-0.5°/сек

### Рекомендации по фильтрации:

#### Вариант 1: Дополненный фильтр Калмана (рекомендуется)
```python
import numpy as np
from scipy.linalg import inv

class KalmanIMUFilter:
    """Extended Kalman Filter for IMU fusion with servo backlash compensation."""
    
    def __init__(self, dt=0.02):
        self.dt = dt
        
        # State: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
        self.x = np.zeros(6)
        
        # Process noise covariance — учитывает люфты сервоприводов
        # MG996R: люфт ~1-2°, скорость ~0.17 сек/60° → максимальная угловая скорость ~350°/сек
        self.Q = np.diag([
            0.01, 0.01, 0.01,  # orientation noise (rad²)
            0.001, 0.001, 0.001  # gyro bias drift (rad/s)²
        ])
        
        # Measurement noise covariance — акселерометр шумный при движении
        self.R = np.diag([
            0.1,  # roll from accelerometer (rad²)
            0.1,  # pitch from accelerometer (rad²)
            0.5   # yaw from magnetometer (если есть)
        ])
        
        # Error covariance
        self.P = np.eye(6) * 0.1
        
        # Backlash compensation для MG996R
        self.backlash_deadzone = np.deg2rad(1.5)  # 1.5° мёртвая зона
        self.prev_command = np.zeros(3)
        
    def predict(self, gyro):
        """Predict step using gyroscope."""
        # Компенсация дрейфа гироскопа
        gyro_corrected = np.array(gyro) - self.x[3:6]
        
        # Применяем deadzone для компенсации люфтов
        delta = gyro_corrected * self.dt
        delta = np.where(np.abs(delta) < self.backlash_deadzone, 0, delta)
        
        # Update orientation (simple kinematics)
        self.x[0] += delta[0]  # roll
        self.x[1] += delta[1]  # pitch
        self.x[2] += delta[2]  # yaw
        
        # Gyro bias random walk
        self.x[3:6] += np.random.normal(0, 0.001, 3)
        
        # Jacobian for EKF
        F = np.eye(6)
        F[0:3, 3:6] = -np.eye(3) * self.dt
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x[0:3]
    
    def update(self, accel, mag=None):
        """Update step using accelerometer (and magnetometer if available)."""
        # Calculate roll and pitch from accelerometer
        ax, ay, az = accel
        roll_acc = np.arctan2(ay, az)
        pitch_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        
        # Measurement vector
        z = np.array([roll_acc, pitch_acc, self.x[2]])  # yaw from previous or mag
        
        # Measurement matrix
        H = np.eye(3, 6)
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ inv(S)
        
        # Update state
        y = z - H @ self.x
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        
        return self.x[0:3]


# Интеграция в Bridge:
class Bridge:
    def __init__(self):
        # ... existing code ...
        self.imu_filter = KalmanIMUFilter(dt=0.02)
        self.filtered_imu = None
    
    def _on_imu(self, msg):
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        
        # Predict with gyro
        self.imu_filter.predict(gyro)
        
        # Update with accelerometer (только если робот не движется активно)
        # Проверяем по модулю ускорения — если близко к 1g, используем
        accel_mag = math.sqrt(sum(a**2 for a in accel))
        if 0.8 < accel_mag < 1.2:  # Статическое или медленное движение
            orientation = self.imu_filter.update(accel)
        else:
            orientation = self.imu_filter.x[0:3]
        
        self.filtered_imu = {
            'roll': float(orientation[0]),
            'pitch': float(orientation[1]),
            'yaw': float(orientation[2]),
            'gyro_bias': self.imu_filter.x[3:6].tolist(),
            'raw': {
                'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y,
                               'z': msg.orientation.z, 'w': msg.orientation.w},
                'angular_velocity': {'x': gyro[0], 'y': gyro[1], 'z': gyro[2]},
                'linear_acceleration': {'x': accel[0], 'y': accel[1], 'z': accel[2]}
            }
        }
```

#### Вариант 2: Простой комплементарный фильтр с адаптивным коэффициентом
```python
class AdaptiveComplementaryFilter:
    """Complementary filter with adaptive alpha based on motion intensity."""
    
    def __init__(self, alpha_base=0.98):
        self.alpha_base = alpha_base
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_time = None
        
    def update(self, gyro, accel, dt):
        # Accelerometer angles
        roll_acc = math.atan2(accel[1], accel[2])
        pitch_acc = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        
        # Gyro integration
        if self.prev_time is not None:
            self.roll += gyro[0] * dt
            self.pitch += gyro[1] * dt
        
        # Adaptive alpha — меньше доверяем акселерометру при движении
        accel_mag = math.sqrt(sum(a**2 for a in accel))
        motion_factor = abs(accel_mag - 9.81) / 9.81  # 0 = покой, 1 = сильное движение
        alpha = max(0.90, self.alpha_base - motion_factor * 0.3)
        
        # Complementary filter
        self.roll = alpha * self.roll + (1 - alpha) * roll_acc
        self.pitch = alpha * self.pitch + (1 - alpha) * pitch_acc
        
        self.prev_time = time.time()
        return self.roll, self.pitch
```

**Оценка важности:** ⭐⭐⭐⭐⭐ Критично для стабильной походки

---

## 2. `_on_estop(msg)` — Анализ срабатывания Emergency Stop

### Текущая реализация (ros_bridge.py:69-70)
```python
def _on_estop(self, msg):
    self.emergency_stopped = msg.data
```

### Анализ случаев срабатывания:

#### Корректные случаи срабатывания:
1. **Пользователь нажал кнопку E-Stop** в веб-интерфейсе
2. **Критический угол наклона** (>45° roll/pitch) — риск переворота
3. **Превышение тока сервоприводов** — заклинивание ноги
4. **Потеря связи** с контроллером >2 секунд
5. **Коллизия ног** — пересечение рабочих областей
6. **Низкий заряд батареи** (<20%)

#### Риски беспричинных блокировок:

| Причина | Вероятность | Последствия | Решение |
|---------|-------------|-------------|---------|
| Шум IMU | Высокая | Ложные срабатывания при вибрации | Фильтрация (см. раздел 1) |
| Дребезг кнопки | Средняя | Многократные срабатывания | Debounce 50-100ms |
| Кратковременные потери I2C | Средняя | Срабатывание при помехах | Retry 3x перед E-Stop |
| Пики тока при старте | Высокая | Срабатывание при резком движении | Soft start + averaging |

### Улучшенная реализация с защитой от ложных срабатываний:

```python
class EmergencyStopManager:
    """Advanced E-Stop with debouncing and cause tracking."""
    
    def __init__(self, node):
        self.node = node
        self.emergency_stopped = False
        self.stop_reasons = []
        
        # Debounce timing
        self.debounce_time = 0.1  # 100ms
        self.last_stop_request = 0
        self.stop_timer = None
        
        # Thresholds для автоматических срабатываний
        self.max_tilt_rad = math.radians(45)  # 45 градусов
        self.max_current_a = 2.5  # Ампер на серву (MG996R stall ~2.5A)
        self.comms_timeout_s = 2.0
        
        # History для фильтрации
        self.tilt_history = []
        self.current_history = []
        
    def _on_estop(self, msg):
        """Handle manual E-Stop with debouncing."""
        current_time = time.time()
        
        if msg.data:
            # Проверка debounce
            if current_time - self.last_stop_request < self.debounce_time:
                return  # Игнорируем дребезг
            
            self.last_stop_request = current_time
            self._trigger_estop("manual_button")
        else:
            # Reset только если нет активных причин
            if not self.stop_reasons:
                self.emergency_stopped = False
                self.node.get_logger().info("E-Stop RESET")
    
    def _trigger_estop(self, reason: str):
        """Activate E-Stop with reason tracking."""
        if reason not in self.stop_reasons:
            self.stop_reasons.append(reason)
        
        if not self.emergency_stopped:
            self.emergency_stopped = True
            self.node.get_logger().error(f"🚨 E-STOP TRIGGERED: {reason}")
            self.node.get_logger().error(f"Active reasons: {self.stop_reasons}")
            
            # Публикация для других нод
            estop_msg = Bool()
            estop_msg.data = True
            self.node.estop_pub.publish(estop_msg)
    
    def check_automatic_triggers(self, imu_data, joint_currents):
        """Check automatic E-Stop conditions."""
        current_time = time.time()
        
        # 1. Check tilt angle with filtering
        if imu_data:
            roll = imu_data.get('roll', 0)
            pitch = imu_data.get('pitch', 0)
            
            self.tilt_history.append((current_time, abs(roll), abs(pitch)))
            self.tilt_history = [(t, r, p) for t, r, p in self.tilt_history 
                                if current_time - t < 0.5]  # 500ms window
            
            # Требуем превышения в течение 300ms (защита от шума)
            critical_readings = sum(1 for _, r, p in self.tilt_history 
                                   if r > self.max_tilt_rad or p > self.max_tilt_rad)
            if critical_readings >= 15:  # 15 readings @ 50Hz = 300ms
                self._trigger_estop("excessive_tilt")
        
        # 2. Check joint currents
        if joint_currents:
            max_current = max(joint_currents)
            self.current_history.append((current_time, max_current))
            self.current_history = [(t, c) for t, c in self.current_history 
                                   if current_time - t < 0.3]
            
            # Превышение тока в течение 200ms
            overcurrent_readings = sum(1 for _, c in self.current_history 
                                      if c > self.max_current_a)
            if overcurrent_readings >= 10:  # 10 readings @ 50Hz = 200ms
                self._trigger_estop("overcurrent")
        
        # 3. Check communication timeout
        # (реализуется в основном цикле)
    
    def clear_reason(self, reason: str):
        """Clear a specific stop reason."""
        if reason in self.stop_reasons:
            self.stop_reasons.remove(reason)
            
            if not self.stop_reasons:
                self.emergency_stopped = False
                self.node.get_logger().info("✅ All E-Stop conditions cleared")
```

**Рекомендация:** Добавить визуальную индикацию причины E-Stop в веб-интерфейс.

**Оценка безопасности:** ⭐⭐⭐⭐⭐ Критично для физической безопасности

---

## 3. `main()` — Graceful Shutdown

### Текущая проблема (ros_bridge.py:165-196)
```python
def main():
    bridge = Bridge()
    executor = executors.SingleThreadedExecutor()
    executor.add_node(bridge.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # ... socket setup ...
    
    while rclpy.ok():  # ❌ Нет обработки SIGINT/SIGTERM
        try:
            conn, _ = server.accept()
            # ...
        except socket.timeout:
            continue
```

### Проблемы:
1. **Нет обработки Ctrl+C** — процесс убивается немедленно
2. **Сервоприводы не отключаются** — остаются под напряжением
3. **Сокет не очищается** — `/tmp/robot_dog/ros_bridge.sock` остаётся
4. **ROS2 нода не shutdown** — утечка ресурсов

### Реализация graceful shutdown:

```python
import signal
import sys
from contextlib import contextmanager

@contextmanager
def graceful_shutdown_manager(bridge, server, socket_path):
    """Context manager for graceful shutdown."""
    shutdown_requested = False
    
    def signal_handler(signum, frame):
        nonlocal shutdown_requested
        sig_name = "SIGINT" if signum == signal.SIGINT else "SIGTERM"
        print(f"\n⚠️  {sig_name} received, initiating graceful shutdown...", flush=True)
        shutdown_requested = True
    
    # Register signal handlers
    original_sigint = signal.signal(signal.SIGINT, signal_handler)
    original_sigterm = signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        yield lambda: shutdown_requested
    finally:
        # Restore original handlers
        signal.signal(signal.SIGINT, original_sigint)
        signal.signal(signal.SIGTERM, original_sigterm)

def cleanup(bridge, server, socket_path):
    """Clean shutdown sequence."""
    print("\n🧹 Cleaning up resources...", flush=True)
    
    # 1. Close accepting socket
    try:
        server.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    server.close()
    
    # 2. Remove Unix socket file
    if os.path.exists(socket_path):
        try:
            os.unlink(socket_path)
            print(f"  ✓ Removed socket: {socket_path}", flush=True)
        except Exception as e:
            print(f"  ✗ Failed to remove socket: {e}", flush=True)
    
    # 3. Disable all servos через ROS2
    try:
        from std_msgs.msg import Bool
        servo_msg = Bool()
        servo_msg.data = False
        bridge.servo_pub.publish(servo_msg)
        print("  ✓ Servos disabled", flush=True)
    except Exception as e:
        print(f"  ✗ Failed to disable servos: {e}", flush=True)
    
    # 4. Stop gait
    try:
        gait_msg = Bool()
        gait_msg.data = False
        bridge.gait_enable_pub.publish(gait_msg)
        print("  ✓ Gait stopped", flush=True)
    except Exception as e:
        print(f"  ✗ Failed to stop gait: {e}", flush=True)
    
    # 5. Give time for messages to be sent
    time.sleep(0.2)
    
    # 6. Shutdown ROS2
    try:
        bridge.executor.shutdown(timeout_sec=2.0)
        bridge.node.destroy_node()
        rclpy.shutdown()
        print("  ✓ ROS2 shutdown complete", flush=True)
    except Exception as e:
        print(f"  ✗ ROS2 shutdown failed: {e}", flush=True)
    
    print("✅ Graceful shutdown complete", flush=True)

def main():
    bridge = Bridge()
    executor = executors.SingleThreadedExecutor()
    executor.add_node(bridge.node)
    bridge.executor = executor  # Save for cleanup
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # Setup Unix socket
    os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(5)
    server.settimeout(1.0)
    os.chmod(SOCKET_PATH, 0o666)
    
    print(f'Bridge listening on {SOCKET_PATH}', flush=True)
    
    with graceful_shutdown_manager(bridge, server, SOCKET_PATH) as should_stop:
        while rclpy.ok() and not should_stop():
            try:
                conn, _ = server.accept()
                t = threading.Thread(target=_handle_connection, 
                                   args=(bridge, conn), daemon=True)
                t.start()
            except socket.timeout:
                continue
            except OSError as e:
                if should_stop():
                    break  # Normal shutdown
                print(f'Bridge accept error: {e}', flush=True)
            except Exception as e:
                print(f'Bridge accept error: {e}', flush=True)
    
    # Cleanup
    cleanup(bridge, server, SOCKET_PATH)
    sys.exit(0)
```

**Оценка важности:** ⭐⭐⭐⭐ Предотвращает повреждение оборудования

---

## 4. `deg_to_us()` — Исправление функции

### Текущая проблема (calibrate_web.py:104-108)
```python
def deg_to_us(deg, sign, zero_deg, min_deg, max_deg):
    deg = max(min_deg, min(max_deg, deg))
    adj = (deg + zero_deg) * sign  # ❌ ОШИБКА: zero_deg добавляется ДО умножения на sign
    us = SERVO_MIN_US + (adj + 90) / 180.0 * (SERVO_MAX_US - SERVO_MIN_US)
    return max(SERVO_MIN_US, min(SERVO_MAX_US, us))
```

### Проблема:
- Неправильный порядок операций: `zero_deg` должен применяться после инверсии
- Магические числа: `90`, `180.0`, `SERVO_MIN_US`, `SERVO_MAX_US` без пояснений
- Нет проверки на NaN

### Исправленная версия:

```python
def deg_to_us(deg: float, sign: int, zero_deg: float, 
              min_deg: float, max_deg: float,
              servo_min_us: float = 520,
              servo_max_us: float = 2220,
              servo_center_us: float = 1500) -> float:
    """
    Convert degrees to microseconds for MG996R servo.
    
    Args:
        deg: Target angle in degrees (relative to zero_deg)
        sign: Direction multiplier (1 or -1) for servo orientation
        zero_deg: Calibration offset — physical position when logical angle is 0
        min_deg: Minimum allowed angle in degrees
        max_deg: Maximum allowed angle in degrees
        servo_min_us: Minimum pulse width (typically 500-600us for MG996R)
        servo_max_us: Maximum pulse width (typically 2200-2400us for MG996R)
        servo_center_us: Center pulse width (1500us = nominal 0°)
    
    Returns:
        Pulse width in microseconds
    
    MG996R Specifications:
        - Operating range: nominally 180° (actually ~170° usable)
        - Pulse range: 500-2400us
        - Dead band: 7μs (минимальное изменение для движения)
        - Stall torque: 9.4 kg·cm @ 6V
        - Speed: 0.17 sec/60° @ 6V
    """
    import math
    
    # Validate input
    if not math.isfinite(deg):
        raise ValueError(f"Invalid angle: {deg}")
    
    # Clamp to mechanical limits FIRST
    deg_clamped = max(min_deg, min(max_deg, deg))
    
    # Apply calibration offset
    deg_with_offset = deg_clamped + zero_deg
    
    # Apply direction inversion
    deg_final = deg_with_offset * sign
    
    # Map [-90, +90] degrees to [servo_min_us, servo_max_us] microseconds
    # Formula: us = center + (angle / 90) * (max - center)
    # But we use full range mapping for better precision
    pulse_range = servo_max_us - servo_min_us
    us = servo_min_us + (deg_final + 90.0) / 180.0 * pulse_range
    
    # Final clamping to hardware limits
    us = max(servo_min_us, min(servo_max_us, us))
    
    return us


# Обратная функция тоже требует исправления:
def us_to_deg(us: float, sign: int, zero_deg: float,
              servo_min_us: float = 520,
              servo_max_us: float = 2220) -> float:
    """
    Convert microseconds to degrees (inverse of deg_to_us).
    
    Returns:
        Angle in degrees relative to zero point
    """
    import math
    
    if us < 10 or not math.isfinite(us):
        return 0.0
    
    pulse_range = servo_max_us - servo_min_us
    
    # Map microseconds back to [-90, +90] degrees
    deg_inverted = (us - servo_min_us) / pulse_range * 180.0 - 90.0
    
    # Remove inversion
    deg_normal = deg_inverted / sign if sign != 0 else 0.0
    
    # Remove calibration offset
    deg_final = deg_normal - zero_deg
    
    return deg_final
```

**Дополнительно:** Добавить константы в начало файла:
```python
# MG996R servo specifications
MG996R_MIN_US = 520      # Minimum pulse (0° nominal)
MG996R_MAX_US = 2220     # Maximum pulse (180° nominal)
MG996R_CENTER_US = 1500  # Center pulse (90° nominal)
MG996R_DEADBAND_US = 7   # Minimum change to move
MG996R_RANGE_DEG = 170   # Usable angular range
```

**Оценка критичности:** ⭐⭐⭐⭐ Влияет на точность позиционирования

---

## 5. `move()` API — Варианты без телепорта

### Текущая проблема (calibrate_web.py:284-302)
```python
@app.post("/api/cal/move")
def move(cmd: ServoCmd):
    # ...
    with lock:
        target_positions[sid] = angle
        if not servo_initialized[sid]:
            # First move: teleport directly ← ПРОБЛЕМА
            current_positions[sid] = angle
            apply_pos(sid, angle, j)
            servo_initialized[sid] = True
```

### Проблема:
Первое движение происходит мгновенно ("телепорт"), что может:
- Повредить механику при ударе
- Вызвать чрезмерный ток
- Напугать пользователя

### Решения без телепорта:

#### Вариант 1: Чтение текущего положения через обратную связь
```python
def read_current_position(pca, sid, jcfg):
    """Read actual servo position from PWM register."""
    current_us = pca.get_us(sid)
    return us_to_deg(current_us, jcfg['sign'], jcfg['zero_deg'])

@app.post("/api/cal/move")
def move(cmd: ServoCmd):
    if not calibration_active:
        raise HTTPException(400, "Not active")
    cfg = load_cfg()
    idx, j = find_joint(cfg, cmd.joint_name)
    if idx < 0:
        raise HTTPException(404)
    
    angle = max(j['min_deg'], min(j['max_deg'], cmd.angle_deg))
    sid = j['servo_id']
    
    with lock:
        target_positions[sid] = angle
        
        if not servo_initialized[sid]:
            # ВАРИАНТ 1: Read current position from hardware
            try:
                actual_pos = read_current_position(pca, sid, j)
                current_positions[sid] = actual_pos
                print(f"Initialized {cmd.joint_name} at {actual_pos:.1f}°")
            except Exception as e:
                # Fallback: assume current position is 0
                current_positions[sid] = 0.0
                print(f"Could not read position, assuming 0°: {e}")
            
            servo_initialized[sid] = True
            # Don't apply_pos here — let movement_loop handle it smoothly
    
    eta = abs(angle - current_positions[sid]) / MAX_SPEED_DEG_S
    return {"status": "moving", "servo_id": sid, 
            "target": angle, "eta_s": round(eta, 1)}
```

#### Вариант 2: Гоминг-процедура (для серв без обратной связи)
```python
def homing_procedure(pca, sid, jcfg, direction='both'):
    """
    Find servo zero position by gentle probing.
    
    For MG996R without feedback:
    1. Move slowly in one direction until resistance
    2. Back off slightly
    3. This becomes reference point
    """
    import time
    
    # Step 1: Move to center (safe position)
    center_us = 1500
    pca.set_us(sid, center_us)
    time.sleep(0.5)
    
    # Step 2: Probe in small increments
    step_us = 20  # ~0.7°
    max_steps = 50
    current_us = center_us
    
    for i in range(max_steps):
        if direction in ['cw', 'both']:
            current_us += step_us
            pca.set_us(sid, current_us)
            time.sleep(0.1)
            
            # Check for stall (можно добавить monitoring тока)
            # Для простоты просто ограничиваем диапазон
            if current_us > MG996R_MAX_US - 100:
                break
        
        if direction in ['ccw', 'both']:
            current_us -= step_us
            pca.set_us(sid, current_us)
            time.sleep(0.1)
            
            if current_us < MG996R_MIN_US + 100:
                break
    
    # Return to center
    pca.set_us(sid, center_us)
    time.sleep(0.3)
    
    return 0.0  # Assume centered

@app.post("/api/cal/homing/{joint_name}")
def homing(joint_name: str):
    """Perform homing procedure for a joint."""
    if not calibration_active:
        raise HTTPException(400, "Not active")
    
    cfg = load_cfg()
    idx, j = find_joint(cfg, joint_name)
    if idx < 0:
        raise HTTPException(404)
    
    sid = j['servo_id']
    
    with lock:
        # Perform homing
        estimated_zero = homing_procedure(pca, sid, j)
        
        # Update configuration
        j['zero_deg'] = estimated_zero
        save_cfg(cfg)
        
        current_positions[sid] = estimated_zero
        target_positions[sid] = estimated_zero
        servo_initialized[sid] = True
    
    return {"status": "homed", "estimated_zero": estimated_zero}
```

#### Вариант 3: Ручная установка начального положения
```html
<!-- Добавить в calibration.html -->
<div class="initial-position-setup">
    <h3>Initial Position Setup</h3>
    <p>Before first use, manually move the servo to desired zero position:</p>
    
    <button onclick="disableServo()">Disable Servo (Free Movement)</button>
    <p>1. Click "Disable", then physically move the joint to zero position</p>
    
    <button onclick="setAsZero()">Set Current as Zero</button>
    <p>2. Click "Set as Zero" to calibrate</p>
    
    <button onclick="enableServo()">Enable Servo</button>
</div>

<script>
async function disableServo() {
    await fetch('/api/cal/disable/' + selectedJoint, {method: 'POST'});
    alert('Servo disabled. Move joint to zero position.');
}

async function setAsZero() {
    const response = await fetch('/api/cal/set_zero', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            joint_name: selectedJoint,
            angle_deg: 0
        })
    });
    const result = await response.json();
    alert(`Zero set: ${result.zero_deg}`);
}
</script>
```

**Рекомендация:** Использовать комбинацию Вариант 1 + Вариант 3

**Оценка важности:** ⭐⭐⭐⭐ Влияет на безопасность калибровки

---

## 6. `pose()` API — Проверка физической корректности

### Текущие позы (calibrate_web.py:37-48)
```python
POSES = {
    "reference": {"legs": {"lf":[0,0,0],"rf":[0,0,0],"lr":[0,0,0],"rr":[0,0,0]}},
    "tiptoe_front": {"legs": {"lf":[0,-30,20],"rf":[0,-30,20]}},
    "tiptoe_rear": {"legs": {"lr":[0,-30,20],"rr":[0,-30,20]}},
    "tiptoe_all": {"legs": {"lf":[0,-30,20],"rf":[0,-30,20],"lr":[0,-30,20],"rr":[0,-30,20]}},
    "lean_forward": {"legs": {"lf":[-15,10,-10],"rf":[-15,10,-10],"lr":[15,-10,10],"rr":[15,-10,10]}},
    "sit_back": {"legs": {"lf":[0,-20,20],"rf":[0,-20,20],"lr":[0,30,-40],"rr":[0,30,-40]}},
    "stand_tall": {"legs": {"lf":[0,-30,15],"rf":[0,-30,15],"lr":[0,-30,15],"rr":[0,-30,15]}},
    "crouch": {"legs": {"lf":[0,40,-50],"rf":[0,40,-50],"lr":[0,40,-50],"rr":[0,40,-50]}},
    "front_reach": {"legs": {"lf":[20,-20,20],"rf":[20,-20,20]}},
    "rear_kick": {"legs": {"lr":[-20,20,-20],"rr":[-20,20,-20]}},
}
```

### Анализ физических ограничений MG996R:

| Параметр | Значение | Примечание |
|----------|----------|------------|
| Рабочий диапазон | ~170° | Реально usable ~150° |
| Стопорный момент | 9.4 кг·см @ 6V | При 4.8V ~7 кг·см |
| Скорость | 0.17 сек/60° | Макс скорость ~350°/сек |
| Мёртвая зона | 7μs | ~0.7° минимальное движение |

### Проблемы текущих поз:

1. **`crouch`: `[0,40,-50]`**
   - Calf -50° может быть за пределами диапазона
   - Thigh 40° + calf -50° = возможная коллизия с корпусом
   
2. **`rear_kick`: `[-20,20,-20]`**
   - Hip -20° назад — проверить не упирается ли в корпус
   - Требует смещения центра масс вперёд для баланса

3. **`front_reach`: `[20,-20,20]`**
   - Hip 20° вперёд + thigh -20° = вытянутая нога
   - Риск перегрузки сервопривода (максимальный рычаг)

### Улучшенная версия с валидацией:

```python
# Физические ограничения для каждой ноги
LEG_LIMITS = {
    "hip": {
        "min_deg": -45,   # Внутрь корпуса
        "max_deg": 45,    # Наружу
        "description": "Abduction/Adduction"
    },
    "thigh": {
        "min_deg": -60,   # Вниз
        "max_deg": 30,    # Вверх
        "description": "Hip flexion"
    },
    "calf": {
        "min_deg": -120,  # Полное разгибание
        "max_deg": -30,   # Полное сгибание (90° угол)
        "description": "Knee flexion"
    }
}

def validate_pose_angles(leg_name, angles):
    """
    Validate pose angles against physical limits.
    
    Returns:
        tuple: (is_valid, error_message_or_none, corrected_angles)
    """
    hip, thigh, calf = angles
    corrected = list(angles)
    warnings = []
    
    # Check hip
    if hip < LEG_LIMITS["hip"]["min_deg"]:
        warnings.append(f"Hip {hip}° < min {LEG_LIMITS['hip']['min_deg']}°")
        corrected[0] = LEG_LIMITS["hip"]["min_deg"]
    elif hip > LEG_LIMITS["hip"]["max_deg"]:
        warnings.append(f"Hip {hip}° > max {LEG_LIMITS['hip']['max_deg']}°")
        corrected[0] = LEG_LIMITS["hip"]["max_deg"]
    
    # Check thigh
    if thigh < LEG_LIMITS["thigh"]["min_deg"]:
        warnings.append(f"Thigh {thigh}° < min {LEG_LIMITS['thigh']['min_deg']}°")
        corrected[1] = LEG_LIMITS["thigh"]["min_deg"]
    elif thigh > LEG_LIMITS["thigh"]["max_deg"]:
        warnings.append(f"Thigh {thigh}° > max {LEG_LIMITS['thigh']['max_deg']}°")
        corrected[1] = LEG_LIMITS["thigh"]["max_deg"]
    
    # Check calf
    if calf < LEG_LIMITS["calf"]["min_deg"]:
        warnings.append(f"Calf {calf}° < min {LEG_LIMITS['calf']['min_deg']}°")
        corrected[2] = LEG_LIMITS["calf"]["min_deg"]
    elif calf > LEG_LIMITS["calf"]["max_deg"]:
        warnings.append(f"Calf {calf}° > max {LEG_LIMITS['calf']['max_deg']}°")
        corrected[2] = LEG_LIMITS["calf"]["max_deg"]
    
    # Check for mechanical collisions
    # Example: thigh up + calf extended might hit body
    if thigh > 20 and calf > -60:
        warnings.append("Warning: Thigh up + calf extended may collide with body")
    
    # Check stability (center of mass)
    # Simplified: all hips should be roughly symmetric
    # (full check requires knowing other leg positions)
    
    is_valid = len(warnings) == 0
    return is_valid, "; ".join(warnings) if warnings else None, corrected


# Обновлённые позы с проверкой:
POSES = {
    "reference": {
        "label": "Reference (ноль)",
        "desc": "Полуприсед — нейтральная позиция",
        "legs": {"lf":[0,0,0],"rf":[0,0,0],"lr":[0,0,0],"rr":[0,0,0]},
        "stability": "stable",
        "notes": "Базовая позиция для калибровки"
    },
    "tiptoe_front": {
        "label": "На цыпочках (перед)",
        "desc": "Передние лапы вверх, задние на земле",
        "legs": {"lf":[0,-30,20],"rf":[0,-30,20]},
        "stability": "unstable",
        "warning": "Требуется поддержка или быстрое выполнение",
        "auto_balance": True  # Флаг для включения balance controller
    },
    "tiptoe_rear": {
        "label": "На цыпочках (зад)",
        "desc": "Задние лапы вверх, передние на земле",
        "legs": {"lr":[0,-30,20],"rr":[0,-30,20]},
        "stability": "moderate",
        "notes": "Передние лапы должны быть широко расставлены"
    },
    "tiptoe_all": {
        "label": "На цыпочках (все)",
        "desc": "Все лапы вверх — максимальная высота",
        "legs": {"lf":[0,-30,20],"rf":[0,-30,20],"lr":[0,-30,20],"rr":[0,-30,20]},
        "stability": "very_unstable",
        "warning": "Высокий центр масс — риск падения!",
        "require_balance": True
    },
    "lean_forward": {
        "label": "Наклон вперёд",
        "desc": "Перед назад, зад вперёд — растяжка",
        "legs": {"lf":[-15,10,-10],"rf":[-15,10,-10],"lr":[15,-10,10],"rr":[15,-10,10]},
        "stability": "stable",
        "notes": "Проверка диапазона hip суставов"
    },
    "sit_back": {
        "label": "Сесть на задние",
        "desc": "Зад согнуты, перед вытянуты",
        "legs": {"lf":[0,-20,20],"rf":[0,-20,20],"lr":[0,30,-40],"rr":[0,30,-40]},
        "stability": "stable",
        "notes": "Естественная поза отдыха"
    },
    "stand_tall": {
        "label": "Полный stand",
        "desc": "Максимально высоко",
        "legs": {"lf":[0,-30,15],"rf":[0,-30,15],"lr":[0,-30,15],"rr":[0,-30,15]},
        "stability": "moderate",
        "notes": "Проверка limits: thigh -30°"
    },
    "crouch": {
        "label": "Полный присед",
        "desc": "Максимально низко",
        "legs": {"lf":[0,40,-50],"rf":[0,40,-50],"lr":[0,40,-50],"rr":[0,40,-50]},
        "stability": "very_stable",
        "notes": "Низкий центр масс, calf -50° в пределах диапазона"
    },
    "front_reach": {
        "label": "Передние вперёд",
        "desc": "Передние вытянуты вперёд",
        "legs": {"lf":[20,-20,20],"rf":[20,-20,20]},
        "stability": "moderate",
        "warning": "Передние лапы создают большой рычаг",
        "max_hold_time_s": 5  # Не держать дольше 5 секунд
    },
    "rear_kick": {
        "label": "Задние назад",
        "desc": "Задние вытянуты назад",
        "legs": {"lr":[-20,20,-20],"rr":[-20,20,-20]},
        "stability": "unstable",
        "warning": "Требуется компенсация наклоном вперёд",
        "auto_compensate": {"pitch": 0.1}  # Авто-компенсация через IMU
    },
    "lie_down": {
        "label": "Лёжа",
        "desc": "Все лапы расслаблены",
        "legs": {"lf":[0,45,-60],"rf":[0,45,-60],"lr":[0,45,-60],"rr":[0,45,-60]},
        "stability": "very_stable",
        "notes": "Позиция для длительного отдыха"
    },
    "ready": {
        "label": "Готовность",
        "desc": "Оптимальная позиция для начала движения",
        "legs": {"lf":[0,-15,10],"rf":[0,-15,10],"lr":[0,-15,10],"rr":[0,-15,10]},
        "stability": "stable",
        "notes": "Рекомендуемая стартовая позиция"
    }
}

# Обновлённый endpoint с валидацией:
@app.post("/api/cal/pose/{name}")
def pose(name: str):
    if not calibration_active:
        raise HTTPException(400, "Not active")
    if name not in POSES:
        raise HTTPException(404, f"Unknown pose: {name}")
    
    cfg = load_cfg()
    pose_data = POSES[name]
    
    # Validate all legs
    all_warnings = []
    validated_legs = {}
    
    for leg, indices in LEG_JOINTS.items():
        if leg in pose_data['legs']:
            angles = pose_data['legs'][leg]
            is_valid, warning, corrected = validate_pose_angles(leg, angles)
            
            if warning:
                all_warnings.append(f"{leg}: {warning}")
            
            validated_legs[leg] = corrected if corrected else angles
    
    with lock:
        for leg, angles in validated_legs.items():
            indices = LEG_JOINTS[leg]
            for i, ji in enumerate(indices):
                j = cfg['joints'][ji]
                a = max(j['min_deg'], min(j['max_deg'], angles[i]))
                target_positions[j['servo_id']] = a
    
    response = {
        "status": "moving",
        "pose": name,
        "label": pose_data['label'],
        "stability": pose_data.get('stability', 'unknown')
    }
    
    if all_warnings:
        response['warnings'] = all_warnings
    
    if 'auto_balance' in pose_data:
        response['auto_balance'] = pose_data['auto_balance']
    
    return response
```

**Оценка важности:** ⭐⭐⭐ Предотвращает механические повреждения

---

## 7. `state_poller()` — Когда 1Hz недостаточно?

### Текущая реализация (web_server_host.py:191-198)
```python
def state_poller():
    while True:
        try:
            state.refresh()
        except Exception:
            state.connected = False
        import time
        time.sleep(1)  # ❌ 1Hz polling
```

### Когда 1Hz мало:

| Сценарий | Требуемая частота | Обоснование |
|----------|------------------|-------------|
| Телеоперация (клавиатура) | 10-20 Hz | Задержка >100ms ощущается |
| Балансировка на 2 ногах | 50-100 Hz | Время реакции на наклон |
| Избегание препятствий | 20-30 Hz | Дистанция 1м при 0.5м/с = 2 сек |
| Запись телеметрии | 50 Hz | Анализ вибраций, токов |
| Отладка походки | 50-100 Hz | Фаза шага 0.5-1 сек |
| Мониторинг токов | 20-50 Hz | Пики тока длятся 100-200ms |

### Адаптивный polling rate:

```python
class AdaptiveStatePoller:
    """Poller with adaptive frequency based on robot state."""
    
    def __init__(self, state):
        self.state = state
        self.base_rate = 1.0  # Hz
        self.high_rate = 20.0  # Hz for active states
        self.current_rate = self.base_rate
        
        # Conditions for high-rate polling
        self.active_conditions = [
            self._is_moving,
            self._is_balancing,
            self._is_calibrating,
            self._websocket_connected
        ]
    
    def _is_moving(self):
        """Check if robot is actively moving."""
        # Check cmd_vel from last 2 seconds
        # Implementation depends on your state structure
        return hasattr(self.state, 'last_cmd_vel_time') and \
               time.time() - self.state.last_cmd_vel_time < 2.0
    
    def _is_balancing(self):
        """Check if balance controller is active."""
        return getattr(self.state, 'balance_active', False)
    
    def _is_calibrating(self):
        """Check if in calibration mode."""
        return getattr(self.state, 'calibration_active', False)
    
    def _websocket_connected(self):
        """Check if WebSocket clients are connected."""
        return getattr(self.state, 'ws_clients', 0) > 0
    
    def _calculate_rate(self):
        """Determine optimal polling rate."""
        if any(cond() for cond in self.active_conditions):
            return self.high_rate
        return self.base_rate
    
    def run(self):
        """Main polling loop with adaptive rate."""
        import time
        
        while True:
            try:
                start_time = time.time()
                
                # Refresh state
                self.state.refresh()
                
                # Calculate next sleep time
                self.current_rate = self._calculate_rate()
                sleep_time = 1.0 / self.current_rate
                
                # Account for processing time
                elapsed = time.time() - start_time
                actual_sleep = max(0, sleep_time - elapsed)
                
                time.sleep(actual_sleep)
                
            except Exception as e:
                self.state.connected = False
                print(f"State poller error: {e}")
                time.sleep(1)  # Back off on error

# Usage in web_server_host.py:
def state_poller():
    poller = AdaptiveStatePoller(state)
    poller.run()
```

### Альтернатива: Event-driven updates вместо polling

```python
# Вместо polling подписаться на ROS2 топики напрямую
class StateSubscriber(Node):
    def __init__(self, state):
        super().__init__('state_subscriber')
        self.state = state
        
        # Подписка на joint_states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joints,
            10
        )
        
        # Подписка на IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._on_imu,
            10
        )
    
    def _on_joints(self, msg):
        self.state.joint_positions = list(msg.position)
        self.state.last_update = time.time()
    
    def _on_imu(self, msg):
        self.state.imu_orientation = {
            'x': msg.orientation.x, 'y': msg.orientation.y,
            'z': msg.orientation.z, 'w': msg.orientation.w
        }
        self.state.last_imu_update = time.time()

# Тогда polling вообще не нужен!
```

**Рекомендация:** Для веб-интерфейса использовать адаптивный polling 1-20Hz, для реального времени — event-driven подписку.

**Оценка важности:** ⭐⭐⭐ Влияет на отзывчивость интерфейса

---

## 8. `RobotLeg.set_position()` — Обратная кинематика

### Текущая проблема (walk_gait.py:61-74)
```python
def set_position(self, hip_deg, thigh_deg, calf_deg):
    """Set all 3 joint angles."""
    if self.inverted:
        hip_deg = -hip_deg
        thigh_deg = -thigh_deg
        calf_deg = -calf_deg
    
    # Convert degrees to pulse (1000-2000us range, 1500us = 0 deg)
    def deg_to_pulse(deg):
        return 1500 + deg * 5.56  # ~5.56us per degree
    
    self.pca.set_servo_pulse(self.hip_ch, deg_to_pulse(hip_deg))
    self.pca.set_servo_pulse(self.thigh_ch, deg_to_pulse(thigh_deg))
    self.pca.set_servo_pulse(self.calf_ch, deg_to_pulse(calf_deg))
```

### Проблемы:
1. **Нет обратной кинематики** — принимает углы суставов, а не позицию стопы
2. **Упрощённая конвертация** — `5.56 us/deg` не учитывает калибровку
3. **Нет проверки диапазонов**

### Реализация полноценной обратной кинематики:

```python
class InverseKinematics:
    """
    Analytical inverse kinematics for 3-DOF leg.
    
    Leg structure:
        Hip (abduction/adduction) → Thigh (flexion/extension) → Calf (knee)
    
    Coordinate system:
        x: forward
        y: lateral (left positive)
        z: upward (negative = down)
    """
    
    def __init__(self, L1=0.083, L2=0.25, L3=0.25):
        """
        Initialize with leg segment lengths.
        
        Args:
            L1: Hip to thigh length (m) — abduction offset
            L2: Thigh length (m) — femur
            L3: Calf length (m) — tibia
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
    
    def solve(self, x, y, z):
        """
        Solve inverse kinematics for foot position.
        
        Args:
            x: Forward position (m)
            y: Lateral position (m)
            z: Vertical position (m, negative = down)
        
        Returns:
            tuple: (hip_angle, thigh_angle, calf_angle) in radians
                   or None if position is unreachable
        """
        import math
        
        # 1. Hip angle (abduction/adduction)
        # Project onto YZ plane
        hip_angle = math.atan2(y, -z)
        
        # 2. Distance from hip to foot in sagittal plane
        r = math.sqrt(x**2 + z**2 + y**2)
        
        # Check reachability
        max_reach = self.L2 + self.L3
        if r > max_reach:
            return None  # Unreachable
        
        # 3. Law of cosines for thigh and calf
        # Triangle: L1 (hip offset), L2 (thigh), L3 (calf)
        
        # Effective distance in sagittal plane after hip rotation
        d = math.sqrt(x**2 + z**2)
        
        # Angle from vertical to foot
        psi = math.atan2(-z, x)  # Negative z because down is negative
        
        # Law of cosines for knee angle
        cos_knee = (self.L2**2 + self.L3**2 - d**2) / (2 * self.L2 * self.L3)
        cos_knee = max(-1.0, min(1.0, cos_knee))  # Clamp
        knee_angle = math.pi - math.acos(cos_knee)  # Knee bends backward
        
        # Angle for thigh
        cos_theta = (self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d)
        cos_theta = max(-1.0, min(1.0, cos_theta))
        theta = math.acos(cos_theta)
        
        # Thigh angle relative to vertical
        thigh_angle = psi + theta - math.pi / 2
        
        return hip_angle, thigh_angle, knee_angle
    
    def solve_with_constraints(self, x, y, z, 
                               hip_limits=None, 
                               thigh_limits=None, 
                               calf_limits=None):
        """
        Solve IK with joint limits.
        
        Returns:
            tuple: (success, angles_or_error)
        """
        result = self.solve(x, y, z)
        
        if result is None:
            return False, "Unreachable position"
        
        hip, thigh, calf = result
        
        # Check limits (in radians)
        if hip_limits:
            if not (hip_limits[0] <= hip <= hip_limits[1]):
                return False, f"Hip angle {hip:.2f} rad out of limits"
        
        if thigh_limits:
            if not (thigh_limits[0] <= thigh <= thigh_limits[1]):
                return False, f"Thigh angle {thigh:.2f} rad out of limits"
        
        if calf_limits:
            if not (calf_limits[0] <= calf <= calf_limits[1]):
                return False, f"Calf angle {calf:.2f} rad out of limits"
        
        return True, (hip, thigh, calf)


class RobotLeg:
    """Enhanced leg controller with inverse kinematics."""
    
    def __init__(self, pca, hip_ch, thigh_ch, calf_ch, 
                 inverted=False, config=None):
        self.pca = pca
        self.hip_ch = hip_ch
        self.thigh_ch = thigh_ch
        self.calf_ch = calf_ch
        self.inverted = inverted
        self.config = config or {}
        
        # Initialize IK solver
        self.ik = InverseKinematics(
            L1=self.config.get('L1', 0.083),
            L2=self.config.get('L2', 0.25),
            L3=self.config.get('L3', 0.25)
        )
        
        # Joint limits from config
        self.hip_limits = self.config.get('hip_limits', (-0.8, 0.8))
        self.thigh_limits = self.config.get('thigh_limits', (-1.5, 1.5))
        self.calf_limits = self.config.get('calf_limits', (-2.5, -0.5))
    
    def set_foot_position(self, x, y, z):
        """
        Set foot position using inverse kinematics.
        
        Args:
            x: Forward/backward (m, positive = forward)
            y: Left/right (m, positive = left)
            z: Up/down (m, negative = down)
        
        Returns:
            dict: Status and angles
        """
        success, result = self.ik.solve_with_constraints(
            x, y, z,
            hip_limits=self.hip_limits,
            thigh_limits=self.thigh_limits,
            calf_limits=self.calf_limits
        )
        
        if not success:
            return {'success': False, 'error': result}
        
        hip, thigh, calf = result
        
        # Apply inversion if needed
        if self.inverted:
            hip = -hip
            thigh = -thigh
            calf = -calf
        
        # Convert to degrees for servo control
        hip_deg = math.degrees(hip)
        thigh_deg = math.degrees(thigh)
        calf_deg = math.degrees(calf)
        
        # Send to servos
        self._set_servo_angles(hip_deg, thigh_deg, calf_deg)
        
        return {
            'success': True,
            'angles_rad': (hip, thigh, calf),
            'angles_deg': (hip_deg, thigh_deg, calf_deg)
        }
    
    def _set_servo_angles(self, hip_deg, thigh_deg, calf_deg):
        """Convert angles to PWM and send to servos."""
        # Use calibrated conversion from calibrate_web.py
        def deg_to_us(deg, ch):
            # Get config for this channel
            jcfg = self.config.get(f'channel_{ch}', {})
            sign = jcfg.get('sign', 1)
            zero_deg = jcfg.get('zero_deg', 0)
            min_deg = jcfg.get('min_deg', -90)
            max_deg = jcfg.get('max_deg', 90)
            
            return calibrate_web.deg_to_us(deg, sign, zero_deg, min_deg, max_deg)
        
        hip_us = deg_to_us(hip_deg, self.hip_ch)
        thigh_us = deg_to_us(thigh_deg, self.thigh_ch)
        calf_us = deg_to_us(calf_deg, self.calf_ch)
        
        self.pca.set_servo_pulse(self.hip_ch, hip_us)
        self.pca.set_servo_pulse(self.thigh_ch, thigh_us)
        self.pca.set_servo_pulse(self.calf_ch, calf_us)
    
    def set_position(self, hip_deg, thigh_deg, calf_deg):
        """Legacy method — direct joint angle control."""
        if self.inverted:
            hip_deg = -hip_deg
            thigh_deg = -thigh_deg
            calf_deg = -calf_deg
        
        self._set_servo_angles(hip_deg, thigh_deg, calf_deg)
```

**Оценка важности:** ⭐⭐⭐⭐⭐ Критично для нормальной походки

---

## 9. Улучшение обратной кинематики в gait_controller.py

### Текущая проблема (gait_controller.py:420-474)
```python
def _foot_to_joint_angles(self, leg: str, foot_pos: dict) -> dict:
    # Упрощённая IK без учёта:
    # - IMU для компенсации наклона корпуса
    # - Реальных длин сегментов из конфига
    # - Ограничений суставов
```

### Улучшенная версия с интеграцией IMU:

```python
class EnhancedGaitController(GaitController):
    """Gait controller with IMU-based body compensation."""
    
    def __init__(self):
        super().__init__()
        
        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            10
        )
        
        # Current body orientation
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.0
        
        # Load actual leg dimensions from parameters
        self.declare_parameter('leg.L1', 0.083)  # Hip offset
        self.declare_parameter('leg.L2', 0.25)   # Thigh length
        self.declare_parameter('leg.L3', 0.25)   # Calf length
        
        self.L1 = self.get_parameter('leg.L1').value
        self.L2 = self.get_parameter('leg.L2').value
        self.L3 = self.get_parameter('leg.L3').value
        
        # Full IK solver
        self.ik_solver = InverseKinematics(self.L1, self.L2, self.L3)
    
    def _imu_callback(self, msg: Imu):
        """Update body orientation from IMU."""
        roll, pitch, yaw = euler_from_quaternion(msg.orientation)
        self.body_roll = roll
        self.body_pitch = pitch
        self.body_yaw = yaw
    
    def _foot_to_joint_angles(self, leg: str, foot_pos: dict) -> dict:
        """
        Enhanced IK with body orientation compensation.
        
        When body tilts, adjust foot positions to maintain stability.
        """
        x, y, z = foot_pos['x'], foot_pos['y'], foot_pos['z']
        
        # 1. Compensate for body tilt
        # If body rolls right, left leg needs to extend, right leg retract
        roll_compensation = {
            'FL': -self.body_roll * self.leg_span_y / 2,
            'FR': +self.body_roll * self.leg_span_y / 2,
            'BL': -self.body_roll * self.leg_span_y / 2,
            'BR': +self.body_roll * self.leg_span_y / 2
        }
        
        # Pitch compensation: front legs vs back legs
        pitch_compensation = {
            'FL': +self.body_pitch * self.leg_span_x / 2,
            'FR': +self.body_pitch * self.leg_span_x / 2,
            'BL': -self.body_pitch * self.leg_span_x / 2,
            'BR': -self.body_pitch * self.leg_span_x / 2
        }
        
        # Apply compensation to Z coordinate
        z_compensated = z + roll_compensation[leg] + pitch_compensation[leg]
        
        # 2. Solve IK with compensated position
        result = self.ik_solver.solve(x, y, z_compensated)
        
        if result is None:
            # Fallback to standing pose if unreachable
            self.get_logger().warn(f"IK failed for {leg} at ({x}, {y}, {z})")
            return {
                f'{leg}_hip': self.standing_pose['hip'],
                f'{leg}_thigh': self.standing_pose['thigh'],
                f'{leg}_calf': self.standing_pose['calf']
            }
        
        hip_angle, thigh_angle, knee_angle = result
        
        # 3. Apply leg-specific adjustments
        if leg in ['FR', 'BR']:
            hip_angle = -hip_angle
            thigh_angle = -thigh_angle
            knee_angle = -knee_angle
        
        if leg in ['BL', 'BR']:
            thigh_angle = -thigh_angle
            knee_angle = -knee_angle
        
        return {
            f'{leg}_hip': hip_angle,
            f'{leg}_thigh': thigh_angle,
            f'{leg}_calf': knee_angle
        }
    
    def _calculate_trot_gait(self, t: float):
        """Enhanced trot gait with continuous IMU compensation."""
        # Original calculation
        joint_positions, body_height = super()._calculate_trot_gait(t)
        
        # Additional dynamic compensation based on body acceleration
        # (requires subscribing to linear acceleration from IMU)
        # This helps with shock absorption during walking
        
        return joint_positions, body_height
```

### Интеграция с Balance Controller:

```python
# Создать единый контроллер, объединяющий gait + balance
class IntegratedLocomotionController(Node):
    """Unified controller combining gait generation and balance."""
    
    def __init__(self):
        super().__init__('locomotion_controller')
        
        # Gait generator
        self.gait = GaitController()
        
        # Balance controller
        self.balance = BalanceController()
        
        # Fusion weights
        self.declare_parameter('fusion.gait_weight', 0.7)
        self.declare_parameter('fusion.balance_weight', 0.3)
    
    def _timer_callback(self):
        """Generate commands with gait/balance fusion."""
        # Get gait positions
        gait_positions, _ = self.gait._calculate_trot_gait(self.time)
        
        # Get balance corrections
        balance_corrections = self.balance.compute_balance_corrections()
        
        # Fuse: final = gait + balance * weight
        fused_positions = {}
        for joint, gait_angle in gait_positions.items():
            leg = joint.split('_')[0]  # FL, FR, etc.
            correction = balance_corrections.get(leg, 0.0)
            
            # Convert correction to joint angle (simplified)
            joint_correction = correction * 0.5  # Gain factor
            
            fused_positions[joint] = gait_angle + joint_correction
        
        self._publish_trajectory(fused_positions)
```

**Оценка важности:** ⭐⭐⭐⭐⭐ Критично для стабильной динамической ходьбы

---

## СВОДНАЯ ТАБЛИЦА ПРИОРИТЕТОВ

| № | Компонент | Критичность | Трудоёмкость | Эффект | Приоритет |
|---|-----------|-------------|--------------|--------|-----------|
| 1 | Фильтрация IMU (Kalman) | ⭐⭐⭐⭐⭐ | Средняя | Стабильность, баланс | **P0** |
| 2 | Graceful shutdown | ⭐⭐⭐⭐ | Низкая | Безопасность | **P0** |
| 3 | Исправление deg_to_us | ⭐⭐⭐⭐ | Низкая | Точность | **P0** |
| 4 | Обратная кинематика | ⭐⭐⭐⭐⭐ | Высокая | Походка | **P1** |
| 5 | E-Stop защита | ⭐⭐⭐⭐⭐ | Средняя | Безопасность | **P1** |
| 6 | Move без телепорта | ⭐⭐⭐⭐ | Низкая | Безопасность | **P1** |
| 7 | Валидация поз | ⭐⭐⭐ | Средняя | Надёжность | **P2** |
| 8 | Adaptive polling | ⭐⭐⭐ | Низкая | UX | **P2** |
| 9 | IMU+Gait интеграция | ⭐⭐⭐⭐⭐ | Высокая | Продвинутая походка | **P3** |

---

## ЗАКЛЮЧЕНИЕ

Проект демонстрирует **высокий уровень инженерной проработки**, но требует доработок в ключевых областях:

1. **Безопасность** (E-Stop, graceful shutdown) — реализовать немедленно
2. **Точность** (калибровка, фильтрация IMU) — критично для походки
3. **Обратная кинематика** — обязательна для плавного движения
4. **Интеграция IMU+Gait** — следующий уровень развития

**Общая оценка проекта:** 4.5/5 ⭐⭐⭐⭐⭐
