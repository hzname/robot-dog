# dog_sensors

ROS2 пакет с драйверами сенсоров для робота-собаки.

## Содержимое

### imu_node.py
Нода для чтения данных с IMU датчика MPU6050 по I2C.

**Публикует:**
- `/imu/data` (sensor_msgs/Imu) — полные данные IMU
- `/imu/tilt` (std_msgs/Float32MultiArray) — углы наклона [roll, pitch] в градусах

**Параметры:**
- `i2c_bus` — номер I2C шины (по умолчанию: 1)
- `publish_rate` — частота публикации в Hz (по умолчанию: 100)
- `frame_id` — TF frame ID (по умолчанию: "imu_link")
- `use_mock` — использовать тестовые данные вместо реального сенсора

### Launch файлы

**imu.launch.py** — запуск только IMU ноды:
```bash
ros2 launch dog_sensors imu.launch.py
ros2 launch dog_sensors imu.launch.py use_mock:=true
```

**imu_balance.launch.py** — запуск IMU + balance_controller:
```bash
ros2 launch dog_sensors imu_balance.launch.py
```

## Требования

Для работы с реальным MPU6050:
```bash
pip install smbus2
```

При отсутствии smbus2 нода автоматически переключается в режим mock.
