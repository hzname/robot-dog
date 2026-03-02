# Деплой на Banana Pi

## Быстрый старт

### 1. Сборка бинарников (на ПК с Rust)

```bash
# Установка cross-compilation toolchain
rustup target add aarch64-unknown-linux-gnu

# Сборка всех Rust нод
cd ~/robot_dog_ws/src

cd dog_control_rust && cargo build --release --target aarch64-unknown-linux-gnu && cd ..
cd dog_hardware_rust && cargo build --release --target aarch64-unknown-linux-gnu && cd ..
cd dog_sensors_rust && cargo build --release --target aarch64-unknown-linux-gnu && cd ..
cd dog_balance_rust && cargo build --release --target aarch64-unknown-linux-gnu && cd ..

# Копирование бинарников
mkdir -p ~/robot_dog_ws/rust_nodes
cp */target/aarch64-unknown-linux-gnu/release/*_rust ~/robot_dog_ws/rust_nodes/
```

### 2. Копирование на Banana Pi

```bash
# На ПК
rsync -avz ~/robot_dog_ws/rust_nodes banana@banana-pi.local:~/robot_dog_ws/
rsync -avz ~/robot_dog_ws/docker-compose.deploy.yml banana@banana-pi.local:~/robot_dog_ws/
rsync -avz ~/robot_dog_ws/install banana@banana-pi.local:~/robot_dog_ws/
```

### 3. Запуск на Banana Pi

```bash
# SSH на Banana Pi
ssh banana@banana-pi.local

cd ~/robot_dog_ws

# Запуск всех нод
docker-compose -f docker-compose.deploy.yml up -d

# Проверка статуса
docker-compose -f docker-compose.deploy.yml ps
docker-compose -f docker-compose.deploy.yml logs -f gait_controller

# Остановка
docker-compose -f docker-compose.deploy.yml down
```

### 4. Телеуправление

```bash
# В отдельном терминале
docker-compose -f docker-compose.deploy.yml --profile teleop run --rm teleop

# Управление:
# W/S - вперёд/назад
# A/D - поворот
# Q/E - стрейф
# SPACE - остановка
# Ctrl+C - выход
```

## Проверка работы

```bash
# Топики
ros2 topic list

# Публикация в /joint_states
ros2 topic echo /joint_states

# Команда скорости
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

## Отладка

```bash
# Логи всех нод
docker-compose -f docker-compose.deploy.yml logs

# Лог конкретной ноды
docker-compose -f docker-compose.deploy.yml logs gait_controller

# Перезапуск одной ноды
docker-compose -f docker-compose.deploy.yml restart gait_controller

# Вход в контейнер
docker exec -it gait_controller_rust bash
```

## Troubleshooting

**Нет доступа к I2C:**
```bash
sudo usermod -aG i2c banana
sudo reboot
```

**Ошибка сборки (linker):**
```bash
# Установка linker для ARM64
sudo apt-get install gcc-aarch64-linux-gnu
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
```

**Docker не запускается:**
```bash
# Проверка Docker на Banana Pi
sudo systemctl status docker
sudo usermod -aG docker banana
# Перелогин
```
