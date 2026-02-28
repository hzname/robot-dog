# Детальная инструкция по установке

## Содержание

1. [Подготовка Banana Pi](#подготовка-banana-pi)
2. [Установка ROS2 Humble](#установка-ros2-humble)
3. [Настройка Docker](#настройка-docker)
4. [Установка проекта](#установка-проекта)
5. [Настройка окружения](#настройка-окружения)

---

## Подготовка Banana Pi

### 1. Установка ОС

Скачайте и установите Ubuntu 22.04 для Banana Pi:

```bash
# Скачать образ
wget https://example.com/ubuntu-22.04-bpi-m5.img.xz

# Записать на SD карту (минимум 32GB)
sudo dd if=ubuntu-22.04-bpi-m5.img of=/dev/sdX bs=4M status=progress
```

### 2. Первоначальная настройка

```bash
# Подключиться по SSH (после первой загрузки)
ssh banana@192.168.1.XXX  # пароль: banana

# Обновление системы
sudo apt update && sudo apt upgrade -y

# Установка базовых пакетов
sudo apt install -y \
    git \
    curl \
    vim \
    htop \
    net-tools \
    i2c-tools

# Включение I2C
sudo apt install raspi-config
sudo raspi-config
# Interface Options → I2C → Enable
```

### 3. Настройка сети (опционально)

```bash
# Статический IP
sudo nano /etc/netplan/01-netcfg.yaml
```

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
```

```bash
sudo netplan apply
```

---

## Установка ROS2 Humble

### 1. Добавление репозиториев

```bash
# Установка зависимостей
sudo apt update && sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common

# Добавление ключа ROS2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Добавление репозитория
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 2. Установка пакетов ROS2

```bash
# Базовая установка (рекомендуется для BPi)
sudo apt install -y ros-humble-ros-base ros-dev-tools

# Дополнительные пакеты
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-rviz2

# Python зависимости
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep

# Инициализация rosdep
sudo rosdep init
rosdep update
```

### 3. Настройка окружения

```bash
# Добавить в ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/robot_dog_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

# Применить
source ~/.bashrc
```

---

## Настройка Docker

### 1. Установка Docker

```bash
# Установка
curl -fsSL https://get.docker.com | sh

# Добавление пользователя в группу docker
sudo usermod -aG docker $USER
newgrp docker

# Установка Docker Compose
sudo apt install -y docker-compose
```

### 2. Проверка Docker

```bash
# Тест
docker run hello-world

# Должно вывести сообщение о успешном запуске
```

---

## Установка проекта

### 1. Клонирование репозитория

```bash
cd ~
git clone https://github.com/yourusername/robot_dog_ws.git
cd robot_dog_ws
```

### 2. Установка Python зависимостей

```bash
# Для работы с PCA9685
pip3 install adafruit-circuitpython-servokit

# Другие зависимости
pip3 install numpy transforms3d
```

### 3. Сборка проекта

```bash
# Установка зависимостей ROS
rosdep install --from-paths src --ignore-src -y

# Сборка
colcon build --symlink-install

# Применение изменений
source install/setup.bash
```

---

## Настройка окружения

### 1. Настройка прав доступа

```bash
# Доступ к I2C без sudo
sudo usermod -aG i2c $USER

# Доступ к GPIO (если нужно)
sudo usermod -aG gpio $USER

# Перелогин или перезагрузка
sudo reboot
```

### 2. Проверка установки

```bash
# Проверка I2C
i2cdetect -y 1
# Должен показать устройство на 0x40

# Проверка ROS2
ros2 topic list

# Проверка проекта
ros2 pkg list | grep dog
```

### 3. Автозапуск (опционально)

```bash
# Создать systemd сервис
sudo nano /etc/systemd/system/robot-dog.service
```

```ini
[Unit]
Description=Robot Dog ROS2
After=network.target

[Service]
Type=simple
User=banana
WorkingDirectory=/home/banana/robot_dog_ws
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch dog_bringup robot_dog.launch.py use_hardware:=true'
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable robot-dog
sudo systemctl start robot-dog
```

---

## Устранение неполадок

### Проблема: Permission denied на I2C

```bash
sudo chmod 666 /dev/i2c-1
# Или добавить пользователя в группу i2c
```

### Проблема: ROS пакеты не найдены

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dog_ws/install/setup.bash
```

### Проблема: Docker контейнер не стартует

```bash
# Проверка логов
docker-compose logs

# Пересборка
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

### Проблема: Сервоприводы не реагируют

```bash
# Проверка I2C
sudo i2cdetect -y 1

# Проверка питания (должно быть 5-6V)
# Проверка соединений
```
