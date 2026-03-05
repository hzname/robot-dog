# План дальнейших работ по проекту robot-dog

**Дата:** 2026-03-05  
**Текущий статус:** Миграция Python → C++ завершена, CI/CD настроен

---

## 📊 Текущий статус

### ✅ Выполнено
- [x] Миграция всех нод с Python на C++ (rclcpp)
- [x] Исправления по аудиту (синтаксис, joint names, includes)
- [x] Unit tests для всех пакетов
- [x] CI/CD pipeline (GitHub Actions)
- [x] Документация (README, CONTRIBUTING)

### 🚧 Что осталось от старого
- Python пакеты (dog_control, dog_sensors, dog_hardware, dog_teleop) — deprecated
- Rust пакеты (*_rust) — не используются
- Очистка в отдельной задаче

---

## 🎯 Этап 1: Стабилизация и тестирование (1-2 недели)

### 1.1 Интеграционное тестирование
**Приоритет:** 🔴 P0 (блокирует всё остальное)

- [ ] Сборка в чистом окружении (Docker)
- [ ] Тестирование в simulation mode
  - Запуск всех нод через `robot_dog.launch.py`
  - Проверка топиков (`ros2 topic list`, `ros2 topic hz`)
  - Проверка связности (rqt_graph)
- [ ] Тестирование телеоперации
  - Keyboard control
  - Joystick control
  - UDP control
- [ ] Тестирование safety features
  - Emergency stop
  - Watchdog timeout
  - Position limits

### 1.2 Рефакторинг (если нужно)
**Приоритет:** 🟡 P1

- [ ] Оптимизация производительности (профилирование)
- [ ] Улучшение error handling
- [ ] Добавление diagnostics (/diagnostics topic)

### 1.3 Очистка legacy
**Приоритет:** 🟢 P2

- [ ] Удаление Python пакетов (или пометка deprecated)
- [ ] Удаление Rust пакетов
- [ ] Обновление .gitignore

---

## 🌐 Этап 2: Веб-интерфейс (2-3 недели)

### 2.1 Веб-калибратор
**Приоритет:** 🔴 P0 (нужен для настройки робота)

**Цель:** Веб-приложение для калибровки сервоприводов и IMU

**Функционал:**
- [ ] **Калибровка нейтрального положения**
  - Установка каждого серва в нейтральное положение
  - Сохранение offset'ов в YAML
  - Визуализация текущих углов
  
- [ ] **Калибровка IMU**
  - Zeroing (установка нуля)
  - Калибровка gyroscope bias
  - Калибровка accelerometer
  - Сохранение calibration data
  
- [ ] **Тестирование сервоприводов**
  - Individual servo control (slider для каждого)
  - Sequence test (поочередное движение)
  - Sweep test (плавное движение через весь диапазон)
  
- [ ] **Лимиты и safety**
  - Установка min/max углов для каждого серва
  - Тестирование limit switches (если есть)

**Технологии:**
- Backend: Python (FastAPI/Flask) + rosbridge_server
- Frontend: React/Vue.js или чистый HTML+JS
- Communication: WebSocket (roslibjs)

**Структура:**
```
web_calibrator/
├── backend/
│   ├── main.py              # FastAPI app
│   ├── ros_interface.py     # ROS2 node wrapper
│   └── calibration.py       # Calibration logic
├── frontend/
│   ├── index.html
│   ├── css/
│   └── js/
│       ├── app.js
│       ├── rosbridge.js
│       └── calibration_ui.js
└── launch/
    └── web_calibrator.launch.py
```

### 2.2 Веб-управление (Teleop UI)
**Приоритет:** 🟡 P1 (удобство использования)

**Цель:** Веб-интерфейс для телеоперации и мониторинга

**Функционал:**
- [ ] **Управление движением**
  - Virtual joystick (для touchscreen)
  - Keyboard controls (WASD)
  - Speed adjustment (slider)
  - Gait type selection (walk/trot/bound)
  
- [ ] **Мониторинг состояния**
  - Real-time joint angles ( gauges )
  - IMU data (roll/pitch/yaw визуализация)
  - Body pose (3D визуализация или 2D projection)
  - Battery level (если есть feedback)
  
- [ ] **Логи и диагностика**
  - ROS2 logs viewer
  - Topic echo (выбор топика)
  - rqt_graph в браузере
  
- [ ] **Emergency controls**
  - E-stop button (big red button)
  - Stand up / Lay down commands
  - Reset controllers

**Технологии:**
- Backend: Python (FastAPI) + rosbridge_server
- Frontend: React или Vue.js
- 3D визуализация: Three.js или ROS3D.js
- Real-time: WebSocket

**Структура:**
```
web_teleop/
├── backend/
│   ├── main.py
│   ├── ros_interface.py
│   └── api/
│       ├── control.py
│       └── telemetry.py
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   ├── Joystick.vue
│   │   │   ├── IMUDisplay.vue
│   │   │   ├── JointGauges.vue
│   │   │   └── EmergencyStop.vue
│   │   ├── views/
│   │   │   ├── Dashboard.vue
│   │   │   └── Settings.vue
│   │   └── App.vue
│   └── public/
└── launch/
    └── web_teleop.launch.py
```

### 2.3 Интеграция веб-интерфейсов
**Приоритет:** 🟢 P2

- [ ] Единый entry point (выбор калибратор/управление)
- [ ] Аутентификация (базовая, если робот в сети)
- [ ] Responsive design (mobile-friendly)
- [ ] Docker container для easy deployment

---

## 🔧 Этап 3: Улучшение аппаратной части (2-4 недели)

### 3.1 Поддержка разных IMU
**Приоритет:** 🟡 P1

- [ ] BNO055 (9-DOF с built-in fusion)
- [ ] LSM6DS3 (6-DOF, low-cost)
- [ ] ICM-20948 (9-DOF, high precision)
- [ ] Автоопределение IMU при старте

### 3.2 Поддержка разных servo drivers
**Приоритет:** 🟡 P1

- [ ] Dynamixel (UART, feedback: position, load, temperature)
- [ ] Feetech STS (UART, similar to Dynamixel)
- [ ] PWM via Raspberry Pi GPIO (для простых servo)

### 3.3 Power management
**Приоритет:** 🟢 P2

- [ ] Battery voltage monitoring
- [ ] Current monitoring (per leg)
- [ ] Auto-shutdown на low battery
- [ ] Power budget management

---

## 🧠 Этап 4: Улучшение алгоритмов (3-4 недели)

### 4.1 Ходьба
**Приоритет:** 🟡 P1

- [ ] Другие типы походок:
  - Walk (4-beat, most stable)
  - Pace (lateral legs together)
  - Bound (front/back pairs)
- [ ] Transition между gait types
- [ ] Adaptive gait (менять в зависимости от terrain)

### 4.2 Балансировка
**Приоритет:** 🟡 P1

- [ ] ZMP (Zero Moment Point) control
- [ ] Compliance control (податливость при контакте)
- [ ] Terrain adaptation (неровная поверхность)

### 4.3 Планирование движения
**Приоритет:** 🟢 P2

- [ ] Body trajectory planning
- [ ] Footstep planning
- [ ] Obstacle avoidance (с LIDAR/камерой)

---

## 📱 Этап 5: Дополнительные фичи (по желанию)

### 5.1 Мобильное приложение
- [ ] Android/iOS app
- [ ] Bluetooth/WiFi connection
- [ ] Virtual joystick + sensors (accelerometer)

### 5.2 Voice control
- [ ] Интеграция с Google Assistant / Alexa
- [ ] Команды: "Stand up", "Walk forward", "Stop"

### 5.3 Autonomous modes
- [ ] Follow me (с камерой)
- [ ] Patrol mode (waypoints)
- [ ] Object tracking

### 5.4 Simulation
- [ ] Gazebo/Isaac Sim интеграция
- [ ] RL training environment

---

## 📅 Примерный таймлайн

| Неделя | Задачи |
|--------|--------|
| 1-2 | Стабилизация, интеграционное тестирование |
| 3-4 | Веб-калибратор (backend + frontend) |
| 5-6 | Веб-управление (backend + frontend) |
| 7-8 | Поддержка разных IMU и servo drivers |
| 9-10 | Улучшение алгоритмов ходьбы |
| 11+ | Дополнительные фичи |

---

## 🛠️ Технический стек для веб-части

### Backend
- **Framework:** FastAPI (Python) — быстрый, async, автодокументация
- **ROS2 интеграция:** rosbridge_server (WebSocket bridge)
- **Альтернатива:** C++ (WebSocket++ + rclcpp) для минимальной latency

### Frontend
- **Framework:** Vue.js 3 (простой, реактивный)
- **UI библиотека:** Vuetify или Element Plus
- **3D:** Three.js (визуализация робота)
- **Графики:** Chart.js или D3.js (телеметрия)

### Communication
- **Protocol:** WebSocket (roslibjs для ROS2)
- **Rate:** 30-50Hz для control, 10Hz для telemetry

### Deployment
- **Docker:** Container с backend + frontend + rosbridge
- **Nginx:** Reverse proxy, SSL

---

## 💡 Рекомендации по приоритетам

1. **Сначала стабилизируй C++ код** — без этого веб-интерфейс бесполезен
2. **Начни с веб-калибратора** — он нужен для настройки железа
3. **Минимальный viable product** — сначала базовый функционал, потом красоты
4. **Тестируй на simulation** — перед запуском на реальном роботе

---

*План составлен: 2026-03-05*  
*Следующий шаг: Стабилизация и интеграционное тестирование*
