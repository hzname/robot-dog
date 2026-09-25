# RobotDog 2.0 🐕

Четвероногий робот на сервоприводах (Banana Pi BPI-M4 Zero, PCA9685, 12 × MG996R) под **ROS 2 Jazzy** (совместим с Lyrical). Управляется **с геймпада, с клавиатуры или с веб-страницы**. Походка — рысь, подобранная в Gazebo под реальную скорость сервоприводов.

Версия 2.0 написана с нуля. Код v1 сохранён в [`legacy/v1/`](legacy/v1), из него собраны все данные о железе.

## Что умеет

- **Движение:** вперёд и назад, шаг вбок, поворот на месте и любые их сочетания. Также встать, лечь, наклон и высота корпуса.
- **Три пульта одновременно:**
  - геймпад USB или Bluetooth (Xbox и PlayStation);
  - клавиатура в терминале, в том числе по SSH;
  - веб-страница: стики для телефона, клавиши для ноутбука, геймпад в браузере.
- **Безопасность:**
  - двигаться можно только с зажатой кнопкой deadman;
  - если пульт замолчал (0.4–0.5 с), робот останавливается;
  - E-STOP с любого пульта сразу обесточивает сервы;
  - скорость суставов ограничена, ноги включаются по очереди.
- **Симуляция в Gazebo** с тем же кодом управления и автоматической проверкой походки `walk_check`.

## Быстрый старт

### На роботе (Banana Pi, Docker)

```bash
git clone https://github.com/hzname/robot-dog.git && cd robot-dog
docker compose up -d --build          # первая сборка на Pi долгая: компилируется C++
```

- **Веб-пульт:** `http://<адрес-робота>:8080`, кнопка «Встать», дальше стики или WASD.
- **Геймпад:** подключите к Pi, затем A — встать, держите LB и двигайте стиками.
- **Клавиатура:** `docker compose exec robot ros2 run dog_teleop keyboard_teleop --ros-args -r __ns:=/dog`

> ⚠ **Перед первым запуском на полу откалибруйте сервы: [docs/CALIBRATION.md](docs/CALIBRATION.md).** Значения по умолчанию — оценка по v1.

### На ПК без железа

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 launch dog_gazebo sim.launch.py                        # Gazebo + веб :8080
ros2 launch dog_bringup robot.launch.py backend:=mock rviz:=true   # без физики, RViz
```

Или в Docker: `docker build -f docker/Dockerfile.sim -t robot-dog-sim .` и `docker run --rm -it --net=host robot-dog-sim`.

## Документация

Отчёты испытаний с роликами из симуляции — [report/](report/README.md). План развёртывания на реальном роботе — [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md), параметры для него вводятся в [tools/robot_setup](tools/robot_setup/README.md).

| | |
|---|---|
| [docs/HARDWARE.md](docs/HARDWARE.md) | **Механика и электроника:** что собрано из v1, где источники противоречат друг другу, питание, моменты, что измерить |
| [docs/PLATFORM.md](docs/PLATFORM.md) | Версии ROS 2 и Gazebo на железе и на ПК (актуально на 09.2026) |
| [docs/CONTROL.md](docs/CONTROL.md) | Раскладки геймпада и клавиатуры, веб-пульт, цепочка безопасности |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Калибровка сервоприводов пошагово |
| [docs/SIMULATION.md](docs/SIMULATION.md) | Gazebo, `walk_check`, как подбиралась походка |
| [docs/TERRAIN.md](docs/TERRAIN.md) | **Подъём, спуск, волны, камни:** пределы уклонов и неровностей по манёврам, компенсация по IMU |
| [docs/HEAD.md](docs/HEAD.md) | Варианты «головы»: камеры, лидар, датчики обрыва; нужен ли второй IMU и хватит ли двух серв |
| [docs/PERCEPTION.md](docs/PERCEPTION.md) | **Симуляция лидаров «крестом» и VL53L1X:** точность пола, обнаружение камней и ступенек, ложные срабатывания, нагрузка на процессор |
| [docs/COMPUTE.md](docs/COMPUTE.md) | Хватит ли одной платы: замер загрузки, что вынести на микроконтроллер, ноутбук или более мощную плату |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | **Развёртывание на железе по этапам:** измерения со схемами, питание, ОС, сборка, сервы, калибровка, первые шаги, рельеф, эксплуатация |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Узлы, топики, режимы, походка, кинематика, тесты |

## Структура

```
ros2_ws/src/
  dog_control/      кинематика, рысь, режимы + locomotion_node        (C++)
  dog_hardware/     драйвер PCA9685, калибровка, датчик тока, IMU      (C++)
  dog_teleop/       геймпад, joy_teleop, клавиатура                   (C++)
  dog_web/          веб-пульт: HTTP + WebSocket + страница             (Python)
  dog_description/  генератор URDF из robot.yaml                      (Python)
  dog_bringup/      launch, конфиги (robot / servos / teleop), calib_pose
  dog_gazebo/       симуляция, walk_check, миры с рельефом, terrain_sweep, perception_check
  dog_perception/   лидары «крестом» + ToF: плоскость пола, карта высот, препятствия (Python)
docker/             образ робота (arm64/amd64) и образ симуляции
legacy/v1/          предыдущая версия (не собирается)
```

## Состояние

| | Статус |
|---|---|
| Сборка без предупреждений, Jazzy и Lyrical | ✅ |
| 109 тестов (юнит + интеграционные: весь стек, веб, геймпад через FIFO, IMU, восприятие) | ✅ на обеих версиях |
| Ходьба в Gazebo (`walk_check` 8/8), Harmonic и Jetty | ✅ |
| Рельеф: уклон 10° (все манёвры), волны и камни 10 мм; пределы в TERRAIN.md | ✅ в CI |
| Компенсация уклона по IMU (MPU6050 находится на шине сам) | ✅ в симуляции, на роботе проверить оси |
| Клавиатурный пульт в настоящем терминале (pty) | ✅ |
| Запуск на реальном роботе | ⏳ нужна калибровка и измерения (см. HARDWARE.md) |
| Удержание курса по гироскопу: увод на прямой ≤ 9° вместо до 34°, развороты 95–110 % вместо 52–72 % | ✅ в симуляции |

## Лицензия

MIT, см. [LICENSE](LICENSE).
