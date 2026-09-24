# Версии ROS 2 на железе и в симуляции

Данные на **24.09.2026**: теги и размеры сняты с Docker Hub, версии пакетов из самих образов. Каждая сборка v2 проверена в этих образах.

## Какие версии доступны

| | **Jazzy Jalisco** (основная) | **Lyrical Luth** |
|---|---|---|
| Выпуск / поддержка | май 2024, LTS до мая 2029 | май 2026, LTS до ≈ мая 2031 |
| Базовая ОС образов | Ubuntu 24.04 | Ubuntu 26.04 |
| **Робот (arm64)** | `ros:jazzy-ros-base`, 0.30 ГБ (rclcpp 28.1.22, сборка 2026-09) | `ros:lyrical-ros-base`, 0.34 ГБ (rclcpp 32.0.2) |
| **Симулятор (ПК, amd64)** | `osrf/ros:jazzy-simulation`: **Gazebo Sim 8.11 (Harmonic)** + ros_gz | `osrf/ros:lyrical-simulation`: **Gazebo Sim 10.5 (Jetty)** + ros_gz |
| Python / GCC | 3.12 / 13.3 | 3.14 / 15.2 |
| Юнит- и интеграционные тесты v2 | 71 / 71 ✅ | 71 / 71 ✅ |
| `walk_check` в Gazebo | 8 / 8 ✅ (3 прогона) | 8 / 8 ✅ |

Образы `osrf/ros:*-simulation` и `*-desktop` существуют только под amd64. На роботе нужен только `ros-base`, его хватает: пакеты v2 не зависят от xacro, joy и ros2_control.

## Робот

- На Banana Pi стоит **Armbian bookworm (Debian 12)**. Бинарные пакеты ROS 2 выпускаются под Ubuntu, под Debian их нет, поэтому ROS запускается в Docker, как и в v1.
- v1 работал на `ros:jazzy`. v2 по умолчанию тоже **Jazzy**: он уже проверен на этой плате, а экосистема вокруг него зрелая.
- Переход на Lyrical занимает одну строку. Код совместим с ним, это проверено:
  ```yaml
  # docker-compose.yml
  args:
    ROS_DISTRO: lyrical
  ```

## Симулятор

«Эмулятор» здесь — это Gazebo на ПК с тем же кодом управления, что и на роботе. Отличие одно: вместо драйвера сервоприводов стоят контроллеры суставов Gazebo (см. [SIMULATION.md](SIMULATION.md)).

- Jazzy работает с Gazebo Harmonic, Lyrical — с Gazebo Jetty. Имена плагинов (`gz-sim-*-system`) у них совпадают, launch-файл один на оба.
- Без GUI: `headless:=true`, а управление через веб-страницу на `:8080`.

## Что пришлось учесть ради Lyrical

Проверка на двух дистрибутивах нашла две несовместимости, обе исправлены:

1. **`logger.warn()` удалён в rclpy** для Lyrical. Используется `logger.warning()`: без этого веб-пульт падал при нажатии E-STOP.
2. **`tests_require` игнорируется setuptools на Python 3.14**, и colcon тихо не запускал pytest-тесты. Теперь используется `extras_require={'test': ['pytest']}`.

CI (`.github/workflows/ci.yml`) собирает и тестирует обе версии, а также запускает `walk_check` в Gazebo на обеих.

## Как обновить эти данные

```bash
# теги и размеры образов
curl -s "https://hub.docker.com/v2/repositories/library/ros/tags?page_size=100&name=lyrical" | jq '.results[].name'
# версии пакетов в образе
docker run --rm ros:jazzy-ros-base bash -c 'dpkg -l | grep ros-jazzy-rclcpp'
docker run --rm osrf/ros:jazzy-simulation gz sim --versions
```
