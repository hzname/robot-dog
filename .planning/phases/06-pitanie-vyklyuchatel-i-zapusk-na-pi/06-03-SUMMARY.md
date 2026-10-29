# 06-03: образ arm64 доказывается запуском — сводка волны

Код `3e05e47`.

## Что сделано

1. Джоба `robot image (arm64) boots the stack`: buildx с `load: true`, таймаут 45 мин.
2. Шаг «preflight отказывает»: в контейнере нет `/dev/i2c-0`, скрипт обязан вернуть 2 —
   иначе проверка была бы резиновым штампом. В логе видно причину.
3. Шаг «образ поднимает стек»: `docker/ci_smoke.sh` в образе запускает
   `robot.launch.py backend:=mock gamepad:=false web:=false imu:=mock power:=mock` и ждёт
   `/dog/state` (до ~90 с, с проверкой, что процесс жив); при неудаче печатает `ros2 node
   list`, `ros2 topic list` и хвост лога.
4. Скрипт лежит в образе, поэтому локальная проверка и CI гоняют один и тот же файл.

## Грабли

- **Entrypoint видит команду, а не аргументы запуска.** `docker run … bash ci_smoke.sh`
  дал entrypoint строку `bash ci_smoke.sh`, preflight ушёл в режим железа и заблокировал
  контейнер (CI: exit code 2). Лечение: скрипт сам спрашивает preflight с `backend:=mock`,
  а CI передаёт `DOG_PREFLIGHT=off` — обход назван явно.
- Сборка образа под qemu идёт ~25 минут: гонять её надо один раз за итерацию, а правки
  проверять локально на том же Dockerfile (amd64) — так и был найден дефект выше.

## Проверки

| Требование | Где | Чем проверено |
|---|---|---|
| CAL-09 (часть 2) | `.github/workflows/ci.yml` (джоба `robot-image`), `docker/ci_smoke.sh`, `docker/Dockerfile` | локально: preflight → 2, смоук → `/dog/state answered`; в CI: та же джоба на arm64 |
