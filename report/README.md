# Отчёты испытаний RobotDog 2.0

| Файл | Что внутри |
|---|---|
| [index.html](index.html) | Полный цикл: тесты, калибровка по камере, ровный пол, уклоны и неровности с IMU и без, удержание курса, пределы. 24 ролика |
| [perception.html](perception.html) | Лидары «крестом», VL53L1X и линейный лидар GS2: точность пола, камни и ступеньки, ложные срабатывания, нагрузка на процессор. 20 роликов |
| [gaits.html](gaits.html) | Походки: ползание (ступени, брус, лестница), стоп у стены, объезд блока, приветствие. 6 роликов |

Ролики лежат отдельно, в [videos/](videos) (MP4 H.264, 960×540), обложки — в [posters/](posters).

**Как открыть.** GitHub показывает HTML исходным текстом. Варианты:
- склонировать репозиторий и открыть `report/index.html` в Chrome, Edge или Safari;
- включить GitHub Pages для ветки (папка `/report`);
- смотреть ролики по отдельности прямо в [videos/](videos): GitHub проигрывает MP4.

Всё собрано из прогонов в Gazebo Harmonic. Как повторить:
- ходьба — [docs/SIMULATION.md](../docs/SIMULATION.md), [docs/TERRAIN.md](../docs/TERRAIN.md);
- датчики — [docs/PERCEPTION.md](../docs/PERCEPTION.md);
- рендер роликов и сборка этих страниц — [tools/sim_video](../tools/sim_video/README.md).
