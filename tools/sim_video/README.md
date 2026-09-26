# Видео прогонов симуляции

Два скрипта делают ролики для отчётов из реальных прогонов. ROS для них не нужен: нужны Python 3, `numpy`, `matplotlib`, `imageio-ffmpeg` и `opencv-contrib-python` (для калибровки).

## Ходьба по рельефу

Сначала запишите прогон. `terrain_sweep` с `--record-dir` сохраняет углы суставов, положение корпуса и IMU с частотой 30 Гц:

```bash
ros2 run dog_gazebo terrain_sweep --terrain slope --levels 10 --record-dir rec/on --out on.json
ros2 run dog_gazebo terrain_sweep --terrain slope --levels 10 --record-dir rec/off \
  --launch-arg slope_compensation:=false --out off.json
```

Затем сделайте видео:

```bash
python3 tools/sim_video/render.py rec/on/slope_10.json slope10_on.mp4 \
  --title "Уклон 10° · IMU включена" --subtitle "подъём, поперёк склона, развороты, спуск"
```

В кадре:
- робот на рельефе: пандус, волны или камни, та же геометрия, что в мире Gazebo;
- список манёвров с итогом walk_check;
- наклон корпуса к поверхности;
- тангаж и крен по IMU;
- какие ноги в воздухе.

Камера поворачивается вслед за роботом. Для предпросмотра куска используйте `--start` и `--end` в секундах.

## Автокалибровка по камере

```bash
python3 tools/sim_video/calib_video.py calibration.mp4
```

Скрипт запускает `robotdog_autocal demo` на виртуальном роботе со скрытыми ошибками и записывает каждое из 240 измерений. В кадре:
- кадр виртуальной камеры с найденными метками;
- робот на подставке и положение камеры;
- точки «импульс → угол» текущего сустава и подобранная модель;
- ошибка нуля всех 12 суставов до и после.

## Датчики и реакция на препятствия

```bash
ros2 launch dog_gazebo sim.launch.py headless:=true web:=false perception:=true terrain:=steps level:=20 &
ros2 run dog_gazebo perception_check --terrain steps --level 20 --seconds 45 --trace steps20.json
python3 tools/sim_video/perception_video.py steps20.json steps20.mp4 --title "Ступеньки 20 мм"
```

В кадре точки лидаров, лучи ToF, линия GS2, состояние реакции (guard) и высота шага каждой ноги.

## HTML-отчёты (`report/`)

Сборщики лежат в [report/](report): `build_report.py` делает `report/index.html`, `build_perception_report.py` — `report/perception.html`, `build_gaits_report.py` — `report/gaits.html` (ползание, объезд, приветствие; оценки — `collect.py gaits`). Им нужны только маленькие файлы из `report/data/` и ролики, которые уже лежат в `report/videos/`. Сырые записи прогонов (мегабайты на прогон) в git не хранятся.

```bash
pip install opencv-python-headless imageio-ffmpeg
python3 tools/sim_video/report/build_report.py --out report
python3 tools/sim_video/report/build_perception_report.py --out report
python3 tools/sim_video/report/build_gaits_report.py --out report
```

Ролики страницы походок рендерятся по списку `report/data/gaits_runs.json` (длинные прогоны ползания — ускоренно, `--speed`, и обрезанные после препятствия, `--end`):

```bash
python3 - <<'PY'
import json, subprocess
for r in json.load(open('tools/sim_video/report/data/gaits_runs.json')):
    cmd = ['python3', 'tools/sim_video/perception_video.py', f"rec/{r['dir']}/{r['name']}.json", f"raw/{r['name']}.mp4",
           '--title', r['setup'], '--speed', str(r.get('speed', 1))] + \
          (['--start', str(r['start'])] if 'start' in r else []) + (['--end', str(r['end'])] if 'end' in r else [])
    subprocess.run(cmd, check=True)
PY
python3 tools/sim_video/report/collect.py gaits --root rec
python3 tools/sim_video/report/build_gaits_report.py --out report --raw raw
```

После новых прогонов:
1. `collect.py` обновляет данные из сырых записей:
   ```bash
   python3 tools/sim_video/report/collect.py perception --root <папка с записями>   # список прогонов: data/perception_runs.json
   python3 tools/sim_video/report/collect.py walk <корень> rec/on_flat_1/flat_0.json ...
   python3 tools/sim_video/report/collect.py tests --jazzy <build Jazzy> --lyrical <build Lyrical>
   ```
2. Свежие ролики передаются через `--raw <папка>`: сборщик пережимает их в 960×540 H.264 и кладёт в `report/videos/` вместе с обложкой.

| Файл | Что внутри |
|---|---|
| `data/walk.json` | итоги `walk_check` / `terrain_sweep` по каждому прогону ходьбы |
| `data/tests.json` | тест-кейсы по пакетам на Jazzy и Lyrical |
| `data/perception_runs.json` | прогоны `perception_check`: папка, имя, мир, настройка |
| `data/perception.json` | их оценки (`scores` из записи, обнаружение пересчитано текущими правилами) |
| `template.html` | страница основного отчёта и общий стиль |
