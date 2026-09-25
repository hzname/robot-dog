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
