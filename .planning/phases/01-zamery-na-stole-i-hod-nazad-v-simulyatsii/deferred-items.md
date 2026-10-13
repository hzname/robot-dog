# Deferred Items — Phase 01

Out-of-scope discoveries logged during plan 01-10 execution (not fixed here, per the executor scope boundary).

## `tools/ci_dispatch/ci_dispatch.sh` — run-state parse shifts fields while the run is in progress

- **Found during:** 01-10 Task 4 (CI dispatch).
- **Issue:** the waiter reads `gh run view --json status,conclusion` as one TSV line. While a run is in progress `conclusion` is an empty string and `(.conclusion // "-")` does not substitute for it; the resulting double tab collapses under bash `read`, so the loop prints `warning: incomplete status` on every poll and only resolves once the run concludes. After a `gh run rerun --failed`, the tool can also decide from the pre-rerun attempt if the run had already concluded before the rerun (seen here: it reported `build + test (lyrical) cancelled` although the rerun attempt had made it `success`).
- **Suggested fix:** map empty conclusions explicitly, e.g. `(.conclusion // "-" | if . == "" then "-" else . end)`, or parse the JSON fields individually instead of a TSV line.
- **Owner:** the tool shipped with plan 01-01; not modified in 01-10 (outside the plan's files).

## GitHub Actions cancelled queued jobs after ~15 min without a runner

- **Found during:** 01-10 Task 4; two events, 19:35:31Z and 20:00:13Z, each ~15 min 0 s after the jobs were queued.
- **Effect:** four jobs concluded `cancelled` without ever getting a runner (`runner_id = 0`, no steps): `build + test (lyrical)` in the first event; `robot image (arm64)` and `robot parameter form (tools/robot_setup)` in the second. The two walk checks and `build + test (jazzy)` stayed green.
- **Workaround used:** `gh run rerun --failed` (run attempt 2) — `build + test (lyrical)` passed; `ci_dispatch.sh --run-id …` then exited 0 with all four awaited jobs success.
- **Watch:** if long-queued jobs keep being cancelled, plan 01-13's acceptance matrix (a long run on a busy runner) may need the same rerun handling.

## `gh run download` aborts on the `robot image` dockerbuild artifact and fetches nothing

- **Found during:** 01-13 Task 3/4 (trial and baseline artifact downloads).
- **Issue:** `gh run download <run> --dir DIR` (no filter) stops at the artifact `hzname~robot-dog~<id>.dockerbuild` (`error extracting zip archive: zip: not a valid zip file`) and leaves the target directory empty — so `ci_dispatch.sh --download` delivers no acceptance artifacts for any full `ci.yml` run. Seen on runs 37414394613, 37416318962, 37416429124 (the tool prints `warning: artifact download failed` and still exits 0).
- **Workaround used:** `gh run download <run> --pattern 'acceptance-*' --dir DIR` downloads only the needed artifacts; all three runs' acceptance artifacts were fetched this way.
- **Suggested fix:** in `ci_dispatch.sh`, when `--download` is set, list the run artifacts (`gh api .../artifacts`) and fetch them one by one with `--name`, warning and skipping the failed ones instead of aborting.
- **Owner:** tool shipped with plan 01-01; will also hit plans 01-16/01-17 downloads (`post-numbers`, `post-fix`, `final5`, `final10`).

## Стенд 09.10: скорости переходов «встать/лечь» и вывод серв из расслабления

- **Found during:** живой прогон стека на подставке (09.10.2026; продолжение чекпоинта владельца на 01-14).
- **Скорости переходов:** «встать» и «лечь» исполняются слишком быстро — инерция корпуса бьёт по сервам; решение владельца — втрое медленнее. **Сделано:** `stance.transition_time` 1.5 → 4.5 с (временно, на обкатку; вернуть/подобрать при тюнинге походки, Phase 7–8).
- **Расслабление серв:** при отпускании выходов робот ложится на пузо, ноги распускаются хаотично; после рестарта выходы подхватывают прежнюю позу (09.10: жёсткое снятие контейнера `docker rm -f` не дало драйверу отпустить сервы — чип остался с позой «лёжа», и она «всплыла» при следующем старте страницы; лечится «держать»). Требование владельца: либо класть робота прямо на пузо, либо выводить из сна максимально плавно, если позволяет железо.
- **Куда идёт:** инженерная часть — SAF-12 (Phase 3): выходы гасятся при перезапуске, первый выход на цель — с ограничением скорости (см. также `docs/REVIEW.md` п. 27–28). Оговорка: у MG996R нет обратной связи — полностью плавный вывод из произвольного расслабленного положения недостижим; смягчают рампинг импульса от известной позы, поза покоя «на пузе», первый переход из «лёжа» и штатное поочерёдное включение ног (0.15 с).
- **Owner:** Phase 3 (драйвер, SAF-12) и Phase 7–8 (порядок работ на железе, тюнинг).

## Quick 261013-e4x: колено связано с бедром

- **Found during:** quick-задача «пределы бедра и колена: карта допустимых положений» (13.10.2026), входные данные — CAD владельца (Fusion 360, серва SPT5435LV вместо MG996R), не замер.
- **Противоречие с D-20 (ждёт подтверждения владельца):** 01-CONTEXT.md D-20 исходит из тяги колена, опирающейся на бедро (`coupled_to` не нужен). CAD показывает серву колена на корпусе с двойным параллелограммом, поэтому в `servos.yaml` теперь `coupled_to: <нога>_thigh_joint`, `coupling: 1.0`, `servo_arm_mm: 0.0`. 01-CONTEXT.md не правился; поправка записана в `docs/DEPLOYMENT.md` (этап 1, «Тяга»). Владельцу подтвердить по роботу; если колено окажется независимым — вернуть прежние строки (`offset_deg −90`, окно 180°) и пределы −165…−15 в `robot.yaml`/`urdf.py`.
- **Открытые пункты в `docs/REVIEW.md`:** п. 29 (положение окна: числа CAD, ползком и `greet` вне области), п. 30 (инструменты под независимое колено, `calib_pose` по умолчанию), п. 31 (`greet` требует s до −121°), п. 32 (`thigh_limits_deg` min −45° ниже упора CAD q1 ≤ −35°; скорость связанной сервы).
- **Корпус с опорами для лежания** (CAD, не в репозитории): 4 сегмента U-профиля, высота 60 мм под низом корпуса; `lie_height` около 0.100 только с опорами, у текущего робота без них остаётся 0.08. Файлы: `C:\Users\Lenovo\Downloads\robotdog_supports\` (STL 4 шт., STEP).
- **Геометрия** (`thigh`, `calf`, `hip_*`) не менялась: остаётся планом 01-15 (замеры в `01-MEASUREMENTS-RAW.md`).
- **Проверка:** смена предела колена в URDF (−165…−15 → −140…−40) проверяется только в CI симуляцией (`walk_check`, `terrain_sweep`); пуша из этой задачи нет. Прогон `LocomotionController` без ROS показал, что `crawl` и `greet` выходят из области (см. п. 29, 31).
- **Честный статус сборки:** C++-тест драйвера собран и пройден локально `g++ -Wall -Wextra -Wpedantic -Werror` с gtest (24 теста); `colcon build`/`colcon test` для Jazzy и Lyrical, Gazebo-проверки CI и поведение на роботе не запускались (в среде нет ROS).
- **Owner:** Phase 3 (драйвер, скорость связанной сервы), Phase 7–8 (замер s0 на 1370 мкс, окно после калибровки), план 01-15 (геометрия).
