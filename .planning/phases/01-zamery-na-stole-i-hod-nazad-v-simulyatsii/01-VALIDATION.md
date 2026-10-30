---
phase: "1"
slug: "zamery-na-stole-i-hod-nazad-v-simulyatsii"
# status lifecycle: draft (seeded by plan-phase) → validated (set by validate-phase §6)
# audit-milestone §5.5 distinguishes NOT-VALIDATED (draft) from PARTIAL (validated + nyquist_compliant: false) (#2117)
status: draft
nyquist_compliant: false
wave_0_complete: false
created: "2026-09-30"
---

# Phase 1 — Validation Strategy

> Контракт проверки фазы для обратной связи во время исполнения. Источник: `01-RESEARCH.md`, раздел «Validation Architecture».

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | GoogleTest (`ament_add_gtest`, ROS-free ядра `dog_control` и новый `dog_bench`), pytest (Python-модули, `tools/`, `dog_description`, `dog_gazebo`), `launch_testing` (существующий `test_mock_bringup`); проверки в Gazebo (`walk_check`, `acceptance`) — CLI, только в GitHub Actions |
| **Config file** | C++: `ros2_ws/src/dog_control/CMakeLists.txt` (`foreach(t …)`), новый `ros2_ws/src/dog_bench/CMakeLists.txt`; Python: `setup.py` с `extras_require={'test': ['pytest']}` (для нового `dog_gazebo/test` добавить) |
| **Quick run command** | `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q tools/robot_setup/test ros2_ws/src/dog_description/test` и локальная сборка gtest затронутого ROS-free ядра (g++/cmake, обёртка CMake лежит вне репозитория: `colcon` на этой машине нет) |
| **Full suite command** | CI: `colcon test --packages-skip dog_gazebo` (job `build-test`) и `gh workflow run ci.yml --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii` (jobs `build-test`, `simulation`, `terrain`) |
| **Estimated runtime** | быстрые локальные проверки ~30 секунд [ASSUMED]; `build-test` ~3 минуты; один элемент матрицы приёмки ~15-20 минут [ASSUMED, первый запуск с `repeats=1` это проверит] |

---

## Sampling Rate

- **After every task commit:** Run `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q <затронутые каталоги>` и локальную сборку gtest затронутого ядра (десятки секунд)
- **After every plan wave:** Run `gh workflow run ci.yml --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii` (полный CI); длинные симуляции только в GitHub Actions (D-24)
- **Before `/gsd-verify-work`:** полный CI зелёный, результат приёмки (4 элемента матрицы), порог `--backward-ratio` выставлен из данных, чекпоинты владельца выполнены
- **Max feedback latency:** 60 секунд локально; CI-проверки заметно дольше и идут по одной симуляции на раннер

---

## Per-Task Verification Map

Собрано из `<verify><automated>` задач планов скриптом (источник истины — сами планы). `Threat Ref` — идентификаторы из `<threat_model>` плана; `File Exists`: ❌ W0 = среди файлов задачи есть ещё не существующие.

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 01-01-01 | 01 | 1 | GAIT-02, GAIT-06 | T-01-01…04 | см. threat_model плана | source assertion | `test "$(git branch --show-current)" = gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii && git merge-base --is-ancestor main HEAD && L=$(git ls-re…` | ✅ | ⬜ pending |
| 01-01-02 | 01 | 1 | GAIT-02, GAIT-06 | T-01-01…04 | см. threat_model плана | CLI | `bash -n tools/ci_dispatch/ci_dispatch.sh && bash tools/ci_dispatch/ci_dispatch.sh --help \| grep -q -- '--wait-job' && bash tools/ci_dispatch/ci_dispa…` (+3) | ❌ W0 | ⬜ pending |
| 01-01-03 | 01 | 1 | GAIT-02, GAIT-06 | T-01-01…04 | см. threat_model плана | gtest | `bash tools/local_gtest/run.sh dog_control kinematics` (+2) | ❌ W0 | ⬜ pending |
| 01-02-01 | 02 | 2 | CAL-16, CAL-17, GAIT-10 | T-01-02-01…05 | см. threat_model плана | pytest | `python3 -m pytest -q tools/robot_setup/test/test_yaml_contract.py` (+2) | ❌ W0 | ⬜ pending |
| 01-02-02 | 02 | 2 | CAL-16, CAL-17, GAIT-10 | T-01-02-01…05 | см. threat_model плана | pytest | `python3 -m pytest -q tools/robot_setup/test/test_robot_setup.py -k "com_x or roundtrips"` (+1) | ✅ | ⬜ pending |
| 01-02-03 | 02 | 2 | CAL-16, CAL-17, GAIT-10 | T-01-02-01…05 | см. threat_model плана | pytest | `python3 -m pytest -q tools/robot_setup/test/test_robot_setup.py -k sensor` (+1) | ✅ | ⬜ pending |
| 01-02-04 | 02 | 2 | CAL-16, CAL-17, GAIT-10 | T-01-02-01…05 | см. threat_model плана | pytest | `python3 -m pytest -q tools/robot_setup/test/test_robot_setup.py -k knee` (+1) | ✅ | ⬜ pending |
| 01-03-01 | 03 | 2 | GAIT-01, GAIT-02 | T-01-03-01…05 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m pytest -q ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py` (+2) | ❌ W0 | ⬜ pending |
| 01-03-02 | 03 | 2 | GAIT-01, GAIT-02 | T-01-03-01…05 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m pytest -q ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py` | ❌ W0 | ⬜ pending |
| 01-03-03 | 03 | 2 | GAIT-01, GAIT-02 | T-01-03-01…05 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m pytest -q ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py` (+2) | ❌ W0 | ⬜ pending |
| 01-04-01 | 04 | 2 | CAL-17, GAIT-06 | T-01-04-01…04 | см. threat_model плана | gtest | `bash tools/local_gtest/run.sh dog_control servo_limits --gtest_filter='ServoLimits.PeakAtShippedGaitIs5p077:ServoLimits.KneeRatioScalesOnlyTheKnee' 2>…` (+2) | ❌ W0 | ⬜ pending |
| 01-04-02 | 04 | 2 | CAL-17, GAIT-06 | T-01-04-01…04 | см. threat_model плана | gtest | `bash tools/local_gtest/run.sh dog_control servo_limits 2>&1 \| grep -F '[ PASSED ] 8 tests.'` (+1) | ❌ W0 | ⬜ pending |
| 01-04-03 | 04 | 2 | CAL-17, GAIT-06 | T-01-04-01…04 | см. threat_model плана | gtest | `bash tools/local_gtest/run.sh dog_control servo_limits 2>&1 \| grep -F '[ PASSED ] 9 tests.'` (+4) | ❌ W0 | ⬜ pending |
| 01-05-01 | 05 | 2 | CAL-17 | T-01-05-01…07 | см. threat_model плана | gtest | `bash -c 'set -o pipefail; bash tools/local_gtest/run.sh dog_bench ina219_fast \| grep -F "[ PASSED ] 6 tests."'` (+1) | ❌ W0 | ⬜ pending |
| 01-05-02 | 05 | 2 | CAL-17 | T-01-05-01…07 | см. threat_model плана | gtest | `bash -c 'set -o pipefail; bash tools/local_gtest/run.sh dog_bench selftest \| grep -F "[ PASSED ] 5 tests."'` (+1) | ❌ W0 | ⬜ pending |
| 01-05-03 | 05 | 2 | CAL-17 | T-01-05-01…07 | см. threat_model плана | gtest | `mkdir -p ros2_ws/build/_local/dog_bench && g++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -Werror -I ros2_ws/src/dog_bench/include ros2_ws/src/dog_bench/…` (+1) | ❌ W0 | ⬜ pending |
| 01-06-01 | 06 | 2 | CAL-17 | T-01-06-01…05 | см. threat_model плана | pytest | `O=$(python3 -m pytest -q tools/servo_speed/tests -k "tracer or find_saturation or synth" 2>&1); RC=$?; echo "$O" \| tail -5; N=$(echo "$O" \| grep -Eo…` | ❌ W0 | ⬜ pending |
| 01-06-02 | 06 | 2 | CAL-17 | T-01-06-01…05 | см. threat_model плана | pytest | `O=$(python3 -m pytest -q tools/servo_speed/tests 2>&1); RC=$?; echo "$O" \| tail -5; N=$(echo "$O" \| grep -Eo '[0-9]+ passed' \| grep -Eo '[0-9]+' \|…` | ❌ W0 | ⬜ pending |
| 01-06-03 | 06 | 2 | CAL-17 | T-01-06-01…05 | см. threat_model плана | pytest | `O=$(python3 -m pytest -q tools/servo_speed/tests 2>&1); RC=$?; echo "$O" \| tail -5; N=$(echo "$O" \| grep -Eo '[0-9]+ passed' \| grep -Eo '[0-9]+' \|…` (+1) | ❌ W0 | ⬜ pending |
| 01-07-01 | 07 | 2 | CAL-16, CAL-17, GAIT-06, GAIT-10 | T-01-07-01…07 | см. threat_model плана | CLI | `python3 -c "import sys; sys.path.insert(0,'tools/robot_setup'); import robot_setup as r; t=open('.planning/phases/01-zamery-na-stole-i-hod-nazad-v-sim…` (+1) | ❌ W0 | ⬜ pending |
| 01-07-02 | 07 | 2 | CAL-16, CAL-17, GAIT-06, GAIT-10 | T-01-07-01…07 | см. threat_model плана | source assertion | `S=.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENT-SHEET.md; for s in '## Часть 2.' 'R100' 'R010' '0x41' '0x44' '0x45' 'do…` | ❌ W0 | ⬜ pending |
| 01-07-03 | 07 | 2 | CAL-16, CAL-17, GAIT-06, GAIT-10 | T-01-07-01…07 | см. threat_model плана | source assertion | `for s in SAF-09 SAF-17 'Phase 7' MG996R 'с ограничением тока' '3.5 В' 'V+' 'D-09'; do grep -qF -- "$s" docs/PARTS_ORDER.md \|\| { echo "нет: $s"; exit…` (+1) | ❌ W0 | ⬜ pending |
| 01-07-04 | 07 | 2 | CAL-16, CAL-17, GAIT-06, GAIT-10 | T-01-07-01…07 | см. threat_model плана | source assertion | `! grep -q '8/8' README.md docs/SIMULATION.md docs/DEPLOYMENT.md && test "$(grep -c '10/10 (stand, 8 манёвров, lie)' docs/DEPLOYMENT.md)" -ge 2 && for …` (+2) | ✅ | ⬜ pending |
| 01-08-01 | 08 | 3 | CAL-17, GAIT-06 | T-01-08-01…06 | см. threat_model плана | gtest | `bash tools/local_gtest/run.sh dog_control locomotion --gtest_filter='Locomotion.AutoPeriodAtStart:Locomotion.JointSpeedsFitTheServos:Locomotion.JointS…` (+3) | ❌ W0 | ⬜ pending |
| 01-08-02 | 08 | 3 | CAL-17, GAIT-06 | T-01-08-01…06 | см. threat_model плана | gtest | `bash tools/local_gtest/run.sh dog_control locomotion --gtest_filter='Locomotion.ReconfigureAcceptedWhenStanding:Locomotion.ReconfigureRejectedWhenWalk…` (+2) | ✅ | ⬜ pending |
| 01-08-03 | 08 | 3 | CAL-17, GAIT-06 | T-01-08-01…06 | см. threat_model плана | source assertion | `N=ros2_ws/src/dog_control/src/locomotion_node.cpp; for k in 'add_on_set_parameters_callback' 'declare_parameter("gait.auto_period"' 'declareNumber("ga…` (+2) | ✅ | ⬜ pending |
| 01-08-04 | 08 | 3 | CAL-17, GAIT-06 | T-01-08-01…06 | см. threat_model плана | command | `OUT=$(git push origin gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii 2>&1); RC=$?; printf '%s\n' "$OUT" \| sed -E 's#https?://[^ ]+#<remote-hid…` (+2) | ❌ W0 | ⬜ pending |
| 01-09-01 | 09 | 3 | CAL-17 | T-01-09-01…13 | см. threat_model плана | gtest | `bash -c 'set -o pipefail; bash tools/local_gtest/run.sh dog_bench ramp \| grep -F "[ PASSED ] 6 tests." && bash tools/local_gtest/run.sh dog_bench ina…` | ❌ W0 | ⬜ pending |
| 01-09-02 | 09 | 3 | CAL-17 | T-01-09-01…13 | см. threat_model плана | gtest | `bash -c 'set -o pipefail; bash tools/local_gtest/run.sh dog_bench safety \| grep -F "[ PASSED ] 9 tests." && for t in ina219_fast selftest ramp; do ba…` | ❌ W0 | ⬜ pending |
| 01-09-03 | 09 | 3 | CAL-17 | T-01-09-01…13 | см. threat_model плана | gtest | `bash -c 'set -o pipefail; for t in ina219_fast:6 selftest:5 ramp:6 safety:9 pwm_out:6; do bash tools/local_gtest/run.sh dog_bench ${t%%:*} \| grep -F …` | ❌ W0 | ⬜ pending |
| 01-10-01 | 10 | 3 | CAL-16, GAIT-10 | T-01-10-01…05 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test -k "ideal_urdf_unchanged or factors or voltage_clamped or…` | ❌ W0 | ⬜ pending |
| 01-10-02 | 10 | 3 | CAL-16, GAIT-10 | T-01-10-01…05 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test/test_servo_profile.py` | ❌ W0 | ⬜ pending |
| 01-10-03 | 10 | 3 | CAL-16, GAIT-10 | T-01-10-01…05 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test -k "body_com_x or ideal_urdf_unchanged"` | ✅ | ⬜ pending |
| 01-10-04 | 10 | 3 | CAL-16, GAIT-10 | T-01-10-01…05 | см. threat_model плана | command | `python3 -m py_compile ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py ros2_ws/src/dog_gazebo/launch/sim.launch.py` (+2) | ✅ | ⬜ pending |
| 01-11-01 | 11 | 3 | GAIT-01, GAIT-02, GAIT-06 | T-01-11-01…06 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo:ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_gazebo/test/test_walk_check_args.py` (+1) | ❌ W0 | ⬜ pending |
| 01-11-02 | 11 | 3 | GAIT-01, GAIT-02, GAIT-06 | T-01-11-01…06 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo:ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_gazebo/test/test_acceptance_cli.py` (+1) | ❌ W0 | ⬜ pending |
| 01-11-03 | 11 | 3 | GAIT-01, GAIT-02, GAIT-06 | T-01-11-01…06 | см. threat_model плана | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo:ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_gazebo/test` (+1) | ❌ W0 | ⬜ pending |
| 01-12-01 | 12 | 4 | CAL-17 | T-01-12-01…09 | см. threat_model плана | gtest | `bash -c 'set -o pipefail; for t in ina219_fast:6 selftest:5 ramp:6 safety:9 pwm_out:6 session:14; do bash tools/local_gtest/run.sh dog_bench ${t%%:*} …` (+1) | ❌ W0 | ⬜ pending |
| 01-12-02 | 12 | 4 | CAL-17 | T-01-12-01…09 | см. threat_model плана | gtest | `bash -c 'set -u; mkdir -p ros2_ws/build/_local/dog_bench && g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror -I ros2_ws/src/dog_bench/include ros2_…` (+1) | ❌ W0 | ⬜ pending |
| 01-12-03 | 12 | 4 | CAL-17 | T-01-12-01…09 | см. threat_model плана | command | `bash -c 'mkdir -p ros2_ws/build/_ci; git push origin gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii >ros2_ws/build/_ci/push.raw 2>&1; S=$?; sed…` (+1) | ❌ W0 | ⬜ pending |
| 01-13-01 | 13 | 5 | CAL-17, GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-13-01…07 | см. threat_model плана | CLI | `python3 -c ' import yaml d = yaml.safe_load(open(".github/workflows/ci.yml")) wd = (d.get("on") or d.get(True))["workflow_dispatch"]["inputs"] assert …` (+2) | ✅ | ⬜ pending |
| 01-13-02 | 13 | 5 | CAL-17, GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-13-01…07 | см. threat_model плана | pytest | `python3 -c 'import yaml; d = yaml.safe_load(open(".github/workflows/ci.yml")); j = d["jobs"]["servo-speed"]; assert j["name"] == "servo speed analysis…` (+2) | ✅ | ⬜ pending |
| 01-13-03 | 13 | 5 | CAL-17, GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-13-01…07 | см. threat_model плана | pytest | ``; журнал в `ros2_ws/build/_ci/trial.log` (там строка `RUN_ID=<id>`). Запуск `ci.yml` гоняет все job (15-25 минут); скрипт ждёт только выбранные и ска…` (+2) | ✅ | ⬜ pending |
| 01-13-04 | 13 | 5 | CAL-17, GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-13-01…07 | см. threat_model плана | CLI | `python3 -c ' import json, os d = ".planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/baseline" sha = open("ros2_ws/build/_ci/bas…` (+2) | ❌ W0 | ⬜ pending |
| 01-14-01 | 14 | 6 | CAL-17 | T-01-14-01…06 | см. threat_model плана | checkpoint (owner) | `bash -c 'B=gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii; CUR=$(git branch --show-current); [ "$CUR" = "$B" ] && T=$(mktemp) && { git push ori…` | ❌ W0 | ⬜ pending |
| 01-14-02 | 14 | 6 | CAL-17 | T-01-14-01…06 | см. threat_model плана | checkpoint (owner) | `bash -c 'set -u; mkdir -p ros2_ws/build/_local/dog_bench; g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror -I ros2_ws/src/dog_bench/include ros2_ws…` | ❌ W0 | ⬜ pending |
| 01-14-03 | 14 | 6 | CAL-17 | T-01-14-01…06 | см. threat_model плана | CLI | `python3 -c "import json,os; d='.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/'; r=os.environ.get('RUN','run_01'); m=json.l…` (+2) | ❌ W0 | ⬜ pending |
| 01-14-04 | 14 | 6 | CAL-17 | T-01-14-01…06 | см. threat_model плана | checkpoint (owner) | `python3 -c "import json,re; d='.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/'; a=json.load(open(d+'servo_speed/analysis_01.json')); t…` | ❌ W0 | ⬜ pending |
| 01-15-01 | 15 | 7 | CAL-16, CAL-17, GAIT-06 | T-01-15-01…07 | см. threat_model плана | checkpoint (owner) | `python3 -c " import re,sys sys.path.insert(0,'tools/robot_setup') import robot_setup as r v=dict(re.findall(r'^([a-z0-9_]+) = (\S+)',open(sys.argv[1],…` | ❌ W0 | ⬜ pending |
| 01-15-02 | 15 | 7 | CAL-16, CAL-17, GAIT-06 | T-01-15-01…07 | см. threat_model плана | CLI | `python3 -c " import re,sys sys.path.insert(0,'tools/robot_setup') import robot_setup as r raw=dict(re.findall(r'^([a-z0-9_]+) = (\S+)',open(sys.argv[1…` (+3) | ❌ W0 | ⬜ pending |
| 01-15-03 | 15 | 7 | CAL-16, CAL-17, GAIT-06 | T-01-15-01…07 | см. threat_model плана | CLI | `python3 -c " import re,sys,yaml t=open(sys.argv[1]+'/01-SERVO-SPEED-RESULT.md',encoding='utf-8').read() g=lambda k: float(re.findall('^'+k+r' = (\S+)'…` (+3) | ❌ W0 | ⬜ pending |
| 01-15-04 | 15 | 7 | CAL-16, CAL-17, GAIT-06 | T-01-15-01…07 | см. threat_model плана | command | `{ P=$(git push origin gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii 2>&1); rc=$?; } && sed -E 's#https?://[^ ]+#[hidden]#g' <<< "$P" && test "…` (+1) | ❌ W0 | ⬜ pending |
| 01-16-01 | 16 | 8 | GAIT-01, GAIT-06 | T-01-16-01…06 | см. threat_model плана | CI-only | `bash tools/ci_dispatch/ci_dispatch.sh --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii -f acceptance=true -f repeats=5 -f cells=all -f stri…` (+2) | ❌ W0 | ⬜ pending |
| 01-16-02 | 16 | 8 | GAIT-01, GAIT-06 | T-01-16-01…06 | см. threat_model плана | gtest | `P=.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-POST-NUMBERS.md; G=$(sed -n 's/^gait_changes: //p' "$P"); S=$(sed -n 's/^code_sha_b…` (+1) | ✅ | ⬜ pending |
| 01-16-03 | 16 | 8 | GAIT-01, GAIT-06 | T-01-16-01…06 | см. threat_model плана | CI-only | `P=.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-POST-NUMBERS.md; G=$(sed -n 's/^gait_changes: //p' "$P") if [ "$G" = not_needed ]; …` (+1) | ❌ W0 | ⬜ pending |
| 01-17-01 | 17 | 9 | GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-17-01…06 | см. threat_model плана | pytest | `mkdir -p ros2_ws/build/_ci && PYTHONPATH=ros2_ws/src/dog_gazebo:ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_gazebo/test && { P=$(…` (+4) | ❌ W0 | ⬜ pending |
| 01-17-02 | 17 | 9 | GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-17-01…06 | см. threat_model плана | CLI | `PYTHONPATH=ros2_ws/src/dog_gazebo python3 -c ' import json, re from dog_gazebo import acceptance_stats as S D = ".planning/phases/01-zamery-na-stole-i…` | ❌ W0 | ⬜ pending |
| 01-17-03 | 17 | 9 | GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-17-01…06 | см. threat_model плана | CLI | `python3 -c ' import re, subprocess, yaml D = ".planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii" kv = dict(re.findall(r"^([a-z_0-9]+): (.*…` (+1) | ✅ | ⬜ pending |
| 01-17-04 | 17 | 9 | GAIT-01, GAIT-02, GAIT-06, GAIT-10 | T-01-17-01…06 | см. threat_model плана | command | `RID=$(sed -n 's/^RUN_ID=\([0-9][0-9]*\)$/\1/p' ros2_ws/build/_ci/final-push.log \| tail -n 1) && test -n "$RID" && C=$(cat ros2_ws/build/_ci/ci_commit…` (+3) | ❌ W0 | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

- [ ] `ros2_ws/src/dog_control/test/test_servo_limits.cpp` — CAL-17, GAIT-06 (+ имя в `foreach(t kinematics gait crawl greet locomotion)`)
- [ ] `ros2_ws/src/dog_description/test/test_servo_profile.py` и кейсы в `test_urdf.py` — GAIT-10, CAL-16
- [ ] `ros2_ws/src/dog_gazebo/test/` (каталог, `test_acceptance_stats.py`, `test_walk_check_args.py`) и `extras_require={'test': ['pytest']}` в `dog_gazebo/setup.py` — GAIT-01, GAIT-02
- [ ] `ros2_ws/src/dog_bench/` (пакет, CMake, `test/test_*.cpp`, `package.xml` format 3) — CAL-17
- [ ] `tools/servo_speed/tests/test_analyze.py` и `synth.py` — CAL-17
- [ ] `tools/robot_setup/test/` кейсы: `body_com_x`, передаточное число колена (золотые значения из C++), согласованность трёх скоростей в YAML, ошибки геометрии несуществующих датчиков — CAL-16, CAL-17
- [ ] `.github/workflows/ci.yml`: `inputs` и job `acceptance` (+ небольшой job тестов `tools/servo_speed`) — GAIT-02
- [ ] Установка фреймворков не нужна: g++, cmake, gtest, pytest, numpy, pyyaml на машине уже есть

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Замеры на столе по листу `01-MEASUREMENT-SHEET.md` (длины, массы, центр масс по X, четыре длины тяги колена, пределы суставов) | CAL-16 | нужен физический робот и инструменты владельца | владелец присылает таблицу в чат; Claude вносит через `robot_setup --cli`, проверяет `--check`, запускает `walk_check` в CI (D-26) |
| Мини-проверка питания мультиметром: 6.0 В на шине, общая земля, VCC PCA9685 = 3.3 В, конденсатор на V+ | CAL-17 | защита оборудования до первого включения | владелец по листу проверки перед `selftest` (D-10) |
| Маркировка шунта INA219 (R100 или R010) и угол между двумя импульсами транспортиром | CAL-17 | значения нельзя узнать программно | владелец сообщает маркировку и угол; значения идут в параметры `--shunt-ohm` и калибровку µs → рад |
| `selftest` и `run` инструмента замера скорости на роботе (одна серва `lf_thigh_joint`, рука на питании) | CAL-17 | нужен робот и INA219 | `docker compose run --rm robot ros2 run dog_bench …` по листу шагов; результат идёт в анализ `tools/servo_speed` |
| Одобрение измеренной скорости и итоговой доли ворот | CAL-17, GAIT-06 | решение владельца (D-11: одна серва не оценивает разброс) | владелец смотрит график «расстояние между следами от скорости» и подтверждает число |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 60s (локально)
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
