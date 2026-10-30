---
last_mapped_commit: d269bd16b0d0f69f3ab623b9cecc4ba545b70517
last_mapped_at: 2026-09-29
---
# Codebase Structure

**Analysis Date:** 2026-09-29

## Directory Layout

```
robot-dog/
├── ros2_ws/                       # Active ROS 2 (Jazzy/Lyrical) colcon workspace (v2)
│   └── src/
│       ├── dog_control/           # C++: kinematics, gaits, locomotion state machine + locomotion_node
│       │   ├── include/dog_control/   # headers (crawl, gait, greet, kinematics, locomotion, odometry, survey)
│       │   ├── src/                   # implementations + locomotion_node.cpp
│       │   └── test/                  # gtest: test_<unit>.cpp
│       ├── dog_hardware/          # C++: PCA9685 driver, calibration, IMU, power monitor
│       │   ├── include/dog_hardware/
│       │   ├── src/                   # *_node.cpp + core .cpp, pca9685_probe.cpp
│       │   └── test/                  # gtest + test_power_monitor.py
│       ├── dog_teleop/            # C++: gamepad, joy_teleop, keyboard
│       ├── dog_web/               # Python: web teleop (HTTP+WebSocket) + static page
│       │   ├── dog_web/               # protocol.py, wsserver.py, calibration.py, web_teleop.py
│       │   ├── static/                # index.html, app.js, style.css
│       │   └── test/
│       ├── dog_description/       # Python: URDF generator from robot.yaml
│       ├── dog_bringup/           # launch + config + calib_pose + mock bringup test
│       │   ├── launch/robot.launch.py
│       │   ├── config/                # robot.yaml, servos.yaml, teleop*.yaml, imu.yaml, power.yaml, dog.rviz
│       │   ├── scripts/calib_pose
│       │   └── test/test_mock_bringup.py
│       ├── dog_gazebo/            # Python: sim launch, world generator, bridges, checks
│       │   ├── launch/sim.launch.py
│       │   ├── worlds/flat.sdf
│       │   └── dog_gazebo/            # terrain.py, joint_command_bridge.py, tof_bridge.py, *_check.py, terrain_sweep.py
│       └── dog_perception/        # C++ (+numpy): perception_node, localization_node
│           ├── include/dog_perception/  # core.hpp, localization.hpp, submaps.hpp
│           ├── src/
│           ├── dog_perception/core.py   # numpy port
│           └── test/
├── robot_dog_ws/                  # UNTRACKED, incomplete JSON-config experiment (no package.xml/CMakeLists)
│   └── src/{dog_hardware,dog_hardware_cpp,dog_web}/
├── robot_configurator.py          # UNTRACKED: validates/computes servo_config.json (_computed sections)
├── test_servo_config_reader.sh    # UNTRACKED: on-robot (Banana Pi over SSH) build/test script
├── tools/                         # Non-ROS utilities
│   ├── robot_setup/               # web/CLI form editing robot.yaml + servos.yaml (+ test/)
│   ├── autocal/                   # robotdog_autocal package: camera servo auto-calibration (+ tests/)
│   └── sim_video/                 # render.py, *_video.py, report/ (build_*.py, data/*.json, template.html)
├── docs/                          # Russian-language design docs (ARCHITECTURE, CONTROL, DEPLOYMENT, HARDWARE, ...) + img/
├── report/                        # Generated HTML reports (index/gaits/localization/perception.html), posters/, videos/
├── docker/                        # Dockerfile (robot), Dockerfile.sim, entrypoint.sh
├── docker-compose.yml             # robot service
├── legacy/v1/                     # Previous version, not built (COLCON_IGNORE); hardware data source
├── .github/workflows/ci.yml       # build+test (jazzy, lyrical), autocal, robot-setup, gazebo simulation
├── README.md, LICENSE
└── .planning/                     # GSD planning artifacts (out of scope)
```

## Directory Purposes

**`ros2_ws/src/dog_control/`:**

- Purpose: Motion brain. Core library `dog_control_core` (no ROS) + `locomotion_node`.
- Contains: `include/dog_control/*.hpp`, `src/*.cpp`, `test/test_*.cpp`.
- Key files: `include/dog_control/locomotion.hpp`, `src/locomotion.cpp`, `src/locomotion_node.cpp`, `src/gait.cpp`, `src/crawl.cpp`, `src/kinematics.cpp`.

**`ros2_ws/src/dog_hardware/`:**

- Purpose: Everything touching I2C hardware.
- Key files: `src/servo_driver.cpp`, `src/servo_driver_node.cpp`, `src/servo_bus.cpp`, `src/imu_node.cpp`, `src/power_monitor_node.cpp`.

**`ros2_ws/src/dog_teleop/`:**

- Purpose: Local operator inputs. Mapping logic in `src/mapping.cpp` (library `dog_teleop_mapping`), nodes in `src/*_node.cpp` and `src/keyboard_teleop.cpp`.

**`ros2_ws/src/dog_web/`:**

- Purpose: Browser control and calibration WebSocket. Static assets under `static/` are installed via glob in `setup.py`.

**`ros2_ws/src/dog_perception/`:**

- Purpose: Terrain guard and localization. `src/core.cpp` (perception), `src/localization.cpp`, `src/submaps.cpp`, nodes `src/perception_node.cpp`, `src/localization_node.cpp`.

**`ros2_ws/src/dog_bringup/`:**

- Purpose: Robot-side composition and configuration. All tunable parameters live in `config/`.

**`ros2_ws/src/dog_gazebo/`:**

- Purpose: Simulation only (skipped in Docker robot image). Worlds are generated in Python (`dog_gazebo/terrain.py`) except `worlds/flat.sdf`.

**`tools/`:**

- Purpose: Laptop/CI utilities; must not require ROS (`autocal`, `robot_setup`, `sim_video`).

**`docs/`:**

- Purpose: Long-form documentation in Russian; `docs/ARCHITECTURE.md` describes nodes/topics/modes.

**`legacy/v1/`:**

- Purpose: Reference only; do not modify or build.

## Key File Locations

**Entry Points:**

- `ros2_ws/src/dog_bringup/launch/robot.launch.py`: robot launch
- `ros2_ws/src/dog_gazebo/launch/sim.launch.py`: simulation launch
- `ros2_ws/src/dog_control/src/locomotion_node.cpp`: locomotion node `main`
- `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`: servo driver node
- `ros2_ws/src/dog_web/dog_web/web_teleop.py`: web node (`web_teleop` console script)
- `docker/entrypoint.sh`: container entry

**Configuration:**

- `ros2_ws/src/dog_bringup/config/robot.yaml`: geometry, stance, gait, crawl, greet, survey, slope, heading, perception, localization, description/sensors
- `ros2_ws/src/dog_bringup/config/servos.yaml`: per-joint PCA9685 calibration
- `ros2_ws/src/dog_bringup/config/teleop.yaml`, `teleop_ps.yaml`: teleop limits and button layouts
- `ros2_ws/src/dog_bringup/config/imu.yaml`, `power.yaml`: sensor params
- `docker-compose.yml`, `docker/Dockerfile`, `docker/Dockerfile.sim`

**Core Logic:**

- `ros2_ws/src/dog_control/src/locomotion.cpp`, `gait.cpp`, `crawl.cpp`, `kinematics.cpp`
- `ros2_ws/src/dog_hardware/src/servo_driver.cpp`
- `ros2_ws/src/dog_perception/src/core.cpp`, `localization.cpp`, `submaps.cpp`
- `ros2_ws/src/dog_description/dog_description/urdf.py`

**Testing:**

- C++ gtest: `ros2_ws/src/<pkg>/test/test_*.cpp` (registered in that package's `CMakeLists.txt`)
- Python pytest: `ros2_ws/src/<pkg>/test/test_*.py`, `tools/autocal/tests/`, `tools/robot_setup/test/`
- Integration: `ros2_ws/src/dog_bringup/test/test_mock_bringup.py`, `ros2_ws/src/dog_teleop/test/test_gamepad_fifo.py`
- Physics checks (not colcon tests): `ros2_ws/src/dog_gazebo/dog_gazebo/*_check.py`, `terrain_sweep.py`
- CI: `.github/workflows/ci.yml`

## Naming Conventions

**Files:**

- ROS packages: `dog_<area>` snake_case (`dog_control`).
- C++ sources/headers: lowercase snake_case, header in `include/<pkg>/<name>.hpp` paired with `src/<name>.cpp` (`gait.hpp`/`gait.cpp`).
- Node executables: `<name>_node` (`locomotion_node`, `servo_driver_node`); teleop keyboard is `keyboard_teleop`.
- Tests: `test_<unit>.cpp` / `test_<unit>.py`.
- Python modules: snake_case; sim checks end in `_check.py`; launch files `<name>.launch.py`.
- Docs: UPPERCASE.md in `docs/` (Russian text).

**Directories:**

- Python package dir repeats the package name (`dog_web/dog_web/`) with `resource/<pkg>` marker file.
- C++ headers under `include/<package_name>/`.

**Code identifiers:** C++ classes `PascalCase` (`LocomotionController`), methods `camelCase`, members with trailing underscore (`joint_pub_`), constants `kName` (`kNumLegs`); ROS params dotted lowercase (`gait.step_height`, `odom.publish`); joint names `<leg>_<type>_joint` with legs `lf/rf/lr/rr` and types `hip/thigh/calf` (`lf_thigh_joint`).

## Where to Add New Code

**New motion feature (gait, scripted sequence):**

- Header `ros2_ws/src/dog_control/include/dog_control/<name>.hpp`, source `ros2_ws/src/dog_control/src/<name>.cpp`.
- Add source to `dog_control_core` in `ros2_ws/src/dog_control/CMakeLists.txt`; add `<name>` to the `foreach(t kinematics gait crawl greet locomotion)` test list with `test/test_<name>.cpp`.
- Wire into `LocomotionController` (`locomotion.hpp/.cpp`) and parameters into `locomotion_node.cpp` + defaults in `ros2_ws/src/dog_bringup/config/robot.yaml`.

**New hardware driver / sensor:**

- Core class in `ros2_ws/src/dog_hardware/include/dog_hardware/` + `src/`, node in `src/<name>_node.cpp`; register in `ros2_ws/src/dog_hardware/CMakeLists.txt`; config in `ros2_ws/src/dog_bringup/config/<name>.yaml`; launch toggle in `ros2_ws/src/dog_bringup/launch/robot.launch.py`. Follow `backend: auto|mock|off` pattern.

**New operator input:**

- Mapping logic in `ros2_ws/src/dog_teleop/src/mapping.cpp`, node in `src/`, publish through `TeleopPublisher`; layout params in `teleop*.yaml`.

**New web/WebSocket message:**

- Validate in `ros2_ws/src/dog_web/dog_web/protocol.py` (ROS-free, unit tested in `test/test_protocol.py`), handle in `web_teleop.py`, UI in `ros2_ws/src/dog_web/static/app.js`.

**New perception feature:**

- C++ in `ros2_ws/src/dog_perception/src/core.cpp` (+ header), mirror in `dog_perception/core.py` if numpy-checked; tests in `test/`.

**New simulation world/terrain:**

- Extend `obstacles()`/`world()` in `ros2_ws/src/dog_gazebo/dog_gazebo/terrain.py`; expose via `terrain:=` in `sim.launch.py`; add a check in `dog_gazebo/*_check.py` and register the console script in `ros2_ws/src/dog_gazebo/setup.py`.

**New robot parameter:**

- Add to `ros2_ws/src/dog_bringup/config/robot.yaml`, declare in the node (`declare_parameter`), and if it affects the model, in `urdf.py`.

**New package:**

- Under `ros2_ws/src/dog_<name>/`; mirror the C++ (`CMakeLists.txt` core lib + node) or Python (`setup.py`, `resource/`) layout; add to docs table in `README.md`. Do not place under `robot_dog_ws/` or `legacy/`.

**Utilities / offline tools:**

- `tools/<tool>/` with its own `README.md`, `tests/`, and a CI job in `.github/workflows/ci.yml`.

**Docs:**

- `docs/<TOPIC>.md`, linked from the `README.md` table.

## Special Directories

**`ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`:**

- Purpose: colcon output. Generated: Yes. Committed: No (`.gitignore`).

**`legacy/v1/`:**

- Purpose: v1 code (Python/C++/Rust packages, `servo_config.json`); the whole `legacy/` tree is excluded from colcon by `legacy/COLCON_IGNORE` (9 v1 packages also carry their own). CI runs colcon from `ros2_ws/`, so it never scans `legacy/`. Generated: No. Committed: Yes.

**`robot_dog_ws/`:**

- Purpose: Untracked, partial v1-style JSON-config code (`servo_config_reader.*`, `configuration.html`). Not built by CI or Docker. Committed: No.

**`report/`:**

- Purpose: Generated HTML reports, posters, videos produced by `tools/sim_video/report/build_*.py` from `tools/sim_video/report/data/*.json`. Generated: Yes. Committed: Yes.

**`tools/sim_video/report/data/`:**

- Purpose: Recorded run data (walk, perception, gaits, localization JSON) feeding reports. Generated: Yes. Committed: Yes.

**`__pycache__/` (root), `.pytest_cache/`:**

- Generated, ignored by `.gitignore`.

**`ros2_ws/src/dog_bringup/config/*.bak`:**

- Purpose: backups made by `tools/robot_setup/robot_setup.py`. Ignored by git.

---

*Structure analysis: 2026-09-29*
