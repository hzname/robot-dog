---
last_mapped_commit: d269bd16b0d0f69f3ab623b9cecc4ba545b70517
last_mapped_at: 2026-09-29
---
# Codebase Concerns

**Analysis Date:** 2026-09-29

Scope: full repo `/home/sg/robot-dog` (ROS 2 workspace `ros2_ws/`, tools in `tools/`, docs in `docs/`, generated report in `report/`, previous version in `legacy/v1/`, and untracked root-level leftovers). `.planning/` is excluded. No `TODO` / `FIXME` / `HACK` / `XXX` markers exist in `ros2_ws/`, `tools/`, `docker/` or `.github/`; the known open issues are tracked as prose in `docs/REVIEW.md` (items 12-25) and in the "Ограничения" sections of `docs/LOCALIZATION.md` and `docs/PERCEPTION.md`. Findings below are cross-checked against the code.

## Tech Debt

**Real-robot path has never run; everything is validated in Gazebo only:**

- Issue: `README.md` (table "Состояние") marks "Запуск на реальном роботе" as pending (calibration + measurements). All guard thresholds, gait parameters, localization gains and perception limits were tuned against simulated, noiseless sensors (`docs/REVIEW.md` items 15, 21).
- Files: `ros2_ws/src/dog_bringup/config/robot.yaml`, `ros2_ws/src/dog_bringup/config/servos.yaml`, `ros2_ws/src/dog_perception/src/core.cpp`, `ros2_ws/src/dog_perception/src/perception_node.cpp`
- Impact: first power-up on hardware can move servos to wrong angles (defaults are "estimated from v1", see header of `ros2_ws/src/dog_bringup/config/servos.yaml`); guard thresholds may produce false stops or missed obstacles on real floors.
- Fix approach: follow `docs/DEPLOYMENT.md` stages; start with `guard_slow_vx 0.06`, `guard_stop_dist 0.35`; record real noise with `tools/robot_setup/robot_setup.py` and set thresholds from p99 noise.

**No drivers for the perception sensors on the real robot:**

- Issue: `dog_perception` consumes `lidar_left/scan`, `lidar_right/scan`, `gs2/scan`, `tof/<name>`. In the repo these topics are produced only by Gazebo (`ros2_ws/src/dog_gazebo/dog_gazebo/tof_bridge.py`, `sim.launch.py`). `ros2_ws/src/dog_hardware/` contains only PCA9685, MPU6050 (`imu_sensor.cpp`) and INA226/INA219 (`power_sensor.cpp`). `docs/DEPLOYMENT.md` (stage 14) points to external packages (`ldlidar_stl_ros2`, `EaiRosForGS2`) and no VL53L1X driver exists anywhere. `robot.launch.py` starts `perception_node` with `perception:=true` but nothing feeds it.
- Files: `ros2_ws/src/dog_hardware/src/`, `ros2_ws/src/dog_bringup/launch/robot.launch.py`, `ros2_ws/src/dog_perception/src/perception_node.cpp`
- Impact: obstacle guard, height map and localization cannot work on hardware; the `docker/Dockerfile` image (ros-base only) has none of the external drivers.
- Fix approach: write `vl53l1x_node.cpp` in `ros2_ws/src/dog_hardware/src/` (same `ServoBus`-style I2C wrapper pattern), add the lidar/GS2 drivers to `docker/Dockerfile` or as separate compose services.

**Python twin of the perception core duplicates the C++ core:**

- Issue: `ros2_ws/src/dog_perception/dog_perception/core.py` (530 lines, numpy) re-implements `ros2_ws/src/dog_perception/src/core.cpp` (788 lines). Both are tested with the same scenarios (`test/test_core.py`, `test/test_core.cpp`), but the C++ localization/submaps have no Python twin, so the duplicated part is only the hazard/plane logic.
- Files: `ros2_ws/src/dog_perception/dog_perception/core.py`, `ros2_ws/src/dog_perception/src/core.cpp`, `ros2_ws/src/dog_gazebo/dog_gazebo/perception_check.py`, `tools/sim_video/perception_video.py`
- Impact: threshold changes must be made twice; the offline checks and videos evaluate the Python version, not the code that runs on the robot.
- Fix approach: add a thin pybind/`ros2 bag` replay path that runs the C++ node in `perception_check`, then delete `core.py`; until then change both files in one commit and keep test scenarios identical.

**Same value defined in several places:**

- Issue: control limits appear in `ros2_ws/src/dog_bringup/config/teleop.yaml` (`max_vx`, `max_vy`, ...), `limits.*` in `ros2_ws/src/dog_bringup/config/robot.yaml`, and as `Limits` dataclass defaults in `ros2_ws/src/dog_web/dog_web/protocol.py`. Default joint calibration exists in `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp` (`defaultCalibration`) and in `servos.yaml`. The comment in `teleop.yaml` ("should not exceed limits.* in robot.yaml") is the only guard.
- Impact: silent drift between what the UI offers and what `locomotion` clamps.
- Fix approach: have `web_teleop` read `limits.*` from `robot.yaml`, or add a startup assertion in `ros2_ws/src/dog_web/dog_web/web_teleop.py`.

**Untracked stray v1-style files at repo root:**

- Issue: `robot_configurator.py` (expects `servo_config.json` next to it, which only exists as `legacy/v1/servo_config.json`), `robot_dog_ws/` (ServoConfigReader in Python and C++, a 948-line `templates/configuration.html`, hardcoded `/home/sg/robot-dog/servo_config.json` and `calibration_dir` default `/home/sg/robot-dog` in `robot_dog_ws/src/dog_hardware/dog_hardware/servo_config_reader.py` and `robot_dog_ws/src/dog_hardware_cpp/src/servo_config_reader.cpp`) and `test_servo_config_reader.sh` (hardcoded ssh target on a private LAN host and `~/robot_dog_ws` path) are untracked. They belong to the v1 architecture (`dog_hardware_cpp`), not to `ros2_ws/`, and are not built by CI or Docker.
- Files: `robot_configurator.py`, `robot_dog_ws/`, `test_servo_config_reader.sh`
- Impact: second, incompatible configuration system (`servo_config.json`) next to `servos.yaml`/`robot.yaml`; confusing for contributors; absolute user paths and a LAN IP would leak if committed.
- Fix approach: delete or move under `legacy/`; if any idea is wanted, port it to `tools/robot_setup/`.

**Committed legacy v1 tree with binaries and third-party script:**

- Issue: `legacy/v1/` (229 tracked files, 2.1 MB) includes compiled ELF test binaries (`legacy/v1/robot_dog_ws/scripts/pca9685_test`, `test_aggressive`, `test_pca9685_cpp_fixed`, `test_raw_i2c`, `walk_final`, `walk_gait_cpp`), `legacy/v1/robot_dog_ws/build.log`, `legacy/v1/robot_dog_ws/src/dog_hardware_cpp/src/pca9685_driver.cpp.backup`, a vendored `legacy/v1/get-docker.sh`, and a file literally named `legacy/v1/To`. `legacy/COLCON_IGNORE` keeps colcon away.
- Impact: repo noise, unreviewable binaries, risk of running an old walking binary on the new wiring. `README.md` says the v1 code "не собирается".
- Fix approach: keep only the `.md` hardware notes and `legacy/v1/servo_config.json` data that `docs/HARDWARE.md` cites; drop binaries and logs, or move the tree to a `legacy-v1` git tag.

**Large binary media in git:**

- Issue: `report/` is 67 MB (65 MB in 67 `report/videos/*.mp4`, 1.6 MB posters); `.git` is 71 MB. Regenerating a video via `tools/sim_video/` adds a new blob to history each time.
- Files: `report/videos/`, `report/posters/`, `tools/sim_video/report/`
- Impact: slow clones on the Banana Pi (the README tells users to `git clone` on the robot), growing repo size.
- Fix approach: publish `report/` via GitHub Pages / release assets or Git LFS; on the robot use `git clone --depth 1` or a sparse checkout excluding `report/` and `legacy/`.

**Documentation drift:**

- Issue: `README.md` "Структура" lists `dog_perception/` as "(Python)" although the node is C++ (`docs/REVIEW.md` item 4), omits `dog_bringup` details, and states "109 тестов" while `docs/REVIEW.md` reports 128. `ros2_ws/src/dog_gazebo/setup.py` and other `setup.py` files use `maintainer_email='hzname@example.com'`.
- Fix approach: update `README.md`; derive test counts from `colcon test-result` in CI output instead of prose.

## Known Bugs

**Backward walking is unreliable on uneven ground and the CI thresholds hide it:**

- Symptoms: backward gait creeps or stalls (0-36 % of commanded distance run to run on waves/stones, `docs/TERRAIN.md`); the knee points backwards so feet catch on bumps.
- Files: `ros2_ws/src/dog_control/src/gait.cpp`, `ros2_ws/src/dog_control/src/crawl.cpp`, `.github/workflows/ci.yml` (steps `walk_check --backward-ratio 0.2`, `terrain_sweep ... --backward-ratio 0`)
- Trigger: any backward command on waves > 10 mm or stones > 20 mm.
- Workaround: `--backward-ratio 0` in CI (checks only no-fall/tilt/sag). Avoid backward commands off flat ground.

**Stair descent fails in simulation:**

- Symptoms: on 3 steps of 50 mm the robot climbs but on descent support feet "creep" to the edge and it falls (`docs/REVIEW.md` item 20).
- Files: `ros2_ws/src/dog_control/src/crawl.cpp`, `ros2_ws/src/dog_control/include/dog_control/crawl.hpp`
- Trigger: crawl gait down an edge > ~25 mm with heading correction.
- Workaround: none; needs better foot position knowledge (contact sensing or servo current).

**Localization on large/symmetric maps:**

- Symptoms: loop closure verified on one ring only; map bends 15-25 cm on the far side; matching occasionally holds the robot back in a bare corridor; near-symmetric house is disambiguated only by furniture (`docs/LOCALIZATION.md`, "Ограничения").
- Files: `ros2_ws/src/dog_perception/src/localization.cpp`, `ros2_ws/src/dog_perception/src/submaps.cpp`, `ros2_ws/src/dog_perception/src/localization_node.cpp`
- Trigger: mapping a house-sized area; relocalization with a poor start cloud.
- Workaround: map a single room; do a survey (`survey` command) before relocalizing.

## Security Considerations

**GitHub personal access token embedded in the git `origin` remote URL:**

- Risk: the `origin` remote URL contains a GitHub personal access token in clear text (reported by the orchestrator; not opened or copied here). Anything that reads `.git/config` (backups, `docker build` contexts that include `.git`, screenshots, `git remote -v` output pasted into issues/CI logs) leaks push access to `hzname/robot-dog` and possibly other repos.
- Files: `.git/config` (do not open or quote; token type: GitHub PAT)
- Current mitigation: none detected.
- Recommendations: revoke/rotate the token in GitHub now; switch the remote to SSH (`git@github.com:hzname/robot-dog.git`) with a deploy key, or to HTTPS with a credential helper (`git config credential.helper store` outside the repo, or `gh auth login`); check that no copy of the URL exists in shell history or in the Pi's `~/robot-dog/.git/config` (README tells users to clone there).

**Web control has no authentication and binds to all interfaces:**

- Risk: `web_teleop` listens on `0.0.0.0:8080` (`ros2_ws/src/dog_web/dog_web/web_teleop.py`, `ros2_ws/src/dog_bringup/config/teleop.yaml` `host: 0.0.0.0`). Any host on the LAN can drive the robot, engage or release E-STOP (`{"type":"estop","active":false}`), and use the calibration channel (`cal_pose`, `cal_set`) to write arbitrary servo parameters while the robot is passive. `allow_calibration` defaults to `True`. `ros2_ws/src/dog_web/dog_web/wsserver.py` does not check the `Origin` header, so a web page opened in any browser on the same network can open `ws://<robot>:8080/ws` (cross-site WebSocket hijacking).
- Files: `ros2_ws/src/dog_web/dog_web/web_teleop.py`, `ros2_ws/src/dog_web/dog_web/wsserver.py`, `ros2_ws/src/dog_web/dog_web/calibration.py`, `ros2_ws/src/dog_bringup/config/teleop.yaml`
- Current mitigation: message size cap (`MAX_MESSAGE = 64 KiB`), 10 s request-read timeout, drive watchdog (0.4 s) and a `passive`-only gate for calibration.
- Recommendations: default `allow_calibration` to `False` and enable it only via the launch argument used by `tools/autocal`; verify `Origin` against the request `Host` in `Server._handle`; add a shared-secret token (query parameter checked at upgrade) or bind to a specific interface; only let the client that holds the drive lock release E-STOP; add a connection cap and idle timeout to `wsserver.py` (a slow client can hold sockets open indefinitely once upgraded).

**Unauthenticated ROS 2 graph on the host network:**

- Risk: `docker-compose.yml` uses `network_mode: host` and `ROS_DOMAIN_ID=0`, with no SROS2. Anyone on the LAN can publish `/dog/estop`, `/dog/joint_commands`, or `ros2 param set /dog/servo_driver ...` directly.
- Files: `docker-compose.yml`, `docker/entrypoint.sh`
- Current mitigation: none; joint slew limit (`max_joint_speed`) and per-servo range clamps in `ros2_ws/src/dog_hardware/src/servo_driver.cpp` bound the damage.
- Recommendations: put the robot on a dedicated network/VLAN, use a non-default `ROS_DOMAIN_ID`, consider `ROS_LOCALHOST_ONLY`/`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` with only the web page exposed.

**Container runs as root with broad device access:**

- Risk: `docker/Dockerfile` has no `USER`; `docker-compose.yml` mounts the whole `/dev/input` and adds a cgroup rule for char major 13.
- Files: `docker/Dockerfile`, `docker-compose.yml`
- Recommendations: create a non-root user in the group `i2c`/`input`, mount only `/dev/input/js0` plus a udev rule for re-plug.

**Config directory bind-mounted over the installed package:**

- Risk: `./ros2_ws/src/dog_bringup/config:/ws/install/dog_bringup/share/dog_bringup/config:ro` means `git pull` on the Pi changes live robot behaviour on the next restart with no review step (calibration, limits).
- Files: `docker-compose.yml`
- Recommendations: keep robot-specific calibration in a separate untracked directory (`/etc/robot-dog/`), leaving the repo config as defaults.

## Safety / Reliability (hardware)

**I2C write errors are ignored in the servo path:**

- Problem: `ServoDriver::write` discards the result of `bus_->setPulseUs(...)`, and `Pca9685Bus::setPulseUs` returns false on a failed `::write` with no logging or retry. A bus fault (see the shared-bus load below) makes commanded and physical positions diverge while `joint_states` still reports the commands.
- Files: `ros2_ws/src/dog_hardware/src/servo_driver.cpp` (line ~262), `ros2_ws/src/dog_hardware/src/servo_bus.cpp`
- Fix approach: count failed writes, publish a diagnostic, and after N consecutive failures call `setEstop(true)`.

**No hardware watchdog for the PWM outputs:**

- Problem: the PCA9685 keeps its last pulses if `servo_driver_node` is killed with SIGKILL or the container dies (`relax_on_exit` runs only from the destructor). On command timeout the driver only logs "holding position" (`ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`, `tick()`), it does not relax.
- Fix approach: relax the servos after a longer timeout (e.g. 2-5 s without `joint_commands`), and add an OE-pin cutoff relay or an MCU watchdog (`docs/COMPUTE.md` discusses a microcontroller).

**`joint_states` on the robot are commands, not measurements:**

- Problem: MG996R servos have no feedback (`docs/REVIEW.md` item 13). The leg-plane reference for perception and slope compensation trusts commanded angles; backlash/sag gives 5-10 mm at the feet.
- Files: `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`, `ros2_ws/src/dog_perception/src/perception_node.cpp`
- Fix approach: keep `reference: auto` (lidar plane primary); add tilt cross-check < 2° (item 18); long term use servos with feedback.

**E-STOP and mode are software-only:**

- Problem: E-STOP is a ROS topic (`estop`, RELIABLE, not latched at the driver); if `servo_driver` restarts (Docker `restart: unless-stopped`) it starts un-estopped. There is no physical kill switch documented in the launch chain.
- Files: `ros2_ws/src/dog_hardware/src/servo_driver.cpp`, `ros2_ws/src/dog_control/src/locomotion_node.cpp`, `docker-compose.yml`
- Fix approach: latch estop with TRANSIENT_LOCAL QoS on the publisher side and on the driver subscription; add a power-rail cutoff.

**Single I2C bus carries everything at unspecified speed:**

- Problem: PCA9685 (100 Hz updates on 12 channels), MPU6050 (100 Hz), INA226, and up to 4 VL53L1X on `/dev/i2c-0`; bus speed is not set anywhere (`docs/REVIEW.md` item 12). Each `setPulseUs` is a separate 5-byte transaction (`Pca9685Bus::writeChannel`), no batch write.
- Files: `ros2_ws/src/dog_hardware/src/servo_bus.cpp`, `ros2_ws/src/dog_hardware/src/imu_sensor.cpp`, `ros2_ws/src/dog_hardware/src/power_sensor.cpp`, `docker-compose.yml` (only `/dev/i2c-0` mapped)
- Fix approach: 400 kHz overlay, use the PCA9685 auto-increment to write all 12 channels in one transaction, ToF at 30 Hz or on a second bus.

**Battery protection is an on-rail heuristic only:**

- Problem: `ros2_ws/src/dog_bringup/config/power.yaml` sets `undervoltage_v: 5.0` on the 6 V BEC rail; the sensor is optional ("if no sensor answers, power monitoring is simply off"), so a missing INA226 silently disables overcurrent E-STOP and undervoltage "lie".
- Files: `ros2_ws/src/dog_bringup/config/power.yaml`, `ros2_ws/src/dog_hardware/src/power_monitor_node.cpp`
- Fix approach: log an ERROR (or refuse to stand) when `backend: auto` finds no sensor on the real robot; monitor the LiPo pack voltage, not only the BEC output.

## Performance Bottlenecks

**Perception and localization CPU load on the Cortex-A53:**

- Problem: perception node is estimated at 0.3-0.4 core on the Banana Pi (`docs/PERCEPTION.md`), `localization` global search brute-forces the whole map ("на A53 в разы медленнее", `docs/LOCALIZATION.md`), `web_teleop` (Python) uses ~8 % of a PC core, more than `locomotion` (`docs/REVIEW.md` item 23).
- Files: `ros2_ws/src/dog_perception/src/localization.cpp` (`WallGrid::updateField`, `match`), `ros2_ws/src/dog_perception/src/localization_node.cpp`, `ros2_ws/src/dog_web/dog_web/web_teleop.py`, `ros2_ws/src/dog_perception/test/test_submaps.cpp` (timeout raised to 300 s)
- Cause: exhaustive search over a 10 cm grid, no coarse-to-fine; unthrottled 100 ms status streaming in the calibration channel.
- Improvement path: coarse-to-fine global search, cap map size (`kMaxCells`), profile `web_teleop` and cache static files (`Server._handle` reads the whole file on every request).

**Lidar scan timing not compensated:**

- Problem: real lidars take 100 ms per revolution; the code assumes an instant scan (`docs/REVIEW.md` item 14, `docs/LOCALIZATION.md`). Trot pitching at ~10°/s gives ~1° plane error per revolution.
- Files: `ros2_ws/src/dog_perception/src/perception_node.cpp`, `ros2_ws/src/dog_perception/src/core.cpp`
- Improvement path: de-skew each point by `time_increment` using the IMU history already kept.

## Fragile Areas

**Guard (hazard reaction) memory and thresholds:**

- Files: `ros2_ws/src/dog_perception/src/core.cpp`, `ros2_ws/src/dog_perception/src/perception_node.cpp`, `ros2_ws/src/dog_perception/dog_perception/core.py`
- Why fragile: behaviour depends on tuned constants (18 mm lidar edge jump, 20 mm GS2 plane threshold, 8-scan confirmation, 5 cm cell memory, 30 mm max step-over) found by trial in simulation; an earlier fix showed that a confirmed "stop" could be evicted by message flood (`docs/REVIEW.md` item 11). ~50 tunables live in `robot.yaml` under `perception:` with weak relationships between them.
- Safe modification: change one threshold at a time and re-run the whole set of `perception_check` scenarios from `.github/workflows/ci.yml` (flat, wall 80, steps 30, greet, survey) in C++ and Python; add a test in both `test/test_core.cpp` and `test/test_core.py`.
- Test coverage: node-level wiring (`perception_node.cpp`, 815 lines) is exercised only by Gazebo runs; no unit test drives the node with recorded scans.

**Gazebo-dependent CI steps are timing-sensitive:**

- Files: `.github/workflows/ci.yml` (jobs `simulation`, `terrain`)
- Why fragile: each perception step launches Gazebo in the background (`&`), then `kill %1; sleep 3; pkill -f "gz sim"`; separate `ROS_DOMAIN_ID`/`GZ_PARTITION` per step exist because a leftover simulation mixed odometry into the next run. Comments record a flake at 39 % vs a 40 % backward threshold. `terrain` job runs 10 sequential sim steps with 6-16 min timeouts on shared runners; `terrain` uses only jazzy while `simulation` uses jazzy and lyrical.
- Safe modification: keep unique domain/partition per step; do not tighten ratios; add retries or run the sim as a service container.
- Test coverage: `dog_gazebo` has no colcon tests (`colcon test --packages-skip dog_gazebo`); its scripts (`walk_check.py`, `terrain_sweep.py`, `perception_check.py`, `localization_check.py`) are checked only via these CI steps and use wall-clock `time.time()` loops with 60 s waits (`ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py`).

**Servo calibration model with rod linkage and coupling:**

- Files: `ros2_ws/src/dog_hardware/src/servo_driver.cpp` (`jointToPulseUs`, `pulseUsToJoint`, `parentAngle`), `ros2_ws/src/dog_hardware/include/dog_hardware/servo_driver.hpp`, `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp` (`onParams`)
- Why fragile: coupled servos (calf follows thigh) are written only when the parent changed; live `ros2 param set` replaces a whole `ServoCalibration` and immediately writes the pulse to a powered servo ("live preview"), so a wrong value moves a real leg instantly. Parameter validation is per-field in `onParams`, with a long `if/else if` chain that must be extended for each new field (and again in `ros2_ws/src/dog_web/dog_web/calibration.py` `CAL_FIELDS`).
- Safe modification: add a field in all three places (`ServoCalibration`, `onParams`, `CAL_FIELDS`) and a case in `ros2_ws/src/dog_hardware/test/test_servo_driver.cpp`.

**Hand-rolled WebSocket/HTTP server:**

- Files: `ros2_ws/src/dog_web/dog_web/wsserver.py`, duplicated client framing in `tools/autocal/robotdog_autocal/client.py`
- Why fragile: no fragmentation-size limit across continuation frames beyond `MAX_MESSAGE`, no ping timeout, no UTF-8 strictness (`errors='replace'`), reads static files fully into memory, and no per-connection limit. The client `_send` in `tools/autocal/robotdog_autocal/client.py` supports payloads only up to 65535 bytes.
- Safe modification: keep `tests` in `ros2_ws/src/dog_web/test/test_wsserver.py` passing; consider `websockets`/`aiohttp` if a pip dependency becomes acceptable (currently avoided so that `ros-base` images need no extra packages).

**Map persistence:**

- Files: `ros2_ws/src/dog_perception/src/localization.cpp` (`save`, plain `std::ofstream` of `.pgm`/`.yaml`/`.walls`), `ros2_ws/src/dog_bringup/config/robot.yaml` (`localization.map: "~/.ros/dog_map"`), `docker-compose.yml`
- Why fragile: the map is written non-atomically on shutdown (Docker `stop_grace_period: 10s`), so a crash mid-write or power loss corrupts it; `~/.ros/dog_map` lives inside the container filesystem, which is lost on `docker compose up --build`/recreate; no volume is declared.
- Safe modification: write to a temp name then `rename`; add a `volumes:` entry for a map directory and point `localization.map` at it.

## Scaling Limits

**Memory / grid size on the Pi (2 GB RAM):**

- Current capacity: `Dockerfile` limits `BUILD_JOBS=2` because of the 2 GB board; runtime map size is bounded by `kMaxCells` in `ros2_ws/src/dog_perception/src/localization.cpp`.
- Limit: 5 cm cells over a large house plus submaps and pose graph (`submaps.cpp`) grow with area walked; global search cost grows with map area.
- Scaling path: run mapping on a laptop (suggested in `docs/LOCALIZATION.md`, "Тяжёлое"), ship only the saved map to the robot.

## Dependencies at Risk

**Python packages assumed present in the runtime image:**

- Risk: `dog_perception/dog_perception/core.py` needs `numpy` and `yaml` (test dependencies only per `ros2_ws/src/dog_perception/package.xml`), while `ament_python_install_package` installs it into the robot image where `ros-base` has neither guaranteed. `tools/autocal/requirements.txt` needs `opencv-contrib-python-headless` (ArUco API changes between OpenCV versions).
- Impact: importing `dog_perception` on the robot fails; autocal breaks on OpenCV major bumps.
- Migration plan: mark the Python twin as test/offline-only (do not install), pin OpenCV in `tools/autocal/requirements.txt`.

**Two ROS distros supported at once (Jazzy and Lyrical):**

- Risk: `lyrical` images (`ros:lyrical-ros-base`, `osrf/ros:lyrical-simulation` = Gazebo Jetty) are matrixed in `.github/workflows/ci.yml`; API differences may surface (e.g. `TARGETS` variables, `rclcpp` QoS).
- Impact: CI duplicates cost; production uses only Jazzy (`docker-compose.yml`).
- Migration plan: keep Lyrical as `fail-fast: false` (already set); drop it if maintenance grows.

## Missing Critical Features

**Tilt/fall protection on the real robot:**

- Problem: IMU is used for slope compensation and heading hold (`ros2_ws/src/dog_control/src/locomotion_node.cpp`), but no fall/tip-over detection (tilt threshold -> lie/relax) was found in `dog_control`; only the Gazebo checks assert tilt < 20°.
- Blocks: safe untethered operation.

**Sensor bring-up and health diagnostics:**

- Problem: no `diagnostic_msgs` publication; silent fallbacks (power sensor off, guard silent for 1 s drops hazard limits, `ros2_ws/src/dog_control/src/locomotion_node.cpp` `guard_timeout`) look identical to "all clear" for the operator on the web page.
- Blocks: knowing whether obstacle protection is really active.

**Odometry drift (no correction other than localization):**

- Problem: dead reckoning drifts with slip 0-40 % and heading from the IMU (`ros2_ws/src/dog_control/include/dog_control/odometry.hpp`); the guard's height map is in the odom frame and floats after metres (`docs/REVIEW.md` items 17, 24).
- Blocks: reliable height-map based foot placement beyond a few metres.

## Test Coverage Gaps

**ROS node wiring (executables) with no direct unit tests:**

- What's not tested: `ros2_ws/src/dog_control/src/locomotion_node.cpp`, `ros2_ws/src/dog_perception/src/perception_node.cpp`, `ros2_ws/src/dog_perception/src/localization_node.cpp`, `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp` (parameter callbacks), `ros2_ws/src/dog_teleop/src/joy_teleop_node.cpp`, `ros2_ws/src/dog_teleop/src/keyboard_teleop.cpp`
- Files: covered indirectly by `ros2_ws/src/dog_bringup/test/test_mock_bringup.py` (259 lines) and Gazebo CI
- Risk: timeout/estop/param-set behaviours regress without a failing unit test.
- Priority: High for `servo_driver_node.cpp` `onParams` and `locomotion_node.cpp` timeouts (safety paths).

**Real I2C hardware code (`Pca9685Bus`) untested against a device:**

- What's not tested: `open`, `writeChannel`, `readReg` in `ros2_ws/src/dog_hardware/src/servo_bus.cpp`; tests use `MockBus` (`test/test_servo_driver.cpp`). `ros2_ws/src/dog_hardware/src/pca9685_probe.cpp` is a manual tool.
- Risk: register-level bugs found only on the robot.
- Priority: Medium (add a fake `/dev/i2c` shim or loopback test for prescale/tick math; `prescaleFor`, `ticksFor` are already pure and testable).

**Web front-end and calibration channel:**

- What's not tested: `ros2_ws/src/dog_web/static/app.js` (201 lines, no JS tests); `ros2_ws/src/dog_web/dog_web/calibration.py` (ROS service bridge) has no test in `ros2_ws/src/dog_web/test/` (only `test_protocol.py`, `test_wsserver.py`).
- Risk: regressions in the E-STOP button or calibration flow reach the robot unnoticed.
- Priority: Medium.

**Tools outside colcon:**

- What's not tested: `tools/sim_video/*.py` and `tools/sim_video/report/*.py` (no tests; depend on committed `data/*.json` and raw records not in git); `docs/img/make_diagrams.py`.
- Risk: reports become unreproducible (`docs/REVIEW.md` item 7 already needed a fix for this).
- Priority: Low.

**CI does not cover the real-robot Docker runtime:**

- What's not tested: the `robot-image` job builds `docker/Dockerfile` for arm64 with `-DBUILD_TESTING=OFF` and never starts it; runs only on non-PR events (`if: github.event_name != 'pull_request'`).
- Risk: launch-time failures (missing runtime dependency, `/dev/i2c-0` open) appear on the robot. Priority: Medium (add `ros2 launch dog_bringup robot.launch.py backend:=mock` smoke test inside the built image).

---

*Concerns audit: 2026-09-29*
