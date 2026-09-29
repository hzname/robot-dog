# Feature Research

**Domain:** Servo (hobby-grade, no joint feedback) quadruped, first floor bring-up. Subsequent milestone on an existing ROS 2 codebase.
**Researched:** 2026-09-29
**Confidence:** MEDIUM. Gap analysis against the code is HIGH (read and verified in the source). Ecosystem comparison (Pupper, Spot Micro, Petoi, Unitree) is LOW to MEDIUM: public docs describe features but rarely thresholds, so thresholds below are engineering proposals, not community standards.

## How to read this file

- **Status** is measured against the repo at commit `53e9653`: `Covered` (works in code/sim), `Partial` (exists but has a hole), `Missing`.
- **Complexity:** LOW (under a day, one file + test), MEDIUM (1-3 days, several files or a new message path), HIGH (multi-day or needs hardware).
- IDs (`SAF-`, `CAL-`, `DIA-`, `TEL-`, `GAIT-`) are proposed requirement IDs. The five categories map 1:1 to requirement groups.
- All "verified" claims below were checked in code by this research pass, not only taken from `.planning/codebase/CONCERNS.md`.

## Verification of CONCERNS.md claims (spot-check results)

| Claim in CONCERNS.md | Verdict | Evidence |
|----------------------|---------|----------|
| I2C write result ignored | CONFIRMED, and worse | `ServoDriver::write` drops `bus_->setPulseUs(...)` (`servo_driver.cpp:262`). Also `setEstop()` calls `relax()` -> `bus_->disableAll()` and ignores the result, then sets `estop_ = true` (`servo_driver.cpp:266-277`). A failed bus can leave servos energised while the UI shows E-STOP. |
| No PWM watchdog | CONFIRMED | `servo_driver_node.cpp:201-204`: on `command_timeout` it only logs "holding position" once (`timed_out_`), never relaxes. A dead `locomotion_node` leaves servos holding torque indefinitely. |
| No tip-over reaction | CONFIRMED | `setImuAttitude()` returns early when tilt exceeds `slope.max_deg` (`locomotion.cpp:~221`); nothing else consumes IMU tilt. No fallen state exists. |
| No `diagnostic_msgs` | CONFIRMED | grep over `ros2_ws/src` finds no `diagnostic` usage. |
| E-STOP is volatile, driver restarts un-estopped | CONFIRMED, but deliberate | `docs/CONTROL.md` explains why volatile (late joiners replay stale `estop` values). Do not switch to TRANSIENT_LOCAL as CONCERNS suggests. The real hole is different: after a driver-only restart, incoming `joint_commands` enable servos straight from OFF at full speed (`ServoDriver::setTargets` PENDING path). |
| Live calibration writes instantly | CONFIRMED | `ServoDriver::setCalibration` calls `write(index)` immediately when the servo is ON (`servo_driver.cpp:287-290`). A mistyped offset jumps a powered leg. |
| Power monitor silently off when no sensor | CONFIRMED | `power_monitor_node.cpp:88` logs INFO only. |
| Default `overcurrent_a: 5.0` is a calibrated value | NOT SAFE TO ASSUME | `power.yaml` defaults to 5 A for 0.5 s -> estop, but `docs/HARDWARE.md` says 12 MG996R draw 6-12 A total when walking (2.5 A peak each). Default would likely false-trigger E-STOP on the first walk. Also a 10 mOhm shunt on INA226 saturates near 8.2 A (81.92 mV full scale, from the chip's datasheet range; MEDIUM confidence) - below expected walking peaks. Needs a measured threshold and possibly a 2-5 mOhm shunt. |
| Gamepad and web "fight" via last-wins | PARTLY OVERSTATED | `JoyMapper::process` publishes a twist only while the deadman is held and one zero twist on release (`mapping.cpp:71-84`). An idle gamepad does not overwrite web input. Two *simultaneously active* sources still interleave. |
| `estop` release by any web client | CONFIRMED | `web_teleop.send_actions` publishes `Bool(active)` from any connected client (`web_teleop.py:104`). |
| Gait params tunable at runtime | FALSE | `locomotion_node.cpp:242-245` only `declare_parameter`s `gait.*`; no set-parameters callback. Retuning on the floor requires a container restart. |

## Ecosystem reference (what comparable robots offer)

| Robot | Relevant features seen | Confidence |
|-------|------------------------|------------|
| Stanford Pupper (CLS6336 servos, RPi) | Calibrate on a stand with legs free; stop the running service before calibrating; guided per-servo alignment script; advice to buy spare servos because they break in assembly and use | MEDIUM (docs fetched) |
| Spot Micro (mike4192/spotMicro, PCA9685) | Explicit state machine idle/sit/stand/angle/walk; `idle` frees servos before shutdown; keyboard servo-move node for calibration; separate PCA9685 frequency calibration; README states the software has NO command limits and can damage hardware | MEDIUM (README fetched) |
| Petoi Bittle | Gyro-based self-righting, slope tolerance around 10 deg, calibration skill posture | LOW (search snippets) |
| Unitree Go1 | "Protected mode": fall protection, over-temperature, low-voltage, e-stop; protective frame for testing | LOW (search snippets) |
| Hobby PCA9685 rigs generally | Servo rail sag (6 V -> 3.5 V under stall) glitches I2C and browns out the controller; MG996R stall about 2.5 A at 6 V | MEDIUM (multiple forum/datasheet hits) |

Takeaway: hobby servo quadrupeds generally ship with calibration scripts, an idle/relax state and a stand. The 1st-floor safety layer (fault latch, tip-over reaction, watchdog, health telemetry) is NOT common in hobby projects - this project is already ahead on software E-STOP/deadman and is asked to match professional robots on fault handling. That is why the "Missing" rows below are table stakes for this milestone even though hobby peers skip them.

## Feature Landscape

### Category 1 - Safety and fault handling (`SAF`)

#### Table stakes

| ID | Feature | Status | Why expected | Complexity | Notes and dependencies |
|----|---------|--------|--------------|------------|------------------------|
| SAF-01 | Checked I2C writes: count failures per channel, rate-limited ERROR log, escalate after N consecutive failures (or a failure rate over a window) to a fault latch | Missing | On first power-up I2C glitches from servo rail sag are the most likely fault; today commanded and physical pose silently diverge | LOW-MED | `ServoDriver::write`, `Pca9685Bus::setPulseUs/writeChannel` (`servo_driver.cpp`, `servo_bus.cpp`). `MockBus` needs fault injection (fail next N writes) to unit-test in `test_servo_driver.cpp`. |
| SAF-02 | Verified E-STOP: check `disableAll()` result, retry, fall back to per-channel `disable()`; do not report "estop" as done until output is confirmed off | Missing | E-STOP that can fail silently is not an E-STOP | LOW | `ServoDriver::setEstop/relax`. Same test hook as SAF-01. |
| SAF-03 | Single fault path: any driver-originated relax (I2C fault, watchdog) latches the driver disarmed AND publishes `estop=true` so `locomotion_node` goes PASSIVE and the UI shows E-STOP | Missing | Today the driver can only receive `estop`, never raise it; locomotion would keep sending commands to a dead driver | MEDIUM | New `estop` publisher in `servo_driver_node.cpp` (mind the self-subscription echo). Reuses the existing volatile QoS choice. Clear only by explicit release + `stand`. |
| SAF-04 | Servo output watchdog: no `joint_commands` for T_relax (proposal 2-3 s, after the existing 0.5 s hold) -> relax outputs, latch via SAF-03 | Missing | If `locomotion_node` crashes/hangs mid-stance, servos hold torque forever (heat, stall against a load). The PCA9685 has no internal timeout | LOW | `servo_driver_node.cpp::tick`. Add param `relax_timeout` in `servos.yaml`. Robot sags to belly from 0.15 m: acceptable at this size. |
| SAF-05 | Tip-over / fall reaction from the IMU: warn tier (stop stepping, lower stance) and hard tier (`|roll|` or `|pitch|` above a limit, debounced about 0.2 s -> relax servos, mode `fallen`, publish state, require explicit re-stand) | Missing | Owner-listed gap. Without it a robot on its back keeps driving servos into the floor/against each other. Sim shows normal trot tilt up to about 13-19 deg and crawl pitch 8.6 deg, so a hard limit near 35-45 deg leaves headroom (value to be tuned from `walk_check` logs) | MEDIUM | Consumes `imu_rpy_` in `locomotion_node.cpp:154-162`; new Mode or PASSIVE+flag in `locomotion.cpp` (`setEstop` shows the PASSIVE reset pattern). Add a sim test (steep slope without compensation flips the robot, per `TERRAIN.md`) and unit test in `test_locomotion.cpp`. Must not conflict with `slope.max_deg = 25` which is a filter, not a safety limit. |
| SAF-06 | IMU-loss escalation: if IMU was seen and goes silent (>0.2 s already detected in `locomotion_node.cpp:338`), stop and lie instead of silently walking uncompensated; refuse to `stand` for floor runs when `imu:=auto` found nothing | Missing | SAF-05 is blind without the IMU. `imu_node` exits when no chip is found | LOW | `locomotion_node.cpp`, `imu_node.cpp`. Make "IMU required" a param, default true in the floor profile (see CAL-07). |
| SAF-07 | Power guard that is trustworthy: measured thresholds (not defaults), loud ERROR (or refuse to stand) when the sensor is missing on the real robot, undervoltage on the battery pack not only the BEC output | Partial | Guard exists (`PowerGuard`, `power_monitor_node.cpp`) but thresholds are untested on hardware and the shunt range is likely too small | MEDIUM | `power.yaml`, `power_sensor.cpp`. Hardware confirm of INA226 presence is still open in PROJECT.md. If no sensor: SAF-08 becomes the only protection, so document it. |
| SAF-08 | Idle-to-rest timeout: standing still (no `cmd_vel`) for T_idle (proposal 60-120 s) -> `lie` and relax | Missing | The only thermal protection available for feedback-less MG996R: standing holds torque on knees (about 0.54 Nm per knee per `HARDWARE.md`). Cheap and effective for "no overheating" acceptance | LOW | `locomotion.cpp` STAND mode timer; param in `robot.yaml`. Publish a state so the UI can tell. |
| SAF-09 | Hardware kill in reach: power switch/fuse in the servo rail, accessible without touching the legs; tether or spotter for first floor run | Covered (docs) | `DEPLOYMENT.md` stage 0/2/9 already require switch, fuse, hand-on-E-STOP, spotter | LOW (hardware) | Make it a hard acceptance gate, not advice. Software E-STOP alone cannot stop a hung process (SIGKILL leaves last PCA9685 pulses). |
| SAF-10 | Deadman, cmd timeouts (0.4-0.5 s), E-STOP from any channel, joint speed limit, staggered leg enable, per-joint clamps, boot in PASSIVE, `stand` required after E-STOP | Covered | Existing chain (`CONTROL.md`, `servo_driver.cpp`, `locomotion.cpp`) | - | Keep; add regression tests when SAF-01..04 touch the same files. |

#### Differentiators

| ID | Feature | Value | Complexity | Notes |
|----|---------|-------|------------|-------|
| SAF-11 | Hardware output-enable cutoff: PCA9685 OE pin driven from a GPIO/MCU heartbeat so a hung or killed process de-energises PWM in hardware | Closes the SIGKILL/container-death hole that SAF-04 cannot | HIGH | Needs wiring and a tiny supervisor (`docs/COMPUTE.md` already discusses an MCU). v2. |
| SAF-12 | Driver "arm" gate: after any driver start or fault the driver stays disarmed until an explicit release, so joint commands from a still-walking `locomotion_node` cannot yank limp legs to the walking pose | Removes the driver-only-restart jump | MEDIUM | Overlaps with SAF-03; can share the latch. Consider merging into v1 if SAF-03 is built as a latch. |
| SAF-13 | Fall recovery / self-righting | Petoi-style convenience | HIGH | Servo torque and knee geometry (`knee_direction: -1`) make it a research topic. Not needed for "first walk". |
| SAF-14 | Thermal estimate from INA226 current x time, per-session budget | Better than a fixed idle timeout | MEDIUM | Only total rail current is available (no per-leg sensing), so the estimate is coarse. Do after real data exists. |
| SAF-15 | Two-tier power response: warn, then `lie`, then E-STOP, with hysteresis | Avoids nuisance E-STOP dropping the robot from stance | LOW-MED | `power_monitor_node.cpp` already has `estop | lie | warn` actions per event; add tiers. |

#### Anti-features

| Anti-Feature | Why avoid | Alternative |
|--------------|-----------|-------------|
| Per-servo temperature/feedback sensing now | MG996R has no feedback; adding it means replacing servos (STS3215-class) - a different milestone | SAF-08 idle timeout, IR thermometer checks in `DEPLOYMENT.md` stage 8/12 |
| Latching `estop` with TRANSIENT_LOCAL (CONCERNS suggestion) | `CONTROL.md` documents why it was rejected (stale replay releases E-STOP) | SAF-03/SAF-12 latch inside the driver |
| Automatic fall recovery before basic stand/walk works | Complex and can damage geartrains | SAF-05 relax and manual re-stand |
| SROS2 / DDS security | High effort, breaks the `ros-base` image goal, no benefit against physical safety | Dedicated network, non-default `ROS_DOMAIN_ID` (docs), see TEL-04 |
| Relying on software E-STOP as the only kill | See SAF-09 | Physical switch |

### Category 2 - Servo calibration and bring-up procedure (`CAL`)

#### Table stakes

| ID | Feature | Status | Why expected | Complexity | Notes and dependencies |
|----|---------|--------|--------------|------------|------------------------|
| CAL-01 | Per-servo `direction`, `offset_deg`, pulse range, limits, rod/coupling model, live tuning via `ros2 param set`, `calib_pose` tool | Covered (sim/bench-untested) | Every peer robot has this (Pupper `calibrate_servos.py`, Spot Micro keyboard node) | - | `servos.yaml`, `servo_driver_node.cpp::onParams`, `dog_bringup/scripts/calib_pose`, `docs/CALIBRATION.md`. Defaults are "estimated from v1" and MUST be treated as unverified. |
| CAL-02 | Wiring and channel bring-up tool: `pca9685_probe check | pulse | off` | Covered | Pupper/Spot Micro have equivalents | - | `pca9685_probe.cpp`. |
| CAL-03 | Staged procedure with a go/no-go gate per stage (bench supply with current limit, servos without horns, horns fitted at 1370 us, stand, tether, floor) | Covered (docs) | Standard practice; Pupper insists on a stand | - | `DEPLOYMENT.md` stages 5-9. The milestone should execute it and record results, not rewrite it. |
| CAL-04 | Live-calibration step guard: reject or ramp a `set` that moves the servo by more than about 10 deg equivalent while powered | Missing | On a real robot a typo (`offset_deg 4.75` vs `47.5`) slams a leg into the body | LOW | `ServoDriver::setCalibration` and `onParams`. Also applies to the web `cal_set` bridge (`dog_web/calibration.py`), which only checks mode `passive/unknown`. |
| CAL-05 | Calibration persistence that survives the container: results saved to a robot-specific file, not only `ros2 param dump` -> hand copy; keep repo defaults separate | Partial | `docker-compose.yml` bind-mounts `config/` over the installed package, so `git pull` silently changes robot behaviour; calibration is the value you least want overwritten | LOW-MED | `tools/autocal --apply --servos-yaml`, `docker-compose.yml`, `servos.yaml`. Minimum: commit calibrated `servos.yaml` on a `robot-calibrated-<date>` tag and record which file the container actually reads. |
| CAL-06 | Calibration sanity check after run: mirrored pairs symmetric, offsets within mechanical spread of the horn spline step (about 7-10 deg), no joint parked at a limit in the stand pose | Missing | Catches horn mounted one spline off or wrong channel before a leg is asked to stand | LOW-MED | Add to `tools/robot_setup/robot_setup.py --check` (already runs in CI on YAML). |
| CAL-07 | "First-floor" limits profile: one launch argument that applies the reduced `limits.*` (e.g. vx 0.05), `heading.hold`/`slope.compensation` policy, required-IMU flag, web calibration off | Missing | `DEPLOYMENT.md` stage 8 says to edit `robot.yaml` and revert later; forgetting to revert or to reduce is a classic failure | LOW | `robot.launch.py` (`OpaqueFunction` args), a second params file layered after `robot.yaml`. |
| CAL-08 | IMU axis and sign verification before compensation is enabled: tilt tests exist as prose (`DEPLOYMENT.md` stage 7); make first floor runs start with `slope.compensation:false` and `heading.hold:false` until the checks pass | Partial | Wrong axis sign turns slope compensation and heading hold into positive feedback (`DEPLOYMENT.md` risk table says the robot spins) | LOW | Config flags exist (`robot.yaml` `slope`, `heading`). Optional scripted self-test is a differentiator (CAL-12). |
| CAL-09 | Arm64 image smoke test and preflight: the built image starts `robot.launch.py backend:=mock`, `docker compose` maps `/dev/i2c-0`, `i2cdetect` sees 0x40 and 0x68, `js0` present, before servos get power | Missing | Goal (3) says "Docker on Banana Pi". CI builds the arm64 image but never runs it (`CONCERNS.md`, verified against `docker/Dockerfile` `CMD` and `ci.yml`) | MEDIUM | `.github/workflows/ci.yml` `robot-image` job, `docker/entrypoint.sh`, `pca9685_probe check`. Runtime stays on `ros-base`. |
| CAL-10 | Session recording: one command that starts a rosbag with the standard topic list for every floor run | Partial | Needed to compare real vs sim (`DEPLOYMENT.md` stage 9-10). Command exists in prose only | LOW | Topics: `joint_commands`, `joint_states`, `servo_pulses`, `imu/data`, `power`, `state`, `cmd_vel`, `diagnostics` (after DIA-01). |
| CAL-11 | Power hardware verification before servos: measured 6.0 V rail, separate 5 V for the Pi, bulk capacitor on V+, common ground, PCA9685 VCC on 3.3 V; I2C at 400 kHz decision | Covered (docs) | Brownout from servo rail sag is the classic hobby failure | LOW (hardware) | `DEPLOYMENT.md` stage 2-3; I2C speed is open (`REVIEW.md` #12). Record the outcome as a checklist result. |

#### Differentiators

| ID | Feature | Value | Complexity | Notes |
|----|---------|-------|------------|-------|
| CAL-12 | Scripted IMU self-test: command body pitch/roll pose in `stand` and assert IMU sign, plus gyro Z sign check | Removes the human-error path in CAL-08 | MEDIUM | `body_pose` topic + `imu/data`. |
| CAL-13 | Guided bring-up wizard: per-channel wiggle with I2C error capture, prompts "did leg X move?", writes a bring-up report | Repeatable, produces evidence for the milestone gate | MEDIUM | Wraps `pca9685_probe` + `calib_pose`. |
| CAL-14 | Automatic full camera calibration (`tools/autocal`) used as primary path, hand calibration as fallback | Already built; needs a first run on hardware to be trusted | LOW (effort) / risk in reality | Results unknown until run; keep hand method in the plan. |
| CAL-15 | Live tuning of `gait.*` and `limits.*` without restart (parameter callback in `locomotion_node`) | Faster iteration on the floor (stage 10 lists many one-at-a-time changes) | LOW-MED | Currently declare-only (verified). |

#### Anti-features

| Anti-Feature | Why avoid | Alternative |
|--------------|-----------|-------------|
| Merging the v1 `servo_config.json` / `robot_configurator.py` / `robot_dog_ws/` into the v2 path | Second configuration system; hard-coded `/home/sg/...` path (`CONCERNS.md`); PROJECT.md marks it out of scope | `servos.yaml` + `robot_setup` |
| Full auto-calibration of dynamics (servo speed/torque identification) now | Needs measurement rigs and time; no feedback sensors | Torque margin check per `HARDWARE.md`, tune from floor logs |
| Building sensor drivers (VL53L1X, lidar, GS2) during bring-up | No hardware, out of scope | Guard stays off on the robot (`perception:=false`) |
| Editing `robot.yaml` geometry to "fix" walking before measurements | Wrong geometry cannot be fixed by calibration (`DEPLOYMENT.md` stage 1) | Do stage 1 measurements first |

### Category 3 - Diagnostics and telemetry (`DIA`)

#### Table stakes

| ID | Feature | Status | Why expected | Complexity | Notes and dependencies |
|----|---------|--------|--------------|------------|------------------------|
| DIA-01 | `diagnostic_msgs/DiagnosticArray` on `diagnostics` from `servo_driver` (armed/relaxed, I2C error counters, clamped joints, last command age), `imu_node` (rate, bias done, stale), `power_monitor` (V, I, sensor present), `locomotion` (mode, tilt, timeout events) | Missing | Silent fallbacks currently look identical to "all clear" (`CONCERNS.md`, confirmed) | MEDIUM | Publish `diagnostic_msgs` directly: it ships with `ros-base` (common_interfaces), while `diagnostic_updater`/aggregator live in the separate `ros/diagnostics` repo and are not in the base image (MEDIUM confidence; check `rosdep`/apt before adding any dependency; the image must stay on `ros-base`). A tiny in-repo helper is enough. |
| DIA-02 | Operator-visible fault banner on the web page and readable from the gamepad session (SSH log): I2C fault, watchdog relax, IMU lost, fallen, battery low, sensor missing | Missing | The operator holding a gamepad next to the robot must know why it went limp | LOW-MED | `web_teleop.py` (already relays `state`, `power`, `perception/guard`), `protocol.py`, `static/app.js`; consumes DIA-01 or `state` extensions. |
| DIA-03 | Rate-limited, greppable fault logging with counters (I2C `write failed`, `clamped`, cmd timeouts, guard silent) | Partial | Some `RCLCPP_WARN` exist (clamp, timeout); I2C errors have none | LOW | `servo_driver_node.cpp`; `DEPLOYMENT.md` stage 12 expects "no `write failed` in the log", which currently cannot appear. |
| DIA-04 | Pi health in the log: CPU load and SoC temperature at 1 Hz (from `/sys/class/thermal`) | Missing | `DEPLOYMENT.md` stages 3/4/10 require < 75 C and < 1.5 cores but only as manual commands; a throttled A53 makes gait timing jitter | LOW | New small node in `dog_hardware`, or a script in the compose stack. Feeds DIA-01. |
| DIA-05 | Loop timing counters: servo_driver update period max/mean and locomotion 50 Hz overrun count | Missing | The 2 GB / A53 / Docker setup is the least measured part; jerky gait with the web page open is a listed risk | LOW-MED | `servo_driver_node.cpp::tick`, `locomotion_node.cpp:291`. |
| DIA-06 | Honest measured-vs-commanded labelling: document/flag that `joint_states` on the robot are commands (no feedback) | Partial | Prevents wrong conclusions from bags and from perception code | LOW | Doc + a field in DIA-01; do not rename the topic (sim parity). |
| DIA-07 | Battery/rail telemetry visible while driving | Covered if INA present | `power` -> web `power` message exists | - | Confirm sensor presence (open in PROJECT.md). |

#### Differentiators

| ID | Feature | Value | Complexity | Notes |
|----|---------|-------|------------|-------|
| DIA-08 | Per-leg current (4 x INA219) for stall and foot-contact detection | Called out in `TERRAIN.md` as the next best improvement (contact sensing); also catches a single stalled leg | HIGH | Hardware; unlocks the descent/stairs weakness. v2. |
| DIA-09 | Aggregator + `rqt_robot_monitor` view on the laptop | Nicer than the web banner | LOW-MED | Only on the PC side; do not put in the robot image. |
| DIA-10 | Post-run report generator (extend `tools/sim_video/report`) from a bag: tilt, speed vs command, current, temperature notes | Makes sim-to-real gap comparison systematic | MEDIUM | Only after several floor runs exist. |

#### Anti-features

| Anti-Feature | Why avoid | Alternative |
|--------------|-----------|-------------|
| Full telemetry dashboard/Grafana stack | Load on a 2 GB board, not needed for a first walk | rosbag + small offline report |
| Streaming high-rate debug topics over the web socket | `web_teleop` is already a CPU outlier (8 % of a PC core, `REVIEW.md` #23) | Throttle status to about 2 Hz; keep bags for detail |
| Perception/localization diagnostics on the robot | No sensors | Skip until hardware exists |

### Category 4 - Teleop hardening (`TEL`)

#### Table stakes

| ID | Feature | Status | Why expected | Complexity | Notes and dependencies |
|----|---------|--------|--------------|------------|------------------------|
| TEL-01 | Deadman, joy-loss stop 0.5 s, web drive timeout 0.4 s, keyboard stop on Ctrl-C, `cmd_vel` timeout, E-STOP button on all three channels | Covered | Standard | - | `joy_teleop_node.cpp`, `gamepad_node`, `web_teleop.py`, `keyboard_teleop.cpp`, `locomotion_node.cpp:297`. Real Bluetooth latency and dropout behaviour untested on hardware; `DEPLOYMENT.md` stage 8 table covers it. |
| TEL-02 | Web calibration channel off unless deliberately enabled; web `estop` release not possible from a stray client | Partial | `allow_calibration` defaults true and the calibration channel can drive `servo_driver` params (`CONCERNS.md`, confirmed in `web_teleop.py`); any client may release E-STOP | LOW | Default `allow_calibration:false` in the robot compose/launch, enable only for autocal sessions. Release requires the same client that holds drive, or a confirm step. |
| TEL-03 | WebSocket `Origin` check against `Host` | Missing | Cheap defence against a browser page on the LAN opening `ws://<robot>:8080/ws` (cross-site WebSocket hijack) | LOW | `dog_web/wsserver.py::Server._handle`; extend `test_wsserver.py`. |
| TEL-04 | One authoritative operator for floor runs: launch profile where only the gamepad publishes (web view-only, keyboard off) | Missing | Three channels with "last command wins" is a hazard when two are live at once; verified that an idle gamepad does not overwrite, but concurrent drivers interleave | LOW | Ride on CAL-07 profile: `web:=view_only`. No new node needed. |
| TEL-05 | Gamepad hot-unplug/reconnect and stale-axis safety verified on the real Bluetooth pad | Covered in code, untested on hardware | The pad is the primary control for the acceptance criterion | LOW (test effort) | `gamepad_node` (5 ms poll). Add to stage 8 checklist evidence. |
| TEL-06 | First-floor speed caps enforced at the mapper and controller: turbo (RB) disabled or capped in the floor profile | Partial | `normal_scale 0.5` + turbo to 1.0 gives 0.15 m/s; first floor should start at about 0.05 | LOW | `teleop.yaml` / `robot.yaml` `limits`, applied by CAL-07. |
| TEL-07 | Mode/fault feedback to the gamepad user (rumble or LED) on E-STOP/fallen/low battery | Missing | Operator often watches the robot, not the screen | LOW-MED | Joystick API rumble is supported by `/dev/input/js*` only through evdev force-feedback; check feasibility. If not, use the web banner (DIA-02). |

#### Differentiators

| ID | Feature | Value | Complexity | Notes |
|----|---------|-------|------------|-------|
| TEL-08 | Source arbitration (gamepad > web > keyboard) with lock and timeout, twist_mux-style | Lets all three stay enabled safely | MEDIUM | `twist_mux` is not in `ros-base`; implement in `TeleopPublisher` or `locomotion_node`. |
| TEL-09 | Shared-secret token for the web socket, bind to a specific interface | Blocks casual LAN clients | MEDIUM | Not a security boundary while the ROS graph is open (`docker-compose.yml` uses host network, domain 0). Decide in requirements whether PROJECT.md's open question becomes v1 or v2. My recommendation: TEL-02 and TEL-03 in v1, token in v2. |
| TEL-10 | Rate-limited "ramp" on button-triggered commands: `stand`/`lie` debounce and refusal during E-STOP | Prevents button mashing | LOW | Already returns accept/reject in `LocomotionController::request`. |

#### Anti-features

| Anti-Feature | Why avoid | Alternative |
|--------------|-----------|-------------|
| Autonomous behaviours from the pad (greet, survey) on the floor run | Extra motion modes add risk with no bearing on the first walk | Keep them off-limits in the floor profile |
| Adding new teleop channels (phone app, VR, voice) | Scope creep | Existing three |
| Full web authentication system (accounts, TLS) | Heavy on a hand-rolled stdlib server (`wsserver.py`) | TEL-02/03 now, token later |

### Category 5 - Gait robustness incl. backward walking (`GAIT`)

Diagnosis of the backward weakness (from `gait.cpp`, `TERRAIN.md`, `SIMULATION.md`): `TrotGait::update` is direction-symmetric - identical `step_height` (20 mm), identical `sin` lift and cosine horizontal blend that starts moving the foot at swing start (`p.z = h*sin(pi*s)` while `p.x/p.y` advance immediately), and `LocomotionController::setVelocity` clamps `vx` symmetrically. With knees pointing backward (`knee_direction: -1`), moving backward drives the knee/shin side into obstacles, and a foot that moves horizontally while still near the ground is stubbed. Measured result: 17-36 % of commanded distance on 10 mm waves, 39 % vs 40 % threshold on flat ground once, and CI carries `--backward-ratio 0.2` (flat) and `0` (uneven) to avoid flakes. Also note that "slope descent" in `walk_check` IS a backward manoeuvre, so the two owner-listed weak spots overlap; only stair descent (crawl gait) is a separate issue.

#### Table stakes

| ID | Feature | Status | Why expected | Complexity | Notes and dependencies |
|----|---------|--------|--------------|------------|------------------------|
| GAIT-01 | Backward walking on flat floor that meets the same bar as forward (>= 40 % of command, no fall, low run-to-run spread) | Partial | Owner's first weak spot; a gamepad user will press "back" on the floor | MEDIUM | `gait.cpp` swing shape. Candidate levers, in order of expected payoff: (a) lift-then-move swing (vertical rise to a fraction of the swing before horizontal advance, direction-aware), (b) asymmetric backward speed cap (e.g. 0.6-0.7 of forward) in `setVelocity`, (c) a slightly forward stance offset for backward, (d) backward-only swing height within the 20-30 mm window (35 mm tipped the robot per `TERRAIN.md`). Must be evaluated together with the stability limit on height. |
| GAIT-02 | Statistical acceptance for backward: N runs (proposal 5) on flat, waves 10 mm, rocks 10 mm, report min/median, raise the CI threshold from 0.2 only after the spread is measured | Missing | The 0-36 % spread means a single run proves nothing; current thresholds hide the weakness | MEDIUM | `walk_check.py`, `terrain_sweep.py`, `.github/workflows/ci.yml`. Run one sim at a time (`TERRAIN.md` warns parallel runs invalidate results). |
| GAIT-03 | Descent by backward walking on slopes up to the first-floor-relevant limit (7-10 deg) stays within the tilt and no-fall gates | Partial | Slope descent is 53-80 % of command at 10 deg today; comes from GAIT-01 | LOW (after GAIT-01) | `TERRAIN.md` table, `terrain_sweep --terrain slope`. Real-robot slope work stays out of scope per PROJECT.md. |
| GAIT-04 | Soft tilt-based derating: above about 15-20 deg tilt the controller reduces speed / stops stepping before the SAF-05 hard limit | Missing | Bridges the gap between "walking fine" and "fallen"; matches `walk_check` tilt gate of 20 deg | LOW-MED | `locomotion.cpp` (`vel_target_` limiting), shares IMU input with SAF-05. |
| GAIT-05 | Sim gate with real-robot margin: repeat `walk_check` with friction, body mass and servo-lag variations (proposal +-20 % mass, low/high friction, added command latency) because the sim uses ideal servos with a 6 rad/s cap and no backlash/sag while MG996R has 1-2 deg backlash | Missing | The whole reason to improve in sim first is transfer; `TERRAIN.md` already recommends a 30 % safety margin. No latency/friction launch args found in `sim.launch.py` (not exhaustively verified) | MEDIUM | `dog_gazebo` launch and URDF params (`urdf.py` has mass keys). |
| GAIT-06 | Flat-floor omnidirectional pass on the tuned gait: forward, back, lateral, both turns, arcs, lie, `walk_check` 8/8 with the first-floor limit profile | Covered (nominal) | Already CI-gated at nominal settings | - | Retain as the regression gate for any gait edit. |
| GAIT-07 | Real-robot odometry/speed scale calibration and gait retune procedure (2 m runs at 0.05-0.12 m/s, `odom.scale`, `gait.period/duty/step_height`) | Covered (docs) | `DEPLOYMENT.md` stage 10 | LOW | Execute after first floor pass; CAL-15 speeds it up. |

#### Differentiators

| ID | Feature | Value | Complexity | Notes |
|----|---------|-------|------------|-------|
| GAIT-08 | Foot-contact-aware swing ("wait for the foot") using per-leg current | Fixes stubbing and stair-descent creep; `TERRAIN.md` and `REVIEW.md` #20 name it as the top improvement | HIGH | Needs DIA-08 hardware. v2. |
| GAIT-09 | Stair descent in crawl gait | Owner listed "descending"; sim currently falls (`REVIEW.md` #20) | HIGH | Depends on knowing where support feet are; out of scope for floor milestone. Keep in v2 or a later sim milestone. |
| GAIT-10 | Servo non-idealities in sim (backlash, first-order lag, torque limit from `HARDWARE.md`, voltage sag) | Sim closer to hardware; complements GAIT-05 | MEDIUM-HIGH | `joint_command_bridge.py` is the insertion point. |
| GAIT-11 | Automatic gait parameter search (sim sweep for `period/duty/step_height/max_step`) | Systematic tuning instead of manual | MEDIUM | Runs are minutes each on a shared CPU; do only after GAIT-02 defines a scalar score. |

#### Anti-features

| Anti-Feature | Why avoid | Alternative |
|--------------|-----------|-------------|
| Raising step height globally to fix stubbing | 35 mm tipped the robot in sim on waves 20-30 mm and rocks 30-40 mm (`TERRAIN.md`) | Direction-aware lift shape, small backward-only increase |
| Reinforcement-learning or MPC gait now | Needs joint feedback, large compute, different architecture | Hand-tuned trot/crawl already validated |
| Lowering CI backward thresholds again to make CI green | Hides the exact weakness this milestone should fix | GAIT-02 statistics |
| Running steps/slopes on the real robot in this milestone | PROJECT.md out of scope; stair descent fails in sim | Flat floor only |

## Feature Dependencies

```
SAF-01 (checked writes) --> SAF-03 (fault path publishes estop) --> DIA-02 (fault banner)
SAF-02 (verified estop) ---> SAF-03
SAF-04 (PWM watchdog) -----> SAF-03
SAF-03 --enhances--> SAF-12 (arm gate) --> SAF-11 (HW OE cutoff, v2)
SAF-06 (IMU required) --> SAF-05 (tip-over) --> GAIT-04 (soft tilt derate)
SAF-07 (power guard measured) --requires--> CAL-11 (rail verified) + physical sensor
SAF-08 (idle rest) has no dependency; needs only locomotion timer
CAL-07 (floor profile) --requires--> SAF-06 flag, TEL-02/TEL-04/TEL-06 settings
CAL-08 (IMU checks) --precedes--> enabling slope.compensation / heading.hold on the floor
CAL-09 (arm64 smoke test) --> CAL-11 --> CAL-01 hand/auto calibration --> CAL-06 sanity --> floor run
DIA-01 (diagnostics) --> DIA-02, CAL-10 (bag topic list), DIA-10 (report)
GAIT-01 (backward fix) --> GAIT-02 (statistics) --> GAIT-03 (slope descent gate)
GAIT-05 (robust sim gate) --enhances--> GAIT-01, GAIT-04
GAIT-08 (contact swing) --requires--> DIA-08 (per-leg current hardware)
TEL-08 (arbitration) --conflicts--> TEL-04 (single-source profile) only in scope; pick TEL-04 for v1
```

### Dependency notes

- **SAF-03 is the keystone.** The driver can currently only receive `estop`; SAF-01, SAF-02 and SAF-04 all need a way to tell `locomotion_node` and the UI that the driver relaxed. Building the latch once avoids three separate ad hoc paths. Watch for the driver hearing its own `estop=true` publication (idempotent handling needed in `servo_driver_node.cpp:156-162`).
- **SAF-05 requires an IMU on the robot at all times** (SAF-06). Both are only meaningful once the IMU axes are verified (CAL-08); a wrong-axis IMU could trigger false hard-tier faults or miss real ones.
- **CAL-07 (floor profile) is the integration point** for SAF-06, TEL-02, TEL-04, TEL-06 and limits; build it early so later work has a target configuration.
- **GAIT items are independent of hardware and can start immediately in sim** (owner's order 1). GAIT-05 should run before the gait is frozen, otherwise the tuned gait may be over-fit to ideal servos.
- **Perception/guard stays off on the robot** (`perception:=false`): no sensor exists, and a silent guard drops limits after 1 s (`guard_timeout`), which is misleading if left on.

## MVP Definition

### Launch With (v1, this milestone)

Order follows the owner's plan (sim gait -> protection -> calibration/Docker -> floor).

- [ ] GAIT-01, GAIT-02, GAIT-03 - backward and slope-descent improvement with statistical proof in sim (owner item 1)
- [ ] GAIT-04, GAIT-05 - soft tilt derating and a perturbed-sim gate for margin
- [ ] SAF-01, SAF-02, SAF-03, SAF-04 - checked writes, verified E-STOP, fault latch, PWM watchdog (owner item 2)
- [ ] SAF-05, SAF-06 - tip-over reaction and IMU-required policy (owner item 2)
- [ ] SAF-07, SAF-08 - measured power thresholds/sensor-required policy, idle rest for thermal safety
- [ ] DIA-01, DIA-02, DIA-03 - diagnostics message, fault banner, real error logs
- [ ] CAL-04, CAL-05, CAL-06, CAL-07, CAL-08 - live-set guard, persistence, sanity check, floor profile, IMU sign gating (owner item 3)
- [ ] CAL-09, CAL-10 - arm64 smoke test/preflight and standard bag command (owner item 3)
- [ ] TEL-02, TEL-03, TEL-04, TEL-06 - calibration off by default, Origin check, single-source floor profile, speed caps
- [ ] SAF-09 gate and executing `DEPLOYMENT.md` stages 5-10 with recorded results (owner items 3-4)

### Add After Validation (v1.x)

- [ ] SAF-12 arm gate, SAF-15 tiered power response - after first hardware evidence on brownouts
- [ ] DIA-04, DIA-05 Pi health and loop timing - after the first floor session shows whether the A53 is a problem
- [ ] CAL-12, CAL-13, CAL-15 - self-test, wizard, live gait tuning if tuning is slow
- [ ] TEL-07 gamepad feedback, TEL-10 command debounce

### Future Consideration (v2+)

- [ ] SAF-11 hardware OE cutoff/MCU watchdog - needs hardware and wiring
- [ ] DIA-08 per-leg current -> GAIT-08 contact-aware swing -> GAIT-09 stair descent
- [ ] TEL-08 source arbitration, TEL-09 token
- [ ] GAIT-10 servo non-idealities in sim, GAIT-11 auto-tuning, SAF-13 self-righting, SAF-14 thermal model

## Feature Prioritization Matrix

| Feature | User Value | Implementation Cost | Priority |
|---------|------------|---------------------|----------|
| SAF-03 fault latch/path | HIGH | MEDIUM | P1 |
| SAF-01 checked I2C writes | HIGH | LOW-MED | P1 |
| SAF-02 verified E-STOP | HIGH | LOW | P1 |
| SAF-04 PWM watchdog | HIGH | LOW | P1 |
| SAF-05 tip-over reaction | HIGH | MEDIUM | P1 |
| SAF-06 IMU required | HIGH | LOW | P1 |
| SAF-08 idle rest | HIGH | LOW | P1 |
| SAF-07 power guard measured | HIGH | MEDIUM | P1 (blocked on sensor confirmation) |
| GAIT-01 backward fix | HIGH | MEDIUM | P1 |
| GAIT-02 statistical acceptance | HIGH | MEDIUM | P1 |
| GAIT-05 perturbed sim gate | MEDIUM | MEDIUM | P1 |
| GAIT-04 soft tilt derate | MEDIUM | LOW-MED | P1 |
| CAL-07 floor profile | HIGH | LOW | P1 |
| CAL-04 live-set guard | HIGH | LOW | P1 |
| CAL-09 arm64 smoke/preflight | HIGH | MEDIUM | P1 |
| DIA-01 diagnostics | MEDIUM | MEDIUM | P1 |
| DIA-02 fault banner | MEDIUM | LOW-MED | P1 |
| TEL-02/03 calibration gate + Origin | MEDIUM | LOW | P1 |
| TEL-04 single-source profile | MEDIUM | LOW | P1 |
| CAL-05/06 persistence + sanity | MEDIUM | LOW-MED | P1 |
| CAL-08 IMU sign gating | HIGH | LOW | P1 |
| SAF-12 arm gate | MEDIUM | MEDIUM | P2 |
| DIA-04/05 Pi health/timing | MEDIUM | LOW-MED | P2 |
| CAL-12/13/15 | LOW-MED | MEDIUM | P2 |
| TEL-08/09 | LOW-MED | MEDIUM | P3 |
| SAF-11, DIA-08, GAIT-08/09 | HIGH later | HIGH | P3 |

**Priority key:** P1 must have for the floor milestone, P2 add when possible, P3 future.

## Competitor Feature Analysis

| Feature | Pupper / Spot Micro | Unitree Go1 (pro) | Our approach |
|---------|--------------------|--------------------|--------------|
| Calibration | Interactive script / keyboard node, done on a stand | Factory calibrated | Keep `calib_pose` + `tools/autocal`; add sanity check and step guard (CAL-04/06) |
| E-STOP | Software stop or power switch only | Software + protected mode + frame | Software E-STOP exists; make it verified and fault-latched; physical switch mandatory |
| Fall handling | None (Pupper), gyro self-right (Petoi) | Fall protection mode | Relax and require re-stand (SAF-05); no self-righting yet |
| Command limits | Spot Micro README: none, "may damage hardware" | Built in | Already has speed/accel limits, clamps, slew limits; add floor profile |
| Thermal / battery | User discipline | Warnings and shutdown | Idle rest, INA226 guard, IR-thermometer procedure |
| Diagnostics | Terminal logs | Proprietary app | `diagnostic_msgs` + web banner |
| Backward gait | Not specifically addressed in public docs found | Symmetric legs (knees do not restrict) | Direction-aware swing for backward-facing knees (GAIT-01) |

## Open questions for requirements

1. Is an INA226/INA219 physically installed? It decides whether SAF-07 is real protection or just documentation. With a 10 mOhm shunt it saturates near 8 A, so verify the shunt value before trusting any threshold.
2. Which fault action is preferred on watchdog/I2C fault: relax (limp, robot drops about 0.15 m) or a controlled `lie`? Relax is the recommendation for a dead controller; `lie` requires a live `locomotion_node`.
3. Hard tip-over threshold: pick from recorded sim tilt distributions (`walk_check` logs), then confirm on the floor with a spotter; the proposal (35-45 deg hard, 15-20 deg soft) is unvalidated.
4. Does "descending" in the owner's list mean slope descent (backward walking, in scope via GAIT-03) or stair descent (crawl, GAIT-09, deferred)?
5. Is web authentication in this milestone (PROJECT.md leaves it open)? Recommendation above: calibration gate and Origin check in v1, token in v2.

## Sources

- `/home/sg/robot-dog/.planning/PROJECT.md`, `/home/sg/robot-dog/.planning/codebase/ARCHITECTURE.md`, `/home/sg/robot-dog/.planning/codebase/CONCERNS.md` - project context (HIGH for what they say; CONCERNS.md re-verified as listed above)
- `/home/sg/robot-dog/docs/CONTROL.md`, `CALIBRATION.md`, `DEPLOYMENT.md`, `TERRAIN.md`, `GAITS.md`, `REVIEW.md`, `HARDWARE.md` - project documentation (HIGH)
- Source files read for verification (HIGH): `ros2_ws/src/dog_hardware/src/servo_driver.cpp`, `servo_driver_node.cpp`, `servo_bus.cpp`, `power_monitor_node.cpp`; `dog_control/src/locomotion.cpp`, `locomotion_node.cpp`, `gait.cpp`; `dog_teleop/src/joy_teleop_node.cpp`, `mapping.cpp`; `dog_web/dog_web/web_teleop.py`; `dog_bringup/config/servos.yaml`, `power.yaml`, `imu.yaml`, `robot.yaml`, `teleop.yaml`; `docker/Dockerfile`
- Stanford Pupper calibration guide: https://pupper.readthedocs.io/en/latest/guide/calibration.html (MEDIUM, fetched)
- Spot Micro project README: https://github.com/mike4192/spotMicro (MEDIUM, fetched)
- Spot Micro servo calibration doc: https://github.com/mike4192/spotMicro/blob/master/docs/servo_calibration.md (LOW, seen in search results only)
- PCA9685 brownout / servo rail sag discussions (search results, forums, e.g. https://forum.arduino.cc/t/pca9685-module-controlling-10-servo-motors-issue/638676) (LOW-MEDIUM)
- MG996R datasheet figures (stall 2.5 A at 6 V, 11 kg-cm): https://components101.com/motors/mg996r-servo-motor-datasheet (MEDIUM)
- Petoi Bittle docs (self-righting, gyro): https://docs.petoi.com/ (LOW, snippets only)
- Unitree Go1 manual / product page (protected mode, fall/overheat/low-voltage protection): https://www.unitree.com/go1/ and https://static.generation-robots.com/media/user-manual-go1-unitree-robotics-de.pdf (LOW, snippets only)
- ROS 2 diagnostics packages (`diagnostic_msgs`, `diagnostic_updater`, aggregator): https://github.com/ros/diagnostics and https://index.ros.org/p/diagnostic_updater/ (MEDIUM; the claim that `diagnostic_updater` is not in `ros-base` should be confirmed with `apt`/`rosdep` before relying on it)
- Foot trajectory / swing height literature (search results, e.g. https://www.nature.com/articles/s41598-024-84060-5): general support that swing shape and height drive stubbing and gait stability; nothing specific to knee-backward robots walking backward (LOW). GAIT-01 levers are engineering proposals to be tested in sim.

---
*Feature research for: servo quadruped first floor bring-up (RobotDog 2.0)*
*Researched: 2026-09-29*
