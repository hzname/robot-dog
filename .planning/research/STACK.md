# Stack Research

**Domain:** Safety and reliability for a servo quadruped (Banana Pi BPI-M4 Zero + PCA9685 + 12x MG996R + MPU6050, ROS 2 Jazzy/Lyrical, C++17, Docker arm64, 2 GB RAM) before its first real-floor run. This is a SUBSEQUENT milestone: the codebase exists, so this file says what to ADD or EXTEND, and what is already HAVE.
**Researched:** 2026-09-29
**Confidence:** MEDIUM-HIGH. Hardware/kernel/ROS facts are HIGH (datasheet, mainline kernel source and ros2/variants read directly). Threshold values and gait effects are MEDIUM-LOW: they are starting points to be confirmed in Gazebo and on the robot.

## Bottom line

1. **No new runtime dependency is needed.** Everything below uses the Linux kernel APIs the repo already uses (i2c-dev), `diagnostic_msgs` (already in ros-base), `launch`, `rosbag2` (in ros-base) and the Python standard library. This is deliberate: `diagnostic_updater`, `twist_mux`, `libgpiod`, `websockets`/`aiohttp` are all NOT in ros-base or differ between Jazzy and Lyrical distros.
2. **Make the I2C failure class impossible to ignore at compile time:** mark `ServoBus::setPulseUs/disable/disableAll` `[[nodiscard]]`. The existing `-Wall -Wextra -Wpedantic` then flags `servo_driver.cpp:262` (`bus_->setPulseUs(...)` result dropped) and every future offender, on both distros, with no extra tool.
3. **Set an I2C timeout, or one dead chip freezes the servo node for seconds.** Mainline `i2c-mv64xxx` (the driver for Allwinner `sun6i-a31-i2c`, i.e. the H618) hard-codes a 1 s adapter timeout and waits a second time when it aborts. One failed `write()` can block ~2 s, and `ServoDriver::update` issues up to 12 of them per 10 ms tick in a single-threaded executor. Call `ioctl(fd, I2C_TIMEOUT, 5)` (units of 10 ms = 50 ms; adapter-wide) in every node that opens the bus.
4. **Write the full 12-channel state every tick in one auto-increment transaction** (state-based, not delta-based). It cuts bus time, updates all 12 outputs at the same STOP (MODE2 OCH=0 default), and self-heals any lost write within one tick.
5. **Watchdog in three tiers.** Tier 1 in `servo_driver_node` (stale commands, I2C fault, latched re-arm) is mandatory. Tier 2 (launch `OnProcessExit` + entrypoint runs `pca9685_probe off`) covers a crashed node. Tier 3 (heartbeat GPIO into an external supervisor that drives the PCA9685 OE pin) is the only thing that covers SIGKILL / OOM / a hung Pi, and is recommended before any UNTETHERED run.
6. **Tilt protection = one scalar, `tilt = acos(1 - 2(qx^2 + qy^2))`, in a new ROS-free `TiltGuard`** in `dog_control`, independent of the slope-compensation 25 deg gate, reacting through the existing `estop` chain. Fall threshold about 30 deg (the static tip-over limit of this geometry is about 28-34 deg).
7. **Diagnostics: publish `diagnostic_msgs/DiagnosticArray` by hand** (about 60 lines in a header), not `diagnostic_updater` (not in ros-base). Record every floor run with `ros2 bag record` (in ros-base).
8. **Web hardening in stdlib Python only:** `allow_calibration` default false, `Origin` check, first-message token with `hmac.compare_digest`, client cap, idle timeout. Note: `CONCERNS.md` refers to a "drive lock" in the web code; there is none (see "Verified against the code").
9. **Backward-trot tuning needs no library.** Instrument first (swing traces, joint speed and clamp counts, forward vs backward), then sweep 3-4 parameters with at least 8 repeats per point, because the backward result already varies 0-92 % run to run.
10. **Found while verifying:** `power.yaml` `overcurrent_a: 5.0` (E-STOP after 0.5 s) contradicts `HARDWARE.md` ("12 MG996R take 6-12 A while walking"). On the real robot this can false-trip the E-STOP on the first walk. Baseline the current with `overcurrent_action: warn` first.

## Verified against the code (spot-check of CONCERNS.md)

| Claim | Result | Evidence |
|-------|--------|----------|
| I2C write results ignored | CONFIRMED | `servo_driver.cpp:262` `bus_->setPulseUs(...)`, return value dropped; `Pca9685Bus::writeChannel` returns bool with no errno, no log, no retry (`servo_bus.cpp:155-163`) |
| No PWM output watchdog | CONFIRMED, and weaker than described | `servo_driver_node.cpp` `tick()` only logs "holding position" after `command_timeout` (0.5 s) and holds forever; `relax_on_exit` runs only in the destructor |
| Restart hazard (new) | CONFIRMED, not in CONCERNS | `ServoDriver::estop_` starts false and servos start `State::OFF`; the first `joint_commands` after a driver restart moves every leg straight to the commanded pose (`setTargets` -> PENDING -> ON with `current_ = target_`, staggered per leg by 0.15 s, no slew) |
| No tilt/fall reaction | CONFIRMED | only `LocomotionController::setImuAttitude` (`locomotion.cpp:213`), which returns early when `abs(tilt) > slope.max_deg` (25 deg); nothing acts on it |
| Stale IMU keeps old slope (new) | CONFIRMED | `slope_valid_` is set once (`locomotion.cpp:231`) and never cleared; after `imu_node` dies the feet stay shifted by the last slope. Only the heading hold checks age (0.2 s, `locomotion_node.cpp:338`) |
| IMU / power sensor loss is silent | CONFIRMED | `imu_node.cpp` and `power_monitor_node.cpp`: "no IMU/current sensor found ... off" (INFO) and exit; a failed read is a WARN every 5 s and nothing else |
| E-STOP not latched at the driver | CONFIRMED | `servo_driver_node.cpp:156`: volatile QoS, any publisher's `false` releases (gamepad Start, web, keyboard `r`); `power_monitor` sends `true` once per event |
| Web: no auth, no Origin check, calibration on | CONFIRMED | `web_teleop.py:68` `allow_calibration` default True; `wsserver.py` `_handle` never reads `origin`; `host: 0.0.0.0` in `teleop.yaml` |
| "Only the client that holds the drive lock may release E-STOP" | WRONG PREMISE | no drive lock exists: `web_teleop.py` keeps a `web_clients` set and one `DriveWatchdog` per client; all teleop nodes publish the same `cmd_vel` (last writer wins) |
| Bus speed unspecified | CONSISTENT | not set anywhere in the repo; mainline default is 100 kHz when DT has no `clock-frequency` (`i2c-mv64xxx.c`). Actual value on the Armbian image is not verified |
| Power thresholds | CONTRADICTION (new) | `power.yaml` 5.0 A / 0.5 s -> `estop`; `HARDWARE.md` 6-12 A total while walking |

## Recommended Stack

Legend for **Status:** HAVE = already in the repo, keep. EXTEND = exists, change it. ADD = new code, still no new dependency.

### A. I2C error detection and recovery

| Technique | Version / interface | Status | Why |
|-----------|--------------------|--------|-----|
| `[[nodiscard]]` on `ServoBus::setPulseUs`, `disable`, `disableAll` (and the `Pca9685Bus` overrides) | C++17 | ADD | Turns "result ignored" into a compile warning that CI (Jazzy + Lyrical, `-Wall -Wextra -Wpedantic`) already reports. Tests that call the bus without checking must use `EXPECT_TRUE(...)`. `relax()` in the destructor must log or `static_cast<void>` explicitly |
| `ioctl(fd, I2C_TIMEOUT, 5)` after `I2C_SLAVE` | Linux i2c-dev | ADD | 50 ms instead of the driver's 1 s. Verified in mainline: `i2cdev_ioctl` sets `client->adapter->timeout = msecs_to_jiffies(arg * 10)`, so it is adapter-wide (all nodes share it: set the same value everywhere: `servo_driver`, `imu`, `power_monitor`, `pca9685_probe`). A 49-byte frame is about 1.1 ms at 400 kHz and 4.4 ms at 100 kHz, so 50 ms leaves wide margin |
| Leave `I2C_RETRIES` alone | Linux i2c-dev | HAVE (default) | The kernel retries only on lost arbitration; this is a single-master bus. Do retries in the application, bounded by the tick budget |
| Errno-aware `write()` wrapper | POSIX | ADD | Retry once on `EINTR` immediately; `ENXIO`/`EREMOTEIO` = NACK (chip missing or powered down), `ETIMEDOUT` = bus hung, `EIO`/`EAGAIN` = glitch/arbitration. Store `last_errno`; stop issuing further writes in the same tick after the first hard failure (never loop 12 times into a hung bus) |
| Whole-frame write: `Pca9685Bus::writeAll()` | PCA9685 datasheet rev 4 (2015), AI=1 already set in `Pca9685Bus::open` | ADD | One `write()` of 1 register byte (0x06) + 12 x 4 bytes. With MODE2 OCH=0 (default, "outputs change on STOP") all 12 servos update at the same instant; a delta-only scheme leaves a lost write uncorrected until the joint moves again. OFF channels are sent as full-off (`LEDn_OFF_H` bit 4 = 0x10) so relaxed servos stay off |
| Health counter with a fault latch | in `ServoDriver` (ROS-free) | ADD | `consecutive_failures`, `failed_frames` (1 s window), `last_errno`. 1-2 failed frames = DEGRADED (retry), 3 consecutive frames (60 ms at 50 Hz) = FAULT: `relax()` and latch, re-arm only by an explicit operator action. Unit-testable through `MockBus::failNext(n, errno)` |
| Cheap read-back verification | `readReg` (already in `Pca9685Bus`) | ADD | Each tick read back one channel's 4 registers (round-robin), and MODE1/PRESCALE once per second. Catches the silent case where a glitch corrupts a byte without a NACK, or the chip was reset (datasheet: MODE1 power-up default has SLEEP=1, oscillator off, outputs off). Mismatch = rewrite and count; 3 in a row = FAULT |
| I2C at 400 kHz (device-tree `clock-frequency = <400000>` through an Armbian user overlay), servo frame at 50 Hz | Armbian overlay | EXTEND | Bus budget today, computed from the code: a channel write is 6 bytes = about 0.55 ms at 100 kHz, 12 channels about 6.7 ms per tick at 100 Hz = about 67 %, plus the IMU (about 1.6 ms x 100 Hz = 16 %) = about 83 % before kernel overhead. `REVIEW.md` item 12 reaches the same conclusion. One 49-byte frame at 50 Hz (the PWM period is 20 ms, faster updates change nothing) is about 5.6 % at 400 kHz. PCA9685 is Fm+, MPU6050 400 kHz max, INA226 faster: 400 kHz is safe for all three |
| `write_us_max` / `write_us_mean` in diagnostics | `steady_clock` | ADD | Doubles as a bus-speed test on the robot (49-byte frame must be about 1.1 ms at 400 kHz and about 4.4 ms at 100 kHz) and as early warning for a marginal bus |
| Stuck-bus (SDA/SCL held low) handling: detect, relax, tell the operator | policy | ADD | The kernel's generic 9-clock GPIO recovery needs `scl-gpios`/`sda-gpios` plus a pinctrl "gpio" state in the device tree; mainline `i2c-mv64xxx` only wires up recovery info when pinctrl is present. Whether the Armbian BPI-M4 Zero DT provides them is NOT verified (LOW). Treat a stuck bus as FAULT + power cycle. MPU6050 is a known offender for holding the bus after a mid-transfer reset |
| Fault-injection unit tests | GoogleTest (`ament_cmake_gtest`, HAVE) | ADD | Same ROS-free core pattern as `test_servo_driver.cpp`: scripted `MockBus` failures for retry, latch, re-arm and "no write storm after first failure". Also build the new core code with `-fsanitize=address,undefined` on the PC once |

Electrical items already in `HARDWARE.md`/`DEPLOYMENT.md` (VCC PCA9685 from 3.3 V, short twisted I2C wires) stay. Add one check: the MPU6050 and INA226 breakouts have their own pull-ups; several in parallel give a low total resistance. Measure SDA to 3.3 V with the board unpowered (aim for 2-4.7 kOhm total).

### B. Output watchdog for the PWM (three tiers)

Facts from the PCA9685 datasheet (NXP rev 4, Table 6, 7.4, 7.5): `OE` is an active-LOW output-enable input; while `OE` is HIGH the outputs take the value in MODE2 `OUTNE[1:0]`, default 00 = LEDn low, i.e. no pulses, MG996R goes limp. The chip has no timeout of its own: it keeps the last PWM forever. Power-on reset only triggers when VDD falls below 0.2 V, so a sag on a 3.3 V rail does not reset it.

| Tier | Mechanism | Covers | Status |
|------|-----------|--------|--------|
| 1 | In `servo_driver_node::tick()`: `command_timeout` 0.5 s stays a hold; add `command_relax_timeout` (default 2.0 s) -> `ServoDriver::relax()` and latch the same way as E-STOP. Resume only after `estop` false is received again (same re-arm as an E-STOP). Fix the restart hazard at the same time: after any relax/fault, or on node start, do not accept `joint_commands` until locomotion is in a "stand/lie" transition that starts from the current pose, or simply require the operator's E-STOP release cycle | locomotion or teleop dead, network dead, I2C fault | EXTEND |
| 2 | `robot.launch.py`: `RegisterEventHandler(OnProcessExit(target_action=servo_driver, on_exit=[ExecuteProcess(cmd=['ros2','run','dog_hardware','pca9685_probe','off'])]))` and `docker/entrypoint.sh` runs `pca9685_probe off` before `exec "$@"`. `pca9685_probe` is already built and installed | the driver process crashed or was killed while launch survives; container restart (`restart: unless-stopped`) | ADD |
| 3 | Hardware heartbeat: `servo_driver_node` toggles one spare GPIO each tick only while healthy (I2C ok, commands fresh, no tilt fault); an external supervisor (a STWD100-class watchdog IC, timeouts 3.4 ms / 6.3 ms / 102 ms / 1.6 s available, or a retriggerable monostable) drives PCA9685 `OE` HIGH, or switches the servo V+ rail, when edges stop. Choose about 100 ms | SIGKILL, OOM kill of the container (2 GB RAM), kernel hang, Pi brown-out that leaves the PCA9685 powered | ADD (hardware, recommended before an untethered run) |

Tier 3 details:
- The PCA9685 breakout usually ties `OE` to GND with a resistor: measure it, and cut/override it. `WDO` on the STWD100 is open-drain active-low, so the polarity into `OE` needs one inverter or a small MOSFET; verify on the bench (I checked the timeout options, not the exact part/output variant: MEDIUM).
- GPIO access: use the kernel GPIO character device directly (`linux/gpio.h`, uAPI v2, kernel 5.10+), mapping `/dev/gpiochipN` in `docker-compose.yml`, exactly like `servo_bus.cpp` uses `linux/i2c-dev.h`. Do not use `libgpiod`: Ubuntu 24.04 (Jazzy image) ships libgpiod 1.6.3 (v1 API), newer distros ship 2.x with a different API, which would split the Jazzy and Lyrical builds.
- An inline physical power switch or relay on the servo V+ line is the cheapest hardware E-STOP and should exist for the first floor run regardless (LOW confidence source: general practice, but the reasoning is direct).
- A dedicated MCU watchdog (`COMPUTE.md`, about 10 USD, plus firmware and a serial protocol) is out of scope for this milestone.

### C. IMU-based tilt / fall detection

| Technique | Interface | Status | Why |
|-----------|-----------|--------|-----|
| `TiltGuard` class in `dog_control` core (ROS-free, `test_tilt_guard.cpp`), fed from `imu/data` in `locomotion_node` | `tilt = acos(1 - 2(qx^2 + qy^2))` = angle between body z and gravity, no roll/pitch decomposition or singularity | ADD | The IMU already gives a quaternion (`imu_node.cpp`); this is `R33` of the rotation matrix. It must be independent of `setImuAttitude`, which deliberately ignores readings past `slope.max_deg` |
| Three levels with debounce | state machine | ADD | OK; WARN at `warn_deg` >= 20 deg for 0.1 s: zero the velocity command and request `lie` (controlled); FALLEN at `fall_deg` >= 30 deg for 0.2 s, or free-fall (`abs(a)` < 0.3 g for 0.1 s), or "lifted" (tilt > `warn_deg` with `abs(a)` near 1 g and quiet gyro): publish `estop` true, servos limp, mode "estop" (already shown in `state` and on the web page). Re-arm by the operator only; no automatic stand-up |
| Where the numbers come from | arithmetic + `TERRAIN.md` | - | Static tip-over: feet at +-0.09 m (pitch) and +-0.115 m (0.06 hip_y + 0.055 hip_offset, roll), CoM about 0.17 m up: about 28 deg pitch, about 34 deg roll. Sim on flat floor peaks at 9 deg, 13-14 deg at 8-10 deg slopes, 19-23 deg right before failing, and `walk_check` itself fails at 20 deg. So 20 = WARN, 30 = point of no return. Tune with the logged tilt histogram of the first floor runs: set FALLEN above 2x the 99.9th percentile of flat-floor tilt but not above 30 deg (LOW-MEDIUM: derived, not measured) |
| IMU health watchdog | `imu/data` age in `locomotion_node`, `imu.required` param (true on the robot) | ADD | Tilt protection is worthless without the IMU. If `required` and the stream is older than 0.3 s while standing/walking: request `lie`, then refuse `stand`/walk until the IMU returns. Also clear `slope_valid_` after 0.5 s of silence (stale slope currently shifts the feet forever) |
| Frozen-sensor check in `imu_sensor.cpp` | N identical raw 14-byte samples | ADD | A hung or reset MPU6050 often keeps returning the same or zero data with no I2C error. Count as a read failure; also re-check WHO_AM_I and PWR_MGMT_1 (0x6B) once per second and re-run the init sequence if the chip fell back to sleep |

`imu_node` already uses `I2C_RDWR` (repeated-start combined read). `Pca9685Bus::readReg` uses two separate syscalls (write, then read); harmless for the PCA9685 (its register pointer is per chip), but use `I2C_RDWR` there too if the read-back is added.

### D. Servo current, brown-out and heat

| Technique | Status | Notes |
|-----------|--------|-------|
| INA226 on the servo V+ rail through `power_monitor_node` + `PowerGuard` (stall current, under-voltage) | HAVE | Keep. INA226 needs the 10 mOhm shunt in `power.yaml`; whether a sensor is fitted is still unconfirmed in `PROJECT.md` |
| Baseline before thresholds | EXTEND | First on-robot runs with `overcurrent_action: warn`; record `/dog/power` while standing up, standing, walking, backward; then set `overcurrent_a` = 1.3-1.5x the p99 of walking and keep `overcurrent_time` 0.5 s. The shipped 5.0 A is below the `HARDWARE.md` estimate of 6-12 A |
| Sensor required on the robot | EXTEND | `backend: i2c` (fail fast) or a `required` flag that raises a diagnostics ERROR and blocks `stand`. Today a missing INA226 just means "power monitoring off". A failed read only warns every 5 s: count consecutive failures and treat 1 s of them as sensor loss |
| Repeat the E-STOP while the condition holds (1 Hz) | EXTEND | `power_monitor` publishes `estop` once on a volatile topic; a node that restarts at that moment never sees it |
| Idle auto-relax (`idle_relax_s`, e.g. 120 s in STAND without input -> lie, 30 s more -> relax) | ADD | MG996R has no temperature or current feedback per servo. The largest avoidable heat source is holding stance for minutes. Trivial in `LocomotionController` (timer, ROS-free) |
| Thermal proxy from total current | ADD (optional) | EMA of INA226 current with tau of about 60 s, WARN/lie above a threshold set from measurement. Confirm with an IR thermometer on the case at 5/10/20 min during the first runs; acceptance = measured, not assumed (no verified servo limit in this research) |
| Power architecture (separate 5 V/3 A for the Pi, BEC for servos at 6 V with 15-20 A, 1000-2200 uF at PCA9685 V+) | HAVE (docs) | Not code. Do it before the floor run; it is what actually prevents Pi brown-outs |
| Staggered leg enable (inrush) | HAVE | `enable_stagger` 0.15 s in `ServoDriver`, keep |

### E. Telemetry and diagnostics

| Technique | Version | Status | Why |
|-----------|---------|--------|-----|
| `diagnostic_msgs/msg/DiagnosticArray` published by hand from `servo_driver`, `imu`, `power_monitor`, `locomotion`, `web_teleop` on `diagnostics` (resolves to `/dog/diagnostics`) | `diagnostic_msgs` 5.3.x on Jazzy (part of `common_interfaces`, an exec dependency of `ros_core`, hence of ros-base; verified in `ros2/variants`) | ADD | Zero new apt packages. Levels OK/WARN/ERROR/STALE. Publish at 1 Hz and immediately on a level change. Suggested names: `servo_driver/i2c` (`writes_ok`, `writes_failed`, `consecutive_failures`, `last_errno`, `write_us_max`), `servo_driver/watchdog` (`cmd_age_s`, state), `imu/health` (`age_s`, `frozen`), `power/rail` (`voltage`, `current`, `sensor_present`), `locomotion/tilt` (`tilt_deg`, level), `web/clients` (`count`) |
| Shared helper `dog_hardware/diagnostics.hpp` (about 60 lines) | C++17 | ADD | Builds a `DiagnosticStatus` from (name, hardware_id, level, message, key/value list). `hardware_id` e.g. `pca9685@i2c-0:0x40` |
| `web_teleop` subscribes to `diagnostics` and shows the worst level next to E-STOP | Python | ADD | Fixes the `CONCERNS.md` gap that a silent fallback looks like "all clear" |
| `ros2 bag record` for every floor run | `rosbag2` (`exec_depend` of `ros_base`); default storage plugin is MCAP in Jazzy (Lyrical: check `ros2 bag record --help`) | HAVE (tool) | Post-mortem of a fall or a tripped E-STOP. Suggested: `ros2 bag record -s mcap --max-cache-size 20000000 --max-bag-duration 300 /dog/joint_commands /dog/servo_pulses /dog/imu/data /dog/power /dog/diagnostics /dog/state /dog/cmd_vel /dog/estop` into a bind-mounted `./logs`. Add `--storage-preset-profile zstd_fast` if CPU allows (MEDIUM). Do not run rqt/rviz on the Pi |
| Loop-timing stats (`tick_dt_max`, deadline misses) in diagnostics | `steady_clock` | ADD | Data first: consider `SCHED_FIFO` for `servo_driver` (`cap_add: SYS_NICE`, `ulimits: rtprio`) only if these numbers show jitter under load. Not before |

### F. Gait tuning for backward walking (trot)

No library. The code path is `TrotGait::update` in `dog_control/src/gait.cpp`: the touchdown target is the symmetric Raibert-style point `neutral + 0.5 * step`, and the swing height is `z = h sin(pi s)`. `SIMULATION.md` already shows why backward is fragile: at duty 0.5 the body acts as an inverted pendulum (time constant sqrt(h/g) about 0.13 s vs a swing of 0.25 s) and drifts forward even under a backward command; duty 0.65 fixed most of it, but backward is still 75-92 % on flat floor and 0-36 % on waves/stones (`TERRAIN.md`).

| Step | Technique | Why |
|------|-----------|-----|
| 1. Instrument | Record per-leg foot xyz, joint angles and speeds, `unreachableCount()` and `ServoDriver::clampedCount()` for forward vs backward on flat floor; plot with the existing matplotlib tooling in `tools/sim_video/` | Establish whether backward swing hits the thigh/calf limits (`min_deg`/`max_deg` in `servos.yaml`), the 5.1 rad/s joint-speed budget or IK unreachable points. Knees bend backward, so backward swing folds the leg. Do not tune blind |
| 2. Swing profile | Direction-dependent lift: `z = h sin(pi s^g)` with `g` < 1 for `vx` < 0 (apex earlier, foot clears before it travels back); optionally a Bezier swing with zero vertical velocity at lift-off and touchdown (the MIT Cheetah 3 approach: Bezier swing + Raibert placement) | The current `sin` has a vertical touchdown speed of about pi h / T_swing (about 0.33 m/s for 20 mm and 0.19 s), which scuffs and slips. One-line change in `gait.cpp`, unit-testable |
| 3. Backward limits | `max_step_backward` about 0.6-0.7x `max_step`, duty +0.05 for `vx` < 0, `limits.max_vx` backward 0.07 m/s for the first floor runs (keep `TERRAIN.md`'s 30 % margin) | More four-leg support and shorter strokes trade speed for stability. Keep swing height 20 mm: `TERRAIN.md` shows 35 mm rocks the robot over |
| 4. Neutral-foot lean | `neutral.x += k_lean * vx` (small `k_lean`, sweep) | Moves the support centre relative to the CoM with velocity sign; cheap to test. A full Raibert velocity-feedback term needs a body-velocity estimate that the real robot does not have (odometry is open loop): skip |
| 5. Acceptance | At least 8 repeated `walk_check` runs per parameter point (one simulation at a time, as `TERRAIN.md` requires), judge by median and minimum: e.g. median >= 50 %, minimum >= 30 %, tilt < 20 deg, no falls | The metric is chaotic (`--backward-ratio` in CI is 0.2 because 40 % once failed at 39 %); single runs are noise. Keep CI floors as they are; add the repeated-run script to `dog_gazebo` (`terrain_sweep`-style) |
| 6. Real servos | Derate joint speed: MG996R is 0.17 s/60 deg at 4.8 V (about 6.2 rad/s) unloaded and slower under load; the gait uses 5.1 rad/s. If the first floor run shows lag, lengthen `period` from 0.55 toward 0.7 rather than raising step height | Sim caps servos at 6 rad/s with no load model |

Descending (crawl) is out of scope for the first floor run; `TERRAIN.md` says it needs contact sensing. Confidence for this section: MEDIUM-LOW (techniques are literature-standard; the size of the effect on this robot is unproven).

### G. Hardening the unauthenticated web teleop (Python standard library only)

| Priority | Technique | Where | Why |
|----------|-----------|-------|-----|
| P0 (before the floor run) | `allow_calibration` default False; enable only via launch argument `calibration:=true` used by `tools/autocal` and the calibration session | `web_teleop.py:68`, `robot.launch.py`, `teleop.yaml` | Calibration writes servo parameters that immediately move a live servo (`CONCERNS.md`, "live preview") |
| P0 | Check `Origin` on the WebSocket upgrade: allow if the header is absent (non-browser clients such as autocal) or if its host:port equals the request `Host`, else 403 | `wsserver.py` `Server._handle` | Stops cross-site WebSocket hijacking from any page opened on the same network (OWASP WebSocket cheat sheet: explicit origin check in the handshake). Browsers always send `Origin` on WS |
| P0 | Bind and firewall: keep `0.0.0.0` only if needed; add an nftables/ufw rule allowing 8080 from the operator subnet only. `network_mode: host` traffic is not bypassing host firewall rules | host | Cheap, no code (MEDIUM: rule syntax depends on the Armbian image) |
| P1 | Shared-secret token, sent as the FIRST WebSocket message (`{"type":"auth","token":...}`), compared with `hmac.compare_digest`; socket closed if no valid auth within 2 s. Token from `secrets.token_urlsafe(16)`, generated on first start into a file in a mounted directory (mode 0600), never logged; the page keeps it in `sessionStorage` after the operator types it once | `web_teleop.py`, `app.js`, `protocol.py` | The OWASP guidance is token in a header or first message, not in the URL. The token stops any host that is not the operator. Plain `ws://` still exposes it on the LAN: acceptable on a private robot network; do not add TLS now (self-signed certificate UX, extra moving parts) |
| P1 | `max_clients` (2-3), idle timeout (close after 30 s of silence, ping/pong), drive-message rate limit (about 50/s, drop the rest), reject non-UTF-8 instead of `errors='replace'` | `wsserver.py`, `web_teleop.py` | OWASP: connection caps, rate limiting, timeouts. Today an authenticated or not client can hold sockets forever |
| P1 | E-STOP policy: any client may ENGAGE; RELEASE needs the authenticated session and is refused while the driver reports a hardware fault or a tilt fault | `web_teleop.py`, servo driver latch (section B) | Combines with Tier 1 latch so a stray `estop=false` cannot release a fault |
| P2 | `cmd_vel` arbitration in `locomotion_node` (per-source topics or a priority + timeout table of about 50 lines) | `locomotion_node.cpp`, `TeleopPublisher` | Gamepad, keyboard and web all publish `cmd_vel`; the last writer wins, so an idle gamepad's zero can fight a web drive. Hand-rolled instead of `twist_mux` (not in ros-base). For the first floor run: gamepad only, `web` up with calibration off |
| ROS graph | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `docker-compose.yml` `environment` (`ROS_LOCALHOST_ONLY` is deprecated since Iron) and a non-default `ROS_DOMAIN_ID` | `docker-compose.yml` | Anyone on the LAN can otherwise publish `/dog/estop` or `ros2 param set` servo calibration. Trade-off: a laptop RViz/`ros2 topic` no longer discovers the robot; use `docker compose exec robot ...` instead, or `ROS_STATIC_PEERS`. Default RMW is Fast DDS in both Jazzy and Lyrical |

The GitHub token embedded in the `origin` remote URL (`.git/config`, file path and type only, not read here): revoke and re-issue it and switch to SSH or a credential helper. This is independent of the robot work.

### Supporting Libraries and tools (all already available)

| Library / tool | Version | Purpose | When to Use |
|----------------|---------|---------|-------------|
| `diagnostic_msgs` | 5.3.x (Jazzy), same package in Lyrical | `DiagnosticArray` | section E; the only new `find_package` |
| `launch` / `launch_ros` | ros-base | `OnProcessExit`, `ExecuteProcess` | Tier 2 watchdog |
| `rosbag2` + MCAP storage | 0.26.x (Jazzy) | Flight recorder | every floor run |
| `ament_cmake_gtest`, `launch_testing` | ros-base | Fault-injection and integration tests | sections A, B, C |
| `i2c-tools` (`i2cdetect`, `i2cget`) | apt on the Armbian host | Bring-up and post-fault checks | host only, not in the ROS image |
| AddressSanitizer + UBSan | GCC 13.3 / 15.2 | Test-time only for the new core code | PC gtest run |

### Development Tools

| Tool | Purpose | Notes |
|------|---------|-------|
| `-DCMAKE_CXX_FLAGS=-Werror` in the CI build only | Enforces "no warnings" for both distros | Not for local builds (a compiler bump on the `ros:lyrical` image should not break developers' machines) |
| `pca9685_probe check/pulse/off` | Bring-up, and Tier 2 relax command | HAVE |
| `tools/robot_setup/robot_setup.py --check` | Validates `robot.yaml`/`servos.yaml`; extend it to validate the new watchdog/tilt/power keys | HAVE |

## Installation

```bash
# No new apt or pip packages. Code changes only.

# dog_hardware and dog_control package.xml
#   <depend>diagnostic_msgs</depend>
# CMakeLists.txt (same pattern as sensor_msgs_TARGETS, builds on Jazzy and Lyrical)
#   find_package(diagnostic_msgs REQUIRED)
#   target_link_libraries(<node> ... ${diagnostic_msgs_TARGETS})

# I2C timeout, once after I2C_SLAVE, in every node that opens the bus (units of 10 ms):
#   if (::ioctl(fd_, I2C_TIMEOUT, 5) < 0) { /* log, continue */ }

# docker-compose.yml (robot service)
#   environment:
#     - ROS_DOMAIN_ID=<non-default>
#     - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
#   devices:
#     - /dev/gpiochipN:/dev/gpiochipN     # only if the Tier 3 heartbeat is built
#   volumes:
#     - ./logs:/logs                       # rosbag2 output

# Armbian host: 400 kHz bus via a user overlay that sets clock-frequency = <400000> on the
# i2c controller node used for /dev/i2c-0, then reboot and check with the write_us_max diagnostic.
```

## Alternatives Considered

| Recommended | Alternative | When to Use Alternative |
|-------------|-------------|-------------------------|
| Hand-rolled `DiagnosticArray` publisher | `diagnostic_updater` + `diagnostic_aggregator` | If the image gains `apt install ros-<distro>-diagnostic-updater` anyway (not in ros-base; verified) and you want frequency/timestamp status helpers and roll-up in RQT |
| Timer-based staleness checks | rmw QoS `deadline` / `liveliness` events | Never for the safety path: behaviour differs across rmw implementations, cannot be unit-tested in the ROS-free core |
| Kernel GPIO uAPI v2 via `linux/gpio.h` | `libgpiod` | If the Pi image pins one distro only; here Jazzy (Ubuntu 24.04, libgpiod 1.6.3) and Lyrical would need different code |
| Supervisor IC / monostable on `OE` | Dedicated MCU watchdog (`COMPUTE.md`) | When the project takes on an MCU for real-time servo timing and the IMU anyway |
| App-level retry + FAULT latch | Kernel bus recovery via DT `scl-gpios`/`sda-gpios` | If a check of the Armbian device tree shows recovery is wired up; then a failed transfer may recover by itself, but keep the FAULT policy |
| In-repo `cmd_vel` priority table | `twist_mux` | If an extra apt package in the robot image becomes acceptable |
| Token + Origin in `wsserver.py` | TLS with a self-signed cert, or SROS2 (`sros2` is in ros_core) | Robot used on untrusted networks; key management on a 2 GB Pi is a project of its own |

## What NOT to Use

| Avoid | Why | Use Instead |
|-------|-----|-------------|
| `ros2_control` `hardware_interface` watchdogs, `controller_manager` | Explicitly excluded by the "ros-base only" constraint, and heavy for 12 open-loop servos | The 3-tier watchdog above |
| `libgpiod`, `smbus2`, `pigpio`, `wiringPi`, `libi2c-dev` | Extra apt/pip packages, version skew between Jazzy and Lyrical images, wrong language for the C++ nodes | `linux/i2c-dev.h` and `linux/gpio.h` directly |
| `imu_filter_madgwick`, `imu_complementary_filter`, `robot_localization` for tilt | Not in ros-base; the repo's `AttitudeFilter` (complementary, accel gate 15 %) already yields a quaternion, and tilt needs only gravity direction | `TiltGuard` on `imu/data` |
| Docker `HEALTHCHECK` calling the `ros2` CLI | Each CLI call starts a Python process and the ROS daemon (tens of MB, seconds) on a 2 GB Pi; and Docker does not restart on "unhealthy" anyway | Diagnostics topic + web page + Tier 1-3 watchdogs |
| `websockets` / `aiohttp` / Flask for the web server | New pip dependency on Python 3.12 (Jazzy) and 3.14 (Lyrical); the stdlib server has its own tests (`test_wsserver.py`) | Harden `wsserver.py` |
| Relying on `I2C_RETRIES` for robustness | Only covers lost arbitration | Application retry + FAULT latch |
| Auto-recovery that re-stands the robot after a fall or a fault | The servos may be stalled or the legs tangled; the restart path moves legs straight to their targets | Explicit operator re-arm |
| `-Werror` in local builds | A GCC bump (13.3 on Jazzy, 15.2 on Lyrical) breaks developers | `-Werror` in CI only |
| Running heavy tools on the Pi (rviz, rqt, mapping) during floor runs | 2 GB RAM, `BUILD_JOBS=2` for a reason; OOM kill is exactly the failure Tier 3 exists for | Record a bag, analyse on the PC |

## Stack Patterns by Variant

**If no INA226 is fitted (unconfirmed in `PROJECT.md`):**
- Skip section D code, but keep the idle auto-relax, and do the first floor runs with an inline switch and an IR thermometer check on servo cases every few minutes.
- Because there is no current data, lower `max_vx` and shorten the runs.

**If the Tier 3 hardware watchdog is not built before the first floor run:**
- Tethered or short runs only (safety line on the body), Tier 1 + Tier 2 active, the physical V+ switch within reach, and `docker compose` `mem_limit`/`oom_score_adj` reviewed so the robot container is not the OOM victim.

**If the bus speed cannot be raised to 400 kHz:**
- Send the frame at 50 Hz (about 22 % of a 100 kHz bus with a 49-byte frame) and drop IMU to 50 Hz; the diagnostics `write_us_max` shows if it is enough.

**If the real robot is gamepad-only for the run:**
- Launch with `web:=false`, or with the web page up and `allow_calibration:=false`, so there is exactly one drive source.

## Version Compatibility

| Package A | Compatible With | Notes |
|-----------|-----------------|-------|
| `diagnostic_msgs` 5.3.x (Jazzy) | Lyrical (same package name in `common_interfaces`) | In `ros_core` via `common_interfaces` in both distros (verified against `ros2/variants` jazzy and rolling). `diagnostic_updater` is not part of `ros_base` (`ros_base` = `ros_core` + rosbag2, geometry2, kdl_parser, urdf, robot_state_publisher) |
| `rosbag2` 0.26.x (Jazzy) | Lyrical | MCAP is the default storage since Iron; check `--storage-preset-profile` names on Lyrical before scripting them |
| Linux i2c-dev `I2C_TIMEOUT` / `I2C_RETRIES` | any kernel | Long-stable ioctls; `I2C_TIMEOUT` is adapter-wide |
| Linux GPIO uAPI v2 | kernel 5.10+ | Armbian current/edge kernels for the H618 are newer (exact BPI-M4 Zero kernel not checked here) |
| Ubuntu 24.04 (Jazzy image) / 26.04 (Lyrical image) | GCC 13.3 / 15.2 | Lyrical released 2026-05-22, LTS to 2031; default RMW Fast DDS. `[[nodiscard]]`, `-Wall -Wextra -Wpedantic` behave the same; check `-Wunused-result` on the first build of each distro |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | Jazzy, Lyrical | Replaces deprecated `ROS_LOCALHOST_ONLY`; documented in the Jazzy "Improved Dynamic Discovery" tutorial |

## Suggested defaults (new parameters, all in YAML, single source of truth)

| Parameter | Default | File | Notes |
|-----------|---------|------|-------|
| `i2c.timeout_ms` | 50 | `servos.yaml`, `imu.yaml`, `power.yaml` | applied through `I2C_TIMEOUT` (value / 10) |
| `servo_driver.update_rate` | 50 | `servos.yaml` | was 100 |
| `servo_driver.fault.max_consecutive_frames` | 3 | `servos.yaml` | latch |
| `servo_driver.command_relax_timeout` | 2.0 s | `servos.yaml` | after the existing 0.5 s hold |
| `tilt.warn_deg` / `tilt.fall_deg` | 20 / 30 | `robot.yaml` | tune from logs |
| `tilt.warn_time` / `tilt.fall_time` | 0.1 s / 0.2 s | `robot.yaml` | debounce |
| `imu.required` | true (robot), false (sim/mock) | `imu.yaml` | blocks stand/walk without IMU |
| `power.overcurrent_a` | measured (temporary `warn`) | `power.yaml` | do not ship 5.0 A unmeasured |
| `idle_relax_s` | 120 | `robot.yaml` | lie, then relax after +30 s |
| `allow_calibration` | false | `teleop.yaml` | launch arg to enable |
| `web.max_clients`, `web.idle_timeout` | 3, 30 s | `teleop.yaml` | |

## Sources

- NXP PCA9685 datasheet, rev 4, 16 April 2015 (read directly from the PDF: MODE1 Table 5, MODE2 Table 6, Table 7 registers, Table 8, 7.4 OE, 7.5 power-on reset) - HIGH. https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
- Linux mainline `drivers/i2c/busses/i2c-mv64xxx.c` (adapter timeout `HZ`, double wait on abort, `clock-frequency` default 100 kHz, `allwinner,sun6i-a31-i2c` compatible, recovery info only with pinctrl) and `drivers/i2c/i2c-dev.c` (`I2C_TIMEOUT` sets `client->adapter->timeout`, units of 10 ms; `I2C_RETRIES`) - HIGH for mainline behaviour, MEDIUM for the Armbian kernel on this board (not checked). https://github.com/torvalds/linux
- Linux kernel `Documentation/i2c/dev-interface` - HIGH. https://www.kernel.org/doc/Documentation/i2c/dev-interface
- `ros2/variants` `ros_base` and `ros_core` package.xml, jazzy and rolling branches (ros_base = ros_core + rosbag2, geometry2, kdl_parser, urdf, robot_state_publisher; ros_core includes `common_interfaces`, `sros2`) - HIGH. https://github.com/ros2/variants
- ROS Jazzy `diagnostic_msgs` 5.3.x and `diagnostic_updater` 4.2.x package pages (versions; `diagnostic_updater` is a separate package from ros/diagnostics) - MEDIUM (index pages; not opened in full). https://docs.ros.org/en/jazzy/p/diagnostic_msgs/
- ROS 2 Jazzy "Improved Dynamic Discovery" (`ROS_AUTOMATIC_DISCOVERY_RANGE`, `ROS_LOCALHOST_ONLY` deprecated since Iron) - MEDIUM (via search snippets; page itself was blocked). https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html
- ROS 2 Lyrical Luth release: 2026-05-22, LTS to May 2031, tier 1 Ubuntu 26.04; Fast DDS remains the default RMW - MEDIUM (Open Robotics announcement and Discourse via search). https://discourse.openrobotics.org/t/ros-2-lyrical-luth-and-11-years-of-fast-dds-as-ros-2-default-middleware/55062
- rosbag2 default storage MCAP (PR #1160, Foxglove and MCAP docs), Jazzy `rosbag2_storage_default_plugins` 0.26.x - MEDIUM. https://github.com/ros2/rosbag2/pull/1160
- OWASP WebSocket Security Cheat Sheet (origin allowlist in the handshake, token/cookie authentication, message size, per-user connection caps, rate limiting, idle timeouts, wss) - HIGH for the recommendations. https://cheatsheetseries.owasp.org/cheatsheets/WebSocket_Security_Cheat_Sheet.html
- STMicroelectronics STWD100 datasheet (timeout options 3.4 ms / 6.3 ms / 102 ms / 1.6 s; open-drain or push-pull WDO) - MEDIUM (search summary; exact variant and polarity to verify on the bench). https://www.st.com/resource/en/datasheet/stwd100.pdf
- Ubuntu package data: `libgpiod-dev` 1.6.3 on 24.04 (noble) - MEDIUM. https://launchpad.net/ubuntu/noble/+package/libgpiod-dev
- I2C bus-hang and 9-clock recovery (UM10204 section 3.1.16 as cited in vendor forums), MPU6050 latch-up reports (espressif/esp-drone #105) - MEDIUM-LOW (forum/issue level). https://github.com/espressif/esp-drone/issues/105
- Bledt et al., "MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot", IROS 2018 (Bezier swing trajectories, Raibert heuristic foot placement) - HIGH for the technique, LOW-MEDIUM for its effect on this robot. https://dspace.mit.edu/bitstream/handle/1721.1/126619/IROS.pdf
- Codebase, read directly (HIGH): `ros2_ws/src/dog_hardware/src/servo_bus.cpp`, `servo_driver.cpp`, `servo_driver_node.cpp`, `power_monitor_node.cpp`, `imu_node.cpp`, `imu_sensor.cpp`; `dog_control/src/gait.cpp`, `locomotion.cpp`, `locomotion_node.cpp`; `dog_web/dog_web/web_teleop.py`, `wsserver.py`; `dog_bringup/config/{robot,power,imu,teleop}.yaml`; `docker-compose.yml`, `docker/Dockerfile`, `docker/entrypoint.sh`; `docs/{HARDWARE,CALIBRATION,TERRAIN,SIMULATION,REVIEW,DEPLOYMENT,COMPUTE}.md`.
- General practice, no single source (LOW): inline power switch as hardware E-STOP, idle auto-relax as heat control, parallel pull-up resistance check.

---
*Stack research for: servo quadruped safety and reliability, first real-floor run*
*Researched: 2026-09-29*
