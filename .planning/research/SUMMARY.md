# Project Research Summary

**Project:** RobotDog 2.0 (milestone: first stable walk on the floor of the real robot)
**Domain:** Servo quadruped (12 x MG996R on one PCA9685, MPU6050, Banana Pi BPI-M4 Zero, ROS 2 Jazzy/Lyrical in Docker arm64). Brownfield: the code exists and is proven only in Gazebo, never on hardware.
**Researched:** 2026-09-29
**Confidence:** MEDIUM-HIGH. Facts about the code, the PCA9685 datasheet and mainline Linux are HIGH. Every numeric threshold (tilt, current, timeouts, gait effect) is a MEDIUM-LOW starting point to confirm in Gazebo and on the robot.

## Executive Summary

This is not a "build a quadruped" project. Gait, kinematics, teleop and the sim harness are done. What separates the robot from the floor is a **safety and bring-up layer** that hobby servo quadrupeds usually skip and this one needs, because the servos have no feedback (no temperature, position or per-leg current) and the only stop is software. All four research files agree on the shape of the answer. Add the layer with no new runtime dependency (kernel i2c-dev, hand-rolled `diagnostic_msgs`, stdlib Python, everything stays on `ros-base`). Keep the logic in ROS-free cores tested with `MockBus` fault injection. Treat every fault as a **latch that needs an explicit operator release**. Auto-resume is the wrong default, because after any relax the driver jumps each servo to its target at full speed.

The core hazard is the same in all four files: the PCA9685 holds its last PWM forever, and every existing software protection lives in the process that can die. Confirmed in code:
- I2C write results are dropped (`servo_driver.cpp:262`, also in `relax()` and E-STOP).
- `command_timeout` only logs and holds.
- `open()` deliberately keeps stale outputs after a driver restart.
- Tilt above 25 deg is ignored.
- A missing IMU or INA226 is silent.

The recommended answer is three tiers:
1. Driver-side accounting, fault latch, watchdog relax and read-back probe.
2. Launch/entrypoint relax on driver exit.
3. A physical switch in the servo V+ line (mandatory before any floor run) and, optionally, a GPIO-heartbeat cutoff on the PCA9685 OE pin.

Tilt protection is one scalar computed from the IMU quaternion, armed by locomotion mode, with two tiers (warn -> `lie`, fallen -> latched E-STOP) and an "IMU required" rule.

Main risks:
- **(a)** Tuning the gait on unverified geometry and an ideal actuator. Fix: a desk measurement step, then a pessimistic-corner sim sweep before the gait is frozen.
- **(b)** Safety numbers that contradict each other or reality. `overcurrent_a: 5.0` conflicts with a 6-12 A walking estimate. The INA226 sits after the BEC and cannot see cell voltage.
- **(c)** New safety code that false-triggers. Examples: tilt on an unverified IMU `axes`, or the watchdog against the one-shot calibration bridge. Result: the first floor run is lost to nuisance E-STOPs.
- **(d)** Exposure by default. Web calibration is on, there is no Origin check, and `ROS_DOMAIN_ID=0` is shared with the simulator on the same LAN.

All are cheap to fix if scheduled before the floor.

## Key Findings

### Recommended Stack (detail: STACK.md)

No new package. Code and configuration only.

**Core techniques:**
- `[[nodiscard]]` on `ServoBus::setPulseUs/disable/disableAll`. CI (`-Wall -Wextra -Wpedantic`, both distros) then flags every dropped result.
- `ioctl(fd, I2C_TIMEOUT, n)` in every node that opens the bus.
  - Mainline `i2c-mv64xxx` defaults to a 1 s timeout, so one dead chip can freeze the single-threaded servo node.
  - The value is adapter-wide (10 ms units), so use the same value everywhere.
  - Files disagree on the number (20 ms vs 50 ms). Pick 20-50 ms. A 49-byte frame takes about 4.4 ms at 100 kHz.
- Whole-frame PCA9685 write, 12 channels in one auto-increment transaction, state-based, at 50 Hz.
  - Bus load drops from about 85 % to a few percent, and all outputs update at one STOP.
  - Add a read-back probe (MODE1 plus one channel round-robin), because the driver writes only on change and a static robot produces no traffic.
- Three-tier output watchdog:
  - `command_relax_timeout` (2.0 s after the existing 0.5 s hold), latched like E-STOP.
  - `OnProcessExit` -> `pca9685_probe off`, plus an entrypoint relax.
  - Physical V+ switch, plus an optional OE heartbeat via kernel GPIO uAPI v2 (not `libgpiod`).
- `TiltGuard`: `tilt = acos(1 - 2(qx^2 + qy^2))`. ROS-free, independent of the 25 deg slope gate, debounced, armed by mode. Plus IMU liveness and a frozen-sensor check.
- `diagnostic_msgs/DiagnosticArray` published by hand (helper of about 60 lines), because `diagnostic_updater` is not in `ros-base`. `ros2 bag record` for every floor run.
- Web hardening in stdlib only: `allow_calibration` default false, `Origin` check, client cap, idle timeout. Token optional.
- Non-default `ROS_DOMAIN_ID` plus `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `docker-compose.yml`.

**Do not use:** `ros2_control` watchdogs, `libgpiod`/`smbus2`/`pigpio`, `diagnostic_updater`, `twist_mux`, `websockets`/`aiohttp`, Docker `HEALTHCHECK` via the `ros2` CLI, auto-recovery that re-stands the robot, `-Werror` in local builds (CI only), heavy tools on the Pi during floor runs.

### Expected Features (detail: FEATURES.md; IDs SAF, CAL, DIA, TEL, GAIT)

**Must have (all P1):**
- **Safety**
  - SAF-01 checked I2C writes.
  - SAF-02 verified E-STOP.
  - SAF-03 single fault path (driver latch that also publishes `estop=true`; the keystone).
  - SAF-04 PWM watchdog.
  - SAF-05 tip-over reaction.
  - SAF-06 IMU-required policy.
  - SAF-07 measured power thresholds and sensor-required policy.
  - SAF-08 idle-to-rest (thermal).
  - SAF-09 physical kill in reach, as a hard gate.
- **Gait**
  - GAIT-01 backward walking on flat floor.
  - GAIT-02 statistical acceptance (8 or more repeats, median and minimum; raise CI `--backward-ratio` only after the spread is known).
  - GAIT-03 slope descent within gates.
  - GAIT-04 soft tilt derating.
  - GAIT-05 perturbed-sim gate.
- **Calibration and bring-up**
  - CAL-04 live-`set` step guard.
  - CAL-05 calibration persistence outside the repo.
  - CAL-06 calibration sanity check.
  - CAL-07 first-floor limits profile.
  - CAL-08 IMU sign gating (compensation and heading hold off until verified).
  - CAL-09 arm64 image smoke test and preflight.
  - CAL-10 standard bag command.
- **Diagnostics and teleop**
  - DIA-01 `diagnostics`.
  - DIA-02 fault banner.
  - DIA-03 real error logging.
  - TEL-02 calibration off by default.
  - TEL-03 `Origin` check.
  - TEL-04 gamepad-only floor profile.
  - TEL-06 first-floor speed caps.

**Should have (v1.x, after first hardware evidence):**
- SAF-12 arm gate (comes free with the SAF-03 latch).
- SAF-15 tiered power response.
- DIA-04/05 Pi health and loop timing.
- CAL-12/13/15 IMU self-test, wizard, live gait tuning.
- TEL-07 rumble, TEL-10 debounce.

**Defer (v2+):**
- SAF-11 hardware OE cutoff/MCU watchdog (see disagreements).
- DIA-08 per-leg current, GAIT-08 contact-aware swing, GAIT-09 stair descent.
- TEL-08 source arbitration, TEL-09 web token.
- GAIT-10 sim servo non-idealities, GAIT-11 auto-tuning.
- SAF-13 self-righting, SAF-14 thermal model.

**Anti-features:**
- TRANSIENT_LOCAL `estop` (rejected by `docs/CONTROL.md`; latch inside the driver instead).
- Per-servo temperature sensing.
- RL/MPC gait.
- SROS2/TLS.
- Stair or slope work on the real robot.
- Merging v1 `servo_config.json` into v2.
- Raising step height globally (35 mm tipped the robot in sim).
- Lowering CI backward thresholds again.

### Architecture Approach (detail: ARCHITECTURE.md)

Every detector reacts locally. It escalates over the one existing stop line, `estop` (volatile, multi-publisher, do not latch it). A small aggregator turns the escalations into one latched, operator-visible state. Latch reasons live at their owner. The aggregate goes on a single-publisher latched topic, `safety/state`. Nothing safety-relevant depends on the web UI, perception or the network.

**Major components:**
1. `ServoBus`/`MockBus`. Every method returns success and keeps `lastError()`. New `probe()`. `MockBus` gets fault injection (`failNext(n)`, `failWrites`, `probeOk`).
2. `ServoDriver` core (ROS-free, time passed as an argument).
   - Write accounting, consecutive-failure latch `HwFault {BUS, WATCHDOG, PROBE}`, output watchdog.
   - Startup policy: do not adopt running outputs by default.
   - Release is refused while the cause persists.
3. `servo_driver_node`. Publishes diagnostics and escalates a latched hardware fault as `estop=true` (handle its own echo idempotently). `mock.fail_writes` test parameters.
4. `TiltMonitor`/`SafetyMonitor` core plus `safety_monitor_node` (new `dog_safety` package).
   - Debounce, hysteresis, arming by locomotion `state`, IMU liveness, `blocked:*` reasons, release gate.
   - Publishes `estop`, `command: lie` and `safety/state` (JSON in a `std_msgs/String`, following the `perception/guard` precedent).
5. `LocomotionController` gets a small `setSafetyHold`. It refuses `stand` and walk while held; `lie` stays allowed.
6. `power_monitor_node`: rail-loss latch, `require_sensor`, repeat `estop` at 1 Hz while the condition holds.
7. `CalibrationBridge` keep-alive (5 Hz re-publish). It must ship in the same change as the driver watchdog, or calibration breaks.
8. Launch profiles, overlay and preflight:
   - `bringup` profile (calibration on, reduced limits, fall detection off) vs `run` profile (calibration off, full safety).
   - `servos.local.yaml` overlay so `git pull` cannot overwrite calibration.
   - `preflight` script.
   - Optional `calibrated:` gate.

**Test seam:** Gazebo exercises tilt, IMU liveness and `safety/state` with identical binaries. Actuator-side faults (bus, watchdog, probe) are proven only with `MockBus` and mock-backend launch tests. Do not emulate relax in the Gazebo bridge.

### Critical Pitfalls (detail: PITFALLS.md, 22 items)

1. **PCA9685 keeps the last PWM after the host dies** (SIGKILL, OOM on 2 GB, hang, Pi brown-out). `open()` also keeps stale outputs after a restart.
   - Fix: watchdog relax, `open()` clears by default, launch/entrypoint relax, and a physical V+ switch as a hard acceptance gate.
   - Verify with `kill -9`, `kill -STOP` and a Pi power pull.
2. **I2C errors are dropped, including on the E-STOP path.** A single-threaded driver also cannot hear E-STOP while blocked in `write()`.
   - Fix: check every result, retry with errno, latch after N failed frames, set `I2C_TIMEOUT`, batch the frame, add a read-back probe.
   - Consider a separate callback group for the E-STOP subscription.
3. **Power numbers are guesses and the battery is not really monitored.**
   - `overcurrent_a: 5.0` (0.5 s -> E-STOP) contradicts `HARDWARE.md` (6-12 A walking) and false-trips the first walk.
   - A 10 mOhm shunt saturates near 8 A.
   - The INA226 after the BEC never sees cell voltage.
   - INA226 presence is unconfirmed, and a missing sensor silently disables protection.
   - Fix: baseline with `warn`, then set about 1.3-1.5x the walking p99. Make the sensor required or report protection as inactive. Add a LiPo low-voltage buzzer.
4. **Tuning the gait on unverified geometry and an ideal actuator.**
   - Fix: a desk-only measurement step (calipers, scale, hip-axis convention, loaded servo speed).
   - Then a `walk_check` sweep at a pessimistic corner: servo speed 4.0/3.5 rad/s, effort 0.8 N.m, friction 0.4-0.6, 30-50 ms delay, backlash, mass +-15 %.
   - Time-box backward walking.
5. **New safety code fails silently or falsely, and exposure is the default.**
   - Tilt armed on a wrong IMU `axes` reads 90 deg at rest and E-STOPs the first stand.
   - The watchdog conflicts with the one-shot calibration `cal_pose`.
   - Power-up jerk from the unknown start pose.
   - `allow_calibration` is true, there is no `Origin` check, and `ROS_DOMAIN_ID=0` runs on a LAN that also runs the simulator (the most likely real incident).

## Implications for Roadmap

Owner's order is preserved: simulation gait, then servo and body protection, then calibration and bring-up, then floor. Research adds a cheap desk prelude to phase 1 and pulls the physical E-STOP parts order to the start, because it needs parts.

### Phase 1: Desk measurement and simulation gait
**Rationale:** The owner's first priority and no hardware risk. A desk-only measurement must precede tuning, otherwise the gait is tuned on the wrong robot (v1 link lengths disagree by up to 70 %). The gait must be final before tilt thresholds are chosen, because backward and descent changes shift the tilt distribution.
**Delivers:**
- Measured `robot.yaml` (`robot_setup --check`, `walk_check` 8/8 on it), hip-axis convention confirmed, one loaded-servo-speed measurement.
- Improved backward gait in sim: direction-aware swing, backward speed cap, small backward-only lift within 20-30 mm.
- Repeated-run acceptance script and a perturbed-sim gate.
- A real `--backward-ratio` threshold restored in CI, and soft tilt derating.

**Addresses:** GAIT-01..05.
**Avoids:** Pitfalls 8, 9, 10, 17.
**Scope:** Slope descent by backward walking is in. Stair descent and contact-aware swing are out.
**Parallel option:** Phase 2 does not touch gait code and may start in parallel if the owner allows. Only the phase 3 threshold choice must wait for the final gait.

### Phase 2: Driver-side protection (actuator layer)
**Rationale:** SAF-03 is the keystone. The driver can only receive `estop` today, and SAF-01/02/04 all need a way to tell locomotion and the UI that the driver relaxed. Build the latch once. Everything is testable with `MockBus` and mock-backend launch tests, no hardware needed.
**Delivers:**
- Bus API and errors: `[[nodiscard]]` bus API and `lastError()`, `MockBus` fault injection, errno-aware writes plus `I2C_TIMEOUT`.
- Verified E-STOP, consecutive-failure latch, `command_relax_timeout` watchdog.
- Startup policy (no adopted outputs), read-back probe, whole-frame write at 50 Hz.
- Assumed-start-pose slew on first enable, staggered PWM ON edges.
- Driver `estop` escalation, diagnostics helper and driver diagnostics.
- `OnProcessExit`/entrypoint `pca9685_probe off`.
- `CalibrationBridge` keep-alive (same change).
- `test_servo_driver_node` and safety-chain launch tests, and an ASan/UBSan run of the new core.

**Addresses:** SAF-01..04, SAF-12, DIA-01 (driver part), DIA-03.
**Avoids:** Pitfalls 1, 2, 3, 6, 14. Must not switch `estop` to TRANSIENT_LOCAL.

### Phase 3: Body protection and the operator layer
**Rationale:** Needs the phase 1 gait for tilt thresholds and the phase 2 diagnostics convention. Bundles everything that decides whether the first stand gets a false stop or a real fall is caught.
**Delivers:**
- `TiltGuard`/`safety_monitor_node`: warn -> `lie`, fallen -> latched E-STOP, IMU liveness and frozen-sensor check, `imu.required`, stale slope cleared.
- Locomotion `setSafetyHold`, `safety/state` and the web banner.
- Power policy: sensor required or reported inactive, rail-loss latch, `estop` repeated at 1 Hz, thresholds in `warn` mode until measured.
- Idle-to-rest (60-120 s -> `lie`, then relax), pulse hysteresis against buzzing, thigh soft limits inside the pulse extremes.
- Web hardening: `allow_calibration` off by default with a launch arg, `Origin` check, client cap, idle timeout.
- Non-default `ROS_DOMAIN_ID` and discovery range.
- CAL-04 live-step guard.
- CAL-07 first-floor profile: gamepad only, speed caps, IMU required, compensation and heading hold off.
- CAL-05 `servos.local.yaml` overlay.
- CAL-06 sanity check in `robot_setup --check`.

**Addresses:** SAF-05..08, DIA-01 (rest), DIA-02, CAL-04..08 (config side), TEL-02, 03, 04, 06.
**Avoids:** Pitfalls 4 (software half), 5, 7 (live step), 11, 12, 13 (config side), 16. Thresholds ship provisional and are finalized from hardware data in phases 4-5.

### Phase 4: Hardware bring-up, calibration and Docker on the Banana Pi
**Rationale:** The first time anything touches the real robot. Gate every step (DEPLOYMENT.md stages 1-8), because this is where servos get damaged. The physical kill switch has no software dependency and needs parts, so order it at the start of the milestone.
**Delivers:**
- Physical E-STOP and power verification:
  - Series switch or relay in the servo V+ line only, not the battery main.
  - Power verified: 6.0 V rail, separate 5 V for the Pi, bulk capacitor, V+ dip scoped above 5.2 V, PCA9685 VCC at 3.3 V, pull-up total measured.
- I2C bus and image:
  - I2C bus number, speed and timeout verified on the Armbian kernel (400 kHz overlay if wiring is proven).
  - arm64 image smoke test in CI and on the Pi (`/dev/i2c-0`, `i2cdetect` sees 0x40 and 0x68).
  - Volumes for logs and bags, `init: true`, cold-boot start without a restart loop.
  - `preflight` script.
- Calibration and first powering:
  - Cradle plus lab PSU with a current limit for first powering.
  - Per-channel probe.
  - Hand calibration first (autocal `--find-limits` skipped, or done on the lab supply).
  - IMU axes, sign and trim, level check under 5 deg, then `calibrated: true`.
  - The `git pull`-safe overlay in use.
  - First `/dog/power` baseline.
- Fault drills on the robot: `kill -9`, `kill -STOP`, Pi power pull, SDA unplug, tilt on the cradle, a deliberate 3 cm E-STOP drop.

**Addresses:** CAL-01/02/03/09/10/11, SAF-09 gate, SAF-07 measurement, CAL-08 verification.
**Avoids:** Pitfalls 3, 4, 5, 6, 7, 13, 15 and 18-22.

### Phase 5: Floor walking with the gamepad
**Rationale:** The acceptance criterion. Only after the gates of phases 2-4 pass.
**Delivers:**
- Tethered or spotter-assisted short runs, then free flat-floor runs at the first-floor limit profile (vx about 0.05 m/s).
- `slope.compensation` and `heading.hold` enabled one at a time, each with a rosbag. One parameter change per run, each committed.
- Rubber foot tips.
- Backward only after forward is proven, and at reduced speed.
- `overcurrent_a` and tilt `fall_deg` finalized from recorded p99. Odometry scale calibration (GAIT-07). Final soak.

**Numeric acceptance (from PITFALLS.md):**
- 10 minutes of walking in all directions on flat floor.
- No fall and no latched fault.
- Zero I2C errors.
- Servo case under 60 C (IR thermometer).
- Minimum rail voltage above 5.5 V.
- Cell voltage above 3.6 V at the end.

**Avoids:** Pitfalls 9, 10, 13, 16, 17.

### Phase Ordering Rationale
- Dependencies:
  - measurement -> gait -> tilt thresholds.
  - driver latch (SAF-03) -> everything that escalates.
  - diagnostics helper -> banner and bag topic list.
  - IMU verified (CAL-08) -> tilt guard enabled and compensation on.
  - physical switch -> any floor run.
- Grouping follows architecture. Actuator-side logic sits in `dog_hardware_core` and is proven with `MockBus`. Monitor-side logic is one small package that runs unchanged in sim and on the robot.
- The `CalibrationBridge` keep-alive is coupled to the driver watchdog. This is the only change that can break an existing workflow, so both ship together.
- Provisional-then-measured: tilt and power thresholds enter in phase 3 as configurable and non-destructive (`warn`), and are finalized from hardware baselines in phases 4-5.

### Research Flags
Needs `/gsd-plan-phase --research-phase`:
- **Phase 1.**
  - Confidence is medium-low on the size of gait effects (swing profile, backward limits).
  - It is not fully verified how to inject latency, friction, backlash and servo speed into `sim.launch.py`, `urdf.py` and `joint_command_bridge.py`.
  - Start with instrumentation (a spike), not tuning.
- **Phase 4.** Hardware unknowns with no source found:
  - H618 TWI behaviour under a short `I2C_TIMEOUT`.
  - Whether bus recovery (`scl-gpios`) is in the Armbian DT.
  - Real I2C bus numbering on the M4 Zero header.
  - Whether the PCA9685 module's OE pin is reachable.
  - INA226 presence and shunt value.
  - Bluetooth and Wi-Fi coexistence.
  - IMU vibration and DLPF behaviour on the running robot.
- **Phase 3 (partial).** Tilt threshold selection needs the final phase 1 tilt traces. This is a data task, not research.

Standard patterns (skip research-phase):
- **Phase 2.** Follows the existing `test_servo_driver.cpp` pattern. Kernel i2c-dev semantics are verified in mainline.
- **Phase 5.** The procedure is already in `docs/DEPLOYMENT.md` stages 8-12. Execute it and record results.

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | MEDIUM-HIGH | Datasheet, mainline source and `ros2/variants` read directly. Not verified on the Armbian kernel or in the built image (`diagnostic_msgs` presence, I2C timeout honoured, GPIO uAPI). Numeric defaults are proposals. |
| Features | MEDIUM | Gap analysis against code is HIGH. Peer-robot comparison is LOW-MEDIUM. The MVP boundary is a judgement, and owner decisions are open. |
| Architecture | MEDIUM-HIGH | Current-code analysis is HIGH. New-package layout, `safety/state`, tilt numbers and the OE-pin idea are MEDIUM and partly conflict with STACK.md. |
| Pitfalls | MEDIUM-HIGH | Code and datasheet facts HIGH. Servo, battery, IMU vibration and Docker-on-SBC field behaviour MEDIUM or LOW. |

**Overall confidence:** MEDIUM-HIGH on what to build and in which order. MEDIUM-LOW on any specific threshold or gait effect until measured.

### Disagreements between research files (resolved here, confirm in planning)
- **Tilt thresholds:**
  - Files disagree: STACK.md 20 warn / 30 fall; ARCHITECTURE.md 30 / 45; FEATURES.md 15-20 soft / 35-45 hard; PITFALLS.md 40-45.
  - The static tip-over limit is about 28-34 deg, so 45 deg detects a fall too late.
  - Recommendation: hard ceiling 30-35 deg, warn 20-25 deg, both YAML parameters set from the phase 1 tilt distribution. Fall above 2x the 99.9th percentile of flat-floor tilt. Warn at least 1.5x margin over the worst passing run.
- **Where tilt logic lives:** STACK.md puts `TiltGuard` in `dog_control`. ARCHITECTURE.md uses a separate `dog_safety` node.
  - Keep a ROS-free core either way.
  - Prefer the separate node: it keeps working if locomotion is wedged and gives one place for `safety/state` and the stand gate. The cost is a small build on the 2 GB Pi.
- **Hardware watchdog (OE heartbeat):**
  - FEATURES.md defers it to v2. STACK.md recommends it before untethered runs. PITFALLS.md puts a hardware cutoff in the protection phase.
  - Recommendation: the physical V+ switch is mandatory for every floor run. First runs are tethered with software tiers 1-2 active. The OE/GPIO heartbeat is required only before untethered running and can be scheduled late in phase 4. Owner decision.
- **Web token:**
  - FEATURES.md defers it. STACK.md and PITFALLS.md rate it P1 and about 20 lines.
  - Recommendation: calibration off by default, `Origin` check, client cap and a non-default ROS domain in v1. Add the token only if the web pult must stay reachable on a shared LAN.
- **Driver fault threshold:** 3 frames at 50 Hz (STACK.md) vs 10 ticks at 100 Hz (PITFALLS.md). Both are about 60-100 ms. Use one YAML parameter, default 3 frames at 50 Hz.

### Gaps to Address
- **Is an INA226/INA219 fitted, and with which shunt?**
  - It decides whether SAF-07 is real protection or documentation.
  - Every dependent rule must report `inactive` in diagnostics, never OK. Confirm physically at the start of phase 4.
  - If absent: lower `max_vx`, shorter runs, IR-thermometer checks, physical switch within reach.
- **Cell voltage is not observed** (the INA226 sits after the BEC). Add a LiPo low-voltage buzzer or a second sensor before any endurance run.
- **Fault action preference:** relax (robot drops about 0.15 m) vs controlled `lie`.
  - Recommended: `lie` for warn tiers.
  - Recommended: relax/E-STOP for dead controller, bus fault, fall and rail collapse.
  - Owner to confirm.
- **"Descending":** slope descent (in scope) or stair descent (deferred)? Recommended: slope descent by backward walking only.
- **Physical E-STOP wiring:** series switch or relay in the servo V+ line only, not the battery main (that also kills the Pi and logs). Owner's call; order parts early.
- **Should `stand` be refused until `calibrated: true`?** It is cheap and closes the first-power-up hazard. Optional, and it can be dropped without touching the rest.
- **Unverified in the built image and on the Armbian kernel:**
  - `diagnostic_msgs` in the built image: add an interface check to the `robot-image` CI job.
  - `I2C_TIMEOUT` honoured and bus speed: measure `write_us_max` on the robot.
- **Unknown, and drives whether the 5.1 rad/s gait fits:** real loaded servo speed, foot friction and vibration profile. Measure in the phase 1 desk step and in phases 4-5.
- **Independent of the robot work:** revoke and re-issue the GitHub token embedded in the `origin` remote, and switch to SSH or a credential helper. Only the file path and type were referenced; the contents were not read.

## Sources

### Primary (HIGH confidence)
- Repository, read directly:
  - `ros2_ws/src/dog_hardware/`, `dog_control/`, `dog_web/`, `dog_bringup/`.
  - `docker-compose.yml`, `docker/`, `.github/workflows/ci.yml`.
  - `docs/` (`HARDWARE`, `CALIBRATION`, `DEPLOYMENT`, `TERRAIN`, `SIMULATION`, `CONTROL`, `REVIEW`, `GAITS`).
  - `.planning/PROJECT.md` and `.planning/codebase/`.
- NXP PCA9685 datasheet rev 4 (OE, MODE2 OUTNE, power-on reset): https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
- Linux mainline `i2c-mv64xxx.c` and `i2c-dev.c` (`I2C_TIMEOUT` adapter-wide, 10 ms units): https://github.com/torvalds/linux
- `ros2/variants` `ros_base`/`ros_core` package.xml: https://github.com/ros2/variants
- OWASP WebSocket Security Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/WebSocket_Security_Cheat_Sheet.html

### Secondary (MEDIUM confidence)
- ROS 2 Jazzy Improved Dynamic Discovery (`ROS_AUTOMATIC_DISCOVERY_RANGE`), Lyrical release notes, rosbag2 MCAP default storage.
- PortSwigger cross-site WebSocket hijacking.
- Tan et al. 2018 and Jabbour et al. 2022 (sim-to-real for low-cost quadrupeds).
- MIT Cheetah 3 (Bezier swing, Raibert placement).
- Stanford Pupper calibration guide and Spot Micro README.
- MG996R datasheet summaries (stall about 2.5 A, dead band about 5 us).
- STWD100 datasheet (variant and polarity to verify on the bench); Ubuntu `libgpiod` 1.6.3 on 24.04.

### Tertiary (LOW confidence, needs validation)
- MPU6050 bus lock-up reports (esp-drone #105), Allwinner `xfer timeout` reports on Armbian forums.
- MPU6050 vibration and 50 Hz aliasing (inference from register settings).
- Petoi/Unitree snippets, real foot-friction values.
- Inline V+ switch and idle auto-relax as general practice.
- Not found anywhere: H618 TWI bus-recovery support, Banana Pi M4 Zero I2C bus numbering, MG996R loaded speed, real foot friction of this robot. These are measurements to take.

---
*Research completed: 2026-09-29*
*Ready for roadmap: yes*
