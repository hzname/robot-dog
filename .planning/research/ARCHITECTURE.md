# Architecture Research

**Domain:** Safety / fault-handling layer and real-robot calibration + bring-up flow for a ROS 2 quadruped (hobby servos on PCA9685, no joint feedback), added to an existing working node graph (subsequent milestone, not greenfield)
**Researched:** 2026-09-29
**Confidence:** MEDIUM-HIGH. Everything about the current code is HIGH (read line by line). The proposed thresholds, the diagnostic_msgs-in-ros-base assumption and the hardware-level options (OE pin, relay) are MEDIUM and are flagged where they occur.

Scope note: this file covers only what is new for the milestone "first stable walk on the floor". Gaits, teleop, perception, Gazebo checks and Docker packaging are treated as existing and are not re-researched.

---

## Verified findings about the current code (corrections and additions to `.planning/codebase/CONCERNS.md`)

CONCERNS.md was spot-checked against the code. What holds, what is missing:

| Claim / question | Verdict | Evidence |
|---|---|---|
| I2C write result is ignored | CONFIRMED | `ServoDriver::write()` (`servo_driver.cpp:258-264`) calls `bus_->setPulseUs(...)` and drops the `bool`. Same for `disableAll()` in `relax()` (`:272-277`) and `disable(old_channel)` in `setCalibration()` (`:288`). `Pca9685Bus::setPulseUs` returns `false` on a short `::write` with no errno kept (`servo_bus.cpp:147-167`). |
| No output watchdog, timeout only logs | CONFIRMED | `servo_driver_node.cpp:198-204`: after `command_timeout` it sets `timed_out_` and logs "holding position"; nothing is relaxed. Servos stay energised forever if `locomotion_node` dies while standing. |
| Tilt is ignored, no fall reaction | CONFIRMED | `LocomotionController::setImuAttitude` returns early when `abs(gp) > slope_max_deg` (25 deg) (`locomotion.cpp:224-226`); no other consumer of tilt exists in `dog_control`. |
| E-STOP is volatile and un-latched at the driver | CONFIRMED, and intentional | `estop_sub_` is `QoS(10).reliable()` in locomotion (`locomotion_node.cpp:109`), driver (`servo_driver_node.cpp:156`), power monitor, all three teleops. Documented reason in `docs/CONTROL.md` (several latched publishers arrive in undefined order). Do NOT switch it to `transient_local`. |
| Power sensor optional, silent when absent | CONFIRMED | `power_monitor_node.cpp:86-90` logs INFO and exits; `imu_node.cpp:76-80` same. A missing IMU therefore means no tilt detection, silently. |
| (new) `ServoDriver::update()` writes only when a target changed | NEW | `servo_driver.cpp:250-255` (`changed[i]`). While the robot stands or lies still there is no I2C traffic at all, so a dead bus or a browned-out PCA9685 is invisible until the next move. Fault reporting on write results alone is not enough; a periodic read-back probe is required. |
| (new) Restart leaves the PCA9685 outputs energised while the driver reports "off" | NEW | `Pca9685Bus::open` "already_running" branch (`servo_bus.cpp:110-118`) deliberately keeps the outputs; the fresh `ServoDriver` starts with `state_ = OFF` and `pulses_ = 0`, so `servo_pulses` claims 0 while the servos still hold the old pose. After a SIGKILL or a docker restart the robot can stay rigid with nobody commanding it. `docs/DEPLOYMENT.md` promises "servos de-energised at start", which this path does not deliver. |
| (new) Driver watchdog vs. the calibration channel | NEW, blocks the watchdog | `CalibrationBridge.pose()` (`calibration.py:127-136`) publishes `joint_commands` exactly once per `cal_pose`. `tools/autocal` then waits for the camera (`settle_s`, several frames). A relax-on-silence watchdog would drop the servos during measurement. `dog_bringup/scripts/calib_pose` publishes at 5 Hz and is fine. The bridge needs a keep-alive before the watchdog lands. |
| (new) Recovery after a relax snaps servos | NEW | After `relax()` state is `OFF`; the next `setTargets` puts servos in `PENDING` and jumps each to its target at full servo speed (`servo_driver.cpp:198-207`, comment "position is unknown while unpowered"). Auto-resuming after a watchdog relax would make a collapsed robot spring to the standing pose. Watchdog and fault reactions must therefore LATCH and need an operator release. |
| (new) IMU silence freezes slope compensation, does not clear it | NEW (minor) | `locomotion_node.cpp:315-317` clears only the yaw rate after 0.2 s; `slope_pitch_/roll_` in the controller keep their last values. |
| (new) Sim E-STOP is not "limp" | NEW (parity gap) | `joint_command_bridge.py` ignores `estop`; locomotion just stops publishing and Gazebo position controllers hold the last target. Limp behaviour (relax, collapse) cannot be reproduced in Gazebo. |
| (new) Mock patterns already exist for CI | REUSABLE | `imu_node` `mock.roll_deg/pitch_deg`, `power_monitor_node` `mock.voltage/current`, both changed at run time by `SetParameters` in `dog_hardware/test/test_power_monitor.py`; full mock bringup in `dog_bringup/test/test_mock_bringup.py`. `MockBus` (`servo_bus.cpp:19-42`) has no fault injection. |

---

## Standard Architecture

### System Overview

Principle: every detector reacts LOCALLY to what it can see (fast, no dependence on other nodes), then ESCALATES over the one existing stop line (`estop`), and a small aggregator turns the escalations into one operator-visible, latched state. Nothing safety-relevant depends on the web UI, the perception stack or the network.

```
                        OPERATOR / TELEOP (unchanged)
   gamepad_node/joy_teleop   keyboard_teleop   web_teleop (allow_calibration -> default OFF)
          │ cmd_vel · command · body_pose · estop ▲                     ▲ safety/state (banner)
          ▼                                        │                    │
 ┌──────────────────────────────────────────────────────────────────────┴────────────┐
 │ locomotion_node 50 Hz  [MODIFIED]                                                  │
 │  LocomotionController.request("stand") refused while safety hold is set  [NEW hook]│
 │  estop -> PASSIVE (existing)   state (latched String, existing)                    │
 └───────┬───────────────────────────────▲──────────────────────────▲────────────────┘
         │ joint_commands 50 Hz          │ state                    │ safety/state
         ▼                               │                          │
 ┌───────────────────────────────┐   ┌───┴──────────────────────────┴──────────────────┐
 │ servo_driver_node [MODIFIED]  │   │ safety_monitor_node [NEW, package dog_safety]    │
 │  ServoDriver core (ROS-free)  │   │  TiltMonitor + IMU liveness + fault aggregation  │
 │   · write-result accounting   │   │  reads: imu/data, state, estop, power,           │
 │   · consecutive-failure latch │   │         diagnostics (from driver/imu/power)      │
 │   · output watchdog (relax)   │   │  writes: estop=true (fall), safety/state (JSON)  │
 │   · startup: no adopted output│   │  release gate: estop true->false AND cond. clear │
 │   · read-back probe 5 Hz      │   └───────────────▲──────────────────────────────────┘
 │  ServoBus: Pca9685Bus | MockBus│                  │ diagnostics (1 Hz + on change)
 │   (+ lastError, probe, fault   │◀── estop ────────┤
 │    injection in MockBus)       │── estop=true ────┘ (driver self-fault escalates too)
 └───────────┬───────────────────┘
             │ I2C /dev/i2c-0 (shared with imu_node 100 Hz, power_monitor 20 Hz)
             ▼
   PCA9685 (0x40) ── OE pin [optional hw] ── servo V+ rail ── SERIES SWITCH/RELAY [hardware E-STOP]
             ▼
       12 x MG996R

 imu_node [MODIFIED: diagnostics]      power_monitor_node [MODIFIED: rail-loss latch, diagnostics,
                                        "required" flag]

 Simulation:  locomotion_node + safety_monitor_node [same binaries, one added Node in sim.launch.py]
              joint_command_bridge (no driver: watchdog/I2C paths are covered by MockBus tests only)
```

### Component Responsibilities

One responsibility per row. NEW = new code, MOD = modified existing.

| Component | Status | Single responsibility | Owns | File |
|---|---|---|---|---|
| `ServoBus` (+`Pca9685Bus`, `MockBus`) | MOD | Report the truth about I2C: every method returns success, keeps `lastError()` (errno text), adds `probe()` (read MODE1 and one channel back). `MockBus` gets fault injection (`failWrites(bool)`, `failNext(n)`, `probeOk(bool)`). | I2C errors, `I2C_TIMEOUT`/`I2C_RETRIES` on `open()` | `dog_hardware/include/dog_hardware/servo_bus.hpp`, `src/servo_bus.cpp` |
| `ServoDriver` core | MOD | Decide when the outputs must go off and refuse to power them again until released: write accounting, consecutive-failure latch, output watchdog, probe scheduling, startup policy. Time is an argument (`now`), so all of it is deterministic in gtest. | `HwFault` latch (`NONE, BUS, WATCHDOG, PROBE`), counters, `health()` snapshot | `dog_hardware/include/dog_hardware/servo_driver.hpp`, `src/servo_driver.cpp` |
| `servo_driver_node` | MOD | ROS shell: publish `diagnostics`, escalate a latched hardware fault as `estop=true`, apply `mock.fail_writes` test parameter, own `relax_timeout` / `i2c_fault_threshold` parameters. | ROS I/O only | `dog_hardware/src/servo_driver_node.cpp` (`tick()`, ctor subs, `onParams`) |
| `TiltMonitor` + `SafetyMonitor` core | NEW | Pure state machine: (attitude, gyro, stamps, mode, estop edges, health inputs) -> (fault set, estop request, release decision). Debounce, hysteresis, arming by mode, IMU liveness. | thresholds, fault latch, release rules | `dog_safety/include/dog_safety/*.hpp`, `src/*.cpp` (lib `dog_safety_core`) |
| `safety_monitor_node` | NEW | ROS shell for the core. Uses the node clock via `rclcpp::create_timer` (sim time works, same as `locomotion_node`). | subscriptions, `safety/state`, `estop` publisher | `dog_safety/src/safety_monitor_node.cpp` |
| `LocomotionController` / `locomotion_node` | MOD (small) | Refuse motion requests while a safety hold is set; nothing else. `setSafetyHold(bool)`; `request("stand"/"crawl"/"trot"/"greet"/"survey")` returns false; "lie" stays allowed. Optionally clear slope state when the IMU goes silent. | mode gating | `dog_control/src/locomotion.cpp:79 request`, `locomotion_node.cpp` (new sub `safety/state`) |
| `power_monitor_node` | MOD | Keep overcurrent -> `estop` and undervoltage -> `lie`. Add: rail collapse (below `rail_loss_v`, default 3.0 V) -> `estop` latch (so re-powering the rail cannot snap the servos); `require_sensor` parameter -> ERROR + `diagnostics` level ERROR when `backend:=auto` finds nothing on the real robot. | power events | `dog_hardware/src/power_monitor_node.cpp:108-137` |
| `imu_node` | MOD | Publish `diagnostics` (rate, read failures). It already skips publishing on read failure; the monitor derives staleness from the missing `imu/data`. | attitude | `dog_hardware/src/imu_node.cpp:93-136` |
| `CalibrationBridge` | MOD | Keep-alive: re-publish the last `cal_pose` at 5 Hz while a calibration client is connected and for `idle_hold` (default 3 s) afterwards. Must land together with the driver watchdog. | calibration session | `dog_web/dog_web/calibration.py` (`pose`, timer) |
| `web_teleop` | MOD (small) | Show `safety/state` (banner with fault names) using the same pattern as `perception/guard` (JSON String -> `protocol.py` -> WebSocket message -> `app.js`). `allow_calibration` default False. | UI only | `dog_web/dog_web/web_teleop.py:68-69`, `protocol.py`, `static/app.js` |
| `robot.launch.py` | MOD | Add `safety_monitor_node`; add argument `calibration` (default false) that sets `allow_calibration`; `servo_config` overlay order (see bring-up). | composition | `dog_bringup/launch/robot.launch.py` `_setup` |
| `sim.launch.py` | MOD (one Node) | Add `safety_monitor_node` with `sim_time`. | composition | `dog_gazebo/launch/sim.launch.py` (node list near `locomotion_node`) |
| `preflight` tool | NEW | Non-interactive pre-floor check run inside the container (CLI script, no new package). | report | `dog_bringup/scripts/preflight` (next to `calib_pose`) |
| Hardware E-STOP | NEW (hardware) | Cut servo V+ independent of any software. | the physical switch/relay | wiring, `docs/HARDWARE.md`, `docs/DEPLOYMENT.md` stage 2/8 |

Why a separate `safety_monitor_node` and not "put fall detection in `LocomotionController`": (1) it must run on the robot and in Gazebo unchanged; `dog_gazebo` launches `locomotion_node` but no hardware nodes, so the logic has to live in a package built in both places; (2) a monitor that can restart independently of the gait stack keeps working when locomotion is wedged; (3) thresholds and latch rules become a small gtest-able core instead of more state inside a 487-line controller; (4) the driver-side watchdog already covers the "locomotion died" case, so the monitor does not need to duplicate it. Cost: one more process (about 0.5 % CPU, negligible RAM) and a package to build on the 2 GB Pi (small).

Why not a full supervisor that owns every fault (single writer of `estop`): it would require changing three teleop paths, `power_monitor` and locomotion to publish `estop_request` instead, and it makes the supervisor a single point of failure for the stop line. Rejected. The stop line stays a shared, multi-publisher, volatile Bool.

---

## Recommended Project Structure

```
ros2_ws/src/
├── dog_hardware/                       # MOD
│   ├── include/dog_hardware/
│   │   ├── servo_bus.hpp               # + lastError(), probe(), MockBus fault injection
│   │   ├── servo_driver.hpp            # + HwFault, HealthSnapshot, watchdog/fault params
│   │   └── power_sensor.hpp            # + rail-loss event in PowerGuard
│   ├── src/  servo_bus.cpp servo_driver.cpp servo_driver_node.cpp power_monitor_node.cpp imu_node.cpp
│   └── test/ test_servo_driver.cpp     # + fault, watchdog, probe, release cases
│           test_servo_driver_node.py   # NEW launch test: mock.fail_writes, kill locomotion
├── dog_safety/                         # NEW package (C++, core lib + node), deps: rclcpp, std_msgs, sensor_msgs, diagnostic_msgs
│   ├── include/dog_safety/  tilt_monitor.hpp safety_monitor.hpp
│   ├── src/  tilt_monitor.cpp safety_monitor.cpp safety_monitor_node.cpp
│   ├── test/ test_tilt_monitor.cpp test_safety_monitor.cpp
│   └── CMakeLists.txt package.xml
├── dog_control/                        # MOD (small): request() gate, safety/state sub
├── dog_web/                            # MOD: calibration keep-alive, safety banner, allow_calibration default
├── dog_bringup/
│   ├── launch/robot.launch.py          # MOD
│   ├── config/  robot.yaml (+ safety: section)  servos.yaml (+ relax_timeout, i2c_fault_threshold, calibrated)
│   │            bringup_limits.yaml    # NEW overlay: low speeds for stand/first-floor stages
│   ├── scripts/preflight               # NEW
│   └── test/ test_mock_bringup.py      # keep; test_safety_chain.py NEW (launch test, own ROS_DOMAIN_ID)
└── dog_gazebo/                         # MOD: sim.launch.py (one Node), walk_check/terrain_sweep read safety/state
```

### Structure Rationale

- **`dog_safety` as its own package:** the only place that must be present in both robot and sim builds and that is neither locomotion nor hardware. Same split as every other package (ROS-free core lib + thin node, tests link the core), so it fits the CMake pattern in `dog_hardware/CMakeLists.txt`. Register it in the README package table (STRUCTURE.md rule for new packages).
- **Driver-side logic inside `dog_hardware_core`:** it must stay next to the bus, run in the same process as the I2C fd, and be testable with `MockBus` and an explicit `now`.
- **No new message package:** status is JSON in a `std_msgs/String` (existing precedent: `perception/guard`) plus the standard `diagnostic_msgs`. Avoids an interface package build on the Pi.

---

## Architectural Patterns

### Pattern 1: Local reaction, shared escalation line, latched reason

**What:** each detector cuts what it owns immediately, then publishes `estop=true` so the rest of the stack agrees, and records a reason. Release is only possible through the operator's `estop` true -> false edge, and only when the cause is gone.
**When to use:** all faults that mean "servos must not be driven now": bus fault, watchdog, tipped, rail collapse, overcurrent.
**Trade-offs:** E-STOP means limp (the robot sags onto its body, `docs/CONTROL.md` already says so). For a tipped robot that is what we want; for a mere sag in the supply a controlled `lie` (existing) is preferred. Reasons are lost if the publishing node restarts; acceptable, the robot restarts PASSIVE.

```cpp
// ServoDriver core (sketch): the latch survives estop(false) until the bus proves healthy
enum class HwFault { NONE, BUS, WATCHDOG, PROBE };
void ServoDriver::setEstop(bool active, double now) {
  if (active) { relax(); estop_ = true; return; }
  // release: refuse while the cause persists
  if (fault_ != HwFault::NONE && !bus_->probe()) { estop_ = true; return; }   // stays latched
  fault_ = HwFault::NONE; consecutive_failures_ = 0; estop_ = false;
}
```

### Pattern 2: Watchdog as a latch, never as auto-resume

**What:** `command_timeout` (0.5 s, existing) keeps meaning "hold pose, log". New `relax_timeout` (default 2.0 s, servos.yaml) means "no `joint_commands` while enabled servos exist": `relax()` + `HwFault::WATCHDOG` + `estop=true`. Resuming needs an operator release.
**When to use:** always while any servo is `ON`. In PASSIVE nothing is enabled, so it is idle by construction.
**Trade-offs:** a 2 s CPU hiccup on the Pi now costs a collapse. Choose 2.0 s to start; measure `joint_commands` gaps during the stage 9 rosbag and tighten to 1.0 s if gaps are < 100 ms. Auto-resume is rejected because of the full-speed jump in `PENDING` (see verified findings).

### Pattern 3: Write-result accounting plus read-back probe

**What:** two independent signals. (a) every `setPulseUs/disable/disableAll` result counted; `i2c_fault_threshold` (default 3) consecutive failures -> `HwFault::BUS`. (b) a 5 Hz `probe()` from `tick()` (read MODE1, compare with expected, read one channel's OFF registers vs. what was last written) -> `HwFault::PROBE` after 3 consecutive bad probes. (b) is what catches a browned-out or reset PCA9685 while the robot stands still and no writes happen.
**When to use:** always on the real bus. `MockBus::probe()` returns true unless injected.
**Trade-offs:** 5 Hz x 2 short transactions is far below the existing load (IMU 100 Hz, PCA9685 up to 12 x 100 Hz writes). Because there is one bus, set `I2C_TIMEOUT` (units of 10 ms, kernel doc) to about 2 (20 ms) and `I2C_RETRIES` to 1 in `Pca9685Bus::open`; the timeout is per adapter, so it also affects `imu_node`. Verify on the BPI-M4 Zero adapter driver (MEDIUM). A hung blocking write in the single-threaded executor also stops the watchdog, which is why the OE/relay hardware layer exists.

### Pattern 4: Mode-armed tilt detection with two tiers and a release gate

**What:** `TiltMonitor` computes body tilt as the angle between body z and gravity from the `imu/data` quaternion (orientation-agnostic combination of roll and pitch), debounced. Armed only while `state` is one of `standing_up, stand, walk, lying_down, greeting, survey` (locomotion says the robot should be on its feet); disarmed in `passive`, `lying`, `estop`, and when `safety.fall.enabled` is false (bring-up on the stand, hand-held). Tier 1 `TILT_WARN` (default 30 deg for 0.5 s): publish `command: lie` once (same pattern as the power monitor's undervoltage action). Tier 2 `FALLEN` (default 45 deg for 0.2 s, or gyro magnitude above a limit with tilt above 25 deg): publish `estop=true`, latch, report `tipped`. Release: on the operator's `estop` false edge the monitor accepts only if tilt has been below `warn` for `release_hold` (default 1.0 s); otherwise it re-asserts `estop=true` within one tick and keeps `safety/state` at fault (the glitch is harmless: locomotion is PASSIVE and refuses motion until "stand").
**Why these numbers (MEDIUM, tune from data):** slope compensation ignores readings above 25 deg (`slope_max_deg`), the Gazebo checks accept tilt < 20 deg on recommended terrain, and `walk_check` calls > 60 deg fallen (`walk_check.py:132`). 30/45 sits between "normal, tested" and "already down". Pick final values from the tilt traces `terrain_sweep` already records: require at least 1.5x margin between the worst passing run and `warn`. Redo this after the gait fixes for backward/descent, because those change the tilt distribution.
**Also required:** IMU liveness. If the IMU was seen and then goes silent > 0.5 s while armed, raise `imu_lost` (level: relax via `lie`, then estop after 3 s). If no IMU was ever seen and `safety.require_imu` is true (default true on the robot profile, false in bring-up and mock), stay in `blocked:imu_missing` so `stand` is refused.
**Mount-orientation trap:** a wrongly configured `axes:` in `imu.yaml` (DEPLOYMENT stage 7) reads 90 deg tilt at rest and would E-STOP the first stand. `preflight` must verify level tilt < 5 deg and the sign of nose-down before `safety.fall.enabled` is turned on.

### Pattern 5: Test seams through the mock backends (parity without hardware)

**What:** every fault has a parameter-driven injection point that already follows the repo's own convention (`mock.*` runtime parameters read by the node).
- `servo_driver_node`: `mock.fail_writes` (bool), `mock.fail_probe` (bool), only honoured when `backend:=mock`. Needs a branch in `onParams` before the `<joint>.<field>` parsing (names like `mock.fail_writes` are currently swallowed by the `rfind('.')` logic and ignored).
- `imu_node`: `mock.roll_deg` / `mock.pitch_deg` (exist).
- `power_monitor_node`: `mock.voltage` / `mock.current` (exist).
- Gazebo: real falls; the descent scenario that falls today (`docs/REVIEW.md` item 20) becomes the positive test.

### Pattern 6 (bring-up): Two launch profiles, calibration is a mode

**What:** the same launch file, two operating profiles. `bringup` (calibration channel on, `bringup_limits.yaml` overlay with reduced speeds, `safety.fall.enabled: false`, `safety.require_imu: false`) and `run` (default; calibration off, full safety). A calibrated flag in `servos.yaml` (`calibrated: false` until autocal `--apply` or a manual edit sets it) makes `safety/state` report `blocked:uncalibrated`, which locomotion turns into "stand refused". Calibration itself does not use locomotion (it publishes `joint_commands` through the bridge), so it is unaffected by the gate.
**Trade-offs:** one more concept for the owner; but it directly prevents the documented first-power-up hazard ("defaults estimated from v1", `servos.yaml` header) and shrinks the unauthenticated 0.0.0.0:8080 calibration surface to the sessions where it is needed. If the owner does not want the gate, it can be dropped without touching the rest.

---

## Data Flow

### Topics, QoS, rates (all under `/dog`)

| Topic | Type | Publisher(s) | Subscriber(s) | QoS | Rate | Status |
|---|---|---|---|---|---|---|
| `joint_commands` | `sensor_msgs/JointState` | locomotion, `CalibrationBridge`, `calib_pose` | servo_driver, joint_command_bridge | reliable, volatile, depth 10 | 50 Hz (locomotion, not in PASSIVE); calibration: 5 Hz keep-alive | existing; keep-alive NEW |
| `estop` | `std_msgs/Bool` | 3 teleops, power_monitor, **servo_driver (fault)**, **safety_monitor (fall)** | locomotion, servo_driver, **safety_monitor** | reliable, VOLATILE, depth 10 (do not latch) | event | existing; 2 new publishers, 1 new subscriber |
| `state` | `std_msgs/String` | locomotion | web_teleop, **safety_monitor** | reliable, transient_local, depth 1 | on change | existing |
| `imu/data` | `sensor_msgs/Imu` | imu_node / Gazebo bridge | locomotion, perception, localization, **safety_monitor** | SensorDataQoS (best effort) | 100 Hz | existing |
| `power` | `sensor_msgs/BatteryState` | power_monitor | web_teleop, CalibrationBridge, **safety_monitor** (rail state) | default reliable, depth 10 | 20 Hz | existing |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | servo_driver, imu_node, power_monitor, safety_monitor | safety_monitor, `preflight`, humans | default reliable, volatile, depth 10 | 1 Hz + immediate on level change | NEW |
| `safety/state` | `std_msgs/String` (JSON: `{"level":"ok\|warn\|fault\|blocked","faults":["tipped","bus"],"reason":"..."}`) | safety_monitor only | locomotion, web_teleop, tests | reliable, transient_local, depth 1 (single publisher, so the CONTROL.md latched-publisher problem does not apply) | on change + 1 Hz heartbeat | NEW |
| `command` | `std_msgs/String` | teleops, power_monitor, **safety_monitor** (`lie` on TILT_WARN / `imu_lost`) | locomotion | default | event | existing; 1 new publisher |
| `servo_pulses`, `joint_states` | `JointState` | servo_driver | CalibrationBridge, tools | default depth 10 | 100 Hz | existing |

`diagnostic_msgs` comes with `common_interfaces` and should be in `ros-base` (MEDIUM: verify with `ros2 interface show diagnostic_msgs/msg/DiagnosticArray` inside the built image before committing to it; fallback is a JSON String on `<node>/health`, same as `safety/state`).

### Fault flow: I2C write failure while walking

```
tick() 100 Hz ─► ServoDriver::update(now) ─► write(i) ─► bus_->setPulseUs() == false
                        │ consecutive_failures_ >= i2c_fault_threshold (3)
                        ▼
              relax() best effort (disableAll; failure only logged)  ── fault_ = BUS, estop_ = true
                        │
   servo_driver_node ──►│ publish diagnostics ERROR "i2c: <errno text>"
                        └► publish estop=true ─► locomotion -> PASSIVE, state "estop"
                                              ─► safety_monitor: safety/state fault:["bus"] (latched)
                                              ─► web banner
   operator: estop false ─► driver: probe() ok? yes -> clear ; no -> stays latched, diagnostics says why
```

### Fault flow: locomotion dies while standing

```
locomotion killed ─► no joint_commands
   t+0.5 s  driver: "holding" (existing log)
   t+2.0 s  driver: relax(), fault_=WATCHDOG, estop=true, diagnostics ERROR
   (safety_monitor, if alive: safety/state fault:["watchdog"])
 docker restart ─► fresh driver starts with outputs released (startup policy below)
```

### Fault flow: tip-over

```
imu/data (100 Hz) ─► TiltMonitor (armed by `state`) ─► > 45 deg for 0.2 s
   ─► safety_monitor publishes estop=true, safety/state fault:["tipped"]
   ─► servo_driver relax() (existing estop path, <= 1 tick), locomotion PASSIVE
   ─► operator releases estop only after the robot is set upright (release_hold 1.0 s below warn)
```

### Startup policy (driver)

`Pca9685Bus::open` keeps the outputs when the chip is already running (no twitch on a driver restart). Keep that, but bound it: the driver starts with `adopted=true`, and unless valid `joint_commands` arrive within `startup_grace` (default 1.0 s), it calls `disableAll()` and reports `servo_pulses = 0` truthfully. With the parameter `adopt_outputs: false` (default on the robot) it disables at open. This closes the "rigid robot after a crash" hole and matches what `docs/DEPLOYMENT.md` promises.

### Calibration and bring-up flow (real robot)

```
 OFFLINE (laptop, no ROS)                       ROBOT (Banana Pi, Docker)                       CI
 ───────────────────────                        ──────────────────────────                       ──
 tools/robot_setup  (form / --cli)               stage 3-4: i2cdetect, docker, mock run            robot_setup --check
   measured geometry, limits, linkages           backend:=mock  (existing test_mock_bringup)       autocal demo + pytest
      │ writes robot.yaml, servos.yaml (+.bak)   stage 5: pca9685_probe check|pulse|off            MockBus fault tests
      ▼                                          per servo, horn fitted at 1370 us
 servos.yaml  <--- single source of truth --->   stage 6: CALIBRATION on the stand
   `calibrated: false`                             launch profile bringup (calibration:=true)
      ▲                                            web_teleop /ws  ◄── autocal (laptop, camera+ArUco)
      │ autocal --apply writes locally               cal_hello / cal_pose / cal_set / cal_status
      │ (or robot: param dump)                       CalibrationBridge ── SetParameters ─► servo_driver
      │ copy back + commit (docs stage 6)                onParams ─► ServoDriver::setCalibration (live write)
      │                                                 keep-alive re-publishes cal_pose at 5 Hz (NEW)
      │                                          stage 7: IMU axes, power thresholds (imu.yaml, power.yaml)
      │                                          stage 8: safety-chain table on the stand
      │                                            ─ automated part: test_safety_chain (mock) already green in CI
      │                                            ─ manual part: E-STOP each pult, servo power cycle, stall a leg
      │                                          preflight (NEW): I2C devices, imu rate ~100 Hz, level tilt < 5 deg,
      │                                            servos.yaml calibrated, safety_monitor alive, diagnostics all OK
      └──── set calibrated: true ────────────►   launch profile run (calibration off, full safety) ─► stage 9 floor
```

Rules that make it hang together:
1. `servos.yaml` is the only calibration store. Provenance: autocal writes a header comment (date, tool version, residuals) with `calibrated: true`; the driver publishes it in `diagnostics`; `safety_monitor` turns `false` into `blocked:uncalibrated`.
2. Live `cal_set` changes are volatile (ROS parameters in the running driver). The persisted file path is: autocal `--servos-yaml` (laptop) or `ros2 param dump` on the robot, then commit. The compose file bind-mounts the repo `config/` read-only over the installed share dir, so a `git pull` on the Pi silently changes live parameters (CONCERNS). Recommended overlay: a robot-local, untracked `servos.local.yaml` (e.g. mounted from `/etc/robot-dog/`) loaded AFTER `servos.yaml` by `robot.launch.py` (later parameter files win). Calibration then survives `git pull` and rebuilds. This is a launch-file change of a few lines plus one compose volume.
3. Calibration only works with locomotion PASSIVE (existing `ALLOWED_MODES` check in `calibration.py`); E-STOP from autocal (Ctrl-C) is the existing `estop` line.
4. The `find_limits` step of autocal deliberately drives joints into stops until current rises. With `overcurrent_a` 5 A / 0.5 s the power monitor can trip E-STOP during it. Either the bring-up profile raises the threshold or `find_limits` uses the per-client `stall_amps 0.8` path (it already does) and the E-STOP is treated as an expected end of that step. Decide in the calibration phase.

---

## Build Order (dependencies)

```
 [A1] ServoBus results + MockBus injection + ServoDriver accounting/latch (gtest only)
   │
 [A2] watchdog + startup policy + probe + node diagnostics/estop escalation + launch test
   │        └─ requires [D1] CalibrationBridge keep-alive, ship in the SAME change (else calibration breaks)
   │
 [B1] dog_safety core (TiltMonitor/SafetyMonitor gtests)  ── independent of A, can run in parallel with A1
   │
 [B2] safety_monitor_node + sim.launch.py + robot.launch.py + `safety/state` + locomotion gate + web banner
   │        └─ needs A2 only for diagnostics aggregation; the tilt path works on the existing estop chain
   │
 [B3] thresholds from sim data (terrain_sweep traces) + Gazebo checks (no false latch on passing terrain,
   │      one positive tip-over) ── do AFTER the gait fixes for backward walking / descent, since those change tilt
   │
 [C]  power_monitor rail-loss latch + require_sensor, imu_node diagnostics, IMU-required rule   (needs A2 diagnostics convention, B2)
   │
 [D2] launch profiles (calibration:=), bringup_limits.yaml, servos.local.yaml overlay, calibrated flag + gate, preflight   (needs B2 for the gate)
   │
 [E]  hardware: series switch/relay in servo V+ (must exist before ANY floor run), optional OE + GPIO; stage 8 manual acceptance on the stand
   ▼
 stage 9 floor run
```

Notes for the roadmap:
- Owner order (gait in sim, then protection, then calibration + Docker, then floor) fits: A1/A2/B1 can start before gait work finishes because they do not touch gait code; only B3 (threshold choice) must wait for the final gait.
- A2 is the only step that can break an existing workflow (calibration). Ship D1 with it.
- E (the physical E-STOP) has no software dependency; it needs a decision from the owner about the wiring (see Open questions) and should be scheduled early because it needs parts.

---

## Test strategy (CI with no hardware)

| Level | What | Where | Notes |
|---|---|---|---|
| 1. Pure unit (gtest, no ROS) | `ServoDriver` with `MockBus`: failed write counting, latch after N, no writes while latched, `setEstop(false)` refused while `probe()` fails and accepted after, watchdog relax at exactly `relax_timeout` using the explicit `now` argument, startup grace, `setCalibration` write failure. `TiltMonitor`: debounce, hysteresis, arming by mode, release gate, IMU liveness. | `dog_hardware/test/test_servo_driver.cpp` (extend), `dog_safety/test/*` | Fast and deterministic; time is a parameter in both cores. The existing `DriverTest` fixture is the model (`test_servo_driver.cpp:113-180`). |
| 2. Register-level | `Pca9685Bus::prescaleFor/ticksFor` already pure; add a loopback/fake-fd test for `writeChannel` byte layout (CONCERNS gap). Optional: `probe()` parsing via an injectable fd. | `dog_hardware/test` | Medium priority. Do not block A1 on it. |
| 3. Node integration (launch_test) | Full mock bringup: `imu mock.pitch_deg=60` via `SetParameters` -> `estop` true, `state=estop`, all `servo_pulses` 0, `safety/state` has `tipped`; back to 0, publish `estop=false` -> recovers; `mock.fail_writes=true` while walking -> fault + estop; SIGKILL/terminate `locomotion_node` -> `servo_pulses` all 0 within `relax_timeout` + margin; `stand` refused while blocked. | `dog_bringup/test/test_safety_chain.py` NEW, plus `dog_hardware/test/test_servo_driver_node.py` NEW | Use the same style as `test_power_monitor.py` (SetParameters client) and `test_mock_bringup.py`. Give each test its own `ROS_DOMAIN_ID` (41 is taken by the power monitor test); use generous timeouts (CI is slow). |
| 4. Gazebo (existing `terrain` / `simulation` jobs) | `walk_check` and `terrain_sweep` additionally subscribe `safety/state` and fail if a fault latches on any passing scenario (false-positive guard). One positive case: the known-falling descent or a spawn with a large pitch, expecting `tipped` before the robot lies flat. | `dog_gazebo/dog_gazebo/walk_check.py`, `terrain_sweep.py`; CI `terrain` job | Keep it to one positive case; the job is timing-sensitive (CONCERNS). Reuse per-step `ROS_DOMAIN_ID`/`GZ_PARTITION`. |
| 5. Tools | `protocol.py` message for the safety banner (pytest, no ROS); autocal keep-alive behaviour against a fake WS server (`tools/autocal/tests`); `calibration.py` keep-alive timer with a stub node (closes a listed gap: no test for the bridge). | `dog_web/test`, `tools/autocal/tests` | |
| 6. Image | `docker run robot-dog:2.0 ros2 launch dog_bringup robot.launch.py backend:=mock` smoke test in the existing `robot-image` job, plus `ros2 interface show diagnostic_msgs/msg/DiagnosticArray` | `.github/workflows/ci.yml` | The job is non-PR and QEMU-slow: keep the smoke to a `ros2 pkg executables` and an interface check, not a full run. Closes the "CI never starts the image" concern cheaply. |

Simulation/real parity statement: the safety logic that can be exercised in Gazebo (tilt, IMU liveness, `safety/state`, locomotion gating, web banner) uses identical binaries in sim and on the robot. The actuator-side logic (bus faults, watchdog relax, probe) exists only with `servo_driver_node` and is proven with `MockBus`; Gazebo cannot show a limp robot (position controllers hold). Do not spend effort emulating relax in `joint_command_bridge`; at most have the bridge drop commands while `estop` so sim and robot agree on "no commands accepted".

---

## Scaling Considerations

The "scale" here is load on a 2 GB Cortex-A53 and one shared I2C bus, not users.

| Concern | Now | With this layer | Adjustment |
|---|---|---|---|
| I2C bus load (`/dev/i2c-0`) | IMU 100 Hz, PCA9685 up to 12 writes x 100 Hz (each a 5-byte transaction), INA 20 Hz | + probe 5 Hz (about 3 short transactions) | Negligible. Batch-write all 12 channels with PCA9685 auto-increment (already enabled in MODE1) later if `read failed` appears in stage 12; bus speed is unspecified today (`docs/REVIEW.md` item 12): set 400 kHz overlay only after wiring is proven. |
| CPU | locomotion 50 Hz, servo_driver 100 Hz publishing two JointStates, web ~8 % of a PC core | + safety_monitor 100 Hz callback (trivial math), diagnostics 1 Hz | Well under 1 %. Perception stays off (no sensors). |
| Memory | 2 GB, `BUILD_JOBS=2` | + one small package | Build with the existing Dockerfile flags; nothing else. |
| Failure blast radius | one process holds the bus fd | unchanged | Only `servo_driver_node` opens the PWM path. `pca9685_probe` must never run while the stack runs (calibration docs already stop the stack first). |

---

## Anti-Patterns

### Anti-Pattern 1: Latching E-STOP with `transient_local` to make it "safe"

**What people do:** make `estop` durable so late joiners see it.
**Why it's wrong:** several publishers (3 teleops, power monitor, plus the two new ones) each keep their own last value; a late subscriber receives them in undefined order and can see `false` last, releasing the stop (documented in `docs/CONTROL.md`).
**Do this instead:** keep `estop` volatile; make each node start safe; latch reasons at their owner; publish the aggregate on a single-publisher latched topic (`safety/state`).

### Anti-Pattern 2: Auto-resume after a watchdog or fault relax

**What people do:** re-enable outputs as soon as commands or the bus come back.
**Why it's wrong:** unpowered servos have unknown position and `ServoDriver` jumps to the target at full speed after a relax; the collapsed robot would spring up.
**Do this instead:** latch, require the operator's `estop` release plus a passing `probe()`, then a fresh "stand".

### Anti-Pattern 3: Detecting bus faults only from write results

**What people do:** count failed writes and call it done.
**Why it's wrong:** `update()` writes only on change, so a static robot produces no writes; the PCA9685 can reset silently on a supply glitch while the driver believes it holds.
**Do this instead:** periodic read-back probe plus write accounting.

### Anti-Pattern 4: Fall detection that is always armed or has hard-coded axes

**What people do:** trigger on `abs(pitch) > x` at all times.
**Why it's wrong:** false E-STOPs while the robot is hand-held on the stand or lying, and at first power-up when IMU `axes:` is wrong.
**Do this instead:** arm from `state`, use total tilt from the quaternion, gate `safety.fall.enabled` on the `preflight` orientation check, keep `enabled: false` in the bring-up profile.

### Anti-Pattern 5: A second configuration system

**What people do:** revive the untracked `robot_dog_ws/` / `servo_config.json` reader for calibration persistence.
**Why it's wrong:** it competes with `servos.yaml`/`robot.yaml` (already flagged in CONCERNS and out of scope in PROJECT.md).
**Do this instead:** overlay YAML (`servos.local.yaml`), same parameter mechanism.

### Anti-Pattern 6: Relying on software for the last line of defence

**What people do:** treat the watchdog and E-STOP as sufficient.
**Why it's wrong:** a SIGKILL leaves the PCA9685 driving its last pulses (the chip has no watchdog); a blocked I2C write in the single-threaded executor also blocks the software watchdog.
**Do this instead:** the physical switch/relay in the servo V+ line is the actual E-STOP; software is the convenient one.

---

## Integration Points

### External / hardware

| Element | Integration | Notes |
|---|---|---|
| Servo V+ rail switch or relay (hardware E-STOP) | In series with the 6 V BEC output to PCA9685 V+, NOT in the battery feed (the DEPLOYMENT stage 2 diagram puts the main switch before both converters, which also kills the Pi) | Keep the Pi alive so logs survive and `power_monitor` sees the rail collapse. Software sees it only through the INA226 (optional today). With a rail-loss latch (component `power_monitor`, phase C) a re-powered rail cannot snap the servos. Confidence HIGH on the reasoning, wiring is the owner's call. |
| PCA9685 OE pin (optional) | OE is active LOW; with OE HIGH the outputs go to the state selected by `MODE2.OUTNE` (00 = driven low, i.e. no pulse). Idea: external pull-up on OE (outputs disabled), a Pi GPIO held LOW by the driver process through libgpiod enables them; when the process dies the kernel releases the line and the pull-up disables the outputs, which covers SIGKILL and container death without a microcontroller. | MEDIUM. Needs a free GPIO pin on the 40-pin header, libgpiod in the image and `/dev/gpiochip*` in compose; the existing `Pca9685Bus` already sets `MODE2 = OUTDRV` with OUTNE=00. Does not cover a hung-but-alive process. Whether the module's OE pin is accessible is unknown. Treat as optional, decide in phase E. |
| INA226 / INA219 | `power_monitor_node` | Not confirmed present (PROJECT.md). Every rule that depends on it (rail loss, overcurrent E-STOP) must be reported as "inactive" in `diagnostics`, never as "ok". |
| Linux i2c-dev | `ioctl(I2C_TIMEOUT)` (10 ms units), `I2C_RETRIES` | Set in `Pca9685Bus::open`; adapter-wide (affects IMU too). |

### Internal boundaries

| Boundary | Communication | Notes |
|---|---|---|
| teleop -> locomotion | existing topics | unchanged; `state` "estop" already tells the UI |
| locomotion <-> driver | `joint_commands` (50 Hz) and `estop` | The watchdog relies on `joint_commands` continuity: locomotion publishes in every mode except PASSIVE (`locomotion.cpp:327-328`), so PASSIVE means "no servo enabled" and the watchdog is idle |
| driver -> safety_monitor | `diagnostics` (level, keys: `i2c_errors`, `consecutive_failures`, `fault`, `calibrated`, `pulses_active`) | monitor filters by `name == "servo_driver"` |
| imu_node -> safety_monitor | `imu/data` best effort | monitor's liveness timer, not a QoS deadline (works in sim) |
| safety_monitor -> locomotion | `safety/state` (latched JSON) | `LocomotionController::setSafetyHold(bool, reason)`; fail-open if the topic has never been received (sim harnesses, unit runs); the launch files always start the monitor and `preflight` checks it |
| web_teleop -> CalibrationBridge | in-process | keep-alive timer owned by the bridge; started on `cal_pose`, cancelled on disconnect after `idle_hold` |
| CalibrationBridge -> servo_driver | `SetParameters` service (existing) | live `setCalibration` writes a powered servo immediately (CONCERNS "fragile"); keep the `passive`-only gate |
| autocal -> robot | WebSocket `cal_*` | unauthenticated; `allow_calibration` default false + launch arg is the mitigation in this milestone; token/Origin check is a separate decision (PROJECT.md: undecided) |
| sim.launch.py | one added `safety_monitor_node` with `use_sim_time` | no driver in sim |

### Modified-code integration map (where exactly)

| Change | File and function |
|---|---|
| Check write results, count, latch | `servo_driver.cpp` `ServoDriver::write` (258), `relax` (272), `setCalibration` (279), `update` (220) |
| Probe, lastError, I2C timeout | `servo_bus.cpp` `Pca9685Bus::open` (82), `writeChannel` (155), new `probe()`; `servo_bus.hpp` interface; `MockBus` (19-42) |
| Watchdog tier | `servo_driver_node.cpp` `tick()` (198) currently only sets `timed_out_`; move the decision into `ServoDriver::update(now)` and keep the log |
| Escalation and diagnostics | `servo_driver_node.cpp` ctor (publishers next to `pulse_pub_`), `tick()` |
| Test parameters | `servo_driver_node.cpp` `onParams` (217): handle `mock.*` before the joint/field split |
| Startup policy | `servo_bus.cpp:110-118` (`already_running`) and driver ctor |
| Safety hold in the controller | `locomotion.cpp` `LocomotionController::request` (79): early return for motion commands; `locomotion.hpp`: `setSafetyHold` |
| Locomotion subscription | `locomotion_node.cpp`: new `safety/state` sub next to `estop_sub_` (109); optional: clear slope state when IMU silent (315-317) |
| Rail loss / require sensor | `power_monitor_node.cpp` `tick()` (121-137), ctor sensor probe (86-90); `power_sensor.hpp` `PowerGuard` |
| Keep-alive | `dog_web/calibration.py` `CalibrationBridge.pose` (127), new timer |
| Banner | `dog_web/web_teleop.py` (subscribe like `_on_guard`), `protocol.py`, `static/app.js` |
| Launch | `robot.launch.py` `_setup` (add node, `calibration` arg, overlay file order); `sim.launch.py` node list (near line 106-110) |
| Config | `robot.yaml`: new `safety:` section (`fall.enabled`, `fall.warn_deg`, `fall.fallen_deg`, `fall.debounce_s`, `fall.release_hold_s`, `require_imu`); `servos.yaml`: `relax_timeout`, `i2c_fault_threshold`, `startup_grace`, `adopt_outputs`, `calibrated`. Add checks to `tools/robot_setup/robot_setup.py --check` so a bad `safety:` block fails CI. |

---

## Open questions for the owner (decisions that change the plan)

1. Hardware E-STOP: is a series switch or relay in the servo V+ line acceptable (and is the OE pin of the PCA9685 module reachable)? Without it the software watchdog cannot cover SIGKILL or a hung driver. Decide before phase E.
2. Is an INA226/INA219 installed? If not, rail-loss and overcurrent protection are absent and the plan must say so; `preflight` will report it.
3. Should `stand` be refused until `calibrated: true` (Pattern 6)? Cheap, prevents the documented first-power-up hazard; optional.
4. Is an E-STOP on tip-over (limp) acceptable as the default reaction, or should a first tier `lie` be tried whenever tilt is below the hard threshold? Proposed: both (warn -> `lie`, fallen -> estop).
5. Does the calibration channel stay available on the robot at run time (LAN-exposed, unauthenticated)? Proposed default: off, enabled per session.

## Sources

- Local code, read directly (HIGH): `ros2_ws/src/dog_hardware/src/servo_driver.cpp`, `servo_driver_node.cpp`, `servo_bus.cpp`, `imu_node.cpp`, `power_monitor_node.cpp`, `include/dog_hardware/servo_driver.hpp`, `servo_bus.hpp`; `ros2_ws/src/dog_control/src/locomotion.cpp`, `locomotion_node.cpp`, `include/dog_control/locomotion.hpp`; `ros2_ws/src/dog_web/dog_web/calibration.py`, `web_teleop.py`; `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py`, `launch/sim.launch.py`; `ros2_ws/src/dog_bringup/launch/robot.launch.py`, `test/test_mock_bringup.py`; `ros2_ws/src/dog_hardware/test/test_power_monitor.py`, `CMakeLists.txt`.
- Project docs (HIGH for what the project states): `.planning/PROJECT.md`, `.planning/codebase/ARCHITECTURE.md`, `STRUCTURE.md`, `CONCERNS.md` (each claim above marked confirmed or corrected against code), `docs/CALIBRATION.md`, `docs/DEPLOYMENT.md` (stages 2, 5-9, 12), `docs/CONTROL.md` (safety chain, why `estop` is volatile), `tools/autocal/README.md`, `tools/robot_setup/README.md`, `docker-compose.yml`, `docker/Dockerfile`, `.github/workflows/ci.yml`.
- PCA9685 datasheet, OE pin and `MODE2.OUTNE` behaviour (HIGH for the datasheet statement, MEDIUM for the GPIO-hold design built on it): [NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf), summary via [NXP product page](https://www.nxp.com/products/power-drivers/lighting-driver-and-controller-ics/led-drivers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-driver:PCA9685).
- Linux i2c-dev `I2C_TIMEOUT` (10 ms units, adapter-wide) and `I2C_RETRIES` (HIGH): [torvalds/linux drivers/i2c/i2c-dev.c](https://github.com/torvalds/linux/blob/master/drivers/i2c/i2c-dev.c), [uapi i2c-dev.h](https://github.com/torvalds/linux/blob/master/include/uapi/linux/i2c-dev.h).
- Not verified (MEDIUM/LOW): that `diagnostic_msgs` is present in the `ros:jazzy-ros-base` / `ros:lyrical-ros-base` images (no ROS installation on the research machine; check in CI); the behaviour of the BPI-M4 Zero I2C adapter driver under a short `I2C_TIMEOUT`; the numeric tilt thresholds (must be tuned from `terrain_sweep` traces and the first rosbags).

---
*Architecture research for: safety / fault handling and real-robot calibration and bring-up of RobotDog 2.0*
*Researched: 2026-09-29*
