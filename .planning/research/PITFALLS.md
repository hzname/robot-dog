# Pitfalls Research

**Domain:** Hobby-servo quadruped (12 x MG996R on one PCA9685, MPU6050, Banana Pi BPI-M4 Zero, ROS 2 in Docker) taking its first floor walk, after being developed only in Gazebo
**Milestone:** first stable floor walk on the real robot (subsequent milestone, existing code base)
**Researched:** 2026-09-29
**Overall confidence:** MEDIUM-HIGH. Code facts and the PCA9685 datasheet are HIGH (read directly). Servo/battery/IMU field behaviour is MEDIUM or LOW (community sources, physics estimates); each item is tagged.

**Confidence tags used below**
- [HIGH] verified in this repo's code/config, or in the NXP PCA9685 datasheet fetched this session
- [MEDIUM] two or more independent sources, or repo docs plus a first-principles calculation
- [LOW] single web source or inference; treat as a hypothesis to test on the robot

**Phase names used for mapping** (the owner's order):
- **A. Sim gait** - improve the gait in Gazebo (backward, descent)
- **B. Protection** - servo/body protection before any floor run
- **C. Bring-up** - servo calibration on the real robot + Docker on the Banana Pi
- **D. Floor** - walking on the floor from the gamepad
- **A0 (recommended, new)** - a desk-only measurement step (geometry, masses, servo speed under load) that costs no risk and should come before or at the start of A. See Pitfall 8.

---

## Critical Pitfalls

### Pitfall 1: PCA9685 keeps driving the last PWM after the host process (or the whole Pi) dies

**What goes wrong:**
The PCA9685 has its own oscillator. Once a pulse is programmed it keeps emitting it with no further I2C traffic. If `servo_driver_node` is killed (SIGKILL, OOM kill on a 2 GB board, segfault, container `kill`), hangs, or the Pi browns out and reboots, all 12 servos stay energised and hold the last pose, blind to everything, including E-STOP. The datasheet says the chip is reset only by VDD dropping below 0.2 V; a Pi reboot that leaves the 3.3 V rail up does not reset it [HIGH: NXP PCA9685 datasheet, section 7.5].

Verified in code [HIGH]:
- `relax_on_exit` runs only in the `ServoDriverNode` destructor (`servo_driver_node.cpp`), i.e. only on a clean exit.
- On command timeout `tick()` only logs "holding position" and keeps the servos powered.
- `Pca9685Bus::open()` has an `already_running` branch that deliberately does **not** call `disableAll()`: after a driver restart the stale pulses are still being emitted while the driver believes every servo is `OFF`.
- `robot.launch.py` has no `respawn`/`on_exit` handling, so if only the driver dies the rest of the stack keeps running.
- `docker-compose.yml` uses `restart: unless-stopped`, so a crashed container comes back on its own, with the stale pose still held.

**Why it happens:**
Software-only safety chain (deadman, timeouts, E-STOP) all live in the process that just died. The v1 idea "do not reset the chip so a restart does not twitch the legs" was reasonable for a bench but is the wrong default for a robot.

**Consequences:**
A leg stuck in mid-stride pushing against the floor or the body: stall current ~2.5 A per servo, cooked servos, stripped gears, a robot that will not fall over but cannot be stopped without cutting power by hand.

**How to avoid:**
1. Driver-level output watchdog: no `joint_commands` for 2-3 s means `disableAll()` (relax), not "hold". Keep the existing 0.5 s hold for short gaps, escalate to relax after the long timeout.
2. `open()` must clear a running chip by default (`disableAll()` when the driver's state is OFF); make "adopt running pose" an explicit opt-in parameter for bench work only.
3. Hardware cutoff (cheap, decisive): put servo V+ behind a MOSFET or relay whose gate has a pull-down and is driven by a GPIO line the driver holds open (libgpiod releases the line when the process dies, so the rail drops). Alternative: pull OE up on the PCA9685 and let the Pi drive it low only while healthy; with the default OUTNE=00 that forces all outputs LOW = no pulse = limp [HIGH: datasheet Table 11, MODE2 OUTNE]. Note the common Adafruit-style module has OE pulled *down*, so this needs a resistor change [MEDIUM: Adafruit guide]. Add the SoC hardware watchdog (`/dev/watchdog`) so a kernel hang also drops the GPIO.
4. Also fit a plain physical switch in the servo V+ line only (not the battery main): DEPLOYMENT.md stage 0 has a fuse and switch in battery "+", which also kills the Pi and its logs.
5. Do **not** copy CONCERNS.md's suggestion to make E-STOP TRANSIENT_LOCAL: `locomotion_node.cpp` has a comment that volatile E-STOP is deliberate (several latched publishers arrive in undefined order at a late subscriber). Latch the fault inside `servo_driver` (a local `fault_` flag cleared only by an explicit release) and rely on the hardware cutoff for the process-dead case.

**Warning signs:**
- After `docker kill -s KILL robot_dog` the legs stay stiff instead of going limp.
- After a Pi power-cycle with servo power on, legs twitch or hold a pose for the ~20-30 s of boot.
- `docker compose restart` mid-walk: legs release only if SIGINT reached the driver in the 10 s grace period.

**Test to require (Phase B exit):** SIGKILL the container; `kill -STOP` the driver process (hang, not death); pull Pi power with servo power on; each must end with servos limp within the specified time.

**Phase to address:** B (software watchdog, open() behaviour, hardware cutoff); verify again in C with real servos.

---

### Pitfall 2: I2C write errors are dropped, so commanded pose is not physical pose, and the E-STOP path itself is unchecked

**What goes wrong:**
`ServoDriver::write()` calls `bus_->setPulseUs(...)` and discards the bool; `Pca9685Bus::writeChannel` returns false on a short `::write` with no errno logging, retry or counter [HIGH: `servo_driver.cpp:262`, `servo_bus.cpp:155-168`]. `joint_states` republishes the commands, so nothing downstream can tell a leg has stopped following. Worse, `relax()` calls `bus_->disableAll()` and ignores its result too: an E-STOP during a bus fault clears the driver's state but may not have reached the chip, so the driver reports "servos off" while they are still energised.

**Why it happens:**
The `MockBus` used by all tests never fails. The real `Pca9685Bus` has never been run against a device (CONCERNS.md test-coverage gap, confirmed).

**Consequences:**
One leg silently frozen or wandering while the gait continues: the robot lurches and falls, or a servo stalls against the floor. Also a wrong belief that "E-STOP worked".

**How to avoid:**
- Check every return value; log `errno` (`EREMOTEIO`, `ETIMEDOUT`, `EBUSY`) on the first failure and throttled afterwards; keep per-channel and total error counters; publish them (diagnostics or a field on `servo_pulses`).
- One retry inside the tick, then after N consecutive failures (start with 10 = 100 ms at 100 Hz) latch a fault: E-STOP + explicit release needed.
- `disableAll()` failure must retry (3x) and fall back to per-channel `disable()`, then escalate to the hardware cutoff of Pitfall 1.
- Add a loopback/fake `/dev/i2c` test for the failure paths (`prescaleFor`/`ticksFor` are already pure and testable).

**Warning signs:** legs twitching without a command; `servo_pulses` non-zero for a limp leg; `read failed`/`write failed` lines in the log during Stage 12 soak (DEPLOYMENT.md says there must be none, but today there is no `write failed` line at all: the code cannot print one).

**Phase to address:** B (implementation); C (error counter must read 0 over a 30-minute soak).

---

### Pitfall 3: A stuck I2C bus, and a single-threaded driver that cannot hear the E-STOP while it waits

**What goes wrong:**
MPU6050 is known to hold SDA low after a transaction is interrupted mid-read (Docker restart, SIGKILL, a Pi reset while the sensor stays powered) [LOW: esp-drone issue #105; generic I2C lock-up literature, MEDIUM for the nine-clock recovery mechanism]. When that happens every device on the bus, including the PCA9685, stops answering until SDA is released. The three I2C users are separate processes (`servo_driver`, `imu`, `power_monitor`) on one adapter; a transfer that waits for the adapter timeout blocks the others too. `ServoDriverNode` runs its 100 Hz timer, the `joint_commands` callback and the `estop` callback on **one** thread, so a blocked `write()` also delays E-STOP handling.

**Why it happens:**
No `I2C_TIMEOUT`/`I2C_RETRIES` ioctl is ever set; `Pca9685Bus::open` only sets `I2C_SLAVE`. Linux i2c-dev takes the timeout in units of 10 ms [MEDIUM: kernel patch discussion "Clarify the unit of ioctl I2C_TIMEOUT"; i2c-dev.c]. Whether the Allwinner H618 TWI driver in the Armbian kernel implements generic bus recovery is unverified [LOW].

**Consequences:**
Whole robot frozen with servos powered (see Pitfall 1), or E-STOP delayed by up to the adapter timeout. On the bench it presents as "the bus died after a container restart and only a power cycle fixes it".

**How to avoid:**
- Set `I2C_TIMEOUT` to the minimum (1 = 10 ms) and `I2C_RETRIES` on every fd; verify the kernel honours it on this adapter.
- Give the E-STOP its own path: a separate callback group/thread in `servo_driver` so the subscription can call `disableAll()` (and the GPIO cutoff) without waiting behind a stuck timer callback.
- Power the IMU from a GPIO-switchable rail or via a transistor if the bus recovery cannot be done in software; at minimum write a bench script that bit-bangs nine SCL pulses (or `i2cdetect` after a re-plug) so a stuck bus is diagnosable in one minute.
- Always shut the IMU/power nodes down cleanly (`stop_signal: SIGINT` is already set; keep it) and never "docker kill" the stack during Stage 7-12 tests.

**Warning signs:** `i2cdetect -y 0` shows nothing or `--` for 0x68/0x40 right after a restart; all three nodes log read/write failures at the same moment; `dmesg` shows `xfer timeout` lines from the sunxi/mv64xxx I2C driver [LOW: Armbian forum reports of `sunxi_i2c_do_xfer ... xfer timeout` on other H-series boards].

**Phase to address:** B (timeouts, threading, error handling); C (reproduce a stuck bus deliberately once, learn the recovery).

---

### Pitfall 4: Power sag and brownout: the battery is not actually monitored, and the thresholds are guesses

**What goes wrong:**
- A stalled MG996R pulls about 2.5 A; 12 of them plus the Pi on one LiPo can exceed the BEC or the pack's capability [MEDIUM: HARDWARE.md, plus web reports of servo brownouts and Pi reboots].
- The current/voltage sensor (INA226) sits between BEC and PCA9685 V+ (HARDWARE.md, DEPLOYMENT.md stage 2). It sees the regulated 6 V rail, which stays at 6.0 V until the pack is nearly empty. So DEPLOYMENT.md stage 12 ("run until the lie-down at low voltage or 3.5 V per cell") cannot work as written: the software never sees the cell voltage, and by the time the rail sags the pack is over-discharged. `undervoltage_v: 5.0` on a 6 V rail is also below the MG996R's 4.8 V floor plus margin: torque and speed are already degraded before it triggers.
- The sensor is optional: `power_monitor` exits silently when no chip answers, so a missing INA226 quietly disables both protections [HIGH: `power_monitor_node.cpp`, `power.yaml` comment].
- `overcurrent_a: 5.0` for 0.5 s is an untested number: normal 1-3 A standing plus trot peaks may cross it (false E-STOPs mid-walk), while a single stalled leg (2.5 A) on top of a 2 A base does not.
- All 12 pulses start at tick 0 (`writeChannel(channel, 0, off)`), so all servo drive stages see synchronised 50 Hz edges: maximal ripple on V+ and 50 Hz coupling into the IMU and I2C wiring [MEDIUM: common PCA9685 practice to stagger ON times; not verified for this hardware].

**Why it happens:**
The design was validated with a perfect power source in simulation; the electrical numbers (6-12 A walking, 25 A peaks) are estimates in HARDWARE.md.

**Consequences:**
Pi reboots when standing up, servos jitter and lose torque, the robot sits down mid-walk, LiPo damaged by deep discharge, or false E-STOPs that make the floor test unusable.

**How to avoid:**
- First power-ups from the lab supply with a 3 A limit, then 5 A, then battery (DEPLOYMENT stage 5).
- Make the INA226 mandatory for floor runs: refuse to leave `passive` (or log ERROR and block `stand`) when `backend: auto` finds no sensor on the real robot.
- Measure real numbers before setting thresholds: record `/dog/power` through stand-up, trot forward/back, turn, and a deliberately blocked leg; set overcurrent to about 1.5x the walking peak (DEPLOYMENT stage 7 says the same) and undervoltage from the measured sag, not from the nominal.
- Cell-level protection outside software: a 2 EUR LiPo low-voltage buzzer on the balance lead, or a second INA226 on the battery side.
- Stagger the PWM ON edges in the driver (`on = channel * 256 mod 4096`) so 12 servo transients do not align.
- Scope V+ at the PCA9685 terminal during stand-up: the dip must not go below 5.2 V; add the 1000-2200 uF capacitor at V+ as already planned.

**Warning signs:** Pi reboot or SSH drop on "stand"; servo jitter/hum that appears only when several legs move; `/dog/power` voltage dips below 5.5 V; `power_monitor` line missing in the boot log; battery voltage under 3.7 V per cell after a short run.

**Phase to address:** B (mandatory sensor, thresholds mechanism, staggered ON edges); C (measure and set numbers); D (endurance run records the sag curve).

---

### Pitfall 5: MG996R stall and overheating, including self-inflicted buzzing and end-stop over-travel

**What goes wrong:**
HARDWARE.md estimates a knee servo carries about 0.54 N.m in trot, which is 50-59 % of the MG996R rating (0.92 N.m at 4.8 V, 1.08 N.m at 6 V) [MEDIUM: estimate]. At 50-60 % of stall torque a servo runs hot within minutes; clone MG996R units often fall short of their datasheet [LOW: forum reports]. Specific, code-verified aggravators:
- **Continuous hunting.** The driver writes whenever `current_` changes. Slope compensation (`slope.filter_tau`) and heading hold feed continuously varying targets, so a servo standing still gets a new value every tick, flipping by one PCA9685 tick (4.9 us). The MG996R dead band is about 5 us [MEDIUM: several spec sheets], so the servo hunts in and out of its dead band: audible buzz, current draw and heat while "standing still".
- **Holding forever.** After a command timeout the driver holds the pose indefinitely with servos energised (Pitfall 1). A dead gamepad battery becomes a slow cook.
- **No thermal sensing.** The only protection is the INA226 total current. One stalled leg (2.5 A) does not necessarily cross a 5 A total threshold (Pitfall 4). There is no per-leg current.
- **Thigh limits equal the full pulse range.** `servos.yaml`: thigh `min_deg -45 / max_deg 135` with `offset_deg 45` maps to a servo delta of exactly -90..+90 deg = 520..2220 us, the extremes. A glitch or a wrong live `offset_deg` drives the servo to its internal end stop and stalls it. (Calf: -75..+75 deg and hip +-40 deg are fine.)
- **Autocal `--find-limits` stalls servos on purpose** ("each joint is moved beyond its limit until it stops following or current spikes", `tools/autocal/README.md`).
- **Servo speed vs. gait demand.** The gait needs up to 5.1 rad/s; MG996R no-load is 6.2 rad/s at 4.8 V (0.17 s/60 deg) and 7.5 rad/s at 6 V. Under load and sag the margin is small or negative (see Pitfall 9).

**How to avoid:**
- Pulse hysteresis in the driver: skip a write when the new pulse differs by less than about 1.5 ticks, or quantise targets; keep the exact value in `current_`.
- Idle policy: N seconds without a valid deadman means `lie`, then relax if the body rests on its chassis in the lie pose (check this physically first; if the legs still carry weight in `lie` you cannot relax).
- Shrink soft limits inside the pulse extremes by at least 5-10 deg for thigh; treat "limit hit" (`joint(s) clamped`) as a hard warning during D.
- Skip `--find-limits` on the first calibration; find limits by hand with servos unpowered (DEPLOYMENT stage 1 already says so). If used, do it on the lab supply with a 3 A current limit and the robot in a cradle.
- Thermal budget: IR thermometer on the knee servo cases every 5 minutes (stage 12), stop at 60-70 deg C, and adopt a rule for the first floor sessions (walk 3 minutes, rest 3 minutes).

**Warning signs:** audible buzzing while standing; knee servo case too hot to hold for 3 seconds; `/dog/power` current not settling in stand; `joint(s) clamped` warnings; a servo that sags in `stand` after a few minutes.

**Phase to address:** B (hysteresis, idle policy, limits margin); C (limits, find-limits avoidance); D (thermal budget and acceptance criterion).

---

### Pitfall 6: Power-up jerk: first enable jumps each servo from an unknown position to the lie pose at full speed

**What goes wrong:**
In `ServoDriver::setTargets` a servo in state `OFF` becomes `PENDING` and, after `enable_stagger`, `ON` with `current_[i] = target_[i]` [HIGH: `servo_driver.cpp:198-206, 232-238`]. There is no slew from the real position (it is unknown): the very first write commands the lie pose and the servo goes there at its own maximum speed and torque. `enable_stagger` (0.15 s per leg) only spreads the inrush. `DEPLOYMENT.md` relies on a procedure ("fold the legs by hand into approximately lie") which is easy to forget and is not enforced. Compounding cases:
- After a driver crash/restart the chip may still emit stale pulses (Pitfall 1), so the "unknown position" is a stiff arbitrary pose.
- After E-STOP the robot collapses to an arbitrary pose; releasing and pressing `stand` jumps every leg.
- The horn-on-spline error after assembly is +-7-10 deg (DEPLOYMENT stage 5); calibrated `offset_deg` absorbs it but eats symmetric travel and makes the first jump larger.

**How to avoid:**
- Physical: a foam cradle/jig that holds the robot in the lie pose; always power servos up with the robot in it (also for every restart during C).
- Software: add an `assumed_start_pose` (= lie pose) so that on first enable `current_` starts from the assumed pose and is slewed to the target at a reduced first-move speed (for example 1.5 rad/s for the first second). It is no worse than today if the assumption is wrong, and much better if it is right.
- Enforce the sequence: Pi and stack up in `passive`; servo V+ on (switch or relay); explicit "release" from the operator; only then `stand`. Refuse `stand` while a fault is latched.
- First-ever powers from the lab supply with a current limit and the horns off (stage 5, keep it as written).

**Warning signs:** a loud clack of all four legs on first `stand`; a leg sweeping through the body; current spike above the supply limit at "stand"; one horn loosened after a few sessions.

**Phase to address:** B (assumed pose ramp, sequence enforcement); C (cradle, procedure).

---

### Pitfall 7: Calibration: config from v1 estimates, source conflicts, live writes that move a powered leg instantly, and a bind-mounted config that `git pull` can overwrite

**What goes wrong:**
- Defaults are "estimated from v1" (`servos.yaml` header). HARDWARE.md marks several values with a red flag because sources conflict: link lengths (55/105/105 vs 60/180/180 mm), hip-axis direction (v1 `walk_final.cpp` swings the hip fore/aft; v2 assumes abduction), `sign +1` on all servos in `servo_config.json` vs `inverted: true` on the right side in v1 params.
- `ros2 param set ...offset_deg` writes the pulse to a powered servo immediately at full speed; `validate()` only checks structure, not the size of the change. A typo (475 instead of 47.5) throws the leg to an end stop (Pitfall 5).
- Calibration stays in RAM until `ros2 param dump` and manual editing of `servos.yaml`; a container restart loses it.
- `docker-compose.yml` bind-mounts `ros2_ws/src/dog_bringup/config` over the installed config, and DEPLOYMENT stage 13 tells the operator to `git pull` on the Pi: a pull can silently replace robot-specific calibration with defaults, or conflict on merge.
- Web calibration (`cal_pose`/`cal_set`) is enabled by default (`allow_calibration: True`) (Pitfall 12).
- Calibration through the camera (`tools/autocal`) has never run on this robot; ArUco angle accuracy depends on the camera calibration and lighting.

**How to avoid:**
- Stage 1 of DEPLOYMENT.md (calipers, scale, `robot_setup --check`) is a hard gate before any powered calibration. If the hip axis is not abduction, stop and change the kinematics; do not tune around it.
- Limit the live step: in `onParams`/`setCalibration` reject an `offset_deg` change larger than about 10 deg per call while the servo is enabled, or slew the "live preview".
- Keep robot-specific calibration in a separate, untracked override file loaded after the repo defaults (`servos.local.yaml` mounted from `/etc/robot-dog/`), and back it up (photo + copy) after each calibration session.
- Verify by independent means: right-angle square on the leg and an angle gauge on the IMU board after calibration (1 deg tolerance, as the doc says); `calib_pose 15 0 -90` must move all feet left.
- Make the `hip` check in CALIBRATION.md step 3 a written pass/fail on the phase gate.

**Warning signs:** `foot target(s) outside the leg workspace` or `joint(s) clamped` in the stand pose; the four legs differ by more than 2 deg in `calib_pose 0 0 -90`; one leg opposite to the others in direction; `git status` on the Pi shows `servos.yaml` modified or `git pull` refuses.

**Phase to address:** C (procedure and override file); B (live-step limit).

---

### Pitfall 8: Tuning the gait in simulation against unverified geometry and mass (the sim phase may need repeating)

**What goes wrong:**
The owner's order puts simulation first, but all geometry and masses in `robot.yaml` are from v1 (`servo_config.json`) and the mass table is an estimate (1.48 kg, HARDWARE.md). The gait is closed-form kinematic (`gait.cpp`) tuned to hip 55 mm, thigh/calf 105 mm, stand height 0.15 m. If the real robot differs (v1 sources disagree by up to 70 % on thigh/calf), the work in A (backward walking, weights, step height) is tuned on the wrong robot and Stage 10's "re-run the sim with real masses" happens after the hardware work.

**How to avoid:**
- Insert **A0**, a half-day desk step with no powered servos: caliper the link lengths and hip spacing, weigh the robot and its parts, run `robot_setup --check`, put the numbers into `robot.yaml`, and re-run `walk_check` (8/8) before any further sim tuning. It is also the cheapest way to find out whether the hip axis convention is right.
- Also measure the two facts the sim cannot give: the loaded speed of one servo on a bench (hang a 300-500 g load at the leg radius, film with a 240 fps phone camera, time 60 deg travel at 5 V and 6 V), and the foot friction on the real floor (see Pitfall 9).

**Warning signs:** `robot_setup` mass check warns above 8 %; leg lengths measured differ from `robot.yaml` by more than 3 mm; the model in Gazebo does not look like the photo of the robot.

**Phase to address:** A0 (before A).

---

### Pitfall 9: The sim-to-real gap for hobby servos: the simulator is an ideal actuator with high friction

**What goes wrong:**
What Gazebo models today [HIGH: `dog_description/urdf.py`, `terrain.py`, `SIMULATION.md`]: a velocity-mode `JointPositionController` with p_gain 25, effort 1.1 N.m and 6 rad/s, no backlash, no compliance under load, no command latency, no voltage sag, foot sphere r 12 mm with mu 1.2 on a floor with mu 1.0, noiseless sensors, joint_states equal to physics truth.

What the real robot has:
- **Speed.** Gait demand up to 5.1 rad/s against 6.2-7.5 rad/s no-load and clearly lower under load and battery sag. The swing leg lands late, timing between diagonal pairs slips, the body rocks. Only an estimate here (loaded MG996R speed is not in any source read), so measure it (Pitfall 8).
- **Backlash and sag.** 1-2 deg gear backlash (HARDWARE.md) is about 2-4 mm at the foot per joint, plus 5-10 mm foot error from sag (REVIEW #13). The gait's swing height is 20 mm, so most of the clearance is consumed by error (see Pitfall 10).
- **No position feedback.** `joint_states` on hardware are the commands, so every downstream estimate assumes the legs are where they were told to be. Sim never exposed this because the sim reports truth.
- **Latency.** The sim controller sees commands in the same process tick; the robot has ROS scheduling on an A53, an I2C write per channel, servo electronics delay. Hobby-servo quadruped literature attributes most of the transfer gap to unmodelled actuator delay, backlash, and operation near the torque limit [MEDIUM: Tan et al. 2018 "Sim-to-Real: Learning Agile Locomotion for Quadruped Robots"; Jabbour et al. 2022 on ultra-low-cost hardware].
- **Friction.** Foot mu 1.0-1.2 in sim vs hard 3D-printed or bare-metal feet on laminate, commonly 0.2-0.5 [LOW: typical values; measure].

**Consequences:** the gait passes 8/8 in the sim yet on the floor it drags, slips, rocks, or walks in circles; the temptation is to retune on the robot with a dozen parameters at once.

**How to avoid (Phase A exit criteria, cheap in the existing harness):**
- Add a robustness sweep to `walk_check` (parameters in `urdf.py`, so one run per setting): servo velocity 4.0 and 3.5 rad/s, effort 0.8 N.m, foot/floor friction 0.4 and 0.6, command delay 30-50 ms (in `joint_command_bridge`), joint backlash +-1.5 deg (noise or dead-band on commanded angle), body mass +-15 %, CoM shifted 1-2 cm, IMU noise and bias. Require forward, lateral and turn to pass at a "pessimistic corner", not at the default.
- Keep the gait conservative: if step timing needs more than about 4.5 rad/s, lengthen `gait.period` or shorten `max_step` before the floor.
- Put rubber tips (a piece of tubing or hot glue) on the feet before the first floor run; measure the real friction with a simple incline test.
- Record the same test in D: forward 2 m with rosbag, and compare `cmd_vel` vs measured distance to the sim's percentages.

**Warning signs:** legs visibly still moving when the next step starts (film in slow motion); the body pitching at the gait frequency more than in sim; feet sliding at touchdown; `joint(s) clamped` in the log (limits/geometry mismatch).

**Phase to address:** A (sweeps and margins); D (validation and retuning one parameter at a time).

---

### Pitfall 10: Backward gait and foot slip: the weakest manoeuvre, hidden by the CI thresholds, worse on hardware

**What goes wrong:**
Backward walking passes 17-36 % of the command on waves and 0-36 % run to run on stones; on flat ground it is 53-75 % and "walks noticeably from run to run"; CI accepts 20 % backward on flat and 0 % on waves/stones (`--backward-ratio`), and a threshold of 40 % once failed at 39 % [HIGH: `TERRAIN.md`, `CONCERNS.md`, verified against the CI thresholds]. Mechanism: knees point backward, so the foot moves toward the knee side, the shank leans into the stride, and the toe stubs on anything taller than the remaining clearance (20 mm swing minus 3-10 mm of real-robot error, Pitfall 9). The swing profile in `gait.cpp` is symmetric in x for forward and backward (`p.z = step_height * sin(pi s)`, cosine blend), so backward has no special clearance or touchdown treatment. The real floor adds joint backlash, real friction (Pitfall 9) and the fact that the trot gait is not backed by contact sensing.

Also **descent** is a sim-only problem: crawl stair descent fails because supporting feet "creep" to the edge and "needs better foot position knowledge (contact sensing or servo current)" (`REVIEW.md` #20, `CONCERNS.md`). The robot has neither sensor. PROJECT.md already puts slopes and steps out of scope on hardware.

**How to avoid:**
- Time-box backward work: give it a quantitative target in sim under the pessimistic corner of Pitfall 9 (for example at least 40 % of the command on flat ground at mu 0.5 and 4 rad/s, no falls on 10 mm waves), and define the floor-milestone acceptance around forward, lateral and turn; backward at a reduced speed (start at 0.03-0.05 m/s) is "supported", not a gate.
- Backward-specific fixes to try in order of cost: a separate speed cap `max_vx_back`, higher swing (in the backward direction only, since the 35 mm test worsened forward), earlier and steeper touchdown, shorter stride.
- Restore `--backward-ratio` to 0.4 in CI once fixed so the fix cannot regress silently.
- Do not spend the milestone on stair descent (needs contact sensing); slope descent by backward walking is the case worth keeping.
- On the floor: back up only after forward is proven, with a hand near the body.

**Warning signs:** toe scuffing marks; the robot backing up less than half the commanded distance; a backing robot turning; the tilt rising in backward-only runs.

**Phase to address:** A (fix or cap); D (verify at reduced speed, hand near the body).

---

### Pitfall 11: No reaction to tipping over; the IMU is used only as a helper and is ignored above 25 deg

**What goes wrong:**
`slope.max_deg: 25` only turns off *slope compensation* above that angle (`locomotion.cpp:221`: `if (!upright || std::abs(gp) > lim ...) return;`). Nothing stops the gait, relaxes the servos, or latches a fault when the robot is on its side or back [HIGH: confirmed, no tilt reaction in `dog_control`]. A fallen robot keeps walking against the floor: stall current, heat, and a battery drained while the operator walks over to it. The hidden trap when adding it: attitude comes from a complementary filter with `accel_gate 0.15 g` and 44 Hz DLPF that has never seen real vibration (Pitfall 13), so a naive threshold either misses a fall or false-triggers on footfall spikes.

**How to avoid:**
- Fall rule: |roll| or |pitch| beyond 40-45 deg (well above the tested 25 deg operating limit and the 15 deg slope tests) for at least 150-200 ms, or accel Z below 0.3 g for more than 300 ms, while in `stand`/`walk` means immediate relax and a latched fault (needs a deliberate release, not just the gamepad's start button).
- Stale IMU while on the floor is also a fault: `imu/data` older than 0.5 s means stop and lie (today the controller silently degrades to "no compensation").
- Test with a real human-tilted robot on the cradle at increasing angle, log the false-positive rate over 30 minutes of trot before turning it on, and use the sim's tilt runs (walk_check tilt logs) to set the margin.
- Keep the fall relax separate from E-STOP semantics of Pitfall 12: a robot that is falling should relax immediately, but "lie then relax" is preferable for a merely deadman-released robot.

**Warning signs:** logs of tilt above 25 deg during otherwise "OK" runs; robot still stepping while on its side in sim `walk_check` failure recordings.

**Phase to address:** B (implementation, thresholds); D (false-positive burn-in, then verification by a deliberate hand-tip).

---

### Pitfall 12: The web pult is unauthenticated, has no Origin check, and shares ROS domain 0 with the simulator

**What goes wrong:**
- `web_teleop` binds `0.0.0.0:8080`; there is no token, no auth; `wsserver.py` never reads the `Origin`/`Host` headers (grep confirms) [HIGH]. Any page open in a browser on the LAN can open `ws://<robot>:8080/ws` (cross-site WebSocket hijacking; browsers do not apply the same-origin policy to WebSockets and there is no preflight) [MEDIUM: PortSwigger Web Security Academy, OWASP WebSocket cheat sheet, several CSWSH advisories]. From there it can drive, release E-STOP, and, because `allow_calibration` defaults to True and calibration is allowed whenever locomotion is `passive` (which E-STOP + release produces on demand), write servo offsets to a powered leg (Pitfall 7).
- `docker-compose.yml` sets `network_mode: host` and `ROS_DOMAIN_ID=0` (the default). The owner develops the same `/dog` namespace in Gazebo on a PC on the same LAN. A simulator or `walk_check` running on the laptop with the default domain publishes `/dog/joint_commands`, `/dog/cmd_vel` and `/dog/estop` that the real robot's `servo_driver` will happily consume, and the reverse. CI already uses a distinct `ROS_DOMAIN_ID` per step for this reason (leftover simulations mixed odometry into the next run, per CONCERNS.md). On a home network with a spare ROS 2 machine this is by far the most likely "security" incident.

**How to avoid:**
- Robot domain: pick a non-default `ROS_DOMAIN_ID` in `docker-compose.yml` and set the PC's sim to another; add `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` inside the container if nothing outside needs DDS (the web page and autocal use HTTP/WebSocket only) [MEDIUM: ROS 2 docs on discovery range; `ROS_LOCALHOST_ONLY` is deprecated in favour of it].
- Web: `allow_calibration: false` by default and turned on only for the calibration session (autocal's launch argument); check `Origin` against `Host` in `Server._handle`; a shared secret token in the URL query (checked at upgrade) is 20 lines and enough; only the connection that holds the drive lock may release E-STOP. Do not add TLS or accounts in this milestone.
- A robot on its own Wi-Fi access point (phone hotspot or a small router) removes most of the LAN exposure for free.
- Rotate the GitHub token embedded in the git remote (PROJECT.md, `.git/config`; not inspected here) and make sure no copy of it sits on the Pi in `~/robot-dog/.git/config`.

**Warning signs:** E-STOP toggles that nobody pressed in the log (`web: E-STOP released` with no operator action); `ros2 node list` on the robot showing two `/dog/locomotion` nodes; `ros2 topic info /dog/joint_commands` showing two publishers.

**Phase to address:** B (Origin/token/calibration default, domain id). Domain-id change is a one-line item, put it in C's compose changes if B is already full.

---

## Moderate Pitfalls

### Pitfall 13: IMU on a vibrating robot: aliasing, the 0.15 g accel gate, no mounting trim, gyro bias measured once

**What goes wrong:**
`imu_sensor.cpp` configures the MPU6050 with DLPF 44 Hz and output at 100 Hz (SMPLRT 9): the DLPF filters at 1 kHz internal rate and the result is decimated to 100 Hz, so vibration content between roughly 44 and 500 Hz partly aliases into 0-50 Hz. The robot has a strong 50 Hz source (12 servo PWM signals and the synchronised V+ ripple of Pitfall 4) sitting exactly at the DLPF corner and the Nyquist frequency [LOW: inference from the register configuration; MPU6050 vibration sensitivity itself is widely reported, MEDIUM]. Also:
- `accel_gate: 0.15` rejects accelerometer samples further than 15 % from 1 g. In sim (noiseless) that only drops footfalls; on a real trotting robot vibration plus footfalls can reject most samples, leaving roll/pitch on the gyro alone, drifting, and slope compensation (up to 40 mm foot shift) acting on drifted attitude.
- No mounting trim: `imu.yaml` has `axes` only. A board 2 deg off-level reads as a permanent 2 deg "slope" and the robot stands crooked by `h * tan(2 deg)`, about 5 mm.
- Gyro bias is measured once in the first second with a 0.05 rad/s stillness tolerance (about 2.9 deg/s, looser than typical MPU6050 bias) and MPU6050 drift with temperature over the first 5-10 minutes warms up on the robot. Heading hold integrates it.
- A wrong axis or gyro-Z sign turns heading hold into positive feedback (the robot spins up; DEPLOYMENT stage 7 covers the test, but only if it is done).

**How to avoid:**
- Soft-mount the IMU board (thin foam tape), near the body centre, away from servos and the power wires; twisted-pair I2C.
- Log raw accel for 60 s standing and 60 s trotting: compute the fraction of samples that pass the gate; tune `accel_gate` from the data, and lower DLPF to 21 Hz (config 4) or add an FIR before decimation; consider reading at 200-400 Hz and decimating in software.
- Add `roll_trim`/`pitch_trim` (measured on the level floor in `stand`) to `imu.yaml`.
- Re-measure gyro bias after 10 minutes of warm-up on the robot; bias should be checked, not assumed constant. Refuse to publish a fresh IMU whose bias measurement did not complete.
- First floor sessions: `slope.compensation: false` and `heading.hold: false`; enable heading hold first (with a 15 deg turn check), then slope compensation, one at a time.

**Warning signs:** `imu/data` orientation drifts by more than 2 deg over one minute of standing; roll/pitch wander during a trot even on a flat floor; heading hold makes the robot spin; `stand` leaves the body visibly tilted on a flat table.

**Phase to address:** C (mounting, axes, trim, bias); D (enable one feature at a time).

---

### Pitfall 14: I2C bus load and speed: about 85 % busy at 100 kHz, 12 separate transactions per tick

**What goes wrong:**
One bus carries the PCA9685 (12 writes of 6 bytes each per 10 ms tick), the MPU6050 (14-byte burst at 100 Hz), and the INA226 (20 Hz). Estimated worst-case bus occupancy at 100 kHz: 12 x 0.56 ms = 6.7 ms of every 10 ms for the PCA9685, +15 % for the IMU, +2 % for the INA226, about 85 %; at 400 kHz about 21 % [MEDIUM: my own calculation from the byte counts in `servo_bus.cpp`/`imu_sensor.cpp`; the actual bus speed on this board is unset in the repo (REVIEW #12) and unverified]. The servo itself only receives a new pulse every 20 ms (50 Hz PWM), so the driver's 100 Hz `update_rate` is twice the useful rate.

**How to avoid:**
- Measure first: `i2cdetect -l`, the DT overlay speed parameter, then log the driver's tick period (p99) during a trot.
- Cheap wins: `update_rate 50`; a single auto-increment write of all changed channels (MODE1 AI is already enabled) instead of 12 transactions; 400 kHz only after wiring is checked (short twisted pair, pull-ups: three modules with their own 4.7-10 k pull-ups in parallel are fine at 3.3 V; one at 5 V is not, HARDWARE.md).
- Keep the "3.3 V VCC for PCA9685" wiring rule (DEPLOYMENT stage 2) as a pre-power check.

**Warning signs:** `IMU read failed` and `current sensor read failed` (throttled every 5 s) appearing only when walking; jittery step timing at 100 Hz; missing IMU samples in `ros2 topic hz /dog/imu/data` (should be about 100 Hz).

**Phase to address:** B (batch write, rate), C (speed, wiring checks).

---

### Pitfall 15: Docker on the Banana Pi: device passthrough, build memory, restart loops, and lost state

**What goes wrong:**
- **Passthrough.** `devices: /dev/i2c-0` requires the node to exist when the container is created: without `i2c-dev` loaded and the overlay enabled, `docker compose up` fails and `restart: unless-stopped` keeps retrying [MEDIUM]. Bus numbering differs between vendor and mainline kernels on Allwinner boards; `/dev/i2c-0` is inherited from v1's kernel [LOW]. The image runs as root, which makes the device permissions work; a non-root hardening move (CONCERNS.md) needs a matching `i2c` GID in the container: do not do this before the first floor walk [MEDIUM: docker/podman device-group issues].
- **Build time and memory.** On-board builds take 30-60 minutes at `BUILD_JOBS=2` and can be OOM-killed on 2 GB; an emulated arm64 build on a laptop is 4-5x slower than native for C++ (up to 10-20x in reports) and occasionally flaky [MEDIUM: docker/buildx issue #2810 and several build-time write-ups]. CI's `robot-image` uses QEMU too, and only on non-PR events. The image is built without tests and never started in CI.
- **State inside the container.** Logs and any rosbag written in the container (`/tmp/walk1`) vanish at `docker compose up --build`; the map path `~/.ros/dog_map` likewise. `~/.ros/log` grows on an SD card.
- **Restart loop and auto-start.** Coming up at boot with servo power already on is safe only if the driver leaves outputs off (Pitfall 1). `restart: unless-stopped` re-runs a crashed stack without anybody looking.
- **PID 1.** `entrypoint.sh` `exec`s `ros2 launch` as PID 1; without an init reaper, zombies and signal handling quirks accumulate (`init: true`).
- **Timing.** No RTC; a Bluetooth gamepad and Wi-Fi share one radio on this class of board [LOW]; thermal throttling above 75 C makes the 50-100 Hz loops jitter.

**How to avoid:**
- Build on the laptop or an arm64 runner, `docker save | ssh docker load` as DEPLOYMENT stage 4 says; use zram on the Pi (stage 3); `--memory` limit off; check `dmesg` for OOM after any on-board build.
- `i2cdetect -l` and probing 0x40/0x68 on every bus in stage 3, before touching compose files; add a smoke test to the robot image (`robot.launch.py backend:=mock` for 10 s) to CI.
- Add volumes for `~/.ros/log` and a bags directory; `init: true`; log rotation.
- Watch `cat /sys/class/thermal/thermal_zone*/temp` and CPU load during a 10-minute walk (target < 1.5 cores and < 75 C).
- Prefer the gamepad on a wired USB connection for the first floor sessions if Bluetooth latency or drop-outs show up.

**Warning signs:** container in a restart loop after boot (`docker ps` shows short uptimes); `Killed` in the build output; `/dev/i2c-*` missing after boot; `docker system df` growing; log timestamps in 1970.

**Phase to address:** C.

---

### Pitfall 16: E-STOP semantics: a hard limp from full stand height, volatile latch, one shot

**What goes wrong:**
E-STOP relaxes all servos immediately (`setEstop(true)` calls `relax()`); from `stand` at 0.15 m the 1.5 kg body drops onto folded legs. Deadman and timeouts hold position; low-voltage triggers `lie`. E-STOP is published once, RELIABLE and volatile, and is not remembered by a restarted driver (by design, Pitfall 1). In the sim this collapse is harmless.

**How to avoid:**
- Two-level stop: soft stop (gamepad deadman release, timeouts, low battery, idle) does "lie then relax"; hard stop (E-STOP button, stall, fault, fall) relaxes immediately. Document which button does what and put the hard one on the gamepad's Back/View and the web page's space bar (already so).
- Test the drop once from a cradle 3 cm high before doing it from full stand on the floor.
- Check the horn screws and shoulder brackets after the first several E-STOPs.

**Phase to address:** B (semantic split); D (drop test).

---

### Pitfall 17: All acceptance thresholds and safety numbers were tuned on noiseless simulation

**What goes wrong:**
Guard thresholds, IMU filter constants, slope gain and heading PI gains, the overcurrent and undervoltage limits are simulation numbers (`REVIEW.md` #15, #21; `CONCERNS.md`). The heading-hold PI (`kp 2.5, ki 1.0`) and slope-compensation filter (`tau 0.8 s`) were tuned with a perfect IMU.

**How to avoid:** for the first floor sessions run with heading hold and slope compensation *off* (DEPLOYMENT risk table already suggests it on IMU axis problems) and enable them one by one with a bag recorded; take a slower first speed limit (`max_vx 0.05`, `max_vy 0.03`, `max_wz 0.3`, DEPLOYMENT stage 8) and lift it in stage 10; record a rosbag every session (`joint_commands`, `imu/data`, `power`, `state`, `cmd_vel`).

**Phase to address:** D.

---

## Minor Pitfalls

### Pitfall 18: Perception, lidar and localization pieces are running on a robot that has none of the sensors
**What goes wrong:** the `perception_node` guard has a `guard_timeout` that drops hazard limits when silent; if someone enables `perception:=true` on the real robot with no sensors, the guard looks "quiet" and the operator may believe obstacle protection is active (`locomotion_node.cpp`). **Prevention:** keep `perception`/`localization` off in `docker-compose.yml` for this milestone and label the web pult "no obstacle protection" (already out of scope in PROJECT.md). **Phase:** C.

### Pitfall 19: Stray v1 files and binaries at the repo root
**What goes wrong:** `robot_configurator.py`, `robot_dog_ws/`, `test_servo_config_reader.sh` (untracked, hard-coded paths and a LAN host) and compiled v1 walking binaries in `legacy/v1/` could be run by mistake on the new wiring. **Prevention:** do not run them; do not commit the untracked ones; on the Pi clone with `git clone --depth 1` (the repo is 71 MB of history, `report/` alone is 67 MB). **Phase:** C.

### Pitfall 20: The 2 GB Pi and the Python web node
**What goes wrong:** `web_teleop` (Python, hand-rolled WebSocket) costs about 8 % of a PC core and reads static files into memory per request; on an A53 that is competing with 100 Hz loops. **Prevention:** measure `top -H` during a walk with the web page open; if control jitter appears, throttle the calibration channel's 100 ms status stream and cache static files. **Phase:** C/D.

### Pitfall 21: Servo horn, screw and linkage looseness look like control problems
**What goes wrong:** loose horn screws and rod ends add backlash beyond the 1-2 deg assumed; an early symptom is drifting calibration between sessions. **Prevention:** thread-lock or nylon-insert screws on horns, torque check before every session (DEPLOYMENT stage 13 checklist), photo of the calibrated stand pose as a reference. **Phase:** C/D.

### Pitfall 22: Time and logs
**What goes wrong:** no RTC means bags and logs start at the wrong date until `chrony` syncs; a session can be uncomparable. **Prevention:** install `chrony` (stage 3) and put the date into the rosbag name. **Phase:** C.

---

## Technical Debt Patterns

Shortcuts that seem reasonable now but hurt later.

| Shortcut | Immediate Benefit | Long-term Cost | When Acceptable |
|----------|-------------------|----------------|-----------------|
| Keep `relax_on_exit` + timeout "hold" as the only stop path | No new code | Robot cannot be stopped when the driver dies (Pitfall 1) | Never for floor runs |
| Ignore `setPulseUs` result | Simpler driver | Blind faults (Pitfall 2) | Only in the mock/sim build |
| Copy `servos.yaml` edits into the tracked repo file | Simple workflow | `git pull` overwrites calibration; merge conflicts on the Pi | Bench only; use an override file for the robot |
| `--backward-ratio 0` in CI on waves/stones | CI stays green | Backward regressions hidden | Only until the fix; restore a real threshold |
| Tune more parameters in D instead of A | Faster feedback | Unrecorded, unreproducible robot-only settings | Only one parameter at a time, each committed |
| Non-default nothing: `ROS_DOMAIN_ID=0`, host network | Zero config | Sim and robot mix (Pitfall 12) | Never on a shared LAN |
| Free-running 100 Hz updates on a 50 Hz PWM | Smoother slew | Doubles bus load, no useful gain | Fine at 400 kHz; wasteful at 100 kHz |
| Keep the INA226 optional | Works without hardware | Silent loss of both power protections | Only on the bench |

## Integration Gotchas

| Integration | Common Mistake | Correct Approach |
|-------------|----------------|------------------|
| PCA9685 | Assume it stops with the host; do not clear on restart | Clear at driver start, watchdog relax, hardware cutoff; OE default is low (enabled) on the common module |
| PCA9685 VCC | Feed 5 V from the servo BEC | 3.3 V from the Pi (pull-ups follow VCC) |
| MPU6050 | Read on a bus that may be stuck; trust default DLPF | Timeouts, clean shutdowns, foam mount, tuned DLPF/gate |
| INA226 | Place after the BEC and call it battery monitoring | Also watch cell voltage (buzzer or a second sensor) |
| Docker device | `devices:` for a node that does not exist at start | Check `/dev/i2c-*` at boot, `init: true`, restart policy reviewed |
| ROS 2 DDS | Domain 0 on the same LAN as the sim | Non-default domain, discovery range LOCALHOST |
| Web pult | Listen on all interfaces without checks | Origin check, token, calibration off by default, own AP |
| Gazebo | Trust the pass on the default actuator model | Sweep speed, torque, friction, delay, backlash |

## Performance Traps

| Trap | Symptoms | Prevention | When It Breaks |
|------|----------|------------|----------------|
| 12 x 6-byte writes every 10 ms at 100 kHz | `read failed` while walking, jittery steps | Batch write, 50 Hz, 400 kHz | About 85 % bus occupancy (estimate) |
| Simultaneous PWM ON edges | V+ ripple, IMU noise at 50 Hz | Stagger ON times | All 12 legs moving together (trot, stand-up) |
| On-board colcon build | OOM kill, 30-60 min | Build on laptop/arm64, zram | Every build on 2 GB |
| Emulated arm64 build in CI | Slow, flaky | Native arm64 runner | Every push to main |
| CPU thermal throttling | Gait timing jitter | Heatsink, watch temp under 75 C | Long sessions, closed case |
| Python web node on the A53 | Control jitter with a browser open | Profile, throttle, cache | Multiple web clients |

## Security Mistakes

| Mistake | Risk | Prevention |
|---------|------|------------|
| Unauthenticated WebSocket on 0.0.0.0:8080 | Anyone on the LAN drives the robot, releases E-STOP | Token + Origin check + own AP |
| `allow_calibration: true` by default | Remote servo offset writes (leg jumps) | Default false; enable per session |
| Cross-site WebSocket hijacking | Any page in the operator's browser can drive it | Origin/Host check (a 5-line fix) |
| ROS domain 0, host network | Sim or another ROS machine commands the robot | Non-default domain, discovery range |
| GitHub token in git remote URL | Push access to the repo and maybe others | Rotate; SSH key or credential helper; check the Pi clone |
| Config bind-mounted over the package | `git pull` changes live behaviour without review | Override file outside the repo |

Risk order for this milestone: ROS domain collision (likely) > calibration exposed by default > CSWSH > general LAN drive > token rotation (do it now, independent of code).

## UX Pitfalls

| Pitfall | User Impact | Better Approach |
|---------|-------------|-----------------|
| E-STOP button and deadman indistinguishable in effect | Operator drops the robot from stand | Show soft vs hard stop; "lie" first |
| No indicator that the INA226/IMU are missing | Operator believes protection is on | Web pult shows sensor OK/off, refuse to stand without them |
| Calibration writes to a live leg | A typo throws a leg | Confirm dialog, bounded step |
| Gamepad dies silently | Robot holds pose and heats | Idle timeout to lie/relax, visible "no pad" |
| Recovery after fall needs shell | Slow, unsafe | Latched fault shown on web, clear via deliberate release |

## "Looks Done But Isn't" Checklist

- [ ] **Software watchdog:** often missing the process-dead case; verify with `kill -9`, `kill -STOP` and Pi power pull, servos limp in the time promised.
- [ ] **I2C error handling:** often missing on `disableAll()`; verify by forcing a failure (unplug SDA for a second) and watching for a latched fault and an error counter.
- [ ] **Tip-over reaction:** often missing the stale-IMU case; verify by tilting the cradled robot beyond the threshold and by killing the `imu` node.
- [ ] **Power protection:** often "sensor missing = off"; verify the stack refuses to stand without the INA226 and that low battery triggers at the measured value.
- [ ] **Calibration:** often only checked in `calib_pose`; verify the 4 legs at stand within 1-2 deg with a square, hip direction, right/left mirror, and that the file is saved outside the repo.
- [ ] **First-power sequence:** often relies on people; verify the cradle, the sequence and the assumed-start-pose ramp.
- [ ] **Sim sweeps:** often only default actuator settings; verify `walk_check` passes at velocity 4 rad/s, effort 0.8 N.m, mu 0.5, 40 ms delay.
- [ ] **Backward gait:** often "passes because the threshold is 0"; verify a real ratio threshold in CI.
- [ ] **Web exposure:** often "LAN only, fine"; verify Origin rejection with a page from another origin, and that calibration is off by default.
- [ ] **Docker on the Pi:** often works on the bench once; verify cold-boot start, `/dev/i2c-0` present, no restart loop, logs and bags outside the container filesystem.
- [ ] **Floor acceptance (D):** the criterion "without falling or overheating" needs numbers: for instance 10 minutes of walking in all directions on a flat floor, no fall, no latched fault, servo case < 60 C by IR thermometer, zero I2C errors, minimum rail voltage above 5.5 V, cell voltage above 3.6 V at the end.

## Recovery Strategies

| Pitfall | Recovery Cost | Recovery Steps |
|---------|---------------|----------------|
| Stale PWM after crash | LOW | Cut servo V+ at the switch; investigate log; add watchdog before the next run |
| Servo stalled or cooked | MEDIUM | Replace (spare on hand, DEPLOYMENT stage 0); re-calibrate that joint; check limits and horn |
| Stripped horn/gears after collapse | MEDIUM | Replace horn/servo; re-do the mount at 1370 us; re-run calibration for that leg |
| Stuck I2C bus | LOW | Stop the stack, bit-bang recovery or power-cycle IMU/PCA9685; then `i2cdetect` |
| Calibration overwritten by `git pull` | LOW | Restore from the override file or from the photo/backup; move to the override file |
| Brownout Pi reboot | LOW-MEDIUM | Check sag with a scope, bigger BEC or capacitor, separate ground routing |
| LiPo over-discharged | HIGH | Pack replacement; add cell monitoring before the next session |
| Sim and robot mixed on one domain | LOW | Change domain id; audit the log for unexpected commands |
| Sim tuned on wrong geometry | MEDIUM | Re-measure, update `robot.yaml`, re-run `walk_check` and the tuning of A |
| Fallen-robot cooked servos | MEDIUM | Replace; add and validate the tilt rule |

## Pitfall-to-Phase Mapping

| # | Pitfall | Prevention Phase | Verification |
|---|---------|------------------|--------------|
| 1 | PWM held after host dies | B (+C) | `kill -9`, `kill -STOP`, Pi power pull: servos limp in under 3 s |
| 2 | Ignored I2C write results | B (+C) | Injected failure latches a fault; counters 0 in a 30-min soak |
| 3 | Stuck I2C bus, blocked E-STOP | B (+C) | Forced bus fault: E-STOP still lands; recovery script works |
| 4 | Power sag, unmonitored cells, thresholds | B, C, D | INA226 mandatory; scope shows V+ above 5.2 V at stand-up; measured thresholds |
| 5 | MG996R stall, heat, buzz, end-stop | B, C, D | Hysteresis: no buzz at rest; case temp < 60 C in D; soft limits inside pulses |
| 6 | Power-up jerk and first pose | B, C | Cradle + ramp: first stand has no clack; sequence enforced |
| 7 | Calibration and config overwrite | C (+B live-step cap) | 4 legs within 1-2 deg with a square; override file; hip direction sign-off |
| 8 | Tuning on unverified geometry | A0 | `robot.yaml` = measured values; `walk_check` 8/8 on them |
| 9 | Sim-to-real actuator and friction gap | A, D | `walk_check` passes at the pessimistic corner; real-floor ratios logged |
| 10 | Backward gait and descent | A, D | Real threshold in CI; floor backward at reduced speed with hand near |
| 11 | No tip-over reaction | B (+D) | Tilt beyond limit relaxes and latches; stale IMU stops; no false trigger in 30 min |
| 12 | Web exposure, ROS domain collision | B (domain id in C) | Foreign-origin WebSocket rejected; two-domain test; calibration off by default |
| 13 | IMU vibration and trim | C, D | Raw accel logs, gate pass-rate, trim set, features enabled one by one |
| 14 | I2C bus load and speed | B, C | Tick p99 below 12 ms; zero errors at the chosen speed |
| 15 | Docker on Banana Pi | C | Cold boot starts cleanly; no restart loop; logs outside the container |
| 16 | E-STOP semantics | B, D | Soft vs hard stop defined; 3 cm drop test |
| 17 | Sim-tuned numbers | D | Heading/slope off then on with a bag each |
| 18-22 | Minor items | C/D | Checklists above |

## Phase-Specific Warnings

| Phase | Likely Pitfall | Mitigation |
|-------|---------------|------------|
| A0 (desk) | Real robot differs from v1 numbers | Measure, update `robot.yaml`, re-run `walk_check`; bench servo speed under load |
| A Sim gait | Tuning to the default actuator; time sink on stair descent | Robustness sweeps; time-box descent (needs contact sensing) and backward |
| B Protection | New watchdog that false-triggers; tilt rule false positives; hold the design comment on volatile E-STOP | Test with injected faults and a 30-minute burn-in; do hardware cutoff in parallel |
| C Bring-up | Powering servos with unchecked geometry; `--find-limits`; `git pull` overwrites calibration; stuck bus after restarts | Gate on stage 1 checks; cradle; lab PSU with limit; override file; clean shutdowns |
| D Floor | Retuning many parameters at once; heat; battery deep discharge | One change per run, committed; thermal budget; cell buzzer; features off then on |

## Claims from `CONCERNS.md` checked against the code

| Claim | Verdict |
|-------|---------|
| I2C write results ignored in the servo path | Confirmed (`servo_driver.cpp:262`, `servo_bus.cpp`); also `relax()`/`disableAll()` |
| No hardware watchdog; `relax_on_exit` only in destructor; timeout only logs | Confirmed; plus the `already_running` open branch and no launch respawn (new) |
| No fall/tilt detection | Confirmed (`slope.max_deg` only disables compensation) |
| E-STOP not latched at the driver | Confirmed, but it is a documented deliberate choice; the suggested TRANSIENT_LOCAL fix conflicts with the design comment in `locomotion_node.cpp` |
| Web: 0.0.0.0, no auth, no Origin check, calibration on by default | Confirmed (grep; `allow_calibration` default True) |
| Compose: host network, domain 0, config bind-mounted | Confirmed; the sim/robot domain collision is a new consequence |
| INA226 optional, silent when missing | Confirmed (`power_monitor` exits without spinning) |
| Bus speed unset, one write per channel | Confirmed (no batch write, no speed setting) |
| Backward gait weak, CI thresholds hide it | Confirmed (`--backward-ratio` values; TERRAIN.md numbers) |

Not spot-checked: perception/localization items (out of scope for this milestone).

## Sources

**Repo (HIGH, read directly):** `.planning/PROJECT.md`, `.planning/codebase/CONCERNS.md`, `docs/HARDWARE.md`, `docs/CALIBRATION.md`, `docs/DEPLOYMENT.md`, `docs/TERRAIN.md`, `docs/PLATFORM.md`, `docs/REVIEW.md`, `docs/SIMULATION.md`, `docs/GAITS.md`; code: `ros2_ws/src/dog_hardware/src/{servo_driver,servo_driver_node,servo_bus,imu_sensor,imu_node,power_monitor_node}.cpp`, `dog_control/src/{gait,locomotion,locomotion_node}.cpp`, `dog_bringup/config/{robot,servos,imu,power,teleop}.yaml`, `dog_bringup/launch/robot.launch.py`, `dog_web/dog_web/{web_teleop,wsserver}.py`, `dog_description/dog_description/urdf.py`, `dog_gazebo/dog_gazebo/terrain.py`, `docker-compose.yml`, `docker/Dockerfile`, `.github/workflows/ci.yml`, `tools/autocal/README.md`.

**Official (HIGH):**
- NXP PCA9685 datasheet (fetched, text-extracted): OE pin behaviour and MODE2 OUTNE (section 7.4, Table 11), power-on reset (7.5, POR outputs LOW, reset needs VDD below 0.2 V), OCH bit. https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

**Web, corroborated (MEDIUM):**
- Linux i2c-dev `I2C_TIMEOUT` units of 10 ms; `I2C_RETRIES`: linux-i2c patch "Clarify the unit of ioctl I2C_TIMEOUT" and `drivers/i2c/i2c-dev.c`. https://github.com/torvalds/linux/blob/master/drivers/i2c/i2c-dev.c
- Cross-site WebSocket hijacking and Origin validation: PortSwigger Web Security Academy https://portswigger.net/web-security/websockets/cross-site-websocket-hijacking ; OWASP WebSocket Security Cheat Sheet https://cheatsheetseries.owasp.org/cheatsheets/WebSocket_Security_Cheat_Sheet.html
- Sim-to-real for low-cost quadrupeds (actuator delay, backlash, torque limits): Tan et al., "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots" https://arxiv.org/pdf/1804.10332 ; Jabbour et al., "Closing the Sim-to-Real Gap for Ultra-Low-Cost, Resource-Constrained Robots" https://sim2real.github.io/assets/papers/2022/jabbour.pdf
- Servo brownout and Pi reboot with shared regulator: Raspberry Pi forum thread https://forums.raspberrypi.com/viewtopic.php?t=87066 (plus similar reports)
- MG996R 2.5 A stall, 5 us dead band, jitter under sag: spec summaries https://www.tinkered.ai/components/servo-mg996r-straight , https://www.espboards.dev/sensors/mg996r/ (secondary spec sites, MEDIUM at best)
- QEMU arm64 build slowness: docker/buildx issue #2810 https://github.com/docker/buildx/issues/2810 and build-time write-ups

**Web, single source or inference (LOW):**
- MPU6050 I2C lock-up: espressif/esp-drone issue #105 https://github.com/espressif/esp-drone/issues/105 ; general recovery via nine clock pulses: https://pebblebay.com/i2c-lock-up-prevention-and-recovery/
- Allwinner `xfer timeout` reports on Armbian: https://forum.armbian.com/topic/6705-sunxi_i2c_do_xfer978-i2c0-xfer-timeout-dev-addr0x48/ (other H-series boards)
- MPU6050 vibration and foam mounting, DLPF: Arduino forum threads https://forum.arduino.cc/t/mpu6050-vibration-noise-filtering-dlpf-and-kalman-filter/898227 ; the 50 Hz aliasing argument is my inference from the register settings
- I2C device group in containers: https://www.ddcutil.com/i2c_permissions_using_group_i2c/ ; https://github.com/containers/podman/issues/16605
- Not found: any source on the H618 TWI driver's bus-recovery support, the Banana Pi M4 Zero header's I2C bus numbering, the loaded speed of a MG996R, real foot friction of this robot. These are measurements to take (Pitfalls 3, 8, 9, 15).

---
*Pitfalls research for: hobby-servo quadruped, first floor walk after sim-only development*
*Researched: 2026-09-29*
