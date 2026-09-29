---
last_mapped_commit: 104fd262ee2afbd8c9ca25310c9647ab91815a75
last_mapped_at: 2026-09-29
---
# Codebase Concerns

<!-- refreshed: 2026-09-29 -->

**Analysis Date:** 2026-09-29

## Tech Debt

### I2C Bus Saturation (Critical Path)

**Issue:** Single I2C bus `/dev/i2c-0` at 100 kHz serves all devices: PCA9685 (12 channels, 100 Hz), MPU6050 (100 Hz), INA226, and 4 × VL53L1X (50 Hz). Theoretical maximum is ~10 kB/s; measured load is 6 kB/s (servos) + 4 kB/s (ToF) + 1.5 kB/s (IMU) = 11.5 kB/s.

**Files:**

- `ros2_ws/src/dog_hardware/src/servo_driver_node.cpp` (servo commands)
- `ros2_ws/src/dog_perception/src/perception_node.cpp` (VL53L1X reads)
- Hardware config: `ros2_ws/src/dog_bringup/config/robot.yaml`

**Impact:**

- Command delays to servos, skipped IMU samples, `read failed` errors on sensors
- Tight timing margins during walking may accumulate jitter

**Fix approach:**

- Enable 400 kHz clock (overlay in `armbianEnv.txt`) and benchmark
- Move VL53L1X to second I2C bus or reduce ToF frequency to 30 Hz
- If GS2 sensor replaces front ToF (issue #16 in REVIEW.md), load drops

**Status:** Identified in docs/REVIEW.md (item #12); not yet tested on robot

---

### Joint States: Commands, Not Measurements

**Issue:** `joint_states` topic on the robot publishes **commanded** angles, not measured positions. MG996R servos have no position feedback. Real errors accumulate from:

- Mechanical slack/backlash (1–3° per joint)
- Servo compliance and load sag (5–10 mm on stops)
- Cumulative error from leg geometry checks

**Files:**

- `ros2_ws/src/dog_control/src/locomotion_node.cpp` (publishes `joint_states`)
- `ros2_ws/src/dog_perception/src/core.cpp` (reads `joint_states` for foot plane)
- URDF generation: `ros2_ws/src/dog_description/dog_description/urdf.py`

**Impact:**

- Foot plane position drifts from reality (5–10 mm), breaking ground detection consistency
- Differential errors across legs cause uneven support and subtle tilts

**Fix approach:**

- Use lidar plane (`reference: auto`) as primary optic; cross-check legs only for large mismatches (> 2°)
- On deployment stage 14, measure actual vs. commanded positions with a level
- Long-term: replace servos with ones providing feedback (e.g., STS3215)

**Status:** Known; mitigation in place (dual-reference logic in perception); not fully resolved

---

### Real Lidar Scan Time vs. Gazebo Instantaneous Scans

**Issue:** Real lidars take ~100 ms per full rotation; Gazebo produces scans instantaneously. During walking, body pitches ~10°/s, so a real scan accumulates ~1° of attitude error over one rotation.

**Files:**

- `ros2_ws/src/dog_perception/src/perception_node.cpp` (processes LaserScan)
- `sensor_msgs/LaserScan` has `time_increment` field for per-ray timestamps

**Impact:**

- Ground plane estimate off by ~1° per scan in simulation vs. real robot
- Pipelined tests using Gazebo ground truth are optimistic for real deployment

**Fix approach:**

- Apply per-ray IMU correction using `time_increment` + IMU history
- Already done for pairs of scans; extend to all points within a single scan
- REVIEW.md item #14 specifies the approach

**Status:** Partially implemented; full per-scan correction pending

---

### Perception Node C++ Complexity and Testing Gap

**Issue:** `perception_node.cpp` (815 lines) and supporting modules (`core.cpp` 788, `localization_node.cpp` 746, `submaps.cpp` 518) handle complex geometry: plane fitting, height map, obstacle memory (grid), loop detection. While tested in simulation, **no field validation on real robot yet**.

**Files:**

- `ros2_ws/src/dog_perception/src/perception_node.cpp` (815 lines)
- `ros2_ws/src/dog_perception/src/core.cpp` (788 lines)
- `ros2_ws/src/dog_perception/src/localization_node.cpp` (746 lines)
- `ros2_ws/src/dog_perception/src/localization.cpp` (707 lines)
- `ros2_ws/src/dog_perception/src/submaps.cpp` (518 lines)
- Test suite: `ros2_ws/src/dog_perception/test/` (test_core.cpp, test_submaps.cpp)

**Impact:**

- Black surfaces, reflective floors, sunlight will show as obstacles or voids
- Sensor noise assumptions from simulation may not hold on real hardware
- High complexity makes debugging field issues difficult

**Fix approach:**

- Deployment stage 14 requires 10 min stationary + 5 min walking noise capture
- Recalibrate sensor thresholds from real data (formula: p99 noise × 1.3)
- Add confidence intervals to grid cells; log high-uncertainty zones for review

**Status:** Captured in REVIEW.md (items #15, #21); not yet deployed

---

### Obstacle Memory Eviction Bug (Fixed but Was Critical)

**Issue:** Fixed in revision 25.09.2026. Obstacle grid used a 600-message ring buffer; with 60 Hz sensor rate, confirmed obstacles ("stop") could be evicted in ~10 seconds if nearby sensors kept reporting "step over" instead. Robot then drove into walls.

**Files:**

- Fixed in: `ros2_ws/src/dog_perception/src/perception_node.cpp`
- Grid abstraction: `dog_perception/core.hpp` (obstacle cell structure)

**Current state:** Uses cell-based grid with persistent counters, not message ring buffer. Last confirmed state ("stop") no longer downgrades.

**Impact:** Was critical safety issue; now resolved.

**Status:** ✅ Fixed; test suite includes "wall nearby" scenario in `perception_check --terrain wall`

---

## Known Bugs

### Backward Movement Variance (Expected, Documented)

**Issue:** Backward walking on uneven terrain varies wildly (0–36 % of commanded distance, run to run). On flat ground it's stable.

**Files:**

- `ros2_ws/src/dog_control/src/crawl.cpp` (gait parameters, backward phase)
- CI: `.github/workflows/ci.yml` line 137 (`--backward-ratio 0` for uneven terrain checks)
- Docs: `docs/TERRAIN.md`

**Impact:**

- CI skips backward on waves/stones to avoid flaky tests
- Deployment stage 11 should avoid backward on rough terrain until root cause identified
- Backward descent on stairs (issue #20 in REVIEW.md) is untested and likely unsafe

**Root cause hypothesis:** Probing leg placement during backward pivot; gyro-based turn correction may amplify small errors into large position drift.

**Fix approach:**

- Instrument backward walking with position trace (lidar corners or measured displacement)
- Possibly reduce descent angle or add explicit braking phase
- REVIEW.md recommends crawl-on-3-legs for descents (25–70 mm); validate on real stairs

**Status:** Known; not a blocker for flat/shallow terrain; flagged for stage 11

---

### Foot Plane Outliers Near Obstacles

**Issue:** Lidar points near vertical walls (±27 mm deviation from plane) contaminate ground-plane fit. Robot detects valid ground but at slightly wrong height/tilt.

**Files:**

- `ros2_ws/src/dog_perception/src/core.cpp` (plane fitting via Jacobi eigenvalue)
- `ros2_ws/src/dog_perception/test/test_core.cpp` (test_plane_near_obstacle)

**Impact:**

- Optic height uncertainty ~10 mm near walls
- Redundancy via dual-reference (lidar + leg check) hides error but burns CPU

**Fix approach:**

- Pre-filter lidar points: discard any >3 cm from leg plane before fitting
- Already specified in REVIEW.md (item #19)

**Status:** Identified; pending implementation

---

### Course Holding Improvements Not Fully Tuned

**Issue:** IMU gyro-based course hold reduced max heading drift from 34° to 9° in simulation. Real offset (gyro bias) can drift 0.3–0.5 °/s and accumulates over long walks.

**Files:**

- `ros2_ws/src/dog_control/src/locomotion_node.cpp` (gyro integration)
- Config: `ros2_ws/src/dog_bringup/config/robot.yaml` (gyro calibration params)

**Impact:**

- Multi-meter straight walks accumulate 10–20° cumulative error
- Localization (issue #25) corrects this via map matching, but without a map, drift is large

**Fix approach:**

- Longer gyro bias calibration at startup (currently 2 s; consider 10 s)
- Real compass (HMC5883L) or visual odometry for long-term drift
- For now, localization_node corrects drift via submap loop closure

**Status:** Expected limitation documented in docs/LOCALIZATION.md; addressed by loop closure

---

## Security Considerations

### GitHub Token Embedded in `.git/config`

**Issue:** `.git/config` contains a GitHub personal access token (PAT) in the remote URL: `https://ghp_<REDACTED>@github.com/hzname/robot-dog.git`

**Files:**

- `.git/config` (line 7, remote URL)

**Risk:**

- Token grants full repository access if exposed
- Visible in shell history, process logs, or if machine is compromised
- Accidentally committed to forked repos or documentation

**Current mitigation:** `.gitignore` does not protect `.git/` contents (by design).

**Recommendation:**

1. **Revoke this token immediately** on GitHub (Settings → Developer settings → Personal access tokens)
2. Use SSH key (`git@github.com:...`) instead of HTTPS + token
3. Or use a deploy key with read-only access for CI
4. Audit git logs for accidental commits of secrets

**Status:** Requires immediate action before sharing repository or deploying to untrusted networks

---

### Docker Secrets Not Isolated

**Issue:** If the robot runs untrusted code in a container (e.g., via ROS launch file injection), environment variables and mounted volumes give access to all hardware and config.

**Files:**

- `docker-compose.yml` (lines 19–26: device and volume mounts)
- `docker/Dockerfile` and `docker/Dockerfile.sim` (base image security)

**Current state:**

- Container runs as root
- Direct access to `/dev/i2c-0`, `/dev/input` (all joysticks)
- Mounted config directory is read-only but world-visible

**Recommendation:**

- Run container as unprivileged user (create `docker_user` in Dockerfile)
- Restrict device access by group (e.g., `c 89:* r` for I2C read-only where possible)
- Use secrets manager (HashiCorp Vault, Docker secrets) for sensitive config
- For now, trust source of ROS launch files and treat robot network as trusted

**Status:** Acceptable for single-operator robot; tighten before deployment in shared space

---

## Performance Bottlenecks

### Web Teleop Python Node CPU Usage

**Issue:** `web_teleop` (Python, 248 lines) uses ~8 % of a Xeon ядра (estimated 0.3 ядра on A53). Slower than `locomotion` (2–4 %) despite simpler task.

**Files:**

- `ros2_ws/src/dog_web/dog_web/web_teleop.py` (248 lines)
- `ros2_ws/src/dog_web/dog_web/wsserver.py` (server loop)
- CI: `.github/workflows/ci.yml` (no web load test; mock test only)

**Impact:**

- On A53 with 4 cores, this is ~1.2 % of total CPU, not critical
- But margin is small; any optimization helps reserve CPU for perception or future vision tasks

**Fix approach:**

- Profile with `cProfile` to identify hotspots
- Consider batching state updates (send every 100 ms instead of every message)
- Cache static assets with proper HTTP cache headers (already done for `.html` files)
- Move heavy JSON parsing to C++ node if needed (low priority)

**Status:** Flagged in REVIEW.md (item #23) and COMPUTE.md; low priority for v2.0

---

### Terrain Perception Accuracy Without Loop Closure

**Issue:** Dead reckoning drifts 0.5–1 m over 5–6 m loops in simulation due to servo backlash and probing-leg placement variance. Loop-closure correction brings error down to 1.7 cm p95, but **only if a loop is detected**.

**Files:**

- `ros2_ws/src/dog_perception/src/localization_node.cpp` (loop detection via pose graph)
- `ros2_ws/src/dog_perception/src/submaps.cpp` (submap matching)
- `ros2_ws/src/dog_gazebo/dog_gazebo/localization_check.py` (test with survey phase)

**Impact:**

- Without a full survey (walking large circle or figure-8), position estimate is unreliable >2 m from start
- Height map drifts relative to walls, causing obstacle map to be misaligned after turning corners

**Fix approach:**

- Require initial "survey" behavior: robot spins slowly for 30 s at deployment start, establishing multiple lidar views to close initial loop
- Use `heading_imu` (gyro course) for long-term hold within ±5° (see issue #24)
- For large indoor spaces, implement A*-based loop search vs. brute-force pose graph (pending)

**Status:** Mitigation in place (survey as deployment step 14); full optimization pending

---

## Fragile Areas

### Servo Configuration Coupling

**Issue:** Servo mechanical directions, electrical polarity, and calibration values live in multiple files and must stay synchronized:

**Files:**

- `ros2_ws/src/dog_bringup/config/robot.yaml` (servo mounting frame, `direction: ±1`, offset, limits)
- `ros2_ws/src/dog_bringup/config/servos.yaml` (per-servo trim/range per channel)
- `ros2_ws/src/dog_control/src/locomotion.cpp` (IK assumes left legs mirror right via negation)
- URDF: `ros2_ws/src/dog_description/dog_description/urdf.py` (link orientations, joint axes)

**Why fragile:**

- A single copy-paste error in `direction` values breaks half the legs
- Offset updates in one file aren't validated against the other
- REVIEW.md item #8 is recent: `robot_setup` form now cross-checks these during input

**Safe modification:**

1. Always edit via `tools/robot_setup/robot_setup.py` form (web or CLI)
2. Run form's built-in checks (colcon test: `test_servo_direction_symmetry`)
3. After manual edits to YAML, run `colcon test` before deploying

**Test coverage:**

- `test/test_servo_driver.cpp`: verify servo commands map correctly to channel outputs
- `test/test_calibration.py`: verify `robot.yaml` → `servos.yaml` consistency
- Integration test: IK + forward kinematics on 200 random stances (docs/REVIEW.md, «Проверено»)

**Status:** Fragile but mitigated by tool; deployment stage 5 requires careful mechanical check

---

### Gazebo Idealized Physics vs. Real Servo Backlash

**Issue:** Simulation assumes:

- Perfect servo compliance at command angles (no lag or overshoot)
- Frictionless joints with zero backlash
- Ideal ground contact (no slip, no settling time after impact)

**Files:**

- `ros2_ws/src/dog_gazebo/dog_gazebo/world/` (physics config: `gravity`, `max_contacts`, friction)
- `dog_description/urdf.py` (joint friction parameters, disabled for performance)

**Impact:**

- Gait gains tuned in Gazebo (especially `kp`, `kd` for foot placement) may be too aggressive on real hardware
- Walking speed may differ 5–20 % on real robot vs. simulation
- Backward walking variance (issue #2 above) partly due to servo lag not modeled

**Real hardware factors not in sim:**

- 1–2° mechanical slack per joint
- 100 ms servo response lag (not modeled)
- Probing contact settling time (~50 ms before pressure is reliable)
- Voltage sag under load (power supply headroom)

**Fix approach:**

- Stage 10 (DEPLOYMENT.md) requires real gait video + side-by-side comparison with `walk_check` video
- If real speed is >10 % slower, reduce gains or increase swing time
- Deployment stage 12: log raw servo current to detect settling delays

**Status:** Known limitation; addressed by explicit sim-to-real validation step in deployment

---

## Scaling Limits

### Localization: Large Indoor Spaces

**Issue:** Submap-based SLAM with brute-force loop detection (pose graph pose-to-pose matching) scales to ~10 submaps (~30 m walking). Beyond that, matching time grows quadratically.

**Files:**

- `ros2_ws/src/dog_perception/src/localization_node.cpp` (submap addition, loop search)
- `ros2_ws/src/dog_perception/src/submaps.cpp` (pose graph, Gauss–Newton fitting)

**Limits:**

- Current: A53 can handle ~10 submaps at 50 Hz perception rate
- House mapping (issue #25 in REVIEW.md): tested in simulation on 5 × 6 m room (2 submaps); larger spaces untested

**Scaling path:**

1. Implement hierarchical search: coarse grid first (0.5 m), then refine to 0.05 m
2. Profile on real A53 with 20+ submaps to find CPU limit
3. Consider visual loop closure (fiducials or learned descriptors) if submaps reach limit

**Status:** Known limitation in REVIEW.md (item #25, "замыкание петель для большого дома"); mitigation planned

---

### Terrain Traversability: Stairs Not Fully Qualified

**Issue:** Crawl mode on 3 legs handles 25–70 mm obstacles, but **stair descent is untested on real robot**. Simulation shows leg slippage risk during backward descent.

**Files:**

- `ros2_ws/src/dog_control/src/crawl.cpp` (leg-lift heights, timing, 3-leg variants)
- `ros2_ws/src/dog_gazebo/dog_gazebo/terrain.py` (stair generation; descent tested in sim only)
- TERRAIN.md (section on "спуск по лестнице")

**Impact:**

- Robot may fall on stair descent if optic feedback (issue #1: joint_states) or gyro calibration is off
- Backward descent worse than forward due to leg visibility uncertainty

**Safe limit:** Tested and working on 50 mm steps forward; unknown backward and on stairs.

**Fix approach:**

- Deployment stage 11 starts with gentle slope (10°); stairs only after proving terrain handling
- Add contact sensors (simple bump switches) on toe to detect edge
- Retest crawl gains on real hardware before scaling to stairs

**Status:** Known; explicitly listed as "Открыто" in REVIEW.md (item #20)

---

## Dependencies at Risk

### NumPy Dependency (Perception Python Module)

**Issue:** `dog_perception/core.py` imports NumPy for sensor geometry calculations. Also needed by `perception_check.py` (simulation validation). Robot image uses `ros:jazzy-ros-base` (no rosdep, no NumPy by default).

**Files:**

- `ros2_ws/src/dog_perception/dog_perception/core.py` (lines 12: `import numpy`)
- `ros2_ws/src/dog_perception/test/test_core.py` (test helper functions)
- `ros2_ws/src/dog_gazebo/dog_gazebo/perception_check.py` (simulation, 582 lines)
- Docker: `docker/Dockerfile` (robot), `docker/Dockerfile.sim` (simulation)

**Impact:**

- Perception Python module would crash on robot if called directly (unlikely; C++ version used)
- Tests and offline tools need NumPy installed
- Simulation container includes it; robot container does not

**Current state:** C++ rewrite (`perception_node.cpp`, 815 lines) removed Python dependency from robot runtime. NumPy still used for offline checks and dev tools.

**Risk:** Low; but if Python perception is re-enabled or testing moves to robot, this breaks silently.

**Mitigation:**

- C++ is production code on robot; Python is development/validation only
- If offloading perception back to Python becomes necessary, must add NumPy to robot Dockerfile

**Status:** Resolved for production; note for future refactoring

---

### Git Configuration with Embedded Token (Duplicate of Security Section)

**Issue:** `.git/config` hardcodes GitHub PAT, which is a secret with repository push access.

**Files:**

- `.git/config`

**Risk:** Critical if repository is shared or deployed.

**Fix:** Revoke token and use SSH key (see Security section above).

---

## Test Coverage Gaps

### Real Robot Sensor Validation

**Issue:** All tests run in simulation or on ПК. Real hardware sensors not yet validated at scale:

- MPU6050: axes, calibration, noise floor on real vibrations
- ToF (VL53L1X): field-of-view behavior, reflectance errors, stray light
- Lidar: noisy near obstacles, performance in outdoor light
- INA226: current measurement linearity, grounding noise

**Files:**

- Test suite: `ros2_ws/src/dog_perception/test/test_core.cpp` (simulation data only)
- `ros2_ws/src/dog_gazebo/dog_gazebo/perception_check.py` (Gazebo validation)
- CI pipeline: `.github/workflows/ci.yml` (Gazebo only, no hardware)

**Impact:**

- Sensor tuning (issue #15, #21 in REVIEW.md) is manual post-deployment
- Unexpected sensor behavior (e.g., reflective floor) discovered late

**Fix approach:**

- Deployment stage 7 (DEPLOYMENT.md): capture raw sensor streams for 5 min (IMU, ToF, lidar)
- Compute noise statistics, record in deployment log
- Add reference calibration checklist for each sensor type

**Status:** Procedure documented; not yet automated

---

### Backward Gait Flakiness

**Issue:** Backward walking on rough terrain (waves, stones) has 0–36 % variance. CI avoids testing backward on uneven terrain (see issue #2 above).

**Files:**

- `.github/workflows/ci.yml` lines 137 (`--backward-ratio 0` skips backward on rough)
- `ros2_ws/src/dog_gazebo/dog_gazebo/terrain_sweep.py` (randomization; different results each run)

**Test gap:** No stable backward-on-rough coverage.

**Fix approach:**

- Instrument backward walking with IK residuals and servo current logging
- Identify if backward fails on specific body pitch angles or probing leg positions
- Consider alternative backward gait or add explicit braking phase

**Status:** Known; documented in TERRAIN.md as "weak manoeuvre"; not a blocker for v2.0 flat-ground use

---

### Obstacle Guard Memory Dynamics

**Issue:** Guard state machine has complex memory behavior (item #11 in REVIEW.md: per-cell counters, grid eviction, "stalled" state). Limited field validation at scale.

**Files:**

- `ros2_ws/src/dog_perception/src/perception_node.cpp` (guard grid logic)
- `ros2_ws/src/dog_gazebo/dog_gazebo/perception_check.py` (test scenarios)
- CI: `.github/workflows/ci.yml` lines 149–160 (wall test, 150–180 mm reach)

**Test scenarios:**

1. ✅ Flat floor: 0 false alarms
2. ✅ Wall 80 mm: stalls before touch
3. ✅ Steps 20–30 mm: all found, no fall
4. ❓ Narrow corridor (walls on both sides)
5. ❓ Dense clutter (many small objects)
6. ❓ Moving obstacles

**Fix approach:**

- Simulation corner cases added as needed during deployment
- Real-world testing of narrow spaces (issue #25: house mapping phase)

**Status:** Core scenarios covered; edge cases expected to emerge during field deployment

---

## Missing Critical Features

### Vision-Based Obstacle Detection

**Issue:** Current perception (lidar + ToF + GS2) is limited:

- Blind spots behind robot (only forward-facing)
- Misses low obstacles (<100 mm) without GS2
- No semantic understanding (wall vs. object vs. floor)

**Impact:** Cannot navigate crowded rooms or react to moving obstacles.

**Current limitations (by design):**

- Robot assumes walkable flat floor; works well in sparse, static environments
- Suitable for warehouse, structured indoor spaces
- Not ready for homes with stairs, clutter, pets, or people

**Planned addition:** Issue #25 (REVIEW.md) includes optional camera-based semantic perception post-v2.0.

**Status:** Not a bug; a known scope limit. Document in user manual.

---

### Dead Reckoning Drift Without Map

**Issue:** Without active localization (map loading/matching), position estimate drifts 0.5–1 m per 5–6 m loops.

**Files:**

- `ros2_ws/src/dog_control/src/locomotion_node.cpp` (DeadReckoning odometry)
- REVIEW.md item #2: deployed on robot; used by guard and height map

**Impact:**

- Relative navigation (e.g., patrol a small area) works for ~30 s
- Long-term positioning requires pre-built map or visual odometry

**Fix approach:**

- For mapped environments, use `localization_node` (issue #25)
- For unmapped spaces, add visual odometry (camera + feature tracking)
- Gyro integration reduces heading drift to ±5° (issue #24, documented in CONTROL.md)

**Status:** Expected limitation; mitigation (map-based localization) in progress (REVIEW.md #25)

---

## Architectural Concerns

### Python + C++ Mixed Codebase

**Issue:** Codebase mixes Python (web server, tools, test helpers) and C++ (control, perception). Two runtime environments, two build systems, separate test suites.

**Files:**

- Python: `ros2_ws/src/dog_web/`, `ros2_ws/src/dog_gazebo/dog_gazebo/`, `ros2_ws/src/dog_perception/dog_perception/`
- C++: `ros2_ws/src/dog_control/`, `ros2_ws/src/dog_hardware/`, `ros2_ws/src/dog_perception/src/` (perception_node rewrite)
- Build: `colcon` (ROS 2 meta-build) handles both

**Tradeoffs:**

- ✅ Python rapid development for tools and simulation
- ✅ C++ real-time performance for control/perception
- ❌ Two separate test suites, dependency on both interpreters
- ❌ Perception logic duplicated: C++ version for production, Python for reference/tests (mitigated: they now use same test scenarios)

**Safe modification:**

- Control logic stays C++ (locomotion_node, servo_driver)
- Tools and tests may be Python or C++; choose based on performance
- Keep test scenarios language-agnostic (JSON inputs/outputs)

**Status:** Acceptable; acknowledged in REVIEW.md item #4 (performance optimization justified C++ rewrite)

---

## Hardware Design Concerns

### I2C Pullup Voltage Mismatch (Mild Risk)

**Issue:** PCA9685 has 5 V pullup resistors on SDA/SCL (to its VCC), but Banana Pi I2C pins are 3.3 V. If VCC_PCA9685 = 5 V, pullups drive 5 V onto 3.3 V pins.

**Files:**

- HARDWARE.md, "Питание" section (mitigation given)
- `ros2_ws/src/dog_hardware/src/servo_driver.cpp` (I2C read/write)

**Mitigation:** DEPLOYMENT.md stage 0 specifies VCC_PCA9685 = 3.3 V (from Banana Pi), not 5 V.

**Status:** ✅ Mitigated; documented; easy to check at assembly

---

### Servo Supply Voltage and Headroom

**Issue:** System nominally runs on 6 V for servos. MG996R specs show:

- 4.8 V: 9.4 kgf·cm holding torque
- 6.0 V: 11 kgf·cm (18 % gain)

Under load (all 12 servos moving), supply voltage sags. If LiPo cell voltage drops below 3 V (nominal 3.7 V/cell, 2S = 7.4 V → 6 V with DC-DC), margin is lost.

**Files:**

- HARDWARE.md, "Питание" section (5 V / 3 A for Pi, 6 V / 15–20 A for servos separately)
- DEPLOYMENT.md stage 2 (power layout), stage 12 (resource testing)

**Impact:**

- Voltage sag → reduced servo torque → feet slip during step → odometry error
- Backward walking (issue #2) might partly be due to insufficient servo torque during probing

**Recommended action:**

- Deployment stage 2: verify separate BEC for Pi and servos
- Stage 12: log servo voltage under load, ensure >5.5 V during walking

**Status:** ⚠️ Design concern; mitigation in deployment plan; not fully validated

---

## Summary of Priorities

| Priority | Area | Fix Effort | Deployment Impact |
|----------|------|-----------|-------------------|
| 🔴 Critical | GitHub token in `.git/config` | 5 min | Blocks sharing/deployment |
| 🔴 Critical | I2C bus saturation (issue #12) | 1–2 days | Jitter on real robot |
| 🟠 High | Joint state feedback (issue #13) | 1 week | Foot plane accuracy |
| 🟠 High | Perception field validation (issues #15, #21) | 2 days | Sensor tuning post-deploy |
| 🟠 High | Real lidar scan timing (issue #14) | 2–3 days | 1° plane error over time |
| 🟡 Medium | Backward walking variance (0–36 %) | 3–5 days | Flaky CI on uneven terrain |
| 🟡 Medium | Obstacle memory edge cases (issue #11) | 1 day | Safety (now fixed, tested) |
| 🟡 Medium | Web teleop CPU usage (issue #23) | 1 day | Low priority, minor gain |
| 🟢 Low | Loop closure for large spaces (issue #25) | 1 week | Long-term SLAM, not v2.0 |
| 🟢 Low | Stairs untested (issue #20) | 2 weeks | Scope limit; safe to defer |

---

*Concerns audit: 2026-09-29*
