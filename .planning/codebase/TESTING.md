---
last_mapped_commit: d269bd16b0d0f69f3ab623b9cecc4ba545b70517
last_mapped_at: 2026-09-29
---
# Testing Patterns

**Analysis Date:** 2026-09-29

Three layers, all wired into CI (`.github/workflows/ci.yml`):

1. **Unit tests** of ROS-independent cores: GoogleTest (C++) and pytest (Python), no ROS graph.
2. **Launch/integration tests** with `launch_testing` against real node processes (mock hardware backends).
3. **Gazebo simulation checks** (`dog_gazebo` CLI tools) that drive the simulated robot and return exit code 0/1; not part of `colcon test`.

`legacy/` has its own old tests (`legacy/v1/tests/test_ros_bridge.py`, C++ tests under `legacy/v1/robot_dog_ws/src/*/test/`); they are not built (`legacy/COLCON_IGNORE`) and not run in CI. Do not extend them.

## Test Framework

**Runner:**

- C++: GoogleTest through `ament_cmake_gtest` (`ament_add_gtest`), C++17, built with `-Wall -Wextra -Wpedantic`.
- Python unit tests: pytest (`python3-pytest`), either via colcon (`ament_python` packages, `ament_add_pytest_test`) or standalone (`python -m pytest`).
- Integration: `launch_testing` + `launch_testing_ament_cmake` (`add_launch_test`) with `unittest.TestCase` classes and `@pytest.mark.launch_test`.
- ROS distros under test: `jazzy` and `lyrical` (CI matrix). Python 3.12 for the standalone tool jobs.
- Config: no `pytest.ini`/`pyproject.toml`; per-package `setup.cfg` only sets script dirs. Test wiring lives in each `CMakeLists.txt` (`if(BUILD_TESTING) ... endif()`).

**Assertion Library:**

- gtest macros: `EXPECT_*` for independent checks, `ASSERT_*` when later lines would dereference or index a failed result (`ASSERT_TRUE(c.request("stand"));`, `ASSERT_TRUE(out.twist);`). Floating point: `EXPECT_NEAR(a, b, tol)` / `EXPECT_DOUBLE_EQ`, never `EXPECT_EQ` on doubles.
- pytest: bare `assert`, `pytest.approx` for floats, `pytest.raises(ValueError)`, `numpy.testing.assert_allclose` for arrays. `unittest` assertions (`assertAlmostEqual`, `assertTrue(..., msg)`) only inside launch tests.

**Run Commands:**

```bash
cd ros2_ws && colcon build && source install/setup.bash          # build everything
colcon test --packages-skip dog_gazebo --event-handlers console_cohesion+   # what CI runs
colcon test-result --verbose                                     # summarize failures
colcon test --packages-select dog_control && colcon test-result  # one package
./build/dog_control/test_kinematics --gtest_filter='Kinematics.RoundTrip*'   # one gtest binary
python -m pytest -q tools/autocal/tests                          # standalone tool tests (needs numpy, opencv-contrib-python-headless)
python -m pytest -q tools/robot_setup/test                       # needs pyyaml
python tools/robot_setup/robot_setup.py --check                  # CI also validates the committed config
python -m robotdog_autocal demo                                  # run from tools/autocal (CI smoke test)
```

Simulation checks (need Gazebo, `osrf/ros:<distro>-simulation`):

```bash
ros2 launch dog_gazebo sim.launch.py headless:=true web:=false &
ros2 run dog_gazebo walk_check --backward-ratio 0.2
ros2 run dog_gazebo terrain_sweep --terrain slope --levels 10 --out slope.json
ros2 run dog_gazebo perception_check --terrain flat --seconds 12 --expect --expect-guard
ros2 run dog_gazebo localization_check --seed 0 --phase mapping --map $PWD/room_map --expect
```

No watch mode and no coverage tooling are configured.

## Test File Organization

**Location:**

- Separate `test/` directory per ROS package, mirroring the unit under test (`ros2_ws/src/dog_control/test/test_kinematics.cpp` for `src/kinematics.cpp`). Tools use `tools/<tool>/tests/` (`autocal`) or `tools/<tool>/test/` (`robot_setup`).

**Naming:**

- Files `test_<unit>.{cpp,py}`. gtest suites: `TEST(<Unit>, <PascalCaseSentenceDescribingBehavior>)` (`TEST(Locomotion, LieWhileWalkingFinishesStepsFirst)`, `TEST(Submaps, ABareCornerIsNoAnswer)`). pytest: `test_<snake_case_sentence>` (`test_bad_messages_produce_errors_only`, `test_robust_plane_ignores_a_stone`). Name tests for the behavior/guarantee, not the function.

**Structure:**

```
ros2_ws/src/<pkg>/
  src/foo.cpp
  include/<pkg>/foo.hpp
  test/test_foo.cpp          # gtest, links <pkg>_core
  test/test_foo.py           # pytest or launch_test
tools/autocal/tests/conftest.py   # adds tools/autocal to sys.path
```

Registering a new C++ test: add `ament_add_gtest(test_foo test/test_foo.cpp)` + `target_link_libraries(test_foo <pkg>_core)` inside `if(BUILD_TESTING)`. `dog_control` loops over names: `foreach(t kinematics gait crawl greet locomotion)` in `ros2_ws/src/dog_control/CMakeLists.txt`: add the name there. Registering a launch test must assign a unique DDS domain (see below).

Current inventory (about 130 test cases):

- `dog_control`: `test_kinematics`, `test_gait`, `test_crawl`, `test_greet`, `test_locomotion` (C++)
- `dog_hardware`: `test_servo_driver`, `test_power`, `test_imu` (C++); `test_power_monitor.py` (launch)
- `dog_perception`: `test_core`, `test_localization`, `test_submaps` (C++, `test_submaps` has `TIMEOUT 300`); `test_core.py` (numpy twin)
- `dog_teleop`: `test_mapping` (C++); `test_gamepad_fifo.py` (launch)
- `dog_web`: `test_protocol.py`, `test_wsserver.py`
- `dog_description`: `test_urdf.py`
- `dog_bringup`: `test_mock_bringup.py` (launch, full stack)
- `tools/autocal/tests`: `test_fit`, `test_geometry`, `test_servo_model`, `test_procedure`, `test_client`
- `tools/robot_setup/test/test_robot_setup.py`
- No tests: `dog_gazebo` (verified by its own check tools), `tools/sim_video`.

## Test Structure

**Suite Organization (gtest, `ros2_ws/src/dog_control/test/test_locomotion.cpp`):**

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "dog_control/locomotion.hpp"

using dog_control::LocomotionController;
using dog_control::Mode;

namespace
{
constexpr double kDt = 0.02;

void run(LocomotionController & c, double seconds)
{
  for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {c.update(kDt);}
}
}  // namespace

TEST(Locomotion, StandUpReachesStandHeight)
{
  LocomotionParams p;
  LocomotionController c(p);
  ASSERT_TRUE(c.request("stand"));
  run(c, p.transition_time + 0.1);
  EXPECT_EQ(c.mode(), Mode::STAND);
  EXPECT_EQ(c.unreachableCount(), 0);
}
```

- Helpers and constants go in an anonymous namespace at the top of the test file (`kDt`, `run`, `footInBody`, `expectNear`). Common test-file-local helpers are duplicated per file rather than shared in a header.
- Drive time-dependent logic by calling `update(dt)` in a loop with a fixed `kDt`; never sleep and never read the wall clock in a unit test.
- Fixture classes (`TEST_F`) only when several tests share heavy setup (`DriverTest` in `ros2_ws/src/dog_hardware/test/test_servo_driver.cpp` builds a `MockBus` + `ServoDriver` in `SetUp()`); otherwise plain `TEST`.
- Add a one-line comment for every non-obvious assertion, stating the physical/logic reason (`EXPECT_LT(ik.q[2], 0.0);  // knee bent backwards`).
- Print context on loop assertions with `<<`: `EXPECT_FALSE(clamped) << deg;`, `ASSERT_TRUE(c.validate().empty()) << c.validate();`.

**pytest (`ros2_ws/src/dog_web/test/test_protocol.py`):**

```python
import json
import pytest
from dog_web import protocol

LIM = protocol.Limits()

def test_drive_is_scaled_and_clamped():
    a = protocol.handle_message('{"type":"drive","vx":1,"vy":-0.5,"wz":7}', LIM)
    assert not a.errors
    assert a.twist == pytest.approx((LIM.max_vx, -0.5 * LIM.max_vy, LIM.max_wz))

@pytest.mark.parametrize('text', ['not json', '[]', '{"type":"dance"}'])
def test_bad_messages_produce_errors_only(text):
    a = protocol.handle_message(text, LIM)
    assert a.errors
```

- Module-level constants for shared inputs, `@pytest.mark.parametrize` for input matrices, `pytest.fixture` with `tmp_path` for file-based tests (`tools/robot_setup/test/test_robot_setup.py` `cfg` fixture copies `robot.yaml`/`servos.yaml` into `tmp_path`).
- Tests that must reach a sibling package do `sys.path.insert(0, os.path.join(os.path.dirname(__file__), ...))` and mark the import `# noqa: E402` (`tools/autocal/tests/test_client.py` imports the robot's real `dog_web.wsserver`).
- Optional dependency: `pytest.importorskip('cv2')` at module top (`tools/autocal/tests/test_procedure.py`).

**Property/sweep style:** Prefer sweeping the workspace and asserting an invariant over hand-picked points. Example: `Kinematics.RoundTripOverWorkspace` nests loops over side, knee direction, x, y, z, asserts `forwardKinematics(inverseKinematics(t)) == t` to 1e-9, counts the cases and asserts `checked > 1000` so the sweep cannot silently become empty. Random data uses a fixed seed (`np.random.default_rng(0)`, `std::mt19937` with a constant seed): tests must be deterministic.

**Patterns:**

- Setup: construct the object under test with default params (`LocomotionController(LocomotionParams{})`); no global state.
- Teardown: RAII in C++; `try/finally` around sockets in Python (`ws.close()`); `setUp`/`tearDown` with `rclpy.init()`/`rclpy.shutdown()` in launch tests.
- Prove safety behavior explicitly: e-stop blocks commands, watchdogs fire once, NaN/`bool`/non-object JSON rejected, clamped targets flagged (`EstopGoesPassiveAndBlocksCommands`, `test_watchdog_fires_once_after_stall`, `UnreachableTargetsAreClampedAndFlagged`).

## Mocking

**Framework:** None (no gmock, no `unittest.mock`). Test doubles are hand-written production classes behind small interfaces.

**Patterns:**

```cpp
// production code ships the fake: ros2_ws/src/dog_hardware/include/dog_hardware/servo_bus.hpp
class MockBus : public ServoBus { /* records pulses_, writes_ */ };

// test: ros2_ws/src/dog_hardware/test/test_servo_driver.cpp
bus = std::make_shared<MockBus>();
driver = std::make_unique<ServoDriver>(bus, names, cals, p);
driver->setTargets(names, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, 10.0);
driver->update(10.0);
EXPECT_NEAR(bus->pulse(i), jointToPulseUs(driver->calibrations()[i], 0.1 * (i + 1)), 1e-9);
```

- Hardware is abstracted (`ServoBus` -> `MockBus` / `Pca9685Bus`; power sensor and IMU have `mock` backends). Launch tests select them with node parameters `backend: mock` (`'power': 'mock'`, `'imu': 'mock'` launch args; `mock.voltage`, `mock.current` parameters that the test changes live through `SetParameters`).
- Time is injected (`now`/`dt` arguments), so no clock mocking is needed.
- Python fakes for the robot live next to the code they fake: `tools/autocal/robotdog_autocal/sim.py` (`FakeRobot`, `VirtualCamera`, `hidden_errors(seed)`), used by tests and by the CI `python -m robotdog_autocal demo`.
- Hardware devices are replaced with OS primitives: a FIFO carrying raw Linux `js_event` structs stands in for `/dev/input/js0` (`ros2_ws/src/dog_teleop/test/test_gamepad_fifo.py`, `struct.pack('IhBB', ms, value, kind, number)`).
- Network is real but local: `Server(str(tmp_path), on_client)` on `127.0.0.1` port `0` with a raw-socket client (`ros2_ws/src/dog_web/test/test_wsserver.py`); bringup test uses fixed `WEB_PORT = 18080`.

**What to Mock:**

- I2C buses, PWM outputs, IMU/current sensors, joystick devices, cameras.

**What NOT to Mock:**

- Kinematics, gait, controller state machines, protocol parsing, YAML round-trips, the real WebSocket server. Test them for real; cross-check independent implementations against each other (URDF forward kinematics vs `dog_control` formulas in `test_urdf.py`; numpy vs C++ perception core; Python `servo_model.py` vs C++ `servo_driver.cpp` reference values).

## Fixtures and Factories

**Test Data:**

```cpp
// factory helpers in an anonymous namespace: test_servo_driver.cpp
ServoCalibration knee()
{
  ServoCalibration c;
  c.channel = 2; c.direction = 1; c.offset_deg = -90.0;
  c.min_deg = -165.0; c.max_deg = -15.0;
  return c;
}
```

```python

# test_core.py: dict literals mirroring robot.yaml

SENSORS = {'x_lidar': True, 'x_lidar_x': 0.10, ...}
GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}
H = 0.150
```

- Synthetic worlds are generated in code: rooms as segment lists sampled into scans (`room()`, `sample()`, `scanFloor()` in `ros2_ws/src/dog_perception/test/test_localization.cpp` and `test_core.cpp`); hidden servo errors from a seed (`sim.hidden_errors(seed)`).
- Shipped config is used as a real fixture: `test_urdf.py` loads `ros2_ws/src/dog_bringup/config/robot.yaml`; `test_robot_setup.py` copies it to `tmp_path`. A test therefore fails when someone commits an inconsistent `robot.yaml`/`servos.yaml`: keep those files valid.
- **Location:** No shared fixtures directory. Only `tools/autocal/tests/conftest.py` exists (path setup). Recorded data for reports lives in `tools/sim_video/report/data/*.json` and is not used by tests.

## Coverage

**Requirements:** None enforced; no coverage tool is configured, no threshold in CI.

**View Coverage:** Not available. To add: build with `--cmake-args -DCMAKE_CXX_FLAGS="--coverage"` and use `gcov`/`lcov`, or `pytest --cov` (needs `pytest-cov`); nothing in the repo depends on it.

## Test Types

**Unit Tests:**

- Scope: one core class or module, deterministic, milliseconds to seconds. All C++ tests except `test_submaps` (up to 300 s timeout because of pose-graph work) and all Python tests in `dog_web`, `dog_description`, `dog_perception`, `tools/*`.

**Integration Tests (launch_testing, real ROS processes, mock hardware):**

- `ros2_ws/src/dog_bringup/test/test_mock_bringup.py`: starts `robot.launch.py backend:=mock web:=true`, then walks through calibration channel over WebSocket, stand-up via topic, walking, `cmd_vel` timeout, web drive, e-stop and lie-down; ends with a `@launch_testing.post_shutdown_test()` class asserting exit codes `[0, -2, -15]`.
- `ros2_ws/src/dog_hardware/test/test_power_monitor.py`: stall -> e-stop, sagging rail -> `lie`; also launches a second node with an absent sensor and asserts it exits 0.
- `ros2_ws/src/dog_teleop/test/test_gamepad_fifo.py`: gamepad + joy_teleop chain from raw bytes.
- **Each launch test must set a unique `ROS_DOMAIN_ID`** in its `add_launch_test(... ENV ROS_DOMAIN_ID=NN)` because colcon runs packages in parallel and all use the `/dog` namespace. Taken: 41 (`dog_hardware`), 42 (`dog_teleop`), 43 (`dog_bringup`). Use 44+ for new ones; CI simulation runs use 60-77 (`terrain_sweep --domain`, `ROS_DOMAIN_ID=71..77`, each with its own `GZ_PARTITION`).
- Set explicit `TIMEOUT` (60-120 s) on every `add_launch_test`.

**Simulation checks (Gazebo, CI jobs `simulation` and `terrain`):**

- `walk_check` runs a fixed maneuver routine (forward/back/sideways/turns/lie) against ground-truth odometry; thresholds are deliberately loose (`min_ratio` 0.4, tilt < 20 deg, body height >= `MIN_BODY_HEIGHT`) and only catch sign errors, falls, broken gaits. `terrain_sweep` re-runs it per terrain/level in a fresh simulation and retries once when the robot never left `passive`. `perception_check` and `localization_check` assert guard behavior and map accuracy with `--expect` / `--expect-guard`.
- Conventions: one PASS/FAIL line per check (`'%-6s %-12s %s' % ('PASS'|'FAIL', name, detail)`), a JSON result file via `--out`/`--trace`, exit code 0 = all passed. Time budgets use simulated time (odometry stamps), not wall time, so slow CI runners do not change commanded distances (`WalkCheck.spin` in `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py`).
- Loosen thresholds only with a comment citing the measurement and doc (see the `--backward-ratio` comments in `.github/workflows/ci.yml` referencing `docs/TERRAIN.md`).
- These need Gazebo and take minutes: `--packages-skip dog_gazebo` in the `build-test` job; the separate `simulation` and `terrain` jobs run them.

**E2E Tests:**

- The Gazebo checks and `test_mock_bringup.py` are the end-to-end tests. There is no real-hardware automated test; on-robot verification is manual per `docs/DEPLOYMENT.md`.
- Untracked `test_servo_config_reader.sh` (repo root) is a manual SSH-to-robot smoke script for the older `robot_dog_ws`; it is not part of CI.

## Common Patterns

**Async / polling ROS tests (no fixed sleeps):**

```python
def spin_until(self, predicate, timeout, publish=None):
    end = time.time() + timeout
    while time.time() < end:
        if publish:
            publish()
        rclpy.spin_once(self.node, timeout_sec=0.05)
        if predicate():
            return True
    return False

self.assertTrue(self.spin_until(lambda: self.state == 'stand', 8.0,
                publish=lambda: self.cmd_pub.publish(String(data='stand'))),
                f'locomotion not ready, state={self.state}')
```

Every launch test defines its own `spin_until` (copy the shape from `test_mock_bringup.py`). Always give the assertion a failure message with the observed value. Negative checks use the same helper and expect `False` (`assertFalse(self.spin_until(lambda: self.state != 'estop', 1.0))`). Publish repeatedly inside the loop for topics with volatile QoS; use `TRANSIENT_LOCAL` subscription for latched `state`.

**Async (asyncio) tests without pytest-asyncio:**

```python
async def _scenario(tmp_path):
    server = Server(str(tmp_path), on_client)
    port = await server.start('127.0.0.1', 0)
    try:
        ...
    finally:
        await server.stop()

def test_server_end_to_end(tmp_path):
    asyncio.run(_scenario(tmp_path))
```

Use `asyncio.run` inside a sync `test_` function; bound waits with `asyncio.wait_for(..., 2.0)`.

**Error Testing:**

```cpp
c.direction = 0;
EXPECT_FALSE(c.validate().empty());       // validate() returns a message; assert non-empty
```

```python
with pytest.raises(ValueError):
    fit_joint('j', ServoCal(), [1300, 1400, 1500], [10.0, 10.0, 10.0])

a = protocol.handle_message(text, LIM)   # handlers return errors, they do not raise
assert a.errors and a.twist is None
```

Assert on message substrings only when the message is part of the contract (Russian validation text in `test_robot_setup.py`, `'passive'` in the calibration refusal in `test_mock_bringup.py`).

**Floating point tolerances:** `1e-9` for closed-form math, `1e-12` for exact round trips, `1e-3` to `2e-3` for physical/geometry (metres), looser for simulation. State the tolerance's reason in a comment when it is not obvious.

## Gaps to Know Before Adding Code

- `ros2_ws/src/dog_gazebo/` (bridges, `terrain.py`, `sim.launch.py`) has no unit tests; regressions surface only in the Gazebo CI jobs.
- ROS node classes (`LocomotionNode`, `ServoDriverNode`, `perception_node.cpp`, `localization_node.cpp`, `web_teleop.py`) are covered only through launch tests of `dog_bringup`/`dog_hardware`/`dog_teleop`; put new logic in the cores so it is unit-testable.
- `dog_web/static/app.js` (browser UI) and `tools/sim_video/` have no tests.
- No linters or coverage gates run in CI, so style and dead code are unchecked by machines.

---

*Testing analysis: 2026-09-29*
