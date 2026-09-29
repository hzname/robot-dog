---
last_mapped_commit: 104fd262ee2afbd8c9ca25310c9647ab91815a75
last_mapped_at: 2026-09-29
---
# Testing Patterns

**Analysis Date:** 2026-09-29

## Test Framework

**Runner:**

- pytest (implicit; no configuration file found)
- Config: No `pytest.ini`, `setup.cfg`, or `pyproject.toml` configuration
- Discover tests by pattern: `test_*.py` files with `test_*` functions

**Assertion Library:**

- pytest built-in assertions
- NumPy assertions: `np.allclose()`, `np.testing.*`
- Math assertions: `math.isfinite()`, `math.isnan()`

**Run Commands:**

```bash

# Standard pytest invocation (no special config)

python -m pytest

# Run specific test file

python -m pytest tools/autocal/tests/test_fit.py

# Run specific test

python -m pytest tools/autocal/tests/test_fit.py::test_direct_fit_recovers_direction_offset_and_scale

# With verbose output

python -m pytest -v
```

## Test File Organization

**Location:**

- Co-located with source: `tools/autocal/tests/` next to `tools/autocal/robotdog_autocal/`
- Separate `test/` directories: `ros2_ws/src/dog_perception/test/`, `tools/robot_setup/test/`
- Python package structure: conftest.py for setup

**Naming:**

- Test files: `test_*.py` (e.g., `test_fit.py`, `test_geometry.py`, `test_servo_model.py`)
- Test functions: `test_*` prefix (e.g., `test_direct_fit_recovers_direction_offset_and_scale`)
- Test helper functions: No prefix, placed in same file (e.g., `camera_positions()`, `run()`, `_gs2_scan()`)

**Structure:**

```
tools/autocal/
├── robotdog_autocal/
│   ├── __init__.py
│   ├── client.py
│   ├── fit.py
│   ├── geometry.py
│   ├── servo_model.py
│   └── ...
├── tests/
│   ├── conftest.py          # Shared fixtures and setup
│   ├── test_fit.py
│   ├── test_geometry.py
│   ├── test_procedure.py
│   ├── test_servo_model.py
│   └── test_client.py
└── setup.py
```

## Test Structure

**Basic Test Function:**

```python
def test_direct_fit_recovers_direction_offset_and_scale():
    # Arrange
    true = ServoCal(direction=-1, offset_deg=48.3, range_deg=184.0)
    believed = ServoCal()
    q = np.linspace(20, 60, 9)
    pulses = [true.joint_to_pulse(a) for a in q]
    noisy = q + np.random.default_rng(0).normal(0, 0.3, len(q))
    
    # Act
    r = fit_joint('j', believed, pulses, noisy)
    
    # Assert
    assert r.direction == -1
    assert r.offset_deg == pytest.approx(48.3, abs=0.4)
    assert r.rms_deg < 0.5
```

**Parametrized Test:**

```python
@pytest.mark.parametrize('view', ['left', 'right'])
@pytest.mark.parametrize('tilt', [0.0, 6.0])
def test_side_views_recover_thigh_and_calf(view, tilt):
    # Test logic using view and tilt parameters
```

**Test with Fixtures:**

```python
@pytest.fixture
def cfg(tmp_path):
    # Setup test configuration
    for f in ('robot.yaml', 'servos.yaml'):
        shutil.copy(os.path.join(rs.CONFIG, f), tmp_path / f)
    return str(tmp_path)

def test_save_changes_only_values_and_keeps_comments(cfg):
    # Test using cfg fixture
```

## Test Fixtures and Setup

**Fixture Definition:**

```python

# conftest.py - minimal setup

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Usage in tests:

# @pytest.fixture

# def cfg(tmp_path):  # tmp_path is built-in pytest fixture

#     return str(tmp_path)

```

**Built-in Fixtures Used:**

- `tmp_path`: pytest-provided temporary directory fixture
- Example: `def test_yaml_update(tmp_path):`

**Test Data:**

- Module-level constants: `SENSORS`, `GEOM`, `H`, `FLOOR` in `test_core.py`
- Reference values: `CPP_REFERENCE` dictionary in `test_servo_model.py`
- Example:
  ```python
  SENSORS = {
      'x_lidar': True, 'x_lidar_x': 0.10, 'x_lidar_y': 0.04, ...
  }
  ```

## Mocking

**Framework:** No dedicated mocking library (unittest.mock) observed in codebase

**Patterns:**

- **Fake implementations:** Classes like `FakeRobot`, `VirtualCamera` in `sim` module
- **Integration testing:** Tests use real implementations where possible
- **Dependency injection:** Pass dependencies as constructor arguments
  ```python
  cal = procedure.Calibrator(robot, cam, vision.MarkerTracker(...))
  ```

**What to Mock:**

- Hardware interfaces: Use fake implementations (`FakeRobot`, `VirtualCamera`)
- File systems: Use `tmp_path` fixture instead of mocking

**What NOT to Mock:**

- Business logic: Test with real implementations
- Mathematical computations: Verify against reference values
- Geometry/physics: Use real implementations with assertions on results

## Assertions and Comparisons

**Float Comparisons:**

```python

# Use pytest.approx for floating point tolerance

assert r.offset_deg == pytest.approx(48.3, abs=0.4)     # absolute tolerance
assert r.direction == pytest.approx(1, abs=1e-9)        # very tight tolerance

# Use math functions where appropriate

assert math.isfinite(t)
assert math.isnan(value)
```

**Exception Testing:**

```python

# Using pytest.raises

def test_joint_that_does_not_move_is_reported():
    with pytest.raises(ValueError):
        fit_joint('j', ServoCal(), [1300, 1400, 1500], [10.0, 10.0, 10.0])
```

**Array/Matrix Assertions:**

```python

# NumPy assertions

assert np.allclose(pts[:, 2], -H, atol=1e-9)
assert np.allclose(feet[:, 2], -0.150, atol=1e-6)

# Element-wise assertions with pytest.approx

for leg in geo.VIEWS[view]['legs']:
    assert got[f'{leg}_thigh_joint'] == pytest.approx(th + i * 3, abs=1e-6)
```

## Test Types

**Unit Tests:**

- Scope: Single function/method in isolation
- Examples: `test_direct_fit_recovers_direction_offset_and_scale()`, `test_linkage_matches_cpp()`
- Focus: Verify computation correctness

**Integration Tests:**

- Scope: Multiple components working together
- Examples: `test_full_calibration_recovers_hidden_errors()`, `test_hazard_guard()`
- Focus: End-to-end workflows

**Property-Based Tests:**

- Round-trip tests: `test_round_trip(link)` - verify encoding then decoding preserves value
- Example:
  ```python
  def test_round_trip(link):
      c = ServoCal(...)
      for deg in range(-130, -49, 5):
          assert c.pulse_to_joint(c.joint_to_pulse(deg)) == pytest.approx(deg, abs=1e-6)
  ```

**E2E Tests:**

- Not explicitly present in isolated test files
- Shell-based E2E in `test_servo_config_reader.sh` for hardware integration

## Common Testing Patterns

**Test Helper Functions:**

```python

# Helper in test file for common setup

def run(robot, views=procedure.ORDER, passes=2, frames=2):
    results = {}
    for _ in range(passes):
        for view in views:
            cam = sim.VirtualCamera(robot, view)
            cal = procedure.Calibrator(robot, cam, ...)
            res = cal.calibrate_view(view)
            cal.apply(res)
            results.update(res)
    return results, cal

# Used in multiple tests

def test_full_calibration_recovers_hidden_errors(seed):
    robot = sim.FakeRobot(true)
    results, cal = run(robot)  # Helper call
    # assertions
```

**Async-like Testing (with Simulated Time):**

```python

# Sequential calls with state progression

def test_hazard_guard_crawl_window_matches_the_cpp_guard():
    g = core.HazardGuard()
    assert g.command(0, (0.0, 0.0), 0.0)[2] != 'crawl'    # t=0
    vx, _, state, _ = g.command(1, (0.2, 0.0), 0.0)       # t=1
    assert state == 'crawl'
    assert g.command(2, (0.85, 0.0), 0.0)[2] == 'crawl'  # t=2
    assert g.command(3, (0.95, 0.0), 0.0)[2] != 'crawl'  # t=3
```

**Nested Validation Testing:**

```python

# Verify expected structure within assertions

def test_validation_catches_what_would_break_the_robot():
    base = rs.load(rs.CONFIG)
    v = dict(base, stand_height=260)
    # Check both level and message content
    assert any('нога достаёт' in t 
               for lv, t in rs.validate(v)[0] if lv == 'error')
```

**Parametrized Complex Scenarios:**

```python
@pytest.mark.parametrize('seed', [1, 2])
def test_full_calibration_recovers_hidden_errors(seed):
    true = sim.hidden_errors(seed)
    robot = sim.FakeRobot(true)
    # Test runs with different seeds
```

## Coverage

**Requirements:** Not enforced; no `.coveragerc` or pytest plugin configuration

**View Coverage:**

```bash

# No standard coverage tool configured

# If needed in future:

python -m pytest --cov=robotdog_autocal --cov-report=term-missing
```

## Test Environment

**Dependencies:**

- pytest
- NumPy (for numerical tests)
- OpenCV with ArUco (optional, skipped if missing): `pytest.importorskip('cv2')`
- YAML (for configuration tests)

**Optional Feature Skipping:**

```python

# Skip test if dependency missing

pytest.importorskip('cv2')

# Graceful degradation in module

try:
    import cv2
except ImportError:
    cv2 = None
```

## Key Testing Principles

**Determinism:**

- Use seeded random numbers: `np.random.default_rng(0)`
- Avoid time-dependent assertions
- Use fixtures for consistent state

**Clarity:**

- Test names describe what's being tested: `test_side_views_recover_thigh_and_calf`
- One logical assertion per test (may have multiple `assert` statements)
- Use helper functions to reduce repetition

**Integration-Focused:**

- Tests use real implementations, not mocks
- Verify against reference implementations (C++ values in `test_servo_model.py`)
- End-to-end scenarios preferred over isolated units

---

*Testing analysis: 2026-09-29*
