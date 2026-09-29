---
last_mapped_commit: 104fd262ee2afbd8c9ca25310c9647ab91815a75
last_mapped_at: 2026-09-29
---
# Coding Conventions

**Analysis Date:** 2026-09-29

## Naming Patterns

**Files:**

- Module files: `lowercase_with_underscores.py` (e.g., `servo_model.py`, `vision.py`)
- Package directories: `lowercase` (e.g., `robotdog_autocal`, `dog_perception`)
- Test files: `test_*.py` (e.g., `test_fit.py`, `test_geometry.py`)

**Functions and Methods:**

- Public functions: `snake_case` (e.g., `fit_joint`, `fk_standing_height`, `is_data_key`, `compute_standing_section`)
- Private functions: Leading underscore `_snake_case` (e.g., `_send`, `_exact`, `_reader`, `_wrap`, `_unit`, `_leg_plane_normal`)
- Private methods in classes: Leading underscore (e.g., `self._send()`, `self._reader()`, `self._call()`)

**Classes:**

- PascalCase (e.g., `RobotClient`, `Calibrator`, `SensorMount`, `MarkerTracker`, `Intrinsics`, `FitResult`, `Linkage`, `ServoCal`)
- Dataclass usage: use `@dataclass` decorator for structured data (e.g., `Linkage`, `ServoCal`, `FitResult`)

**Variables and Constants:**

- Instance attributes: `snake_case` (e.g., `self.sock`, `self.replies`, `self.latest_status`)
- Private instance attributes: Leading underscore (e.g., `self._send_lock`, `self._alive`, `self._detect`, `self._obj`)
- Module-level constants: `CAPS_WITH_UNDERSCORES` (e.g., `DOC_PREFIX = "_"`, `BASE_POSE`, `SWEEP`, `ORDER`, `DICTIONARY`)

**Type Hints:**

- Use type hints in function signatures: `def fit_joint(joint, cal: ServoCal, pulses, angles_eff)`
- Use type hints for complex parameters and return types
- Class attributes in dataclasses get type hints: `@dataclass` fields automatically typed

## Code Style

**Formatting:**

- No explicit formatter configuration found (no `.flake8`, `.pylintrc`, or `black` config)
- 4-space indentation (standard Python)
- Line continuations allowed (observed in multi-line function calls and string formatting)

**Linting:**

- No linting configuration file detected
- Follow PEP 8 conventions implicitly

## Import Organization

**Order:**

1. Standard library imports (e.g., `import json`, `import math`, `import os`)
2. Third-party imports (e.g., `import numpy as np`, `import cv2`, `import yaml`)
3. Local imports (e.g., `from . import geometry as geo`, `from .servo_model import ServoCal`)
4. Conditional/optional imports with error handling:
   ```python
   try:
       import cv2
   except ImportError:
       cv2 = None
   ```

**Path Aliases:**

- Relative imports within packages: `from . import module`, `from .submodule import Class`
- Absolute imports: `import sys`, `from pathlib import Path`

## Error Handling

**Patterns:**

- Use specific exception types: `ValueError`, `RuntimeError`, `TimeoutError`, `ConnectionError`
- Raise with descriptive messages including context:
  ```python
  raise ValueError(
      f"FK error: negative discriminant ({discriminant:.6f}). "
      f"Impossible geometry: thigh={thigh_length}, calf={calf_length}, "
      f"calf_angle={calf_angle_rad}"
  )
  ```
- Custom exceptions for domain-specific errors (e.g., `class MissingMarkers(RuntimeError)`)
- Exception handling in I/O operations: `except (OSError, ConnectionError, ValueError):`
- Re-raise with context when needed: `raise TimeoutError(f'...') from None`

## Logging

**Framework:** No dedicated logging framework used; uses `print()` for output

**Patterns:**

- Pass a `log` callable as parameter: `def __init__(self, ..., log=print)`
- Call it for status updates: `self.log(f'message')`
- Use string interpolation for formatting: `f'formatted {value}'`

## Comments

**When to Comment:**

- Module docstrings: Describe purpose, usage, and key concepts
- Section separators: Use dashed lines for major sections: `# ──────────────────────────────────`
- Non-obvious algorithms: Explain the mathematical or logical approach
- Bilingual comments: Some modules use Russian and English (e.g., `robot_configurator.py`)

**Docstrings:**

- Module level: Triple-quoted docstrings at module start
- Function level: Describe purpose, parameters, and return values in docstrings
- Class level: Document class purpose and responsibility
- Example from `robot_configurator.py`:
  ```python
  def fk_standing_height(hip_length: float, thigh_length: float,
                         calf_length: float, calf_angle_rad: float) -> dict:
      """
      Вычисляет высоту стояния через прямую кинематику.
      Computes standing height via forward kinematics.
      """
  ```

## Function Design

**Size:** Keep functions focused and reasonably sized; decompose complex logic into helper functions

**Parameters:**

- Use descriptive parameter names matching the domain (e.g., `hip_length`, `thigh_length`, `calf_angle_rad`)
- Use type hints in function signatures
- Default values for optional parameters

**Return Values:**

- Return dictionaries with descriptive keys for multiple values:
  ```python
  return {
      "target_dist_m": round(target_dist, 6),
      "standing_height_m": round(standing_height, 6),
  }
  ```
- Use dataclasses for complex return types: `FitResult`, `ServoCal`
- Return `None` implicitly or explicitly where appropriate

## Module Design

**Exports:**

- Module-level functions and classes are public by default
- Use leading underscore for private/internal functions
- Group related functions together (e.g., all FK-related functions, validation functions)

**Barrel Files:**

- Not heavily used; direct imports from specific modules are preferred
- Example: `from robotdog_autocal.servo_model import ServoCal`

## Class Design Patterns

**Dataclasses:**

- Use `@dataclass` for data containers with automatic `__init__`, `__repr__`, and other methods
- Example: `@dataclass class Linkage:` with typed fields

**Properties:**

- Use `@property` for computed attributes:
  ```python
  @property
  def beam(self):
      return self.R[:, 0]
  ```

**Factory Methods:**

- Use `@classmethod` for alternative constructors:
  ```python
  @classmethod
  def approximate(cls, width, height, hfov_deg=65.0):
      # ...
      return cls([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]])
  ```

**Private Methods:**

- Prefix with underscore: `def _send(self, obj):`, `def _reader(self):`

## Special Patterns

**Section Organization:**

- Use dashed comment lines to separate logical sections:
  ```python
  # ──────────────────────────────────────────────────────────────
  #  FK: Forward Kinematics
  # ──────────────────────────────────────────────────────────────
  ```

**Validation Functions:**

- Return lists of warning/error messages rather than raising exceptions
- Example: `def validate_body(cfg: dict) -> list[str]:`
- Caller decides how to handle warnings

**Configuration and Constants:**

- Store configuration in JSON/YAML files
- Load and validate configurations separately
- Store derived/computed values in `_computed` sections

---

*Convention analysis: 2026-09-29*
