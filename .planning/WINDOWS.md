---
schema_version: 1
open_count: 4
waived_count: 0
fixed_count: 2
total_count: 6
last_updated: 2026-10-05T21:37:47.356Z
---

# Broken Windows Ledger

> Cross-phase defect register. With `workflow.windows_enforce` enabled, `/gsd-ship` blocks while `open_count > 0`.
> Waive with `gsd-tools windows waive <id> "<reason>"` (reason required).
> Mark fixed with `gsd-tools windows fixed <id>`.

| id | phase | kind | file | line | description | status | reason | recorded_at | resolved_at |
|----|-------|------|------|------|-------------|--------|--------|-------------|-------------|
| 1 | 01 | deviation | ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py |  | test_main_threshold_warns_on_falls asserts the plan's exact fall(s) message template ('has 1 fall(s)'), not the literal word 'falls' | open |  | 2026-10-05T09:04:01.291Z |  |
| 2 | 01 | deviation | .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-08-PLAN.md |  | Task 3 verify #2's CMakeLists/package.xml clause compares to main, which dependency plan 01-04 legitimately changed (68885c2); plan 01-08 ran the equivalent check against the plan base - both files untouched by this plan | fixed |  | 2026-10-05T18:42:21.292Z | 2026-10-05T18:42:27.625Z |
| 3 | 01 | deviation | ros2_ws/src/dog_bench/src/i2c_bus.cpp |  | FakeI2cBus::writeBytes filled registers one past the addressed one (regs[first + i]); fixed so data[1] lands on the addressed register as on the real wire - the bench release buffer must write ALL_LED_OFF_H (0xFD), not PRE_SCALE (0xFE) (fix in 839513c) | fixed |  | 2026-10-05T19:06:49.670Z | 2026-10-05T19:06:57.803Z |
| 4 | 01 | deviation | ros2_ws/src/dog_bench/test/test_session.cpp |  | Tracer recording runs the first four grid speeds (ids 0..39) because analyze.load_meta (01-06) requires a grid of at least four speeds; the task-1 verify id range was refined 10 -> 40 (plan bug fixed in the test, plan files untouched) | open |  | 2026-10-05T21:37:47.038Z |  |
| 5 | 01 | deviation | ros2_ws/src/dog_bench/test/test_session.cpp |  | pca_error case uses a pulse-only write fault (PulseFaultPwm): FakePwm::failWritesFrom also fails release() and would turn the planned 'released, code 1' into exit 3 (plan bug fixed in the test) | open |  | 2026-10-05T21:37:47.197Z |  |
| 6 | 01 | deviation | .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-12-PLAN.md |  | Task 3 verify regex refined to the real CI log (no per-file colcon test-result lines): gtest '14 tests from 3 test suites ran' + '[  PASSED  ] 14 tests.' + global summary zeros; the plan explicitly allows this refinement | open |  | 2026-10-05T21:37:47.356Z |  |

````json
[
  {
    "id": 1,
    "kind": "deviation",
    "phase": "01",
    "file": "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py",
    "line": null,
    "description": "test_main_threshold_warns_on_falls asserts the plan's exact fall(s) message template ('has 1 fall(s)'), not the literal word 'falls'",
    "status": "open",
    "reason": "",
    "recorded_at": "2026-10-05T09:04:01.291Z",
    "resolved_at": null,
    "milestone": null
  },
  {
    "id": 2,
    "kind": "deviation",
    "phase": "01",
    "file": ".planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-08-PLAN.md",
    "line": null,
    "description": "Task 3 verify #2's CMakeLists/package.xml clause compares to main, which dependency plan 01-04 legitimately changed (68885c2); plan 01-08 ran the equivalent check against the plan base - both files untouched by this plan",
    "status": "fixed",
    "reason": "",
    "recorded_at": "2026-10-05T18:42:21.292Z",
    "resolved_at": "2026-10-05T18:42:27.625Z",
    "milestone": null
  },
  {
    "id": 3,
    "kind": "deviation",
    "phase": "01",
    "file": "ros2_ws/src/dog_bench/src/i2c_bus.cpp",
    "line": null,
    "description": "FakeI2cBus::writeBytes filled registers one past the addressed one (regs[first + i]); fixed so data[1] lands on the addressed register as on the real wire - the bench release buffer must write ALL_LED_OFF_H (0xFD), not PRE_SCALE (0xFE) (fix in 839513c)",
    "status": "fixed",
    "reason": "",
    "recorded_at": "2026-10-05T19:06:49.670Z",
    "resolved_at": "2026-10-05T19:06:57.803Z",
    "milestone": null
  },
  {
    "id": 4,
    "kind": "deviation",
    "phase": "01",
    "file": "ros2_ws/src/dog_bench/test/test_session.cpp",
    "line": null,
    "description": "Tracer recording runs the first four grid speeds (ids 0..39) because analyze.load_meta (01-06) requires a grid of at least four speeds; the task-1 verify id range was refined 10 -> 40 (plan bug fixed in the test, plan files untouched)",
    "status": "open",
    "reason": "",
    "recorded_at": "2026-10-05T21:37:47.038Z",
    "resolved_at": null,
    "milestone": null
  },
  {
    "id": 5,
    "kind": "deviation",
    "phase": "01",
    "file": "ros2_ws/src/dog_bench/test/test_session.cpp",
    "line": null,
    "description": "pca_error case uses a pulse-only write fault (PulseFaultPwm): FakePwm::failWritesFrom also fails release() and would turn the planned 'released, code 1' into exit 3 (plan bug fixed in the test)",
    "status": "open",
    "reason": "",
    "recorded_at": "2026-10-05T21:37:47.197Z",
    "resolved_at": null,
    "milestone": null
  },
  {
    "id": 6,
    "kind": "deviation",
    "phase": "01",
    "file": ".planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-12-PLAN.md",
    "line": null,
    "description": "Task 3 verify regex refined to the real CI log (no per-file colcon test-result lines): gtest '14 tests from 3 test suites ran' + '[  PASSED  ] 14 tests.' + global summary zeros; the plan explicitly allows this refinement",
    "status": "open",
    "reason": "",
    "recorded_at": "2026-10-05T21:37:47.356Z",
    "resolved_at": null,
    "milestone": null
  }
]
````
