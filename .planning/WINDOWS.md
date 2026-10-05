---
schema_version: 1
open_count: 1
waived_count: 0
fixed_count: 1
total_count: 2
last_updated: 2026-10-05T18:42:27.625Z
---

# Broken Windows Ledger

> Cross-phase defect register. With `workflow.windows_enforce` enabled, `/gsd-ship` blocks while `open_count > 0`.
> Waive with `gsd-tools windows waive <id> "<reason>"` (reason required).
> Mark fixed with `gsd-tools windows fixed <id>`.

| id | phase | kind | file | line | description | status | reason | recorded_at | resolved_at |
|----|-------|------|------|------|-------------|--------|--------|-------------|-------------|
| 1 | 01 | deviation | ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py |  | test_main_threshold_warns_on_falls asserts the plan's exact fall(s) message template ('has 1 fall(s)'), not the literal word 'falls' | open |  | 2026-10-05T09:04:01.291Z |  |
| 2 | 01 | deviation | .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-08-PLAN.md |  | Task 3 verify #2's CMakeLists/package.xml clause compares to main, which dependency plan 01-04 legitimately changed (68885c2); plan 01-08 ran the equivalent check against the plan base - both files untouched by this plan | fixed |  | 2026-10-05T18:42:21.292Z | 2026-10-05T18:42:27.625Z |

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
  }
]
````
