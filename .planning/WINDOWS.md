---
schema_version: 1
open_count: 1
waived_count: 0
fixed_count: 0
total_count: 1
last_updated: 2026-10-05T09:04:01.291Z
---

# Broken Windows Ledger

> Cross-phase defect register. With `workflow.windows_enforce` enabled, `/gsd-ship` blocks while `open_count > 0`.
> Waive with `gsd-tools windows waive <id> "<reason>"` (reason required).
> Mark fixed with `gsd-tools windows fixed <id>`.

| id | phase | kind | file | line | description | status | reason | recorded_at | resolved_at |
|----|-------|------|------|------|-------------|--------|--------|-------------|-------------|
| 1 | 01 | deviation | ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py |  | test_main_threshold_warns_on_falls asserts the plan's exact fall(s) message template ('has 1 fall(s)'), not the literal word 'falls' | open |  | 2026-10-05T09:04:01.291Z |  |

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
  }
]
````
