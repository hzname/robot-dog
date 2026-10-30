## jazzy / ideal

### Backward acceptance: jazzy / ideal (scoring)
Repeats requested: 5, commit: 83ce7c9b99ad

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 65.2% | 79.4% | 0 | 8.7 | 35.7 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 62.4% | 72.5% | 0 | 26.1 | 35.6 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 56.1% | 59.4% | 0 | 10.1 | 21.5 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 34.7% | 44.4% | 0 | 5.6 | 24.8 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 18.0% | 66.4% | 0 | 21.3 | 30.8 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 65.2% over n=5 on flat_A_bwd10, ideal model)

## jazzy / real

### Backward acceptance: jazzy / real (scoring)
Repeats requested: 5, commit: 83ce7c9b99ad

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 53.5% | 62.2% | 0 | 6.2 | 33.4 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 51.4% | 54.9% | 0 | 15.8 | 33.3 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 28.5% | 32.2% | 0 | 11.1 | 20.2 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 41.4% | 48.5% | 0 | 8.7 | 21.2 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 52.1% | 76.0% | 0 | 7.6 | 22.1 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)

## lyrical / ideal

### Backward acceptance: lyrical / ideal (reference only)
Repeats requested: 5, commit: 83ce7c9b99ad

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 52.4% | 61.4% | 0 | 9.1 | 35.5 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 73.2% | 82.9% | 0 | 22.6 | 35.3 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 50.4% | 57.5% | 0 | 9.7 | 21.8 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 38.8% | 44.1% | 0 | 3.8 | 24.2 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 59.2% | 69.4% | 0 | 22.6 | 25.9 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 52.4% over n=5 on flat_A_bwd10, ideal model)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## lyrical / real

### Backward acceptance: lyrical / real (reference only)
Repeats requested: 5, commit: 83ce7c9b99ad

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 45.2% | 58.4% | 0 | 5.9 | 33.3 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 49.7% | 57.6% | 0 | 22.4 | 33.3 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 12.6% | 27.4% | 0 | 9.2 | 20.2 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 42.4% | 49.6% | 0 | 6.6 | 20.5 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 35.9% | 51.6% | 0 | 11.6 | 20.5 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

