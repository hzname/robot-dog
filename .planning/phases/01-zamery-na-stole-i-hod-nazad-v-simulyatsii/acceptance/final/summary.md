## run A: jazzy / real

### Backward acceptance: jazzy / real (scoring)
Repeats requested: 5, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 61.9% | 65.6% | 0 | 10.8 | 33.7 | FAIL |
| flat_B_bwd10 | score_b | 5 | 0 | 48.2% | 53.1% | 0 | 19.2 | 33.8 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 14.1% | 27.9% | 0 | 9.1 | 20.4 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 37.0% | 43.0% | 0 | 10.2 | 22.5 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 46.7% | 60.3% | 0 | 13.4 | 27.1 | PASS |

Verdict: FAIL
- flat_A_bwd10: max/dyaw5/=10.8 deg > 10.0

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)

## run B: jazzy / real

### Backward acceptance: jazzy / real (scoring)
Repeats requested: 10, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 50.2% | 57.1% | 0 | 6.8 | 33.8 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)

## run A: jazzy / ideal

### Backward acceptance: jazzy / ideal (scoring)
Repeats requested: 5, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 43.5% | 69.4% | 0 | 8.5 | 35.5 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 75.6% | 78.6% | 0 | 22.1 | 35.4 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 56.3% | 61.7% | 0 | 7.9 | 21.3 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 60.5% | 70.2% | 0 | 3.1 | 24.4 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 51.3% | 78.4% | 0 | 4.3 | 30.0 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.30 (min ratio 43.5% over n=5 on flat_A_bwd10, ideal model)

## run B: jazzy / ideal

### Backward acceptance: jazzy / ideal (scoring)
Repeats requested: 10, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 64.9% | 84.7% | 0 | 7.9 | 33.5 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 64.9% over n=10 on flat_A_bwd10, ideal model)

## run A: lyrical / real

### Backward acceptance: lyrical / real (reference only)
Repeats requested: 5, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 43.0% | 61.0% | 0 | 4.0 | 36.6 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 29.7% | 50.3% | 0 | 21.5 | 36.4 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 26.7% | 31.6% | 0 | 7.9 | 22.5 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 38.3% | 50.3% | 0 | 8.7 | 25.1 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 52.5% | 60.0% | 0 | 5.9 | 27.8 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## run B: lyrical / real

### Backward acceptance: lyrical / real (reference only)
Repeats requested: 10, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 46.0% | 56.3% | 0 | 8.7 | 36.0 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## run A: lyrical / ideal

### Backward acceptance: lyrical / ideal (reference only)
Repeats requested: 5, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 54.4% | 72.5% | 0 | 11.3 | 35.8 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 60.9% | 76.0% | 0 | 25.1 | 35.7 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 46.0% | 51.2% | 0 | 7.6 | 22.1 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 34.3% | 42.0% | 0 | 11.9 | 24.4 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 55.0% | 79.2% | 0 | 6.9 | 27.2 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 54.4% over n=5 on flat_A_bwd10, ideal model)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## run B: lyrical / ideal

### Backward acceptance: lyrical / ideal (reference only)
Repeats requested: 10, commit: 0917551cda72

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 54.7% | 68.0% | 0 | 15.1 | 35.6 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 54.7% over n=10 on flat_A_bwd10, ideal model)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

