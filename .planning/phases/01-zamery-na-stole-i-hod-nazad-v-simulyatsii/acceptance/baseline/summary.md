## run A: jazzy / ideal

### Backward acceptance: jazzy / ideal (scoring)
Repeats requested: 5, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 50.3% | 57.3% | 0 | 7.2 | 34.6 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 55.4% | 78.4% | 0 | 31.2 | 34.5 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 62.9% | 66.1% | 0 | 13.5 | 20.4 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 34.6% | 60.9% | 0 | 5.0 | 23.3 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 45.8% | 58.0% | 0 | 7.0 | 28.5 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 50.3% over n=5 on flat_A_bwd10, ideal model)

## run A: jazzy / real

### Backward acceptance: jazzy / real (scoring)
Repeats requested: 5, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 50.4% | 61.3% | 0 | 4.7 | 33.3 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 49.3% | 55.0% | 0 | 19.4 | 33.0 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 20.9% | 28.8% | 0 | 13.4 | 19.5 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 50.9% | 60.1% | 0 | 7.5 | 21.7 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 52.5% | 54.7% | 0 | 8.9 | 25.7 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)

## run A: lyrical / ideal

### Backward acceptance: lyrical / ideal (reference only)
Repeats requested: 5, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 51.1% | 73.1% | 0 | 10.1 | 34.4 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 56.1% | 68.9% | 0 | 21.3 | 34.3 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 55.8% | 61.7% | 0 | 14.3 | 20.8 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 38.5% | 38.8% | 0 | 9.3 | 22.9 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 42.8% | 68.4% | 0 | 6.9 | 24.2 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 51.1% over n=5 on flat_A_bwd10, ideal model)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## run A: lyrical / real

### Backward acceptance: lyrical / real (reference only)
Repeats requested: 5, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 5 | 0 | 49.1% | 59.3% | 0 | 3.7 | 32.9 | PASS |
| flat_B_bwd10 | score_b | 5 | 0 | 50.2% | 53.6% | 0 | 26.0 | 33.1 | PASS |
| flat_B_bwd05 | report | 5 | 0 | 6.4% | 21.7% | 0 | 11.1 | 19.9 | report |
| waves10_A_bwd10 | terrain | 5 | 0 | 37.6% | 50.3% | 0 | 8.4 | 21.4 | PASS |
| rocks10_A_bwd10 | terrain | 5 | 0 | 47.1% | 51.4% | 0 | 3.2 | 23.1 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## run B: jazzy / ideal

### Backward acceptance: jazzy / ideal (scoring)
Repeats requested: 10, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 47.8% | 70.3% | 0 | 6.8 | 34.6 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.35 (min ratio 47.8% over n=10 on flat_A_bwd10, ideal model)

## run B: jazzy / real

### Backward acceptance: jazzy / real (scoring)
Repeats requested: 10, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 42.1% | 61.2% | 0 | 7.5 | 35.6 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)

## run B: lyrical / ideal

### Backward acceptance: lyrical / ideal (reference only)
Repeats requested: 10, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 54.0% | 70.5% | 0 | 9.4 | 34.4 | PASS |

Verdict: PASS

Push-CI threshold (D-05): --backward-ratio 0.40 (min ratio 54.0% over n=10 on flat_A_bwd10, ideal model)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.

## run B: lyrical / real

### Backward acceptance: lyrical / real (reference only)
Repeats requested: 10, commit: adcf83467cfb

| cell | rule | n | invalid | ratio min | ratio median | falls | max abs dyaw5, deg | wall_s median | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_A_bwd10 | score_a | 10 | 0 | 37.1% | 59.8% | 0 | 6.2 | 34.5 | PASS |

Verdict: PASS

Push-CI threshold: not derivable (needs servo_model ideal, cell flat_A_bwd10, at least 5 valid repeats)
Reference distro (D-06): no 40%% requirement, floor ratio applies; results do not gate the phase.
