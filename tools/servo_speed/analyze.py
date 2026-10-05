"""Offline servo speed analysis from INA219 current traces (CAL-17, D-08).

Finds the saturation speed of a servo from one bench-ramp recording (the
dog_bench ramp of plan 01-09 / the run of plan 01-12): the smallest commanded
speed after which the current traces of neighbour speeds stop differing is
``v_sat``; the candidate for ``servo.max_speed``. No ROS, numpy only; the CLI
and the CSV reader are added by the follow-up tasks of plan 01-06.

Input (rows of one recording, as produced by tools/servo_speed/synth.py):

* ``t_s`` - CLOCK_MONOTONIC seconds;
* ``stroke_id`` - stroke 0.. without gaps; -1 outside strokes; rows from the
  start of the command to the end of the hold after a stroke carry its number;
* ``direction`` - +1 when cmd_us rises inside a stroke and -1 when it falls,
  hold rows included; outside strokes (stroke_id -1: APPROACH and REST of the
  01-09 ramp) -1, 0 and +1 are all allowed;
* ``cmd_us`` - issued command pulse;
* ``shunt_raw`` - signed shunt register, LSB 10 uV, current = raw*10e-6/shunt_ohm;
* ``bus_raw`` - bus register as read: voltage in bits 15:3, LSB 4 mV, empty
  (None) when the bus was not read in that tick.

Steps: per stroke the current is aligned at the front of the current rise (the
0..20 ms start desync does not change v_sat); the base is the median of the
PRE_S window before the first command change (the tail of the previous
hold); traces are compared over WINDOW_S from the front; v_sat is the
smallest usable speed after which at least MIN_PAIRS consecutive
neighbour-speed pairs stay at or below
``eps = max(EPS_SIGMAS*sigma_rep, EPS_REL*I_plateau)`` (relative units; the
absolute shunt value cancels). A grid that never saturates reports
``not_saturated`` and null instead of the top grid speed.
"""

import math

import numpy as np

TICK_S = 0.001  # [s] sample step of the recording
# [s] base window before the stroke reference point. With hold_s >= 0.4 s the
# PRE_S = 0.15 window lies no earlier than 0.25 s after the end of the
# previous command, after the braking tail of the servo (up to 0.16 s at
# v_max 3.5 rad/s, 0.20 s at 3.0). A 0.30 s window caught that tail: sigma_b
# grew and fast strokes lost the front (prototype at v_max 4.0: 28 of 150
# strokes lost, v_sat None). Thresholds and tolerances were not changed.
PRE_S = 0.15
SEARCH_S = 1.3   # [s] recording window per stroke after the reference point
WINDOW_S = 0.6   # [s] trace comparison window from the front
SMOOTH_N = 5     # moving-average window (5 ms at TICK_S)
QUIET_N = 10     # quiet samples that end a stroke
LEVEL_S = 0.30   # [s] window after the reference point for the level
LEVEL_PCT = 95   # percentile of that window used as the level
FRONT_FRAC = 0.3    # front threshold: fraction of the level
FRONT_SIGMAS = 6.0  # front threshold: base sigmas
EPS_SIGMAS = 3.0    # eps floor: repetition sigmas
EPS_REL = 0.02      # eps floor: fraction of the plateau current
MIN_PAIRS = 3       # consecutive pairs that must stay at or below eps
MIN_STROKES = 2     # valid strokes per direction for a usable speed
AGREE_TOL = 0.15    # v_sat vs v_dur plateau: agreement tolerance
PLATEAU_TOL = 0.07  # top-three v_dur scatter allowed when v_sat is None
SHUNT_LSB_V = 10e-6  # [V] shunt register LSB
BUS_LSB_V = 0.004    # [V] bus register LSB


def find_saturation(speeds, distances, eps, min_pairs=MIN_PAIRS):
    """First speed after which the trace distance stays at the noise level.

    ``distances[j]`` is the distance between ``speeds[j]`` and
    ``speeds[j+1]``. Returns ``speeds[k]`` for the smallest k such that every
    j >= k has ``distances[j] <= eps`` and at least ``min_pairs`` pairs
    remain; None when there is no such k. Mismatched lengths raise
    ValueError.
    """
    if len(speeds) != len(distances) + 1:
        raise ValueError('speeds and distances lengths must differ by one')
    distances = list(distances)
    n = len(distances)
    for k in range(0, n - min_pairs + 1):
        if all(d <= eps for d in distances[k:]):
            return list(speeds)[k]
    return None


def extract_strokes(rows, meta):
    """Per-stroke, front-aligned current traces.

    Returns ``(strokes, rejected)``: a list of dicts with stroke_id,
    direction (sign of the first row of the group), trace, duration_s,
    plateau, angle_rad, and the number of stroke groups that could not be
    aligned (no command change, no front, no quiet end).
    """
    strokes = []
    rejected = 0
    if not rows:
        return strokes, rejected
    t = np.array([row[0] for row in rows], dtype=float)
    sid = np.array([int(row[1]) for row in rows], dtype=int)
    direction = np.array([int(row[2]) for row in rows], dtype=int)
    cmd = np.array([row[3] for row in rows], dtype=float)
    shunt = np.array([row[4] for row in rows], dtype=float)
    order = np.argsort(t, kind='stable')
    t, sid, direction, cmd, shunt = (a[order] for a in (t, sid, direction, cmd, shunt))
    us_per_deg = float(meta['us_per_deg'])

    n_target = int(round((PRE_S + SEARCH_S) / TICK_S)) + 1
    n_win = int(round(WINDOW_S / TICK_S))
    n_base = int(round((PRE_S - 0.02) / TICK_S)) + 1
    i0 = int(round(PRE_S / TICK_S))
    n_level = int(round(LEVEL_S / TICK_S))

    for uid in np.unique(sid[sid >= 0]):
        mask = sid == uid
        cmd_g = cmd[mask]
        t_g = t[mask]
        change = np.flatnonzero(np.abs(cmd_g - cmd_g[0]) > 0.5)
        if change.size == 0:
            rejected += 1
            continue
        t0 = t_g[change[0]]
        t_end = t_g[-1]
        t_hi = min(t0 + SEARCH_S, t_end)
        n_real = int(round((t_hi - (t0 - PRE_S)) / TICK_S)) + 1
        n_real = max(n_real, 1)
        grid = (t0 - PRE_S) + TICK_S * np.arange(n_real)
        vals = np.interp(grid, t, shunt)
        if n_real < n_target:
            vals = np.concatenate([vals, np.full(n_target - n_real, vals[-1])])
        else:
            vals = vals[:n_target]
        if n_base < 20:
            rejected += 1
            continue
        base = float(np.median(vals[:n_base]))
        x = np.convolve(vals - base, np.ones(SMOOTH_N) / SMOOTH_N, mode='same')
        sigma_b = float(np.std(x[:n_base]))
        level = float(np.percentile(x[i0:i0 + n_level], LEVEL_PCT))
        thr = max(FRONT_FRAC * level, FRONT_SIGMAS * sigma_b)
        hits = np.flatnonzero(x[i0 + 1:] >= thr)
        if hits.size == 0:
            rejected += 1
            continue
        f = i0 + 1 + int(hits[0])
        below = x < thr
        e = None
        for k in range(f + 10, len(x) - QUIET_N + 1):
            if below[k:k + QUIET_N].all():
                e = k
                break
        if e is None:
            rejected += 1
            continue
        duration = (e - f) * TICK_S
        if duration < 0.01:
            rejected += 1
            continue
        mid = (e - f) // 4
        plateau = float(np.median(x[f + mid:e - mid]))
        angle = math.radians(abs(cmd_g[-1] - cmd_g[0]) / us_per_deg)
        trace = x[f:f + n_win]
        if trace.size < n_win:
            trace = np.concatenate([trace, np.full(n_win - trace.size, trace[-1])])
        d = int(direction[mask][0])
        strokes.append({
            'stroke_id': int(uid),
            'direction': 1 if d > 0 else (-1 if d < 0 else 0),
            'trace': trace,
            'duration_s': float(duration),
            'plateau': float(plateau),
            'angle_rad': float(angle),
        })
    return strokes, rejected


def _kernel(strokes, meta, dirs):
    """Core over a set of directions: usable speeds, distances, eps, v_sat."""
    speeds = [float(s) for s in meta['speeds_rad_s']]
    per_speed = int(meta['strokes_per_speed'])
    by_speed = {}
    for st in strokes:
        by_speed.setdefault(st['stroke_id'] // per_speed, []).append(st)
    usable = []
    for si in sorted(by_speed):
        if si < 0 or si >= len(speeds):
            continue
        if all(sum(1 for st in by_speed[si] if st['direction'] == dd) >= MIN_STROKES
               for dd in dirs):
            usable.append(si)
    if len(usable) < 2:
        raise ValueError('too few usable strokes')

    mean_trace = {}
    sigmas = []
    for si in usable:
        for dd in dirs:
            traces = [st['trace'] for st in by_speed[si] if st['direction'] == dd]
            mean_trace[(si, dd)] = np.mean(np.stack(traces), axis=0)
            halves = [np.mean(np.stack(traces[0::2]), axis=0),
                      np.mean(np.stack(traces[1::2]), axis=0)]
            sigmas.append(float(np.sqrt(np.mean((halves[0] - halves[1]) ** 2))))
    sigma_rep = float(np.median(sigmas))

    distances = []
    for a, b in zip(usable[:-1], usable[1:]):
        parts = [float(np.mean((mean_trace[(a, dd)] - mean_trace[(b, dd)]) ** 2))
                 for dd in dirs]
        distances.append(float(np.sqrt(np.mean(parts))))

    means = [float(np.mean([st['plateau'] for st in by_speed[si]
                            if st['direction'] in dirs]))
             for si in usable[-3:]]
    i_plateau = float(np.median(means))
    if i_plateau <= 0:
        raise ValueError('no current step found')

    eps = max(EPS_SIGMAS * sigma_rep, EPS_REL * i_plateau)
    usable_speeds = [speeds[si] for si in usable]
    v_sat = find_saturation(usable_speeds, distances, eps)
    v_dur = []
    for si in usable:
        ds = [st['angle_rad'] / st['duration_s']
              for st in by_speed[si] if st['direction'] in dirs]
        v_dur.append(float(np.median(ds)))
    return {
        'usable_speeds': usable_speeds,
        'distances': distances,
        'eps': eps,
        'sigma_rep': sigma_rep,
        'i_plateau': i_plateau,
        'v_sat': v_sat,
        'v_dur': v_dur,
    }


def plateau_duration(speeds, v_dur, v_sat):
    """Plateau of the duration-derived speed (the second, independent estimate).

    With a v_sat: median v_dur over speeds >= v_sat. Without: median of the
    three largest speeds when their scatter (max - min)/median is within
    PLATEAU_TOL, else None (the plateau was not reached).
    """
    if v_sat is not None:
        vals = [d for s, d in zip(speeds, v_dur) if s >= v_sat]
        if not vals:
            return None
        return float(np.median(vals))
    top = [d for _s, d in list(zip(speeds, v_dur))[-3:]]
    if len(top) < 3:
        return None
    med = float(np.median(top))
    if med <= 0 or (max(top) - min(top)) / med > PLATEAU_TOL:
        return None
    return med


def cross_check(v_sat, v_dur_plateau, tol=AGREE_TOL):
    """Cross-check the kink against the duration plateau (D-08).

    Returns ``(agreement, servo_max_speed, flag)``: with both values and a
    relative difference within ``tol`` the kink wins; on disagreement the
    smaller (safe side) wins with the flag; a missing value is
    ``not_saturated``.
    """
    if v_sat is None or v_dur_plateau is None:
        return False, None, 'not_saturated'
    if abs(v_sat - v_dur_plateau) / max(v_sat, v_dur_plateau) <= tol:
        return True, float(v_sat), None
    return False, float(min(v_sat, v_dur_plateau)), 'v_sat_v_dur_disagree'


def analyze(rows, meta):
    """Full analysis of one recording; returns a json.dumps-ready dict."""
    strokes, rejected = extract_strokes(rows, meta)
    res = _kernel(strokes, meta, (1, -1))
    i_plateau = res['i_plateau']
    shunt_ohm = float(meta['shunt_ohm'])
    usable = res['usable_speeds']
    v_sat = res['v_sat']
    v_dur_plateau = plateau_duration(usable, res['v_dur'], v_sat)
    agreement, servo_max, cross_flag = cross_check(v_sat, v_dur_plateau)

    per_direction = {}
    for dd, name in ((1, 'up'), (-1, 'down')):
        try:
            kdir = _kernel(strokes, meta, (dd,))
            per_direction[name] = {
                'v_sat_rad_s': None if kdir['v_sat'] is None else float(kdir['v_sat']),
                'v_dur_plateau_rad_s': plateau_duration(kdir['usable_speeds'],
                                                        kdir['v_dur'], kdir['v_sat']),
                'plateau_current_a': float(kdir['i_plateau'] * SHUNT_LSB_V / shunt_ohm),
            }
        except ValueError:
            per_direction[name] = {'v_sat_rad_s': None,
                                   'v_dur_plateau_rad_s': None,
                                   'plateau_current_a': None}

    flags = []
    if cross_flag is not None:
        flags.append(cross_flag)
    if len(usable) < MIN_PAIRS + 1:
        flags.append('insufficient_data')
    if v_sat is not None and usable and v_sat == usable[0]:
        flags.append('saturated_at_first_speed')
    up_v = per_direction['up']['v_sat_rad_s']
    down_v = per_direction['down']['v_sat_rad_s']
    if (up_v is None) != (down_v is None):
        flags.append('direction_mismatch')
    elif up_v is not None:
        pos = {v: i for i, v in enumerate(usable)}
        if up_v not in pos or down_v not in pos or abs(pos[up_v] - pos[down_v]) > 1:
            flags.append('direction_mismatch')
    stop_reason = meta.get('stop_reason')
    if stop_reason not in (None, '', 'completed'):
        flags.append('run_incomplete:%s' % stop_reason)
    groups = {row[1] for row in rows if row[1] >= 0}
    if groups and rejected > 0.2 * len(groups):
        flags.append('many_strokes_rejected')

    bus_vals = [(row[5] >> 3) * BUS_LSB_V for row in rows if row[5] is not None]

    return {
        'v_sat_rad_s': None if v_sat is None else float(v_sat),
        'v_sat_status': 'ok' if v_sat is not None else 'not_saturated',
        'v_dur_plateau_rad_s': v_dur_plateau,
        'v_dur_rad_s': [float(d) for d in res['v_dur']],
        'agreement': bool(agreement),
        'servo_max_speed_rad_s': None if servo_max is None else float(servo_max),
        'flag': '; '.join(flags) if flags else None,
        'plateau_current_a': float(i_plateau * SHUNT_LSB_V / shunt_ohm),
        'noise_floor': float(res['sigma_rep'] / i_plateau),
        'bus_v_mean': float(np.mean(bus_vals)) if bus_vals else None,
        'per_direction': per_direction,
        'speeds_rad_s': [float(s) for s in usable],
        'distance_rel': [float(d / i_plateau) for d in res['distances']],
        'eps_rel': float(res['eps'] / i_plateau),
        'strokes': {'used': len(strokes), 'rejected': int(rejected)},
    }
