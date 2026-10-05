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

import argparse
import csv
import json
import math
import os
import sys

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
MAX_ROWS = 5000000  # row cap of load_csv (a full ramp recording is ~1e5 rows)
SHUNT_LSB_V = 10e-6  # [V] shunt register LSB
BUS_LSB_V = 0.004    # [V] bus register LSB

CSV_FIELDS = ('t_s', 'stroke_id', 'direction', 'cmd_us', 'shunt_raw', 'bus_raw')
CSV_HEADER = ','.join(CSV_FIELDS)


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


def _float_field(text, lineno, field):
    try:
        value = float(text)
    except ValueError:
        raise ValueError('line %d: %s is not a number' % (lineno, field))
    if not math.isfinite(value):
        raise ValueError('line %d: %s must be finite' % (lineno, field))
    return value


def _int_field(text, lineno, field):
    try:
        return int(text)
    except ValueError:
        raise ValueError('line %d: %s is not an integer' % (lineno, field))


def load_csv(path):
    """Read a recording CSV into rows (t_s, stroke_id, direction, cmd_us,
    shunt_raw, bus_raw); an empty bus_raw becomes None. ValueError names the
    file line and the field; see the module docstring for the contract."""
    rows = []
    with open(path, newline='', encoding='utf-8') as fh:
        reader = csv.reader(fh)
        try:
            header = next(reader)
        except StopIteration:
            raise ValueError('line 1: expected header %s' % CSV_HEADER)
        if [cell.strip() for cell in header] != list(CSV_FIELDS):
            raise ValueError('line 1: header must be %s' % CSV_HEADER)
        for lineno, parts in enumerate(reader, start=2):
            if len(rows) >= MAX_ROWS:
                raise ValueError('line %d: more than %d data rows' % (lineno, MAX_ROWS))
            if len(parts) != len(CSV_FIELDS):
                raise ValueError('line %d: expected %d fields' % (lineno, len(CSV_FIELDS)))
            t_s = _float_field(parts[0], lineno, 't_s')
            stroke_id = _int_field(parts[1], lineno, 'stroke_id')
            direction = _int_field(parts[2], lineno, 'direction')
            cmd_us = _float_field(parts[3], lineno, 'cmd_us')
            shunt_raw = _int_field(parts[4], lineno, 'shunt_raw')
            bus_raw = None
            if parts[5].strip() != '':
                bus_raw = _int_field(parts[5], lineno, 'bus_raw')
            if stroke_id < -1:
                raise ValueError('line %d: stroke_id %d is below -1' % (lineno, stroke_id))
            if stroke_id >= 0:
                if direction not in (1, -1):
                    raise ValueError('line %d: direction %d inside a stroke must be '
                                     '+1 or -1' % (lineno, direction))
            elif direction not in (-1, 0, 1):
                raise ValueError('line %d: direction %d outside a stroke must be '
                                 '-1, 0 or +1' % (lineno, direction))
            rows.append((t_s, stroke_id, direction, cmd_us, shunt_raw, bus_raw))
    return rows


def validate_meta(meta):
    """Validate run metadata; ValueError names the offending field.

    Required: shunt_ohm, us_per_deg, speeds_rad_s (strictly ascending, at
    least 4), strokes_per_speed (even, at least 4); bool is never a number.
    amp_deg, center_us, channel, hold_s, rest_s, ina_config and stop_reason
    stay optional.
    """
    if not isinstance(meta, dict):
        raise ValueError('meta must be a JSON object')
    for field in ('shunt_ohm', 'us_per_deg', 'speeds_rad_s', 'strokes_per_speed'):
        if field not in meta:
            raise ValueError('meta field "%s" is required' % field)
    for field in ('shunt_ohm', 'us_per_deg'):
        value = meta[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('"%s" must be a number' % field)
        if not math.isfinite(value) or value <= 0:
            raise ValueError('"%s" must be positive' % field)
    speeds = meta['speeds_rad_s']
    if not isinstance(speeds, list) or len(speeds) < 4:
        raise ValueError('"speeds_rad_s" must list at least 4 speeds')
    for value in speeds:
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value):
            raise ValueError('"speeds_rad_s" must contain finite numbers')
    for low, high in zip(speeds[:-1], speeds[1:]):
        if not low < high:
            raise ValueError('"speeds_rad_s" must be strictly ascending')
    strokes_per_speed = meta['strokes_per_speed']
    if isinstance(strokes_per_speed, bool) or not isinstance(strokes_per_speed, int):
        raise ValueError('"strokes_per_speed" must be an integer')
    if strokes_per_speed < 4 or strokes_per_speed % 2:
        raise ValueError('"strokes_per_speed" must be even and at least 4')


def load_meta(path):
    """Read a <csv>.meta.json file and validate it."""
    with open(path, encoding='utf-8') as fh:
        meta = json.load(fh)
    validate_meta(meta)
    return meta


def analyze(rows, meta):
    """Full analysis of one recording; returns a json.dumps-ready dict."""
    validate_meta(meta)
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


def plot_result(result, path):
    """Write the two-panel graph of one result.

    Top: distance between neighbour traces vs the upper speed of the pair,
    log Y, with the eps line and the v_sat line. Bottom: duration-derived
    speed vs the grid, the y = x diagonal and the v_dur plateau. matplotlib
    is imported here only, so the analysis itself stays numpy-only.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    speeds = result['speeds_rad_s']
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(8.0, 8.0))
    top.semilogy(speeds[1:], result['distance_rel'], marker='o')
    top.axhline(result['eps_rel'], color='red', linestyle='--', label='eps')
    if result['v_sat_rad_s'] is not None:
        top.axvline(result['v_sat_rad_s'], color='green', linestyle='--',
                    label='v_sat')
    top.set_xlabel('commanded speed [rad/s]')
    top.set_ylabel('trace distance (relative)')
    top.set_title('Distance between neighbour traces vs speed')
    top.legend()
    bottom.plot(speeds, result['v_dur_rad_s'], marker='o', label='v_dur')
    bottom.plot(speeds, speeds, linestyle=':', color='gray', label='v_dur = v_cmd')
    if result['v_dur_plateau_rad_s'] is not None:
        bottom.axhline(result['v_dur_plateau_rad_s'], color='orange', linestyle='--',
                       label='v_dur plateau')
    bottom.set_xlabel('commanded speed [rad/s]')
    bottom.set_ylabel('duration-derived speed [rad/s]')
    bottom.set_title('Stroke duration vs speed')
    bottom.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _fmt(value):
    return 'None' if value is None else '%g' % value


def main(argv=None):
    """CLI: analyze a recording CSV; exit 0 ok / 1 not saturated / 2 input error."""
    parser = argparse.ArgumentParser(
        description='Find the servo saturation speed from INA219 current traces (CAL-17).')
    parser.add_argument('--csv', required=True,
                        help='recording CSV: %s' % CSV_HEADER)
    parser.add_argument('--meta', help='run metadata JSON (default: <csv>.meta.json)')
    parser.add_argument('--out', help='write the result JSON here')
    parser.add_argument('--plot', help='write the graph here (requires matplotlib)')
    args = parser.parse_args(argv)
    meta_path = args.meta if args.meta else args.csv + '.meta.json'
    try:
        protected = (os.path.realpath(args.csv), os.path.realpath(meta_path))
        for target in (args.out, args.plot):
            if target and os.path.realpath(target) in protected:
                print('analyze: --out/--plot must not overwrite --csv/--meta',
                      file=sys.stderr)
                return 2
        rows = load_csv(args.csv)
        meta = load_meta(meta_path)
        result = analyze(rows, meta)
        if args.plot:
            try:
                plot_result(result, args.plot)
            except ImportError:
                print('analyze: --plot needs matplotlib: '
                      'python3 -m pip install matplotlib', file=sys.stderr)
                return 2
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as fh:
                json.dump(result, fh, indent=2, sort_keys=True)
    except (ValueError, OSError) as exc:
        print('analyze: %s' % exc, file=sys.stderr)
        return 2
    if result['v_sat_rad_s'] is None:
        print('v_sat: not saturated')
    else:
        print('v_sat = %s rad/s (%s)' % (_fmt(result['v_sat_rad_s']),
                                         result['v_sat_status']))
    print('v_dur plateau = %s rad/s' % _fmt(result['v_dur_plateau_rad_s']))
    print('agreement: %s' % ('yes' if result['agreement'] else 'no'))
    print('servo.max_speed candidate = %s rad/s' % _fmt(result['servo_max_speed_rad_s']))
    if result['flag']:
        print('WARN flag: %s' % result['flag'])
    print('bus_v_mean = %s' % _fmt(result['bus_v_mean']))
    return 0 if result['v_sat_status'] == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
