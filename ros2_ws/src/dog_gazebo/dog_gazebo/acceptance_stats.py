"""acceptance_stats: pure-stdlib statistics for the backward-walk acceptance
(GAIT-01, GAIT-02).

No ROS and no simulation here: pytest runs this module on Python 3.12 and
3.14 without rclpy or numpy. A repeat is one walk_check --trace result,
classified as ok | fell | no_stand | error and summarised over at least five
repeats (minimum and median, D-03); the D-01, D-06 and D-07 rules turn the
summary into reasons and a pass verdict, and the D-05 push-CI threshold is
derived from the minimum of cell flat_A_bwd10 on the ideal model:

  python3 -m dog_gazebo.acceptance_stats threshold RESULT.json [...]

prints `distro=<d> min_ratio=<m> push_threshold=<v>`; exit 0 when a threshold
is derived, 1 when it cannot be, 2 for an unreadable file or a schema that is
not 1. Nothing is written to disk. The acceptance CLI (plan 01-11), the CI job
(plan 01-13) and the push-CI threshold edit (plan 01-17) call these functions.
"""

import json
import math
import re
import statistics

from dog_gazebo.terrain_sweep import never_stood

# ---------- constants

SCHEMA_VERSION = 1
MIN_REPEATS = 5
# Equals walk_check.MIN_BODY_HEIGHT: a 42 mm sag under the 0.15 m stand height.
MIN_BODY_HEIGHT = 0.108
MAX_TILT_DEG = 20.0
MAX_DYAW5_DEG = 10.0
MIN_RATIO = 0.40
# The worst_tilt threshold in WalkCheck.spin (walk_check.py): a repeat that
# tilted past it counts as a fall even when the last manoeuvre left no
# fallen=True flag behind.
FALL_TILT_DEG = 60.0
# The current push-CI threshold (walk_check --backward-ratio 0.2, D-06): the
# floor ratio for a reference distro instead of the 40 % requirement.
DEFAULT_FLOOR_RATIO = 0.2

DISTROS = ('jazzy', 'lyrical')
SCORING = {'jazzy': True, 'lyrical': False}  # D-06: Jazzy is the scoring distro
SERVO_MODELS = ('ideal', 'real')
STATUSES = ('ok', 'fell', 'no_stand', 'error')
PUSH_CELL = 'flat_A_bwd10'

# D-03 cells in order: terrain, level, mode, cmd_vx, maneuvers, rule. Mode B
# (heading_hold:=false slope_compensation:=false) and the rocks10 seed per
# repeat are applied by the plan 01-11 CLI, not by this module.
CELLS = {
    'flat_A_bwd10': {'terrain': 'flat', 'level': 0, 'mode': 'A', 'cmd_vx': -0.10,
                     'maneuvers': ('backward', 'left', 'right'), 'rule': 'score_a'},
    'flat_B_bwd10': {'terrain': 'flat', 'level': 0, 'mode': 'B', 'cmd_vx': -0.10,
                     'maneuvers': ('backward', 'left', 'right'), 'rule': 'score_b'},
    'flat_B_bwd05': {'terrain': 'flat', 'level': 0, 'mode': 'B', 'cmd_vx': -0.05,
                     'maneuvers': ('backward',), 'rule': 'report'},
    'waves10_A_bwd10': {'terrain': 'waves', 'level': 10, 'mode': 'A', 'cmd_vx': -0.10,
                        'maneuvers': ('backward',), 'rule': 'terrain'},
    'rocks10_A_bwd10': {'terrain': 'rough', 'level': 10, 'mode': 'A', 'cmd_vx': -0.10,
                        'maneuvers': ('backward',), 'rule': 'terrain'},
}


# ---------- numbers

def _num(value):
    """A finite float for int/float input (bool excluded); None otherwise."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    out = float(value)
    return out if math.isfinite(out) else None


# ---------- classification

def classify_run(data):
    """One repeat status: ok | fell | no_stand | error.

    error    - not a dict, no non-empty results list, a truthy error field, or
               a record that is not a dict with a name;
    no_stand - the stand request never reached locomotion (never_stood): an
               infrastructure failure, not a repeat (a replacement is run);
    fell     - a record carries fallen=True or tilted past FALL_TILT_DEG
               (maneuver() keeps the worst tilt, so a fall in the last
               manoeuvre is visible without the skipped-record flag);
    ok       - everything else.
    """
    if not isinstance(data, dict):
        return 'error'
    results = data.get('results')
    if not isinstance(results, list) or not results or data.get('error'):
        return 'error'
    for rec in results:
        if not isinstance(rec, dict) or 'name' not in rec:
            return 'error'
    if never_stood(data):
        return 'no_stand'
    for rec in results:
        tilt = _num(rec.get('tilt_deg'))
        if rec.get('fallen') is True or (tilt is not None and tilt > FALL_TILT_DEG):
            return 'fell'
    return 'ok'


# ---------- repeat records

def run_record(data, wall_s=None):
    """One runs[] entry from a walk_check --trace dict - the only translator.

    ratio is the backward manoeuvre's share of the command; ratios and
    dyaw5_deg are keyed by manoeuvre; tilt_deg is the worst and z the lowest
    over the manoeuvres (records with a ratio; stand and lie do not count).
    """
    results = data.get('results') if isinstance(data, dict) else None
    if not isinstance(results, list):
        results = []
    ratio = None
    for rec in results:
        if isinstance(rec, dict) and rec.get('name') == 'backward':
            ratio = _num(rec.get('ratio'))
            break
    ratios, dyaw5, tilts, zs = {}, {}, [], []
    for rec in results:
        if not isinstance(rec, dict) or 'ratio' not in rec:
            continue  # manoeuvre records carry a ratio; stand and lie do not
        name = rec.get('name')
        ratios[name] = _num(rec.get('ratio'))
        if 'dyaw5_deg' in rec:
            dyaw5[name] = _num(rec.get('dyaw5_deg'))
        tilt = _num(rec.get('tilt_deg'))
        if tilt is not None:
            tilts.append(tilt)
        z = _num(rec.get('z'))
        if z is not None:
            zs.append(z)
    return {'status': classify_run(data), 'ratio': ratio, 'ratios': ratios,
            'dyaw5_deg': dyaw5, 'tilt_deg': max(tilts) if tilts else None,
            'z': min(zs) if zs else None, 'wall_s': _num(wall_s)}


# ---------- summary

def summarize_cell(cell, runs, distro, floor_ratio=DEFAULT_FLOOR_RATIO):
    """Summarise the repeats of one cell: n, min/median, falls, pass, reasons.

    cell is a CELLS name or a dict with the same fields; runs are run_record
    dicts. Valid repeats are ok and fell; n_invalid is no_stand plus error.
    Reasons carry the D-01, D-06 and D-07 rules. pass is None for a report
    cell ('report only: not scored') and for fewer than MIN_REPEATS valid
    repeats ('insufficient data' first); otherwise pass is not reasons.
    Missing data (dyaw5_deg, the backward ratio, tilt_deg and z on terrain) is
    a failure reason, never a skipped check.
    """
    if isinstance(cell, str):
        if cell not in CELLS:
            raise ValueError('unknown cell %r' % cell)
        cell = CELLS[cell]
    elif not isinstance(cell, dict):
        raise ValueError('cell must be a CELLS name or a cell dict')
    if distro not in DISTROS:
        raise ValueError('distro must be one of %s' % (list(DISTROS),))
    floor = _num(floor_ratio)
    if floor is None:
        raise ValueError('floor_ratio must be a finite number')
    if not isinstance(runs, list):
        raise ValueError('runs must be a list of records')
    valid = []
    n_invalid = 0
    for run in runs:
        status = run.get('status') if isinstance(run, dict) else None
        if status not in STATUSES:
            raise ValueError('status must be one of %s' % (list(STATUSES),))
        if status in ('ok', 'fell'):
            valid.append(run)
        else:
            n_invalid += 1
    ok_runs = [r for r in valid if r['status'] == 'ok']
    falls = sum(1 for r in valid if r['status'] == 'fell')

    ratios = [v for v in (_num(r.get('ratio')) for r in valid) if v is not None]
    ratio_min = min(ratios) if ratios else None
    ratio_median = statistics.median(ratios) if ratios else None

    maneuvers = cell['maneuvers']
    rule = cell['rule']
    max_abs = None
    missing_dyaw5 = 0
    for run in valid:
        dyaw = run.get('dyaw5_deg')
        dyaw = dyaw if isinstance(dyaw, dict) else {}
        for name in maneuvers:
            value = _num(dyaw.get(name))
            if value is None:
                if run['status'] == 'ok':
                    missing_dyaw5 += 1
            else:
                max_abs = abs(value) if max_abs is None else max(max_abs, abs(value))

    if rule == 'report':
        return {'n': len(valid), 'n_invalid': n_invalid, 'ratio_min': ratio_min,
                'ratio_median': ratio_median, 'falls': falls,
                'max_abs_dyaw5_deg': max_abs, 'pass': None,
                'reasons': ['report only: not scored']}

    reasons = []
    if falls:
        reasons.append('falls=%d' % falls)
    if rule in ('score_a', 'score_b'):
        needed = MIN_RATIO if SCORING[distro] else floor
        missing_ratio = sum(1 for r in ok_runs if _num(r.get('ratio')) is None)
        if ratio_min is None:
            reasons.append('no ratio data')
        elif ratio_min < needed:
            reasons.append('ratio_min=%.3f < %.2f' % (ratio_min, needed))
        if missing_ratio:
            reasons.append('backward ratio missing in %d run(s)' % missing_ratio)
    if rule == 'score_a' and SCORING[distro]:
        if missing_dyaw5:
            reasons.append('dyaw5 missing in %d case(s)' % missing_dyaw5)
        if max_abs is not None and max_abs > MAX_DYAW5_DEG:
            reasons.append('max|dyaw5|=%.1f deg > %.1f' % (max_abs, MAX_DYAW5_DEG))
    if rule == 'terrain':
        tilt_bad = 0
        z_bad = 0
        data_missing = 0
        for run in ok_runs:
            tilt = _num(run.get('tilt_deg'))
            z = _num(run.get('z'))
            if tilt is not None and tilt >= MAX_TILT_DEG:
                tilt_bad += 1
            if z is not None and z <= MIN_BODY_HEIGHT:
                z_bad += 1
            if tilt is None or z is None:
                data_missing += 1
        if tilt_bad:
            reasons.append('tilt_deg >= 20.0 in %d run(s)' % tilt_bad)
        if z_bad:
            reasons.append('z <= 0.108 in %d run(s)' % z_bad)
        if data_missing:
            reasons.append('tilt_deg/z missing in %d run(s)' % data_missing)

    if len(valid) < MIN_REPEATS:
        reasons.insert(0, 'insufficient data: n=%d < %d valid repeats'
                       % (len(valid), MIN_REPEATS))
        passed = None
    else:
        passed = not reasons
    return {'n': len(valid), 'n_invalid': n_invalid, 'ratio_min': ratio_min,
            'ratio_median': ratio_median, 'falls': falls,
            'max_abs_dyaw5_deg': max_abs, 'pass': passed, 'reasons': reasons}


# ---------- push-CI threshold (D-05)

def derive_push_threshold(min_ratio, lo=0.2, hi=0.4, step=0.05, margin=0.8):
    """Push-CI --backward-ratio from the minimum over >= 5 repeats (D-05):
    clamp(floor_to_step(margin * min_ratio), lo, hi); 0.52 -> 0.40,
    0.45 -> 0.35, 0.30 -> 0.20 (the floor is never lowered). The round(..., 9)
    absorbs floating point noise at a step boundary (0.4375 -> 0.35).
    """
    value = _num(min_ratio)
    if value is None:
        raise ValueError('min_ratio must be a finite number')
    raw = math.floor(round(margin * value / step, 9)) * step
    return round(min(hi, max(lo, raw)), 2)
