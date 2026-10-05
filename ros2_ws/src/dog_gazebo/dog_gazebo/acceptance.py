"""acceptance: repeat the backward-walk acceptance cells with a fresh
simulation each time (GAIT-01, GAIT-02, D-03).

CI only: simulations belong in GitHub Actions (D-04, D-24) - a local run is
warned about and distorts the results. One repeat is one fresh
`terrain_sweep.run_level` launch on its own ROS domain; the walk_check tail
keeps the acceptance thresholds out of the run (`--min-ratio 0
--backward-ratio 0`, D-17) and the scoring lives in acceptance_stats only.
Results are written after every repeat (schema 1 JSON + the markdown
summary), so an interrupted job keeps what it measured.

  ros2 run dog_gazebo acceptance --distro jazzy --repeats 5 --cells all
  python3 -m dog_gazebo.acceptance --dry-run --cells flat_B_bwd05 --repeats 2

The real run happens in the CI acceptance job (plan 01-13); locally only
--dry-run and the pure functions are exercised.
"""

import argparse
import json
import math
import os
import sys
import time

from dog_gazebo import acceptance_stats as stats
from dog_gazebo.terrain_sweep import never_stood, run_level

# ---------- constants

DOMAIN_BASE = 80
DOMAIN_SPAN = 10
MAX_REPEATS = 20
DEFAULT_REPEATS = stats.MIN_REPEATS
ATTEMPT_FACTOR = 2
# base + DOMAIN_SPAN - 1 must stay inside the ROS domain range 0..101.
MAX_DOMAIN = 101 - (DOMAIN_SPAN - 1)


# ---------- cell argument builders

def cell_launch_args(cell, servo_model):
    """sim.launch.py tail for a cell: mode B drops the heading hold and the
    slope compensation; the real servo model is selected last."""
    if cell not in stats.CELLS:
        raise ValueError('unknown cell %r' % (cell,))
    if servo_model not in stats.SERVO_MODELS:
        raise ValueError('unknown servo model %r' % (servo_model,))
    args = []
    if stats.CELLS[cell]['mode'] == 'B':
        args += ['heading_hold:=false', 'slope_compensation:=false']
    if servo_model == 'real':
        args.append('servo_model:=real')
    return args


def cell_walk_check_args(cell):
    """walk_check tail for a cell: the subset, the speed and zero thresholds
    (the walk_check ok flag must not enter the verdict, D-17)."""
    if cell not in stats.CELLS:
        raise ValueError('unknown cell %r' % (cell,))
    data = stats.CELLS[cell]
    return ['--maneuvers', ','.join(data['maneuvers']),
            '--backward-speed', '%g' % abs(data['cmd_vx']),
            '--min-ratio', '0', '--backward-ratio', '0']


def domain_for(base, launch_no):
    """The ROS domain of a launch: base + launch_no wrapping over 80..89
    (60-77 belong to the terrain job, 41-43 to the launch tests)."""
    return base + launch_no % DOMAIN_SPAN


def seed_for(cell, attempt):
    """The layout seed: the attempt number on rough ground, 0 elsewhere."""
    return attempt if cell['terrain'] == 'rough' else 0


def plan_runs(cells, repeats):
    """The launch plan: {'cell', 'repeat', 'seed'} in CELLS order."""
    for name in cells:
        if name not in stats.CELLS:
            raise ValueError('unknown cell %r' % (name,))
    records = []
    for name in stats.CELLS:
        if name not in cells:
            continue
        for repeat in range(1, repeats + 1):
            records.append({'cell': name, 'repeat': repeat,
                            'seed': seed_for(stats.CELLS[name], repeat)})
    return records


# ---------- arguments

def _positive_float(text):
    """--floor-ratio: a finite share in (0, 1]."""
    value = float(text)
    if not math.isfinite(value) or not 0 < value <= 1:
        raise argparse.ArgumentTypeError('must be in (0, 1], got %r' % (text,))
    return value


def _repeats(text):
    """--repeats: 1..MAX_REPEATS."""
    value = int(text)
    if not 1 <= value <= MAX_REPEATS:
        raise argparse.ArgumentTypeError('must be in 1..%d, got %r' % (MAX_REPEATS, text))
    return value


def _domain(text):
    """--domain: a base that keeps base + DOMAIN_SPAN - 1 <= 101."""
    value = int(text)
    if not 0 <= value <= MAX_DOMAIN:
        raise argparse.ArgumentTypeError('must be in 0..%d (base + %d <= 101)'
                                         % (MAX_DOMAIN, DOMAIN_SPAN - 1))
    return value


def _cells(text):
    """--cells: `all` or known names separated by commas, no duplicates."""
    if text == 'all':
        return tuple(stats.CELLS)
    names = text.split(',')
    if not names or any(not name for name in names):
        raise argparse.ArgumentTypeError('give cell names separated by commas or all')
    unknown = [name for name in names if name not in stats.CELLS]
    if unknown:
        raise argparse.ArgumentTypeError('unknown cell(s) %s (known: %s)'
                                         % (', '.join(unknown), ', '.join(stats.CELLS)))
    if len(set(names)) != len(names):
        raise argparse.ArgumentTypeError('duplicate cell name in %r' % (text,))
    wanted = set(names)
    return tuple(name for name in stats.CELLS if name in wanted)


def build_parser(environ=None):
    """The acceptance parser; environ supplies the --distro and --git-sha
    defaults (the CI job passes them through env, not through run:)."""
    env = os.environ if environ is None else environ
    ap = argparse.ArgumentParser(
        prog='acceptance',
        description='Backward-walk acceptance: fresh simulations per repeat (CI only, D-04/D-24).')
    ap.add_argument('--repeats', type=_repeats, default=DEFAULT_REPEATS,
                    help='valid repeats per cell, 1..%d (default: %d)'
                    % (MAX_REPEATS, DEFAULT_REPEATS))
    ap.add_argument('--cells', type=_cells, default=tuple(stats.CELLS),
                    help='comma separated cell names or all (default: all)')
    ap.add_argument('--servo-model', choices=stats.SERVO_MODELS, default='ideal',
                    help='ideal | real (default: ideal)')
    ap.add_argument('--distro', choices=stats.DISTROS, default=env.get('ROS_DISTRO'),
                    help='jazzy | lyrical (default: $ROS_DISTRO)')
    ap.add_argument('--domain', type=_domain, default=DOMAIN_BASE,
                    help='first ROS domain, wrapping over %d (default: %d)'
                    % (DOMAIN_SPAN, DOMAIN_BASE))
    ap.add_argument('--floor-ratio', type=_positive_float, default=stats.DEFAULT_FLOOR_RATIO,
                    help='reference-distro floor ratio (default: %.2f)' % stats.DEFAULT_FLOOR_RATIO)
    ap.add_argument('--out', default=None,
                    help='result JSON (default: acceptance_<distro>_<servo_model>.json)')
    ap.add_argument('--summary', default=None,
                    help='markdown summary (default: --out without .json + _summary.md)')
    ap.add_argument('--git-sha', default=env.get('GITHUB_SHA', ''),
                    help='commit id for the summary (default: $GITHUB_SHA)')
    ap.add_argument('--strict', action='store_true',
                    help='exit 1 unless the verdict passes')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the launch plan and exit without running anything')
    return ap


def parse_args(argv=None, environ=None):
    """Parse the CLI; argparse does not validate the --distro default, so the
    $ROS_DISTRO fallback is checked after parsing."""
    env = os.environ if environ is None else environ
    ap = build_parser(env)
    args = ap.parse_args(argv)
    if args.distro not in stats.DISTROS:
        ap.error('--distro must be one of %s (or set ROS_DISTRO), got %r'
                 % (list(stats.DISTROS), args.distro))
    if args.out is None:
        args.out = 'acceptance_%s_%s.json' % (args.distro, args.servo_model)
    if args.summary is None:
        args.summary = os.path.splitext(args.out)[0] + '_summary.md'
    return args


# ---------- execution

def _flush(args, cells_runs):
    """Build the schema 1 result and write the JSON and the summary after
    every repeat: a temp file plus os.replace, so an interrupted job keeps a
    complete file (T-01-11-05)."""
    result = stats.build_result(args.distro, args.servo_model, args.repeats,
                                args.git_sha, cells_runs, args.floor_ratio)
    for path, text in ((args.out, json.dumps(result, indent=1)),
                       (args.summary, stats.render_summary(result))):
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as handle:
            handle.write(text + '\n')
        os.replace(tmp, path)
    return result


def execute(args, runner=run_level, clock=time.monotonic):
    """Run every cell repeat by repeat, strictly one simulation at a time
    (D-24); returns the last schema 1 result.

    One attempt is one fresh run_level launch (plus one more, with
    .retry.sim.log, if the robot never left passive). A no_stand attempt is
    replaced (up to 2*n attempts per cell); an error is not replaced - it
    takes one of the n repeat slots, so a systematic failure does not double
    the job time. Records go through acceptance_stats.run_record only; no
    thresholds live here (D-17).
    """
    cells_runs = {name: [] for name in args.cells}
    stem = os.path.splitext(args.out)[0]
    launch_no = 0
    result = None
    for name in args.cells:
        cell = stats.CELLS[name]
        extra = cell_walk_check_args(name)
        launch_args = cell_launch_args(name, args.servo_model)
        valid = 0
        no_stand = 0
        attempts = 0
        while (valid < args.repeats and attempts - no_stand < args.repeats
               and attempts < ATTEMPT_FACTOR * args.repeats):
            attempts += 1
            seed = seed_for(cell, attempts)
            domain = domain_for(args.domain, launch_no)
            launch_no += 1
            sim_log = '%s_%s_r%02d.sim.log' % (stem, name, attempts)
            start = clock()
            data = runner(cell['terrain'], cell['level'], seed, domain, extra,
                          launch_args=launch_args, keep=None, sim_log=sim_log)
            if never_stood(data):
                print('%s: robot never left passive - relaunching the simulation once'
                      % name, flush=True)
                data = runner(cell['terrain'], cell['level'], seed, domain, extra,
                              launch_args=launch_args, keep=None,
                              sim_log=sim_log.replace('.sim.log', '.retry.sim.log'))
            wall_s = clock() - start
            record = stats.run_record(data, wall_s)
            cells_runs[name].append(record)
            if record['status'] in ('ok', 'fell'):
                valid += 1
            elif record['status'] == 'no_stand':
                no_stand += 1
            ratio = record['ratio']
            print('%-16s #%d/%d %-8s ratio=%s wall_s=%.1f'
                  % (name, attempts, args.repeats, record['status'],
                     'n/a' if ratio is None else '%.1f%%' % (100 * ratio), wall_s), flush=True)
            result = _flush(args, cells_runs)
    return result


# ---------- CLI

def main(argv=None, runner=None, clock=None, environ=None):
    """Parse the CLI, run the repeats and return 0/1/2 (2 via argparse).

    0: artifacts were produced (the verdict may still fail without --strict);
    1: infrastructure failure (a cell without a valid repeat) or --strict
    without a passing verdict. The verdict comes from acceptance_stats only
    (D-06, D-07, D-17); simulations run one at a time (D-24) and outside
    GitHub Actions a warning is printed (D-04).
    """
    env = os.environ if environ is None else environ
    if runner is None:
        runner = run_level
    if clock is None:
        clock = time.monotonic
    args = parse_args(argv, environ=environ)
    if args.dry_run:
        records = plan_runs(args.cells, args.repeats)
        for launch_no, record in enumerate(records):
            name = record['cell']
            cell = stats.CELLS[name]
            print('DRY-RUN %s #%d/%d terrain=%s level=%g seed=%g domain=%d '
                  'launch_args=%s walk_check_args=%s'
                  % (name, record['repeat'], args.repeats, cell['terrain'], cell['level'],
                     record['seed'], domain_for(args.domain, launch_no),
                     ' '.join(cell_launch_args(name, args.servo_model)),
                     ' '.join(cell_walk_check_args(name))), flush=True)
        print('planned: %d simulations (up to %d with replacements of no_stand)'
              % (len(records), ATTEMPT_FACTOR * len(records)), flush=True)
        return 0
    if env.get('GITHUB_ACTIONS') != 'true':
        print('warning: not running in GitHub Actions: simulations belong in CI '
              '(D-04, D-24), local load distorts results (docs/TERRAIN.md)', file=sys.stderr)
    try:
        result = execute(args, runner=runner, clock=clock)
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        print('FAIL: infrastructure: %s' % exc, flush=True)
        return 1
    if result['verdict']['failures']:
        for failure in result['verdict']['failures']:
            print('FAIL %s' % failure, flush=True)
    else:
        print('PASS verdict', flush=True)
    if any(result['cells'][name]['summary']['n'] == 0 for name in args.cells):
        return 1
    if args.strict and result['verdict']['pass'] is not True:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
