"""terrain_sweep: run walk_check on a series of terrains and summarise.

  ros2 run dog_gazebo terrain_sweep --terrain slope --levels 0 5 10 15 20 --out slope.json

Each level starts a fresh headless simulation (own ROS domain and Gazebo
partition, so several sweeps can run side by side), drives the standard
routine and records per-maneuver pass/fail with the measured numbers.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time


def run_level(kind, level, seed, domain, extra, launch_args=(), keep=None):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), GZ_PARTITION=f'sweep{domain}')
    trace = keep or tempfile.mktemp(suffix='.json')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'dog_gazebo', 'sim.launch.py', 'headless:=true', 'web:=false',
         f'terrain:={kind}', f'level:={level}', f'seed:={seed}', *launch_args],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        check = subprocess.run(
            ['ros2', 'run', 'dog_gazebo', 'walk_check', '--terrain', kind, '--level', str(level),
             '--trace', trace] + (['--record'] if keep else []) + extra,
            env=env, capture_output=True, text=True, timeout=240)
        print(check.stdout, flush=True)
        with open(trace) as f:
            data = json.load(f)
        data.pop('trace', None)
        data['launch_args'] = list(launch_args)
        return data
    except (subprocess.TimeoutExpired, OSError, ValueError) as exc:
        return {'terrain': kind, 'level': level, 'error': str(exc), 'results': []}
    finally:
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=20)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(['pkill', '-f', f'GZ_PARTITION=sweep{domain}'], check=False)
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--terrain', required=True, choices=['flat', 'slope', 'waves', 'rough'])
    ap.add_argument('--levels', type=float, nargs='+', required=True)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--domain', type=int, default=60)
    ap.add_argument('--out', required=True)
    ap.add_argument('--launch-arg', action='append', default=[], metavar='NAME:=VALUE',
                    help='extra sim.launch.py argument, e.g. slope_compensation:=false')
    ap.add_argument('--record-dir', help='keep full recordings (joints, IMU; 30 Hz) here, '
                    'one <terrain>_<level>.json per level, for tools/sim_video')
    args, extra = ap.parse_known_args()
    rows = []
    for k, level in enumerate(args.levels):
        print(f'=== {args.terrain} {level:g}', flush=True)
        keep = None
        if args.record_dir:
            os.makedirs(args.record_dir, exist_ok=True)
            keep = os.path.join(args.record_dir, f'{args.terrain}_{level:g}.json')
        rows.append(run_level(args.terrain, level, args.seed, args.domain + k % 5, extra,
                              args.launch_arg, keep))
        with open(args.out, 'w') as f:
            json.dump(rows, f, indent=1)
    ok = all(r.get('results') and all(x['ok'] for x in r['results']) for r in rows)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
