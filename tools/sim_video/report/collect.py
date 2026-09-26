#!/usr/bin/env python3
"""Refresh the report data (data/*.json) from raw results.

  # test cases per package, from colcon build folders of both distros
  collect.py tests --jazzy ros2_ws/build --lyrical build_lyrical

  # walk_check recordings (terrain / heading runs): key = path below ROOT
  collect.py walk ROOT rec/on_flat_1/flat_0.json ...

  # perception_check recordings listed in data/perception_runs.json
  collect.py perception --root /path/with/recordings

  # perception_check recordings listed in data/gaits_runs.json (crawl, going round, greeting)
  collect.py gaits --root /path/with/recordings

  # localization_check recordings: <name>.json in ROOT (map0, loc1, loc2, loc1ns)
  collect.py localization --root /path/with/recordings

The raw recordings are large (MB per run) and stay out of git; the report
builders only need what this script extracts.
"""
import argparse
import collections
import glob
import json
import os
import sys
import types
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'data')
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))


def load(name, default):
    p = os.path.join(DATA, name)
    return json.load(open(p, encoding='utf-8')) if os.path.exists(p) else default


def save(name, obj):
    with open(os.path.join(DATA, name), 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    print(f'{name} updated')


def cmd_tests(args):
    counts, suites = {}, 0
    for distro, build in (('jazzy', args.jazzy), ('lyrical', args.lyrical)):
        c = collections.Counter()
        # ament_cmake: build/<pkg>/test_results/<pkg>/*.xml; ament_python: build/<pkg>/pytest.xml
        files = glob.glob(os.path.join(build, '*', 'test_results', '*', '*.xml')) + \
            glob.glob(os.path.join(build, '*', 'pytest.xml'))
        for f in files:
            pkg = os.path.basename(os.path.dirname(f))
            c[pkg] += sum(1 for _ in ET.parse(f).getroot().iter('testcase'))
        counts[distro] = dict(c)
        if distro == 'jazzy':
            suites = len(files)
    save('tests.json', {'counts': counts, 'cases': sum(counts['jazzy'].values()), 'colcon_tests': suites})


def cmd_walk(args):
    walk = load('walk.json', {})
    for key in args.files:
        d = json.load(open(os.path.join(args.root, key), encoding='utf-8'))
        walk[key] = {'terrain': d['terrain'], 'results': d['results']}
    save('walk.json', walk)


def perception_check():
    """dog_gazebo.perception_check without ROS installed (only its scoring is used)."""
    sys.path[:0] = [os.path.join(REPO, 'ros2_ws/src/dog_gazebo'), os.path.join(REPO, 'ros2_ws/src/dog_perception')]
    for m in ['rclpy', 'rclpy.qos', 'geometry_msgs.msg', 'nav_msgs.msg', 'sensor_msgs.msg', 'std_msgs.msg']:
        mod = sys.modules.setdefault(m, types.ModuleType(m))
        for n in ['Twist', 'Vector3Stamped', 'Odometry', 'DurabilityPolicy', 'QoSProfile', 'ReliabilityPolicy',
                  'qos_profile_sensor_data', 'Imu', 'JointState', 'LaserScan', 'Range', 'Float32MultiArray',
                  'Float64MultiArray', 'String']:
            setattr(mod, n, object)
    from dog_gazebo import perception_check as pc
    return pc


def cmd_perception(args):
    pc = perception_check()
    runs = load('perception_runs.json', [])
    out = load('perception.json', {})
    for r in runs:
        path = os.path.join(args.root, r['dir'], r['name'] + '.json')
        if not os.path.exists(path):
            if r['name'] not in out:
                print(f'missing {path}')
            continue
        rec = json.load(open(path, encoding='utf-8'))
        s = rec['scores']
        if rec['terrain'] in ('steps', 'wall'):  # re-score with the current rules
            s['detection'] = pc.score_detection(rec['terrain'], rec['level'], rec['hazards'], rec['trace'])
        out[r['name']] = s
    save('perception.json', out)


def cmd_gaits(args):
    """data/gaits.json: the scores of the runs in data/gaits_runs.json (crawl,
    going round, greeting), without the per-message statistics."""
    runs = load('gaits_runs.json', [])
    out = load('gaits.json', {})
    for r in runs:
        path = os.path.join(args.root, r['dir'], r['name'] + '.json')
        if not os.path.exists(path):
            if r['name'] not in out:
                print(f'missing {path}')
            continue
        s = json.load(open(path, encoding='utf-8'))['scores']
        out[r['name']] = {k: v for k, v in s.items() if k not in ('cpu', 'map', 'ground_lidar', 'ground_feet')}
    save('gaits.json', out)


def cmd_localization(args):
    """data/localization.json: scores of the localization_check runs."""
    out = load('localization.json', {})
    for name in ('map0', 'loc1', 'loc2', 'loc1ns'):
        path = os.path.join(args.root, name + '.json')
        if os.path.exists(path):
            out[name] = json.load(open(path, encoding='utf-8'))['scores']
            log = os.path.join(args.root, name + '.sim.log')  # the node's word on the relocalization
            if os.path.exists(log):
                import re
                m = re.findall(r'relocalization over (\d+) points: found \((\d+) % on the walls, next best (\d+) %\)',
                               open(log, encoding='utf-8', errors='replace').read())
                if m:
                    out[name]['reloc'] = {'points': int(m[-1][0]), 'best': int(m[-1][1]), 'second': int(m[-1][2])}
        elif name not in out:
            print(f'missing {path}')
    save('localization.json', out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    t = sub.add_parser('tests')
    t.add_argument('--jazzy', required=True)
    t.add_argument('--lyrical', required=True)
    w = sub.add_parser('walk')
    w.add_argument('root')
    w.add_argument('files', nargs='+')
    p = sub.add_parser('perception')
    p.add_argument('--root', required=True, help='folder holding <dir>/<name>.json of each run')
    g = sub.add_parser('gaits')
    g.add_argument('--root', required=True, help='folder holding <dir>/<name>.json of each run')
    lo = sub.add_parser('localization')
    lo.add_argument('--root', required=True, help='folder holding <name>.json of each run')
    args = ap.parse_args()
    {'tests': cmd_tests, 'walk': cmd_walk, 'perception': cmd_perception, 'gaits': cmd_gaits,
     'localization': cmd_localization}[args.cmd](args)


if __name__ == '__main__':
    main()
