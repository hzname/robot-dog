#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Переносит подстройку со страницы поз (servo_pose_trim.json) в offset_deg
servos.yaml — то, что читают ноды (servo_driver).

    python3 servo_pose_apply.py --trim servo_pose_trim.json --servos servos.yaml [--write]

offset_deg = опорный угол монтажной позы − подстройка:
    hip 0°, thigh +45°, calf −90° (как в DEPLOYMENT.md, «Монтажная поза»).

Без --write — только печатает, что изменится (сухой прогон). С --write —
переписывает значения offset_deg в строках суставов, рядом сохраняет .bak.

Строки с coupled_to (колено, связанное с бедром) пропускаются: их offset_deg
— это s = бедро + колено в середине окна импульсов, а не угол колена, и
ставится вручную (docs/CALIBRATION.md, раздел «Колено связано с бедром»).
"""
import argparse
import json
import re
import shutil
import sys

JOINTS = [
    (0,  'lf_hip_joint',   0.0),
    (1,  'lf_thigh_joint', 45.0),
    (2,  'lf_calf_joint',  -90.0),
    (3,  'rf_hip_joint',   0.0),
    (4,  'rf_thigh_joint', 45.0),
    (5,  'rf_calf_joint',  -90.0),
    (6,  'lr_hip_joint',   0.0),
    (7,  'lr_thigh_joint', 45.0),
    (8,  'lr_calf_joint',  -90.0),
    (9,  'rr_hip_joint',   0.0),
    (10, 'rr_thigh_joint', 45.0),
    (11, 'rr_calf_joint',  -90.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trim', required=True, help='servo_pose_trim.json со страницы')
    ap.add_argument('--servos', required=True, help='путь к servos.yaml')
    ap.add_argument('--write', action='store_true', help='записать (иначе сухой прогон)')
    args = ap.parse_args()

    with open(args.trim) as f:
        data = json.load(f)
    trims = {int(k): float(v) for k, v in data.get('trims', {}).items()}

    want = {}
    for ch, name, ref in JOINTS:
        want[name] = ref - trims.get(ch, 0.0)

    with open(args.servos) as f:
        text = f.read()

    pat = re.compile(r'^(\s*([a-z_]+_joint):\s+\{.*?offset_deg:)\s*(-?\d+(?:\.\d+)?)(.*)$')
    coupled = re.compile(r'coupled_to:\s*[\'"]?[a-z_]+_joint')
    out, changed, skipped = [], [], []
    for line in text.splitlines():
        m = pat.match(line)
        if m and m.group(2) in want and coupled.search(line):
            skipped.append(m.group(2))  # offset_deg of a coupled joint is s, not the knee angle
        elif m and m.group(2) in want:
            name = m.group(2)
            new_val = '%6.1f' % want[name]
            if m.group(3) != new_val.strip():
                changed.append((name, m.group(3), new_val.strip()))
            line = m.group(1) + new_val + m.group(4)
        out.append(line)

    if skipped:
        print('пропущены связанные суставы (offset_deg ставится вручную, docs/CALIBRATION.md): %s'
              % ', '.join(skipped))

    if not changed:
        print('offset_deg уже соответствуют подстройке — менять нечего')
        return 0

    for name, old, new in changed:
        print('%-16s offset_deg: %s -> %s' % (name, old, new))

    if not args.write:
        print('\nсухой прогон (без --write ничего не записано)')
        return 0

    shutil.copy2(args.servos, args.servos + '.bak')
    with open(args.servos, 'w') as f:
        f.write('\n'.join(out) + ('\n' if text.endswith('\n') else ''))
    print('\nзаписано: %s (бэкап %s.bak)' % (args.servos, args.servos))
    return 0


if __name__ == '__main__':
    sys.exit(main())
