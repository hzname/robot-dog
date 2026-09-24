"""Gazebo worlds for terrain tests.

  flat           level ground
  slope  level   flat start area, then a ramp of `level` degrees rising along
                 +x from x = RAMP_START. The robot walks onto it (climb),
                 steps sideways and turns on it (traverse), and walks back
                 down (descend) - the way a real robot meets a slope
  waves  level   cylinders across the path, bump height `level` mm
                 (a curved, washboard-like surface)
  rough  level   scattered stones up to `level` mm high

The spawn area (radius 0.2 m) is kept free of obstacles so the robot always
starts from a well-defined stance.
"""

import math
import random

HEADER = '''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="terrain">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
'''
FOOTER = '''  </world>
</sdf>
'''
RAMP_START = 0.25  # [m] start of the ramp in the slope world
RAMP_LENGTH = 3.0
SURFACE = '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>'


def _box(name, pose, size, color='0.55 0.55 0.55 1'):
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    geo = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    return (f'    <model name="{name}"><static>true</static>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose><link name="l">'
            f'<collision name="c">{geo}{SURFACE}</collision>'
            f'<visual name="v">{geo}<material><diffuse>{color}</diffuse><ambient>{color}</ambient>'
            f'</material></visual></link></model>\n')


def _cylinder_y(name, x, z, r, length, color='0.45 0.5 0.6 1'):
    geo = f'<geometry><cylinder><radius>{r:.4f}</radius><length>{length:.3f}</length></cylinder></geometry>'
    return (f'    <model name="{name}"><static>true</static>'
            f'<pose>{x:.4f} 0 {z:.4f} {math.pi / 2:.5f} 0 0</pose><link name="l">'
            f'<collision name="c">{geo}{SURFACE}</collision>'
            f'<visual name="v">{geo}<material><diffuse>{color}</diffuse><ambient>{color}</ambient>'
            f'</material></visual></link></model>\n')


def _ground():
    return _box('ground', (0, 0, -0.05, 0, 0, 0), (40, 40, 0.1), '0.7 0.7 0.7 1')


def world(kind='flat', level=0.0, seed=0):
    """SDF text of a test world."""
    parts = [HEADER]
    if kind == 'flat':
        parts.append(_ground())
    elif kind == 'slope':
        th = math.radians(level)
        parts.append(_ground())
        if th > 0:
            # slab whose top face starts at (RAMP_START, 0) and rises along +x
            L, T = RAMP_LENGTH, 0.1
            cx = RAMP_START + 0.5 * L * math.cos(th) + 0.5 * T * math.sin(th)
            cz = 0.5 * L * math.sin(th) - 0.5 * T * math.cos(th)
            parts.append(_box('ramp', (cx, 0, cz, 0, -th, 0), (L, 4.0, T), '0.65 0.62 0.55 1'))
    elif kind == 'waves':
        parts.append(_ground())
        h = level / 1000.0
        r = 0.09  # gentle curvature: arc of a 90 mm radius
        k = 0
        for i in range(-12, 13):
            x = i * 0.16
            if abs(x) < 0.2 or h <= 0:
                continue
            parts.append(_cylinder_y(f'wave{k}', x, h - r, r, 4.0))
            k += 1
        # waves across the side path too, for left / right steps
        for i in range(-10, 11):
            y = i * 0.16
            if abs(y) < 0.2 or h <= 0:
                continue
            geo_len = 0.32  # short ridges between the long ones
            for xx in (-0.08, 0.08):
                parts.append(
                    f'    <model name="wy{k}"><static>true</static><pose>{xx:.3f} {y:.4f} {h - r:.4f} 0 {math.pi / 2:.5f} 0'
                    f'</pose><link name="l"><collision name="c"><geometry><cylinder><radius>{r}</radius>'
                    f'<length>{geo_len}</length></cylinder></geometry>{SURFACE}</collision><visual name="v">'
                    f'<geometry><cylinder><radius>{r}</radius><length>{geo_len}</length></cylinder></geometry>'
                    f'</visual></link></model>\n')
                k += 1
    elif kind == 'rough':
        parts.append(_ground())
        rng = random.Random(seed)
        h = level / 1000.0
        n = 0
        while n < 260 and h > 0:
            x, y = rng.uniform(-1.6, 1.6), rng.uniform(-1.2, 1.2)
            if math.hypot(x, y) < 0.2:
                continue
            s = rng.uniform(0.03, 0.07)
            hh = rng.uniform(0.4 * h, h)
            parts.append(_box(f'stone{n}', (x, y, hh / 2, 0, 0, rng.uniform(0, math.pi)), (s, s, hh),
                              '0.5 0.47 0.42 1'))
            n += 1
    else:
        raise ValueError(f'unknown terrain {kind!r} (flat, slope, waves, rough)')
    parts.append(FOOTER)
    return ''.join(parts)


def spawn_pose(kind, level, stand_z=0.25):
    """(z, pitch) to spawn the robot on the terrain's (flat) start area."""
    return stand_z, 0.0


def surface(kind, level, x):
    """(height [m], unit normal) of the smooth ground under world x.
    Obstacles of rough / waves are not included: they are what is tested."""
    if kind == 'slope' and level > 0 and x > RAMP_START:
        th = math.radians(level)
        return (x - RAMP_START) * math.tan(th), (-math.sin(th), 0.0, math.cos(th))
    return 0.0, (0.0, 0.0, 1.0)


def normal(kind, level):
    """Ground normal at the start area."""
    return surface(kind, level, 0.0)[1]
