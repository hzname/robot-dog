"""Gazebo worlds for terrain tests.

  flat           level ground
  slope  level   flat start area, then a ramp of `level` degrees rising along
                 +x from x = RAMP_START. The robot walks onto it (climb),
                 steps sideways and turns on it (traverse), and walks back
                 down (descend) - the way a real robot meets a slope
  waves  level   cylinders across the path, bump height `level` mm
                 (a curved, washboard-like surface)
  rough  level   scattered stones up to `level` mm high
  room           localization: a 5 x 4 m room with 1.2 m walls, a cabinet,
                 a sofa, a table and a chest. `seed` picks where in the room
                 the robot starts (ROOM_STARTS): the room is placed round it
  house          loop closure: a corridor 1.5 m wide round a 7 x 4 m core
                 (10 x 7 m outside), walls 1.2 m, a few pieces of furniture and
                 long bare stretches; `seed` picks the start (HOUSE_STARTS)
  steps  level   perception test course, steps of `level` mm: the robot starts
                 on a platform with a 20 mm stone in the left foot corridor
                 (x 0.8 m) and one in the right (x 1.4 m), steps down at
                 x 2.2 m and up again at x 3.0 m

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
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
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
KINDS = ('flat', 'slope', 'waves', 'rough', 'steps', 'wall', 'stairs', 'bar', 'block', 'room', 'house')
# room world, in room coordinates (centre of the floor, x along the long side)
ROOM_SIZE, ROOM_WALL_H = (5.0, 4.0), 1.2
ROOM_FURNITURE = (  # name, centre x, y, size x, y, z, lifted (table top)
    ('cabinet', 2.30, 1.30, 0.40, 1.00, 1.20, 0.0),
    ('sofa', -0.60, -1.60, 1.80, 0.80, 0.45, 0.0),
    ('table', -1.40, 1.20, 1.20, 0.70, 0.03, 0.70),
    ('chest', 1.65, -1.45, 0.50, 0.50, 0.50, 0.0))
TABLE_LEGS = ((-0.55, -0.30), (0.55, -0.30), (-0.55, 0.30), (0.55, 0.30))
# where the robot starts (room x, y, yaw) for each seed
ROOM_STARTS = ((-0.8, 0.0, 0.0), (0.9, -0.4, 2.2), (0.3, 0.6, -1.9))
# house world (house coordinates): outer walls, the core, furniture as boxes
HOUSE_OUTER, HOUSE_CORE = (10.0, 7.0), (7.0, 4.0)
HOUSE_FURNITURE = (  # name, x0, y0, x1, y1, height
    ('cabinet', -1.0, 3.1, 0.2, 3.5, 1.2),
    ('shelf', 4.6, -1.0, 5.0, 0.2, 1.0),
    ('chest', -5.0, -3.5, -4.4, -2.9, 0.6),
    ('bench', 1.0, -2.3, 1.6, -2.0, 0.45))
HOUSE_STARTS = ((-4.25, -1.5, 1.5708), (4.25, 1.0, -1.5708), (0.5, 2.75, 3.1416))
# the loop round the core along the corridors' middle
HOUSE_LOOP = ((-4.25, -2.75), (4.25, -2.75), (4.25, 2.75), (-4.25, 2.75))
RAMP_START = 0.25  # [m] start of the ramp in the slope world
RAMP_LENGTH = 3.0
STEP_DOWN_X, STEP_UP_X = 2.2, 3.0  # steps world
WALL_X = 1.2  # wall world: near face of a block across the path
# stairs world: three steps up (riser = level mm, tread 0.30 m), a landing,
# three steps down
STAIRS_UP = (0.8, 1.1, 1.4)
STAIRS_DOWN = (2.3, 2.6, 2.9)
BAR_X, BAR_DEPTH = 1.0, 0.04    # bar world: a bar across the path, level mm high
BLOCK_X, BLOCK_SIZE = 1.0, (0.20, 0.20)  # block world: too tall, narrow: go round it


def stairs_height(level, x):
    """Ground height of the stairs world at x."""
    h = level / 1000.0
    n = sum(x >= e for e in STAIRS_UP) - sum(x >= e for e in STAIRS_DOWN)
    return n * h
STONES = ((0.80, 0.13), (1.40, -0.13))  # steps world: (x of the near edge, y) of 20 mm stones
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


def _cylinder(name, pose, r, length, color='0.45 0.5 0.6 1'):
    x, y, z, roll, pitch, yaw = pose
    geo = f'<geometry><cylinder><radius>{r:.4f}</radius><length>{length:.3f}</length></cylinder></geometry>'
    return (f'    <model name="{name}"><static>true</static>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose><link name="l">'
            f'<collision name="c">{geo}{SURFACE}</collision>'
            f'<visual name="v">{geo}<material><diffuse>{color}</diffuse><ambient>{color}</ambient>'
            f'</material></visual></link></model>\n')


def _ground():
    return _box('ground', (0, 0, -0.05, 0, 0, 0), (40, 40, 0.1), '0.7 0.7 0.7 1')


def room_to_world(seed=0, kind='room'):
    """(x, y, yaw) of the room (house) frame in the world: the robot spawns
    at the world origin facing +x, so the room is its start pose inverted."""
    starts = HOUSE_STARTS if kind == 'house' else ROOM_STARTS
    sx, sy, syaw = starts[seed % len(starts)]
    c, s_ = math.cos(syaw), math.sin(syaw)
    return -(c * sx + s_ * sy), -(-s_ * sx + c * sy), -syaw


def room_boxes(seed=0):
    """Walls and furniture of the room world as world-frame boxes."""
    ox, oy, oyaw = room_to_world(seed)
    c, s_ = math.cos(oyaw), math.sin(oyaw)
    L, W = ROOM_SIZE
    t = 0.10
    boxes = [  # name, room x, y, z, size
        ('wall_s', 0.0, -(W + t) / 2, ROOM_WALL_H / 2, (L + 2 * t, t, ROOM_WALL_H)),
        ('wall_n', 0.0, (W + t) / 2, ROOM_WALL_H / 2, (L + 2 * t, t, ROOM_WALL_H)),
        ('wall_w', -(L + t) / 2, 0.0, ROOM_WALL_H / 2, (t, W, ROOM_WALL_H)),
        ('wall_e', (L + t) / 2, 0.0, ROOM_WALL_H / 2, (t, W, ROOM_WALL_H))]
    for name, x, y, sx, sy, sz, lift in ROOM_FURNITURE:
        boxes.append((name, x, y, lift + sz / 2, (sx, sy, sz)))
        if lift > 0:
            for k, (lx, ly) in enumerate(TABLE_LEGS):
                boxes.append((f'{name}_leg{k}', x + lx, y + ly, lift / 2, (0.05, 0.05, lift)))
    out = []
    for name, x, y, z, size in boxes:
        out.append({'shape': 'box', 'name': name, 'x': ox + c * x - s_ * y, 'y': oy + s_ * x + c * y, 'z': z,
                    'size': size, 'yaw': oyaw})
    return out


def house_boxes(seed=0):
    """Walls, core and furniture of the house world as world-frame boxes."""
    ox, oy, oyaw = room_to_world(seed, 'house')
    c, s_ = math.cos(oyaw), math.sin(oyaw)
    L, W = HOUSE_OUTER
    cl, cw = HOUSE_CORE
    t, h = 0.10, ROOM_WALL_H
    boxes = [
        ('wall_s', 0.0, -(W + t) / 2, h / 2, (L + 2 * t, t, h)),
        ('wall_n', 0.0, (W + t) / 2, h / 2, (L + 2 * t, t, h)),
        ('wall_w', -(L + t) / 2, 0.0, h / 2, (t, W, h)),
        ('wall_e', (L + t) / 2, 0.0, h / 2, (t, W, h)),
        ('wall_core', 0.0, 0.0, h / 2, (cl, cw, h))]
    for name, x0, y0, x1, y1, hh in HOUSE_FURNITURE:
        boxes.append((name, (x0 + x1) / 2, (y0 + y1) / 2, hh / 2, (x1 - x0, y1 - y0, hh)))
    return [{'shape': 'box', 'name': name, 'x': ox + c * x - s_ * y, 'y': oy + s_ * x + c * y, 'z': z,
             'size': size, 'yaw': oyaw} for name, x, y, z, size in boxes]


def obstacles(kind, level, seed=0):
    """Obstacles of the waves / rough worlds as plain shapes (also used to
    draw the terrain in videos):
      {'shape': 'cyl_y', 'x', 'z', 'r', 'length'}          cylinder along y
      {'shape': 'cyl_x', 'x', 'y', 'z', 'r', 'length'}     cylinder along x
      {'shape': 'box', 'x', 'y', 'z', 'size': (sx, sy, sz), 'yaw'}"""
    out = []
    if kind == 'room':
        return room_boxes(seed)
    if kind == 'house':
        return house_boxes(seed)
    h = level / 1000.0
    if h <= 0:
        return out
    if kind == 'waves':
        r = 0.09  # gentle curvature: arc of a 90 mm radius
        for i in range(-12, 13):
            x = i * 0.16
            if abs(x) >= 0.2:
                out.append({'shape': 'cyl_y', 'x': x, 'z': h - r, 'r': r, 'length': 4.0})
        # short ridges across the side path, for left / right steps
        for i in range(-10, 11):
            y = i * 0.16
            if abs(y) >= 0.2:
                for xx in (-0.08, 0.08):
                    out.append({'shape': 'cyl_x', 'x': xx, 'y': y, 'z': h - r, 'r': r, 'length': 0.32})
    elif kind == 'steps':
        for x, y in STONES:
            out.append({'shape': 'box', 'x': x + 0.03, 'y': y, 'z': h + 0.01, 'size': (0.06, 0.14, 0.02),
                        'yaw': 0.0})
    elif kind == 'bar':  # a bar across the path: step over it in the crawl
        out.append({'shape': 'box', 'x': BAR_X + BAR_DEPTH / 2, 'y': 0.0, 'z': h / 2, 'size': (BAR_DEPTH, 1.6, h),
                    'yaw': 0.0})
    elif kind == 'block':  # too tall and narrow: go round it
        sx, sy = BLOCK_SIZE
        out.append({'shape': 'box', 'x': BLOCK_X + sx / 2, 'y': 0.0, 'z': h / 2, 'size': (sx, sy, h), 'yaw': 0.0})
    elif kind == 'wall':  # a block too tall to step over (level = height in mm)
        out.append({'shape': 'box', 'x': WALL_X + 0.05, 'y': 0.0, 'z': h / 2, 'size': (0.10, 0.80, h), 'yaw': 0.0})
    elif kind == 'rough':
        rng = random.Random(seed)
        while len(out) < 260:
            x, y = rng.uniform(-1.6, 1.6), rng.uniform(-1.2, 1.2)
            if math.hypot(x, y) < 0.2:
                continue
            size = rng.uniform(0.03, 0.07)
            hh = rng.uniform(0.4 * h, h)
            out.append({'shape': 'box', 'x': x, 'y': y, 'z': hh / 2, 'size': (size, size, hh),
                        'yaw': rng.uniform(0, math.pi)})
    return out


def world(kind='flat', level=0.0, seed=0):
    """SDF text of a test world."""
    if kind not in KINDS:
        raise ValueError(f'unknown terrain {kind!r} ({", ".join(KINDS)})')
    parts = [HEADER, _ground()]
    if kind == 'slope':
        th = math.radians(level)
        if th > 0:
            # slab whose top face starts at (RAMP_START, 0) and rises along +x
            L, T = RAMP_LENGTH, 0.1
            cx = RAMP_START + 0.5 * L * math.cos(th) + 0.5 * T * math.sin(th)
            cz = 0.5 * L * math.sin(th) - 0.5 * T * math.cos(th)
            parts.append(_box('ramp', (cx, 0, cz, 0, -th, 0), (L, 4.0, T), '0.65 0.62 0.55 1'))
    if kind == 'stairs' and level > 0:
        h = level / 1000.0
        edges = sorted(STAIRS_UP + STAIRS_DOWN) + [STAIRS_DOWN[-1] + 1.0]
        for k in range(len(edges) - 1):
            z = stairs_height(level, edges[k])
            if z > 0:
                parts.append(_box(f'stair{k}', ((edges[k] + edges[k + 1]) / 2, 0, z / 2, 0, 0, 0),
                                  (edges[k + 1] - edges[k], 3.0, z), '0.62 0.62 0.66 1'))
    if kind == 'steps' and level > 0:
        h = level / 1000.0
        parts.append(_box('platform1', ((-1.0 + STEP_DOWN_X) / 2, 0, h / 2, 0, 0, 0),
                          (STEP_DOWN_X + 1.0, 4.0, h), '0.62 0.62 0.66 1'))
        parts.append(_box('platform2', ((STEP_UP_X + 6.0) / 2, 0, h / 2, 0, 0, 0),
                          (6.0 - STEP_UP_X, 4.0, h), '0.62 0.62 0.66 1'))
    for k, o in enumerate(obstacles(kind, level, seed)):
        if o['shape'] == 'cyl_y':
            parts.append(_cylinder(f'wave{k}', (o['x'], 0, o['z'], math.pi / 2, 0, 0), o['r'], o['length']))
        elif o['shape'] == 'cyl_x':
            parts.append(_cylinder(f'ridge{k}', (o['x'], o['y'], o['z'], 0, math.pi / 2, 0), o['r'], o['length']))
        elif kind in ('room', 'house'):
            wall = o['name'].startswith('wall')
            parts.append(_box(o['name'], (o['x'], o['y'], o['z'], 0, 0, o['yaw']), o['size'],
                              '0.82 0.80 0.76 1' if wall else '0.55 0.42 0.30 1'))
        else:
            parts.append(_box(f'stone{k}', (o['x'], o['y'], o['z'], 0, 0, o['yaw']), o['size'],
                              '0.5 0.47 0.42 1'))
    parts.append(FOOTER)
    return ''.join(parts)


def height(kind, level, x, y, obs=()):
    """Ground height at (x, y) including obstacles (for drawing)."""
    z = surface(kind, level, x)[0]
    for o in obs:
        if o['shape'] == 'cyl_y':
            d = abs(x - o['x'])
            if d < o['r']:
                z = max(z, o['z'] + math.sqrt(o['r'] ** 2 - d * d))
        elif o['shape'] == 'cyl_x':
            d = abs(y - o['y'])
            if d < o['r'] and abs(x - o['x']) < o['length'] / 2:
                z = max(z, o['z'] + math.sqrt(o['r'] ** 2 - d * d))
        else:
            c, s_ = math.cos(o['yaw']), math.sin(o['yaw'])
            dx, dy = x - o['x'], y - o['y']
            u, v = c * dx + s_ * dy, -s_ * dx + c * dy
            if abs(u) < o['size'][0] / 2 and abs(v) < o['size'][1] / 2:
                z = max(z, o['z'] + o['size'][2] / 2)
    return z


def spawn_pose(kind, level, stand_z=0.25):
    """(z, pitch) to spawn the robot on the terrain's (flat) start area."""
    if kind == 'steps':
        return stand_z + level / 1000.0, 0.0
    return stand_z, 0.0


def surface(kind, level, x):
    """(height [m], unit normal) of the smooth ground under world x.
    Obstacles of rough / waves are not included: they are what is tested."""
    if kind == 'steps' and level > 0:
        h = level / 1000.0
        return (h if x < STEP_DOWN_X or x >= STEP_UP_X else 0.0), (0.0, 0.0, 1.0)
    if kind == 'stairs' and level > 0:
        return stairs_height(level, x), (0.0, 0.0, 1.0)
    if kind == 'slope' and level > 0 and x > RAMP_START:
        th = math.radians(level)
        return (x - RAMP_START) * math.tan(th), (-math.sin(th), 0.0, math.cos(th))
    return 0.0, (0.0, 0.0, 1.0)


def normal(kind, level):
    """Ground normal at the start area."""
    return surface(kind, level, 0.0)[1]
