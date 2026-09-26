"""perception_check: walks the simulated dog straight ahead with dog_perception
running and scores the X lidars and the VL53L1X sensors against the known
terrain.

  ros2 launch dog_gazebo sim.launch.py headless:=true web:=false perception:=true terrain:=steps level:=30 &
  ros2 run dog_gazebo perception_check --terrain steps --level 30 --seconds 30 --trace run.json

Scores:
  ground     height / roll / pitch of the body over the floor, from the lidars
             and from the legs, vs. the truth (RMS)
  detection  steps world: for each stone / step, how far ahead of the front
             feet each sensor first reported it, and where the lidar put it
  false      hazard reports per metre on terrain without hazards (flat, and
             the slope away from the ramp start)
  map        elevation map vs. the true terrain (RMS, stones found)
  cpu        processing time of the perception node
The trace (with --trace) also holds the raw scans and ToF readings for videos.
"""

import argparse
import bisect
import collections
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan, Range
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from dog_gazebo import terrain
from dog_perception import core

FRONT_FOOT = 0.15   # front feet land up to this far ahead of the body centre [m]
TOF = ('fl', 'fr', 'fc', 'rc')


def _rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def features(kind, level):
    """Known hazards: (name, kind, x of the near edge, (y0, y1) or None = full width)."""
    if kind == 'wall' and level > 0:
        return [('wall', 'up', terrain.WALL_X, (-0.4, 0.4))]
    if kind == 'stairs' and level > 0:
        return [(f'stair_up{k + 1}', 'up', x, None) for k, x in enumerate(terrain.STAIRS_UP)] + \
            [(f'stair_down{k + 1}', 'down', x, None) for k, x in enumerate(terrain.STAIRS_DOWN)]
    if kind == 'bar' and level > 0:
        return [('bar', 'up', terrain.BAR_X, None)]
    if kind == 'block' and level > 0:
        return [('block', 'up', terrain.BLOCK_X, (-terrain.BLOCK_SIZE[1] / 2, terrain.BLOCK_SIZE[1] / 2))]
    if kind != 'steps':
        return []
    (xl, yl), (xr, yr) = terrain.STONES
    return [('stone_left', 'up', xl, (yl - 0.07, yl + 0.07)),
            ('stone_right', 'up', xr, (yr - 0.07, yr + 0.07)),
            ('step_down', 'down', terrain.STEP_DOWN_X, None),
            ('step_up', 'up', terrain.STEP_UP_X, None)]


# Sensor mounts and stand height: the same robot.yaml the simulation runs
# with (a copy here went stale whenever the mounts were changed).
SENSOR_PARAMS, _GEOMETRY, _STANCE = core.robot_config()
STAND_HEIGHT = float(_STANCE.get('stand_height', 0.15))
GS2_LINE_X = core.gs2_line_x(SENSOR_PARAMS, STAND_HEIGHT) if SENSOR_PARAMS.get('gs2') else 0.28
FAMILIES = ('lidar', 'tof', 'gs2')
CORRIDOR_Y = {'left': 0.12, 'centre': 0.0, 'right': -0.12}


def hazard_places(hazards, trace):
    """World (x, y) each hazard report points at: the lidar's nearest point in
    its corridor, or the spot of the ToF beam on the floor."""
    mounts = core.mounts_from_params(SENSOR_PARAMS)
    ts = np.array([e['t'] for e in trace])
    out = []
    for h in hazards:
        e = trace[int(np.clip(np.searchsorted(ts, h['t_rx']), 0, len(ts) - 1))]
        R = core.quat_to_rot(*e['q'])
        p = np.array([e['x'], e['y'], e['z']])
        if h['source'] == 'lidar':
            b = np.array([h['x'], CORRIDOR_Y[h['corridor']], 0.0])
        elif h['source'] == 'gs2':
            b = np.array([h['x'] if h.get('x') is not None else GS2_LINE_X,
                          h['y'] if h.get('y') is not None else CORRIDOR_Y[h['corridor']], 0.0])
        else:
            m = mounts[h['source']]
            rng = h.get('expected') or 0.25
            b = m.p + rng * m.beam
        w = p + R @ b
        out.append((float(w[0]), float(w[1]), float(e['x']) + FRONT_FOOT * math.cos(math.radians(e['yaw']))))
    return out


def score_detection(kind, level, hazards, trace):
    """For each known hazard and sensor family: how far ahead of the front feet
    it was first reported (m), and how far off the report was (mm)."""
    walk = [h for h in hazards if h['phase'] == 'walk']
    places = hazard_places(walk, trace) if walk else []
    det = {}
    walk_x = [e['x'] for e in trace if e['phase'] == 'walk']
    reach = (max(walk_x) if walk_x else 0.0) + FRONT_FOOT + 0.3  # the sensors look this far at least
    for name, kind_f, edge, ys in features(kind, level):
        det[name] = {}
        if edge > reach and kind != 'wall':  # the robot stopped (or fell) before it came near
            det[name] = {'not_reached': True}
            continue
        if ys is not None:
            # a stone the robot walks past sideways is no hazard: lateral offset
            # of its centre from the body axis when the robot is 0.4 m before it
            e = min((e for e in trace if e['phase'] == 'walk'), key=lambda e: abs(e['x'] + 0.4 - edge))
            yaw = math.radians(e['yaw'])
            dx, dy = edge + 0.03 - e['x'], (ys[0] + ys[1]) / 2 - e['y']
            lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if abs(lateral) > 0.25:
                det[name] = {'not_in_path': True, 'lateral_m': round(lateral, 2)}
                continue
        for fam in FAMILIES:
            if fam == 'gs2' and edge > reach - 0.3 + GS2_LINE_X - FRONT_FOOT + 0.02:
                det[name][fam] = {'not_reached': True}  # the line (0.14 m ahead of the feet) never got there
                continue
            best = None
            for h, (wx, wy, foot_x) in zip(walk, places):
                if not h['source'].startswith(fam) or h['kind'] != kind_f:
                    continue
                if ys is not None and not (ys[0] - 0.06 < wy < ys[1] + 0.06):
                    continue
                if not (edge - 0.15 < wx < edge + (0.12 if ys else 0.15)):
                    continue
                ahead = edge - foot_x
                if ahead < -0.05:
                    continue
                if best is None or h['t_rx'] < best[0]:
                    by = h['source'] if fam == 'tof' else \
                        f"{h['corridor']}/{h.get('how')}" if fam == 'gs2' else h['corridor']
                    best = (h['t_rx'], ahead, wx - edge, by)
            det[name][fam] = None if best is None else {
                'ahead_m': round(best[1], 3), 'where_err_mm': round(1e3 * best[2]), 'by': best[3]}
    return det


class PerceptionCheck:
    def __init__(self, kind, level, seconds, speed, greet=False):
        self.kind, self.level, self.seconds, self.speed = kind, level, seconds, speed
        self.greet = greet
        self.states = []  # [t, locomotion state] on every change
        self.node = rclpy.create_node('perception_check', namespace='dog')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n = self.node
        self.state, self.odom, self.joints, self.imu = None, None, {}, None
        self.commands = {}
        self.hazards, self.ground, self.tof, self.scans, self.trace = [], [], [], [], []
        self.guard = []  # [t, state, max_vx, [step per leg]] on every change
        self.stats, self.map = None, None
        self.t0 = time.time()
        self.phase = 'setup'
        n.create_subscription(String, 'state', self.on_state, latched)
        n.create_subscription(Odometry, 'odom', self.on_odom, 10)
        n.create_subscription(JointState, 'joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 10)
        # what the locomotion commands (vs. joint_states: what the servos reach)
        n.create_subscription(JointState, 'joint_commands', lambda m: self.commands.update(zip(m.name, m.position)), 10)
        n.create_subscription(Imu, 'imu/data', lambda m: setattr(self, 'imu', m), qos_profile_sensor_data)
        n.create_subscription(String, 'perception/hazards', self.on_hazards, 50)
        n.create_subscription(Vector3Stamped, 'perception/ground_lidar', lambda m: self.on_ground('lidar', m), 50)
        n.create_subscription(Vector3Stamped, 'perception/ground_feet', lambda m: self.on_ground('feet', m), 50)
        n.create_subscription(Float32MultiArray, 'perception/tof', self.on_tof, 100)
        n.create_subscription(String, 'perception/guard', self.on_guard, 20)
        n.create_subscription(Float64MultiArray, 'perception/stats', lambda m: setattr(self, 'stats', list(m.data)), 1)
        n.create_subscription(Float32MultiArray, 'perception/map', lambda m: setattr(self, 'map', list(m.data)), 1)
        for name in ('lidar_left', 'lidar_right', 'gs2'):
            n.create_subscription(LaserScan, f'{name}/scan', lambda m, s=name: self.on_scan(s, m),
                                  qos_profile_sensor_data)
        self.cmd = n.create_publisher(String, 'command', 10)
        self.vel = n.create_publisher(Twist, 'cmd_vel', 10)
        self.stats_start = None
        self.odom_hist = collections.deque(maxlen=500)  # (stamp [s], position, R)

    def now(self):
        return round(time.time() - self.t0, 3)

    # ------------------------------------------------------------ recording
    def pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y, p.position.z]), _rot(p.orientation)

    def pose_at(self, stamp):
        """Ground-truth pose at a message time stamp (nearest odom sample)."""
        t = stamp.sec + stamp.nanosec * 1e-9
        ts = [h[0] for h in self.odom_hist]
        i = min(max(bisect.bisect_left(ts, t), 0), len(ts) - 1)
        if i > 0 and t - ts[i - 1] < ts[i] - t:
            i -= 1
        return self.odom_hist[i][1], self.odom_hist[i][2]

    def on_odom(self, msg):
        self.odom = msg
        pp = msg.pose.pose
        self.odom_hist.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                               np.array([pp.position.x, pp.position.y, pp.position.z]), _rot(pp.orientation)))
        t = self.now()
        if self.trace and t - self.trace[-1]['t'] < 1 / 30:
            return
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        e = {'t': t, 'phase': self.phase, 'state': self.state,
             'x': round(p.position.x, 4), 'y': round(p.position.y, 4), 'z': round(p.position.z, 4),
             'yaw': round(math.degrees(yaw), 2), 'roll': 0.0, 'pitch': 0.0,
             'q': [round(v, 5) for v in (q.x, q.y, q.z, q.w)],
             'j': {k: round(v, 4) for k, v in self.joints.items()},
             'c': {k: round(v, 4) for k, v in self.commands.items()}}
        if self.imu is not None:
            iq = self.imu.orientation
            r = math.atan2(2 * (iq.w * iq.x + iq.y * iq.z), 1 - 2 * (iq.x * iq.x + iq.y * iq.y))
            pi = math.asin(max(-1.0, min(1.0, 2 * (iq.w * iq.y - iq.z * iq.x))))
            e['imu'] = [round(math.degrees(r), 2), round(math.degrees(pi), 2)]
        self.trace.append(e)

    def on_scan(self, name, msg):
        if self.phase == 'walk':
            self.scans.append({'t': self.now(), 'name': name, 'a0': msg.angle_min, 'da': msg.angle_increment,
                               'r': [round(v, 3) if math.isfinite(v) else None for v in msg.ranges]})

    def on_hazards(self, msg):
        if self.odom is None:
            return
        p, R = self.pose()
        for h in json.loads(msg.data):
            h['t_rx'] = self.now()
            h['robot'] = [round(float(p[0]), 4), round(float(p[1]), 4)]
            h['phase'] = self.phase
            if h.get('x') is not None:  # lidar / GS2: nearest point of the hazard, projected into the world
                w = p + R @ np.array([h['x'], 0.0, 0.0])
                h['world_x'] = round(float(w[0]), 4)
            self.hazards.append(h)

    def on_ground(self, src, msg):
        if self.odom is None or self.phase != 'walk':
            return
        p, R = self.pose_at(msg.header.stamp)
        h, n = terrain.surface(self.kind, self.level, float(p[0]))
        nb = R.T @ np.array(n)  # true floor normal in the body frame
        roll_t = math.atan2(nb[1], nb[2])
        pitch_t = math.atan2(-nb[0], math.hypot(nb[1], nb[2]))
        height_t = float((p[2] - h) * n[2])
        v = msg.vector
        self.ground.append({'t': self.now(), 'src': src, 'x': round(float(p[0]), 3),
                            'est': [v.x, v.y, v.z], 'true': [roll_t, pitch_t, height_t]})

    def on_state(self, msg):
        self.state = msg.data
        if not self.states or self.states[-1][1] != msg.data:
            self.states.append([self.now(), msg.data])

    def on_guard(self, msg):
        g = json.loads(msg.data)
        row = [self.now(), g['state'], g['max_vx'], g['step']]
        if not self.guard or self.guard[-1][1:] != row[1:]:
            self.guard.append(row + [self.phase])

    def on_tof(self, msg):
        if self.phase == 'walk':
            d = list(msg.data)
            self.tof.append([self.now(), int(d[0])] + [None if math.isnan(v) else round(v, 4) for v in d[1:]])

    # ------------------------------------------------------------ run
    def spin(self, seconds, publish=None):
        end = time.time() + seconds
        while time.time() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def run(self):
        end = time.time() + 90
        while (self.odom is None or self.state is None) and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None:
            print('FAIL: no simulation')
            return False
        self.spin(2.0)
        self.phase = 'stand'
        self.spin(5.0, lambda: self.cmd.publish(String(data='stand')) if self.state in ('passive', 'lying') else None)
        self.spin(2.0)  # ToF calibration on the flat start
        self.stats_start = (time.time(), self.stats)
        self.phase = 'walk'
        tw = Twist()
        tw.linear.x = 0.0 if self.greet else self.speed
        x0 = float(self.pose()[0][0])
        if self.greet:  # standing still: sit, paws up, wave, stand up again
            self.spin(0.5, lambda: self.vel.publish(tw))
            self.cmd.publish(String(data='greet'))
        self.spin(self.seconds, lambda: self.vel.publish(tw))
        self.phase = 'stop'
        self.spin(1.5, lambda: self.vel.publish(Twist()))
        self.walked = float(self.pose()[0][0]) - x0
        self.stats_end = (time.time(), self.stats)
        self.spin(1.5)
        return True

    # ------------------------------------------------------------ scores
    def scores(self):
        out = {'terrain': self.kind, 'level': self.level, 'walked_m': round(self.walked, 3)}
        if self.greet:
            walk = [e for e in self.trace if e['phase'] == 'walk' and 'imu' in e]
            seen = [st for _, st in self.states]
            i = seen.index('greeting') if 'greeting' in seen else -1
            out['greet'] = {
                'started': i >= 0,
                'finished': i >= 0 and 'stand' in seen[i + 1:],
                'seconds': round(next((t for t, st in self.states[i + 1:] if st == 'stand'), math.nan)
                                 - self.states[i][0], 1) if i >= 0 else None,
                'max_nose_up_deg': round(-min((e['imu'][1] for e in walk), default=0.0), 1),
                'drift_m': round(float(np.hypot(*(self.pose()[0][:2]))), 3)}
        # ground
        for src in ('lidar', 'feet'):
            g = [x for x in self.ground if x['src'] == src]
            if self.kind == 'slope':  # away from the ramp start, where the floor under the legs changes
                g = [x for x in g if abs(x['x'] - terrain.RAMP_START) > 0.35]
            if self.kind == 'steps':
                g = [x for x in g if min(abs(x['x'] - terrain.STEP_DOWN_X), abs(x['x'] - terrain.STEP_UP_X)) > 0.35]
            if g:
                e = np.array([x['est'] for x in g]) - np.array([x['true'] for x in g])
                out[f'ground_{src}'] = {'n': len(g),
                                        'height_rms_mm': round(1e3 * float(np.sqrt(np.mean(e[:, 2] ** 2))), 1),
                                        'height_bias_mm': round(1e3 * float(np.mean(e[:, 2])), 1),
                                        'roll_rms_deg': round(math.degrees(float(np.sqrt(np.mean(e[:, 0] ** 2)))), 2),
                                        'pitch_rms_deg': round(math.degrees(float(np.sqrt(np.mean(e[:, 1] ** 2)))), 2)}
        # reaction (guard) and how the walk went
        walk = [e for e in self.trace if e['phase'] == 'walk']
        tilt = [max(abs(v) for v in e['imu']) for e in walk if 'imu' in e]
        out['max_tilt_deg'] = round(max(tilt), 1) if tilt else None
        out['fell'] = bool(tilt) and max(tilt) > 35
        if self.guard:
            t_end = walk[-1]['t'] if walk else self.now()
            share = collections.Counter()
            gw = [g for g in self.guard if g[4] == 'walk']
            for k, g in enumerate(gw):
                t1 = gw[k + 1][0] if k + 1 < len(gw) else t_end
                share[g[1]] += max(0.0, t1 - g[0])
            total = sum(share.values()) or 1.0
            out['guard'] = {k: round(v / total, 3) for k, v in share.items()}
            out['guard']['high_steps'] = sum(1 for g in gw if any(h is not None for h in g[3]))
        # stairs / bar / block: got across, did not touch, came back to its line
        if self.kind in ('stairs', 'bar', 'block') and walk:
            allw = walk + [e for e in self.trace if e['phase'] == 'stop']
            xmax = max(e['x'] for e in allw)
            goal = {'stairs': terrain.STAIRS_UP[-1] + 0.35, 'bar': terrain.BAR_X + terrain.BAR_DEPTH + 0.35,
                    'block': terrain.BLOCK_X + terrain.BLOCK_SIZE[0] + 0.35}[self.kind]
            out['crossing'] = {'x_max': round(xmax, 3), 'goal_x': goal, 'crossed': xmax > goal,
                               'final_y': round(allw[-1]['y'], 3), 'max_abs_y': round(max(abs(e['y']) for e in allw), 3)}
            if self.kind == 'stairs':
                out['crossing']['top_reached'] = xmax > terrain.STAIRS_UP[-1] + 0.25
                out['crossing']['down_reached'] = xmax > terrain.STAIRS_DOWN[-1] + 0.35
            if self.kind == 'block':
                # body (0.23 x 0.12) and feet (0.18 x 0.23 + reach) vs the block: gap in the world
                sx, sy = terrain.BLOCK_SIZE
                gap = min(max(abs(e['x'] - (terrain.BLOCK_X + sx / 2)) - sx / 2 - 0.15,
                              abs(e['y']) - sy / 2 - 0.13) for e in allw)
                out['crossing']['min_gap_m'] = round(gap, 3)
                # back on its line once past it (the walk goes on for metres after)
                past = [abs(e['y']) for e in allw if e['x'] > goal]
                out['crossing']['back_on_line'] = bool(past) and min(past) < 0.1
                out['crossing']['touched'] = gap < 0.0
        if self.kind == 'wall' and walk:
            xmax = max(e['x'] for e in walk + [e for e in self.trace if e['phase'] == 'stop'])
            out['wall'] = {'body_to_wall_m': round(terrain.WALL_X - xmax, 3),
                           'front_feet_to_wall_m': round(terrain.WALL_X - xmax - 0.09, 3),
                           'touched': terrain.WALL_X - xmax - 0.09 < 0.02}
        # detection
        if self.kind in ('steps', 'wall', 'stairs', 'bar', 'block'):
            out['detection'] = score_detection(self.kind, self.level, self.hazards, self.trace)
        # false reports: hazards not explained by a known feature
        def explained(h):
            x = h.get('world_x', h['robot'][0] + 0.3)
            if self.kind in ('steps', 'wall', 'stairs', 'bar', 'block'):
                return any(abs(x - e) < 0.35 or 0 < e - (h['robot'][0] + FRONT_FOOT) < 1.2
                           for _, _, e, _ in features(self.kind, self.level))
            if self.kind == 'slope':
                return abs(x - terrain.RAMP_START) < 0.35 or abs(h['robot'][0] + 0.4 - terrain.RAMP_START) < 0.4
            return self.kind in ('waves', 'rough')  # bumps everywhere: all reports are real
        fa = {}
        for src in FAMILIES:
            hs = [h for h in self.hazards if h['phase'] == 'walk' and h['source'].startswith(src)]
            fa[src] = {'reports': len(hs), 'unexplained': sum(not explained(h) for h in hs),
                       'unexplained_per_m': round(sum(not explained(h) for h in hs) / max(self.walked, 0.1), 2)}
        out['reports'] = fa
        # map
        if self.map:
            ox, oy, res, n = self.map[:4]
            n = int(n)
            m = np.array(self.map[4:], float).reshape(n, n)
            cx = ox + (np.arange(n) + 0.5) * res
            cy = oy + (np.arange(n) + 0.5) * res
            obs = terrain.obstacles(self.kind, self.level)
            err, cells = [], 0
            for i in range(n):
                for j in range(n):
                    if not math.isnan(m[i, j]):
                        cells += 1
                        err.append(m[i, j] - terrain.height(self.kind, self.level, cx[i], cy[j], obs))
            err = np.array(err)
            out['map'] = {'cells': cells, 'area_m2': round(cells * res * res, 2),
                          'rms_mm': round(1e3 * float(np.sqrt(np.mean(err ** 2))), 1) if cells else None,
                          'within_10mm': round(float(np.mean(np.abs(err) < 0.01)), 3) if cells else None}
        # cpu
        if self.stats_start and self.stats_start[1] and self.stats_end[1]:
            (t0, s0), (t1, s1) = self.stats_start, self.stats_end
            d = np.array(s1) - np.array(s0)
            wall = t1 - t0
            out['cpu'] = {'process_pct_of_core': round(100 * d[0] / wall, 1),
                          'messages_per_s': round(d[6] / wall, 1) if len(d) > 7 else None,
                          'callbacks_pct_of_core': round(100 * d[7] / 1e3 / wall, 1) if len(d) > 7 else None,
                          'lidar_cycles_per_s': round(d[1] / wall, 1),
                          'lidar_points_per_s': round(d[2] / wall, 0),
                          'lidar_ms_per_scan': round(d[3] / max(d[1], 1), 3),
                          'lidar_pct_of_core': round(100 * d[3] / 1e3 / wall, 2),
                          'tof_msgs_per_s': round(d[4] / wall, 1),
                          'tof_ms_per_msg': round(d[5] / max(d[4], 1), 4),
                          'tof_pct_of_core': round(100 * d[5] / 1e3 / wall, 2)}
            if len(d) >= 14:
                c = out['cpu']
                c['rates_per_s'] = {'joint_states': round(d[8] / wall, 1), 'imu': round(d[10] / wall, 1),
                                    'odom': round(d[12] / wall, 1), 'tof': c['tof_msgs_per_s'],
                                    'lidar': round(c['lidar_cycles_per_s'], 1)}
                c['ms_per_msg'] = {'joint_states': round(d[9] / max(d[8], 1), 4),
                                   'imu': round(d[11] / max(d[10], 1), 4),
                                   'odom': round(d[13] / max(d[12], 1), 4),
                                   'tof': c['tof_ms_per_msg'], 'lidar': c['lidar_ms_per_scan']}
                # rclpy's own cost per message (deserialisation, executor): the
                # process time not spent inside our callbacks
                if len(d) >= 17:
                    c['rates_per_s']['gs2'] = round(d[14] / wall, 1)
                    c['ms_per_msg']['gs2'] = round(d[16] / max(d[14], 1), 4)
                    c['gs2_points_per_s'] = round(d[15] / wall, 0)
                    c['gs2_pct_of_core'] = round(100 * d[16] / 1e3 / wall, 2)
                c['rclpy_ms_per_msg'] = round(1e3 * (d[0] - d[7] / 1e3) / max(d[6], 1), 4)
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--terrain', default='flat', choices=list(terrain.KINDS))
    ap.add_argument('--level', type=float, default=0.0)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--speed', type=float, default=0.12)
    ap.add_argument('--trace', help='write scores + full recording to this JSON file')
    ap.add_argument('--greet', action='store_true',
                    help='instead of walking: the greeting (sit, paws up, wave, stand up) on the spot')
    ap.add_argument('--expect', action='store_true',
                    help='exit 1 unless the lidars found every hazard in the path (steps) '
                         'and gave no unexplained report (other terrains)')
    ap.add_argument('--expect-guard', action='store_true',
                    help='exit 1 unless the hazard guard did the right thing: flat - never stopped and '
                         'slowed < 10 %% of the time; wall - stopped without touching it; any - no fall')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    chk = PerceptionCheck(args.terrain, args.level, args.seconds, args.speed, args.greet)
    try:
        ok = chk.run()
        res = chk.scores() if ok else {'error': 'no simulation'}
        print(json.dumps(res, indent=1, ensure_ascii=False))
        if ok and args.expect:
            missed = [n for n, v in res.get('detection', {}).items()
                      if not v.get('not_in_path') and not v.get('not_reached') and not v.get('lidar')]
            false = res['reports']['lidar']['unexplained_per_m']
            ok = not missed and false <= 1.0
            print(('PASS' if ok else 'FAIL') + f': missed {missed or "none"}, '
                  f'unexplained lidar reports {false}/m')
        if ok and args.expect_guard:
            g = res.get('guard') or {}
            why = []
            if res.get('fell'):
                why.append('fell')
            if args.terrain == 'flat':
                if g.get('stop', 0) > 0:
                    why.append(f"stopped on a flat floor ({g['stop']:.0%} of the time)")
                if g.get('caution', 0) + g.get('step_over', 0) > 0.10:
                    why.append('slowed down on a flat floor for more than 10 % of the time')
            if args.terrain in ('stairs', 'bar', 'block'):
                cr = res.get('crossing') or {}
                if not cr.get('crossed'):
                    why.append(f"did not get across (x {cr.get('x_max')} < {cr.get('goal_x')})")
                if args.terrain != 'block' and not g.get('crawl'):
                    why.append('never used the crawl')
                if args.terrain == 'block':
                    if cr.get('touched', True):
                        why.append(f"touched the block (gap {cr.get('min_gap_m')} m)")
                    if not cr.get('back_on_line'):
                        why.append('did not come back to its line after passing the block')
            if args.greet:
                gr = res.get('greet') or {}
                if not gr.get('finished'):
                    why.append('the greeting did not start or did not end standing')
                if gr.get('max_nose_up_deg', 0) < 25:
                    why.append(f"did not sit up (nose up {gr.get('max_nose_up_deg')} deg)")
            elif args.terrain == 'wall':
                w = res.get('wall') or {}
                if not g.get('stop'):
                    why.append('never stopped before the wall')
                if w.get('touched', True):
                    why.append(f"touched the wall (front feet {w.get('front_feet_to_wall_m')} m)")
            ok = not why
            print(('PASS' if ok else 'FAIL') + ': guard ' + ('; '.join(why) or f'{g}'))
        if args.trace:
            with open(args.trace, 'w') as f:
                json.dump({'terrain': args.terrain, 'level': args.level, 'scores': res, 'trace': chk.trace,
                           'sensors': SENSOR_PARAMS, 'stand_height': STAND_HEIGHT,
                           'hazards': chk.hazards, 'guard': chk.guard, 'ground': chk.ground, 'tof': chk.tof, 'scans': chk.scans,
                           'map': chk.map, 'states': chk.states}, f)
    finally:
        chk.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
