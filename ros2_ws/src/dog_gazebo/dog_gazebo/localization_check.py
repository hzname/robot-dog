"""localization_check: the robot maps the room world, then finds itself in the
stored map from another start and follows its route (docs/LOCALIZATION.md).

  # 1. mapping from start 0: survey, a loop round the room, map saved
  ros2 launch dog_gazebo sim.launch.py headless:=true web:=false terrain:=room seed:=0 \\
      dead_reckoning:=true gyro_bias:=0.3 localization:=true localization_mode:=mapping map:=/tmp/room &
  ros2 run dog_gazebo localization_check --seed 0 --phase mapping --trace map.json --expect
  # 2. localization from start 1 in that map
  ros2 launch dog_gazebo sim.launch.py ... terrain:=room seed:=1 ... localization_mode:=localize map:=/tmp/room &
  ros2 run dog_gazebo localization_check --seed 1 --map-seed 0 --phase localize --trace loc.json --expect

The robot is steered along the route by the TRUE pose (Gazebo's odom): the
check is a test rig, not a navigation stack. What it scores is the robot's
own estimate - localization/pose - against the truth, next to dead reckoning
(odom_dr: the walked twist and a gyro heading with a bias) against the truth.
The map frame is where the robot stood when the mapping began, in dead
reckoning's frame then: a few cm and degrees off the world's origin after
standing up with a drifting gyro, and fixed by nothing but that moment. So
the mapping run fits the rigid transform that best puts its estimates on the
truth (what is left is the map's own distortion) and writes it to
<map>.truth.json (--map). A run from another start reads it and carries its
truth into the same frame through the room's placement in both worlds: its
errors say whether the robot finds the same place at the same map
coordinates.
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from dog_gazebo import terrain

# route round the room (room coordinates): clear of the furniture by >= 0.35 m
ROUTE = ((-0.9, -0.5), (1.0, -0.5), (1.2, 0.4), (-0.5, 0.5))


def house_route(seed, laps):
    """Round the house's core from the start, the way the robot faces: the
    corners in turn, `laps` times, then on past the first corner to half the
    next side - over ground mapped at the start, where the loop closes."""
    sx, sy, syaw = terrain.HOUSE_STARTS[seed % len(terrain.HOUSE_STARTS)]
    corners = terrain.HOUSE_LOOP  # counter-clockwise
    ccw = sx * math.sin(syaw) - sy * math.cos(syaw) > 0  # heading x position (about the core's centre)
    a0 = math.atan2(sy, sx)

    def ahead(c):  # angle still to go round the centre to reach corner c
        d = math.atan2(c[1], c[0]) - a0
        return (d if ccw else -d) % (2 * math.pi) or 2 * math.pi
    first = min(range(4), key=lambda k: ahead(corners[k]))
    step = 1 if ccw else -1
    order = [corners[(first + step * k) % 4] for k in range(4)]
    route = order * laps + [order[0]]
    a, b = order[0], order[1]
    route.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
    return route


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def _inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


def _fit(pairs):
    """Rigid 2D transform (x, y, yaw) that best maps the first poses of the
    pairs onto the second (least squares over the positions)."""
    a = np.array([p[0][:2] for p in pairs])
    b = np.array([p[1][:2] for p in pairs])
    ma, mb = a.mean(0), b.mean(0)
    h = (a - ma).T @ (b - mb)
    th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(th), math.sin(th)
    t = mb - np.array([[c, -s], [s, c]]) @ ma
    return (float(t[0]), float(t[1]), th)


def _true_segments(world, seed):
    """Footprint edges of the world's walls and furniture (world frame)."""
    segs = []
    for b in terrain.obstacles(world, 0, seed):
        sx, sy = b['size'][:2]
        c, s = math.cos(b['yaw']), math.sin(b['yaw'])
        pts = [(b['x'] + c * u - s * v, b['y'] + s * u + c * v)
               for u, v in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
        segs += [(pts[k], pts[(k + 1) % 4]) for k in range(4)]
    a = np.array([p[0] for p in segs])
    b = np.array([p[1] for p in segs])
    return a, b


def _nearest_on(pts, a, b):
    """Nearest point on any segment (a[i], b[i]) for each of pts, and the distance."""
    d = b - a
    t = np.clip(((pts[:, None, :] - a[None]) * d[None]).sum(-1) / (d * d).sum(-1)[None], 0.0, 1.0)
    q = a[None] + t[..., None] * d[None]
    dist = np.linalg.norm(pts[:, None, :] - q, axis=-1)
    k = dist.argmin(1)
    return q[np.arange(len(pts)), k], dist[np.arange(len(pts)), k]


def map_to_world_icp(walls, world, seed, init, iterations=30):
    """Place the map's walls on the true ones (ICP, point to segment) from
    `init`: (map -> world transform, distances of the walls after it)."""
    a, b = _true_segments(world, seed)
    off = tuple(init)
    for _ in range(iterations):
        c, s = math.cos(off[2]), math.sin(off[2])
        w = walls @ np.array([[c, s], [-s, c]]) + np.array(off[:2])
        q, dist = _nearest_on(w, a, b)
        keep = dist < max(0.3, np.percentile(dist, 80))
        step = _fit([(tuple(p), tuple(t)) for p, t in zip(w[keep], q[keep])])
        off = _compose(step, off)
        if math.hypot(step[0], step[1]) < 1e-4 and abs(step[2]) < 1e-5:
            break
    c, s = math.cos(off[2]), math.sin(off[2])
    w = walls @ np.array([[c, s], [-s, c]]) + np.array(off[:2])
    return off, _nearest_on(w, a, b)[1]


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class LocalizationCheck:
    def __init__(self, seed, map_seed, phase, speed, laps, survey, map_path=None, world='room'):
        self.seed, self.phase, self.speed, self.laps, self.survey = seed, phase, speed, laps, survey
        self.map_path = map_path
        self.map_offset = None  # map frame -> the mapping run's world frame
        if phase == 'localize' and map_path:
            with open(map_path + '.truth.json') as f:
                self.map_offset = tuple(json.load(f)['map_to_world'])
        # world of this run -> world of the mapping run (the room placed in both)
        self.world = world
        self.route = [tuple(p) for p in ROUTE] * laps if world == 'room' else house_route(seed, laps)
        W, M = terrain.room_to_world(seed, world), terrain.room_to_world(map_seed, world)
        self.world_to_map = _compose(M, _inverse(W))
        self.room_to_world = W
        self.node = rclpy.create_node('localization_check', namespace='dog')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.truth = self.dr = self.loc = self.status = self.state = None
        self.loc_t = -1.0
        self.states, self.trace = [], []
        n = self.node
        n.create_subscription(Odometry, 'odom', self.on_truth, 10)
        n.create_subscription(Odometry, 'odom_dr', lambda m: setattr(self, 'dr', m), 10)
        n.create_subscription(PoseStamped, 'localization/pose', self.on_loc, 10)
        n.create_subscription(String, 'localization/status', lambda m: setattr(self, 'status', json.loads(m.data)),
                              latched)
        n.create_subscription(String, 'state', self.on_state, latched)
        self.cmd = n.create_publisher(String, 'command', 10)
        self.loc_cmd = n.create_publisher(String, 'localization/command', 10)
        self.vel = n.create_publisher(Twist, 'cmd_vel', 10)
        self.phase_name = 'start'
        self.last_rec = -1.0

    # ------------------------------------------------------------ inputs
    def now(self):
        s = self.truth.header.stamp
        return s.sec + s.nanosec * 1e-9

    def on_state(self, m):
        self.state = m.data
        if self.truth is not None:
            self.states.append([round(self.now(), 2), m.data])

    def on_loc(self, m):
        self.loc = m
        self.loc_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def pose_world(self):
        p = self.truth.pose.pose
        return (p.position.x, p.position.y, _yaw(p.orientation))

    def on_truth(self, m):
        self.truth = m
        t = self.now()
        if t - self.last_rec < 0.2:
            return
        self.last_rec = t
        tm = _compose(self.world_to_map, self.pose_world())
        e = {'t': round(t, 2), 'phase': self.phase_name, 'state': self.state,
             'truth': [round(v, 4) for v in tm],
             'world': [round(v, 4) for v in self.pose_world()]}
        if self.dr is not None:
            p = self.dr.pose.pose
            e['dr'] = [round(p.position.x, 4), round(p.position.y, 4), round(_yaw(p.orientation), 4)]
        if self.loc is not None and t - self.loc_t < 0.5:
            p = self.loc.pose
            e['loc'] = [round(p.position.x, 4), round(p.position.y, 4), round(_yaw(p.orientation), 4)]
        if self.status is not None:
            e['status'] = self.status.get('status')
            e['inliers'] = self.status.get('inliers')
            e['loops'] = self.status.get('loops')
        self.trace.append(e)

    # ------------------------------------------------------------ driving
    def spin(self, seconds, publish=None):
        end = time.time() + seconds
        while time.time() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin_until(self, cond, sim_seconds, publish=None):
        """Until cond() or `sim_seconds` of simulated time (at most 4x by the clock)."""
        t_end = self.now() + sim_seconds
        end = time.time() + 4 * sim_seconds
        while not cond() and self.now() < t_end and time.time() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return cond()

    def goto(self, gx, gy, timeout):
        """Walk to (gx, gy) in the world, turning towards it on the way."""
        def step():
            x, y, yaw = self.pose_world()
            err = _wrap(math.atan2(gy - y, gx - x) - yaw)
            tw = Twist()
            tw.angular.z = max(-0.4, min(0.4, 1.2 * err))
            tw.linear.x = self.speed if abs(err) < 0.35 else 0.0
            self.vel.publish(tw)
        ok = self.spin_until(lambda: math.hypot(gx - self.pose_world()[0], gy - self.pose_world()[1]) < 0.12,
                             timeout, step)
        self.vel.publish(Twist())
        return ok

    def run(self):
        end = time.time() + 90
        while (self.truth is None or self.state is None) and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.truth is None:
            print('FAIL: no simulation')
            return False
        self.spin(2.0)
        self.phase_name = 'stand'
        self.spin(3.0, lambda: self.cmd.publish(String(data='stand')) if self.state in ('passive', 'lying') else None)
        self.spin_until(lambda: self.state == 'stand', 60)
        self.spin_until(lambda: False, 1.0)
        self.t_start = self.now()
        if self.survey:
            self.phase_name = 'survey'
            self.cmd.publish(String(data='survey'))
            self.spin_until(lambda: self.state == 'survey', 3.0)
            self.spin_until(lambda: self.state == 'stand', 30.0)
        self.phase_name = 'relocalize'
        self.t_survey_end = self.now()
        self.spin_until(lambda: self.status is not None and self.status.get('status') == 'tracking', 20.0)
        self.t_tracking = self.now() if self.status and self.status.get('status') == 'tracking' else None
        self.phase_name = 'route'
        self.reached = 0
        room = self.room_to_world
        for rx, ry in self.route:
            gx, gy, _ = _compose(room, (rx, ry, 0.0))
            if self.goto(gx, gy, 120.0):
                self.reached += 1
        self.phase_name = 'end'
        self.spin_until(lambda: False, 2.0, lambda: self.vel.publish(Twist()))
        if self.phase == 'mapping':
            self.loc_cmd.publish(String(data='save'))
            self.spin(1.5)
        return True

    # ------------------------------------------------------------ scores
    def scores(self):
        out = {'phase': self.phase, 'world': self.world, 'seed': self.seed, 'route_points': len(self.route),
               'reached': self.reached}
        tr = [e for e in self.trace if e['phase'] in ('route', 'end')]
        if self.t_tracking is not None:
            out['relocalized_s'] = round(self.t_tracking - self.t_survey_end, 1)
        else:
            out['relocalized_s'] = None

        def first_offset(key):  # its frame starts where the robot stood: put it there
            first = next((e for e in self.trace if key in e), None)
            return None if first is None else _compose(tuple(first['truth']), _inverse(tuple(first[key])))

        def err(key, off):
            es = [e for e in tr if key in e]
            if not es or off is None:
                return None
            pos, yaw = [], []
            for e in es:
                p = _compose(off, tuple(e[key]))
                pos.append(math.hypot(p[0] - e['truth'][0], p[1] - e['truth'][1]))
                yaw.append(abs(math.degrees(_wrap(p[2] - e['truth'][2]))))
            return {'mean_m': round(float(np.mean(pos)), 3), 'p95_m': round(float(np.percentile(pos, 95)), 3),
                    'max_m': round(float(np.max(pos)), 3), 'final_m': round(pos[-1], 3),
                    'yaw_p95_deg': round(float(np.percentile(yaw, 95)), 2), 'yaw_final_deg': round(yaw[-1], 2),
                    'samples': len(es)}
        if self.phase == 'mapping':
            # the saved map's walls placed on the true ones: how good the map
            # is, and where its frame is in the world (for the other runs)
            pairs = [(e['loc'], e['truth']) for e in self.trace
                     if 'loc' in e and e['phase'] in ('survey', 'relocalize', 'route', 'end')]
            init = _fit(pairs) if len(pairs) > 10 else (0.0, 0.0, 0.0)
            self.map_offset = init
            walls_file = (self.map_path or '') + '.walls'
            try:
                walls = np.loadtxt(walls_file).reshape(-1, 2)
            except OSError:
                walls = np.zeros((0, 2))
            if len(walls) > 50:
                self.map_offset, dist = map_to_world_icp(walls, self.world, self.seed, init)
                out['map_walls'] = {'mean_m': round(float(dist.mean()), 3),
                                    'p95_m': round(float(np.percentile(dist, 95)), 3),
                                    'within_5cm': round(float((dist < 0.05).mean()), 3), 'cells': len(walls)}
            out['map_to_world'] = [round(v, 4) for v in self.map_offset]
        out['localization'] = err('loc', self.map_offset)
        out['dead_reckoning'] = err('dr', first_offset('dr'))
        out['tracking_share'] = round(sum(1 for e in tr if e.get('status') == 'tracking') / max(len(tr), 1), 3)
        if tr:
            walked = sum(math.hypot(b['truth'][0] - a['truth'][0], b['truth'][1] - a['truth'][1])
                         for a, b in zip(tr, tr[1:]))
            out['walked_m'] = round(walked, 2)
        if self.status:
            out['map_cells'] = self.status.get('cells')
            out['submaps'] = self.status.get('submaps')
            out['loops'] = self.status.get('loops')
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', choices=('room', 'house'), default='room')
    ap.add_argument('--seed', type=int, default=0, help='start of this run (terrain.ROOM_STARTS / HOUSE_STARTS)')
    ap.add_argument('--map-seed', type=int, default=None, help='start of the mapping run (default: --seed)')
    ap.add_argument('--phase', choices=('mapping', 'localize'), default='mapping')
    ap.add_argument('--speed', type=float, default=0.12)
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--no-survey', action='store_true')
    ap.add_argument('--trace', help='write scores + the recording to this JSON file')
    ap.add_argument('--map', help='map path (no extension): mapping writes <map>.truth.json, '
                                  'localize reads it (where the map frame is in the world)')
    ap.add_argument('--expect', action='store_true',
                    help='exit 1 unless: the route was walked; localize - found within 20 s of the survey, '
                         'tracking >= 90 %% of the route, position p95 <= 0.05 m, heading p95 <= 4 deg '
                         '(in the map frame placed on the world by the mapping run)')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    chk = LocalizationCheck(args.seed, args.seed if args.map_seed is None else args.map_seed, args.phase,
                            args.speed, args.laps, not args.no_survey, args.map, args.world)
    ok = False
    try:
        ok = chk.run()
        res = chk.scores() if ok else {'error': 'no simulation'}
        print(json.dumps(res, indent=1))
        if ok and args.phase == 'mapping' and args.map and res.get('map_to_world'):
            with open(args.map + '.truth.json', 'w') as f:
                json.dump({'map_to_world': res['map_to_world'], 'seed': args.seed, 'world': args.world}, f)
        if args.trace:
            with open(args.trace, 'w') as f:
                json.dump({'scores': res, 'states': chk.states, 'trace': chk.trace,
                           'seed': args.seed, 'world_to_map': chk.world_to_map}, f)
        if ok and args.expect:
            why = []
            if res['reached'] < res['route_points']:
                why.append(f"reached {res['reached']} of {res['route_points']} route points")
            if args.phase == 'localize' and (res['relocalized_s'] is None or res['relocalized_s'] > 20):
                why.append('not relocalized within 20 s after the survey')
            if res['tracking_share'] < 0.9:
                why.append(f"tracking only {100 * res['tracking_share']:.0f} % of the route")
            lo = res.get('localization') or {}
            if lo.get('p95_m', 1) > 0.05:
                why.append(f"position error p95 {lo.get('p95_m')} m")
            if lo.get('yaw_p95_deg', 99) > 4.0:
                why.append(f"heading error p95 {lo.get('yaw_p95_deg')} deg")
            ok = not why
            print(('PASS' if ok else 'FAIL') + ': localization ' + ('; '.join(why) if why else
                  f"p95 {lo.get('p95_m')} m / {lo.get('yaw_p95_deg')} deg, dead reckoning final "
                  f"{(res.get('dead_reckoning') or {}).get('final_m')} m"))
    finally:
        chk.vel.publish(Twist())
        chk.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
