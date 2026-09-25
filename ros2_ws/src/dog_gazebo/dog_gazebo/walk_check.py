"""walk_check: drives the simulated dog through a fixed routine and checks the
ground-truth odometry from Gazebo. Exit code 0 = all maneuvers passed.

  ros2 launch dog_gazebo sim.launch.py headless:=true web:=false &
  ros2 run dog_gazebo walk_check
  ros2 run dog_gazebo walk_check --trace run.json   # also save the odometry trace
  ros2 run dog_gazebo walk_check --trace run.json --record   # + joints and IMU at 30 Hz

Thresholds are deliberately loose: an open-loop trot on a 1.5 kg servo dog
slips and drifts; this catches sign errors, falls and broken gaits.

On terrain (sim.launch.py terrain:=slope|waves|rough) pass the same terrain
so tilt and body height are judged against the ground, not the horizon:
  ros2 run dog_gazebo walk_check --terrain slope --level 10
Distances are measured along the slope in the robot's starting frame.
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from std_msgs.msg import String

from dog_gazebo import terrain


def _rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def _rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# Body sag limit: 42 mm under the stand height (0.15 m). Before the foot fix
# (calf to the contact point) the simulated body stood at 0.162 m and the
# limit read 0.12 m; the same physical criterion is 0.108 m now.
MIN_BODY_HEIGHT = 0.108

class WalkCheck:
    def __init__(self, kind='flat', level=0.0, min_ratio=0.4, max_tilt=20.0, seconds=5.0,
                 record=False):
        self.normal = np.array(terrain.normal(kind, level))
        self.kind, self.level = kind, level
        self.min_ratio, self.max_tilt, self.seconds = min_ratio, max_tilt, seconds
        self.fallen = False
        self.record = record
        self.joints = {}
        self.imu = None
        self.node = rclpy.create_node('walk_check', namespace='dog')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = None
        self.odom = None
        self.yaw_unwrapped = 0.0
        self._last_yaw = None
        self.node.create_subscription(String, 'state', self._on_state, latched)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        if record:
            self.node.create_subscription(
                JointState, 'joint_states',
                lambda m: self.joints.update(zip(m.name, m.position)), 10)
            self.node.create_subscription(
                Imu, 'imu/data', lambda m: setattr(self, 'imu', m), qos_profile_sensor_data)
        self.cmd = self.node.create_publisher(String, 'command', 10)
        self.vel = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.results = []
        self.phase = 'setup'
        self.trace = []
        self._t0 = time.time()

    def _on_state(self, msg):
        self.state = msg.data

    def _on_odom(self, msg):
        self.odom = msg
        yaw = _rpy(msg.pose.pose.orientation)[2]
        if self._last_yaw is not None:
            d = yaw - self._last_yaw
            self.yaw_unwrapped += math.atan2(math.sin(d), math.cos(d))
        self._last_yaw = yaw
        t = time.time() - self._t0
        if not self.trace or t - self.trace[-1]['t'] >= (1 / 30 if self.record else 0.05):
            p = msg.pose.pose.position
            r, pi, _ = _rpy(msg.pose.pose.orientation)
            self.trace.append({'t': round(t, 3), 'phase': self.phase, 'state': self.state,
                               'x': round(p.x, 4), 'y': round(p.y, 4), 'z': round(p.z, 4),
                               'roll': round(math.degrees(r), 2), 'pitch': round(math.degrees(pi), 2),
                               'yaw': round(math.degrees(self.yaw_unwrapped), 2)})
            if self.record:
                q = msg.pose.pose.orientation
                e = self.trace[-1]
                e['q'] = [round(v, 5) for v in (q.x, q.y, q.z, q.w)]
                e['j'] = {k: round(v, 4) for k, v in self.joints.items()}
                if self.imu is not None:
                    ir, ip, _ = _rpy(self.imu.orientation)
                    e['imu'] = [round(math.degrees(ir), 2), round(math.degrees(ip), 2)]

    def spin(self, seconds, publish=None):
        end = time.time() + seconds
        worst_tilt = 0.0
        while time.time() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.odom:
                worst_tilt = max(worst_tilt, self.tilt())
        if worst_tilt > 60.0:
            self.fallen = True
        return worst_tilt

    def ground(self):
        p = self.odom.pose.pose.position
        h, n = terrain.surface(self.kind, self.level, p.x)
        return h, np.array(n)

    def tilt(self):
        """Angle between the body's up axis and the local ground normal [deg]."""
        up = _rot(self.odom.pose.pose.orientation)[:, 2]
        _, n = self.ground()
        return math.degrees(math.acos(max(-1.0, min(1.0, float(up @ n)))))

    def height(self):
        """Body height above the local ground surface, along its normal [m]."""
        p = self.odom.pose.pose.position
        h, n = self.ground()
        return float((p.z - h) * n[2])

    def pose(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z, self.yaw_unwrapped

    def check(self, name, ok, detail, **values):
        self.results.append((name, ok, detail, values))
        print('%-6s %-12s %s' % ('PASS' if ok else 'FAIL', name, detail), flush=True)

    def maneuver(self, name, vx, vy, wz, seconds, expect):
        self.phase = name
        if self.fallen:
            self.check(name, False, 'skipped: robot has fallen', fallen=True)
            return
        p0 = self.odom.pose.pose.position
        R0 = _rot(self.odom.pose.pose.orientation)
        yaw0 = self.yaw_unwrapped
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        tilt = self.spin(seconds, lambda: self.vel.publish(t))
        # release the stick like an operator: zero twist, then coast to a stop
        tilt = max(tilt, self.spin(1.5, lambda: self.vel.publish(Twist())))
        p1 = self.odom.pose.pose.position
        d = np.array([p1.x - p0.x, p1.y - p0.y, p1.z - p0.z])
        dx, dy = float(d @ R0[:, 0]), float(d @ R0[:, 1])  # along the body axes (on the slope)
        dyaw = self.yaw_unwrapped - yaw0
        z1 = self.height()
        moved = {'x': dx, 'y': dy, 'yaw': dyaw}
        axis, target = expect
        ratio = moved[axis] / target
        ok = ratio > self.min_ratio and tilt < self.max_tilt and z1 > MIN_BODY_HEIGHT and not self.fallen
        self.check(name, ok, 'dx=%+.2fm dy=%+.2fm dyaw=%+.0fdeg  (%d%% of command)  tilt<=%.0fdeg z=%.3f' % (
            dx, dy, math.degrees(dyaw), 100 * ratio, tilt, z1),
            cmd=[vx, vy, wz], seconds=seconds, dx=dx, dy=dy, dyaw_deg=math.degrees(dyaw),
            ratio=ratio, tilt_deg=tilt, z=z1)

    def run(self):
        print('waiting for simulation...', flush=True)
        end = time.time() + 60
        while (self.odom is None or self.state is None) and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None or self.state is None:
            print('FAIL: no /dog/odom or /dog/state - is sim.launch.py running?')
            return 1
        self.spin(2.0)
        self.phase = 'stand'
        self.spin(4.0, lambda: self.cmd.publish(String(data='stand'))
                  if self.state in ('passive', 'lying') else None)
        self.spin(1.0)
        z = self.height()
        self.check('stand', self.state == 'stand' and 0.14 < z < 0.19 and self.tilt() < self.max_tilt,
                   'state=%s z=%.3f tilt=%.0fdeg' % (self.state, z, self.tilt()), z=z)
        T = self.seconds
        if self.kind == 'slope':
            # climb onto the ramp, traverse and turn on it, then walk back down
            self.maneuver('forward', 0.12, 0, 0, T, ('x', 0.12 * T))
            self.maneuver('left', 0, 0.06, 0, T, ('y', 0.06 * T))
            self.maneuver('right', 0, -0.06, 0, T, ('y', -0.06 * T))
            self.maneuver('turn_ccw', 0, 0, 0.5, T * 0.5, ('yaw', 0.25 * T))
            self.maneuver('turn_cw', 0, 0, -0.5, T * 0.5, ('yaw', -0.25 * T))
            self.maneuver('backward', -0.10, 0, 0, T * 1.4, ('x', -0.14 * T))
        else:
            self.maneuver('forward', 0.12, 0, 0, T, ('x', 0.12 * T))
            self.maneuver('backward', -0.10, 0, 0, T, ('x', -0.10 * T))
            self.maneuver('left', 0, 0.06, 0, T, ('y', 0.06 * T))
            self.maneuver('right', 0, -0.06, 0, T, ('y', -0.06 * T))
            self.maneuver('turn_ccw', 0, 0, 0.5, T, ('yaw', 0.5 * T))
            self.maneuver('turn_cw', 0, 0, -0.5, T, ('yaw', -0.5 * T))
        self.phase = 'lie'
        self.cmd.publish(String(data='lie'))
        # finishes the steps first; wall-clock wait, so allow for a slow simulation
        end = time.time() + 10.0
        while self.state != 'lying' and time.time() < end and not self.fallen:
            self.spin(0.5)
        self.spin(0.5)
        z = self.height()
        self.check('lie', self.state == 'lying' and z < 0.12 and not self.fallen,
                   'state=%s z=%.3f' % (self.state, z), z=z)
        failed = [r for r in self.results if not r[1]]
        print('%d/%d passed' % (len(self.results) - len(failed), len(self.results)))
        return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description='Drive the simulated dog and check the odometry.')
    ap.add_argument('--trace', help='write results + odometry trace to this JSON file')
    ap.add_argument('--terrain', default='flat', choices=['flat', 'slope', 'waves', 'rough'])
    ap.add_argument('--level', type=float, default=0.0, help='slope [deg] or obstacle height [mm]')
    ap.add_argument('--min-ratio', type=float, default=0.4, help='share of the command to pass')
    ap.add_argument('--max-tilt', type=float, default=20.0, help='body tilt vs. the ground [deg]')
    ap.add_argument('--seconds', type=float, default=5.0, help='duration of each maneuver')
    ap.add_argument('--record', action='store_true',
                    help='with --trace: also record joints and IMU at 30 Hz')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    checker = WalkCheck(args.terrain, args.level, args.min_ratio, args.max_tilt, args.seconds,
                        args.record)
    try:
        code = checker.run()
        if args.trace:
            with open(args.trace, 'w') as f:
                json.dump({'terrain': args.terrain, 'level': args.level,
                           'results': [{'name': n, 'ok': ok, 'detail': d, **v}
                                       for n, ok, d, v in checker.results],
                           'trace': checker.trace}, f)
    finally:
        checker.node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
