"""walk_check: drives the simulated dog through a fixed routine and checks the
ground-truth odometry from Gazebo. Exit code 0 = all maneuvers passed.

  ros2 launch dog_gazebo sim.launch.py headless:=true web:=false &
  ros2 run dog_gazebo walk_check

Thresholds are deliberately loose: an open-loop trot on a 1.5 kg servo dog
slips and drifts; this catches sign errors, falls and broken gaits.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class WalkCheck:
    def __init__(self):
        self.node = rclpy.create_node('walk_check', namespace='dog')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = None
        self.odom = None
        self.yaw_unwrapped = 0.0
        self._last_yaw = None
        self.node.create_subscription(String, 'state', self._on_state, latched)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.cmd = self.node.create_publisher(String, 'command', 10)
        self.vel = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.results = []

    def _on_state(self, msg):
        self.state = msg.data

    def _on_odom(self, msg):
        self.odom = msg
        yaw = _rpy(msg.pose.pose.orientation)[2]
        if self._last_yaw is not None:
            d = yaw - self._last_yaw
            self.yaw_unwrapped += math.atan2(math.sin(d), math.cos(d))
        self._last_yaw = yaw

    def spin(self, seconds, publish=None):
        end = time.time() + seconds
        worst_tilt = 0.0
        while time.time() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.odom:
                r, p, _ = _rpy(self.odom.pose.pose.orientation)
                worst_tilt = max(worst_tilt, abs(r), abs(p))
        return math.degrees(worst_tilt)

    def pose(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z, self.yaw_unwrapped

    def check(self, name, ok, detail):
        self.results.append((name, ok, detail))
        print('%-6s %-12s %s' % ('PASS' if ok else 'FAIL', name, detail), flush=True)

    def maneuver(self, name, vx, vy, wz, seconds, expect):
        x0, y0, _, yaw0 = self.pose()
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        tilt = self.spin(seconds, lambda: self.vel.publish(t))
        tilt = max(tilt, self.spin(1.5))  # coast to a stop
        x1, y1, z1, yaw1 = self.pose()
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)
        dyaw = yaw1 - yaw0
        moved = {'x': dx, 'y': dy, 'yaw': dyaw}
        axis, target = expect
        ratio = moved[axis] / target
        ok = ratio > 0.4 and tilt < 20.0 and z1 > 0.12
        self.check(name, ok, 'dx=%+.2fm dy=%+.2fm dyaw=%+.0fdeg  (%d%% of command)  tilt<=%.0fdeg z=%.3f' % (
            dx, dy, math.degrees(dyaw), 100 * ratio, tilt, z1))

    def run(self):
        print('waiting for simulation...', flush=True)
        end = time.time() + 60
        while (self.odom is None or self.state is None) and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None or self.state is None:
            print('FAIL: no /dog/odom or /dog/state - is sim.launch.py running?')
            return 1
        self.spin(2.0)
        self.spin(4.0, lambda: self.cmd.publish(String(data='stand'))
                  if self.state in ('passive', 'lying') else None)
        self.spin(1.0)
        z = self.pose()[2]
        self.check('stand', self.state == 'stand' and 0.14 < z < 0.19,
                   'state=%s z=%.3f' % (self.state, z))
        T = 5.0
        self.maneuver('forward', 0.12, 0, 0, T, ('x', 0.12 * T))
        self.maneuver('backward', -0.10, 0, 0, T, ('x', -0.10 * T))
        self.maneuver('left', 0, 0.06, 0, T, ('y', 0.06 * T))
        self.maneuver('right', 0, -0.06, 0, T, ('y', -0.06 * T))
        self.maneuver('turn_ccw', 0, 0, 0.5, T, ('yaw', 0.5 * T))
        self.maneuver('turn_cw', 0, 0, -0.5, T, ('yaw', -0.5 * T))
        self.cmd.publish(String(data='lie'))
        self.spin(3.0)
        z = self.pose()[2]
        self.check('lie', self.state == 'lying' and z < 0.12, 'state=%s z=%.3f' % (self.state, z))
        failed = [r for r in self.results if not r[1]]
        print('%d/%d passed' % (len(self.results) - len(failed), len(self.results)))
        return 1 if failed else 0


def main():
    rclpy.init()
    checker = WalkCheck()
    try:
        code = checker.run()
    finally:
        checker.node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
