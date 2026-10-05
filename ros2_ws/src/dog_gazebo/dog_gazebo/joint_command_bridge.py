"""Splits /dog/joint_commands (sensor_msgs/JointState) into one std_msgs/Float64
per joint for Gazebo's JointPositionController plugins (via ros_gz_bridge).
In simulation this node stands in for the servo driver.

servo_model:=real adds the servo imperfections the SDF/DART cannot carry: a
gear backlash (a per-joint dead zone, parametrised here in degrees) and a
command delay (a queue released on the node clock - the simulation clock under
use_sim_time). Both parameters default to zero; with zeros the node forwards
every position in the same call, exactly as before (D-15)."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from dog_description.servo_profile import CommandShaper
from dog_description.urdf import joint_names, sim_command_topic

TICK_S = 0.002  # [s] delay queue drain period (500 Hz); simulation time under use_sim_time


class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')
        ns = self.declare_parameter('robot_namespace', 'dog').value
        backlash_deg = self.declare_parameter('backlash_deg', 0.0).value
        delay_s = self.declare_parameter('delay_s', 0.0).value
        self.shaper = CommandShaper(math.radians(backlash_deg), delay_s)
        self.pubs = {j: self.create_publisher(Float64, sim_command_topic(ns, j), 10)
                     for j in joint_names()}
        self.create_subscription(JointState, 'joint_commands', self.on_commands, 10)
        if backlash_deg != 0.0 or delay_s != 0.0:
            self.get_logger().info(
                f'servo profile: backlash {backlash_deg} deg, delay {delay_s} s')
        if delay_s > 0.0:
            self.create_timer(TICK_S, self.on_timer)

    def on_commands(self, msg: JointState):
        names, positions = [], []
        for name, pos in zip(msg.name, msg.position):
            if name in self.pubs:
                names.append(name)
                positions.append(float(pos))
        if names:
            now = self._now()
            self.shaper.push(now, names, positions)
            self._drain(now)

    def on_timer(self):
        self._drain(self._now())

    def _drain(self, now):
        for name, y in self.shaper.pop_ready(now):
            self.pubs[name].publish(Float64(data=y))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
