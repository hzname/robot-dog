"""Splits /dog/joint_commands (sensor_msgs/JointState) into one std_msgs/Float64
per joint for Gazebo's JointPositionController plugins (via ros_gz_bridge).
In simulation this node stands in for the servo driver."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from dog_description.urdf import joint_names, sim_command_topic


class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')
        ns = self.declare_parameter('robot_namespace', 'dog').value
        self.pubs = {j: self.create_publisher(Float64, sim_command_topic(ns, j), 10)
                     for j in joint_names()}
        self.create_subscription(JointState, 'joint_commands', self.on_commands, 10)

    def on_commands(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            pub = self.pubs.get(name)
            if pub is not None:
                pub.publish(Float64(data=float(pos)))


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
