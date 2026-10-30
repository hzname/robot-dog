#!/usr/bin/env python3
"""Wait for the latched /dog/state inside the container (CAL-09 smoke).

`ros2 topic echo` subscribes volatile, and /dog/state is published latched: a newcomer
only sees the next change of state, which may be minutes away. Here the subscriber asks
for transient_local durability, so the stored message is delivered immediately.

One process, its stderr visible, and the message printed - under emulation the CLI would
be another slow start whose errors we would not see.
"""

import sys
import time

try:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except Exception as exc:                     # noqa: BLE001
    print('smoke: cannot import rclpy: %r' % (exc,), file=sys.stderr)
    sys.exit(1)

BUDGET_S = 90.0


def main():
    rclpy.init()
    node = rclpy.create_node('smoke_state_probe')
    got = []
    qos = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/dog/state', lambda m: got.append(m.data), qos)
    deadline = time.time() + BUDGET_S
    try:
        while time.time() < deadline and not got:
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not got:
        print('smoke: /dog/state said nothing - even the latched copy', file=sys.stderr)
        return 1
    print('smoke: /dog/state answered: %s' % got[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
