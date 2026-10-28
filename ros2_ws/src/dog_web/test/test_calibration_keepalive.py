"""The calibration keep-alive keeps the driver's command timeout away.

A calibration session is mostly the operator looking at the leg, so without a
repeated pose the driver would relax and latch after a couple of seconds (SAF-04).
The bridge therefore republishes the last pose at 5 Hz while a session is open.
"""

import asyncio

import pytest
from builtin_interfaces.msg import Time

from dog_web.calibration import CalibrationBridge


class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeRclpyTime:
    def to_msg(self):
        return Time()


class FakeClock:
    def now(self):
        return FakeRclpyTime()


class FakeNode:
    """Just enough of an rclpy node for the bridge: no DDS, no graph."""

    def __init__(self, publisher, mode='passive'):
        self._publisher = publisher
        self.mode = mode

    def get_namespace(self):
        return '/dog'

    def create_publisher(self, *args, **kwargs):
        return self._publisher

    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, srv_type, name):  # noqa: ARG002 - the type is unused
        return type('Client', (), {'srv_name': name})()

    def get_clock(self):
        return FakeClock()


def _bridge(mode='passive'):
    publisher = FakePublisher()
    return CalibrationBridge(FakeNode(publisher, mode)), publisher


def test_pose_is_published_once():
    bridge, publisher = _bridge()
    bridge.pose({'lf_hip_joint': 10.0})
    assert len(publisher.sent) == 1
    assert publisher.sent[0].name == ['lf_hip_joint']
    assert publisher.sent[0].position[0] == pytest.approx(0.1745, abs=1e-3)


def test_keepalive_repeats_the_last_pose_and_stops_with_the_session():
    bridge, publisher = _bridge()

    async def scenario():
        bridge.pose({'lf_hip_joint': 10.0})
        bridge.open_session()
        await asyncio.sleep(0.5)          # 5 Hz over half a second
        during = len(publisher.sent)
        assert during >= 3, 'the keep-alive must repeat the pose'
        bridge.close_session()
        await asyncio.sleep(0.4)
        after = len(publisher.sent)
        await asyncio.sleep(0.4)
        assert len(publisher.sent) == after, 'the keep-alive must stop with the session'

    asyncio.run(scenario())


def test_two_sessions_keep_one_loop():
    bridge, publisher = _bridge()

    async def scenario():
        bridge.pose({'lf_hip_joint': 0.0})
        bridge.open_session()
        bridge.open_session()
        await asyncio.sleep(0.3)
        bridge.close_session()
        count = len(publisher.sent)
        await asyncio.sleep(0.3)
        assert len(publisher.sent) > count, 'one session is still open, so it keeps going'
        bridge.close_session()
        await asyncio.sleep(0.3)
        idle = len(publisher.sent)
        await asyncio.sleep(0.3)
        assert len(publisher.sent) == idle

    asyncio.run(scenario())


def test_keepalive_is_silent_once_the_robot_is_walking():
    bridge, publisher = _bridge(mode='passive')

    async def scenario():
        bridge.pose({'lf_hip_joint': 0.0})
        bridge.open_session()
        await asyncio.sleep(0.3)
        bridge.node.mode = 'walk'          # locomotion now publishes real commands
        count = len(publisher.sent)
        await asyncio.sleep(0.4)
        assert len(publisher.sent) == count, 'a walking robot must not get stale poses'
        bridge.close_session()

    asyncio.run(scenario())
