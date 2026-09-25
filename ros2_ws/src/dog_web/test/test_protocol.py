import json

import pytest

from dog_web import protocol

LIM = protocol.Limits()


def test_drive_is_scaled_and_clamped():
    a = protocol.handle_message('{"type":"drive","vx":1,"vy":-0.5,"wz":7}', LIM)
    assert not a.errors
    assert a.twist == pytest.approx((LIM.max_vx, -0.5 * LIM.max_vy, LIM.max_wz))


def test_stop_command_and_estop_zero_the_twist():
    assert protocol.handle_message('{"type":"stop"}', LIM).twist == (0.0, 0.0, 0.0)
    a = protocol.handle_message('{"type":"command","name":"stand"}', LIM)
    assert a.command == 'stand' and a.twist == (0.0, 0.0, 0.0)
    a = protocol.handle_message('{"type":"estop","active":true}', LIM)
    assert a.estop is True and a.twist == (0.0, 0.0, 0.0)
    a = protocol.handle_message('{"type":"estop","active":false}', LIM)
    assert a.estop is False and a.twist is None


def test_pose_limits():
    a = protocol.handle_message('{"type":"pose","pitch":-2,"height":1.0}', LIM)
    assert a.pose == pytest.approx((-LIM.max_pitch, LIM.max_height))


@pytest.mark.parametrize('text', [
    'not json', '[]', '{"type":"dance"}', '{"type":"command","name":"sit"}',
    '{"type":"drive","vx":"fast"}', '{"type":"drive","vx":NaN}', '{"type":"drive","vx":true}',
    '{"type":"estop","active":"yes"}',
])
def test_bad_messages_produce_errors_only(text):
    a = protocol.handle_message(text, LIM)
    assert a.errors
    assert a.twist is None and a.command is None and a.estop is None and a.pose is None


def test_watchdog_fires_once_after_stall():
    w = protocol.DriveWatchdog(0.4)
    assert not w.expired(10.0)
    w.feed((0.1, 0, 0), 10.0)
    assert not w.expired(10.3)
    assert w.expired(10.5)
    assert not w.expired(11.0)
    w.feed((0, 0, 0), 12.0)
    assert not w.expired(13.0)


def test_hello_and_state_are_json():
    assert json.loads(protocol.hello(LIM))['limits']['max_vx'] == LIM.max_vx
    s = json.loads(protocol.state('walk', False, 2))
    assert s == {'type': 'state', 'mode': 'walk', 'estop': False, 'clients': 2}


def test_guard_message():
    m = json.loads(protocol.guard('{"state": "stop", "max_vx": 0.0, "step": [null, null, null, null], "d": 0.2812}'))
    assert m == {'type': 'guard', 'state': 'stop', 'd': 0.28}
    m = json.loads(protocol.guard('{"state": "weird", "d": null}'))
    assert m == {'type': 'guard', 'state': 'clear', 'd': None}
