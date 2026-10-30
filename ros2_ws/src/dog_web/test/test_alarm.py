"""The alarm banner: one line, the worst node's own words (DIA-02).

The page has room for one banner, so the rules are worth pinning: an ERROR wins
over a WARN whatever order the statuses arrive in, ties keep the first one, and
nothing at all means "all clear" rather than "unknown".
"""

import json

from dog_web import protocol


def test_an_error_outranks_a_warning_whatever_the_order():
    warn = ('warn', 'locomotion', 'tilt 24 deg: the walk slows down')
    error = ('error', 'servo_driver', 'bus fault latched: write failed ch=3')
    for items in ([warn, error], [error, warn]):
        level, source, reason = protocol.worst_status(items)
        assert level == 'error'
        assert source == 'servo_driver'
        assert 'bus fault' in reason


def test_a_warning_shows_when_nothing_is_worse():
    level, source, _ = protocol.worst_status(
        [('ok', 'imu', 'calibrated'), ('warn', 'power_monitor', 'servo current high')])
    assert (level, source) == ('warn', 'power_monitor')


def test_all_clear_is_ok_with_nothing_to_say():
    assert protocol.worst_status([]) == ('ok', '', '')
    assert protocol.worst_status([('ok', 'imu', 'calibrated')])[0] == 'ok'


def test_an_unknown_level_does_not_hide_a_real_fault():
    level, source, _ = protocol.worst_status(
        [('catastrophe', 'x', 'nonsense'), ('warn', 'power_monitor', 'low rail')])
    assert (level, source) == ('warn', 'power_monitor')


def test_the_banner_message_carries_the_reason_verbatim():
    text = protocol.alarm('error', 'servo_driver', 'bus fault latched: no joint commands', 12.3456)
    msg = json.loads(text)
    assert msg['type'] == 'alarm'
    assert msg['level'] == 'error'
    assert msg['source'] == 'servo_driver'
    assert msg['reason'] == 'bus fault latched: no joint commands'
    assert msg['since'] == 12.3
