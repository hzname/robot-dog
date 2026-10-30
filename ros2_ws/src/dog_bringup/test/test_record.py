"""The recording list is a contract: every name must still exist in the sources.

No ROS here - the launch module is loaded with stubbed launch imports, so this runs
in a plain python job and fails fast when a topic is renamed and the bag silently
loses it.
"""

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, '..', 'launch', 'record.launch.py')
SRC = os.path.dirname(os.path.dirname(HERE))


def _load():
    for name in ('launch', 'launch.actions', 'launch.substitutions'):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['launch'].LaunchDescription = lambda *a: None
    sys.modules['launch.actions'].DeclareLaunchArgument = lambda *a, **k: None
    sys.modules['launch.actions'].ExecuteProcess = lambda *a, **k: None
    sys.modules['launch.substitutions'].LaunchConfiguration = lambda *a, **k: None
    import importlib.util
    spec = importlib.util.spec_from_file_location('record_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


record = _load()


def _sources():
    for root, _, files in os.walk(SRC):
        for f in files:
            if f.endswith(('.cpp', '.hpp', '.py')) and 'test' not in root:
                yield os.path.join(root, f)


def test_the_list_is_flat_and_namespaced():
    assert len(record.TOPICS) == len(set(record.TOPICS)), 'a topic is listed twice'
    assert all(t.startswith('/dog/') for t in record.TOPICS), record.TOPICS


def test_every_recorded_topic_exists_in_the_sources():
    text = ''
    for path in _sources():
        with open(path, encoding='utf-8', errors='replace') as f:
            text += f.read()
    missing = [t for t in record.TOPICS if '"%s"' % t.split('/', 2)[2] not in text
               and "'%s'" % t.split('/', 2)[2] not in text]
    assert not missing, 'these topics no longer appear in the code: %s' % missing


def test_the_post_mortem_topics_are_recorded():
    for name in ('/dog/state', '/dog/imu/data', '/dog/power', '/dog/estop',
                 '/dog/joint_commands', '/dog/servo_pulses', '/dog/profile'):
        assert name in record.TOPICS, name


def test_the_topics_are_named_not_positional():
    """rosbag2 deprecates the positional list and warns about it on every run, so the
    launch names the flag. The day the positional form disappears this still works."""
    text = open(record.__file__, encoding='utf-8').read()
    assert "'--topics'" in text
    assert "'--compression-mode'" not in text      # unreadable bags, see DEPLOYMENT stage 9


def test_each_run_gets_its_own_directory():
    first = record.default_output(0)
    second = record.default_output(1)
    assert first != second
    assert first.startswith(os.path.expanduser('~'))
    assert 'dog-bags' in first


def test_the_launch_description_builds():
    assert record.generate_launch_description() is None   # stubbed LaunchDescription
