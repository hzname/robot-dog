"""SAF-16 without the robot: does the guard drop the outputs when the driver vanishes?

The script talks to docker compose, so the test puts a fake `docker` on PATH and watches
what the guard does from the outside: it must notice the container disappearing and call
`pca9685_probe off`, it must not hammer the chip, and it must not pretend to have succeeded
when the probe failed. What stays on the bench is the wiring - an I2C bus, a chip and a
servo - not the logic.
"""

import os
import stat
import subprocess
import sys
import time

import pytest

GUARD = os.path.join(os.path.dirname(__file__), '..', 'servo-guard.sh')

FAKE_DOCKER = '''#!/usr/bin/env bash
# Records every call; answers the two questions the guard asks.
echo "docker $*" >> "$CALLS"
case "$*" in
  *"ps --status running --services"*) cat "$SERVICES" 2>/dev/null || true ;;
  *"pca9685_probe off"*)
      if [ -f "$PROBE_FAILS" ]; then exit 1; fi
      echo "probe-off" >> "$CALLS" ;;
esac
exit 0
'''


@pytest.fixture
def bench(tmp_path):
    """A fake bench: docker that lies, a directory that is the compose project."""
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    fake = bindir / 'docker'
    fake.write_text(FAKE_DOCKER)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    (tmp_path / 'docker-compose.yml').write_text('services: {}\n')
    calls = tmp_path / 'calls.log'
    calls.write_text('')
    env = dict(os.environ,
               PATH='%s:%s' % (bindir, os.environ['PATH']),
               SERVO_GUARD_COMPOSE_DIR=str(tmp_path),
               SERVO_GUARD_STATE=str(tmp_path / 'state'),
               SERVO_GUARD_COOLDOWN='0',
               CALLS=str(calls),
               SERVICES=str(tmp_path / 'services.txt'),
               PROBE_FAILS=str(tmp_path / 'probe-fails'))
    return type('Bench', (), {'dir': tmp_path, 'calls': calls, 'env': env,
                             'services': tmp_path / 'services.txt'})


def run(bench, *args, timeout=20):
    return subprocess.run(['bash', GUARD] + list(args), env=bench.env,
                          capture_output=True, text=True, timeout=timeout)


def wait_for(predicate, seconds=8.0):
    end = time.time() + seconds
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.2)
    return predicate()


def test_the_guard_drops_the_outputs_when_the_container_disappears(bench):
    bench.services.write_text('robot\n')            # the driver is up
    watcher = subprocess.Popen(['bash', GUARD, 'watch'], env=bench.env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        time.sleep(2.0)                              # let the loop see it running
        bench.services.write_text('')                # kill -9 / OOM: the container is gone
        assert wait_for(lambda: 'probe-off' in bench.calls.read_text()), \
            'сторож не позвал pca9685_probe off:\n' + bench.calls.read_text()
    finally:
        watcher.terminate()
        watcher.wait(timeout=10)


def test_a_container_that_was_never_seen_does_not_trigger_a_drop(bench):
    bench.services.write_text('')                    # nothing running from the start
    watcher = subprocess.Popen(['bash', GUARD, 'watch'], env=bench.env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        time.sleep(2.5)
        assert 'probe-off' not in bench.calls.read_text(), bench.calls.read_text()
    finally:
        watcher.terminate()
        watcher.wait(timeout=10)


def test_the_off_mode_drops_the_outputs_on_a_normal_stop(bench):
    done = run(bench, 'off')
    assert done.returncode == 0, done.stderr
    assert 'probe-off' in bench.calls.read_text()
    assert 'outputs dropped' in done.stderr


def test_the_cooldown_stops_a_second_drop(bench):
    bench.env['SERVO_GUARD_COOLDOWN'] = '100'
    bench.dir.joinpath('state').write_text('%d\n' % int(time.time()))
    done = run(bench, 'off')
    assert done.returncode == 0
    assert 'cooldown' in done.stderr
    assert 'probe-off' not in bench.calls.read_text()


def test_a_failed_probe_is_reported_not_swallowed(bench):
    bench.dir.joinpath('probe-fails').write_text('1')
    done = run(bench, 'off')
    assert done.returncode == 1
    assert 'failed to drop the outputs' in done.stderr


def test_an_unknown_mode_is_refused(bench):
    done = run(bench, 'restart')
    assert done.returncode == 2
    assert 'usage' in done.stderr
