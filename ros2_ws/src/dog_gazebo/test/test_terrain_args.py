"""The terrain friction input (plan 02-01, task 1).

The world text must not move while nobody asks for a different floor: the three
golden hashes below were taken from the shipped generator before `mu` became a
parameter, so any accidental change to the layout or the contact block fails here
instead of quietly making the phase-1 baseline incomparable.
"""

import hashlib

from dog_gazebo import terrain

GOLDEN = {
    ('flat', 0.0, 0): '3d3720e1ed49d01b3413e92bb95d4daa06b49fd61fd256828ee1c7f8f34fe358',
    ('slope', 10.0, 0): '5d2ebca0074d905f836223261529d0413b76065be53df2412dec9de2f12b2795',
    ('waves', 10.0, 0): 'b15031946141d87ed9aa85f7a076b6522488281ebf13a568b88245a17a569f40',
}
SHIPPED = '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>'


def test_shipped_surface_string_is_unchanged():
    assert terrain.friction_surface() == SHIPPED
    assert terrain.friction_surface(1.0) == SHIPPED


def test_shipped_worlds_are_byte_identical():
    for (kind, level, seed), sha in GOLDEN.items():
        text = terrain.world(kind, level, seed)
        assert hashlib.sha256(text.encode()).hexdigest() == sha, kind


def test_mu_reaches_every_shape():
    # ground + ramp: both must carry the slippery floor, not just the ground
    text = terrain.world('slope', 10.0, 0, mu=0.6)
    assert '<mu>0.6</mu><mu2>0.6</mu2>' in text
    assert '<mu>1.0<' not in text
    assert text.count('<mu>0.6</mu>') >= 2


def test_mu_reaches_obstacles():
    text = terrain.world('waves', 10.0, 0, mu=0.6)
    assert '<mu>1.0<' not in text
    assert text.count('<mu>0.6</mu>') > 2


def test_world_rejects_unknown_terrain():
    try:
        terrain.world('moon')
    except ValueError as exc:
        assert 'unknown terrain' in str(exc)
    else:
        raise AssertionError('unknown terrain accepted')
