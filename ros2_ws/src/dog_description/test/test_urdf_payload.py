"""The trunk payload (plan 02-01, task 2).

A load on the body changes the trunk mass and its inertia by the same recipe; a
zero load must leave the URDF exactly as it was, otherwise the ideal-model pin
and every phase-1 number stop meaning the same thing.
"""

import xml.etree.ElementTree as ET

import pytest

from dog_description.urdf import DEFAULT_DESCRIPTION, build_urdf
from dog_description.servo_profile import parse_override

GEOMETRY = {'hip_offset': 0.045, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}

TOTAL_MASS = 0.80 + 4 * (0.06 + 0.08 + 0.03)   # trunk + four legs = 1.48 kg


def test_default_payload_keeps_the_urdf_untouched():
    plain = build_urdf(GEOMETRY)
    zero = build_urdf(GEOMETRY, dict(DEFAULT_DESCRIPTION, payload_mass=0.0))
    assert plain == zero


def test_payload_adds_to_the_trunk_mass():
    up = build_urdf(GEOMETRY, {'payload_mass': 0.2 * TOTAL_MASS})
    assert '<mass value="1.0960"/>' in up
    assert '<mass value="0.8000"/>' not in up
    down = build_urdf(GEOMETRY, {'payload_mass': -0.2 * TOTAL_MASS})
    assert '<mass value="0.5040"/>' in down


def test_payload_moves_the_inertia():
    plain = ET.fromstring(build_urdf(GEOMETRY))
    heavy = ET.fromstring(build_urdf(GEOMETRY, {'payload_mass': 0.296}))

    def trunk_ixx(root):
        trunk = root.find(".//link[@name='trunk']")
        return float(trunk.find('inertial/inertia').get('ixx'))

    assert trunk_ixx(heavy) > trunk_ixx(plain)


def test_absurd_negative_payload_is_refused():
    with pytest.raises(ValueError, match='too negative'):
        build_urdf(GEOMETRY, {'payload_mass': -0.6})


def test_payload_must_be_finite():
    with pytest.raises(ValueError, match='finite'):
        build_urdf(GEOMETRY, {'payload_mass': float('nan')})


def test_parse_override_takes_signed_values_only_on_request():
    assert parse_override('payload_mass', '-0.296', signed=True) == -0.296
    with pytest.raises(ValueError, match='negative'):
        parse_override('payload_mass', '-0.296')
    assert parse_override('payload_mass', '') is None
