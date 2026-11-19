"""Tests for robot-independent Cartesian adapter policy."""

from types import SimpleNamespace

import pytest

from thesis_simulation.profile_cartesian import position_only_is_permitted


@pytest.mark.parametrize('joint_count', (5, 4))
def test_underactuated_profile_allows_translation_only(joint_count):
    """Permit position-only IK when an underactuated tool does not rotate."""
    profile = SimpleNamespace(joint_count=joint_count)
    assert position_only_is_permitted(profile, 0.0)
    assert position_only_is_permitted(profile, 1.0e-6)


def test_six_axis_profile_never_drops_orientation():
    """Keep the full pose contract for a six-axis profile."""
    profile = SimpleNamespace(joint_count=6)
    assert not position_only_is_permitted(profile, 0.0)


def test_underactuated_profile_rejects_explicit_rotation():
    """Do not silently discard an explicit orientation request."""
    profile = SimpleNamespace(joint_count=5)
    assert not position_only_is_permitted(profile, 1.0e-5)


@pytest.mark.parametrize('angle', (-1.0, float('nan')))
def test_invalid_orientation_motion_is_rejected(angle):
    """Reject malformed rotation magnitudes before choosing an IK mode."""
    with pytest.raises(ValueError):
        position_only_is_permitted(SimpleNamespace(joint_count=5), angle)
