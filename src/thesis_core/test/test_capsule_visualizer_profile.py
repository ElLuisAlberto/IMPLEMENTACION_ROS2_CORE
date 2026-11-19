"""Pure tests for profile-driven capsule marker configuration."""

import pytest
from types import SimpleNamespace

from thesis_core.capsule_visualizer import (
    validate_capsule_configuration,
    CapsuleVisualizer,
)


def test_arbitrary_profile_capsules_are_accepted():
    """Accept marker configuration without the six JACO2 radii."""
    validate_capsule_configuration(
        ('base', 'shoulder', 'tool'),
        ('base_link', 'link_1', 'link_5'),
        ('link_1', 'link_5', 'tool0'),
        (0.12, 0.09, 0.04),
        (0.1, 0.8, 0.3, 0.4),
    )


@pytest.mark.parametrize(
    'radii',
    ((0.1, -0.1), (0.1, float('nan'))),
)
def test_invalid_profile_capsule_radius_is_rejected(radii):
    with pytest.raises(ValueError, match='positive and finite'):
        validate_capsule_configuration(
            ('first', 'second'),
            ('a', 'b'),
            ('b', 'c'),
            radii,
            (0.1, 0.8, 0.3, 0.4),
        )


def test_evidence_requires_fresh_source_and_receipt():
    visualizer = SimpleNamespace(max_evidence_age=0.5)
    msg = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0))
    fresh = CapsuleVisualizer._fresh
    assert fresh(visualizer, msg, 20.0, 20.2, 10_200_000_000)
    assert not fresh(visualizer, msg, 19.0, 20.2, 10_200_000_000)
    assert not fresh(visualizer, msg, 20.0, 20.2, 10_600_000_000)
    assert not fresh(visualizer, msg, 20.0, 20.2, 9_900_000_000)
