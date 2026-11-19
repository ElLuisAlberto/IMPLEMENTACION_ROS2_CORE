"""Pure tests for profile-driven current proximity evaluation."""

import pytest
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from thesis_core.profile_proximity_monitor import (
    ProfileProximityMonitor,
    classify_clearance,
    profile_clearance,
)
from thesis_core.robot_profile import CapsuleSegment


class FiveAxisProfile:
    """Minimal non-JACO profile used to exercise pure geometry."""

    joint_names = tuple(f'axis_{index}' for index in range(5))

    @staticmethod
    def capsule_segments(positions):
        """Return two simple capsule axes from a five-value vector."""
        assert len(positions) == 5
        return (
            CapsuleSegment(
                'base', (0.0, 0.0, 0.0), (0.0, 0.0, 0.5), 0.1
            ),
            CapsuleSegment(
                'arm', (0.0, 0.0, 0.5), (positions[0], 0.0, 0.8), 0.05
            ),
        )


def test_five_axis_profile_clearance_uses_every_capsule():
    """The current monitor evaluates arbitrary DOF and capsule counts."""
    result = profile_clearance(
        FiveAxisProfile(),
        (0.4, 0.0, 0.0, 0.0, 0.0),
        (0.4, 0.2, 0.8),
        0.05,
    )

    assert result.segment_name == 'arm'
    assert result.clearance == pytest.approx(0.10)


@pytest.mark.parametrize(
    'clearance,expected',
    [
        (0.31, 'ALLOW'),
        (0.30, 'WARNING'),
        (0.15, 'WARNING'),
        (0.051, 'WARNING'),
        (0.05, 'STOP'),
    ],
)
def test_nearby_clearance_is_not_predictive_reduction(clearance, expected):
    """Proximity state reports warning/stop, not a 15 cm speed reduction."""
    assert classify_clearance(clearance, 0.30, 0.05) == expected


def test_invalid_warning_and_stop_order_is_rejected():
    """An invalid warning/stop ordering cannot start the monitor."""
    with pytest.raises(ValueError, match='thresholds'):
        classify_clearance(1.0, 0.05, 0.10)


@pytest.mark.parametrize(
    'state,expected',
    [
        ('ALLOW', (0.05, 0.80, 0.35, 0.38)),
        ('WARNING', (1.00, 0.82, 0.05, 0.70)),
        ('REDUCTION', (1.00, 0.35, 0.02, 0.78)),
        ('STOP', (1.00, 0.03, 0.03, 0.88)),
    ],
)
def test_profile_capsule_colors_match_historical_visualizer(state, expected):
    """Profile overlays retain the proven JACO2 visual state palette."""
    assert ProfileProximityMonitor._color(state) == expected


def test_profile_markers_cover_full_capsules_and_highlight_nearest():
    """Both hemisphere endpoints are visible without changing the geometry."""
    class Publisher:
        def publish(self, value):
            self.value = value

    capsules = (
        CapsuleSegment('base', (0.0, 0.0, 0.0), (0.0, 0.0, 0.5), 0.1),
        CapsuleSegment('tool', (0.5, 0.0, 0.8), (0.5, 0.0, 0.8), 0.05),
    )
    capsule_publisher = Publisher()
    obstacle_publisher = Publisher()
    stub = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        _color=ProfileProximityMonitor._color,
        profile=SimpleNamespace(world_frame='world', capsule_segments=lambda _: capsules),
        positions=(0.0,), capsule_publisher=capsule_publisher,
        obstacle_publisher=obstacle_publisher,
        obstacle_marker_publisher=obstacle_publisher,
        obstacle=SimpleNamespace(center=Point()),
    )
    ProfileProximityMonitor._publish_markers(stub, 'STOP', 0.12, 'tool')
    markers = capsule_publisher.value.markers
    assert len(markers) == 6
    assert [marker.id for marker in markers] == list(range(6))
    assert [marker.type for marker in markers] == [
        Marker.CYLINDER, Marker.SPHERE, Marker.SPHERE,
        Marker.CYLINDER, Marker.SPHERE, Marker.SPHERE,
    ]
    assert markers[1].pose.position.z == pytest.approx(0.0)
    assert markers[2].pose.position.z == pytest.approx(0.5)
    assert markers[4].scale.x == pytest.approx(0.1)
    assert markers[0].color.g == pytest.approx(0.80)
    assert markers[3].color.r == pytest.approx(1.0)
