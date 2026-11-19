"""Missing evidence must stop motion without inventing a measured distance."""

from types import SimpleNamespace
from unittest.mock import patch

import pytest
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Point, TransformStamped
from tf2_msgs.msg import TFMessage

from thesis_core.proximity_monitor_node import ProximityMonitorNode


def _stamp(seconds=10, nanoseconds=0):
    return TimeMessage(sec=seconds, nanosec=nanoseconds)


def _source_node():
    now = SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: _stamp(),
    )
    return SimpleNamespace(
        obstacle_source_mode='gazebo',
        obstacle_pose_timeout_sec=0.5,
        obstacle_max_age_sec=0.5,
        obstacle_future_tolerance_sec=0.05,
        dynamic_obstacle_center=Point(x=2.0, y=0.0, z=0.65),
        dynamic_obstacle_stamp=_stamp(9, 900_000_000),
        dynamic_obstacle_velocity=(0.0, 0.0, 0.0),
        last_obstacle_pose_time=100.0,
        previous_state=None,
        get_parameter=lambda name: SimpleNamespace(value={
            'obstacle_radius': 0.12, 'status_publishing_enabled': True,
        }[name]),
        get_clock=lambda: SimpleNamespace(now=lambda: now),
    )


@pytest.mark.parametrize('missing_field', (
    'dynamic_obstacle_center', 'dynamic_obstacle_stamp',
    'last_obstacle_pose_time',
))
def test_missing_gazebo_evidence_never_becomes_fixed_spawn_pose(missing_field):
    node = _source_node()
    setattr(node, missing_field, None)
    with patch('thesis_core.proximity_monitor_node.time.monotonic', return_value=100.1):
        assert ProximityMonitorNode._obstacle(node) is None


def test_expired_gazebo_receipt_does_not_get_a_new_timestamp():
    node = _source_node()
    with patch('thesis_core.proximity_monitor_node.time.monotonic', return_value=100.6):
        assert ProximityMonitorNode._obstacle(node) is None


@pytest.mark.parametrize('stamp', (
    _stamp(0), _stamp(9), _stamp(10, 60_000_000), _stamp(9, 1_000_000_000),
))
def test_recent_receipt_does_not_hide_bad_source_timestamp(stamp):
    node = _source_node()
    node.dynamic_obstacle_stamp = stamp
    with patch('thesis_core.proximity_monitor_node.time.monotonic', return_value=100.1):
        assert ProximityMonitorNode._obstacle(node) is None


def test_valid_moved_gazebo_obstacle_keeps_its_measured_pose_and_stamp():
    node = _source_node()
    with patch('thesis_core.proximity_monitor_node.time.monotonic', return_value=100.1):
        point, radius, velocity, stamp = ProximityMonitorNode._obstacle(node)
    assert point.x == 2.0
    assert radius == 0.12
    assert velocity == (0.0, 0.0, 0.0)
    assert stamp is node.dynamic_obstacle_stamp


def test_missing_obstacle_publishes_explicit_invalid_stop():
    node = _source_node()
    messages = []
    node._obstacle = lambda: None
    node.reference_frame = 'world'
    node.publisher = SimpleNamespace(publish=messages.append)
    node.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    node._publish_tf_stop = lambda *args: ProximityMonitorNode._publish_tf_stop(node, *args)
    ProximityMonitorNode.evaluate(node)
    assert len(messages) == 1
    message = messages[0]
    assert message.state == 'STOP'
    assert message.limiting_segment == 'OBSTACLE_UNAVAILABLE'
    assert not message.geometry_valid
    assert message.geometry_profile_id == 'jaco2_legacy'
    assert len(message.geometry_asset_hash) == 64


@pytest.mark.parametrize(
    ('clearance', 'expected'),
    ((0.15, 'WARNING'), (0.051, 'WARNING'), (0.05, 'STOP')),
)
def test_current_clearance_has_no_fixed_reduction_band(clearance, expected):
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value={
            'stop_distance': 0.05,
            'warning_distance': 0.30,
        }[name]),
    )

    assert (
        ProximityMonitorNode._state_for_clearance(node, clearance)
        == expected
    )


def test_tf_failure_preserves_geometry_identity_but_not_validity():
    node = _source_node()
    messages = []
    node.reference_frame = 'world'
    node.publisher = SimpleNamespace(publish=messages.append)
    node.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    ProximityMonitorNode._publish_tf_stop(
        node, _stamp(), Point(x=2.0), 0.12, 'TF_UNAVAILABLE',
    )
    assert not messages[0].geometry_valid
    assert messages[0].geometry_profile_id == 'jaco2_legacy'
    assert messages[0].state == 'STOP'


def _pose(x, stamp):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.child_frame_id = 'safety_obstacle'
    transform.transform.translation.x = x
    return TFMessage(transforms=[transform])


def test_gazebo_velocity_uses_simulation_acquisition_time():
    node = _source_node()
    node.obstacle_frame = 'safety_obstacle'
    node.previous_obstacle_center = None
    node.previous_obstacle_time = None
    with patch('thesis_core.proximity_monitor_node.time.monotonic', side_effect=(100, 104)):
        ProximityMonitorNode.obstacle_pose_callback(node, _pose(1.0, _stamp(8)))
        ProximityMonitorNode.obstacle_pose_callback(node, _pose(2.0, _stamp(10)))
    assert node.dynamic_obstacle_velocity == (0.5, 0.0, 0.0)
    assert node.last_obstacle_pose_time == 104


def test_one_millisecond_source_period_still_measures_velocity():
    node = _source_node()
    node.obstacle_frame = 'safety_obstacle'
    node.previous_obstacle_center = None
    node.previous_obstacle_time = None
    with patch('thesis_core.proximity_monitor_node.time.monotonic', side_effect=(100, 101)):
        ProximityMonitorNode.obstacle_pose_callback(node, _pose(1.0, _stamp(10)))
        ProximityMonitorNode.obstacle_pose_callback(
            node, _pose(1.001, _stamp(10, 1_000_000)),
        )
    assert node.dynamic_obstacle_velocity[0] == pytest.approx(1.0)


@pytest.mark.parametrize('second_stamp', (_stamp(10), _stamp(9)))
def test_duplicate_or_backward_stamp_does_not_create_a_velocity(second_stamp):
    node = _source_node()
    node.obstacle_frame = 'safety_obstacle'
    node.previous_obstacle_center = None
    node.previous_obstacle_time = None
    with patch('thesis_core.proximity_monitor_node.time.monotonic', side_effect=(100, 101)):
        ProximityMonitorNode.obstacle_pose_callback(node, _pose(1.0, _stamp(10)))
        ProximityMonitorNode.obstacle_pose_callback(node, _pose(2.0, second_stamp))
    assert node.dynamic_obstacle_velocity == (0.0, 0.0, 0.0)


def test_invalid_stop_keeps_publishing_without_repeating_the_same_warning():
    node = _source_node()
    messages = []
    warnings = []
    node.reference_frame = 'world'
    node.previous_state = None
    node.publisher = SimpleNamespace(publish=messages.append)
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    for _ in range(2):
        ProximityMonitorNode._publish_tf_stop(
            node, _stamp(), Point(), 0.0, 'OBSTACLE_UNAVAILABLE',
        )
    assert len(messages) == 2
    assert len(warnings) == 1
    assert all(not message.geometry_valid for message in messages)
    ProximityMonitorNode._publish_tf_stop(
        node, _stamp(), Point(), 0.0, 'TF_UNAVAILABLE',
    )
    assert len(messages) == 3
    assert len(warnings) == 2
    node.previous_state = 'ALLOW'
    ProximityMonitorNode._publish_tf_stop(
        node, _stamp(), Point(), 0.0, 'TF_UNAVAILABLE',
    )
    assert len(messages) == 4
    assert len(warnings) == 3


def test_nonfinite_gazebo_pose_invalidates_previous_sample():
    node = _source_node()
    node.obstacle_frame = 'safety_obstacle'
    node.previous_obstacle_center = node.dynamic_obstacle_center
    node.previous_obstacle_time = 9.9
    ProximityMonitorNode.obstacle_pose_callback(node, _pose(float('nan'), _stamp()))
    assert node.dynamic_obstacle_center is None
    assert node.dynamic_obstacle_stamp is None
    assert ProximityMonitorNode._obstacle(node) is None
