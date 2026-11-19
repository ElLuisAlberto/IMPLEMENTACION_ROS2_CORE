"""ROS-boundary contract tests for the four-arm coordinator."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from rclpy.qos import HistoryPolicy, ReliabilityPolicy

from thesis_core.geometry_profile import (
    NOMINAL,
    GeometrySnapshot,
    legacy_jaco_geometry_profile,
)
from thesis_core.multiarm_coordinator_node import MultiArmCoordinatorNode
from thesis_core.multiarm_ros_contract import (
    geometry_snapshot_qos,
    geometry_decision_is_valid,
    required_snapshot_frames,
    stamp_is_strictly_newer,
    time_message_key,
    transform_message_matrix,
)


def test_legacy_snapshot_requires_all_seven_chain_frames_once():
    profile = legacy_jaco_geometry_profile(frame_prefix='arm_03_jaco')
    snapshot = GeometrySnapshot(profile, NOMINAL, 10.0, 0.2)

    frames = required_snapshot_frames(snapshot)

    assert frames[0] == 'arm_03_jaco_end_effector'
    assert 'arm_03_jaco_link_base' in frames
    assert len(frames) == 7
    assert len(set(frames)) == len(frames)


def test_transform_message_is_normalized_and_includes_translation():
    message = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=2.0),
    ))

    matrix = transform_message_matrix(message)

    assert tuple(row[3] for row in matrix[:3]) == (1.0, 2.0, 3.0)
    assert matrix[0][:3] == pytest.approx((1.0, 0.0, 0.0))


def test_incomplete_or_zero_quaternion_transform_is_rejected():
    with pytest.raises(ValueError, match='incomplete'):
        transform_message_matrix(SimpleNamespace())
    zero = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
    ))
    with pytest.raises(ValueError, match='zero quaternion'):
        transform_message_matrix(zero)


def test_only_completed_geometry_evaluations_are_valid():
    assert geometry_decision_is_valid('CLEAR')
    assert geometry_decision_is_valid('INTERROBOT_CLEARANCE_STOP')
    assert not geometry_decision_is_valid('TF_UNAVAILABLE')
    assert not geometry_decision_is_valid('SNAPSHOT_STALE')


def test_geometry_transport_keeps_only_latest_reliable_descriptor():
    qos = geometry_snapshot_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE


def test_geometry_stamp_order_is_strict_and_normalized():
    stamp = SimpleNamespace(sec=12, nanosec=345)

    key = time_message_key(stamp)

    assert key == 12_000_000_345
    assert stamp_is_strictly_newer(key, None)
    assert stamp_is_strictly_newer(key, key - 1)
    assert not stamp_is_strictly_newer(key, key)
    assert not stamp_is_strictly_newer(key, key + 1)
    assert not stamp_is_strictly_newer(0, None)
    with pytest.raises(ValueError, match='normalized'):
        time_message_key(SimpleNamespace(sec=1, nanosec=1_000_000_000))


def test_coordinator_never_replaces_a_record_with_equal_or_older_stamp():
    logger = Mock()
    node = SimpleNamespace(
        _records={},
        _last_snapshot_stamps={},
        get_logger=lambda: logger,
    )

    def message(sec):
        return SimpleNamespace(
            robot_id='arm_02',
            stamp=SimpleNamespace(sec=sec, nanosec=0),
        )

    initial = message(10)
    newest = message(11)
    equal = message(11)
    older = message(9)
    with patch(
        'thesis_core.multiarm_coordinator_node.time.monotonic',
        side_effect=(20.0, 21.0),
    ):
        MultiArmCoordinatorNode._snapshot_callback(node, 'arm_02', initial)
        MultiArmCoordinatorNode._snapshot_callback(node, 'arm_02', newest)
        MultiArmCoordinatorNode._snapshot_callback(node, 'arm_02', equal)
        MultiArmCoordinatorNode._snapshot_callback(node, 'arm_02', older)

    assert node._records['arm_02'] == (newest, 21.0)
    assert node._last_snapshot_stamps['arm_02'] == 11_000_000_000
