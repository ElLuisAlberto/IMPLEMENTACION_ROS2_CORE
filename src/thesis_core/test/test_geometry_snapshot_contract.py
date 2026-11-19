"""Tests for complete, fail-closed geometry snapshot conversion."""

from types import SimpleNamespace

import pytest

from thesis_core.geometry_snapshot_contract import (
    geometry_snapshot_from_message,
)


HASH = 'a' * 64


def point(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def time_value(sec=10, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def message():
    return SimpleNamespace(
        stamp=time_value(),
        valid_for=time_value(0, 500_000_000),
        session_id='session-a',
        source_kind='gazebo',
        robot_id='arm_02',
        profile_id='fixture_v1',
        geometry_version='1.0.0-test',
        asset_hash=HASH,
        state='nominal',
        reference_frame='world',
        mount_frame='arm_02_tool_mount',
        tcp_frame='arm_02_gripper_tcp',
        valid=True,
        primitive_names=['arm', 'fixture'],
        primitive_types=['CAPSULE', 'SPHERE'],
        start_frames=['arm_02_base', 'arm_02_gripper_base'],
        end_frames=['arm_02_tip', ''],
        start_offsets=[point(), point(0.1, 0.0, 0.0)],
        end_offsets=[point(), point()],
        radii=[0.1, 0.05],
    )


def test_complete_message_builds_immutable_active_profile():
    result = geometry_snapshot_from_message(message())
    assert result.profile.robot_id == 'arm_02'
    assert result.session_id == 'session-a'
    assert result.source_kind == 'gazebo'
    assert len(result.primitives) == 2


@pytest.mark.parametrize('field', (
    'primitive_types', 'start_frames', 'end_frames', 'start_offsets',
    'end_offsets', 'radii',
))
def test_parallel_array_mismatch_rejects_complete_snapshot(field):
    value = message()
    setattr(value, field, getattr(value, field)[:-1])
    with pytest.raises(ValueError, match='length'):
        geometry_snapshot_from_message(value)


def test_invalid_unknown_or_physical_source_fails_closed():
    value = message()
    value.valid = False
    with pytest.raises(ValueError, match='invalid'):
        geometry_snapshot_from_message(value)
    value = message()
    value.state = 'desconocido'
    with pytest.raises(ValueError, match='configured'):
        geometry_snapshot_from_message(value)
    value = message()
    value.source_kind = 'physical'
    with pytest.raises(ValueError, match='source_kind'):
        geometry_snapshot_from_message(value)


def test_nonfinite_offset_and_duplicate_name_are_rejected():
    value = message()
    value.start_offsets[0].x = float('nan')
    with pytest.raises(ValueError, match='finite'):
        geometry_snapshot_from_message(value)
    value = message()
    value.primitive_names[1] = value.primitive_names[0]
    with pytest.raises(ValueError, match='unique'):
        geometry_snapshot_from_message(value)
