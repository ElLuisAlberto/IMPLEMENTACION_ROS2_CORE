"""Tests for geometry evidence emitted by each offline identity."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from thesis_core.geometry_profile import (
    NOMINAL,
    GeometryProfile,
    SpherePrimitive,
    StateEnvelope,
    legacy_jaco_geometry_profile,
    offline_fixture_jaco_geometry_profile,
)
from thesis_simulation.geometry_snapshot_source_contract import (
    complete_joint_sample,
    geometry_transport_manifest,
    validate_offline_identity,
)
from thesis_simulation.geometry_snapshot_source_node import (
    GeometrySnapshotSourceNode,
)


def test_legacy_manifest_keeps_all_six_arm_capsules_and_prefixed_frames():
    profile = legacy_jaco_geometry_profile(
        robot_id='arm_03', frame_prefix='arm_03_jaco'
    )

    result = geometry_transport_manifest(profile, NOMINAL)

    assert len(result.primitive_names) == 6
    assert set(result.primitive_types) == {'CAPSULE'}
    assert result.start_frames[0] == 'arm_03_jaco_link_base'
    assert result.end_frames[-1] == 'arm_03_jaco_end_effector'


def test_sphere_manifest_uses_empty_unused_end_frame_and_zero_offset():
    profile = GeometryProfile(
        robot_id='arm_02',
        profile_id='offline_fixture',
        geometry_version='1.0.0',
        asset_hash='a' * 64,
        mount_frame='fixture_mount',
        tcp_frame='fixture_tcp',
        envelopes=(StateEnvelope(
            NOMINAL,
            (SpherePrimitive('fixture', 'fixture_frame', 0.1),),
        ),),
    )

    result = geometry_transport_manifest(profile, NOMINAL)

    assert result.primitive_types == ('SPHERE',)
    assert result.end_frames == ('',)
    assert result.end_offsets == ((0.0, 0.0, 0.0),)


def test_offline_fixture_profile_composes_arm_and_test_tool_volume():
    profile = offline_fixture_jaco_geometry_profile(
        robot_id='arm_02', frame_prefix='arm_02_jaco'
    )

    result = geometry_transport_manifest(profile, NOMINAL)

    assert len(result.primitive_names) == 7
    assert result.primitive_types.count('CAPSULE') == 6
    assert result.primitive_types.count('SPHERE') == 1
    assert result.start_frames[-1] == 'arm_02_jaco_fixture_body'
    assert result.start_offsets[-1] == (0.0, 0.0, 0.06)
    assert profile.profile_id == 'jaco2_offline_fixture'
    assert profile.mount_frame == 'arm_02_jaco_tool_mount'
    assert profile.tcp_frame == 'arm_02_jaco_gripper_tcp'


def test_joint_sample_requires_six_unique_names_and_finite_positions():
    required = tuple(f'arm_04_jaco_joint_{index}' for index in range(1, 7))
    assert complete_joint_sample(required, (0.0,) * 6, required)
    assert not complete_joint_sample(required[:-1], (0.0,) * 5, required)
    assert not complete_joint_sample(required, (0.0,) * 5, required)
    assert not complete_joint_sample(
        required, (0.0, 0.0, math.nan, 0.0, 0.0, 0.0), required
    )


def test_only_the_fixed_offline_role_map_is_accepted():
    validate_offline_identity('arm_01', 'synthetic_state')
    validate_offline_identity('arm_02', 'gazebo')
    with pytest.raises(ValueError, match='source_kind'):
        validate_offline_identity('arm_01', 'gazebo')
    with pytest.raises(ValueError, match='source_kind'):
        validate_offline_identity('arm_04', 'physical')


def test_source_never_publishes_equal_or_older_joint_stamp():
    required = tuple(f'arm_02_jaco_joint_{index}' for index in range(1, 7))
    frames = frozenset(
        f'arm_02_jaco_link_{index}' for index in range(1, 7)
    )
    publish = Mock()
    node = SimpleNamespace(
        required_joint_names=required,
        expected_dynamic_frames=frames,
        _last_published_stamp=2_000_000_000,
        _joint_stamps=set(),
        _tf_frames_by_stamp={3_000_000_000: set(frames)},
        _prune_stamp_caches=lambda newest: None,
        _publish_snapshot=publish,
        get_logger=Mock,
    )

    def joint_state(sec):
        return SimpleNamespace(
            name=required,
            position=(0.0,) * 6,
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=sec, nanosec=0)
            ),
        )

    GeometrySnapshotSourceNode._joint_state_callback(
        node, joint_state(1)
    )
    GeometrySnapshotSourceNode._joint_state_callback(
        node, joint_state(2)
    )
    GeometrySnapshotSourceNode._joint_state_callback(
        node, joint_state(3)
    )

    publish.assert_called_once()
    assert publish.call_args.args[0].sec == 3
    assert node._last_published_stamp == 3_000_000_000
    assert 1_000_000_000 not in node._joint_stamps
    assert 2_000_000_000 not in node._joint_stamps
