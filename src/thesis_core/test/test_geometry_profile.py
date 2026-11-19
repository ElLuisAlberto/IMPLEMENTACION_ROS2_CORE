"""Tests for immutable, versioned safety geometry profiles."""

import json
import math

import pytest

from thesis_core.geometry_profile import (
    CLOSED,
    NOMINAL,
    OPEN,
    UNKNOWN,
    WITH_PAYLOAD,
    CapsulePrimitive,
    GeometryProfile,
    GeometrySnapshot,
    SpherePrimitive,
    StateEnvelope,
    legacy_jaco_geometry_profile,
    synthetic_gripper_jaco_geometry_profile,
)
from thesis_core.jaco_kinematics import CAPSULE_RADII, SEGMENT_NAMES


ASSET_HASH = 'a' * 64


def profile(envelopes=None):
    if envelopes is None:
        primitive = SpherePrimitive('body', 'gripper_base', 0.02)
        envelopes = (
            StateEnvelope(CLOSED, (primitive,)),
            StateEnvelope(OPEN, (primitive,)),
        )
    return GeometryProfile(
        robot_id='arm_01',
        profile_id='gripper_v1',
        geometry_version='1.0.0',
        asset_hash=ASSET_HASH,
        mount_frame='tool_mount',
        tcp_frame='gripper_tcp',
        envelopes=envelopes,
    )


def test_primitives_normalize_finite_coordinates_and_require_frames():
    capsule = CapsulePrimitive(
        'finger', 'gripper_base', 'gripper_tip', 0.01,
        start_offset=[0, 0, 0], end_offset=[0.1, 0, 0],
    )
    sphere = SpherePrimitive('housing', 'gripper_base', 0.02, [0, 0, 0])

    assert capsule.end_offset == (0.1, 0.0, 0.0)
    assert sphere.center == (0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        CapsulePrimitive('bad', '', 'tip', 0.01)
    with pytest.raises(ValueError):
        SpherePrimitive('bad', 'base', math.inf)


def test_state_envelope_rejects_duplicate_primitive_names():
    primitive = SpherePrimitive('duplicated', 'gripper_base', 0.02)
    with pytest.raises(ValueError, match='unique'):
        StateEnvelope(OPEN, (primitive, primitive))


@pytest.mark.parametrize('invalid_hash', ('abc', 'A' * 64, 'g' * 64))
def test_profile_requires_sha256_asset_hash(invalid_hash):
    with pytest.raises(ValueError, match='SHA-256'):
        GeometryProfile(
            'arm_01', 'profile', '1', invalid_hash,
            'mount', 'tcp', (StateEnvelope(OPEN, (
                SpherePrimitive('body', 'mount', 0.01),
            )),),
        )


def test_unknown_state_uses_open_envelope_conservatively():
    opened = SpherePrimitive('open_tip', 'gripper_base', 0.04)
    result = profile((
        StateEnvelope(CLOSED, (SpherePrimitive('closed_tip', 'gripper_base', 0.01),)),
        StateEnvelope(OPEN, (opened,)),
    )).envelope_for(UNKNOWN)

    assert result.state == OPEN
    assert result.primitives == (opened,)


def test_unknown_state_without_open_envelope_fails_closed():
    incomplete = profile((
        StateEnvelope(CLOSED, (SpherePrimitive('body', 'gripper_base', 0.02),)),
    ))

    with pytest.raises(ValueError, match='requires a configured open'):
        incomplete.envelope_for(UNKNOWN)


def test_unknown_state_with_a_possible_payload_fails_closed():
    loaded = profile((
        StateEnvelope(CLOSED, (SpherePrimitive('body', 'gripper_base', 0.02),)),
        StateEnvelope(OPEN, (SpherePrimitive('open', 'gripper_base', 0.04),)),
        StateEnvelope(
            WITH_PAYLOAD,
            (SpherePrimitive('payload', 'gripper_base', 0.08),),
        ),
    ))

    with pytest.raises(ValueError, match='payload'):
        loaded.envelope_for(UNKNOWN)


def test_snapshot_is_immutable_serializable_and_time_bounded():
    snapshot = GeometrySnapshot(profile(), UNKNOWN, 10.0, 0.25)
    encoded = json.loads(snapshot.to_json())

    assert snapshot.effective_state == OPEN
    assert snapshot.is_fresh(10.0)
    assert snapshot.is_fresh(10.25)
    assert not snapshot.is_fresh(10.251)
    assert not snapshot.is_fresh(math.nan)
    assert encoded['effective_state'] == OPEN
    with pytest.raises(AttributeError):
        snapshot.state = NOMINAL


def test_legacy_factory_preserves_current_six_capsules_and_empty_states():
    result = legacy_jaco_geometry_profile(frame_prefix='arm_03_jaco')
    envelope = result.envelope_for(UNKNOWN)

    assert result.mount_frame == 'arm_03_jaco_link_6'
    assert result.tcp_frame == 'arm_03_jaco_end_effector'
    assert tuple(item.name for item in envelope.primitives) == SEGMENT_NAMES
    assert tuple(item.radius for item in envelope.primitives) == CAPSULE_RADII
    assert all(item.start_frame.startswith('arm_03_jaco_')
               for item in envelope.primitives)
    with pytest.raises(ValueError, match='no envelope'):
        result.envelope_for(WITH_PAYLOAD)


@pytest.mark.parametrize('prefix', ('arm_02_jaco', 'arm_03_jaco', 'arm_04_jaco'))
def test_synthetic_gripper_swept_envelope_covers_all_box_corners(prefix):
    result = synthetic_gripper_jaco_geometry_profile('robot', prefix)
    sphere = result.envelope_for(OPEN).primitives[-1]
    assert sphere.frame == f'{prefix}_gripper_base'
    assert result.mount_frame == f'{prefix}_tool_mount'
    assert result.tcp_frame == f'{prefix}_gripper_tcp'
    for x in (-0.055, 0.055):
        for y in (-0.04, 0.04):
            for z in (0.0, 0.13):
                assert math.dist((x, y, z), sphere.center) < sphere.radius
    assert result.envelope_for(UNKNOWN).primitives == result.envelope_for(CLOSED).primitives
    assert len(result.envelope_for(NOMINAL).primitives) == len(SEGMENT_NAMES) + 1
    with pytest.raises(ValueError, match='no envelope'):
        result.envelope_for(WITH_PAYLOAD)
