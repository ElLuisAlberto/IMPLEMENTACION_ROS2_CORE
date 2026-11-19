"""Projection tests linking a profile snapshot to multi-arm occupancy."""

import math

import pytest

from thesis_core.geometry_profile import (
    CLOSED,
    OPEN,
    CapsulePrimitive,
    GeometryProfile,
    GeometrySnapshot,
    SpherePrimitive,
    StateEnvelope,
)
from thesis_core.geometry_snapshot_projection import (
    project_snapshot_to_world,
    transform_point,
)


ASSET_HASH = 'b' * 64
IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


def profile():
    """Return a small two-primitive gripper profile for projection."""
    primitives = (
        CapsulePrimitive(
            'finger', 'gripper_base', 'gripper_tip', 0.01,
            (0.0, 0.0, 0.0), (0.02, 0.0, 0.0),
        ),
        SpherePrimitive('housing', 'gripper_base', 0.03, (0.1, 0.0, 0.0)),
    )
    return GeometryProfile(
        'arm_01', 'gripper_v1', '1.0.0', ASSET_HASH,
        'tool_mount', 'gripper_tcp',
        (StateEnvelope(CLOSED, primitives), StateEnvelope(OPEN, primitives)),
    )


def test_projection_applies_frame_pose_and_preserves_provenance():
    snapshot = GeometrySnapshot(profile(), OPEN, 10.0, 0.25)
    transforms = {
        'gripper_base': (
            (0.0, -1.0, 0.0, 1.0),
            (1.0, 0.0, 0.0, 2.0),
            (0.0, 0.0, 1.0, 3.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        'gripper_tip': (
            (1.0, 0.0, 0.0, 1.5),
            (0.0, 1.0, 0.0, 2.0),
            (0.0, 0.0, 1.0, 3.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
    }

    result = project_snapshot_to_world(snapshot, transforms)

    assert result.reference_frame == 'world'
    assert result.expectation.profile_id == 'gripper_v1'
    assert result.capsules[0].start == pytest.approx((1.0, 2.0, 3.0))
    assert result.capsules[0].end == pytest.approx((1.52, 2.0, 3.0))
    assert result.spheres[0].center == pytest.approx((1.0, 2.1, 3.0))


def test_missing_frame_fails_closed_instead_of_creating_partial_occupancy():
    snapshot = GeometrySnapshot(profile(), OPEN, 10.0, 0.25)

    with pytest.raises(ValueError, match='missing world transform'):
        project_snapshot_to_world(snapshot, {'gripper_base': IDENTITY})


def test_reflected_or_nonfinite_transform_is_rejected():
    reflected = (
        (-1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    with pytest.raises(ValueError, match='right-handed'):
        transform_point(reflected, (0.0, 0.0, 0.0))
    malformed = (
        (1.0, 0.0, 0.0, math.nan),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    with pytest.raises(ValueError, match='finite'):
        transform_point(malformed, (0.0, 0.0, 0.0))
