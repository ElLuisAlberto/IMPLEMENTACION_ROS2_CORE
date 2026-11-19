import math

from thesis_core.jaco_kinematics import (
    capsule_segments,
    end_effector_transform,
    identity_matrix,
    minimum_sphere_clearance,
    multiply,
    transform_matrix,
    transform_point,
    transform_vector,
    translation,
)


INITIAL_JOINTS = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)


def test_capsule_segment_count():
    assert len(capsule_segments(INITIAL_JOINTS)) == 6


def test_initial_pose_clearance_matches_tf_measurement():
    result = minimum_sphere_clearance(
        INITIAL_JOINTS,
        (0.60, 0.0, 0.65),
        0.12,
    )

    clearance, name, _, _, _ = result
    assert name == 'upper_arm_to_forearm'
    assert math.isclose(clearance, 0.395002, abs_tol=1.0e-5)


def test_identity_world_from_base_preserves_legacy_fk_and_capsules():
    legacy_transform = end_effector_transform(INITIAL_JOINTS)
    legacy_segments = capsule_segments(INITIAL_JOINTS)

    assert end_effector_transform(
        INITIAL_JOINTS, identity_matrix()
    ) == legacy_transform
    assert capsule_segments(INITIAL_JOINTS, identity_matrix()) == legacy_segments


def test_world_from_base_rotates_and_translates_fk_and_every_capsule():
    world_from_base = transform_matrix(
        (2.0, -3.0, 0.5), (0.0, 0.0, math.pi / 2.0)
    )
    local_transform = end_effector_transform(INITIAL_JOINTS)
    world_transform = end_effector_transform(INITIAL_JOINTS, world_from_base)

    assert world_transform == multiply(world_from_base, local_transform)
    assert translation(world_transform) == transform_point(
        world_from_base, translation(local_transform)
    )

    local_segments = capsule_segments(INITIAL_JOINTS)
    world_segments = capsule_segments(INITIAL_JOINTS, world_from_base)
    assert len(world_segments) == len(local_segments)
    for local_segment, world_segment in zip(local_segments, world_segments):
        assert world_segment == tuple(
            transform_point(world_from_base, point) for point in local_segment
        )


def test_transform_vector_rotates_without_translation():
    world_from_base = transform_matrix(
        (12.0, -5.0, 3.0), (0.0, 0.0, math.pi / 2.0)
    )

    assert transform_point(world_from_base, (1.0, 0.0, 0.0)) == (
        12.0, -4.0, 3.0
    )
    rotated = transform_vector(world_from_base, (1.0, 0.0, 0.0))
    assert math.isclose(rotated[0], 0.0, abs_tol=1.0e-12)
    assert math.isclose(rotated[1], 1.0, abs_tol=1.0e-12)
    assert math.isclose(rotated[2], 0.0, abs_tol=1.0e-12)


def test_invalid_world_from_base_fails_closed():
    reflected = identity_matrix()
    reflected[0][0] = -1.0
    malformed = identity_matrix()[:3]

    for invalid in (reflected, malformed):
        try:
            capsule_segments(INITIAL_JOINTS, invalid)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid world_from_base must be rejected')
