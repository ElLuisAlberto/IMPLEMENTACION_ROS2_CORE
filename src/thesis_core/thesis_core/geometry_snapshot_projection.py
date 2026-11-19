"""
Resolve a versioned local geometry snapshot into shared-world occupancy.

The module is deliberately independent of ROS TF.  A ROS adapter can provide
``world_from_frame`` matrices from exact-stamp transforms, while tests and
offline validation use the same deterministic conversion before handing the
result to the four-arm coordinator.
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence, Tuple

from thesis_core.geometry_profile import (
    CapsulePrimitive,
    GeometrySnapshot,
    SpherePrimitive,
)
from thesis_core.multiarm_coordination import (
    CapsuleOccupancy,
    RobotOccupancySnapshot,
    SphereOccupancy,
)


Matrix4 = Tuple[Tuple[float, float, float, float], ...]
_EPSILON = 1.0e-9


def _matrix4(values: Sequence[Sequence[float]], frame: str) -> Matrix4:
    """Validate a finite, right-handed homogeneous world-from-frame matrix."""
    try:
        matrix = tuple(tuple(float(value) for value in row) for row in values)
    except (TypeError, ValueError) as exception:
        raise ValueError(f'{frame} transform must be a finite 4x4 matrix') from exception
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError(f'{frame} transform must be a 4x4 matrix')
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise ValueError(f'{frame} transform must be finite')
    if any(abs(matrix[3][index]) > _EPSILON for index in range(3)) or not math.isclose(
            matrix[3][3], 1.0, abs_tol=_EPSILON):
        raise ValueError(f'{frame} transform must be homogeneous')
    rotation = tuple(row[:3] for row in matrix[:3])
    for row in rotation:
        if not math.isclose(sum(value * value for value in row), 1.0, abs_tol=_EPSILON):
            raise ValueError(f'{frame} transform rotation must be normalized')
    for first in range(3):
        for second in range(first + 1, 3):
            if not math.isclose(
                    sum(rotation[first][index] * rotation[second][index]
                        for index in range(3)),
                    0.0,
                    abs_tol=_EPSILON):
                raise ValueError(f'{frame} transform rotation must be orthogonal')
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2]
                          - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2]
                            - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1]
                            - rotation[1][1] * rotation[2][0])
    )
    if not math.isclose(determinant, 1.0, abs_tol=_EPSILON):
        raise ValueError(f'{frame} transform rotation must be right-handed')
    return matrix


def transform_point(matrix: Sequence[Sequence[float]], point: Sequence[float]):
    """Apply a validated homogeneous transform to one finite local point."""
    resolved = _matrix4(matrix, 'frame')
    try:
        x_value, y_value, z_value = (float(value) for value in point)
    except (TypeError, ValueError) as exception:
        raise ValueError('point must contain three finite values') from exception
    if not all(math.isfinite(value) for value in (x_value, y_value, z_value)):
        raise ValueError('point must contain three finite values')
    return tuple(
        resolved[row][0] * x_value
        + resolved[row][1] * y_value
        + resolved[row][2] * z_value
        + resolved[row][3]
        for row in range(3)
    )


def _frame_matrix(
    transforms: Mapping[str, Sequence[Sequence[float]]], frame: str
) -> Matrix4:
    """Return one declared frame transform or reject incomplete geometry."""
    try:
        values = transforms[frame]
    except (KeyError, TypeError) as exception:
        raise ValueError(f'missing world transform for frame {frame!r}') from exception
    return _matrix4(values, frame)


def project_snapshot_to_world(
    snapshot: GeometrySnapshot,
    world_from_frame: Mapping[str, Sequence[Sequence[float]]],
    reference_frame: str = 'world',
) -> RobotOccupancySnapshot:
    """
    Resolve every primitive in a fresh profile snapshot into ``world``.

    Transform lookup is intentionally all-or-nothing: missing or malformed TF
    cannot produce a partial robot occupancy that would under-estimate space.
    Freshness is checked by the coordinator against its authoritative clock.
    """
    if not isinstance(snapshot, GeometrySnapshot):
        raise ValueError('snapshot must be a GeometrySnapshot')
    if not isinstance(reference_frame, str) or not reference_frame.strip():
        raise ValueError('reference_frame must be a non-empty string')
    capsules = []
    spheres = []
    for primitive in snapshot.primitives:
        if isinstance(primitive, CapsulePrimitive):
            capsules.append(CapsuleOccupancy(
                primitive.name,
                transform_point(
                    _frame_matrix(world_from_frame, primitive.start_frame),
                    primitive.start_offset,
                ),
                transform_point(
                    _frame_matrix(world_from_frame, primitive.end_frame),
                    primitive.end_offset,
                ),
                primitive.radius,
            ))
        elif isinstance(primitive, SpherePrimitive):
            spheres.append(SphereOccupancy(
                primitive.name,
                transform_point(
                    _frame_matrix(world_from_frame, primitive.frame),
                    primitive.center,
                ),
                primitive.radius,
            ))
        else:
            raise ValueError('snapshot contains an unsupported primitive')
    return RobotOccupancySnapshot(
        robot_id=snapshot.profile.robot_id,
        profile_id=snapshot.profile.profile_id,
        geometry_version=snapshot.profile.geometry_version,
        asset_hash=snapshot.profile.asset_hash,
        reference_frame=reference_frame,
        stamp_sec=snapshot.stamp_sec,
        ttl_sec=snapshot.ttl_sec,
        capsules=tuple(capsules),
        spheres=tuple(spheres),
    )
