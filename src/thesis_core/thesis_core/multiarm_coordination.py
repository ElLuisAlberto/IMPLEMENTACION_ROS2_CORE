"""
Pure, fail-closed geometry coordination for four simulated manipulators.

This module deliberately has no ROS dependency.  It is the deterministic
kernel used by a future ROS coordinator after each robot has produced an
immutable occupancy snapshot in the shared ``world`` reference frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
from typing import Mapping, Optional, Sequence, Tuple

from thesis_core.clearance_geometry import (
    DEGENERATE_SEGMENT_EPSILON_SQ,
    Point3,
    finite_point,
    positive_radius,
)


EPSILON = 1.0e-12
FOUR_ROBOT_COUNT = 4


def _nonempty_text(value: str, name: str) -> str:
    """Return a non-empty identity field without surrounding whitespace."""
    result = str(value).strip()
    if not result:
        raise ValueError(f'{name} must not be empty')
    return result


def _finite_nonnegative(value: float, name: str) -> float:
    """Return a finite non-negative scalar."""
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f'{name} must be finite and non-negative')
    return result


@dataclass(frozen=True)
class CapsuleOccupancy:
    """One world-frame capsule that belongs to a robot occupancy snapshot."""

    name: str
    start: Point3
    end: Point3
    radius: float

    def __post_init__(self) -> None:
        object.__setattr__(self, 'name', _nonempty_text(self.name, 'name'))
        object.__setattr__(self, 'start', finite_point(self.start, 'start'))
        object.__setattr__(self, 'end', finite_point(self.end, 'end'))
        object.__setattr__(
            self, 'radius', positive_radius(self.radius, 'radius')
        )


@dataclass(frozen=True)
class SphereOccupancy:
    """One world-frame sphere, for a gripper feature or authorized payload."""

    name: str
    center: Point3
    radius: float

    def __post_init__(self) -> None:
        object.__setattr__(self, 'name', _nonempty_text(self.name, 'name'))
        object.__setattr__(
            self, 'center', finite_point(self.center, 'center')
        )
        object.__setattr__(
            self, 'radius', positive_radius(self.radius, 'radius')
        )


@dataclass(frozen=True)
class GeometryExpectation:
    """The profile identity configured for a named robot instance."""

    profile_id: str
    geometry_version: str
    asset_hash: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self, 'profile_id', _nonempty_text(self.profile_id, 'profile_id')
        )
        object.__setattr__(
            self,
            'geometry_version',
            _nonempty_text(self.geometry_version, 'geometry_version'),
        )
        object.__setattr__(
            self, 'asset_hash', _nonempty_text(self.asset_hash, 'asset_hash')
        )


@dataclass(frozen=True)
class RobotOccupancySnapshot:
    """Immutable geometry and provenance for a single robot at one time."""

    robot_id: str
    profile_id: str
    geometry_version: str
    asset_hash: str
    reference_frame: str
    stamp_sec: float
    ttl_sec: float
    capsules: Tuple[CapsuleOccupancy, ...]
    spheres: Tuple[SphereOccupancy, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(
            self, 'robot_id', _nonempty_text(self.robot_id, 'robot_id')
        )
        object.__setattr__(
            self, 'profile_id', _nonempty_text(self.profile_id, 'profile_id')
        )
        object.__setattr__(
            self,
            'geometry_version',
            _nonempty_text(self.geometry_version, 'geometry_version'),
        )
        object.__setattr__(
            self, 'asset_hash', _nonempty_text(self.asset_hash, 'asset_hash')
        )
        object.__setattr__(
            self,
            'reference_frame',
            _nonempty_text(self.reference_frame, 'reference_frame'),
        )
        object.__setattr__(
            self, 'stamp_sec', _finite_nonnegative(self.stamp_sec, 'stamp_sec')
        )
        ttl = float(self.ttl_sec)
        if not math.isfinite(ttl) or ttl <= 0.0:
            raise ValueError('ttl_sec must be finite and greater than zero')
        object.__setattr__(self, 'ttl_sec', ttl)

        capsules = tuple(self.capsules)
        spheres = tuple(self.spheres)
        if not capsules and not spheres:
            raise ValueError('snapshot must contain at least one primitive')
        if not all(isinstance(item, CapsuleOccupancy) for item in capsules):
            raise ValueError('capsules must contain CapsuleOccupancy values')
        if not all(isinstance(item, SphereOccupancy) for item in spheres):
            raise ValueError('spheres must contain SphereOccupancy values')
        names = tuple(item.name for item in (*capsules, *spheres))
        if len(set(names)) != len(names):
            raise ValueError(
                'primitive names must be unique within a snapshot'
            )
        object.__setattr__(self, 'capsules', capsules)
        object.__setattr__(self, 'spheres', spheres)

    @property
    def expectation(self) -> GeometryExpectation:
        """Return the identity used to authorize this snapshot."""
        return GeometryExpectation(
            self.profile_id, self.geometry_version, self.asset_hash
        )


@dataclass(frozen=True)
class PrimitiveClearance:
    """Clearance witness between two primitives from different robots."""

    clearance: float
    first_robot_id: str
    first_primitive: str
    second_robot_id: str
    second_primitive: str


@dataclass(frozen=True)
class RobotPairClearance:
    """Minimum clearance of one pair of robot occupancy snapshots."""

    first_robot_id: str
    second_robot_id: str
    minimum: PrimitiveClearance
    evaluated_combinations: int


@dataclass(frozen=True)
class MultiArmDecision:
    """Deterministic global authorization result for all four robots."""

    state: str
    reason_code: str
    clearance_threshold: float
    minimum_clearance: Optional[float]
    limiting: Optional[PrimitiveClearance]
    pair_clearances: Tuple[RobotPairClearance, ...]
    evaluated_robot_pairs: Tuple[Tuple[str, str], ...]


def _dot(first: Point3, second: Point3) -> float:
    return sum(first[index] * second[index] for index in range(3))


def _subtract(first: Point3, second: Point3) -> Point3:
    return tuple(first[index] - second[index] for index in range(3))


def _add_scaled(point: Point3, axis: Point3, factor: float) -> Point3:
    return tuple(point[index] + axis[index] * factor for index in range(3))


def closest_points_between_segments(
    first_start: Sequence[float],
    first_end: Sequence[float],
    second_start: Sequence[float],
    second_end: Sequence[float],
) -> Tuple[Point3, Point3]:
    """Return closest witnesses for two finite segments, including points."""
    p0 = finite_point(first_start, 'first segment start')
    p1 = finite_point(first_end, 'first segment end')
    q0 = finite_point(second_start, 'second segment start')
    q1 = finite_point(second_end, 'second segment end')
    axis_p = _subtract(p1, p0)
    axis_q = _subtract(q1, q0)
    between = _subtract(p0, q0)
    first_length = _dot(axis_p, axis_p)
    second_length = _dot(axis_q, axis_q)
    first_dot_second = _dot(axis_p, axis_q)
    first_offset = _dot(axis_p, between)
    second_offset = _dot(axis_q, between)

    if (
        first_length <= DEGENERATE_SEGMENT_EPSILON_SQ
        and second_length <= DEGENERATE_SEGMENT_EPSILON_SQ
    ):
        return p0, q0
    if first_length <= DEGENERATE_SEGMENT_EPSILON_SQ:
        second_factor = min(
            1.0, max(0.0, second_offset / second_length)
        )
        return p0, _add_scaled(q0, axis_q, second_factor)
    if second_length <= DEGENERATE_SEGMENT_EPSILON_SQ:
        first_factor = min(1.0, max(0.0, -first_offset / first_length))
        return _add_scaled(p0, axis_p, first_factor), q0

    denominator = first_length * second_length - first_dot_second ** 2
    if denominator > EPSILON:
        first_factor = min(
            1.0,
            max(
                0.0,
                (
                    first_dot_second * second_offset
                    - first_offset * second_length
                )
                / denominator,
            ),
        )
    else:
        first_factor = 0.0

    second_numerator = first_dot_second * first_factor + second_offset
    if second_numerator <= 0.0:
        second_factor = 0.0
        first_factor = min(1.0, max(0.0, -first_offset / first_length))
    elif second_numerator >= second_length:
        second_factor = 1.0
        first_factor = min(
            1.0,
            max(0.0, (first_dot_second - first_offset) / first_length),
        )
    else:
        second_factor = second_numerator / second_length
    return (
        _add_scaled(p0, axis_p, first_factor),
        _add_scaled(q0, axis_q, second_factor),
    )


def capsule_capsule_clearance(
    first: CapsuleOccupancy, second: CapsuleOccupancy
) -> float:
    """Return signed surface clearance between two finite capsules."""
    first_point, second_point = closest_points_between_segments(
        first.start, first.end, second.start, second.end
    )
    return math.dist(first_point, second_point) - first.radius - second.radius


def capsule_sphere_clearance(
    capsule: CapsuleOccupancy, sphere: SphereOccupancy
) -> float:
    """Return signed surface clearance between a capsule and a sphere."""
    axis = _subtract(capsule.end, capsule.start)
    to_center = _subtract(sphere.center, capsule.start)
    length_squared = _dot(axis, axis)
    if length_squared <= DEGENERATE_SEGMENT_EPSILON_SQ:
        closest = capsule.start
    else:
        factor = min(1.0, max(0.0, _dot(axis, to_center) / length_squared))
        closest = _add_scaled(capsule.start, axis, factor)
    return math.dist(closest, sphere.center) - capsule.radius - sphere.radius


def sphere_sphere_clearance(
    first: SphereOccupancy, second: SphereOccupancy
) -> float:
    """Return signed surface clearance between two spherical features."""
    return (
        math.dist(first.center, second.center) - first.radius - second.radius
    )


def _pair_clearance(
    first: RobotOccupancySnapshot, second: RobotOccupancySnapshot
) -> RobotPairClearance:
    """Evaluate all supported primitive pairs and retain the limiting one."""
    results = []
    for first_capsule in first.capsules:
        for second_capsule in second.capsules:
            results.append(PrimitiveClearance(
                capsule_capsule_clearance(first_capsule, second_capsule),
                first.robot_id,
                first_capsule.name,
                second.robot_id,
                second_capsule.name,
            ))
        for second_sphere in second.spheres:
            results.append(PrimitiveClearance(
                capsule_sphere_clearance(first_capsule, second_sphere),
                first.robot_id,
                first_capsule.name,
                second.robot_id,
                second_sphere.name,
            ))
    for first_sphere in first.spheres:
        for second_capsule in second.capsules:
            results.append(PrimitiveClearance(
                capsule_sphere_clearance(second_capsule, first_sphere),
                first.robot_id,
                first_sphere.name,
                second.robot_id,
                second_capsule.name,
            ))
        for second_sphere in second.spheres:
            results.append(PrimitiveClearance(
                sphere_sphere_clearance(first_sphere, second_sphere),
                first.robot_id,
                first_sphere.name,
                second.robot_id,
                second_sphere.name,
            ))
    if not results:
        raise ValueError('robot pair has no primitive combinations')
    minimum = min(
        results,
        key=lambda item: (
            item.clearance,
            item.first_primitive,
            item.second_primitive,
        ),
    )
    return RobotPairClearance(
        first.robot_id,
        second.robot_id,
        minimum,
        len(results),
    )


def _stop(
    reason_code: str,
    threshold: float,
    evaluated_pairs: Tuple[Tuple[str, str], ...] = (),
) -> MultiArmDecision:
    """Return one fail-closed decision with no untrusted geometry result."""
    return MultiArmDecision(
        'STOP',
        reason_code,
        threshold,
        None,
        None,
        (),
        evaluated_pairs,
    )


def fail_closed_decision(
    reason_code: str, clearance_threshold: float
) -> MultiArmDecision:
    """Expose a validated STOP for runtime admission failures."""
    reason = _nonempty_text(reason_code, 'reason_code')
    threshold = _finite_nonnegative(
        clearance_threshold, 'clearance_threshold'
    )
    return _stop(reason, threshold)


def _snapshot_fresh(snapshot: RobotOccupancySnapshot, now_sec: float) -> bool:
    """Return true only for a non-future snapshot inside its declared TTL."""
    age = now_sec - snapshot.stamp_sec
    return -EPSILON <= age <= snapshot.ttl_sec + EPSILON


def evaluate_four_arm_safety(
    snapshots: Sequence[RobotOccupancySnapshot],
    now_sec: float,
    clearance_threshold: float,
    expected_geometry: Optional[Mapping[str, GeometryExpectation]] = None,
) -> MultiArmDecision:
    """
    Evaluate four synchronized occupancy snapshots without ROS dependencies.

    The coordinator fails closed when the set is not exactly four unique
    robots, provenance does not match the configured geometry, frame identity
    differs, or any snapshot is future-dated/stale.  Different robots may use
    different approved profiles; ``expected_geometry`` binds each robot to its
    intended profile identity.
    """
    current_time = _finite_nonnegative(now_sec, 'now_sec')
    threshold = _finite_nonnegative(clearance_threshold, 'clearance_threshold')
    values = tuple(snapshots)
    if len(values) != FOUR_ROBOT_COUNT:
        return _stop('FOUR_ROBOT_SET_REQUIRED', threshold)
    if not all(isinstance(value, RobotOccupancySnapshot) for value in values):
        return _stop('SNAPSHOT_TYPE_INVALID', threshold)
    ordered = tuple(sorted(values, key=lambda value: value.robot_id))
    robot_ids = tuple(value.robot_id for value in ordered)
    if len(set(robot_ids)) != FOUR_ROBOT_COUNT:
        return _stop('ROBOT_ID_DUPLICATE', threshold)
    frames = {value.reference_frame for value in ordered}
    if len(frames) != 1:
        return _stop('REFERENCE_FRAME_MISMATCH', threshold)
    for snapshot in ordered:
        if snapshot.stamp_sec > current_time + EPSILON:
            return _stop('SNAPSHOT_TIMESTAMP_FUTURE', threshold)
        if not _snapshot_fresh(snapshot, current_time):
            return _stop('SNAPSHOT_STALE', threshold)
        if expected_geometry is not None:
            expected = expected_geometry.get(snapshot.robot_id)
            if expected is None or snapshot.expectation != expected:
                return _stop('GEOMETRY_PROFILE_MISMATCH', threshold)

    pairs = tuple(combinations(ordered, 2))
    evaluated_pairs = tuple(
        (first.robot_id, second.robot_id) for first, second in pairs
    )
    results = tuple(_pair_clearance(first, second) for first, second in pairs)
    limiting_pair = min(
        results,
        key=lambda item: (
            item.minimum.clearance,
            item.first_robot_id,
            item.second_robot_id,
        ),
    )
    limiting = limiting_pair.minimum
    state = 'ALLOW' if limiting.clearance > threshold else 'STOP'
    reason = 'CLEAR' if state == 'ALLOW' else 'INTERROBOT_CLEARANCE_STOP'
    return MultiArmDecision(
        state,
        reason,
        threshold,
        limiting.clearance,
        limiting,
        results,
        evaluated_pairs,
    )
