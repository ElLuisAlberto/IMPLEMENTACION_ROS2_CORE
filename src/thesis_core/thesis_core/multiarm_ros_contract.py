"""Pure helpers at the ROS boundary of the multi-arm coordinator."""

from __future__ import annotations

from typing import Tuple

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from thesis_core.geometry_profile import (
    CapsulePrimitive,
    GeometrySnapshot,
    SpherePrimitive,
)
from thesis_core.rigid_transform import (
    transform_from_translation_quaternion,
)


VALID_GEOMETRY_DECISION_CODES = frozenset((
    'CLEAR',
    'INTERROBOT_CLEARANCE_STOP',
))


def geometry_snapshot_qos() -> QoSProfile:
    """Keep only the latest reliable descriptor at each ROS boundary."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def time_message_key(stamp) -> int:
    """Return a comparable nanosecond key for a validated ROS-like stamp."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as exception:
        raise ValueError('stamp must contain integer sec and nanosec') from exception
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('stamp must be normalized and non-negative')
    return sec * 1_000_000_000 + nanosec


def stamp_is_strictly_newer(stamp_key: int, previous_key) -> bool:
    """Accept an initial stamp or one strictly newer than the previous key."""
    candidate = int(stamp_key)
    if candidate <= 0:
        return False
    return previous_key is None or candidate > int(previous_key)


def required_snapshot_frames(snapshot: GeometrySnapshot) -> Tuple[str, ...]:
    """Return the unique TF frames needed for an all-or-nothing projection."""
    if not isinstance(snapshot, GeometrySnapshot):
        raise ValueError('snapshot must be a GeometrySnapshot')
    frames = set()
    for primitive in snapshot.primitives:
        if isinstance(primitive, CapsulePrimitive):
            frames.add(primitive.start_frame)
            frames.add(primitive.end_frame)
        elif isinstance(primitive, SpherePrimitive):
            frames.add(primitive.frame)
        else:
            raise ValueError('snapshot contains an unsupported primitive')
    if not frames:
        raise ValueError('snapshot must require at least one TF frame')
    return tuple(sorted(frames))


def transform_message_matrix(transform_stamped):
    """Convert a ROS-like TransformStamped into a homogeneous matrix."""
    try:
        transform = transform_stamped.transform
        translation = transform.translation
        rotation = transform.rotation
        xyz = (translation.x, translation.y, translation.z)
        xyzw = (rotation.x, rotation.y, rotation.z, rotation.w)
    except AttributeError as exception:
        raise ValueError('transform message is incomplete') from exception
    return transform_from_translation_quaternion(xyz, xyzw)


def geometry_decision_is_valid(reason_code: str) -> bool:
    """Distinguish evaluated geometry from a fail-closed admission STOP."""
    return str(reason_code) in VALID_GEOMETRY_DECISION_CODES
