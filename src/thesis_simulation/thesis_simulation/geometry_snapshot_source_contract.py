"""Pure serialization contract for per-instance geometry evidence."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

from thesis_core.geometry_profile import (
    CapsulePrimitive,
    GeometryProfile,
    SpherePrimitive,
)


CANONICAL_ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')


@dataclass(frozen=True)
class GeometryTransportManifest:
    """Parallel primitive arrays ready for ``GeometrySnapshot.msg``."""

    primitive_names: Tuple[str, ...]
    primitive_types: Tuple[str, ...]
    start_frames: Tuple[str, ...]
    end_frames: Tuple[str, ...]
    start_offsets: Tuple[Tuple[float, float, float], ...]
    end_offsets: Tuple[Tuple[float, float, float], ...]
    radii: Tuple[float, ...]


def geometry_transport_manifest(
    profile: GeometryProfile,
    state: str,
) -> GeometryTransportManifest:
    """Serialize one complete envelope without losing arm primitives."""
    if not isinstance(profile, GeometryProfile):
        raise ValueError('profile must be a GeometryProfile')
    envelope = profile.envelope_for(state)
    names = []
    kinds = []
    start_frames = []
    end_frames = []
    start_offsets = []
    end_offsets = []
    radii = []
    for primitive in envelope.primitives:
        names.append(primitive.name)
        radii.append(primitive.radius)
        if isinstance(primitive, CapsulePrimitive):
            kinds.append('CAPSULE')
            start_frames.append(primitive.start_frame)
            end_frames.append(primitive.end_frame)
            start_offsets.append(primitive.start_offset)
            end_offsets.append(primitive.end_offset)
        elif isinstance(primitive, SpherePrimitive):
            kinds.append('SPHERE')
            start_frames.append(primitive.frame)
            end_frames.append('')
            start_offsets.append(primitive.center)
            end_offsets.append((0.0, 0.0, 0.0))
        else:
            raise ValueError('profile contains an unsupported primitive')
    return GeometryTransportManifest(
        tuple(names),
        tuple(kinds),
        tuple(start_frames),
        tuple(end_frames),
        tuple(start_offsets),
        tuple(end_offsets),
        tuple(radii),
    )


def complete_joint_sample(
    names: Sequence[str],
    positions: Sequence[float],
    required_names: Sequence[str],
) -> bool:
    """Accept only a finite sample containing every configured arm joint."""
    actual_names = tuple(str(name) for name in names)
    required = tuple(str(name) for name in required_names)
    if (
        len(required) != 6
        or len(set(required)) != len(required)
        or any(not name for name in required)
        or len(actual_names) != len(positions)
        or len(set(actual_names)) != len(actual_names)
    ):
        return False
    by_name = dict(zip(actual_names, positions))
    if any(name not in by_name for name in required):
        return False
    try:
        return all(math.isfinite(float(by_name[name])) for name in required)
    except (TypeError, ValueError):
        return False


def validate_offline_identity(robot_id: str, source_kind: str) -> None:
    """Reject any physical source or an offline role assigned to the wrong ID."""
    if robot_id not in CANONICAL_ROBOT_IDS:
        raise ValueError('robot_id must be arm_01 through arm_04')
    expected = 'synthetic_state' if robot_id == 'arm_01' else 'gazebo'
    if source_kind != expected:
        raise ValueError(f'{robot_id} source_kind must be {expected}')
