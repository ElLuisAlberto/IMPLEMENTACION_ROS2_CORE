"""Fail-closed conversion of ROS-like geometry snapshot messages."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

from thesis_core.geometry_profile import (
    CapsulePrimitive,
    CONFIGURED_STATES,
    GeometryProfile,
    GeometrySnapshot,
    SpherePrimitive,
    StateEnvelope,
)


MAX_PRIMITIVES = 128
OFFLINE_SOURCE_KINDS = ('synthetic_state', 'gazebo', 'replay')


def _text(value, field_name: str) -> str:
    result = str(value)
    if not result or result != result.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return result


def _seconds(value, field_name: str) -> float:
    try:
        result = float(value.sec) + float(value.nanosec) * 1.0e-9
    except (AttributeError, TypeError, ValueError) as exception:
        raise ValueError(f'{field_name} must be a ROS time/duration value') from exception
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f'{field_name} must be finite and non-negative')
    return result


def _point(value, field_name: str) -> Tuple[float, float, float]:
    try:
        result = (float(value.x), float(value.y), float(value.z))
    except (AttributeError, TypeError, ValueError) as exception:
        raise ValueError(f'{field_name} must contain finite xyz values') from exception
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f'{field_name} must contain finite xyz values')
    return result


def _parallel(values: Iterable, expected: int, field_name: str):
    result = tuple(values)
    if len(result) != expected:
        raise ValueError(f'{field_name} length must match primitive_names')
    return result


def geometry_snapshot_from_message(
    message,
    *,
    maximum_primitives: int = MAX_PRIMITIVES,
) -> GeometrySnapshot:
    """Build an immutable local snapshot or reject the entire descriptor."""
    if not bool(getattr(message, 'valid', False)):
        raise ValueError('geometry snapshot is explicitly invalid')
    session_id = _text(message.session_id, 'session_id')
    source_kind = _text(message.source_kind, 'source_kind')
    if source_kind not in OFFLINE_SOURCE_KINDS:
        raise ValueError(f'unsupported offline source_kind: {source_kind!r}')
    robot_id = _text(message.robot_id, 'robot_id')
    profile_id = _text(message.profile_id, 'profile_id')
    geometry_version = _text(message.geometry_version, 'geometry_version')
    asset_hash = _text(message.asset_hash, 'asset_hash')
    state = _text(message.state, 'state')
    if state not in CONFIGURED_STATES:
        raise ValueError('active geometry state must be explicitly configured')
    _text(message.reference_frame, 'reference_frame')
    stamp_sec = _seconds(message.stamp, 'stamp')
    ttl_sec = _seconds(message.valid_for, 'valid_for')
    if stamp_sec <= 0.0:
        raise ValueError('stamp must be greater than zero')
    if ttl_sec <= 0.0:
        raise ValueError('valid_for must be greater than zero')

    names = tuple(message.primitive_names)
    if not names or len(names) > int(maximum_primitives):
        raise ValueError('primitive count is outside the configured bounds')
    primitive_types = _parallel(
        message.primitive_types, len(names), 'primitive_types'
    )
    start_frames = _parallel(
        message.start_frames, len(names), 'start_frames'
    )
    end_frames = _parallel(message.end_frames, len(names), 'end_frames')
    start_offsets = _parallel(
        message.start_offsets, len(names), 'start_offsets'
    )
    end_offsets = _parallel(
        message.end_offsets, len(names), 'end_offsets'
    )
    radii = _parallel(message.radii, len(names), 'radii')

    primitives = []
    for index, raw_name in enumerate(names):
        name = _text(raw_name, f'primitive_names[{index}]')
        kind = _text(
            primitive_types[index], f'primitive_types[{index}]'
        ).upper()
        radius = float(radii[index])
        if kind == 'CAPSULE':
            primitives.append(CapsulePrimitive(
                name=name,
                start_frame=_text(
                    start_frames[index], f'start_frames[{index}]'
                ),
                end_frame=_text(
                    end_frames[index], f'end_frames[{index}]'
                ),
                radius=radius,
                start_offset=_point(
                    start_offsets[index], f'start_offsets[{index}]'
                ),
                end_offset=_point(
                    end_offsets[index], f'end_offsets[{index}]'
                ),
            ))
        elif kind == 'SPHERE':
            primitives.append(SpherePrimitive(
                name=name,
                frame=_text(
                    start_frames[index], f'start_frames[{index}]'
                ),
                radius=radius,
                center=_point(
                    start_offsets[index], f'start_offsets[{index}]'
                ),
            ))
        else:
            raise ValueError(f'unsupported primitive type: {kind!r}')

    profile = GeometryProfile(
        robot_id=robot_id,
        profile_id=profile_id,
        geometry_version=geometry_version,
        asset_hash=asset_hash,
        mount_frame=_text(message.mount_frame, 'mount_frame'),
        tcp_frame=_text(message.tcp_frame, 'tcp_frame'),
        envelopes=(StateEnvelope(state, tuple(primitives)),),
    )
    snapshot = GeometrySnapshot(profile, state, stamp_sec, ttl_sec)
    # Keep transport identity available without weakening immutable geometry.
    object.__setattr__(snapshot, 'session_id', session_id)
    object.__setattr__(snapshot, 'source_kind', source_kind)
    object.__setattr__(snapshot, 'reference_frame', str(message.reference_frame))
    return snapshot
