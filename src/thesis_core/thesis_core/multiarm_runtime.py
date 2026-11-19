"""Synchronized runtime admission before four-arm geometry evaluation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from thesis_core.multiarm_coordination import (
    GeometryExpectation,
    MultiArmDecision,
    RobotOccupancySnapshot,
    fail_closed_decision,
    evaluate_four_arm_safety,
)


CANONICAL_ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')


@dataclass(frozen=True)
class RuntimeOccupancy:
    """One projected occupancy plus transport/session evidence."""

    occupancy: RobotOccupancySnapshot
    session_id: str
    source_kind: str
    received_monotonic_sec: float

    def __post_init__(self):
        if not isinstance(self.occupancy, RobotOccupancySnapshot):
            raise ValueError('occupancy must be a RobotOccupancySnapshot')
        for name in ('session_id', 'source_kind'):
            value = str(getattr(self, name))
            if not value or value != value.strip():
                raise ValueError(f'{name} must be a non-empty trimmed string')
        received = float(self.received_monotonic_sec)
        if not math.isfinite(received) or received < 0.0:
            raise ValueError('received_monotonic_sec must be finite and non-negative')
        object.__setattr__(self, 'received_monotonic_sec', received)


@dataclass(frozen=True)
class RuntimeDecision:
    """Decision plus temporal evidence exposed by the ROS adapter."""

    decision: MultiArmDecision
    maximum_snapshot_age: float
    snapshot_skew: float


def _stop(reason, threshold, age=float('nan'), skew=float('nan')):
    return RuntimeDecision(
        fail_closed_decision(reason, threshold), float(age), float(skew)
    )


def evaluate_runtime_batch(
    values: Sequence[RuntimeOccupancy],
    *,
    now_sec: float,
    monotonic_now_sec: float,
    clearance_threshold: float,
    session_id: str,
    expected_geometry: Mapping[str, GeometryExpectation],
    expected_sources: Mapping[str, str],
    maximum_receive_age_sec: float,
    maximum_snapshot_skew_sec: float,
) -> RuntimeDecision:
    """Admit one atomic four-source set and invoke the deterministic kernel."""
    threshold = float(clearance_threshold)
    now = float(now_sec)
    monotonic_now = float(monotonic_now_sec)
    receive_limit = float(maximum_receive_age_sec)
    skew_limit = float(maximum_snapshot_skew_sec)
    scalars = (threshold, now, monotonic_now, receive_limit, skew_limit)
    if not all(math.isfinite(value) for value in scalars):
        raise ValueError('runtime limits and clocks must be finite')
    if min(threshold, now, monotonic_now) < 0.0:
        raise ValueError('threshold and clocks must be non-negative')
    if receive_limit <= 0.0 or skew_limit < 0.0:
        raise ValueError('receive age must be positive and skew non-negative')
    if not isinstance(session_id, str) or not session_id.strip():
        raise ValueError('session_id must be non-empty')
    if tuple(sorted(expected_geometry)) != CANONICAL_ROBOT_IDS:
        return _stop('EXPECTED_GEOMETRY_INCOMPLETE', threshold)
    if tuple(sorted(expected_sources)) != CANONICAL_ROBOT_IDS:
        return _stop('EXPECTED_SOURCES_INCOMPLETE', threshold)
    records = tuple(values)
    if len(records) != len(CANONICAL_ROBOT_IDS):
        return _stop('FOUR_ROBOT_SET_REQUIRED', threshold)
    if not all(isinstance(item, RuntimeOccupancy) for item in records):
        return _stop('RUNTIME_OCCUPANCY_TYPE_INVALID', threshold)
    by_id = {item.occupancy.robot_id: item for item in records}
    if tuple(sorted(by_id)) != CANONICAL_ROBOT_IDS or len(by_id) != len(records):
        return _stop('CANONICAL_ROBOT_SET_REQUIRED', threshold)
    if any(item.session_id != session_id for item in records):
        return _stop('SESSION_MISMATCH', threshold)
    if any(
        item.source_kind != expected_sources[item.occupancy.robot_id]
        for item in records
    ):
        return _stop('SOURCE_KIND_MISMATCH', threshold)
    receive_ages = tuple(
        monotonic_now - item.received_monotonic_sec for item in records
    )
    if any(age < 0.0 for age in receive_ages):
        return _stop('RECEIVE_TIMESTAMP_FUTURE', threshold)
    maximum_age = max(receive_ages)
    stamps = tuple(item.occupancy.stamp_sec for item in records)
    skew = max(stamps) - min(stamps)
    if maximum_age > receive_limit:
        return _stop('SNAPSHOT_RECEIVE_STALE', threshold, maximum_age, skew)
    if skew > skew_limit:
        return _stop('SNAPSHOT_SKEW_EXCEEDED', threshold, maximum_age, skew)
    decision = evaluate_four_arm_safety(
        tuple(item.occupancy for item in records),
        now,
        threshold,
        expected_geometry,
    )
    return RuntimeDecision(decision, maximum_age, skew)
