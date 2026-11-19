"""
Deterministically construct inflated swept capsules.

The legacy JACO2 kinematics remain the default model.  Callers may provide a
different joint count, capsule radii and segment function without changing the
sampling or visualization contract.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence, Tuple, cast

from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments


Point3 = Tuple[float, float, float]
JointConfiguration = Tuple[float, ...]
CapsuleSegment = Tuple[Sequence[float], Sequence[float]]
CapsuleSegments = Callable[
    [Sequence[float]], Sequence[CapsuleSegment]
]


@dataclass(frozen=True)
class CapsuleAxis:
    """One inflated capsule axis and its spatial samples."""

    start: Point3
    end: Point3
    radius: float
    points: Tuple[Point3, ...]


@dataclass(frozen=True)
class SweptVolume:
    """Discrete future capsules grouped by time and segment."""

    samples: Tuple[Tuple[CapsuleAxis, ...], ...]
    current_capsules: Tuple[CapsuleAxis, ...]
    predicted_points: Tuple[Tuple[Point3, ...], ...]
    endpoint_traces: Tuple[Tuple[Point3, ...], ...]
    margin: float
    maximum_spacing: float


def _finite_point(values: Sequence[float], name: str) -> Point3:
    if len(values) != 3:
        raise ValueError(f'{name} must contain three values')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} contains a non-finite value')
    return cast(Point3, result)


def _configuration(
    values: Sequence[float], expected_count: int,
) -> JointConfiguration:
    if len(values) != expected_count:
        raise ValueError(
            'all configurations must use the model joint count'
        )
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError('configuration contains a non-finite value')
    return cast(JointConfiguration, result)


def sample_capsule_axis(
    start: Sequence[float],
    end: Sequence[float],
    maximum_spacing: float,
) -> Tuple[Point3, ...]:
    """Sample a segment including both endpoints with bounded spacing."""
    start_point = _finite_point(start, 'start')
    end_point = _finite_point(end, 'end')
    spacing = float(maximum_spacing)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('maximum_spacing must be positive and finite')

    length = math.dist(start_point, end_point)
    steps = max(1, math.ceil(length / spacing))
    intermediate = tuple(cast(Point3, tuple(
        start_point[axis]
        + (end_point[axis] - start_point[axis]) * index / steps
        for axis in range(3)
    )) for index in range(1, steps))
    return (start_point, *intermediate, end_point)


def build_swept_volume(
    configurations: Sequence[Sequence[float]],
    margin: float,
    maximum_spacing: float,
    capsule_radii: Sequence[float] = CAPSULE_RADII,
    segment_function: CapsuleSegments = capsule_segments,
    joint_count: int | None = None,
) -> SweptVolume:
    """
    Return the union samples of inflated model capsules over time.

    ``capsule_radii`` and ``segment_function`` define the collision model.
    Their defaults reproduce the historical six-axis JACO2 result exactly.
    ``joint_count`` is inferred from the first configuration unless supplied,
    which allows five- and seven-axis profiles while rejecting mixed vectors.
    """
    if not configurations:
        raise ValueError('at least one configuration is required')
    inflated_margin = float(margin)
    spacing = float(maximum_spacing)
    if not math.isfinite(inflated_margin) or inflated_margin < 0.0:
        raise ValueError('margin must be finite and non-negative')
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('maximum_spacing must be positive and finite')

    radii = tuple(float(radius) for radius in capsule_radii)
    if (
        not radii
        or not all(math.isfinite(radius) and radius >= 0.0
                   for radius in radii)
    ):
        raise ValueError(
            'capsule_radii must contain finite non-negative values'
        )
    model_joint_count = (
        len(configurations[0]) if joint_count is None else int(joint_count)
    )
    if model_joint_count <= 0:
        raise ValueError('joint_count must be greater than zero')
    if not callable(segment_function):
        raise TypeError('segment_function must be callable')

    temporal_samples = []
    for configuration in configurations:
        positions = _configuration(configuration, model_joint_count)
        segments = tuple(segment_function(positions))
        if len(segments) != len(radii):
            raise ValueError('kinematics and capsule radii size mismatch')
        axes = tuple(
            CapsuleAxis(
                start=_finite_point(start, 'capsule start'),
                end=_finite_point(end, 'capsule end'),
                radius=float(radius) + inflated_margin,
                points=sample_capsule_axis(start, end, spacing),
            )
            for (start, end), radius in zip(segments, radii)
        )
        temporal_samples.append(axes)

    samples = tuple(temporal_samples)
    predicted_points = []
    endpoint_traces = []
    for segment_index in range(len(radii)):
        unique = {}
        trace = []
        for sample in samples:
            axis = sample[segment_index]
            trace.append(axis.end)
            for point in axis.points:
                key = tuple(round(value, 9) for value in point)
                unique.setdefault(key, point)
        predicted_points.append(tuple(unique.values()))
        endpoint_traces.append(tuple(trace))

    return SweptVolume(
        samples=samples,
        current_capsules=samples[0],
        predicted_points=tuple(predicted_points),
        endpoint_traces=tuple(endpoint_traces),
        margin=inflated_margin,
        maximum_spacing=spacing,
    )


def maximum_endpoint_travel(volume: SweptVolume) -> float:
    """Return the greatest accumulated endpoint travel of all links."""
    return max((
        sum(math.dist(first, second) for first, second in zip(
            trace, trace[1:]
        ))
        for trace in volume.endpoint_traces
    ), default=0.0)
