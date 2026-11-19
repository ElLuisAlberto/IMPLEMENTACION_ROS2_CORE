"""
Pure, deterministic metrics for offline multi-arm safety campaigns.

The functions in this module deliberately have no ROS dependency.  They
operate on seconds captured by one time domain and reject incomplete or
ambiguous measurements instead of silently dropping them.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


def _finite_nonnegative(value: float, name: str) -> float:
    """Normalize one elapsed-time or timestamp value."""
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f'{name} must be finite and non-negative')
    return numeric


def _samples(values: Iterable[float], name: str) -> Tuple[float, ...]:
    """Return a non-empty immutable sequence of valid metric samples."""
    result = tuple(_finite_nonnegative(value, name) for value in values)
    if not result:
        raise ValueError(f'{name} must not be empty')
    return result


def percentile(values: Iterable[float], percentile_value: float) -> float:
    """Return a linearly interpolated percentile of non-negative samples."""
    requested = float(percentile_value)
    if not math.isfinite(requested) or not 0.0 <= requested <= 100.0:
        raise ValueError('percentile_value must be finite and within [0, 100]')
    ordered = sorted(_samples(values, 'values'))
    if len(ordered) == 1:
        return ordered[0]
    position = (requested / 100.0) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


@dataclass(frozen=True)
class MetricSummary:
    """Descriptive summary suitable for one reproducible campaign metric."""

    count: int
    mean: float
    stddev: float
    minimum: float
    maximum: float
    p50: float
    p95: float
    p99: float


def summarize(values: Iterable[float]) -> MetricSummary:
    """Summarize finite, non-negative samples using population standard deviation."""
    samples = _samples(values, 'values')
    count = len(samples)
    mean = sum(samples) / count
    variance = sum((value - mean) ** 2 for value in samples) / count
    return MetricSummary(
        count=count,
        mean=mean,
        stddev=math.sqrt(variance),
        minimum=min(samples),
        maximum=max(samples),
        p50=percentile(samples, 50.0),
        p95=percentile(samples, 95.0),
        p99=percentile(samples, 99.0),
    )


def jitter_summary(timestamps_sec: Iterable[float]) -> MetricSummary:
    """
    Summarize absolute inter-arrival deviations from their mean period.

    Timestamps must be strictly increasing and belong to a single clock
    domain.  A one-sample stream has no observable jitter and is rejected.
    """
    timestamps = _samples(timestamps_sec, 'timestamps_sec')
    if len(timestamps) < 2:
        raise ValueError('timestamps_sec must contain at least two samples')
    intervals = tuple(
        following - previous
        for previous, following in zip(timestamps, timestamps[1:])
    )
    if any(interval <= 0.0 for interval in intervals):
        raise ValueError('timestamps_sec must be strictly increasing')
    expected_period = sum(intervals) / len(intervals)
    return summarize(abs(interval - expected_period) for interval in intervals)


@dataclass(frozen=True)
class AgeSkewSummary:
    """Age distribution and acquisition skew for one coherent snapshot set."""

    ages: MetricSummary
    skew_sec: float


def age_skew_summary(
    evaluation_time_sec: float,
    acquisition_times_sec: Iterable[float],
) -> AgeSkewSummary:
    """Measure snapshot ages and max-min acquisition skew at evaluation time."""
    evaluation_time = _finite_nonnegative(
        evaluation_time_sec, 'evaluation_time_sec'
    )
    acquisitions = _samples(acquisition_times_sec, 'acquisition_times_sec')
    ages = tuple(evaluation_time - stamp for stamp in acquisitions)
    if any(age < 0.0 for age in ages):
        raise ValueError('acquisition_times_sec must not be in the future')
    return AgeSkewSummary(
        ages=summarize(ages),
        skew_sec=max(acquisitions) - min(acquisitions),
    )


def decision_control_latency_summary(
    decision_time_sec: float,
    control_times_sec: Iterable[float],
) -> MetricSummary:
    """Summarize decision-to-control delays in one shared time domain."""
    decision_time = _finite_nonnegative(decision_time_sec, 'decision_time_sec')
    control_times = _samples(control_times_sec, 'control_times_sec')
    latencies = tuple(control_time - decision_time for control_time in control_times)
    if any(latency < 0.0 for latency in latencies):
        raise ValueError('control_times_sec must not precede decision_time_sec')
    return summarize(latencies)
