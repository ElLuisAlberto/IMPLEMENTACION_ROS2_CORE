"""Regression tests for pure multi-arm campaign measurements."""

import math

import pytest

from thesis_telemetry.multiarm_metrics import (
    age_skew_summary,
    decision_control_latency_summary,
    jitter_summary,
    percentile,
    summarize,
)


def test_summary_reports_required_descriptive_statistics_and_percentiles():
    result = summarize((1.0, 2.0, 3.0, 4.0))

    assert result.count == 4
    assert result.mean == pytest.approx(2.5)
    assert result.stddev == pytest.approx(math.sqrt(1.25))
    assert result.minimum == 1.0
    assert result.maximum == 4.0
    assert result.p50 == pytest.approx(2.5)
    assert result.p95 == pytest.approx(3.85)
    assert result.p99 == pytest.approx(3.97)


def test_percentile_interpolates_and_rejects_invalid_inputs():
    assert percentile((10.0, 20.0), 50.0) == pytest.approx(15.0)
    with pytest.raises(ValueError, match='must not be empty'):
        percentile((), 50.0)
    with pytest.raises(ValueError, match='within'):
        percentile((1.0,), 100.1)


@pytest.mark.parametrize('values', ((-0.1,), (math.nan,), (math.inf,)))
def test_metrics_reject_non_finite_or_negative_samples(values):
    with pytest.raises(ValueError, match='finite and non-negative'):
        summarize(values)


def test_jitter_uses_deviation_from_mean_interarrival_period():
    result = jitter_summary((0.0, 1.0, 3.0, 4.0))

    assert result.count == 3
    assert result.mean == pytest.approx(4.0 / 9.0)
    assert result.minimum == pytest.approx(1.0 / 3.0)
    assert result.maximum == pytest.approx(2.0 / 3.0)


@pytest.mark.parametrize('timestamps', ((1.0,), (0.0, 0.0), (2.0, 1.0)))
def test_jitter_requires_strictly_increasing_timestamp_series(timestamps):
    with pytest.raises(ValueError):
        jitter_summary(timestamps)


def test_age_and_skew_track_the_four_acquisition_times():
    result = age_skew_summary(10.0, (9.5, 9.0, 8.0, 9.25))

    assert result.ages.count == 4
    assert result.ages.minimum == pytest.approx(0.5)
    assert result.ages.maximum == pytest.approx(2.0)
    assert result.skew_sec == pytest.approx(1.5)


def test_age_and_skew_reject_future_or_invalid_timestamps():
    with pytest.raises(ValueError, match='future'):
        age_skew_summary(1.0, (1.1,))
    with pytest.raises(ValueError, match='finite and non-negative'):
        age_skew_summary(-1.0, (0.0,))


def test_decision_to_control_latency_summary_requires_nonnegative_delays():
    result = decision_control_latency_summary(5.0, (5.0, 5.2, 5.5))

    assert result.count == 3
    assert result.minimum == pytest.approx(0.0)
    assert result.maximum == pytest.approx(0.5)
    assert result.p50 == pytest.approx(0.2)
    with pytest.raises(ValueError, match='precede'):
        decision_control_latency_summary(5.0, (4.9,))
