"""Profile-model regression tests for the historical safety supervisor."""

from types import SimpleNamespace

import numpy as np
import pytest

from thesis_core.safety_supervisor_node import (
    SafetySupervisorNode,
    JogClockGuard,
    kinematic_tf_position_error,
    profile_geometry,
)


def test_jog_clock_guard_rejects_pause_and_pre_pause_intent_on_resume():
    guard = JogClockGuard(timeout=0.5)
    assert not guard.accepts(100, 0.0, 100)
    assert guard.accepts(110, 0.1, 110)
    assert not guard.accepts(110, 0.7, 110)
    assert not guard.accepts(120, 0.8, 110)
    assert guard.accepts(120, 0.8, 120)
    assert not guard.accepts(50, 0.9, 50)
    assert not guard.accepts(60, 1.0, 50)
    assert guard.accepts(60, 1.0, 60)


def _profile():
    capsules = (
        SimpleNamespace(name='base', radius=0.10),
        SimpleNamespace(name='tool', radius=0.05),
    )

    def evaluated(positions):
        assert len(positions) == 5
        return (
            SimpleNamespace(
                start=(0.0, 0.0, 0.0),
                end=(positions[0], 0.0, 0.2),
            ),
            SimpleNamespace(
                start=(positions[0], 0.0, 0.2),
                end=(positions[1], 0.0, 0.4),
            ),
        )

    def forward(positions):
        transform = np.eye(4)
        transform[:3, 3] = (positions[0], positions[1], 0.4)
        return {'tool_link': transform}

    return SimpleNamespace(
        collision_capsules=capsules,
        capsule_segments=evaluated,
        forward=forward,
        tip_link='tool_link',
    )


def test_profile_geometry_adapts_five_axis_capsules_and_fk():
    names, radii, segments, tip_position = profile_geometry(_profile())
    positions = (0.2, 0.3, 0.0, 0.0, 0.0)

    assert names == ('base', 'tool')
    assert radii == pytest.approx((0.10, 0.05))
    assert len(segments(positions)) == 2
    assert tip_position(positions) == pytest.approx((0.2, 0.3, 0.4))
    assert kinematic_tf_position_error(
        positions,
        (0.2, 0.3, 0.4),
        tip_position,
    ) == pytest.approx(0.0)


def test_current_clearance_uses_configured_five_axis_model():
    names, radii, segments, _ = profile_geometry(_profile())
    node = SimpleNamespace(
        segment_names=names,
        capsule_radii=radii,
        capsule_function=segments,
        joint_names=tuple(f'axis_{index}' for index in range(5)),
    )

    result = SafetySupervisorNode.current_sphere_clearance(
        node,
        (0.2, 0.3, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.2),
        0.05,
    )

    assert len(result) == 5
    assert result[0] == pytest.approx(0.6280109889280517)
    assert result[1] == 'tool'


def test_predictive_threshold_reports_buffer_and_non_approaching_exception():
    """Expose the actual reduction threshold, including withdrawal behavior."""
    params = {
        'stop_distance': 0.05,
        'predictive_stop_reserve_m': 0.005,
    }
    node = SimpleNamespace(
        protective_margin=lambda: 0.02,
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
    )
    approaching = SimpleNamespace(sample_clearances=(0.10, 0.08))
    withdrawing = SimpleNamespace(sample_clearances=(0.052, 0.060))

    assert SafetySupervisorNode.predictive_threshold_for_horizon(
        node, 0.10, approaching
    ) == pytest.approx(0.055)
    assert SafetySupervisorNode.predictive_threshold_for_horizon(
        node, 0.052, withdrawing
    ) == pytest.approx(0.02)
    assert SafetySupervisorNode.predictive_threshold_for_horizon(
        node, 0.04, approaching
    ) == pytest.approx(0.02)


@pytest.mark.parametrize(('end_clearance', 'expected'), [
    (0.08, 'WARNING'),
    (0.11, 'WARNING'),
    (0.04, 'REDUCTION'),
    (-1.0, 'STOP'),
])
def test_horizon_anticipates_stop_zone(
        end_clearance, expected):
    params = {'stop_distance': 0.05, 'warning_distance': 0.30,
              'predictive_stop_reserve_m': 0.005,
              'runtime_minimum_speed_scale': 0.05,
              'runtime_scale_search_iterations': 10}
    node = SimpleNamespace(protective_margin=lambda: 0.02,
                           get_parameter=lambda name: SimpleNamespace(value=params[name]))
    current = 0.11

    def evaluator(scale):
        clearance = current + scale * (end_clearance - current)
        geometry = SimpleNamespace(minimum=SimpleNamespace(clearance=min(current, clearance)),
                                   sample_clearances=(current, clearance),
                                   first_collision_time=None)
        node.last_horizon_clearance = geometry
        return ((geometry.minimum.clearance,), 0, 0.0, -1.0)

    evaluator(1.0)
    decision, _, selected = SafetySupervisorNode.decide_horizon(
        node, current, node.last_horizon_clearance, evaluator)
    assert decision.state == expected
    if expected == 'REDUCTION':
        assert 0 < decision.speed_scale < 1
        assert selected.minimum.clearance >= 0.055


@pytest.mark.parametrize(('end_clearance', 'expected'), [
    (0.05, 'REDUCTION'),
    (0.03, 'STOP'),
])
def test_stop_zone_only_allows_verified_withdrawal(end_clearance, expected):
    params = {'stop_distance': 0.05, 'warning_distance': 0.30,
              'predictive_stop_reserve_m': 0.005,
              'runtime_minimum_speed_scale': 0.05,
              'runtime_scale_search_iterations': 10,
              'withdrawal_speed_scale': 0.10,
              'withdrawal_minimum_progress_m': 0.002,
              'withdrawal_monotonic_tolerance_m': 0.001}
    node = SimpleNamespace(protective_margin=lambda: 0.02,
                           get_parameter=lambda name: SimpleNamespace(value=params[name]))
    current = 0.04

    def evaluator(scale):
        end = current + scale * (end_clearance - current)
        geometry = SimpleNamespace(minimum=SimpleNamespace(clearance=min(current, end)),
                                   sample_clearances=(current, end),
                                   first_collision_time=None)
        node.last_horizon_clearance = geometry
        return ((geometry.minimum.clearance,), 0, 0.0, -1.0)

    evaluator(1.0)
    decision, _, _ = SafetySupervisorNode.decide_horizon(
        node, current, node.last_horizon_clearance, evaluator)
    assert decision.state == expected
    if expected == 'REDUCTION':
        assert decision.reason_code == 'PROTECTIVE_WITHDRAWAL'
        assert decision.speed_scale == pytest.approx(0.1)


def test_non_approaching_motion_inside_predictive_buffer_is_not_blocked():
    params = {'stop_distance': 0.05, 'warning_distance': 0.30,
              'predictive_stop_reserve_m': 0.005,
              'runtime_minimum_speed_scale': 0.05,
              'runtime_scale_search_iterations': 10}
    node = SimpleNamespace(protective_margin=lambda: 0.02,
                           get_parameter=lambda name: SimpleNamespace(value=params[name]))

    def evaluator(scale):
        geometry = SimpleNamespace(minimum=SimpleNamespace(clearance=0.052),
                                   sample_clearances=(0.052, 0.052 + 0.01 * scale),
                                   first_collision_time=None)
        node.last_horizon_clearance = geometry
        return ((0.052,), 0, 0.0, -1.0)
    evaluator(1.0)
    decision, _, _ = SafetySupervisorNode.decide_horizon(
        node, 0.052, node.last_horizon_clearance, evaluator)
    assert decision.state == 'WARNING'
    assert decision.speed_scale == 1.0


def test_close_safe_prediction_keeps_full_speed():
    params = {'stop_distance': 0.05, 'warning_distance': 0.30,
              'predictive_stop_reserve_m': 0.005,
              'runtime_minimum_speed_scale': 0.05,
              'runtime_scale_search_iterations': 10}
    node = SimpleNamespace(protective_margin=lambda: 0.02,
                           get_parameter=lambda name: SimpleNamespace(value=params[name]))
    current = 0.128

    def evaluator(scale):
        clearance = current + scale * (0.10 - current)
        geometry = SimpleNamespace(
            minimum=SimpleNamespace(clearance=min(current, clearance)),
            sample_clearances=(current, clearance),
            first_collision_time=None,
        )
        node.last_horizon_clearance = geometry
        return ((geometry.minimum.clearance,), 0, 0.0, -1.0)

    evaluator(1.0)
    decision, _, _ = SafetySupervisorNode.decide_horizon(
        node, current, node.last_horizon_clearance, evaluator,
    )

    assert decision.state == 'WARNING'
    assert decision.speed_scale == 1.0


def test_predicted_protective_volume_intrusion_scales_the_jog():
    params = {'stop_distance': 0.05, 'warning_distance': 0.30,
              'predictive_stop_reserve_m': 0.005,
              'runtime_minimum_speed_scale': 0.05,
              'runtime_scale_search_iterations': 10}
    node = SimpleNamespace(protective_margin=lambda: 0.02,
                           get_parameter=lambda name: SimpleNamespace(value=params[name]))
    current = 0.128

    def evaluator(scale):
        clearance = current + scale * (0.03 - current)
        geometry = SimpleNamespace(
            minimum=SimpleNamespace(clearance=min(current, clearance)),
            sample_clearances=(current, clearance),
            first_collision_time=None,
        )
        node.last_horizon_clearance = geometry
        return ((geometry.minimum.clearance,), 0, 0.0, -1.0)

    evaluator(1.0)
    decision, _, selected = SafetySupervisorNode.decide_horizon(
        node, current, node.last_horizon_clearance, evaluator,
    )

    assert decision.state == 'REDUCTION'
    assert 0.0 < decision.speed_scale < 1.0
    assert selected.minimum.clearance >= 0.055
