"""Exact reuse avoids redundant FK without changing moving-obstacle safety."""
from types import SimpleNamespace

import pytest

from thesis_core.horizon_clearance import evaluate_joint_horizon
from thesis_core.safety_supervisor_node import memoize_horizon_evaluator


def test_stationary_horizon_reuses_fk_but_evaluates_obstacle_motion():
    calls = []

    def capsules(joints):
        calls.append(joints)
        return (((0., 0., 0.), (0., 0., 1.)),)

    result = evaluate_joint_horizon(
        [(0.,)] * 3, (0., 0.5, 1.), (2., 0., 0.5), (-1., 0., 0.),
        0.1, capsule_radii=(0.1,), segment_names=('arm',),
        capsule_function=capsules, joint_count=1,
    )
    assert len(calls) == 1
    assert result.sample_clearances == pytest.approx((1.8, 1.3, 0.8))


def test_exact_scale_cache_restores_the_correct_witness():
    node = SimpleNamespace(last_horizon_clearance=None)
    calls = []

    def evaluate(scale):
        calls.append(scale)
        node.last_horizon_clearance = f'geometry-{scale}'
        return scale * 2

    cached = memoize_horizon_evaluator(node, evaluate)
    assert cached(1.0) == 2.0
    assert cached(0.5) == 1.0
    assert cached(1.0) == 2.0
    assert node.last_horizon_clearance == 'geometry-1.0'
    assert calls == [1.0, 0.5]
