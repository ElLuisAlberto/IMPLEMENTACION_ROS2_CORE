"""Admission and sampled-horizon regression tests for the generic supervisor."""

import math
import xml.etree.ElementTree as ET

import pytest

from thesis_multi_robot.supervisor import (
    classify_clearance, duration_seconds, evaluate_horizon,
    horizon_sample_count, reach_bound, remaining_duration, stamp_is_fresh,
    validate_target,
)


def test_horizon_detects_collision_between_safe_endpoints():
    result = evaluate_horizon((0.0,), (1.0,), 10.0,
                              lambda q, t: abs(q[0] - 0.5) - 0.05)
    assert result.state == 'STOP'
    assert not result.accepted
    assert result.clearance == pytest.approx(-0.05)


@pytest.mark.parametrize('clearance', [math.nan, math.inf, -math.inf, -0.1, 0.0])
def test_unknown_or_colliding_geometry_fails_closed(clearance):
    assert not classify_clearance(clearance).accepted


def test_reduction_is_rejected_until_retiming_is_implemented():
    result = classify_clearance(0.05)
    assert result.state == 'REDUCTION'
    assert not result.accepted
    assert classify_clearance(0.15).accepted
    assert classify_clearance(0.30).state == 'ALLOW'


def test_horizon_uses_both_endpoints_and_time_offsets():
    samples = []
    result = evaluate_horizon((0.0, 1.0), (0.2, 0.0), 2.0,
                             lambda q, t: samples.append((q, t)) or 1.0)
    assert result.accepted
    assert len(samples) == 21
    assert samples[0] == ((0.0, 1.0), 0.0)
    assert samples[-1] == ((0.2, 0.0), 2.0)


@pytest.mark.parametrize('stamp', [0, 400_000_000, 1_100_000_000])
def test_source_timestamp_rejection(stamp):
    assert not stamp_is_fresh(1_000_000_000, stamp)


def test_fresh_timestamp_accepted():
    assert stamp_is_fresh(1_000_000_000, 900_000_000)


def test_remaining_horizon_uses_ros_time_and_retains_final_monitoring():
    assert remaining_duration(2_000_000_000, 5_000_000_000) == 3.0
    assert remaining_duration(5_000_000_000, 5_000_000_000) == 0.1
    assert remaining_duration(8_000_000_000, 5_000_000_000) == 0.1
    assert remaining_duration(0, 100_000_000_000) == 30.0


@pytest.mark.parametrize('seconds,nanoseconds', [
    (-1, 0), (0, -1), (0, 1_000_000_000), (0, 1), (31, 0), (0.5, 0),
])
def test_invalid_ros_duration_fields_rejected(seconds, nanoseconds):
    with pytest.raises(ValueError):
        duration_seconds(seconds, nanoseconds)


def test_ros_duration_conversion():
    assert duration_seconds(0, 100_000_000) == 0.1
    assert duration_seconds(2, 500_000_000) == 2.5


def test_horizon_bound_keeps_minimum_for_stationary_robots():
    assert horizon_sample_count((0.,) * 5, (0.,) * 5, 30., 3.) == 20


def test_horizon_bound_accounts_for_each_joint_and_peer_motion():
    assert horizon_sample_count((0.,) * 6, (0.1,) * 6, 2., 2.) == 60
    # 1.0 m own motion + 0.4 m obstacle + 0.6 m peer = 2 m.
    assert horizon_sample_count((0.,), (0.5,), 2., 2.,
                                peer_motion=((3., 0.1),), obstacle_speed=0.2) == 100


def test_long_or_fast_horizon_exceeding_budget_fails_closed():
    with pytest.raises(ValueError, match='sampling budget'):
        horizon_sample_count((0.,) * 6, (4.5,) * 6, 30., 3.)


@pytest.mark.parametrize('reach,speed', [(math.nan, 0.), (2., math.nan),
                                       (math.inf, 0.), (0., 0.), (2., -1.)])
def test_malformed_sampling_bounds_fail_closed(reach, speed):
    with pytest.raises(ValueError):
        horizon_sample_count((0.,), (0.1,), 1., reach, obstacle_speed=speed)


def test_reach_bound_ignores_cell_placement_but_covers_local_capsule():
    root = ET.fromstring('''<robot><joint name="mount"><parent link="world"/>
        <origin xyz="100 100 100"/></joint><joint name="arm"><parent link="base"/>
        <origin xyz="0 0 2"/></joint></robot>''')
    capsules = [{'start': [0, 0, 0], 'end': [0, 0, 1], 'radius': 0.2}]
    assert reach_bound(root, capsules) == pytest.approx(3.2)


def test_target_velocity_and_urdf_limits_enforced():
    limits = [(-1.0, 1.0, 0.2)]
    validate_target((0.0,), (0.1,), limits, 1.0)
    with pytest.raises(ValueError):
        validate_target((0.0,), (0.2,), limits, 1.0)
    with pytest.raises(ValueError):
        validate_target((0.0,), (1.1,), limits, 10.0)
    with pytest.raises(ValueError):
        validate_target((0.0,), (math.nan,), limits, 1.0)


@pytest.mark.parametrize('start,target,duration', [
    ((), (), 1.0), ((0.0,), (0.0, 0.1), 1.0),
    ((math.nan,), (0.0,), 1.0), ((0.0,), (0.0,), math.inf),
    ((0.0,), (0.0,), 0.01),
])
def test_malformed_horizon_rejected(start, target, duration):
    with pytest.raises(ValueError):
        evaluate_horizon(start, target, duration, lambda q, t: 1.0)
