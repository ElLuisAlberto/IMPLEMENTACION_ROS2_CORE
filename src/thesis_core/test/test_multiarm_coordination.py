"""Tests for the deterministic four-arm safety coordination kernel."""

import pytest

from thesis_core.multiarm_coordination import (
    CapsuleOccupancy,
    GeometryExpectation,
    RobotOccupancySnapshot,
    SphereOccupancy,
    capsule_capsule_clearance,
    capsule_sphere_clearance,
    evaluate_four_arm_safety,
)


def capsule(name, start, end, radius=0.10):
    """Build one compact test capsule."""
    return CapsuleOccupancy(name, start, end, radius)


def snapshot(robot_id, x_value, stamp_sec=10.0, ttl_sec=1.0):
    """Build a valid one-capsule robot occupancy in the shared world frame."""
    return RobotOccupancySnapshot(
        robot_id=robot_id,
        profile_id='jaco_gripper_v1',
        geometry_version='1.0.0',
        asset_hash='sha256:approved',
        reference_frame='world',
        stamp_sec=stamp_sec,
        ttl_sec=ttl_sec,
        capsules=(capsule('body', (x_value, 0.0, 0.0),
                          (x_value, 1.0, 0.0)),),
    )


def approved_expectations():
    """Return the configured identity for four test robots."""
    return {
        f'arm_{index:02d}': GeometryExpectation(
            'jaco_gripper_v1', '1.0.0', 'sha256:approved'
        )
        for index in range(1, 5)
    }


def four_snapshots():
    """Return mutually clear snapshots separated by one metre."""
    return tuple(snapshot(f'arm_{index:02d}', float(index))
                 for index in range(1, 5))


def test_capsule_capsule_clearance_uses_finite_axes():
    first = capsule('first', (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.20)
    second = capsule('second', (0.0, 2.0, 0.0), (1.0, 2.0, 0.0), 0.30)
    assert capsule_capsule_clearance(first, second) == pytest.approx(1.50)


def test_capsule_sphere_clearance_accepts_a_point_capsule():
    point_capsule = capsule('point', (0.0, 0.0, 0.0),
                            (0.0, 0.0, 0.0), 0.10)
    sphere = SphereOccupancy('payload', (1.0, 0.0, 0.0), 0.20)
    assert capsule_sphere_clearance(
        point_capsule, sphere
    ) == pytest.approx(0.70)


def test_four_clear_robots_allow_and_evaluate_all_six_pairs():
    decision = evaluate_four_arm_safety(
        four_snapshots(),
        now_sec=10.5,
        clearance_threshold=0.20,
        expected_geometry=approved_expectations(),
    )
    assert decision.state == 'ALLOW'
    assert decision.reason_code == 'CLEAR'
    assert len(decision.evaluated_robot_pairs) == 6
    assert len(decision.pair_clearances) == 6
    assert decision.minimum_clearance == pytest.approx(0.80)


def test_interrobot_overlap_stops_and_identifies_limiting_pair():
    values = list(four_snapshots())
    values[1] = snapshot('arm_02', 1.15)
    decision = evaluate_four_arm_safety(values, 10.5, 0.05)
    assert decision.state == 'STOP'
    assert decision.reason_code == 'INTERROBOT_CLEARANCE_STOP'
    assert decision.limiting.first_robot_id == 'arm_01'
    assert decision.limiting.second_robot_id == 'arm_02'
    assert decision.minimum_clearance < 0.0


def test_stale_snapshot_fails_closed_without_pair_evaluation():
    values = list(four_snapshots())
    values[2] = snapshot('arm_03', 3.0, stamp_sec=8.0, ttl_sec=1.0)
    decision = evaluate_four_arm_safety(values, 10.5, 0.05)
    assert decision.state == 'STOP'
    assert decision.reason_code == 'SNAPSHOT_STALE'
    assert not decision.evaluated_robot_pairs


def test_future_snapshot_fails_closed():
    values = list(four_snapshots())
    values[0] = snapshot('arm_01', 1.0, stamp_sec=11.0)
    decision = evaluate_four_arm_safety(values, 10.5, 0.05)
    assert decision.reason_code == 'SNAPSHOT_TIMESTAMP_FUTURE'


def test_expected_geometry_mismatch_fails_closed():
    expected = approved_expectations()
    expected['arm_04'] = GeometryExpectation(
        'unexpected', '1.0.0', 'sha256:approved'
    )
    decision = evaluate_four_arm_safety(
        four_snapshots(), 10.5, 0.05, expected
    )
    assert decision.state == 'STOP'
    assert decision.reason_code == 'GEOMETRY_PROFILE_MISMATCH'


def test_reference_frame_mismatch_fails_closed():
    values = list(four_snapshots())
    values[3] = RobotOccupancySnapshot(
        robot_id='arm_04',
        profile_id='jaco_gripper_v1',
        geometry_version='1.0.0',
        asset_hash='sha256:approved',
        reference_frame='other_world',
        stamp_sec=10.0,
        ttl_sec=1.0,
        capsules=(capsule('body', (4.0, 0.0, 0.0), (4.0, 1.0, 0.0)),),
    )
    decision = evaluate_four_arm_safety(values, 10.5, 0.05)
    assert decision.state == 'STOP'
    assert decision.reason_code == 'REFERENCE_FRAME_MISMATCH'


def test_duplicate_or_incomplete_robot_set_fails_closed():
    values = list(four_snapshots())
    values[3] = snapshot('arm_03', 4.0)
    duplicate = evaluate_four_arm_safety(values, 10.5, 0.05)
    incomplete = evaluate_four_arm_safety(four_snapshots()[:3], 10.5, 0.05)
    assert duplicate.reason_code == 'ROBOT_ID_DUPLICATE'
    assert incomplete.reason_code == 'FOUR_ROBOT_SET_REQUIRED'


def test_snapshot_rejects_duplicate_primitive_names():
    with pytest.raises(ValueError, match='unique'):
        RobotOccupancySnapshot(
            'arm_01', 'profile', '1', 'hash', 'world', 1.0, 1.0,
            (capsule('same', (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),),
            (SphereOccupancy('same', (1.0, 1.0, 1.0), 0.1),),
        )
