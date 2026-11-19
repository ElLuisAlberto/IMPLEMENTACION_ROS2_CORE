"""Tests for synchronized runtime admission before geometry evaluation."""

from thesis_core.multiarm_coordination import (
    CapsuleOccupancy,
    GeometryExpectation,
    RobotOccupancySnapshot,
)
from thesis_core.multiarm_runtime import RuntimeOccupancy, evaluate_runtime_batch


IDS = tuple(f'arm_{index:02d}' for index in range(1, 5))


def record(robot_id, stamp=10.0, receive=20.0, session='session-a'):
    occupancy = RobotOccupancySnapshot(
        robot_id=robot_id,
        profile_id='profile',
        geometry_version='1',
        asset_hash='hash',
        reference_frame='world',
        stamp_sec=stamp,
        ttl_sec=1.0,
        capsules=(CapsuleOccupancy(
            'body', (float(robot_id[-1]), 0.0, 0.0),
            (float(robot_id[-1]), 0.5, 0.0), 0.1,
        ),),
    )
    source = 'synthetic_state' if robot_id == 'arm_01' else 'gazebo'
    return RuntimeOccupancy(occupancy, session, source, receive)


def expectations():
    return {
        robot_id: GeometryExpectation('profile', '1', 'hash')
        for robot_id in IDS
    }


def sources():
    return {
        robot_id: ('synthetic_state' if robot_id == 'arm_01' else 'gazebo')
        for robot_id in IDS
    }


def evaluate(values, **changes):
    arguments = dict(
        now_sec=10.2,
        monotonic_now_sec=20.1,
        clearance_threshold=0.05,
        session_id='session-a',
        expected_geometry=expectations(),
        expected_sources=sources(),
        maximum_receive_age_sec=0.5,
        maximum_snapshot_skew_sec=0.1,
    )
    arguments.update(changes)
    return evaluate_runtime_batch(values, **arguments)


def test_valid_atomic_batch_evaluates_six_pairs():
    result = evaluate(tuple(record(robot_id) for robot_id in IDS))
    assert result.decision.state == 'ALLOW'
    assert len(result.decision.evaluated_robot_pairs) == 6


def test_session_source_and_expected_geometry_are_mandatory():
    values = [record(robot_id) for robot_id in IDS]
    values[0] = record('arm_01', session='old')
    assert evaluate(values).decision.reason_code == 'SESSION_MISMATCH'
    assert evaluate(
        tuple(record(robot_id) for robot_id in IDS),
        expected_geometry={},
    ).decision.reason_code == 'EXPECTED_GEOMETRY_INCOMPLETE'
    wrong_sources = sources()
    wrong_sources['arm_01'] = 'gazebo'
    assert evaluate(
        tuple(record(robot_id) for robot_id in IDS),
        expected_sources=wrong_sources,
    ).decision.reason_code == 'SOURCE_KIND_MISMATCH'


def test_receive_timeout_and_snapshot_skew_stop_before_geometry():
    values = [record(robot_id) for robot_id in IDS]
    values[2] = record('arm_03', receive=19.0)
    assert evaluate(values).decision.reason_code == 'SNAPSHOT_RECEIVE_STALE'
    values = [record(robot_id) for robot_id in IDS]
    values[3] = record('arm_04', stamp=10.2)
    assert evaluate(values).decision.reason_code == 'SNAPSHOT_SKEW_EXCEEDED'


def test_physical_or_duplicate_identity_cannot_enter_offline_batch():
    values = [record(robot_id) for robot_id in IDS]
    values[3] = record('arm_03')
    assert evaluate(values).decision.reason_code == 'CANONICAL_ROBOT_SET_REQUIRED'
    physical = list(record(robot_id) for robot_id in IDS)
    physical[0] = RuntimeOccupancy(
        physical[0].occupancy, 'session-a', 'physical', 20.0
    )
    assert evaluate(physical).decision.reason_code == 'SOURCE_KIND_MISMATCH'
