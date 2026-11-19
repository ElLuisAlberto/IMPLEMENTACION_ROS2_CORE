"""A received STOP sentinel must not make simulation availability READY."""

from types import MethodType, SimpleNamespace

import pytest
from thesis_interfaces.msg import ProximityStatus

from thesis_simulation.system_readiness_node import SystemReadinessNode


def node():
    value = SimpleNamespace(
        last_received={key: 10.0 for key in (
            '/clock', '/joint_states', '/thesis/proximity_status',
            '/thesis/obstacle_source',
        )},
        timeout_sec=0.75,
        proximity_issue='esperando geometría de proximidad válida',
    )
    value._mark_received = lambda key: value.last_received.__setitem__(key, 10.0)
    value.proximity_callback = MethodType(SystemReadinessNode.proximity_callback, value)
    return value


def message(state='ALLOW', segment='upper_arm_to_forearm', valid=True):
    value = ProximityStatus()
    value.state = state
    value.reference_frame = 'world'
    value.limiting_segment = segment
    value.geometry_valid = valid
    value.minimum_clearance = 0.0 if state == 'STOP' else 0.4
    value.obstacle_radius = 0.12
    return value


@pytest.mark.parametrize('reason', ('TF_UNAVAILABLE', 'OBSTACLE_UNAVAILABLE'))
def test_recent_unmeasured_stop_is_not_ready(reason):
    value = node()
    value.proximity_callback(message('STOP', reason, False))
    assert reason in SystemReadinessNode._freshness_status(value, 10.1)[0]


@pytest.mark.parametrize('state', ('ALLOW', 'WARNING', 'REDUCTION', 'STOP'))
def test_valid_geometric_state_is_available_even_when_supervisor_must_stop(state):
    value = node()
    value.proximity_callback(message(state))
    assert SystemReadinessNode._freshness_status(value, 10.1) == []


def test_valid_recovery_clears_missing_geometry_but_not_freshness_requirements():
    value = node()
    value.proximity_callback(message('STOP', 'TF_UNAVAILABLE', False))
    value.proximity_callback(message())
    assert SystemReadinessNode._freshness_status(value, 10.1) == []
    assert SystemReadinessNode._freshness_status(value, 11.0)
