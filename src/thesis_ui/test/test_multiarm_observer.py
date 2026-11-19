"""Passive dashboard contract: isolation, malformed data and freshness."""

import math
from types import SimpleNamespace

from sensor_msgs.msg import JointState
from thesis_interfaces.msg import MultiArmSafetyStatus

from thesis_ui.multiarm_observer import MultiArmObserver, ROBOT_IDS


class SubscriptionOnlyNode:
    """Expose no publisher/client API so accidental writers fail immediately."""

    def __init__(self):
        self.topics = []

    def create_subscription(self, message_type, topic, callback, qos):
        self.topics.append(topic)
        return callback


def observer():
    clock = [10.0]
    node = SubscriptionOnlyNode()
    return MultiArmObserver(node, monotonic=lambda: clock[0]), clock, node


def sample(robot_id, positions=None):
    message = JointState()
    message.name = [f'{robot_id}_jaco_joint_{index}' for index in range(1, 7)]
    message.position = [0.0] * 6 if positions is None else positions
    return message


def test_observer_has_only_six_subscription_endpoints():
    state, _, node = observer()
    assert set(node.topics) == {
        *(f'/{robot_id}/joint_states' for robot_id in ROBOT_IDS),
        '/thesis/hybrid_system_readiness', '/thesis/multiarm_safety_status',
    }
    assert len(state.subscriptions) == 6


def test_samples_are_kept_per_identity_and_expire():
    state, clock, _ = observer()
    for index, robot_id in enumerate(ROBOT_IDS):
        state.receive_arm(robot_id, sample(robot_id, [float(index)] * 6))
    for index, robot_id in enumerate(ROBOT_IDS):
        status, values = state.arm_state(robot_id)
        assert status == 'Estado reciente'
        assert values.positions == (float(index),) * 6
    clock[0] += 0.8
    assert all(state.arm_state(robot_id)[0] == 'Datos vencidos'
               for robot_id in ROBOT_IDS)


def test_wrong_identity_duplicate_missing_and_nonfinite_samples_are_invalid():
    state, _, _ = observer()
    state.receive_arm('arm_02', sample('arm_03'))
    assert state.arm_state('arm_02')[0] == 'Dato inválido'
    invalid = sample('arm_02')
    invalid.name[1] = invalid.name[0]
    state.receive_arm('arm_02', invalid)
    assert state.arm_state('arm_02')[0] == 'Dato inválido'
    invalid = sample('arm_02', [math.nan] + [0.0] * 5)
    state.receive_arm('arm_02', invalid)
    assert state.arm_state('arm_02')[0] == 'Dato inválido'
    state.receive_arm('arm_02', sample('arm_02'))
    assert state.arm_state('arm_02')[0] == 'Estado reciente'


def test_global_allow_disappears_when_ttl_expires_or_status_is_invalid():
    state, clock, _ = observer()
    message = MultiArmSafetyStatus()
    message.valid = True
    message.state = 'ALLOW'
    message.valid_for.nanosec = 120_000_000
    message.minimum_clearance = 0.4
    state.receive_safety(message)
    assert state.safety_state()[0] == 'ALLOW'
    clock[0] += 0.121
    assert state.safety_state() == (
        'VENCIDO', 'Evaluación global sin dato reciente.', None,
    )
    message.valid = False
    message.reason = 'GEOMETRY_MISSING'
    state.receive_safety(message)
    assert state.safety_state()[0] == 'NO VÁLIDO'
    message.valid_for.nanosec = 0
    state.receive_safety(message)
    assert state.safety_state()[0] == 'VENCIDO'


def test_readiness_never_survives_missing_recent_observations():
    state, clock, _ = observer()
    state.receive_readiness(SimpleNamespace(data='READY'))
    assert state.readiness_text().endswith('READY')
    clock[0] += 1.6
    assert 'sin dato reciente' in state.readiness_text()
