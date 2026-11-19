"""Read-only observations of the offline four-identity simulation.

This object creates subscriptions on the owning GUI node.  It creates no
command publishers, service clients, timers, executor or additional ROS node.
Its display state is deliberately not an authorization for movement.
"""

from dataclasses import dataclass
import math
import time


ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')


@dataclass(frozen=True)
class ArmObservation:
    """One complete six-joint sample and its local reception time."""

    positions: tuple
    received_at: float


class MultiArmObserver:
    """Subscribe to the existing hybrid scenario without affecting control."""

    def __init__(self, node, freshness_sec=0.75, monotonic=time.monotonic):
        from rclpy.qos import (
            HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from thesis_interfaces.msg import MultiArmSafetyStatus

        if not math.isfinite(freshness_sec) or freshness_sec <= 0:
            raise ValueError('freshness_sec must be finite and positive')
        self.freshness_sec = float(freshness_sec)
        self.monotonic = monotonic
        self.arms = {}
        self.invalid_arms = set()
        self.readiness = None
        self.readiness_at = None
        self.safety = None
        self.safety_at = None
        self.safety_lifetime = 0.0
        self.subscriptions = []
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        for robot_id in ROBOT_IDS:
            self.subscriptions.append(node.create_subscription(
                JointState, f'/{robot_id}/joint_states',
                lambda message, identity=robot_id: self.receive_arm(
                    identity, message
                ), state_qos,
            ))
        self.subscriptions.append(node.create_subscription(
            String, '/thesis/hybrid_system_readiness',
            self.receive_readiness, status_qos,
        ))
        self.subscriptions.append(node.create_subscription(
            MultiArmSafetyStatus, '/thesis/multiarm_safety_status',
            self.receive_safety, status_qos,
        ))

    def receive_arm(self, robot_id, message):
        """Reject incomplete, duplicate or non-finite joint samples."""
        names = tuple(message.name)
        positions = tuple(message.position)
        required = tuple(
            f'{robot_id}_jaco_joint_{index}' for index in range(1, 7)
        )
        if len(names) != len(positions) or len(set(names)) != len(names):
            self.invalid_arms.add(robot_id)
            return
        values = dict(zip(names, positions))
        if not all(name in values and math.isfinite(values[name])
                   for name in required):
            self.invalid_arms.add(robot_id)
            return
        self.arms[robot_id] = ArmObservation(
            tuple(float(values[name]) for name in required), self.monotonic()
        )
        self.invalid_arms.discard(robot_id)

    def receive_readiness(self, message):
        self.readiness = str(message.data)
        self.readiness_at = self.monotonic()

    def receive_safety(self, message):
        self.safety = message
        self.safety_at = self.monotonic()
        duration = message.valid_for
        seconds = float(duration.sec) + float(duration.nanosec) * 1.0e-9
        self.safety_lifetime = (
            min(seconds, 0.20) if math.isfinite(seconds) and seconds > 0
            and 0 <= duration.nanosec < 1_000_000_000 else 0.0
        )

    def arm_state(self, robot_id):
        """Return a status and sample; stale samples never appear live."""
        sample = self.arms.get(robot_id)
        if robot_id in self.invalid_arms:
            return 'Dato inválido', sample
        if sample is None:
            return 'Sin datos', None
        if self.monotonic() - sample.received_at > self.freshness_sec:
            return 'Datos vencidos', sample
        return 'Estado reciente', sample

    def readiness_text(self):
        if self.readiness_at is None:
            return 'Disponibilidad: esperando el escenario híbrido.'
        if self.monotonic() - self.readiness_at > 1.5:
            return 'Disponibilidad: monitor sin dato reciente.'
        return 'Disponibilidad informada: ' + self.readiness

    def safety_state(self):
        """Display only a recently received supported coordinator result."""
        message = self.safety
        if message is None:
            return 'SIN DATOS', 'Esperando evaluación global.', None
        if self.monotonic() - self.safety_at >= self.safety_lifetime:
            return 'VENCIDO', 'Evaluación global sin dato reciente.', None
        if not message.valid or message.state not in ('ALLOW', 'STOP'):
            return 'NO VÁLIDO', str(message.reason), None
        clearance = float(message.minimum_clearance)
        detail = str(message.reason)
        if message.first_robot_id and message.second_robot_id:
            detail += (
                f' · Par: {message.first_robot_id} / {message.second_robot_id}'
            )
        return (
            str(message.state), detail,
            clearance if math.isfinite(clearance) else None,
        )
