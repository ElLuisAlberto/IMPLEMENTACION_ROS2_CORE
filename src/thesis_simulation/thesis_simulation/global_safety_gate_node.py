"""Single-authority command relay guarded by global multi-arm safety."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from thesis_core.multiarm_runtime import CANONICAL_ROBOT_IDS
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import (
    ExecutionControl,
    JointCommand,
    MultiArmSafetyStatus,
)
from thesis_simulation.global_safety_gate import (
    GlobalSafetyEvidence,
    GlobalSafetyGate,
)


def _seconds(value):
    """Convert a ROS-like Time or Duration message to seconds."""
    return float(value.sec) + float(value.nanosec) * 1.0e-9


class GlobalSafetyGateNode(Node):
    """Relay candidates to one adapter only while global ALLOW is fresh."""

    def __init__(self):
        super().__init__('global_safety_gate')
        self.declare_parameter('robot_id', 'arm_02')
        self.declare_parameter('expected_session_id', 'hybrid-three-sim')
        self.declare_parameter('evaluation_rate_hz', 25.0)
        self.declare_parameter('maximum_status_receive_age_sec', 0.20)
        robot_id = str(self.get_parameter('robot_id').value)
        self.declare_parameter(
            'global_status_topic', '/thesis/multiarm_safety_status'
        )
        self.declare_parameter(
            'candidate_command_topic',
            f'/{robot_id}/thesis/candidate_supervised_command',
        )
        self.declare_parameter(
            'supervised_command_topic',
            f'/{robot_id}/thesis/supervised_command',
        )
        self.declare_parameter(
            'candidate_jog_command_topic',
            f'/{robot_id}/thesis/candidate_supervised_jog_command',
        )
        self.declare_parameter(
            'supervised_jog_command_topic',
            f'/{robot_id}/thesis/supervised_jog_command',
        )
        self.declare_parameter(
            'execution_control_topic',
            f'/{robot_id}/thesis/execution_control',
        )

        if robot_id not in CANONICAL_ROBOT_IDS[1:]:
            raise ValueError(
                'global gate may drive only arm_02 through arm_04'
            )
        self.robot_id = robot_id
        rate = float(self.get_parameter('evaluation_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('evaluation_rate_hz must be finite and positive')
        self.gate = GlobalSafetyGate(
            str(self.get_parameter('expected_session_id').value),
            float(
                self.get_parameter('maximum_status_receive_age_sec').value
            ),
        )
        topic_names = tuple(
            str(self.get_parameter(name).value)
            for name in (
                'candidate_command_topic',
                'supervised_command_topic',
                'candidate_jog_command_topic',
                'supervised_jog_command_topic',
                'execution_control_topic',
            )
        )
        if (
            any(not topic for topic in topic_names)
            or len(set(topic_names)) != 5
        ):
            raise ValueError('gate input and output topics must be unique')

        self.command_publisher = self.create_publisher(
            JointCommand, topic_names[1], 10
        )
        jog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.jog_publisher = self.create_publisher(
            JointCommand, topic_names[3], jog_qos
        )
        self.control_publisher = self.create_publisher(
            ExecutionControl, topic_names[4], 10
        )
        self.status_subscription = self.create_subscription(
            MultiArmSafetyStatus,
            str(self.get_parameter('global_status_topic').value),
            self.status_callback,
            10,
        )
        self.command_subscription = self.create_subscription(
            JointCommand, topic_names[0], self.command_callback, 10
        )
        self.jog_subscription = self.create_subscription(
            JointCommand, topic_names[2], self.jog_callback, jog_qos
        )
        self._active_command_id = None
        self._last_clearance = 0.0
        self.timer = self.create_timer(
            1.0 / rate,
            self.watchdog_callback,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f'Global gate ready for {robot_id}; candidate topics are the '
            'only configured path to simulation adapter outputs'
        )

    def _clocks(self):
        return (
            self.get_clock().now().nanoseconds * 1.0e-9,
            time.monotonic(),
        )

    def status_callback(self, message):
        """Admit only newer status from the configured simulation session."""
        try:
            evidence = GlobalSafetyEvidence(
                session_id=message.session_id,
                sequence=message.decision_sequence,
                state=message.state,
                valid=message.valid,
                stamp_sec=_seconds(message.stamp),
                ttl_sec=_seconds(message.valid_for),
            )
            accepted = self.gate.observe(evidence, time.monotonic())
        except (TypeError, ValueError) as exception:
            self.gate.reject('STATUS_MESSAGE_INVALID')
            accepted = False
            self.get_logger().warning(f'Invalid global status: {exception}')
        clearance = float(message.minimum_clearance)
        self._last_clearance = clearance if math.isfinite(clearance) else 0.0
        decision = self.gate.decide(*self._clocks())
        if not accepted or not decision.allowed:
            self._publish_stop(decision.reason_code)

    def _relay_or_stop(self, message, publisher):
        """Relay one candidate atomically or emit a matching STOP control."""
        self._active_command_id = str(message.command_id)
        decision = self.gate.decide(*self._clocks())
        if decision.allowed:
            publisher.publish(message)
            return
        self._publish_stop(decision.reason_code)

    def command_callback(self, message):
        self._relay_or_stop(message, self.command_publisher)

    def jog_callback(self, message):
        self._relay_or_stop(message, self.jog_publisher)

    def watchdog_callback(self):
        """Stop an already-relayed command when global evidence goes stale."""
        decision = self.gate.decide(*self._clocks())
        if not decision.allowed:
            self._publish_stop(decision.reason_code)

    def _publish_stop(self, reason_code):
        if self._active_command_id is None:
            return
        message = ExecutionControl()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = self._active_command_id
        message.state = 'STOP'
        message.speed_scale = 0.0
        message.minimum_clearance = self._last_clearance
        message.limiting_segment = 'multiarm_global_gate'
        message.protective_margin = 0.0
        message.time_to_protective_volume = -1.0
        message.time_to_collision = -1.0
        message.supervised_time_to_protective_volume = -1.0
        message.supervised_time_to_collision = -1.0
        message.minimum_time_from_now = -1.0
        message.reason_code = str(reason_code)
        message.reason = (
            f'{self.robot_id} stopped by global multi-arm gate: '
            f'{reason_code}'
        )
        message.requested_state = 'STOP'
        message.requested_speed_scale = 0.0
        message.requested_reason_code = str(reason_code)
        message.transition = True
        message.transition_reason = 'GLOBAL_MULTIARM_INTERLOCK'
        message.current_clearance = self._last_clearance
        message.nominal_clearance = self._last_clearance
        message.supervised_clearance = self._last_clearance
        self.control_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    spin_node(GlobalSafetyGateNode())
