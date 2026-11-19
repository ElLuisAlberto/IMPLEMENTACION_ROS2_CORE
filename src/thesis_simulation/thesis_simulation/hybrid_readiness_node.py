"""Publish aggregate readiness for three Gazebo arms and one synthetic arm."""

from __future__ import annotations

import math
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import MultiArmSafetyStatus
from thesis_simulation.geometry_snapshot_source_contract import (
    complete_joint_sample,
)
from thesis_simulation.hybrid_readiness_contract import (
    CANONICAL_ROBOT_IDS,
    GAZEBO_ROBOT_IDS,
    evaluate_hybrid_readiness,
)


class HybridReadinessNode(Node):
    """Require live evidence without treating geometric STOP as a fault."""

    def __init__(self):
        super().__init__('hybrid_readiness')
        self.declare_parameter('readiness_rate_hz', 2.0)
        self.declare_parameter('freshness_timeout_sec', 0.75)
        self.declare_parameter('controller_request_timeout_sec', 1.0)
        self.declare_parameter('simulation_output_enabled', False)
        self.declare_parameter('expected_session_id', 'hybrid-three-sim')
        rate = float(self.get_parameter('readiness_rate_hz').value)
        self.freshness_timeout = float(
            self.get_parameter('freshness_timeout_sec').value
        )
        self.request_timeout = float(
            self.get_parameter('controller_request_timeout_sec').value
        )
        self.output_enabled = bool(
            self.get_parameter('simulation_output_enabled').value
        )
        self.expected_session_id = str(
            self.get_parameter('expected_session_id').value
        )
        if (
            not self.expected_session_id
            or self.expected_session_id != self.expected_session_id.strip()
        ):
            raise ValueError('expected_session_id must be a trimmed string')
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (rate, self.freshness_timeout, self.request_timeout)
        ):
            raise ValueError('readiness rates and timeouts must be positive')

        self.received = {}
        self.global_status_valid = False
        self.global_reason = 'GLOBAL_STATUS_MISSING'
        self.controller_states = {}
        self.controller_futures = {robot_id: None for robot_id in GAZEBO_ROBOT_IDS}
        self.controller_started = {robot_id: None for robot_id in GAZEBO_ROBOT_IDS}
        self.controller_clients = {
            robot_id: self.create_client(
                ListControllers,
                f'/{robot_id}/{robot_id}_controller_manager/list_controllers',
            )
            for robot_id in GAZEBO_ROBOT_IDS
        }

        self.create_subscription(
            ClockMessage, '/clock', self._clock_callback, 10
        )
        for robot_id in CANONICAL_ROBOT_IDS:
            self.create_subscription(
                JointState,
                f'/{robot_id}/joint_states',
                lambda message, identity=robot_id: self._joint_callback(
                    identity, message
                ),
                10,
            )
        self.create_subscription(
            MultiArmSafetyStatus,
            '/thesis/multiarm_safety_status',
            self._global_status_callback,
            10,
        )
        self.ready_publisher = self.create_publisher(
            Bool, '/thesis/hybrid_system_ready', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/thesis/hybrid_system_readiness', 10
        )
        self.last_status = None
        self.timer = self.create_timer(
            1.0 / rate,
            self.evaluate,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _clock_callback(self, message):
        if message.clock.sec > 0 or message.clock.nanosec > 0:
            self.received['clock'] = time.monotonic()

    def _joint_callback(self, robot_id, message):
        required = tuple(
            f'{robot_id}_jaco_joint_{index}' for index in range(1, 7)
        )
        if complete_joint_sample(message.name, message.position, required):
            self.received[f'{robot_id}_joint_state'] = time.monotonic()

    def _global_status_callback(self, message):
        self.received['global_status'] = time.monotonic()
        session_matches = str(message.session_id) == self.expected_session_id
        state_supported = str(message.state) in ('ALLOW', 'STOP')
        self.global_status_valid = bool(
            message.valid and session_matches and state_supported
        )
        if not session_matches:
            self.global_reason = 'GLOBAL_STATUS_SESSION_MISMATCH'
        elif not state_supported:
            self.global_reason = 'GLOBAL_STATUS_STATE_INVALID'
        else:
            self.global_reason = str(message.reason_code)

    def _poll_controllers(self):
        now = time.monotonic()
        for robot_id in GAZEBO_ROBOT_IDS:
            future = self.controller_futures[robot_id]
            if future is not None and future.done():
                try:
                    response = future.result()
                    self.controller_states[robot_id] = {
                        item.name: item.state.lower()
                        for item in response.controller
                    }
                    self.received[
                        f'{robot_id}_controller_state'
                    ] = now
                except Exception:  # Runtime service failure withdraws readiness.
                    self.controller_states.pop(robot_id, None)
                    self.received.pop(
                        f'{robot_id}_controller_state', None
                    )
                self.controller_futures[robot_id] = None
                self.controller_started[robot_id] = None
            elif future is not None:
                started = self.controller_started[robot_id]
                if now - started > self.request_timeout:
                    future.cancel()
                    self.controller_futures[robot_id] = None
                    self.controller_started[robot_id] = None
                    self.controller_states.pop(robot_id, None)
                    self.received.pop(
                        f'{robot_id}_controller_state', None
                    )
            client = self.controller_clients[robot_id]
            if not client.service_is_ready():
                self.controller_states.pop(robot_id, None)
                self.received.pop(f'{robot_id}_controller_state', None)
            if (
                self.controller_futures[robot_id] is None
                and client.service_is_ready()
            ):
                self.controller_futures[robot_id] = client.call_async(
                    ListControllers.Request()
                )
                self.controller_started[robot_id] = now

    def evaluate(self):
        """Publish a monotonic readiness decision for the complete topology."""
        self._poll_controllers()
        ready, reasons = evaluate_hybrid_readiness(
            self.received,
            self.controller_states,
            monotonic_now_sec=time.monotonic(),
            freshness_timeout_sec=self.freshness_timeout,
            global_status_valid=self.global_status_valid,
            simulation_output_enabled=self.output_enabled,
        )
        if not self.global_status_valid and self.global_reason:
            reasons = tuple((*reasons, self.global_reason))
        status = 'READY' if ready else 'WAITING:' + ','.join(reasons)
        ready_message = Bool()
        ready_message.data = ready
        status_message = String()
        status_message.data = status
        self.ready_publisher.publish(ready_message)
        self.status_publisher.publish(status_message)
        if status != self.last_status:
            if ready:
                self.get_logger().info(status)
            else:
                self.get_logger().warning(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    spin_node(HybridReadinessNode())
