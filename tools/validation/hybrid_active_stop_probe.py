#!/usr/bin/env python3
"""
Probe deterministic fail-safe cancellation in the three-arm simulation.

The probe waits for continuous READY and fresh global ALLOW evidence, sends
one small candidate trajectory to arm_02, observes real simulated movement,
and only then terminates the exact coordinator process through a pidfd.  It
never addresses hardware topics and never republishes the command.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import os
from pathlib import Path
import select
import signal
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from thesis_interfaces.msg import (
    ExecutionTrajectory,
    JointCommand,
    MultiArmSafetyStatus,
)


ROBOT_IDS = ('arm_02', 'arm_03', 'arm_04')
STABLE_EVIDENCE_SEC = 1.5
READY_RECEIVE_MAX_AGE_SEC = 0.75
SAFETY_RECEIVE_MAX_AGE_SEC = 0.18
JOINT_RECEIVE_MAX_AGE_SEC = 0.25
JOINT_RECEIVE_MAX_SKEW_SEC = 0.05
MOTION_THRESHOLD_RAD = 0.003
ISOLATION_TOLERANCE_RAD = 0.0001
POST_CANCEL_DRIFT_TOLERANCE_RAD = 0.003
CANCEL_LATENCY_BUDGET_SEC = 1.0


def _seconds(message):
    return float(message.sec) + float(message.nanosec) * 1.0e-9


def _ordered_positions(message: JointState, robot_id: str):
    """Return six positions in the configured controller joint order."""
    values = dict(zip(message.name, message.position))
    names = tuple(f'{robot_id}_jaco_joint_{index}' for index in range(1, 7))
    if not all(name in values for name in names):
        return None
    positions = tuple(float(values[name]) for name in names)
    if not all(math.isfinite(value) for value in positions):
        return None
    return names, positions


def _maximum_change(first, second):
    if first is None or second is None:
        return None
    return max(abs(a - b) for a, b in zip(first, second))


def open_verified_coordinator_pidfd(pid: int):
    """Open a stable process reference and verify the installed entry point."""
    pidfd = os.pidfd_open(pid)
    expected = (
        Path(__file__).resolve().parents[2]
        / 'install/thesis_core/lib/thesis_core/multiarm_coordinator'
    ).resolve()
    try:
        tokens = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        paths = {
            Path(token.decode()).resolve()
            for token in tokens
            if token and token.startswith(b'/')
        }
        if expected not in paths:
            raise ValueError(
                f'PID {pid} does not execute the expected entry point '
                f'{expected}'
            )
    except Exception:
        os.close(pidfd)
        raise
    return pidfd, str(expected)


class ActiveStopProbe(Node):
    """Run one fail-safe movement/cancellation trial in Gazebo only."""

    def __init__(
        self,
        delta_rad: float,
        duration_sec: float,
        timeout_sec: float,
        coordinator_pid: int,
        coordinator_pidfd: int,
        coordinator_entrypoint: str,
    ):
        super().__init__(
            'hybrid_active_stop_probe',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        self.delta_rad = delta_rad
        self.duration_sec = duration_sec
        self.deadline = time.monotonic() + timeout_sec
        self.coordinator_pid = coordinator_pid
        self.coordinator_pidfd = coordinator_pidfd
        self.coordinator_entrypoint = coordinator_entrypoint
        self.pidfd_poll = select.poll()
        self.pidfd_poll.register(coordinator_pidfd, select.POLLIN)
        self.command_id = f'active-stop-test-{uuid.uuid4().hex}'

        self.stage = 'WAITING'
        self.done = False
        self.exit_code = 1
        self.failure_reason = None
        self.ready = False
        self.ready_since = None
        self.last_ready_receive = None
        self.allow = False
        self.allow_since = None
        self.last_safety_receive = None
        self.safety_session_id = None
        self.last_safety_sequence = None
        self.positions = {}
        self.position_receive = {}
        self.baseline = {}
        self.max_excursion = {robot_id: 0.0 for robot_id in ROBOT_IDS}
        self.status_history = []
        self.final_status = None
        self.send_monotonic = None
        self.pending_monotonic = None
        self.accepted_monotonic = None
        self.fault_monotonic = None
        self.canceled_monotonic = None
        self.cancel_positions = None
        self.post_cancel_drift_rad = None
        self.coordinator_exit_observed = False

        self.publisher = self.create_publisher(
            JointCommand,
            '/arm_02/thesis/candidate_supervised_command',
            10,
        )
        self.create_subscription(
            Bool,
            '/thesis/hybrid_system_ready',
            self._ready_callback,
            10,
        )
        self.create_subscription(
            MultiArmSafetyStatus,
            '/thesis/multiarm_safety_status',
            self._safety_callback,
            10,
        )
        self.create_subscription(
            ExecutionTrajectory,
            '/arm_02/thesis/execution_trajectory',
            self._execution_callback,
            10,
        )
        for robot_id in ROBOT_IDS:
            self.create_subscription(
                JointState,
                f'/{robot_id}/joint_states',
                lambda message, item=robot_id: self._joint_callback(
                    item, message
                ),
                10,
            )
        self.create_timer(
            0.05,
            self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _ready_callback(self, message):
        now = time.monotonic()
        value = bool(message.data)
        self.last_ready_receive = now
        if value and not self.ready:
            self.ready_since = now
        elif not value:
            self.ready_since = None
        self.ready = value

    def _safety_callback(self, message):
        now = time.monotonic()
        session_id = str(message.session_id)
        sequence = int(message.decision_sequence)
        ttl_sec = _seconds(message.valid_for)
        source_age = (
            self.get_clock().now().nanoseconds * 1.0e-9
            - _seconds(message.stamp)
        )
        interarrival_ok = (
            self.last_safety_receive is None
            or now - self.last_safety_receive
            <= SAFETY_RECEIVE_MAX_AGE_SEC
        )
        sequence_ok = (
            self.last_safety_sequence is None
            or sequence > self.last_safety_sequence
        )
        session_ok = bool(session_id) and (
            self.safety_session_id is None
            or session_id == self.safety_session_id
        )
        valid_allow = bool(
            message.valid
            and message.state == 'ALLOW'
            and ttl_sec > 0.0
            and -0.05 <= source_age <= ttl_sec
            and interarrival_ok
            and sequence_ok
            and session_ok
        )
        self.last_safety_receive = now
        if session_ok and self.safety_session_id is None:
            self.safety_session_id = session_id
        if session_ok:
            self.last_safety_sequence = sequence
        if valid_allow and not self.allow:
            self.allow_since = now
        elif not valid_allow:
            self.allow_since = None
        self.allow = valid_allow

    def _joint_callback(self, robot_id, message):
        ordered = _ordered_positions(message, robot_id)
        if ordered is None:
            return
        positions = ordered[1]
        self.positions[robot_id] = positions
        self.position_receive[robot_id] = time.monotonic()
        if robot_id in self.baseline:
            excursion = _maximum_change(
                self.baseline[robot_id], positions
            )
            self.max_excursion[robot_id] = max(
                self.max_excursion[robot_id], excursion
            )

    def _execution_callback(self, message):
        if message.command_id != self.command_id:
            return
        status = str(message.status)
        self.status_history.append(status)
        self.final_status = {'status': status, 'detail': message.detail}
        now = time.monotonic()
        if status == 'PENDING' and self.stage == 'SENT':
            self.pending_monotonic = now
            self.stage = 'PENDING'
            return
        if status == 'ACCEPTED' and self.stage == 'PENDING':
            self.accepted_monotonic = now
            self.stage = 'ACCEPTED'
            print('ACTIVE_STOP_PROBE_ACCEPTED', flush=True)
            return
        if status == 'CANCELED' and self.stage == 'FAULT_INJECTED':
            self.canceled_monotonic = now
            self.cancel_positions = self.positions.get('arm_02')
            self.stage = 'SETTLING'
            return
        if status in ('REJECTED', 'FAILED', 'DRY_RUN', 'SUCCEEDED'):
            self.failure_reason = f'unexpected terminal status: {status}'
            self._finish(2)
            return
        if status == 'CANCELED':
            self.failure_reason = 'CANCELED occurred before fault injection'
            self.canceled_monotonic = now
            self.cancel_positions = self.positions.get('arm_02')
            self._finish(2)
            return
        self.failure_reason = f'out-of-order execution status: {status}'
        self._finish(2)

    def _preconditions_met(self, now):
        if not (
            self.ready
            and self.allow
            and self.ready_since is not None
            and self.allow_since is not None
            and now - self.ready_since >= STABLE_EVIDENCE_SEC
            and now - self.allow_since >= STABLE_EVIDENCE_SEC
            and self.last_ready_receive is not None
            and now - self.last_ready_receive <= READY_RECEIVE_MAX_AGE_SEC
            and self.last_safety_receive is not None
            and now - self.last_safety_receive
            <= SAFETY_RECEIVE_MAX_AGE_SEC
        ):
            return False
        if not all(robot_id in self.positions for robot_id in ROBOT_IDS):
            return False
        ages = tuple(
            now - self.position_receive[robot_id]
            for robot_id in ROBOT_IDS
        )
        if max(ages) > JOINT_RECEIVE_MAX_AGE_SEC:
            return False
        receive_times = tuple(
            self.position_receive[robot_id] for robot_id in ROBOT_IDS
        )
        if max(receive_times) - min(receive_times) > JOINT_RECEIVE_MAX_SKEW_SEC:
            return False
        if self.publisher.get_subscription_count() != 1:
            return False
        if self.count_publishers(
            '/arm_02/thesis/execution_trajectory'
        ) != 1:
            return False
        return True

    def _publish_command(self):
        names = tuple(f'arm_02_jaco_joint_{index}' for index in range(1, 7))
        target = list(self.positions['arm_02'])
        target[0] += self.delta_rad
        command = JointCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_id = self.command_id
        command.joint_names = list(names)
        command.positions = target
        command.duration.sec = int(self.duration_sec)
        command.duration.nanosec = int(
            round((self.duration_sec - int(self.duration_sec)) * 1.0e9)
        )
        self.baseline = dict(self.positions)
        self.send_monotonic = time.monotonic()
        self.publisher.publish(command)
        self.stage = 'SENT'
        print(
            f'ACTIVE_STOP_PROBE_SENT command_id={self.command_id}',
            flush=True,
        )

    def _inject_fault(self):
        while True:
            try:
                signal.pidfd_send_signal(
                    self.coordinator_pidfd, signal.SIGTERM, None, 0
                )
                break
            except InterruptedError:
                continue
            except OSError as exception:
                if exception.errno == errno.ESRCH:
                    self.coordinator_exit_observed = True
                self.failure_reason = (
                    f'coordinator fault injection failed: {exception}'
                )
                self._finish(2)
                return
        self.fault_monotonic = time.monotonic()
        self.stage = 'FAULT_INJECTED'
        print(
            'ACTIVE_STOP_PROBE_FAULT_INJECTED '
            f'pid={self.coordinator_pid}',
            flush=True,
        )

    def _fail(self, reason):
        if self.failure_reason is None:
            self.failure_reason = reason
        if self.stage in ('ACCEPTED', 'PENDING'):
            self._inject_fault()
            return
        self._finish(2)

    def _success_failure(self):
        if self.failure_reason is not None:
            return self.failure_reason
        if self.status_history != ['PENDING', 'ACCEPTED', 'CANCELED']:
            return 'execution lifecycle was not PENDING,ACCEPTED,CANCELED'
        if self.fault_monotonic is None or self.canceled_monotonic is None:
            return 'fault or cancellation timestamp missing'
        latency = self.canceled_monotonic - self.fault_monotonic
        if not 0.0 <= latency <= CANCEL_LATENCY_BUDGET_SEC:
            return 'cancellation latency exceeded budget'
        if self.max_excursion['arm_02'] < MOTION_THRESHOLD_RAD:
            return 'arm_02 movement was not observed before fault injection'
        if any(
            self.max_excursion[robot_id] > ISOLATION_TOLERANCE_RAD
            for robot_id in ('arm_03', 'arm_04')
        ):
            return 'an uncommanded simulated arm exceeded isolation tolerance'
        if (
            self.post_cancel_drift_rad is None
            or self.post_cancel_drift_rad > POST_CANCEL_DRIFT_TOLERANCE_RAD
        ):
            return 'arm_02 post-cancel drift exceeded tolerance'
        return None

    def _result(self):
        latency = (
            self.canceled_monotonic - self.fault_monotonic
            if self.canceled_monotonic is not None
            and self.fault_monotonic is not None
            else None
        )
        return {
            'passed': self.exit_code == 0,
            'failure_reason': self.failure_reason,
            'command_id': self.command_id,
            'coordinator': {
                'pid': self.coordinator_pid,
                'entrypoint': self.coordinator_entrypoint,
                'pidfd_verified': True,
                'exit_observed': self.coordinator_exit_observed,
            },
            'safety_session_id': self.safety_session_id,
            'delta_rad': self.delta_rad,
            'duration_sec': self.duration_sec,
            'stage': self.stage,
            'final_status': self.final_status,
            'status_history': self.status_history,
            'send_monotonic': self.send_monotonic,
            'accepted_monotonic': self.accepted_monotonic,
            'fault_monotonic': self.fault_monotonic,
            'canceled_monotonic': self.canceled_monotonic,
            'fault_to_cancel_latency_sec': latency,
            'maximum_excursion_rad': self.max_excursion,
            'post_cancel_drift_rad': self.post_cancel_drift_rad,
            'thresholds': {
                'stable_evidence_sec': STABLE_EVIDENCE_SEC,
                'motion_rad': MOTION_THRESHOLD_RAD,
                'isolation_rad': ISOLATION_TOLERANCE_RAD,
                'post_cancel_drift_rad': POST_CANCEL_DRIFT_TOLERANCE_RAD,
                'cancel_latency_sec': CANCEL_LATENCY_BUDGET_SEC,
            },
        }

    def _finish(self, exit_code):
        if self.done:
            return
        self.exit_code = exit_code
        print(
            'ACTIVE_STOP_PROBE_RESULT ' + json.dumps(self._result()),
            flush=True,
        )
        self.done = True

    def _tick(self):
        now = time.monotonic()
        if self.done:
            return
        if self.pidfd_poll.poll(0):
            self.coordinator_exit_observed = True
        if self.stage == 'WAITING' and self._preconditions_met(now):
            self._publish_command()
            return
        if self.stage == 'SENT' and now - self.send_monotonic > 2.0:
            self._fail('PENDING status timeout')
            return
        if self.stage == 'PENDING' and now - self.pending_monotonic > 2.0:
            self._fail('ACCEPTED status timeout')
            return
        if self.stage == 'ACCEPTED':
            if self.max_excursion['arm_02'] >= MOTION_THRESHOLD_RAD:
                self._inject_fault()
                return
            if now - self.accepted_monotonic > 3.0:
                self._fail('arm_02 movement onset timeout')
                return
        if (
            self.stage == 'FAULT_INJECTED'
            and now - self.fault_monotonic > CANCEL_LATENCY_BUDGET_SEC
        ):
            self._fail('CANCELED status timeout')
            return
        if self.stage == 'SETTLING':
            if now - self.canceled_monotonic >= 1.0:
                self.post_cancel_drift_rad = _maximum_change(
                    self.cancel_positions,
                    self.positions.get('arm_02'),
                )
                self.failure_reason = self._success_failure()
                self.stage = 'COMPLETE'
                self._finish(0 if self.failure_reason is None else 2)
                return
        if now > self.deadline:
            self._fail('overall probe timeout')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--delta-rad', type=float, default=0.15)
    parser.add_argument('--duration-sec', type=float, default=8.0)
    parser.add_argument('--timeout-sec', type=float, default=60.0)
    parser.add_argument('--terminate-coordinator-pid', type=int, required=True)
    arguments = parser.parse_args()
    if not 0.0 < arguments.delta_rad <= 0.20:
        parser.error('--delta-rad must be in (0.0, 0.20]')
    if not 5.0 <= arguments.duration_sec <= 15.0:
        parser.error('--duration-sec must be in [5.0, 15.0]')
    if arguments.timeout_sec <= arguments.duration_sec:
        parser.error('--timeout-sec must exceed --duration-sec')
    try:
        pidfd, entrypoint = open_verified_coordinator_pidfd(
            arguments.terminate_coordinator_pid
        )
    except (OSError, ValueError) as exception:
        parser.error(str(exception))

    rclpy.init()
    node = ActiveStopProbe(
        arguments.delta_rad,
        arguments.duration_sec,
        arguments.timeout_sec,
        arguments.terminate_coordinator_pid,
        pidfd,
        entrypoint,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        node.failure_reason = 'probe interrupted'
        node._finish(130)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(pidfd)
    raise SystemExit(node.exit_code)


if __name__ == '__main__':
    main()
