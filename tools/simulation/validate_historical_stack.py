#!/usr/bin/env python3
"""Validate one full historical GUI/safety simulation profile live."""

from __future__ import annotations

import argparse
import json
import math
import time
import uuid


PROFILES = {
    'jaco2': 6,
    'ur17g': 6,
    'p3020': 5,
    'cs625': 6,
}


def parse_arguments(argv=None):
    """Parse one independently launched profile validation request."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=tuple(PROFILES), required=True)
    parser.add_argument('--timeout', type=float, default=40.0)
    parser.add_argument('--delta', type=float, default=0.01)
    parser.add_argument('--duration', type=float, default=1.5)
    result = parser.parse_args(argv)
    if not math.isfinite(result.timeout) or result.timeout <= 0.0:
        parser.error('--timeout must be positive and finite')
    if not math.isfinite(result.delta) or abs(result.delta) > 0.05:
        parser.error('--delta must be finite and no greater than 0.05 rad')
    if not math.isfinite(result.duration) or not 0.5 <= result.duration <= 5.0:
        parser.error('--duration must belong to [0.5, 5.0] seconds')
    return result


def execute(arguments):
    """Exercise preview, admission, execution and final-state evidence."""
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from thesis_interfaces.msg import (
        CommandDecision,
        ExecutionTrajectory,
        JointCommand,
        JointTrajectoryPrediction,
        ProximityStatus,
        TrajectoryPrediction,
    )
    from visualization_msgs.msg import MarkerArray

    name = arguments.robot
    dof = PROFILES[name]
    command_id = f'historical-{name}-{uuid.uuid4().hex}'
    prefix = '' if name == 'jaco2' else f'/{name}'
    expected_joint_names = (
        tuple(f'j2n6s300_joint_{index}' for index in range(1, 7))
        if name == 'jaco2' else None
    )
    rclpy.init(args=[])
    node = Node(
        f'validate_historical_{name}_{uuid.uuid4().hex[:8]}',
        parameter_overrides=[Parameter('use_sim_time', value=True)],
    )
    data = {
        'readiness': None,
        'kinematic': None,
        'state': None,
        'proximity': None,
        'decision': None,
        'executions': [],
        'joint_prediction': None,
        'safety_prediction': None,
        'marker_namespaces': set(),
    }

    def wait_for(predicate, timeout, explanation):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(explanation)
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS stopped during validation')

    def state_callback(message):
        values = dict(zip(message.name, message.position))
        expected = expected_joint_names or tuple(
            joint for joint in message.name if joint.startswith(name + '_')
        )
        if len(expected) != dof or not all(
            joint in values and math.isfinite(float(values[joint]))
            for joint in expected
        ):
            return
        data['state'] = (
            expected,
            tuple(float(values[joint]) for joint in expected),
            time.monotonic(),
        )

    def readiness_callback(message):
        data['readiness'] = message.data

    def kinematic_callback(message):
        data['kinematic'] = message.data

    def proximity_callback(message):
        data['proximity'] = message

    def decision_callback(message):
        if message.command_id == command_id:
            data['decision'] = message

    def execution_callback(message):
        if message.command_id == command_id:
            data['executions'].append(message)

    def joint_prediction_callback(message):
        if message.command_id == command_id:
            data['joint_prediction'] = message

    def safety_prediction_callback(message):
        if message.command_id == command_id:
            data['safety_prediction'] = message

    def marker_callback(message):
        data['marker_namespaces'].update(
            marker.ns for marker in message.markers
        )

    subscriptions = [
        node.create_subscription(
            JointState,
            f'{prefix}/joint_states',
            state_callback,
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            String,
            f'{prefix}/thesis/system_readiness',
            readiness_callback,
            10,
        ),
        node.create_subscription(
            String,
            f'{prefix}/thesis/kinematic_tf_status',
            kinematic_callback,
            10,
        ),
        node.create_subscription(
            ProximityStatus,
            f'{prefix}/thesis/proximity_status',
            proximity_callback,
            10,
        ),
        node.create_subscription(
            CommandDecision,
            f'{prefix}/thesis/command_decision',
            decision_callback,
            10,
        ),
        node.create_subscription(
            ExecutionTrajectory,
            f'{prefix}/thesis/execution_trajectory',
            execution_callback,
            10,
        ),
        node.create_subscription(
            JointTrajectoryPrediction,
            f'{prefix}/thesis/joint_trajectory_prediction',
            joint_prediction_callback,
            10,
        ),
        node.create_subscription(
            TrajectoryPrediction,
            f'{prefix}/thesis/trajectory_prediction',
            safety_prediction_callback,
            10,
        ),
        node.create_subscription(
            MarkerArray,
            f'{prefix}/thesis/horizon_volume',
            marker_callback,
            10,
        ),
    ]
    preview_publisher = node.create_publisher(
        JointCommand, f'{prefix}/thesis/preview_intent', 10
    )
    candidate_publisher = node.create_publisher(
        JointCommand, f'{prefix}/thesis/candidate_command', 10
    )

    try:
        wait_for(
            lambda: (
                data['readiness'] == 'READY'
                and data['kinematic'] is not None
                and data['kinematic'].startswith('VALID')
                and data['state'] is not None
                and data['proximity'] is not None
                and preview_publisher.get_subscription_count() > 0
                and candidate_publisher.get_subscription_count() > 0
            ),
            arguments.timeout,
            'full historical stack did not become READY',
        )
        joint_names, initial, _ = data['state']
        target = list(initial)
        target[0] += arguments.delta
        command = JointCommand()
        command.stamp = node.get_clock().now().to_msg()
        command.command_id = command_id
        command.joint_names = list(joint_names)
        command.positions = target
        duration_ns = round(arguments.duration * 1.0e9)
        command.duration.sec, command.duration.nanosec = divmod(
            duration_ns, 1_000_000_000
        )
        preview_publisher.publish(command)
        candidate_publisher.publish(command)
        wait_for(
            lambda: (
                data['joint_prediction'] is not None
                and data['decision'] is not None
            ),
            5.0,
            'future projection or historical decision was not published',
        )
        if not data['decision'].accepted:
            raise RuntimeError(
                'command rejected: '
                f'{data["decision"].reason_code}: '
                f'{data["decision"].reason}'
            )
        wait_for(
            lambda: any(
                message.status == 'SUCCEEDED'
                for message in data['executions']
            ),
            arguments.duration + 8.0,
            'Gazebo execution did not report SUCCEEDED',
        )
        wait_for(
            lambda: (
                data['safety_prediction'] is not None
                and 'predicted_volume' in data['marker_namespaces']
                and 'current_capsules' in data['marker_namespaces']
            ),
            5.0,
            'safety prediction or RViz future markers are incomplete',
        )
        prediction = data['joint_prediction']
        if (
            tuple(prediction.joint_names) != tuple(joint_names)
            or prediction.sample_count < 2
            or len(prediction.positions) != prediction.sample_count * dof
        ):
            raise RuntimeError('joint prediction dimensions are invalid')
        wait_for(
            lambda: (
                data['state'] is not None
                and abs(data['state'][1][0] - target[0]) <= 0.02
            ),
            3.0,
            'final joint state did not reach the approved target',
        )
        result = {
            'robot': name,
            'dof': dof,
            'ready': data['readiness'],
            'kinematic_tf': data['kinematic'],
            'proximity_state': data['proximity'].state,
            'proximity_clearance_m': data['proximity'].minimum_clearance,
            'decision': data['decision'].state,
            'decision_reason_code': data['decision'].reason_code,
            'execution_statuses': [
                message.status for message in data['executions']
            ],
            'prediction_samples': prediction.sample_count,
            'prediction_source': prediction.source,
            'safety_prediction': data['safety_prediction'].state,
            'marker_namespaces': sorted(data['marker_namespaces']),
            'joint_names': list(joint_names),
            'target_delta_rad': arguments.delta,
        }
        print(json.dumps(result, sort_keys=True))
        return 0
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(json.dumps({
            'robot': name,
            'ready': data['readiness'],
            'kinematic_tf': data['kinematic'],
            'error': str(error),
        }, sort_keys=True))
        return 1
    finally:
        del subscriptions
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    """Run the live historical-stack validator."""
    return execute(parse_arguments(argv))


if __name__ == '__main__':
    raise SystemExit(main())
