#!/usr/bin/env python3
"""Send one candidate to the simulation profile supervisor.

Examples (source the workspace and select its simulation ROS_DOMAIN_ID first):
  python3 tools/simulation/command_robot.py --robot ur17g --joint-index 1 --delta 0.02
  python3 tools/simulation/command_robot.py --robot p3020 --xyz 0.8 4.0 1.2 --position-only

Cartesian XYZ is an absolute position in world, in metres. RPY is radians,
using fixed-axis roll/pitch/yaw. Without --rpy the current orientation is
preserved, unless --position-only is requested. Joint indices follow the
profile's active-joint order, so P3020 indices 1..5 refer to joints 1,2,3,5,6.
Exit zero means accepted by the supervisor, not completed motion.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import uuid
import xml.etree.ElementTree as ET


ROBOTS = ('jaco2', 'ur17g', 'p3020', 'cs625')
READ_TIMEOUT = 30.0


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', required=True, choices=ROBOTS)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--joint-index', type=int, help='1-based index among active arm joints')
    mode.add_argument('--xyz', nargs=3, type=float, metavar=('X', 'Y', 'Z'), help='absolute world position in metres')
    parser.add_argument('--delta', type=float, help='joint displacement in radians; requires --joint-index')
    parser.add_argument('--duration', type=float, default=2.0, help='requested trajectory duration, seconds (default 2)')
    parser.add_argument('--rpy', nargs=3, type=float, metavar=('ROLL', 'PITCH', 'YAW'), help='world target orientation in radians')
    parser.add_argument('--position-only', action='store_true', help='solve XYZ without constraining tool orientation')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0.1 <= args.duration <= 30.0:
        parser.error('--duration must be finite and within [0.1, 30] seconds')
    if args.joint_index is not None:
        count = 5 if args.robot == 'p3020' else 6
        if not 1 <= args.joint_index <= count:
            parser.error(f'--joint-index must lie within [1, {count}] for {args.robot}')
        if args.delta is None or not math.isfinite(args.delta):
            parser.error('--joint-index requires a finite --delta in radians')
        if args.rpy is not None or args.position_only:
            parser.error('--rpy and --position-only require --xyz')
    else:
        if args.delta is not None:
            parser.error('--delta requires --joint-index')
        if not all(math.isfinite(value) for value in args.xyz):
            parser.error('--xyz values must be finite')
        if args.rpy is not None and not all(math.isfinite(value) for value in args.rpy):
            parser.error('--rpy values must be finite')
        if args.rpy is not None and args.position_only:
            parser.error('--rpy and --position-only cannot be combined')
    return args


def execute(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from scipy.spatial.transform import Rotation
    from thesis_interfaces.msg import CommandDecision, JointCommand
    from thesis_multi_robot.kinematics import RobotKinematics
    from thesis_multi_robot.profiles import source_module

    rclpy.init(args=[])
    node = Node(
        'simulation_command_' + uuid.uuid4().hex[:10],
        parameter_overrides=[Parameter('use_sim_time', value=True)],
    )
    started = time.monotonic()
    name = args.robot
    module = source_module(name)
    names = tuple(name + '_' + joint for joint in module.JOINT_NAMES)
    state = None
    decision = None
    command_id = 'sim-' + name + '-' + uuid.uuid4().hex

    def state_callback(message):
        nonlocal state
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise ValueError('malformed joint state')
            values = dict(zip(message.name, message.position))
            positions = tuple(float(values[joint]) for joint in names)
            if not all(math.isfinite(value) for value in positions):
                raise ValueError('non-finite joint state')
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            state = (positions, stamp, time.monotonic())
        except (KeyError, TypeError, ValueError):
            state = None

    def decision_callback(message):
        nonlocal decision
        if message.command_id == command_id:
            decision = message

    def fresh_state():
        if state is None:
            return False
        now_ns = node.get_clock().now().nanoseconds
        age = (now_ns - state[1]) / 1e9
        return now_ns > 0 and state[1] > 0 and -0.05 <= age <= 0.5 and time.monotonic() - state[2] <= 0.5

    def wait_until(predicate, deadline, explanation):
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(explanation)
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped')

    try:
        joint_sub = node.create_subscription(JointState, f'/{name}/joint_states', state_callback, qos_profile_sensor_data)
        decision_sub = node.create_subscription(CommandDecision, f'/{name}/thesis/command_decision', decision_callback, 10)
        publisher = node.create_publisher(JointCommand, f'/{name}/thesis/candidate_command', 10)
        client = node.create_client(GetParameters, f'/{name}/robot_state_publisher/get_parameters')
        wait_until(client.service_is_ready, started + READ_TIMEOUT, 'simulation robot_description service was not discovered within 30 s')
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = client.call_async(request)
        wait_until(future.done, started + READ_TIMEOUT, 'timed out reading simulation robot_description')
        response = future.result()
        if response is None or len(response.values) != 1 or response.values[0].type != Parameter.Type.STRING.value:
            raise ValueError('robot_description parameter is absent or is not a string')
        root = ET.fromstring(response.values[0].string_value)
        backends = [plugin.text for plugin in root.findall('ros2_control/hardware/plugin')]
        namespaces = [namespace.text for namespace in root.findall('gazebo/plugin/ros/namespace')]
        if backends != ['gz_ros2_control/GazeboSimSystem'] or namespaces != ['/' + name]:
            raise ValueError('description does not identify the requested isolated Gazebo simulation backend')
        model = RobotKinematics(root, names)
        tip = name + '_' + module.TIP_LINK
        wait_until(
            lambda: fresh_state() and publisher.get_subscription_count() > 0
                    and node.count_publishers(f'/{name}/thesis/command_decision') > 0,
            started + READ_TIMEOUT,
            'fresh simulation state and supervisor candidate/decision endpoints were not discovered within 30 s',
        )
        seed = state[0]
        if args.joint_index is not None:
            positions = list(seed)
            positions[args.joint_index - 1] += args.delta
            model.forward(positions)  # Validate finite values and URDF bounds.
            command_kind = 'joint_delta'
        else:
            target = model.forward(seed)[tip].copy()
            target[:3, 3] = args.xyz
            if args.rpy is not None:
                target[:3, :3] = Rotation.from_euler('xyz', args.rpy).as_matrix()
            positions = model.inverse(target, seed, tip, position_only=args.position_only)
            command_kind = 'cartesian_position' if args.position_only else 'cartesian_pose'
        # Process the next clock/state update after local IK and stamp exactly
        # once. Candidate publication is never retried with a repeated ID.
        rclpy.spin_once(node, timeout_sec=0.05)
        wait_until(fresh_state, time.monotonic() + READ_TIMEOUT, 'fresh state was lost while preparing the candidate')
        command = JointCommand()
        command.command_id = command_id
        command.stamp = node.get_clock().now().to_msg()
        command.joint_names = list(names)
        command.positions = [float(value) for value in positions]
        duration_ns = round(args.duration * 1e9)
        command.duration.sec, command.duration.nanosec = divmod(duration_ns, 1_000_000_000)
        publisher.publish(command)
        wait_until(lambda: decision is not None, time.monotonic() + READ_TIMEOUT, 'supervisor decision timed out after 30 s; candidate was not retried')
        print(json.dumps({
            'robot': name, 'command_id': command_id, 'command_kind': command_kind,
            'accepted': bool(decision.accepted), 'state': decision.state,
            'reason_code': decision.reason_code, 'reason': decision.reason,
            'duration_s': args.duration, 'joint_names': list(names),
            'positions_rad': list(command.positions), 'execution_confirmed': False,
        }, sort_keys=True))
        return 0 if decision.accepted else 1
    except (ValueError, RuntimeError, TimeoutError, KeyError) as error:
        print(json.dumps({
            'robot': name, 'command_id': command_id, 'accepted': False,
            'state': 'STOP', 'reason_code': 'CLIENT_ERROR', 'reason': str(error),
            'execution_confirmed': False,
        }, sort_keys=True))
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    args = parse_arguments(argv)
    try:
        return execute(args)
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
