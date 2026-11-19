#!/usr/bin/env python3
"""Isolated simulation probe: continuously jog as a sphere approaches."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=('jaco2', 'ur17g', 'p3020', 'cs625'), required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--trace', type=Path, help='Create a JSONL event trace (must not exist)')
    parser.add_argument('--free', action='store_true', help='Place the JACO2 obstacle outside the workspace')
    parser.add_argument('--jog-seconds', type=float, default=23.0,
                        help='Duration of sustained JOG after warmup; remainder observes stopping')
    parser.add_argument('--inject-stop-after', type=float, help='Place sphere in stop zone after this many seconds of JOG')
    parser.add_argument('--withdraw-after', type=float, help='Reverse joint 1 JOG after this many seconds of motion')
    parser.add_argument('--drop-obstacle-after', type=float, help='Suspend obstacle publications after this many seconds of motion')
    parser.add_argument('--drop-obstacle-seconds', type=float, default=2.0)
    parser.add_argument('--freeze-tf-after', type=float, help='Suspend isolated robot_state_publisher after this many seconds of motion')
    parser.add_argument('--freeze-tf-seconds', type=float, default=2.0)
    parser.add_argument('--pause-clock-after', type=float, help='Pause isolated Gazebo after this many seconds of motion')
    parser.add_argument('--pause-clock-seconds', type=float, default=2.0)
    parser.add_argument('--braking-reserve-m', type=float, help='Set and record supervisor braking reserve before motion')
    args = parser.parse_args()
    trace = args.trace.open('x') if args.trace else None

    def record(kind, message):
        if trace is None:
            return
        from rosidl_runtime_py.convert import message_to_ordereddict
        trace.write(json.dumps({'event': kind, 'monotonic': time.monotonic(),
                                'message': message_to_ordereddict(message)}) + '\n')
        trace.flush()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['IGN_PARTITION'] = 'jog_probe_' + uuid.uuid4().hex
    directory = Path(tempfile.mkdtemp(prefix='jog_probe_'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'logs')
    log = (directory / 'launch.log').open('x')
    process = subprocess.Popen([
        'ros2', 'launch', 'thesis_multi_robot', 'historical_robot.launch.py',
        f'robot:={args.robot}', 'headless:=true', 'start_gui:=false',
        'start_rviz:=false', 'demo_obstacle:=false', f'runtime_dir:={directory}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    frozen_tf_pid = None
    tf_injected = False
    clock_paused = False
    clock_injected = False

    def set_pause(paused):
        completed = subprocess.run([
            'ign', 'service', '-s', '/world/jaco_world/control',
            '--reqtype', 'ignition.msgs.WorldControl',
            '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000',
            '--req', 'pause: true' if paused else 'pause: false',
        ], capture_output=True, text=True, timeout=4)
        if completed.returncode or 'data: true' not in completed.stdout:
            raise RuntimeError(f'Gazebo pause request failed: {completed.stdout} {completed.stderr}')
    result = {'robot': args.robot, 'runtime': str(directory), 'pass': False}
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.time import Time
        import tf2_ros
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter as RosParameter
        from thesis_interfaces.msg import JointCommand, Obstacle, ExecutionControl, ProximityStatus, JointTrajectoryPrediction, TrajectoryPrediction
        from thesis_core.robot_profile import load_robot_profile, load_robot_profiles

        manifest = directory / 'cell.json'
        deadline = time.monotonic() + 45
        while args.robot != 'jaco2' and not manifest.exists():
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('launch did not produce manifest')
            time.sleep(.1)
        profile = (load_robot_profile() if args.robot == 'jaco2'
                   else load_robot_profiles(manifest)[args.robot])
        rclpy.init()
        node = Node('isolated_jog_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
        prefix = '' if args.robot == 'jaco2' else '/' + args.robot
        state = {'positions': None, 'proximity': None, 'applied': [], 'controls': [], 'received': [], 'velocities': [], 'ready': None, 'joint_stamp': None, 'joint_received': 0., 'proximity_received': 0.}
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)

        def joint_callback(message):
            record('joint_state', message)
            values = dict(zip(message.name, message.position))
            if all(name in values for name in profile.joint_names):
                state['positions'] = tuple(values[name] for name in profile.joint_names)
                state['joint_stamp'] = message.header.stamp
                state['joint_received'] = time.monotonic()
                speeds = dict(zip(message.name, message.velocity))
                if all(name in speeds for name in profile.joint_names):
                    state['velocities'].append((time.monotonic(), speeds[profile.joint_names[0]]))

        def proximity_callback(message):
            record('proximity', message)
            state['proximity'] = message
            state['proximity_received'] = time.monotonic()

        def readiness_callback(message):
            record('readiness', message)
            state['ready'] = (message.data, time.monotonic())

        def control_callback(message):
            record('execution_control', message)
            if message.command_id.startswith('probe-'):
                state['controls'].append((time.monotonic(), message.state, message.speed_scale,
                                          message.current_clearance, message.nominal_clearance))

        def applied_callback(message):
            record('applied_jog', message)
            if message.command_id.startswith('probe-'):
                state['applied'].append((time.monotonic(), tuple(message.positions)))

        subscriptions = [
            node.create_subscription(JointState, prefix + '/joint_states', joint_callback, qos_profile_sensor_data),
            node.create_subscription(ProximityStatus, prefix + '/thesis/proximity_status', proximity_callback, 10),
            node.create_subscription(ExecutionControl, prefix + '/thesis/execution_control', control_callback, 10),
            node.create_subscription(JointCommand, prefix + '/thesis/applied_jog_command', applied_callback, 10),
            node.create_subscription(String, prefix + '/thesis/system_readiness', readiness_callback, 10),
            node.create_subscription(String, prefix + '/thesis/jog_horizon_evidence', lambda msg: record('jog_horizon_evidence', msg), 10),
            node.create_subscription(JointTrajectoryPrediction, prefix + '/thesis/joint_trajectory_prediction', lambda msg: record('joint_prediction', msg), 10),
            node.create_subscription(TrajectoryPrediction, prefix + '/thesis/trajectory_prediction', lambda msg: record('trajectory_prediction', msg), 10),
        ]
        obstacle_topic = ('/thesis/obstacle_input' if args.robot == 'jaco2'
                          else prefix + '/thesis/obstacle')
        obstacle_pub = node.create_publisher(Obstacle, obstacle_topic, 10)
        jog_pub = node.create_publisher(JointCommand, prefix + '/thesis/jog_intent', 10)
        parameter_client = node.create_client(SetParameters, prefix + '/safety_supervisor_node/set_parameters')
        parameter_future = None
        parameter_set = args.braking_reserve_m is None
        started = time.monotonic()
        motion_started = None
        previous_clock = 0
        clock_advanced = False
        initial = None
        closest = None
        next_send = started
        while time.monotonic() - started < 70 and (motion_started is None or time.monotonic() - motion_started < args.jog_seconds + 5):
            if process.poll() is not None:
                raise RuntimeError('launch exited')
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            clock_now = node.get_clock().now().nanoseconds
            clock_advanced = clock_advanced or (previous_clock > 0 and clock_now > previous_clock)
            previous_clock = clock_now
            if now < next_send or state['positions'] is None:
                continue
            next_send = now + .05
            current = state['positions']
            if initial is None:
                initial = current
                # Anchor the incoming sphere above the currently most distal
                # capsule, then approach slowly in world +X.
                capsule = profile.capsule_segments(current)[-1]
                closest = tuple((a + b) / 2 for a, b in zip(capsule.start, capsule.end))
                if args.robot == 'jaco2':
                    future = list(current)
                    future[0] += 0.01
                    future_capsule = profile.capsule_segments(future)[-1]
                    future_center = tuple((a + b) / 2 for a, b in zip(future_capsule.start, future_capsule.end))
                    direction = tuple(b - a for a, b in zip(closest, future_center))
                    norm = math.sqrt(sum(value * value for value in direction))
                    if norm < 1e-8:
                        raise RuntimeError('cannot establish approach direction')
                    closest = tuple(a + 0.20 * d / norm for a, d in zip(closest, direction))
                    if args.free:
                        closest = (10.0, 10.0, 10.0)
            phase = max(0., now - started - 7.)
            offset = max(.12, .70 - .045 * phase)
            obstacle = Obstacle()
            obstacle.header.stamp = node.get_clock().now().to_msg()
            obstacle.header.frame_id = 'world'
            active_center = closest
            if args.inject_stop_after is not None and motion_started is not None and now - motion_started >= args.inject_stop_after:
                if 'injected_center' not in result:
                    capsule = profile.capsule_segments(current)[-1]
                    center = tuple((a + b) / 2 for a, b in zip(capsule.start, capsule.end))
                    result['injected_center'] = (center[0] + .16, center[1], center[2])
                active_center = result['injected_center']
            obstacle.center.x = active_center[0] if args.robot == 'jaco2' else active_center[0] + offset
            obstacle.center.y = active_center[1]
            obstacle.center.z = active_center[2]
            obstacle.velocity.x = -.045 if args.robot != 'jaco2' and phase > 0. and offset > .12 else 0.
            obstacle.radius = .04
            obstacle.uncertainty = .0
            motion_age = None if motion_started is None else now - motion_started
            if (args.pause_clock_after is not None and motion_age is not None
                    and motion_age >= args.pause_clock_after and not clock_injected):
                set_pause(True)
                clock_paused = True
                clock_injected = True
                result['clock_pause_monotonic'] = time.monotonic()
            if (clock_paused and motion_age is not None and
                    motion_age >= args.pause_clock_after + args.pause_clock_seconds):
                set_pause(False)
                clock_paused = False
                result['clock_resume_monotonic'] = time.monotonic()
            if (args.freeze_tf_after is not None and motion_age is not None
                    and motion_age >= args.freeze_tf_after and not tf_injected):
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit():
                        continue
                    try:
                        pid = int(entry.name)
                        command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ')
                        if (b'/robot_state_publisher/robot_state_publisher' in command
                                and os.getpgid(pid) == process.pid):
                            os.kill(pid, signal.SIGSTOP)
                            frozen_tf_pid = pid
                            tf_injected = True
                            result['frozen_tf_pid'] = pid
                            break
                    except (ProcessLookupError, PermissionError, FileNotFoundError):
                        continue
                if not tf_injected:
                    raise RuntimeError('isolated robot_state_publisher not found')
            if (frozen_tf_pid is not None and motion_age is not None and
                    motion_age >= args.freeze_tf_after + args.freeze_tf_seconds):
                os.kill(frozen_tf_pid, signal.SIGCONT)
                frozen_tf_pid = None
            dropping = (args.drop_obstacle_after is not None and motion_age is not None
                        and args.drop_obstacle_after <= motion_age <
                        args.drop_obstacle_after + args.drop_obstacle_seconds)
            if not dropping:
                obstacle_pub.publish(obstacle)
                record('obstacle', obstacle)
            if motion_started is None:
                ready = state['ready']
                prox = state['proximity']
                valid = (clock_advanced and ready is not None and ready[0] == 'READY'
                         and now - ready[1] < 2.5 and now - state['joint_received'] < .5
                         and prox is not None and now - state['proximity_received'] < .5
                         and getattr(prox, 'geometry_valid', False))
                if valid and args.robot == 'jaco2':
                    try:
                        frames = ['j2n6s300_link_base'] + [f'j2n6s300_link_{i}' for i in range(1, 6)] + [profile.tip_link]
                        model = profile.capsule_segments(current)
                        endpoints = [model[0].start] + [segment.end for segment in model]
                        errors = []
                        for frame, expected in zip(frames, endpoints):
                            transform = tf_buffer.lookup_transform('world', frame, Time.from_msg(state['joint_stamp']))
                            actual = transform.transform.translation
                            errors.append(math.dist(expected, (actual.x, actual.y, actual.z)))
                        valid = max(errors) <= .02
                        result['fk_tf_endpoint_errors_m'] = dict(zip(frames, errors))
                    except tf2_ros.TransformException:
                        valid = False
                if valid:
                    if not parameter_set:
                        if parameter_future is None and parameter_client.service_is_ready():
                            request = SetParameters.Request()
                            request.parameters = [RosParameter('braking_reserve_m', value=args.braking_reserve_m).to_parameter_msg()]
                            parameter_future = parameter_client.call_async(request)
                        if parameter_future is None or not parameter_future.done():
                            continue
                        response = parameter_future.result()
                        if response is None or not all(item.successful for item in response.results):
                            raise RuntimeError(f'braking reserve rejected: {response}')
                        result['braking_reserve_m'] = args.braking_reserve_m
                        parameter_set = True
                    motion_started = now
                    initial = current
                    result['readiness_before_jog'] = ready[0]
                    result['obstacle_center'] = closest
            if motion_started is not None and now - motion_started < args.jog_seconds:
                jog = JointCommand()
                jog.stamp = node.get_clock().now().to_msg()
                jog.command_id = f'probe-{int((now - started) * 20)}'
                jog.joint_names = list(profile.joint_names)
                target = list(current)
                name = profile.joint_names[0]
                lower, upper = profile.joint_position_limits[name]
                direction = (-1.0 if args.withdraw_after is not None and
                             now - motion_started >= args.withdraw_after else 1.0)
                target[0] = min(upper, max(lower, target[0] + direction * .10))
                jog.positions = target
                jog.duration.sec = 1
                jog_pub.publish(jog)
                record('jog_intent', jog)
        if initial is None:
            raise RuntimeError('no joint state received')
        if motion_started is None:
            raise RuntimeError(f'readiness not achieved: {state["ready"]}')
        controls = state['controls']
        applied = state['applied']
        gaps = [b[0] - a[0] for a, b in zip(applied, applied[1:])]
        result.update(
            initial=initial, final=state['positions'],
            joint_1_displacement=state['positions'][0] - initial[0],
            control_count=len(controls), applied_count=len(applied),
            min_scale=min((entry[2] for entry in controls), default=None),
            reductions=sum(entry[1] == 'REDUCTION' for entry in controls),
            stops=sum(entry[1] == 'STOP' for entry in controls),
            max_applied_gap=max(gaps, default=None),
            max_measured_speed=max((abs(v) for _, v in state['velocities']), default=None),
            controls=controls[-10:],
        )
        # A reduction alone does not demonstrate velocity tracking or stopping.
        # Keep JACO2 inconclusive until those correlated checks are implemented.
        result['pass'] = bool(args.robot != 'jaco2' and applied and controls and result['reductions'])
        if args.robot == 'jaco2':
            result['validation_status'] = 'inconclusive: measured tracking and braking are not yet correlated'
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        if clock_paused:
            try:
                set_pause(False)
            except Exception:
                pass
        if frozen_tf_pid is not None:
            try:
                os.kill(frozen_tf_pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        if trace is not None:
            trace.close()
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
