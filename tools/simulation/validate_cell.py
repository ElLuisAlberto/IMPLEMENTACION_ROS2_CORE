#!/usr/bin/env python3
"""Start an isolated simulation and test states, actions and admission.

Run after sourcing Humble and install/setup.bash. All ROS traffic uses a
dedicated domain and all child processes are stopped on success or failure.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['all', 'jaco2', 'ur17g', 'p3020', 'cs625'], default='all')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=88)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['IGN_PARTITION'] = 'thesis_validation_' + uuid.uuid4().hex
    runtime = Path(tempfile.mkdtemp(prefix='thesis_validate_'))
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(runtime / 'ros_logs')
    log_path = args.output / f'{args.robot}_launch.log'
    if log_path.exists():
        raise FileExistsError(log_path)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from thesis_interfaces.msg import JointCommand, Obstacle, CommandDecision, ProximityStatus
    from controller_manager_msgs.srv import ListControllers
    from thesis_multi_robot.geometry import world_capsules
    from thesis_multi_robot.kinematics import RobotKinematics
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time

    log = log_path.open('x')
    process = subprocess.Popen([
        'ros2', 'launch', 'thesis_multi_robot', 'cell.launch.py',
        f'robot:={args.robot}', 'headless:=true', 'supervision:=true',
        'demo_obstacle:=false', f'runtime_dir:={runtime}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'robot': args.robot, 'domain': args.domain, 'runtime': str(runtime), 'checks': [], 'pass': False}
    node = None
    try:
        deadline = time.monotonic() + 45
        while not (runtime / 'cell.json').exists():
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Launch did not produce runtime manifest')
            time.sleep(.1)
        manifest = json.loads((runtime / 'cell.json').read_text())
        rclpy.init()
        node = Node('cell_validation', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        states, statuses, decisions = {}, {}, {}
        state_times = {}
        obstacles = {name: (0., -8., 1.) for name in manifest['robots']}
        pause_obstacles = set()
        publishers, commands, clients, subscriptions = {}, {}, {}, []
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        for name in manifest['robots']:
            def state_callback(message, name=name):
                states[name] = dict(zip(message.name, message.position))
                state_times[name] = time.monotonic()
            subscriptions.extend([
                node.create_subscription(JointState, f'/{name}/joint_states', state_callback, qos_profile_sensor_data),
                node.create_subscription(ProximityStatus, f'/{name}/thesis/proximity_status',
                                         lambda msg, name=name: statuses.__setitem__(name, msg), 10),
                node.create_subscription(CommandDecision, f'/{name}/thesis/command_decision',
                                         lambda msg: decisions.__setitem__(msg.command_id, msg), 10),
            ])
            publishers[name] = node.create_publisher(Obstacle, f'/{name}/thesis/obstacle', 10)
            commands[name] = node.create_publisher(JointCommand, f'/{name}/thesis/candidate_command', 10)
            clients[name] = node.create_client(ListControllers, f'/{name}/controller_manager/list_controllers')

        def publish_obstacles():
            for name, position in obstacles.items():
                if name in pause_obstacles:
                    continue
                msg = Obstacle()
                msg.header.frame_id = 'world'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.center.x, msg.center.y, msg.center.z = position
                msg.radius, msg.uncertainty = .12, .02
                publishers[name].publish(msg)

        timer = node.create_timer(.05, publish_obstacles)

        def spin_until(predicate, timeout=30., label='condition'):
            deadline = time.monotonic() + timeout
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Launch terminated while waiting for {label}')
                if time.monotonic() > deadline:
                    raise TimeoutError(f'{label}; statuses=' + repr({n: s.state for n, s in statuses.items()}))
                rclpy.spin_once(node, timeout_sec=.02)

        spin_until(lambda: all(n in states and time.monotonic()-state_times[n] < .5
                              and n in statuses and statuses[n].state == 'ALLOW'
                              for n in manifest['robots']), 120., 'fresh joint states and ALLOW for every robot')
        result['checks'].append('all robot states fresh; all supervisors ALLOW')
        for name, profile in manifest['robots'].items():
            client = clients[name]
            spin_until(client.service_is_ready, label=f'{name} controller service')
            future = client.call_async(ListControllers.Request())
            spin_until(future.done, label=f'{name} controller list')
            active = {c.name for c in future.result().controller if c.state == 'active'}
            controller_deadline = time.monotonic() + 60
            while not {'joint_state_broadcaster', 'arm_controller'} <= active:
                if time.monotonic() > controller_deadline:
                    raise AssertionError(f'{name}: inactive controllers {active}')
                future = client.call_async(ListControllers.Request())
                spin_until(future.done, label=f'{name} controller activation')
                active = {c.name for c in future.result().controller if c.state == 'active'}
                rclpy.spin_once(node, timeout_sec=.05)
            names = profile['joint_names']
            if not all(j in states[name] for j in names):
                raise AssertionError(f'{name}: joint namespace mismatch')
            model = RobotKinematics(ET.parse(profile['urdf']).getroot(), names)
            positions = tuple(states[name][j] for j in names)
            expected = model.forward(positions)[profile['tip_link']]
            spin_until(lambda: tf_buffer.can_transform('world', profile['tip_link'], Time()), label=f'{name} TF')
            actual = tf_buffer.lookup_transform('world', profile['tip_link'], Time()).transform.translation
            error = sum((a-b)**2 for a,b in zip(expected[:3, 3], (actual.x,actual.y,actual.z))) ** .5
            if error > .01:
                raise AssertionError(f'{name} FK/TF mismatch {error}')
            result['checks'].append(f'{name}: controllers active; {len(names)} joints; FK/TF error={error:.6f}m')

            def send(target, label, duration=1.5, stamp=None):
                msg = JointCommand()
                msg.command_id = name + '_' + label + '_' + uuid.uuid4().hex
                msg.stamp = stamp or node.get_clock().now().to_msg()
                msg.joint_names = names
                msg.positions = list(target)
                msg.duration.sec = int(duration)
                msg.duration.nanosec = round((duration-int(duration))*1e9)
                commands[name].publish(msg)
                spin_until(lambda: msg.command_id in decisions, 20., f'{name} {label} decision')
                return decisions[msg.command_id]

            target = list(positions)
            target[0] += .02
            decision = send(target, 'move')
            if not decision.accepted:
                raise AssertionError(f'{name} safe command rejected: {decision.reason}')
            spin_until(lambda: max(abs(states[name][j]-q) for j,q in zip(names,target)) < .008,
                       25., f'{name} movement tracking')
            # Wait in simulation time for the action terminal result before next command.
            end_stamp = node.get_clock().now().nanoseconds + 2_000_000_000
            spin_until(lambda: node.get_clock().now().nanoseconds > end_stamp, 20., f'{name} settling')
            decision = send(positions, 'return')
            if not decision.accepted:
                raise AssertionError(f'{name} return rejected: {decision.reason}')
            spin_until(lambda: max(abs(states[name][j]-q) for j,q in zip(names,positions)) < .008,
                       25., f'{name} return tracking')
            end_stamp = node.get_clock().now().nanoseconds + 2_000_000_000
            spin_until(lambda: node.get_clock().now().nanoseconds > end_stamp, 20., f'{name} return settling')
            result['checks'].append(f'{name}: supervised +0.02rad and return reached <0.008rad error')

            own = world_capsules(profile['capsules'], model.forward(positions))
            runtime_target = list(positions)
            runtime_target[0] += .08
            runtime_decision = send(runtime_target, 'runtime_stop', duration=4.)
            if not runtime_decision.accepted:
                raise AssertionError(f'{name} runtime test initial goal rejected: {runtime_decision.reason}')
            spin_until(lambda: abs(states[name][names[0]]-positions[0]) > .004,
                       15., f'{name} runtime test starts moving')
            obstacles[name] = tuple(own[-1]['start'])
            spin_until(lambda: decisions[runtime_decision.command_id].reason_code == 'RUNTIME_STOP',
                       15., f'{name} runtime cancellation')
            stopped_stamp = node.get_clock().now().nanoseconds + 500_000_000
            spin_until(lambda: node.get_clock().now().nanoseconds > stopped_stamp, 15., 'cancel settling')
            held = [states[name][j] for j in names]
            stopped_stamp = node.get_clock().now().nanoseconds + 1_000_000_000
            spin_until(lambda: node.get_clock().now().nanoseconds > stopped_stamp, 15., 'hold observation')
            drift = max(abs(states[name][j]-q) for j,q in zip(names,held))
            if drift > .003 or abs(states[name][names[0]]-runtime_target[0]) < .02:
                raise AssertionError(f'{name} runtime STOP did not hold; drift={drift}')
            result['checks'].append(f'{name}: active goal cancelled on obstacle; hold drift={drift:.6f}rad')

            obstacles[name] = tuple(own[-1]['start'])
            spin_until(lambda: statuses[name].state == 'STOP', 10., f'{name} collision STOP')
            decision = send(target, 'blocked')
            if decision.accepted or decision.state != 'STOP':
                raise AssertionError(f'{name} collision command was not stopped')
            result['checks'].append(f'{name}: obstacle overlap rejected with STOP')
            obstacles[name] = (0., -8., 1.)
            spin_until(lambda: statuses[name].state == 'ALLOW', label=f'{name} restore ALLOW')
            pause_obstacles.add(name)
            spin_until(lambda: statuses[name].state == 'STOP', 10., f'{name} stale obstacle STOP')
            decision = send(target, 'stale')
            if decision.accepted:
                raise AssertionError(f'{name} stale input command accepted')
            pause_obstacles.remove(name)
            spin_until(lambda: statuses[name].state == 'ALLOW', label=f'{name} restored source')
            result['checks'].append(f'{name}: stale obstacle rejected and recovery observed')
        if len(manifest['robots']) > 1:
            concurrent = {}
            for name, profile in manifest['robots'].items():
                target = [states[name][j] for j in profile['joint_names']]
                target[0] += .01
                msg = JointCommand()
                msg.command_id = name + '_concurrent_' + uuid.uuid4().hex
                msg.stamp = node.get_clock().now().to_msg()
                msg.joint_names = profile['joint_names']
                msg.positions = target
                msg.duration.sec = 2
                commands[name].publish(msg)
                concurrent[name] = (msg.command_id, target)
            spin_until(lambda: all(command_id in decisions for command_id, _ in concurrent.values()),
                       20., 'concurrent supervisor decisions')
            if not all(decisions[cid].accepted for cid, _ in concurrent.values()):
                raise AssertionError('Concurrent goals rejected: ' + repr({n: decisions[cid].reason
                    for n, (cid, _) in concurrent.items()}))
            spin_until(lambda: all(max(abs(states[name][joint]-q)
                for joint,q in zip(manifest['robots'][name]['joint_names'], target)) < .004
                for name,(_,target) in concurrent.items()), 30., 'simultaneous tracking')
            end_stamp = node.get_clock().now().nanoseconds + 2_000_000_000
            spin_until(lambda: node.get_clock().now().nanoseconds > end_stamp, 20., 'simultaneous settling')
            if any(not decisions[cid].accepted for cid,_ in concurrent.values()):
                raise AssertionError('Concurrent movement produced a runtime STOP')
            result['checks'].append('four concurrent supervised +0.01rad goals reached <0.004rad error without runtime STOP')
        cli_robot = next(reversed(manifest['robots']))
        command_tool = Path(__file__).with_name('command_robot.py')
        cli = subprocess.Popen(['python3', str(command_tool), '--robot', cli_robot,
                                '--joint-index', '1', '--delta', '.005', '--duration', '2'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            spin_until(lambda: cli.poll() is not None, 40., 'operator candidate tool')
            stdout, stderr = cli.communicate(timeout=2)
            if cli.returncode != 0 or not json.loads(stdout)['accepted']:
                raise AssertionError(f'Operator tool failed: {stdout} {stderr}')
            result['operator_cli'] = json.loads(stdout)
            result['checks'].append(f'operator command_robot.py read live {cli_robot} description and submitted an accepted supervised candidate')
        finally:
            if cli.poll() is None:
                cli.terminate()
                cli.wait(timeout=5)
        result['pass'] = True
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        if node is not None:
            node.destroy_node()
        if 'rclpy' in locals() and rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            # Let launch signal its children once; group SIGINT would hit each
            # child twice when launch also forwards the shutdown request.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        log.close()
        result_path = args.output / f'{args.robot}_result.json'
        with result_path.open('x') as stream:
            json.dump(result, stream, indent=2)
        print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
