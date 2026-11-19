#!/usr/bin/env python3
"""Validate the unified GUI backend and Qt entry point against live Gazebo."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=92)
    parser.add_argument(
        '--robot',
        choices=('all', 'jaco2', 'ur17g', 'p3020', 'cs625'),
        default='all',
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['IGN_PARTITION'] = 'thesis_gui_validation_' + uuid.uuid4().hex
    runtime = Path(tempfile.mkdtemp(prefix='thesis_gui_validate_'))
    os.environ['ROS_LOG_DIR'] = str(runtime / 'ros_logs')
    launch_log = (args.output / 'launch.log').open('x')
    gui_log = (args.output / 'gui.log').open('x')
    launch = subprocess.Popen([
        'ros2', 'launch', 'thesis_multi_robot', 'cell.launch.py',
        f'robot:={args.robot}', 'headless:=true', 'supervision:=true',
        'demo_obstacle:=true', 'start_gui:=false',
        f'runtime_dir:={runtime}',
    ], stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    gui = None
    node = None
    result = {
        'domain': args.domain,
        'robot': args.robot,
        'runtime': str(runtime),
        'checks': [],
        'pass': False,
    }
    try:
        manifest_path = runtime / 'cell.json'
        deadline = time.monotonic() + 45.0
        while not manifest_path.exists():
            if launch.poll() is not None:
                raise RuntimeError('cell launch terminated before manifest')
            if time.monotonic() > deadline:
                raise TimeoutError('cell manifest was not generated')
            time.sleep(0.1)

        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from thesis_multi_robot.multi_robot_gui_node import MultiRobotGuiNode

        rclpy.init()
        node = MultiRobotGuiNode(str(manifest_path), use_sim_time=True)

        def spin_until(predicate, timeout, label):
            deadline = time.monotonic() + timeout
            while not predicate():
                if launch.poll() is not None:
                    raise RuntimeError(f'cell launch terminated while waiting: {label}')
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=0.05)

        spin_until(
            lambda: all(
                node.snapshot(name).state_fresh
                and node.snapshot(name).proximity_fresh
                and node.snapshot(name).proximity_state == 'ALLOW'
                for name in node.robot_names
            ),
            120.0,
            'fresh state and ALLOW for every GUI profile',
        )
        result['checks'].append(
            f'GUI backend discovered {len(node.robot_names)} fresh '
            'profile(s) in ALLOW'
        )
        # ALLOW can precede the arm action servers by a few seconds while the
        # controller spawners are still activating. Keep processing fresh
        # state before exercising the operator path.
        controller_settle = time.monotonic() + 6.0
        while time.monotonic() < controller_settle:
            rclpy.spin_once(node, timeout_sec=0.05)
        dofs = {
            name: len(node.profile(name).joint_names)
            for name in node.robot_names
        }
        all_dofs = {'jaco2': 6, 'ur17g': 6, 'p3020': 5, 'cs625': 6}
        expected_dofs = (
            all_dofs
            if args.robot == 'all'
            else {args.robot: all_dofs[args.robot]}
        )
        if dofs != expected_dofs:
            raise AssertionError(f'unexpected profile DOFs: {dofs}')
        result['dofs'] = dofs

        controller_clients = {
            name: node.create_client(
                ListControllers, f'/{name}/controller_manager/list_controllers'
            )
            for name in node.robot_names
        }
        for name, client in controller_clients.items():
            spin_until(
                client.service_is_ready,
                120.0,
                f'{name} controller manager service',
            )

            def controllers_active(client=client):
                future = client.call_async(ListControllers.Request())
                spin_until(future.done, 10.0, 'controller list response')
                active = {
                    controller.name
                    for controller in future.result().controller
                    if controller.state == 'active'
                }
                return {'joint_state_broadcaster', 'arm_controller'} <= active

            spin_until(
                controllers_active,
                120.0,
                f'{name} controllers active',
            )
        result['checks'].append('all profile controllers are active')

        def wait_reached(name, command_id, timeout, label):
            deadline = time.monotonic() + timeout
            while True:
                snapshot = node.snapshot(name)
                if (
                    snapshot.command_id == command_id
                    and snapshot.command_status == 'REACHED'
                ):
                    return snapshot
                if (
                    snapshot.command_id == command_id
                    and snapshot.command_status in {
                        'REJECTED', 'DECISION_TIMEOUT'
                    }
                ):
                    raise AssertionError(
                        f'{label}: {snapshot.command_status}; '
                        f'{snapshot.decision_state}: '
                        f'{snapshot.decision_reason}'
                    )
                if launch.poll() is not None:
                    raise RuntimeError(
                        f'cell launch terminated while waiting: {label}'
                    )
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f'{label}; status={snapshot.command_status}; '
                        f'decision={snapshot.decision_state}; '
                        f'reason={snapshot.decision_reason}; '
                        f'error={snapshot.maximum_target_error}'
                    )
                rclpy.spin_once(node, timeout_sec=0.05)

        for name in node.robot_names:
            initial = node.snapshot(name)
            start = tuple(initial.positions)
            other_ids = {
                peer: node.snapshot(peer).command_id
                for peer in node.robot_names if peer != name
            }
            command_id = node.send_joint_delta(name, 0, 0.02, 2.0)
            wait_reached(
                name, command_id, 30.0, f'{name} GUI joint target reached'
            )
            for peer, previous in other_ids.items():
                if node.snapshot(peer).command_id != previous:
                    raise AssertionError(f'{name} command leaked into {peer}')
            settle = time.monotonic() + 2.5
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=0.05)
            return_id = node.send_joint_target(name, start, 2.0)
            wait_reached(
                name, return_id, 30.0, f'{name} GUI return reached'
            )
            result['checks'].append(
                f'{name}: GUI candidate routed only to selected profile and returned'
            )
            settle = time.monotonic() + 2.5
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=0.05)

        gui_environment = os.environ.copy()
        gui_environment['QT_QPA_PLATFORM'] = 'offscreen'
        gui = subprocess.Popen([
            'ros2', 'run', 'thesis_multi_robot', 'multi_robot_gui',
            '--ros-args', '-p', 'use_sim_time:=true',
            '-p', f'manifest_path:={manifest_path}',
        ], stdout=gui_log, stderr=subprocess.STDOUT, env=gui_environment)
        gui_deadline = time.monotonic() + 5.0
        while time.monotonic() < gui_deadline:
            if gui.poll() is not None:
                raise RuntimeError(
                    f'Qt GUI exited during smoke test with code {gui.returncode}'
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        result['checks'].append(
            'Qt entry point stayed alive offscreen with the runtime manifest'
        )
        result['node_diagnostics'] = {
            name: node.diagnostics(name) for name in node.robot_names
        }
        result['pass'] = True
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
        if node is not None:
            result['node_diagnostics'] = {
                name: node.diagnostics(name) for name in node.robot_names
            }
            result['snapshots'] = {
                name: {
                    'state_fresh': node.snapshot(name).state_fresh,
                    'state_age': node.snapshot(name).state_age,
                    'proximity_state': node.snapshot(name).proximity_state,
                    'proximity_fresh': node.snapshot(name).proximity_fresh,
                    'command_status': node.snapshot(name).command_status,
                    'decision_state': node.snapshot(name).decision_state,
                    'decision_reason': node.snapshot(name).decision_reason,
                }
                for name in node.robot_names
            }
    finally:
        if gui is not None and gui.poll() is None:
            gui.send_signal(signal.SIGINT)
            try:
                gui.wait(timeout=8)
            except subprocess.TimeoutExpired:
                gui.kill()
                gui.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        if 'rclpy' in locals() and rclpy.ok():
            rclpy.shutdown()
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                launch.kill()
                launch.wait(timeout=5)
        launch_log.close()
        gui_log.close()
        (args.output / 'result.json').write_text(
            json.dumps(result, indent=2) + '\n'
        )
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
