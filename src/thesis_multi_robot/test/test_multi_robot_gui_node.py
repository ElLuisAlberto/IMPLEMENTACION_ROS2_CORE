"""Contracts for the profile-driven, simulation-only GUI backend."""

import json
import time

import pytest
import rclpy
from thesis_interfaces.msg import CommandDecision

from thesis_multi_robot.multi_robot_gui_node import (
    MAX_CARTESIAN_ORIENTATION_STEP,
    MAX_CARTESIAN_POSITION_STEP,
    MultiRobotGuiNode,
    _LiveState,
    _Proximity,
    _finite_vector,
    load_ui_profiles,
)
from thesis_multi_robot.profiles import build_cell


def test_loader_preserves_profile_dof_limits_and_tip(tmp_path):
    manifest = build_cell(tmp_path, profiles=('ur17g', 'p3020'), single=True)
    profiles = load_ui_profiles(manifest)
    assert tuple(profiles) == ('ur17g', 'p3020')
    assert len(profiles['ur17g'].joint_names) == 6
    assert len(profiles['p3020'].joint_names) == 5
    for profile in profiles.values():
        assert len(profile.joint_limits) == len(profile.joint_names)
        assert profile.tip_link in profile.model.forward(profile.initial_positions)
        assert all(lower < upper and velocity > 0.0
                   for lower, upper, velocity in profile.joint_limits)


def test_loader_rejects_an_empty_manifest(tmp_path):
    path = tmp_path / 'cell.json'
    path.write_text(json.dumps({'schema_version': 1, 'robots': {}}))
    with pytest.raises(ValueError, match='empty'):
        load_ui_profiles(path)


@pytest.mark.parametrize('values', [(0.0,), (0.0, float('nan'))])
def test_finite_vector_rejects_wrong_or_nonfinite_values(values):
    with pytest.raises(ValueError):
        _finite_vector(values, 2, 'target')


class _Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def gui_node(tmp_path):
    manifest = build_cell(tmp_path, profiles=('ur17g', 'p3020'), single=True)
    rclpy.init()
    node = None
    try:
        node = MultiRobotGuiNode(str(manifest))
        for name in node.robot_names:
            profile = node.profile(name)
            stamp = node.get_clock().now().nanoseconds
            node._states[name] = _LiveState(
                profile.initial_positions,
                (0.0,) * len(profile.joint_names),
                time.monotonic(),
                stamp,
            )
            node._proximity[name] = _Proximity(
                'ALLOW', 1.0, time.monotonic(), stamp
            )
            node._candidate_publishers[name] = _Publisher()
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_backend_routes_only_to_selected_profile(gui_node):
    target = list(gui_node.profile('ur17g').initial_positions)
    target[0] += 0.01
    command_id = gui_node.send_joint_target('ur17g', target, 2.0)
    assert command_id.startswith('gui-ur17g-')
    assert len(gui_node._candidate_publishers['ur17g'].messages) == 1
    assert not gui_node._candidate_publishers['p3020'].messages
    assert gui_node._candidate_publishers['ur17g'].messages[0].joint_names == list(
        gui_node.profile('ur17g').joint_names
    )


def test_backend_waits_for_controller_success_before_reached(gui_node):
    name = 'ur17g'
    target = list(gui_node.profile(name).initial_positions)
    target[0] += 0.01
    command_id = gui_node.send_joint_target(name, target, 2.0)
    state = gui_node._states[name]
    gui_node._states[name] = _LiveState(
        tuple(target),
        state.velocities,
        time.monotonic(),
        state.stamp_ns,
    )

    decision = CommandDecision()
    decision.command_id = command_id
    decision.accepted = True
    decision.state = 'ALLOW'
    decision.reason_code = 'SIMULATION_GOAL_ACCEPTED'
    decision.reason = decision.reason_code
    gui_node._decision_callback(name, decision)
    assert gui_node.snapshot(name).command_status == 'TRACKING'

    decision.reason_code = 'SIMULATION_GOAL_SUCCEEDED'
    decision.reason = decision.reason_code
    gui_node._decision_callback(name, decision)
    assert gui_node.snapshot(name).command_status == 'REACHED'


def test_backend_supports_five_axis_profile(gui_node):
    assert len(gui_node.profile('p3020').joint_names) == 5
    command_id = gui_node.send_joint_delta('p3020', 4, 0.01, 2.0)
    assert command_id.startswith('gui-p3020-')
    message = gui_node._candidate_publishers['p3020'].messages[0]
    assert len(message.positions) == 5


def test_backend_fails_closed_for_stop_or_stale_state(gui_node):
    state = gui_node._proximity['p3020']
    gui_node._proximity['p3020'] = _Proximity(
        'STOP', state.clearance, time.monotonic(), state.stamp_ns
    )
    with pytest.raises(RuntimeError, match='STOP'):
        gui_node.send_joint_delta('p3020', 0, 0.01, 2.0)
    gui_node._states['ur17g'].received_at -= 1.0
    with pytest.raises(RuntimeError, match='stale'):
        gui_node.send_joint_delta('ur17g', 0, 0.01, 2.0)


def test_backend_rejects_large_cartesian_steps(gui_node):
    snapshot = gui_node.snapshot('ur17g')
    xyz = list(snapshot.tool_xyz)
    xyz[0] += MAX_CARTESIAN_POSITION_STEP + 0.001
    with pytest.raises(ValueError, match='0.10 m'):
        gui_node.send_cartesian_target('ur17g', xyz, snapshot.tool_rpy, 2.0)

    rpy = list(snapshot.tool_rpy)
    rpy[2] += MAX_CARTESIAN_ORIENTATION_STEP + 0.01
    with pytest.raises(ValueError, match='20 degree'):
        gui_node.send_cartesian_target(
            'ur17g', snapshot.tool_xyz, rpy, 2.0
        )
