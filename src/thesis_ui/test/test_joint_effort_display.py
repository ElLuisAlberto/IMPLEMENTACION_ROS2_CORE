"""Keep missing or malformed effort data distinct from zero torque."""

from types import SimpleNamespace

from sensor_msgs.msg import JointState
from thesis_ui.joint_control_gui import JointGuiNode


def test_joint_state_effort_is_named_and_cleared_when_missing():
    node = SimpleNamespace(
        joint_names=('j1', 'j2'), current_positions={}, current_velocities={},
        current_efforts={}, last_state_time=None,
    )
    message = JointState()
    message.name = ['j2', 'j1']
    message.position = [0.0, 0.1]
    message.effort = [2.5, -1.25]
    JointGuiNode.state_callback(node, message)
    assert node.current_efforts == {'j1': -1.25, 'j2': 2.5}

    message.effort = []
    JointGuiNode.state_callback(node, message)
    assert node.current_efforts == {}
