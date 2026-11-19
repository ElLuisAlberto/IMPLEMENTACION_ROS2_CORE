"""The visual reference reports the actual short controller command."""
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from thesis_simulation.simulation_command_adapter import SimulationCommandAdapter


def test_applied_reference_reports_short_target_and_command_id():
    output = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=10, nanosec=2))),
        joint_names=('axis_1', 'axis_2'),
        applied_jog_publisher=SimpleNamespace(publish=output.append),
    )
    SimulationCommandAdapter.publish_applied_jog(
        node, 'command-1', (0.1, -0.2), 0.1,
    )
    message = output[0]
    assert message.command_id == 'command-1'
    assert list(message.joint_names) == ['axis_1', 'axis_2']
    assert list(message.positions) == [0.1, -0.2]
    assert message.duration.sec == 0 and message.duration.nanosec == 100_000_000
    assert message.stamp.sec == 10
