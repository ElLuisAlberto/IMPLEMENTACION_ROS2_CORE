"""Robot-profile contracts shared by the historical simulation adapters."""

from types import SimpleNamespace

from thesis_simulation.simulation_command_adapter import (
    profile_motion_contract,
)
from thesis_simulation.system_readiness_node import (
    default_readiness_endpoints,
)


def profile(name='p3020', joint_count=5, namespace='/p3020'):
    """Return a small profile-shaped object for pure adapter tests."""
    names = tuple(f'{name}_joint_{index}' for index in range(joint_count))
    prefix = '' if namespace == '/' else namespace
    return SimpleNamespace(
        joint_names=names,
        joint_position_limits={joint: (-1.0, 1.0) for joint in names},
        joint_velocity_limits={joint: 0.25 for joint in names},
        continuous_joints=frozenset(),
        joint_states_topic=f'{prefix}/joint_states',
        controller_action=(
            f'{prefix}/arm_controller/follow_joint_trajectory'
        ),
        controller_paths=SimpleNamespace(
            namespace=namespace,
            controller_manager=f'{prefix}/controller_manager',
        ),
        world_name='thesis_cell',
    )


def test_motion_adapter_accepts_five_axis_profile():
    robot = profile()

    names, positions, velocities, continuous = profile_motion_contract(robot)

    assert len(names) == 5
    assert set(positions) == set(names)
    assert set(velocities) == set(names)
    assert not continuous


def test_readiness_endpoints_are_scoped_to_selected_robot():
    endpoints = default_readiness_endpoints(profile())

    assert endpoints['joint_states'] == '/p3020/joint_states'
    assert endpoints['proximity'] == '/p3020/thesis/proximity_status'
    assert endpoints['controller_service'] == (
        '/p3020/controller_manager/list_controllers'
    )
    assert endpoints['controller_action'] == (
        '/p3020/arm_controller/follow_joint_trajectory'
    )


def test_legacy_readiness_endpoints_remain_unnamespaced():
    endpoints = default_readiness_endpoints(
        profile(name='jaco2', joint_count=6, namespace='/')
    )

    assert endpoints['joint_states'] == '/joint_states'
    assert endpoints['proximity'] == '/thesis/proximity_status'
    assert endpoints['controller_service'] == (
        '/controller_manager/list_controllers'
    )
