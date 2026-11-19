"""
Offline launch for three Gazebo JACO arms plus one synthetic identity.

The launch has one Gazebo process and one /clock bridge.  arm_02 through
arm_04 each own one entity and controller manager.  arm_01 owns neither: its
joint state is generated deterministically only to provide the fourth logical
state/TF source required by the multi-arm safety campaign.
"""

from __future__ import annotations

import os
from pathlib import Path
import uuid

import yaml

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from thesis_simulation.hybrid_scenario import (
    GAZEBO_ROBOT_IDS,
    build_hybrid_three_sim_scenario,
)
from thesis_core.geometry_profile import (
    legacy_jaco_geometry_profile,
    offline_fixture_jaco_geometry_profile,
    synthetic_gripper_jaco_geometry_profile,
)


def _description(command):
    return {
        'robot_description': ParameterValue(Command(command), value_type=str),
        'use_sim_time': True,
    }


def _pose_arguments(pose):
    xyz = ' '.join(str(value) for value in pose[:3])
    rpy = ' '.join(str(value) for value in pose[3:])
    return xyz, rpy


def _manager_fqn(instance):
    return f'/{instance.namespace}/{instance.controller_manager_name}'


def _gazebo_environment(simulation_share, description_share):
    plugin_prefix = get_package_prefix('gz_ros2_control')
    plugin_paths = [os.path.join(plugin_prefix, 'lib')]
    current_plugins = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH')
    if current_plugins:
        plugin_paths.append(current_plugins)

    resource_paths = [
        os.path.dirname(description_share),
        os.path.dirname(simulation_share),
    ]
    current_resources = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')
    if current_resources:
        resource_paths.append(current_resources)
    return {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join(plugin_paths),
        'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join(resource_paths),
    }


def _load_scenario(path):
    with Path(path).open(encoding='utf-8') as stream:
        return build_hybrid_three_sim_scenario(yaml.safe_load(stream))


def _geometry_profile(instance):
    if instance.tool_mode == 'synthetic_gripper':
        return synthetic_gripper_jaco_geometry_profile(
            instance.robot_id, instance.tf_prefix
        )
    if instance.tool_mode == 'fixture':
        return offline_fixture_jaco_geometry_profile(
            instance.robot_id, instance.tf_prefix
        )
    return legacy_jaco_geometry_profile(
        instance.robot_id, frame_prefix=instance.tf_prefix
    )


def generate_launch_description():
    simulation_share = get_package_share_directory('thesis_simulation')
    description_share = get_package_share_directory('thesis_description')
    scenario_path = os.path.join(
        simulation_share, 'config', 'hybrid_three_sim.yaml'
    )
    scenario = _load_scenario(scenario_path)
    geometry_profiles = tuple(
        _geometry_profile(instance) for instance in scenario.instances
    )

    world_file = os.path.join(
        simulation_share, 'worlds', 'hybrid_three_jaco_empty.sdf'
    )
    gazebo_xacro = os.path.join(
        simulation_share, 'urdf', 'hybrid_jaco.ros2_control.xacro'
    )
    synthetic_xacro = os.path.join(
        simulation_share, 'urdf', 'hybrid_jaco.synthetic.xacro'
    )
    gazebo_environment = _gazebo_environment(
        simulation_share, description_share
    )

    headless = LaunchConfiguration('headless')
    initial_pose = LaunchConfiguration('initial_pose')
    simulation_output_enabled = LaunchConfiguration(
        'simulation_output_enabled'
    )
    session_id = LaunchConfiguration('session_id')

    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', world_file],
        condition=UnlessCondition(headless),
        output='screen',
        additional_env=gazebo_environment,
    )
    gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '-v', '3', world_file],
        condition=IfCondition(headless),
        output='screen',
        additional_env=gazebo_environment,
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='hybrid_gazebo_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    actions = [
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run the single Gazebo server without its GUI.',
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='default',
            choices=('default', 'z_ready'),
            description='Initial joint preset for the three Gazebo arms.',
        ),
        DeclareLaunchArgument(
            'simulation_output_enabled',
            default_value='false',
            description=(
                'Activate trajectory controllers only after campaign '
                'prerequisites are verified; false loads them inactive.'
            ),
        ),
        DeclareLaunchArgument(
            'session_id',
            default_value=f'hybrid-three-sim-{uuid.uuid4().hex}',
            description=(
                'Unique epoch shared by geometry, coordinator and gates. '
                'Normally leave the generated value unchanged.'
            ),
        ),
        gazebo_gui,
        gazebo_headless,
        clock_bridge,
    ]

    synthetic = scenario.instance('arm_01')
    synthetic_xyz, synthetic_rpy = _pose_arguments(synthetic.base_pose)
    synthetic_description = _description([
        'xacro ',
        synthetic_xacro,
        ' robot_name:=',
        f'{synthetic.robot_id}_jaco_synthetic',
        ' prefix:=',
        synthetic.tf_prefix,
        ' root_name:=',
        f'{synthetic.tf_prefix}_root',
        ' base_xyz:="',
        synthetic_xyz,
        '" base_rpy:="',
        synthetic_rpy,
        '" tool_mode:=',
        synthetic.tool_mode,
    ])
    actions.extend([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=synthetic.namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[synthetic_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=synthetic.namespace,
            name='synthetic_state_source',
            output='screen',
            parameters=[
                synthetic_description,
                {
                    'rate': 50,
                    'publish_default_positions': True,
                    'publish_default_velocities': True,
                },
            ],
        ),
        Node(
            package='thesis_simulation',
            executable='geometry_snapshot_source',
            namespace=synthetic.namespace,
            name='geometry_snapshot_source',
            output='screen',
            parameters=[{
                'robot_id': synthetic.robot_id,
                'source_kind': synthetic.source,
                'tf_prefix': synthetic.tf_prefix,
                'tool_mode': synthetic.tool_mode,
                'session_id': session_id,
                'joint_states_topic': (
                    f'/{synthetic.namespace}/joint_states'
                ),
                'geometry_topic': (
                    f'/{synthetic.namespace}/geometry_snapshot'
                ),
                'required_joint_names': [
                    f'{synthetic.tf_prefix}_joint_{index}'
                    for index in range(1, 7)
                ],
            }],
        ),
    ])

    for instance_index, robot_id in enumerate(GAZEBO_ROBOT_IDS):
        instance = scenario.instance(robot_id)
        xyz, rpy = _pose_arguments(instance.base_pose)
        controller_config = os.path.join(
            simulation_share, 'config', f'{robot_id}_controllers.yaml'
        )
        robot_description = _description([
            'xacro ',
            gazebo_xacro,
            ' robot_name:=',
            instance.entity_name,
            ' namespace:=',
            instance.namespace,
            ' prefix:=',
            instance.tf_prefix,
            ' root_name:=',
            f'{instance.tf_prefix}_root',
            ' base_xyz:="',
            xyz,
            '" base_rpy:="',
            rpy,
            '" tool_mode:=',
            instance.tool_mode,
            ' initial_pose:=',
            initial_pose,
            # The plugin already applies its <ros><namespace>.  A relative
            # node name resolves to /arm_N/robot_state_publisher; an absolute
            # path would be prefixed again by the Humble plugin.
            ' robot_param_node:=robot_state_publisher',
            ' controller_manager_name:=',
            instance.controller_manager_name,
            ' controller_config:=',
            controller_config,
        ])

        state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=instance.namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        )
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_id}',
            output='screen',
            arguments=[
                '-name',
                instance.entity_name,
                '-topic',
                f'/{instance.namespace}/robot_description',
                '-x',
                '0.0',
                '-y',
                '0.0',
                '-z',
                '0.0',
            ],
        )
        manager_fqn = _manager_fqn(instance)
        state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawn_{robot_id}_joint_state_broadcaster',
            output='screen',
            arguments=[
                f'{robot_id}_joint_state_broadcaster',
                '--controller-manager',
                manager_fqn,
                '--controller-manager-timeout',
                '60',
                '--service-call-timeout',
                '10',
            ],
        )
        active_controller = Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawn_{robot_id}_arm_controller_active',
            output='screen',
            condition=IfCondition(simulation_output_enabled),
            arguments=[
                instance.arm_controller_name,
                '--controller-manager',
                manager_fqn,
                '--controller-manager-timeout',
                '60',
                '--service-call-timeout',
                '10',
            ],
        )
        inactive_controller = Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawn_{robot_id}_arm_controller_inactive',
            output='screen',
            condition=UnlessCondition(simulation_output_enabled),
            arguments=[
                instance.arm_controller_name,
                '--inactive',
                '--controller-manager',
                manager_fqn,
                '--controller-manager-timeout',
                '60',
                '--service-call-timeout',
                '10',
            ],
        )
        command_adapter = Node(
            package='thesis_simulation',
            executable='simulation_command_adapter',
            namespace=instance.namespace,
            name='simulation_command_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'simulation_output_enabled': ParameterValue(
                    simulation_output_enabled,
                    value_type=bool,
                ),
                'joint_names': [
                    f'{instance.tf_prefix}_joint_{index}'
                    for index in range(1, 7)
                ],
                'action_name': (
                    f'/{instance.namespace}/'
                    f'{instance.arm_controller_name}/'
                    'follow_joint_trajectory'
                ),
                'joint_states_topic': (
                    f'/{instance.namespace}/joint_states'
                ),
                'controller_state_topic': (
                    f'/{instance.namespace}/'
                    f'{instance.arm_controller_name}/controller_state'
                ),
                'supervised_command_topic': (
                    f'/{instance.namespace}/thesis/supervised_command'
                ),
                'execution_control_topic': (
                    f'/{instance.namespace}/thesis/execution_control'
                ),
                'execution_trajectory_topic': (
                    f'/{instance.namespace}/thesis/execution_trajectory'
                ),
                'jog_output_topic': (
                    f'/{instance.namespace}/'
                    f'{instance.arm_controller_name}/joint_trajectory'
                ),
                'pipeline_timing_topic': (
                    f'/{instance.namespace}/thesis/pipeline_timing'
                ),
                'supervised_jog_command_topic': (
                    f'/{instance.namespace}/thesis/'
                    'supervised_jog_command'
                ),
                'global_safety_required': True,
                'global_safety_topic': (
                    '/thesis/multiarm_safety_status'
                ),
                'global_safety_session_id': session_id,
                'global_safety_max_receive_age_sec': 0.20,
            }],
        )
        geometry_source = Node(
            package='thesis_simulation',
            executable='geometry_snapshot_source',
            namespace=instance.namespace,
            name='geometry_snapshot_source',
            output='screen',
            parameters=[{
                'robot_id': instance.robot_id,
                'source_kind': instance.source,
                'tf_prefix': instance.tf_prefix,
                'tool_mode': instance.tool_mode,
                'session_id': session_id,
                'joint_states_topic': (
                    f'/{instance.namespace}/joint_states'
                ),
                'geometry_topic': (
                    f'/{instance.namespace}/geometry_snapshot'
                ),
                'required_joint_names': [
                    f'{instance.tf_prefix}_joint_{index}'
                    for index in range(1, 7)
                ],
            }],
        )
        global_gate = Node(
            package='thesis_simulation',
            executable='global_safety_gate',
            namespace=instance.namespace,
            name='global_safety_gate',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id': instance.robot_id,
                'expected_session_id': session_id,
            }],
        )

        actions.extend([
            state_publisher,
            command_adapter,
            geometry_source,
            global_gate,
            # Entity insertion is staggered because all three ros2_control
            # plugins share the Gazebo server process and plugin loader.
            # Controller startup itself remains service/event driven below.
            TimerAction(
                period=2.0 + 6.0 * instance_index,
                actions=[spawn],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=state_broadcaster,
                    on_exit=[active_controller, inactive_controller],
                )
            ),
        ])

    actions.append(Node(
        package='thesis_core',
        executable='multiarm_coordinator',
        name='multiarm_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'session_id': session_id,
            'geometry_topics': [
                f'/{robot_id}/geometry_snapshot'
                for robot_id in ('arm_01', 'arm_02', 'arm_03', 'arm_04')
            ],
            'expected_source_kinds': [
                instance.source for instance in scenario.instances
            ],
            'expected_profile_ids': [
                profile.profile_id for profile in geometry_profiles
            ],
            'expected_geometry_versions': [
                profile.geometry_version for profile in geometry_profiles
            ],
            'expected_asset_hashes': [
                profile.asset_hash for profile in geometry_profiles
            ],
        }],
    ))
    actions.append(Node(
        package='thesis_simulation',
        executable='hybrid_readiness',
        name='hybrid_readiness',
        output='screen',
        parameters=[{
            'simulation_output_enabled': ParameterValue(
                simulation_output_enabled,
                value_type=bool,
            ),
            'expected_session_id': session_id,
        }],
    ))

    return LaunchDescription(actions)
