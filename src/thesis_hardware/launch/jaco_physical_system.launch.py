"""Start the complete preventive stack for the physical Kinova JACO2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    PythonExpression,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the physical stack with output disabled by default."""
    bridge_share = get_package_share_directory(
        'thesis_hardware_bridge'
    )
    description_share = get_package_share_directory(
        'thesis_description'
    )
    core_share = get_package_share_directory(
        'thesis_core'
    )

    default_hardware_config = os.path.join(
        bridge_share, 'config', 'jaco_hardware.yaml'
    )
    default_capsule_config = os.path.join(
        core_share, 'config', 'jaco_capsules.yaml'
    )
    default_obstacle_scenario_config = os.path.join(
        core_share, 'config', 'virtual_obstacle_scenarios.yaml'
    )
    xacro_file = os.path.join(
        description_share, 'urdf', 'j2n6s300_standalone.xacro'
    )
    rviz_file = os.path.join(
        description_share, 'rviz', 'jaco.rviz'
    )

    mock_hardware = LaunchConfiguration('mock_hardware')
    output_enabled = LaunchConfiguration('hardware_output_enabled')
    expected_serial = LaunchConfiguration('expected_serial_number')
    hardware_config = LaunchConfiguration('hardware_config_file')
    capsule_config = LaunchConfiguration('capsule_config_file')
    obstacle_scenario_config = LaunchConfiguration(
        'obstacle_scenario_config_file'
    )
    obstacle_input_topic = LaunchConfiguration('obstacle_input_topic')
    use_demo_obstacle = LaunchConfiguration('use_demo_obstacle')
    start_gui = LaunchConfiguration('start_gui')
    start_rviz = LaunchConfiguration('start_rviz')
    allow_gui_arm_control = LaunchConfiguration(
        'allow_gui_arm_control'
    )
    start_telemetry = LaunchConfiguration('start_telemetry')
    telemetry_output_directory = LaunchConfiguration(
        'telemetry_output_directory'
    )
    kinova_library_dir = LaunchConfiguration('kinova_library_dir')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        )
    }

    virtual_table = LaunchConfiguration('virtual_workstation')
    def preset(virtual, normal):
        return PythonExpression([repr(virtual), " if '", virtual_table,
                                 "'.lower() == 'true' else ", repr(normal)])

    table_parameters = {
        'table_enabled': ParameterValue(virtual_table, value_type=bool),
        # Physical world is at the arm base, i.e. at the virtual tabletop.
        'table_center': [0.0, 0.0, -0.02],
    }
    def demo_source(context):
        radius_override = {}
        if (virtual_table.perform(context) == 'true' and
                LaunchConfiguration('demo_obstacle_scenario').perform(context).upper() == 'FIXED'):
            radius_override['radius'] = 0.12
        return [Node(
            package='thesis_core',
            executable='obstacle_demo_source',
            name='obstacle_demo_source',
            output='screen',
            condition=IfCondition(use_demo_obstacle),
            parameters=[
                obstacle_scenario_config,
                radius_override,
                {
                    'use_sim_time': False,
                    'topic': obstacle_input_topic,
                    'scenario': LaunchConfiguration(
                        'demo_obstacle_scenario'
                    ),
                    'insert_stop_active': ParameterValue(
                        LaunchConfiguration('demo_insert_stop_active'),
                        value_type=bool,
                    ),
                    # Retained for FIXED compatibility with older launches.
                    'x': ParameterValue(
                        LaunchConfiguration('demo_obstacle_x'),
                        value_type=float,
                    ),
                    'y': ParameterValue(
                        LaunchConfiguration('demo_obstacle_y'),
                        value_type=float,
                    ),
                    'z': ParameterValue(
                        LaunchConfiguration('demo_obstacle_z'),
                        value_type=float,
                    ),
                },
            ],
        )]

    declarations = [
        DeclareLaunchArgument('virtual_workstation', default_value='false',
                              choices=('true', 'false'),
                              description='Use the base-relative virtual table and demo sphere.'),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Use an offline JACO state and motion backend.',
        ),
        DeclareLaunchArgument(
            'hardware_output_enabled',
            default_value='false',
            description=(
                'Permit runtime arming of physical or mock motion.'
            ),
        ),
        DeclareLaunchArgument(
            'expected_serial_number',
            default_value='',
            description='Exact serial required when more than one arm exists.',
        ),
        DeclareLaunchArgument(
            'hardware_config_file',
            default_value=default_hardware_config,
            description='JACO adapter ROS 2 parameter file.',
        ),
        DeclareLaunchArgument(
            'capsule_config_file',
            default_value=default_capsule_config,
            description='Physical-arm capsule geometry parameter file.',
        ),
        DeclareLaunchArgument(
            'obstacle_scenario_config_file',
            default_value=default_obstacle_scenario_config,
            description='Named single-sphere scenario parameter file.',
        ),
        DeclareLaunchArgument(
            'obstacle_input_topic',
            default_value='/thesis/obstacle_input',
            description='Obstacle input before transformation to world.',
        ),
        DeclareLaunchArgument(
            'use_demo_obstacle',
            default_value=preset('true', 'false'),
            description=(
                'Publish a synthetic obstacle. Use only for offline/bench '
                'tests.'
            ),
        ),
        DeclareLaunchArgument(
            'demo_obstacle_x',
            default_value=preset('0.60', '2.0'),
            description='Synthetic test obstacle X coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'demo_obstacle_y',
            default_value='0.0',
            description='Synthetic test obstacle Y coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'demo_obstacle_z',
            default_value=preset('0.52', '0.65'),
            description='Synthetic test obstacle Z coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'demo_obstacle_scenario',
            default_value=preset('FIXED', 'PARK'),
            description='FIXED, PARK, BLOCK_START or INSERT_STOP.',
        ),
        DeclareLaunchArgument(
            'demo_insert_stop_active',
            default_value='false',
            description=(
                'Move INSERT_STOP from PARK to its calibrated STOP pose.'
            ),
        ),
        DeclareLaunchArgument(
            'start_gui',
            default_value='false',
            description=(
                'Start the joint command interface. Disabled by default for '
                'read-only physical connection.'
            ),
        ),
        DeclareLaunchArgument(
            'allow_gui_arm_control',
            default_value='false',
            description=(
                'Show arm/disarm controls in the physical GUI. Runtime '
                'arming still requires hardware_output_enabled=true.'
            ),
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2.',
        ),
        DeclareLaunchArgument(
            'start_telemetry',
            default_value='true',
            description='Start the read-only CSV telemetry observer.',
        ),
        DeclareLaunchArgument(
            'telemetry_output_directory',
            default_value='~/thesis_telemetry',
            description='Parent directory for physical-session telemetry.',
        ),
        DeclareLaunchArgument(
            'kinova_library_dir',
            default_value=os.path.join(
                os.environ.get(
                    'KINOVA_ROOT',
                    os.path.expanduser(
                        '~/Escritorio/TESIS_2_DEPENDENCIES/'
                        'kinova-ros/kinova_driver'
                    ),
                ),
                'lib',
                'x86_64-linux-gnu',
            ),
            description=(
                'Directory containing the Kinova command and communication '
                'layer shared libraries.'
            ),
        ),
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {
                'use_sim_time': False,
                # The hardware adapter publishes measured joints at 20 Hz.
                # A 20 Hz cap here can skip every other sample due to normal
                # scheduling jitter, leaving exact obstacle-stamp TF lookups
                # without a bracketing transform.  Keep the cap above the
                # measured-state source rate; this changes TF distribution
                # only, never safety thresholds or fail-closed behavior.
                'publish_frequency': 40.0,
            }],
        ),
        Node(
            package='thesis_hardware_bridge',
            executable='jaco_hardware_node',
            name='jaco_hardware_adapter',
            output='screen',
            additional_env={
                'LD_LIBRARY_PATH': [
                    kinova_library_dir,
                    ':',
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH', default_value=''
                    ),
                ],
            },
            parameters=[
                hardware_config,
                {
                    'mock_hardware': ParameterValue(
                        mock_hardware, value_type=bool
                    ),
                    'hardware_output_enabled': ParameterValue(
                        output_enabled, value_type=bool
                    ),
                    'expected_serial_number': expected_serial,
                },
            ],
        ),
        Node(
            package='thesis_core',
            executable='obstacle_input_bridge',
            name='obstacle_input_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'input_topic': obstacle_input_topic,
                'output_topic': '/thesis/obstacle_in_model_frame',
                'target_frame': 'world',
                'max_age_sec': 0.5,
            }],
        ),
        OpaqueFunction(function=demo_source),
        Node(
            package='thesis_core',
            executable='proximity_monitor',
            name='proximity_monitor',
            output='screen',
            parameters=[
                capsule_config,
                table_parameters,
                {
                    'use_sim_time': False,
                    'obstacle_source_mode': 'topic',
                    'obstacle_input_topic':
                        '/thesis/obstacle_in_model_frame',
                },
            ],
        ),
        Node(
            package='thesis_core',
            executable='safety_supervisor',
            name='safety_supervisor_node',
            output='screen',
            parameters=[table_parameters, {
                'use_sim_time': False,
                'require_proximity_status': True,
                'runtime_recovery_required_samples': 5,
                'runtime_max_scale_increment': 0.10,
                'runtime_recovery_sample_period_sec': 0.10,
                'jog_command_timeout_sec': 0.25,
                'max_jog_message_age_sec': 0.15,
            }],
        ),
        Node(
            package='thesis_simulation',
            executable='cartesian_command_adapter',
            name='cartesian_command_adapter',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='thesis_simulation',
            executable='cartesian_jog_adapter',
            name='cartesian_jog_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'require_hardware_arm_epoch': True,
            }],
        ),
        Node(
            package='thesis_simulation',
            executable='cartesian_trajectory_adapter',
            name='cartesian_trajectory_adapter',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='thesis_core',
            executable='horizon_preview',
            name='horizon_preview',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'source': 'auto',
                'horizon_sec': 1.0,
                'samples': 21,
                'margin_m': 0.02,
                'rate_hz': 10.0,
            }],
        ),
        Node(
            package='thesis_core',
            executable='capsule_visualizer',
            name='capsule_visualizer',
            output='screen',
            parameters=[capsule_config, {'use_sim_time': False}],
        ),
        Node(
            package='thesis_hardware',
            executable='hardware_readiness',
            name='hardware_readiness',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'require_armed': False,
                'require_proximity_status': True,
                'timeout_sec': 0.50,
                'rate_hz': 2.0,
            }],
        ),
        Node(
            package='thesis_ui',
            executable='joint_gui',
            name='joint_control_gui_node',
            output='screen',
            condition=IfCondition(start_gui),
            parameters=[{
                'use_sim_time': False,
                'require_system_readiness': True,
                'operation_mode': 'hardware',
                'allow_hardware_arm_control': ParameterValue(
                    allow_gui_arm_control, value_type=bool
                ),
            }],
        ),
        Node(
            package='thesis_telemetry',
            executable='telemetry_logger',
            name='telemetry_logger',
            output='screen',
            condition=IfCondition(start_telemetry),
            parameters=[{
                'use_sim_time': False,
                'output_directory': telemetry_output_directory,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(start_rviz),
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_file],
        ),
    ]
    return LaunchDescription(declarations + nodes)
