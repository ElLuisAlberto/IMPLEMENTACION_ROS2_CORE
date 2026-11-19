from thesis_core.work_table import JACO2_SIM_TABLE_SIZE, MOUNT_HEIGHT
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulation_output_enabled = LaunchConfiguration(
        'simulation_output_enabled'
    )
    require_proximity_status = LaunchConfiguration(
        'require_proximity_status'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    mounted = LaunchConfiguration('mounted_table')
    return LaunchDescription([
        DeclareLaunchArgument('mounted_table', default_value='false'),
        DeclareLaunchArgument(
            'simulation_output_enabled',
            default_value='false',
            description=(
                'Habilita salida únicamente al controlador de Gazebo'
            ),
        ),

        DeclareLaunchArgument(
            'require_proximity_status',
            default_value='true',
            description=(
                'Reject commands when proximity status is unavailable'
            ),
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),

        Node(
            package='thesis_core',
            executable='safety_supervisor',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
                'require_proximity_status': ParameterValue(
                    require_proximity_status,
                    value_type=bool,
                ),
                'table_enabled': ParameterValue(mounted, value_type=bool),
                'table_size': list(JACO2_SIM_TABLE_SIZE),
                'table_with_legs': ParameterValue(mounted, value_type=bool),
                'mount_height': ParameterValue(PythonExpression([
                    str(MOUNT_HEIGHT), " if '", mounted, "'.lower() == 'true' else 0.0",
                ]), value_type=float),
                'runtime_recovery_required_samples': 5,
                'runtime_max_scale_increment': 0.10,
                'runtime_recovery_sample_period_sec': 0.10,
                'jog_command_timeout_sec': 0.25,
            }],
        ),

        Node(
            package='thesis_simulation',
            executable='simulation_command_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
                'simulation_output_enabled':
                    ParameterValue(
                        simulation_output_enabled,
                        value_type=bool,
                    ),
                'control_replan_cooldown_sec': 0.75,
                'jog_control_period_sec': 0.10,
                'jog_command_timeout_sec': 0.18,
            }],
        ),
    ])
