"""Launch one profile with the complete historical safety/UI/RViz stack."""

import os

from ament_index_python.packages import get_package_share_directory
from thesis_core.work_table import SPHERE_HEIGHT
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def select_robot_launch(context):
    """Select exact legacy JACO2 or one profile-driven simulated arm."""
    selected = LaunchConfiguration('robot').perform(context)
    if selected == 'jaco2':
        simulation_share = get_package_share_directory('thesis_simulation')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                simulation_share, 'launch', 'thesis_system.launch.py'
            )),
            launch_arguments={
                'use_sim_time': 'true',
                'obstacle_source_mode': PythonExpression([
                    "'gazebo' if '", LaunchConfiguration('demo_obstacle'),
                    "'.lower() == 'true' else 'topic'",
                ]),
                'use_demo_obstacle': LaunchConfiguration('demo_obstacle'),
                'obstacle_x': LaunchConfiguration('obstacle_x'),
                'obstacle_y': LaunchConfiguration('obstacle_y'),
                'obstacle_z': LaunchConfiguration('obstacle_z'),
                'obstacle_radius': LaunchConfiguration('obstacle_radius'),
                'simulation_output_enabled': 'true',
                'start_gui': LaunchConfiguration('start_gui'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'headless': LaunchConfiguration('headless'),
            }.items(),
        )]

    share = get_package_share_directory('thesis_multi_robot')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'cell.launch.py')
        ),
        launch_arguments={
            'robot': selected,
            'headless': LaunchConfiguration('headless'),
            'supervision': 'false',
            'historical_stack': 'true',
            'demo_obstacle': LaunchConfiguration('demo_obstacle'),
            'start_gui': LaunchConfiguration('start_gui'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'runtime_dir': LaunchConfiguration('runtime_dir'),
            'obstacle_x': LaunchConfiguration('obstacle_x'),
            'obstacle_y': LaunchConfiguration('obstacle_y'),
            'obstacle_z': LaunchConfiguration('obstacle_z'),
            'obstacle_radius': LaunchConfiguration('obstacle_radius'),
        }.items(),
    )]


def generate_launch_description():
    """Expose one command for legacy JACO2 or a commercial profile."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='jaco2',
            choices=('jaco2', 'ur17g', 'p3020', 'cs625'),
            description='Robot profile launched independently.',
        ),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('demo_obstacle', default_value='true'),
        DeclareLaunchArgument('start_gui', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('runtime_dir', default_value=''),
        DeclareLaunchArgument('obstacle_x', default_value='0.60'),
        DeclareLaunchArgument('obstacle_y', default_value='0.0'),
        DeclareLaunchArgument('obstacle_z', default_value=str(SPHERE_HEIGHT)),
        DeclareLaunchArgument('obstacle_radius', default_value='0.12'),
        OpaqueFunction(function=select_robot_launch),
    ])
