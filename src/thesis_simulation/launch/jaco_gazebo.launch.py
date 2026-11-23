import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from thesis_core.work_table import SPHERE_HEIGHT, TABLE_CENTER, JACO2_SIM_TABLE_SIZE, MOUNT_HEIGHT
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from thesis_simulation.simulation_scene import spherical_obstacle_sdf


def _spawn_scene_obstacle(context, model_file):
    """Only the Gazebo source owns a physical sphere in this scene."""
    source = LaunchConfiguration('obstacle_source_mode').perform(context)
    if source != 'gazebo':
        return []
    radius = LaunchConfiguration('obstacle_radius').perform(context)
    sdf = spherical_obstacle_sdf(
        Path(model_file).read_text(encoding='utf-8'), radius,
    )
    return [Node(
        package='ros_gz_sim', executable='create', name='spawn_safety_obstacle',
        output='screen', arguments=[
            '-name', 'safety_obstacle', '-string', sdf,
            '-x', LaunchConfiguration('obstacle_x'),
            '-y', LaunchConfiguration('obstacle_y'),
            '-z', LaunchConfiguration('obstacle_z'),
        ],
    )]


def generate_launch_description():

    obstacle_x = LaunchConfiguration('obstacle_x')
    obstacle_y = LaunchConfiguration('obstacle_y')
    obstacle_z = LaunchConfiguration('obstacle_z')
    obstacle_source_mode = LaunchConfiguration('obstacle_source_mode')
    initial_pose = LaunchConfiguration('initial_pose')
    start_rviz = LaunchConfiguration('start_rviz')
    headless = LaunchConfiguration('headless')

    simulation_share = get_package_share_directory(
        'thesis_simulation'
    )
    core_share = get_package_share_directory('thesis_core')

    description_share = get_package_share_directory(
        'thesis_description'
    )

    # =========================================================
    # Gazebo Fortress resource and plugin paths
    # =========================================================

    gz_ros2_control_prefix = get_package_prefix(
        'gz_ros2_control'
    )

    system_plugin_paths = [
        os.path.join(
            gz_ros2_control_prefix,
            'lib'
        )
    ]

    existing_system_plugin_path = os.environ.get(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        ''
    )

    if existing_system_plugin_path:
        system_plugin_paths.append(
            existing_system_plugin_path
        )

    gazebo_system_plugin_path = os.pathsep.join(
        system_plugin_paths
    )

    resource_paths = [
        os.path.dirname(description_share),
        os.path.dirname(simulation_share),
    ]

    existing_resource_path = os.environ.get(
        'IGN_GAZEBO_RESOURCE_PATH',
        ''
    )

    if existing_resource_path:
        resource_paths.append(
            existing_resource_path
        )

    gazebo_resource_path = os.pathsep.join(
        resource_paths
    )

    xacro_file = os.path.join(
        simulation_share,
        'urdf',
        'jaco_gazebo.ros2_control.xacro'
    )

    world_file = os.path.join(
        simulation_share,
        'worlds',
        'jaco2_workstation.sdf'
    )

    obstacle_file = os.path.join(
        simulation_share,
        'models',
        'safety_obstacle',
        'model.sdf'
    )

    rviz_config = os.path.join(
        description_share,
        'rviz',
        'jaco.rviz'
    )

    capsule_config = os.path.join(
        core_share,
        'config',
        'jaco_capsules.yaml'
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                xacro_file,
                ' initial_pose:=',
                initial_pose,
                f' mount_height:={MOUNT_HEIGHT}',
            ]),
            value_type=str
        )
    }

    # =========================================================
    # Robot description
    # =========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            # Match the 100 Hz joint-state broadcaster so exact-stamp TF
            # consumers do not routinely wait for the default 20 Hz output.
            {'use_sim_time': True, 'publish_frequency': 100.0}
        ]
    )

    # =========================================================
    # Gazebo Fortress
    # =========================================================

    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-v',
            '3',
            world_file
        ],
        condition=UnlessCondition(headless),
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
                gazebo_system_plugin_path,

            'IGN_GAZEBO_RESOURCE_PATH':
                gazebo_resource_path,
        }
    )

    gazebo_headless = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-s',
            '-v',
            '3',
            world_file
        ],
        condition=IfCondition(headless),
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
                gazebo_system_plugin_path,

            'IGN_GAZEBO_RESOURCE_PATH':
                gazebo_resource_path,
        }
    )

    # =========================================================
    # Spawn JACO from /robot_description
    # =========================================================

    spawn_jaco = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_jaco',
        output='screen',
        arguments=[
            '-name',
            'jaco',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0'
        ]
    )

    spawn_obstacle = OpaqueFunction(
        function=_spawn_scene_obstacle,
        kwargs={'model_file': obstacle_file},
    )

    # =========================================================
    # Controllers
    # =========================================================

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    # =========================================================
    # RViz
    # =========================================================

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ]
    )

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_pose_bridge',
        condition=IfCondition(PythonExpression([
            "'", obstacle_source_mode, "' == 'gazebo'",
        ])),
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[(
            '/model/safety_obstacle/pose',
            '/world/jaco_world/pose/info',
        )],
        arguments=[
            (
                '/model/safety_obstacle/pose@'
                'tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(start_rviz),
        parameters=[{'use_sim_time': True}],
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    capsule_visualizer = Node(
        package='thesis_core',
        executable='capsule_visualizer',
        name='capsule_visualizer',
        output='screen',
        parameters=[
            capsule_config,
            {'use_sim_time': True, 'per_obstacle_markers': True}
        ]
    )

    proximity_monitor = Node(
        package='thesis_core',
        executable='proximity_monitor',
        name='proximity_monitor',
        output='screen',
        parameters=[
            capsule_config,
            {
                'use_sim_time': True,
                'obstacle_source_mode': obstacle_source_mode,
                'obstacle_input_topic':
                    '/thesis/obstacle_in_model_frame',
                'obstacle_x': ParameterValue(
                    obstacle_x,
                    value_type=float,
                ),
                'obstacle_y': ParameterValue(
                    obstacle_y,
                    value_type=float,
                ),
                'obstacle_z': ParameterValue(
                    obstacle_z,
                    value_type=float,
                ),
                'obstacle_radius': ParameterValue(
                    LaunchConfiguration('obstacle_radius'), value_type=float,
                ),
                'table_enabled': True,
                'table_with_legs': True,
                'table_center': list(TABLE_CENTER),
                'table_size': list(JACO2_SIM_TABLE_SIZE),
                'table_excluded_segments': [
                    'base_to_shoulder',
                    'shoulder_to_upper_arm',
                ],
            }
        ]
    )

    obstacle_input_bridge = Node(
        package='thesis_core',
        executable='obstacle_input_bridge',
        name='obstacle_input_bridge',
        condition=IfCondition(PythonExpression([
            "'", obstacle_source_mode, "' == 'topic'",
        ])),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/thesis/obstacle_input',
            'output_topic': '/thesis/obstacle_in_model_frame',
            'target_frame': 'world',
            'max_age_sec': 0.5,
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'obstacle_x',
            default_value='0.60',
            description='Gazebo safety obstacle X position in metres',
        ),
        DeclareLaunchArgument(
            'obstacle_y',
            default_value='0.0',
            description='Gazebo safety obstacle Y position in metres',
        ),
        DeclareLaunchArgument(
            'obstacle_z',
            default_value=str(SPHERE_HEIGHT),
            description='Gazebo safety obstacle Z position in metres',
        ),
        DeclareLaunchArgument(
            'obstacle_radius', default_value='0.12',
            description='Radius shared by the Gazebo sphere and its monitor.',
        ),
        DeclareLaunchArgument(
            'obstacle_source_mode',
            default_value='gazebo',
            choices=('gazebo', 'topic'),
            description=(
                'Obstacle source: gazebo (default) or topic (validated input)'
            ),
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='z_ready',
            choices=('default', 'z_ready'),
            description=(
                'Initial Gazebo joint pose preset; z_ready avoids the '
                'Cartesian-JOG singularity.'
            ),
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz for interactive simulation runs.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the Ignition Gazebo server without its GUI.',
        ),

        robot_state_publisher,

        gazebo,
        gazebo_headless,
        clock_bridge,
        pose_bridge,

        # Wait for Gazebo server before spawning robot
        TimerAction(
            period=2.0,
            actions=[
                spawn_jaco
            ]
        ),

        TimerAction(
            period=2.5,
            actions=[
                spawn_obstacle
            ]
        ),

        # gz_ros2_control creates controller_manager
        # when the robot is inserted.
        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                arm_controller_spawner
            ]
        ),

        rviz,
        capsule_visualizer,
        proximity_monitor,
        obstacle_input_bridge
    ])
