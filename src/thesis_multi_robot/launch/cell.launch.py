"""Three independent nominal robot profiles in a single Fortress world."""
import copy
import xml.etree.ElementTree as ET
import json
import os
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from thesis_core.work_table import SPHERE_HEIGHT, MOUNT_HEIGHT, SIM_TABLE_SIZE
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def historical_remappings(name):
    """Map the unnamespaced historical ROS contract to one robot profile."""
    prefix = f'/{name}'
    topics = (
        '/joint_states',
        '/arm_controller/controller_state',
        '/arm_controller/joint_trajectory',
        '/arm_controller/follow_joint_trajectory',
        '/robot_description',
        '/thesis/system_ready',
        '/thesis/system_readiness',
        '/thesis/kinematic_tf_status',
        '/thesis/preview_intent',
        '/thesis/jog_intent',
        '/thesis/cartesian_jog_intent',
        '/thesis/candidate_command',
        '/thesis/cartesian_candidate',
        '/thesis/cartesian_trajectory_candidate',
        '/thesis/cartesian_trajectory_cancel',
        '/thesis/cartesian_trajectory_cancel/status',
        '/thesis/cartesian_trajectory_status',
        '/thesis/supervised_command',
        '/thesis/supervised_jog_command',
        '/thesis/applied_jog_command',
        '/thesis/trajectory_prediction',
        '/thesis/joint_trajectory_prediction',
        '/thesis/command_decision',
        '/thesis/execution_control',
        '/thesis/execution_trajectory',
        '/thesis/pipeline_timing',
        '/thesis/proximity_status',
        '/thesis/horizon_volume',
        '/thesis/robot_capsules',
        '/thesis/joint_link_skeleton',
        '/thesis/obstacle_visualization',
        '/thesis/obstacle_input',
        '/thesis/obstacle_in_model_frame',
        '/thesis/hardware/connected',
        '/thesis/hardware/armed',
        '/thesis/hardware/arm_epoch',
        '/thesis/hardware/arm_request',
        '/thesis/hardware/set_armed',
    )
    return [(topic, prefix + topic) for topic in topics]


def launch_cell(context):
    from thesis_multi_robot.profiles import PROFILES, build_cell
    selected = LaunchConfiguration('robot').perform(context)
    if selected != 'all' and selected not in PROFILES:
        raise ValueError('robot must be all, jaco2, ur17g, p3020 or cs625')
    headless = (
        LaunchConfiguration('headless').perform(context).lower() == 'true'
    )
    supervision = (
        LaunchConfiguration('supervision').perform(context).lower()
        == 'true'
    )
    historical = (
        LaunchConfiguration('historical_stack').perform(context).lower()
        == 'true'
    )
    if historical and selected == 'all':
        raise ValueError(
            'historical_stack requires one independently launched robot'
        )
    if historical and selected == 'jaco2':
        raise ValueError(
            'Use historical_robot.launch.py for JACO2 so its exact legacy '
            'simulation and safety stack remains active'
        )
    requested_output = LaunchConfiguration('runtime_dir').perform(context)
    generated_output = not requested_output
    output = requested_output or tempfile.mkdtemp(prefix='thesis_cell_')
    visual_cache = (
        Path(tempfile.gettempdir()) / 'thesis_multi_robot_visual_cache'
        if generated_output
        else Path(output) / 'visual_cache'
    )
    try:
        manifest_path = build_cell(
            output,
            PROFILES if selected == 'all' else (selected,),
            single=selected != 'all',
            visual_cache_dir=visual_cache,
        )
    except Exception:
        if generated_output:
            shutil.rmtree(output, ignore_errors=True)
        raise
    manifest = json.loads(manifest_path.read_text())
    start_gui = (
        LaunchConfiguration('start_gui').perform(context).lower() == 'true'
    )
    start_rviz = (
        LaunchConfiguration('start_rviz').perform(context).lower() == 'true'
    )
    share = get_package_share_directory('thesis_multi_robot')
    plugin_dir = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    resource_paths = list(dict.fromkeys(
        path
        for profile in manifest['robots'].values()
        for path in profile.get('resource_search_paths', [])
    ))

    def prepend_resource_paths(variable):
        existing = os.environ.get(variable, '')
        return os.pathsep.join(
            resource_paths + ([existing] if existing else [])
        )

    env = {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': (
            plugin_dir
            + os.pathsep
            + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
        ),
        # SDFormat translates package:// URIs to model://.  Fortress then
        # resolves the package directory through its resource search path.
        # Keep both names because IGN_* is the Fortress-era spelling while
        # GZ_SIM_* is used by newer Gazebo releases.
        'IGN_GAZEBO_RESOURCE_PATH': prepend_resource_paths(
            'IGN_GAZEBO_RESOURCE_PATH'
        ),
        'GZ_SIM_RESOURCE_PATH': prepend_resource_paths(
            'GZ_SIM_RESOURCE_PATH'
        ),
    }
    cmd = ['ign', 'gazebo', '-r', '-v', '2']
    if headless:
        cmd.append('-s')
    # Generate the active stations from the same SDF used by standalone JACO2.
    assets = Path(get_package_share_directory('thesis_simulation'))
    # Individual arms share the same laboratory as the standalone JACO2.
    scene = ET.parse(
        assets / 'worlds/jaco2_workstation.sdf'
        if selected != 'all' else os.path.join(share, 'worlds/cell.sdf')
    )
    world = scene.getroot().find('world')
    world.set('name', 'thesis_cell')
    existing_table = world.find("model[@name='jaco_work_table']")
    if existing_table is not None:
        world.remove(existing_table)
    template = ET.parse(assets / 'worlds/jaco2_workstation.sdf').find(
        ".//model[@name='jaco_work_table']")
    for name, profile in manifest['robots'].items():
        table = copy.deepcopy(template)
        table.set('name', name + '_work_table')
        x, y, base_z = profile['base_xyz']
        riser = base_z - MOUNT_HEIGHT
        link = table.find('link')
        if riser > 1e-9:
            for kind in ('visual', 'collision'):
                block = ET.SubElement(link, kind, name='mount_adapter_' + kind)
                ET.SubElement(block, 'pose').text = f'0 0 {MOUNT_HEIGHT + riser/2} 0 0 0'
                box = ET.SubElement(ET.SubElement(block, 'geometry'), 'box')
                ET.SubElement(box, 'size').text = f'0.18 0.18 {riser}'
        ET.SubElement(table, 'pose').text = f'{x} {y} 0 0 0 0'
        world.append(table)
    demo_obstacle = LaunchConfiguration('demo_obstacle').perform(context).lower() == 'true'
    if (supervision or historical) and demo_obstacle:
        from thesis_simulation.simulation_scene import spherical_obstacle_sdf
        radius = float(LaunchConfiguration('obstacle_radius').perform(context))
        sphere = ET.fromstring(spherical_obstacle_sdf(
            (assets / 'models/safety_obstacle/model.sdf').read_text(), radius)).find('model')
        xyz = [LaunchConfiguration('obstacle_' + a).perform(context) for a in 'xyz']
        ET.SubElement(sphere, 'pose').text = ' '.join(xyz + ['0', '0', '0'])
        world.append(sphere)
    scene_path = Path(output) / 'cell_world.sdf'
    scene.write(scene_path, encoding='utf-8')
    cmd.append(str(scene_path))
    gazebo = ExecuteProcess(cmd=cmd, additional_env=env, output='screen')
    actions = [
        gazebo,
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))],
        )),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cell_clock',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
            output='screen',
        ),
    ]
    if generated_output:
        def cleanup_generated_runtime(_event, _context):
            shutil.rmtree(output, ignore_errors=True)

        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=cleanup_generated_runtime,
        )))
    controller_pipelines = []
    for name, profile in manifest['robots'].items():
        description = Path(profile['urdf']).read_text()
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': description,
                'use_sim_time': True,
            }],
            output='screen',
        )
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_' + name,
            arguments=[
                '-world', 'thesis_cell', '-name', name,
                '-file', profile['urdf'],
            ],
            output='screen',
        )
        broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=name,
            arguments=[
                'joint_state_broadcaster',
                '-c', f'/{name}/controller_manager',
                '--controller-manager-timeout', '120',
            ],
            output='screen',
        )
        controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=name,
            arguments=[
                'arm_controller',
                '-c', f'/{name}/controller_manager',
                '-p', profile['controller_config'],
                '--controller-manager-timeout', '120',
            ],
            output='screen',
        )
        actions.append(rsp)
        controller_pipelines.append((spawn, broadcaster, controller))
        if supervision and not historical:
            actions.append(Node(
                package='thesis_multi_robot',
                executable='profile_supervisor',
                namespace=name,
                name='profile_supervisor',
                parameters=[{
                    'use_sim_time': True,
                    'manifest_path': str(manifest_path),
                    'robot_name': name,
                }],
                output='screen',
            ))
    if (supervision or historical) and demo_obstacle:
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='cell_obstacle_bridge',
            arguments=['/model/safety_obstacle/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        ))
        actions.append(Node(
            package='thesis_multi_robot',
            executable='demo_obstacle',
            name='cell_demo_obstacle',
            parameters=[{
                'use_sim_time': True,
                'track_gazebo': True,
                'robot_names': list(manifest['robots']),
                'x': float(LaunchConfiguration('obstacle_x').perform(context)),
                'y': float(LaunchConfiguration('obstacle_y').perform(context)),
                'z': float(LaunchConfiguration('obstacle_z').perform(context)),
                'radius': float(
                    LaunchConfiguration('obstacle_radius').perform(context)
                ),
            }],
            output='screen',
        ))
    # gz_ros2_control loads each model plugin synchronously and asks its
    # robot_state_publisher for robot_description.  Loading four plugins at
    # once can race DDS service discovery and stall Gazebo before any
    # controller manager exists.  Serialize the complete spawn/controller
    # chain while keeping all state publishers and supervisors alive.
    for index, (spawn, broadcaster, controller) in enumerate(
        controller_pipelines
    ):
        actions.extend([
            RegisterEventHandler(OnProcessExit(
                target_action=spawn,
                on_exit=[broadcaster],
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=broadcaster,
                on_exit=[controller],
            )),
        ])
        if index + 1 < len(controller_pipelines):
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=controller,
                on_exit=[controller_pipelines[index + 1][0]],
            )))
    if controller_pipelines:
        actions.append(controller_pipelines[0][0])
    if historical:
        name = selected
        common_parameters = {
            'use_sim_time': True,
            'manifest_path': str(manifest_path),
            'robot_name': name,
        }
        remappings = historical_remappings(name)
        actions.extend([
            Node(
                package='thesis_core',
                executable='profile_proximity_monitor',
                namespace=name,
                name='profile_proximity_monitor',
                parameters=[{**common_parameters, 'table_enabled': True,
                             'table_size': list(SIM_TABLE_SIZE), 'table_with_legs': True}],
                output='screen',
            ),
            Node(
                package='thesis_core',
                executable='safety_supervisor',
                namespace=name,
                name='safety_supervisor_node',
                parameters=[{
                    **common_parameters,
                    'table_enabled': True,
                    'table_size': list(SIM_TABLE_SIZE),
                    'table_with_legs': True,
                    'require_proximity_status': True,
                }],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='thesis_simulation',
                executable='simulation_command_adapter',
                namespace=name,
                name='simulation_command_adapter',
                parameters=[{
                    **common_parameters,
                    'simulation_output_enabled': True,
                }],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='thesis_core',
                executable='horizon_preview',
                namespace=name,
                name='horizon_preview',
                parameters=[{
                    'use_sim_time': True,
                    'profile_manifest': str(manifest_path),
                    'robot_name': name,
                    'source': 'auto',
                }],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='thesis_simulation',
                executable='cartesian_command_adapter',
                namespace=name,
                name='cartesian_command_adapter',
                parameters=[common_parameters],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='thesis_simulation',
                executable='cartesian_jog_adapter',
                namespace=name,
                name='cartesian_jog_adapter',
                parameters=[common_parameters],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='thesis_simulation',
                executable='cartesian_trajectory_adapter',
                namespace=name,
                name='cartesian_trajectory_adapter',
                parameters=[common_parameters],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='thesis_simulation',
                executable='system_readiness',
                namespace=name,
                name='system_readiness',
                parameters=[{
                    **common_parameters,
                    'use_sim_time': False,
                    'obstacle_source_mode': 'topic',
                    'obstacle_input_topic': f'/{name}/thesis/obstacle',
                    'use_demo_obstacle': demo_obstacle,
                    'require_gui': start_gui,
                }],
                remappings=remappings,
                output='screen',
            ),
        ])
        if start_gui:
            actions.append(Node(
                package='thesis_ui',
                executable='joint_gui',
                namespace=name,
                name='joint_control_gui_node',
                parameters=[{
                    **common_parameters,
                    'require_system_readiness': True,
                    'operation_mode': 'simulation',
                    'allow_hardware_arm_control': False,
                }],
                remappings=remappings,
                output='screen',
            ))
        if start_rviz:
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                namespace=name,
                name='rviz2',
                parameters=[{'use_sim_time': True}],
                arguments=['-d', os.path.join(share, 'rviz/profile.rviz')],
                remappings=remappings,
                output='screen',
            ))
    elif start_gui:
        actions.append(Node(
            package='thesis_multi_robot',
            executable='multi_robot_gui',
            name='multi_robot_gui',
            parameters=[{
                'use_sim_time': True,
                'manifest_path': str(manifest_path),
            }],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='all'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('supervision', default_value='true'),
        DeclareLaunchArgument('historical_stack', default_value='false'),
        DeclareLaunchArgument('demo_obstacle', default_value='true'),
        DeclareLaunchArgument('start_gui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('obstacle_x', default_value='0.60'),
        DeclareLaunchArgument('obstacle_y', default_value='0.0'),
        DeclareLaunchArgument('obstacle_z', default_value=str(SPHERE_HEIGHT)),
        DeclareLaunchArgument('obstacle_radius', default_value='0.12'),
        DeclareLaunchArgument('runtime_dir', default_value=''),
        OpaqueFunction(function=launch_cell),
    ])
