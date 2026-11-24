"""Static Gazebo preview of one arm with the uncalibrated CAD system.

No controller, supervisor, actuator interface or physical robot is started.
"""
import os
import tempfile
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from thesis_core.work_table import SIM_TABLE_SIZE
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    from thesis_multi_robot.profiles import build_robot

    profile = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    directory = Path(tempfile.mkdtemp(prefix='thesis_system_gazebo_'))
    model = build_robot(
        profile,
        profile,
        directory,
        xyz=(0.0, 0.0, 0.75),
        visual_cache_dir=directory / 'visual_cache',
        provisional_gripper=True,
    )
    manifest = {
        'schema_version': 1,
        'world': 'jaco_world',
        'robots': {profile: model},
        'scope': 'nominal_simulation',
        'supervision': 'experimental',
    }
    manifest_path = directory / 'ur17g_static_cell.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    model_path = Path(model['urdf'])
    root = ET.parse(model_path).getroot()
    mount_joint = root.find("joint[@name='ur17g_world_mount']")
    if mount_joint is not None:
        mount_origin = mount_joint.find('origin')
        if mount_origin is not None:
            mount_origin.set('xyz', '0.0 0.0 0.75')
    for control in root.findall('ros2_control'):
        root.remove(control)
    for gazebo in root.findall('gazebo'):
        if gazebo.find('plugin') is not None:
            root.remove(gazebo)
    ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
    model_path.write_text(ET.tostring(root, encoding='unicode'))
    description = model_path.read_text()

    share = Path(get_package_share_directory('thesis_multi_robot'))
    thesis_assets = Path(get_package_share_directory('thesis_simulation'))
    scene_path = thesis_assets / 'worlds' / 'jaco2_workstation.sdf'
    paths = os.pathsep.join(model['resource_search_paths'])
    env = {'IGN_GAZEBO_RESOURCE_PATH': paths + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
           'GZ_SIM_RESOURCE_PATH': paths + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')}
    command = ['ign', 'gazebo', '-r', '-v', '2']
    if headless:
        command.append('-s')
    command.append(str(scene_path))
    common_parameters = {
        'use_sim_time': False,
        'manifest_path': str(manifest_path),
        'robot_name': profile,
    }
    rviz_config = share / 'rviz' / 'profile.rviz'
    return [
        LogInfo(msg='STATIC UR17G: complete table, obstacle, capsules and horizon preview.'),
        ExecuteProcess(cmd=command, additional_env=env, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=profile,
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace=profile,
             parameters=[{
                 'robot_description': description,
                 'zeros': {
                     'ur17g_shoulder_pan_joint': 0.0,
                     'ur17g_shoulder_lift_joint': -1.15,
                     'ur17g_elbow_joint': 1.10,
                     'ur17g_wrist_1_joint': -1.52,
                     'ur17g_wrist_2_joint': 0.0,
                     'ur17g_wrist_3_joint': 0.0,
                 },
             }],
             output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'jaco_world', '-name', profile, '-file', str(model_path)],
             output='screen'),
        Node(
            package='thesis_core',
            executable='profile_proximity_monitor',
            namespace=profile,
            name='profile_proximity_monitor',
            parameters=[{
                **common_parameters,
                'table_enabled': True,
                'table_size': list(SIM_TABLE_SIZE),
                'table_with_legs': True,
                'state_topic': '/ur17g/joint_states',
                'obstacle_topic': '/ur17g/thesis/obstacle',
                'status_topic': '/ur17g/thesis/proximity_status',
                'capsule_marker_topic': '/ur17g/thesis/robot_capsules',
                'obstacle_marker_topic': '/ur17g/thesis/obstacle_visualization',
            }],
            output='screen',
        ),
        Node(
            package='thesis_core',
            executable='horizon_preview',
            namespace=profile,
            name='horizon_preview',
            parameters=[{
                **common_parameters,
                'profile_manifest': str(manifest_path),
                'source': 'auto',
                'joint_state_topic': '/ur17g/joint_states',
                'marker_topic': '/ur17g/thesis/horizon_volume',
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=profile,
            name='rviz',
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur17g', choices=['ur17g', 'p3020', 'cs625']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_launch),
    ])
