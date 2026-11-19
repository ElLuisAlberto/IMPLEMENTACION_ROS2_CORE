"""Static Gazebo preview of one arm with the uncalibrated CAD system.

No controller, supervisor, actuator interface or physical robot is started.
"""
import os
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    from thesis_multi_robot.profiles import build_robot

    profile = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    directory = Path(tempfile.mkdtemp(prefix='thesis_system_gazebo_'))
    model = build_robot(profile, profile, directory, provisional_gripper=True)
    model_path = Path(model['urdf'])
    root = ET.parse(model_path).getroot()
    for control in root.findall('ros2_control'):
        root.remove(control)
    for gazebo in root.findall('gazebo'):
        if gazebo.find('plugin') is not None:
            root.remove(gazebo)
    ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
    model_path.write_text(ET.tostring(root, encoding='unicode'))
    description = model_path.read_text()
    share = Path(get_package_share_directory('thesis_multi_robot'))
    paths = os.pathsep.join(model['resource_search_paths'])
    env = {'IGN_GAZEBO_RESOURCE_PATH': paths + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
           'GZ_SIM_RESOURCE_PATH': paths + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')}
    command = ['ign', 'gazebo', '-r', '-v', '2']
    if headless:
        command.append('-s')
    command.append(str(share / 'worlds' / 'cell.sdf'))
    return [
        LogInfo(msg='PREVIEW ONLY: static CAD, estimated mount, no control or safety.'),
        ExecuteProcess(cmd=command, additional_env=env, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'thesis_cell', '-name', profile, '-file', str(model_path)],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur17g', choices=['ur17g', 'p3020', 'cs625']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_launch),
    ])
