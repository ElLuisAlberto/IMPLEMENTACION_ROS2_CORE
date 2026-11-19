"""RViz-only preview of the static CAD with an uncalibrated tool mount.

No simulator, arm controller, supervisor or robot command interface is started.
"""
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _preview(context):
    from thesis_multi_robot.profiles import build_robot

    profile = LaunchConfiguration('robot').perform(context)
    directory = tempfile.mkdtemp(prefix='thesis_gripper_preview_')
    model = build_robot(profile, profile, directory, provisional_gripper=True)
    description = Path(model['urdf']).read_text()
    rviz = Path(get_package_share_directory('thesis_multi_robot')) / 'rviz'
    config = next(rviz.glob('*.rviz'))
    return [
        LogInfo(msg='PREVIEW ONLY: uncalibrated static gripper, no safety or commands.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'robot_description': description}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(config)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur17g',
                              choices=['ur17g', 'p3020', 'cs625']),
        OpaqueFunction(function=_preview),
    ])
