from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('sistema_gripper_instrumentado_description'))
    robot_description = Command(['xacro ', str(share / 'urdf' / 'sistema_gripper_instrumentado_static.urdf.xacro')])
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config' / 'sistema_gripper.rviz')], output='screen'),
    ])
