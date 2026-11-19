import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    share = Path(get_package_share_directory('sistema_gripper_instrumentado_description'))
    resources = str(share / 'models') + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        ExecuteProcess(cmd=['gz', 'sim', '-r', str(share / 'worlds' / 'sistema_gripper_visual.sdf')], output='screen'),
    ])
