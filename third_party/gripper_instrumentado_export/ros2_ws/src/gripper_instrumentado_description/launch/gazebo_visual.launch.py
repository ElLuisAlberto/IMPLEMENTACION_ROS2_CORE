import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    share = Path(get_package_share_directory('gripper_instrumentado_description'))
    model_root = share / 'models'
    world = share / 'worlds' / 'gripper_visual.sdf'
    current_resources = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = str(model_root) + os.pathsep + current_resources

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', str(world)],
            output='screen',
        ),
    ])
