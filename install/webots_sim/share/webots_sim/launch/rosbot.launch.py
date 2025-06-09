from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    webots_world_path = os.path.join(
        get_package_share_directory('webots_sim'),
        'worlds',
        'null_world.wbt'  # replace with your actual world file
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['webots', webots_world_path],
            output='screen',
            name='webots_simulator',
        ),
    ])
