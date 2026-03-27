from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('claus_gazebo')
    world = os.path.join(pkg_path, 'worlds', 'sim_environment.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'
        )
    ])
