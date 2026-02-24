from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    manipulator_pkg = FindPackageShare('lab_manipulator')
    params_path = PathJoinSubstitution([manipulator_pkg, 'config', 'manipulator_params.yaml'])
    world_path = PathJoinSubstitution([manipulator_pkg, 'worlds', 'chassis_world.sdf'])
    print("WORLD PATH:", world_path)

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '4', world_path],
        output='screen'
    )

    lab_manipulator_node = Node(
        package='lab_manipulator',
        executable='lab_arm',
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        lab_manipulator_node,
    ])