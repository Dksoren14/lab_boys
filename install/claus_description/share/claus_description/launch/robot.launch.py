from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('claus_description')

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'a300',
        'a300.urdf.xacro'
    )

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command([
                    "xacro ",
                    xacro_file
                ])
            }]
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "a300",
                "-topic", "robot_description",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "1.0",
                "-R", "0.0",
                "-P", "0.0",
                "-Y", "1.57"
            ]
        )

    ])