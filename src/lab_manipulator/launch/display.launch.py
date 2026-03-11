from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    urdf_file = os.path.join(
        os.getenv('HOME'),
        'lab_boys/src/lab_manipulator/urdf/my_robot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])