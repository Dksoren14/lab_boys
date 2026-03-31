from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get package paths (portable)
    gazebo_pkg = get_package_share_directory('claus_gazebo')

    # World file
    world = os.path.join(gazebo_pkg, 'worlds', 'sim_environment.world')

    # Models path (portable, no ~/ stuff)
    source_models = os.path.expanduser('~/lab_boys/src/claus_sim/claus_gazebo/models')

    clearpath_path = os.path.expanduser("~/lab_boys/src/clearpath_common/clearpath_platform_description")

    print(clearpath_path)

    resource_paths = ":".join([source_models, clearpath_path])

    # Xacro path (portable)
    xacro_file = PathJoinSubstitution([
        FindPackageShare("claus_description"),
        "urdf",
        "r100_wrapper.xacro"
    ])

    # Process Xacro → robot_description
    robot_description = Command([
        "xacro ",
        xacro_file,
        " platform_config:=generic"
    ])

    # Publish robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'claus_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([

        # Let Gazebo find your models
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_paths
        ),

        # Start Gazebo with your world
        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'
        ),

        # Start robot_state_publisher FIRST
        robot_state_publisher,

        # Delay spawn to avoid race condition
        TimerAction(
            period=2.0,
            actions=[spawn_robot]
        ),
    ])