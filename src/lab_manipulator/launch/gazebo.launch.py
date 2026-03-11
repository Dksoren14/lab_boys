from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, PathJoinSubstitution
import os


def generate_launch_description():
    manipulator_pkg = FindPackageShare('lab_manipulator')
    params_path = PathJoinSubstitution([manipulator_pkg, 'config', 'manipulator_params.yaml'])
    world_path = PathJoinSubstitution([manipulator_pkg, 'worlds', 'world.sdf'])
    chassis_path = PathJoinSubstitution([manipulator_pkg, 'urdf', 'chassis.urdf'])
    clearpath_path = FindPackageShare('clearpath_platform_description').find('clearpath_platform_description')
    pkg_prefix = get_package_prefix("clearpath_platform_description")
    resource_path = os.path.join(pkg_prefix, "share")

    
    xacro_file = PathJoinSubstitution([
        FindPackageShare("lab_manipulator"),
        "urdf",
        "j100_wrapper.xacro"
    ])

    robot_description = Command([
        "xacro ",
        xacro_file,
        " platform_config:=generic"
    ])

    
    set_gz_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', world_path],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'j100',
            '-x', '0',
            '-y', '0',
            '-z', '0.4'
        ],
        output='screen'
    )
    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )
    # Self made robot exercise
    #spawn_entity = Node(
    #    package='ros_gz_sim',
    #    executable='create',
    #    arguments=[
    #        '-topic', '/robot_description',
    #        '-name', 'vehicle_blue',
    #        '-x', '0',
    #        '-y', '0',
    #        '-z', '0.01'
    #    ],
    #    output='screen'
    #)
    

    lab_manipulator_node = Node(
        package='lab_manipulator',
        executable='lab_arm',
        output='screen'
    )
    gazebo_topic = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/j100/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',],
            output='screen'
        )
    
    return LaunchDescription([
        set_gz_resources,
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
        #spawn_entity,
        lab_manipulator_node,
        gazebo_topic
    ])