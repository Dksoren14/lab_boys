from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get package paths (portable)
    gazebo_pkg = get_package_share_directory('claus_gazebo')

    # World file
    world = os.path.join(gazebo_pkg, 'worlds', 'sim_environment.world')

    # Models path
    source_models = os.path.expanduser('~/lab_boys/src/claus_sim/claus_gazebo/models')
    claus_description_path = get_package_share_directory('claus_description')

    resource_paths = ":".join([source_models, claus_description_path])



    # Xacro path (portable)
    xacro_file = PathJoinSubstitution([
        FindPackageShare("claus_description"),
        "urdf",
        "ridgeback.urdf.xacro"
    ])

    # Process Xacro, robot_description
    robot_description = Command([
        "xacro ",
        xacro_file,
        " ",
        "platform_config:=generic"
    ]),
    value_type=str

    set_gz_resources = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_paths
        )
    
    gazebo_world =  ExecuteProcess(
            cmd=['gz', 'sim', world, '-r'],
            output='screen'
        )

    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[
        {
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }
    ],
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

    spawn_robot_delayed = TimerAction(
            period=5.0,
            actions=[spawn_robot]
        )
    
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        )
    
    gazebo_topic = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/r100/tf', '/tf'),
        ],
        output='screen'
    )

    odom_to_tf_converter = Node(
        package='claus_control',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
        )   


    return LaunchDescription([
        set_gz_resources,
        gazebo_world,
        robot_state_publisher,
        spawn_robot_delayed,
        bridge,
        odom_to_tf_converter
    ])



joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen'
)

mecanum_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['mecanum_controller'],
    output='screen'
)