from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
import os


def generate_launch_description():

    # Get package paths (portable)
    claus_gazebo_pkg = get_package_share_directory('claus_gazebo')
    manipulator_pkg = FindPackageShare('lab_manipulator')
    lab_base_pkg = FindPackageShare('lab_base')

    # World file
    world = os.path.join(claus_gazebo_pkg, 'worlds', 'sim_environment.world')

    # Models path
    source_models = os.path.expanduser('~/lab_boys/src/claus_sim/claus_gazebo/models')
    claus_description_path = get_package_share_directory('claus_description')

    resource_paths = ":".join([source_models, claus_description_path])
    
    #Params
    params_path = PathJoinSubstitution([lab_base_pkg, 'config', 'setup_sim.yaml'])
    params_nav2 = PathJoinSubstitution([lab_base_pkg, 'config', 'nav2_params.yaml'])


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

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    static_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', '/odom'],
        output='screen'
    )

    static_baselink_remap = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '/base_link', 'base_link'],
        output='screen'
    )

    lab_base_node = Node(
        package='lab_base',
        executable='lab_chassis',
        name='chassis',
        namespace='lab_base/chassis',
        parameters=[params_path],
        remappings=[('/lab_base/chassis/lab_boys/out/base_state', '/lab_boys/out/base_state')],
        output='screen'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py"
            ])
        ]),
        launch_arguments=[
            ("params_file", params_nav2),
            ("use_sim_time", "true"),
            ("use_composition", "False"),
            ("map", PathJoinSubstitution([lab_base_pkg, 'config', 'empty_map.yaml'])) 
        ]
    )

    lab_base_delayed = TimerAction(
        period=5.0,
        actions=[lab_base_node]
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
        #remappings=[
        #    ('/model/r100/tf', '/tf'),
        #],
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
        static_map_odom,
        static_odom_baselink,
        static_baselink_remap,
        spawn_robot_delayed,
        nav2,
        lab_base_delayed,
        gazebo_topic,
        odom_to_tf_converter,
    ])


