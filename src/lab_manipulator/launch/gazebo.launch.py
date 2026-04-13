from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
import os


def generate_launch_description():
    manipulator_pkg = FindPackageShare('lab_manipulator')
    lab_base_pkg = FindPackageShare('lab_base')
    world_path = PathJoinSubstitution([manipulator_pkg, 'worlds', 'world.sdf'])
    clearpath_path = FindPackageShare('clearpath_platform_description').find('clearpath_platform_description')
    pkg_prefix = get_package_prefix("clearpath_platform_description")
    resource_path = os.path.join(pkg_prefix, "share")
    params_path = PathJoinSubstitution([lab_base_pkg, 'config', 'setup_sim.yaml'])
    params_nav2 = PathJoinSubstitution([lab_base_pkg, 'config', 'nav2_params.yaml'])

    xacro_file = PathJoinSubstitution([
        FindPackageShare("lab_manipulator"),
        "urdf",
        "r100_wrapper.xacro"
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
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    spawn_lab_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'r100',
            '-x', '0',
            '-y', '0',
            '-z', '0.4'
        ],
        output='screen'
    )

    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[spawn_lab_robot]
    )

    # --- TF tree fixes ---
    # Bridges r100/odom -> odom and r100/base_link -> base_link
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    static_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'r100/odom'],
        output='screen'
    )

    static_baselink_remap = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'r100/base_link', 'base_link'],
        output='screen'
    )

    gazebo_topic = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/r100/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/r100/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/car_world/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
            '/model/r100/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/r100/tf', '/tf'),
        ],
        output='screen'
    )

    lab_manipulator_node = Node(
        package='lab_manipulator',
        executable='lab_arm',
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
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': PathJoinSubstitution([lab_base_pkg, 'config', 'empty_map.yaml']),
            'use_sim_time': True
        }],
        output='screen'
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resources,
        gazebo,
        robot_state_publisher,
        static_map_odom,
        static_odom_baselink,
        static_baselink_remap,
        spawn_robot_delayed,
        nav2,              # nav2_bringup handles map_server via the map argument
        lab_manipulator_node,
        lab_base_delayed,
        gazebo_topic
    ])