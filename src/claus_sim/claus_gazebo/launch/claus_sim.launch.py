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
    pkg_prefix = get_package_prefix("clearpath_platform_description")

    # World file
    world = os.path.join(claus_gazebo_pkg, 'worlds', 'empty.world')

    # Models path
    source_models = os.path.expanduser('~/lab_boys/src/claus_sim/claus_gazebo/models')
    claus_description_path = get_package_share_directory('claus_description')

    resource_paths = ":".join([source_models, claus_description_path])
    resource_path = os.path.join(pkg_prefix, "share")
    
    #Params
    params_path = PathJoinSubstitution([lab_base_pkg, 'config', 'setup_sim.yaml'])
    params_nav2 = PathJoinSubstitution([lab_base_pkg, 'config', 'nav2_params.yaml'])
    slam_params = PathJoinSubstitution([FindPackageShare("claus_nav"), "config", "mapper_params_online_async.yaml"])

    # Xacro path (portable)
    #xacro_file = PathJoinSubstitution([
    #    FindPackageShare("claus_description"),
    #    "urdf",
    #    "ridgeback.urdf.xacro"
    #])

    xacro_file = PathJoinSubstitution([
        FindPackageShare("lab_manipulator"),
        "urdf",
        "r100_wrapper.xacro"
    ])
    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " ",
            "platform_config:=generic"
        ]),
        value_type=str
    )

    set_gz_resources = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_paths
        )
    set_gz_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path
    )
    set_localhost = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY',
        value='1'
    )
    
    gazebo_world =  ExecuteProcess(
            cmd=['gz', 'sim', world, '-r'],
            output='screen'
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0,   
        }],
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
            period=5.0,
            actions=[spawn_robot]
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
            ("slam", "True"),
            ("slam_params_file", slam_params),   # ← add this line
        ]
    )
    nav2_delayed = TimerAction(
        period=15.0,
        actions=[nav2]
    )

    lab_base_delayed = TimerAction(
        period=5.0,
        actions=[lab_base_node]
    )

    
    gazebo_topic = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
           '/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/model/r100/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            #'/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            #'/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
            '/model/r100/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
        ],
        remappings=[
            ('/world/default/clock', '/clock'),
            ('/model/r100/odometry', '/odom'),
            ('/model/r100/cmd_vel', '/cmd_vel'),
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',  # ← match RSP
        }],
        output='screen'
    )

    odom_to_tf_converter = Node(
        package='claus_control',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ) 

    slam = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare("slam_toolbox"),
            "launch",
            "online_async_launch.py"
        ])
    ]),
    launch_arguments={
        "use_sim_time": "true",
        "params_file": slam_params
    }.items()
)

    return LaunchDescription([
        set_gz_resources,
        set_localhost,
        gazebo_world,
        gazebo_topic,
        TimerAction(period=3.0, actions=[robot_state_publisher]),
        TimerAction(period=6.0, actions=[spawn_lab_robot, odom_to_tf_converter]),
        TimerAction(period=10.0, actions=[lab_base_node]),
        TimerAction(period=20.0, actions=[nav2]),  # Nav2 owns SLAM now
    ])


