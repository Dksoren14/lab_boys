import os

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    claus_gazebo_pkg = get_package_share_directory('claus_gazebo')
    lab_base_pkg = FindPackageShare('lab_base')
    pkg_prefix = get_package_prefix('clearpath_platform_description')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    lab_base = LaunchConfiguration('lab_base')

    world = os.path.join(claus_gazebo_pkg, 'worlds', 'labyrinth.world')
    rviz_config = os.path.join(claus_gazebo_pkg, 'rviz_configs', 'default.rviz')

    source_models = os.path.join(claus_gazebo_pkg, 'models')
    claus_description_prefix = get_package_prefix('claus_description')
    resource_paths = [
        source_models,
        ':',
        os.path.join(claus_description_prefix, 'share'),
        ':',
        os.path.join(pkg_prefix, 'share'),
        ':',
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ]

    params_path = PathJoinSubstitution([lab_base_pkg, 'config', 'setup_sim.yaml'])
    params_nav2 = PathJoinSubstitution([lab_base_pkg, 'config', 'nav2_params.yaml'])
    slam_params = PathJoinSubstitution([
        FindPackageShare('claus_nav'),
        'config',
        'mapper_params_online_async.yaml',
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('lab_manipulator'),
        'urdf',
        'r100_wrapper.xacro',
    ])
    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' ',
            'platform_config:=generic',
        ]),
        value_type=str,
    )

    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths,
    )

    set_localhost_discovery = SetEnvironmentVariable(
        name='ROS_AUTOMATIC_DISCOVERY_RANGE',
        value='LOCALHOST',
    )

    snap_env_vars = [
        'SNAP',
        'SNAP_ARCH',
        'SNAP_COMMON',
        'SNAP_CONTEXT',
        'SNAP_COOKIE',
        'SNAP_DATA',
        'SNAP_EUID',
        'SNAP_INSTANCE_NAME',
        'SNAP_LAUNCHER_ARCH_TRIPLET',
        'SNAP_LIBRARY_PATH',
        'SNAP_NAME',
        'SNAP_REAL_HOME',
        'SNAP_REVISION',
        'SNAP_UID',
        'SNAP_USER_COMMON',
        'SNAP_USER_DATA',
        'SNAP_VERSION',
    ]
    gazebo_clean_env = [item for var in snap_env_vars for item in ('-u', var)]

    gazebo_server = ExecuteProcess(
        cmd=['env', *gazebo_clean_env, 'gz', 'sim', '-s', '-r', world],
        output='screen',
    )

    gazebo_gui = ExecuteProcess(
        cmd=['env', *gazebo_clean_env, 'gz', 'sim', '-g'],
        condition=IfCondition(gui),
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'frame_prefix': 'r100/',
        }],
        remappings=[
            ('/joint_states', '/joint_states_throttled'),
            ('robot_description', '/r100/robot_description'),
        ],
        output='screen',
    )

    spawn_lab_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/r100/robot_description',
            '-name', 'r100',
            '-x', '0',
            '-y', '0',
            '-z', '0.4',
        ],
        output='screen',
    )

    lab_base_node = Node(
        package='lab_base',
        executable='lab_chassis',
        name='chassis',
        namespace='lab_base/chassis',
        parameters=[params_path],
        remappings=[
            (
                '/lab_base/chassis/lab_boys/out/base_state',
                '/lab_boys/out/base_state',
            ),
        ],
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ]),
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam_params_file': slam_params,
        }.items(),
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_nav2, {'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
    )

    planner_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['planner_server'],
        }],
        output='screen',
    )

    throttle_joint_states = Node(
        package='topic_tools',
        executable='throttle',
        arguments=[
            'messages',
            '/joint_states',
            '50.0',
            '/joint_states_throttled',
        ],
        output='screen',
    )

    gazebo_topic = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/r100/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/r100/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/sensors/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/r100/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/default/model/r100/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/default/clock', '/clock'),
            ('/model/r100/odometry', '/odom'),
            ('/model/r100/cmd_vel', '/cmd_vel'),
            ('/model/r100/tf', '/tf'),
            ('/world/default/model/r100/joint_state', '/joint_states'),
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start the Gazebo GUI in addition to the simulation server.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz with the R100 simulation display config.',
        ),
        DeclareLaunchArgument(
            'lab_base',
            default_value='true',
            description='Start the lab_chassis node that publishes /cmd_vel.',
        ),
        set_gz_resources,
        set_localhost_discovery,
        gazebo_server,
        gazebo_gui,
        gazebo_topic,
        TimerAction(period=3.0, actions=[robot_state_publisher]),
        TimerAction(period=6.0, actions=[spawn_lab_robot, throttle_joint_states]),
        TimerAction(period=8.0, actions=[rviz_node]),
        TimerAction(period=10.0, actions=[slam]),
        TimerAction(period=15.0, actions=[planner_server, planner_lifecycle_manager]),
        TimerAction(
            period=28.0,
            actions=[lab_base_node],
            condition=IfCondition(lab_base),
        ),
    ])