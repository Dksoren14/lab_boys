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
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    claus_pkg = get_package_share_directory('claus') #CHECK- (new pkg)
    claus_prefix = get_package_prefix('claus')
    gazebo_pkg = get_package_share_directory('gazebo') #CHECK- (new pkg)
    control_pkg = FindPackageShare('control')
    pkg_prefix = get_package_prefix('clearpath_platform_description')
    lab_mani_prefix = get_package_prefix('manipulator')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    control_enabled = LaunchConfiguration('control') #changed from lab_base to control
    aruco = LaunchConfiguration('aruco')

    world = os.path.join(gazebo_pkg, 'worlds', 'sim_environment.world') #changed reference to gazebo pkg
    rviz_config = os.path.join(gazebo_pkg, 'rviz_configs', 'default.rviz') #changed reference to gazebo pkg

    source_models = os.path.join(gazebo_pkg, 'models') #changed reference to gazebo pkg
    resource_paths = [
        source_models,
        ':',
        os.path.join(pkg_prefix, 'share'),
        ':',
        os.path.join(lab_mani_prefix, 'share'),
        ':',
        os.path.join(claus_prefix, 'share'),
        ':',
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ]#-CHECK

    params_path = PathJoinSubstitution([control_pkg, 'config', 'claus_setup_sim.yaml']) 
    params_nav2 = PathJoinSubstitution([control_pkg, 'config', 'nav2_params.yaml'])
    slam_params = PathJoinSubstitution([
        FindPackageShare('control'),
        'config',
        'claus_slamParam_sim.yaml',
    ]) #-CHECK

    xacro_file = PathJoinSubstitution([
        FindPackageShare('claus'), #CHECK- (new pkg)
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
    ) #-CHECK- (new pkg and xacro file)

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

    sim_aruco_node = Node(
    package='sensors',
    executable='sim_aruco_node',
    name='claus_sim_aruco_sensor',
    condition=IfCondition(aruco),
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
            '-x', '-0.65', #-0.65 if using laboratory.world. 0.0 otherwise
            '-y', '0', 
            '-z', '0.4',
        ],
        output='screen',
    )

    control_node = Node( #check - changed from lab_base_node to control_node, and package from lab_base to control
        package='control',
        executable='lab_chassis',
        name='chassis',
        namespace='control/chassis',
        parameters=[params_path],
        remappings=[
            (
                '/control/chassis/lab_boys/out/base_state',
                '/lab_boys/out/base_state',
            ),
        ],
        output='screen',
    )

    slam = IncludeLaunchDescription(  #CHECK-
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
    ) #-CHECK

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
            'node_names': [
                            'keepout_map_server',
                            'costmap_filter_info_server',
                            'planner_server',
                        ],
        }],
        output='screen',
    )
    
    keepout_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_map_server',
        parameters=[
            params_nav2,
            {'use_sim_time': True},
            {'yaml_filename': os.path.join(get_package_share_directory('gazebo'), 'maps', 'sim_map.yaml')},
            {'topic_name': '/keepout_filter_mask'},
            {'frame_id': 'map'},
        ],
        output='screen',
    )
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        parameters=[
            params_nav2,
            {'use_sim_time': True},
            {'type': 0},
            {'filter_info_topic': 'costmap_filter_info'},
            {'mask_topic': '/keepout_filter_mask'},
            {'base': 0.0},
            {'multiplier': 1.0},
        ],
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
            '/sensors/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/r100/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/default/model/r100/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/front_realsense/image@sensor_msgs/msg/Image@gz.msgs.Image',
            #'/front_realsense/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            #'/front_realsense/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
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
    lidar_merger = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',

        parameters=[{
            'use_sim_time': True,
            'destination_frame': 'r100/chassis_link',
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/scan',
            'laserscan_topics': 'sensors/front_lidar/scan sensors/rear_lidar/scan',

            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 20.0,          
        }]
    )

    scan_volatile_relay = Node(
        package='claus',
        executable='scan_volatile_relay.py',
        name='scan_volatile_relay',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI in addition to the simulation server.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the R100 simulation display config.',
        ),
        DeclareLaunchArgument(
            'control',
            default_value='true',
            description='Start the lab_chassis node that publishes /cmd_vel.',
        ),
        DeclareLaunchArgument(
            'aruco',
            default_value='true',
            description='Start the simulated ArUco marker detection node.',
        ),
        set_gz_resources,
        set_localhost_discovery,
        gazebo_server,
        gazebo_gui,
        gazebo_topic,
        TimerAction(period=3.0, actions=[robot_state_publisher]),
        TimerAction(period=6.0, actions=[spawn_lab_robot, throttle_joint_states, lidar_merger]),
        TimerAction(period=7.0, actions=[scan_volatile_relay]),
        TimerAction(period=8.0, actions=[rviz_node]),
        TimerAction(period=9.0, actions=[sim_aruco_node]),
        TimerAction(period=10.0, actions=[slam]),
        TimerAction(period=15.0, actions=[keepout_map_server, costmap_filter_info_server]),
        TimerAction(period=20.0, actions=[
            planner_server,
            planner_lifecycle_manager,
        ]),
        TimerAction(
            period=28.0,
            actions=[control_node],
            condition=IfCondition(control_enabled),
        ),
    ])