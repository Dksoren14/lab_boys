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
    claus_pkg = get_package_share_directory('claus') 
    claus_prefix = get_package_prefix('claus')
    gazebo_pkg = get_package_share_directory('gazebo') 
    control_pkg = FindPackageShare('control')
    pkg_prefix = get_package_prefix('clearpath_platform_description')
    lab_mani_prefix = get_package_prefix('manipulator')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    control_enabled = LaunchConfiguration('control') 

    rviz_config = os.path.join(gazebo_pkg, 'rviz_configs', 'claus_real.rviz')

    source_models = os.path.join(gazebo_pkg, 'models') #changed reference to gazebo pkg

    params_path = PathJoinSubstitution([control_pkg, 'config', 'claus_setup.yaml']) #
    slam_params = PathJoinSubstitution([
        FindPackageShare('control'),
        'config',
        'claus_slamParam.yaml',
    ]) #-CHECK

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

    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths,
    )

    set_localhost_discovery = SetEnvironmentVariable(
        name='ROS_AUTOMATIC_DISCOVERY_RANGE',
        value='LOCALHOST',
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
            (
                '/cmd_vel',
                '/ridgeback_velocity_controller/cmd_vel',
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
            'use_sim_time': 'false',
            'autostart': 'true',
            'slam_params_file': slam_params,
        }.items(),
    ) 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    lidar_merger =Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[{
            'use_sim_time': False,
            'destination_frame': 'chassis_link',
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/scan',
            'laserscan_topics': 'front/scan rear/scan',
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.0087,  # ~0.5 degree resolution (720 samples / 360°)
            'scan_time': 0.1,           # matches your 10hz update rate
            'range_min': 0.05,          # matches your lidar range min
            'range_max': 20.0,          # matches your lidar range max
        }]
    )
    #scan_volatile_relay = Node(
    #    package='claus',
    #    executable='scan_volatile_relay.py',
    #    name='scan_volatile_relay',
    #    output='screen',
    #)

    return LaunchDescription([
        DeclareLaunchArgument(
            'control',
            default_value='true',
            description='Start the lab_chassis node that publishes /cmd_vel.',
        ),
        set_gz_resources,
        set_localhost_discovery,
        TimerAction(period=6.0, actions=[lidar_merger]),
        #TimerAction(period=7.0, actions=[scan_volatile_relay]),
        TimerAction(period=8.0, actions=[rviz_node]),
        TimerAction(period=10.0, actions=[slam]),
        TimerAction(
            period=28.0,
            actions=[control_node],
            condition=IfCondition(control_enabled),
        ),
       ])