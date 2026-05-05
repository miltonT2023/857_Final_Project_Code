import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MAP_DIR = '/home/nvidia/857_Final_Project_Code/maps'


def latest_map_yaml():
    if not os.path.isdir(DEFAULT_MAP_DIR):
        return ''

    yaml_paths = [
        os.path.join(DEFAULT_MAP_DIR, name)
        for name in os.listdir(DEFAULT_MAP_DIR)
        if name.endswith('.yaml')
        and not name.endswith('.labels.yaml')
        and name != 'map_labels.yaml'
    ]
    if not yaml_paths:
        return ''

    return max(yaml_paths, key=os.path.getmtime)


def generate_launch_description():
    package_share = get_package_share_directory('milton_final_project')
    qbot_share = get_package_share_directory('qbot_platform')
    nav2_share = get_package_share_directory('nav2_bringup')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=latest_map_yaml(),
        description='Full path to the saved map yaml file.',
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_share,
            'config',
            'qbot_navigation_params.yaml',
        ),
        description='Nav2 parameters file.',
    )

    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qbot_share, 'launch', 'qbot_platform_launch.py')
        )
    )

    qbot_lidar_tf_node = Node(
        package='qbot_platform',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen',
    )

    qbot_odometry_node = Node(
        package='milton_final_project',
        executable='qbot_odometry_node',
        name='qbot_odometry_node',
        output='screen',
        parameters=[{
            'speed_topic': '/qbot_speed_feedback',
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_rate_hz': 30.0,
        }],
    )

    q_shutdown_node = Node(
        package='milton_final_project',
        executable='q_shutdown_node',
        name='q_shutdown_node',
        output='screen',
    )

    q_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=q_shutdown_node,
            on_exit=[
                Shutdown(reason='q pressed or shutdown requested'),
            ],
        )
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'false',
            'slam': 'False',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    delayed_nav2_bringup = TimerAction(
        period=6.0,
        actions=[nav2_bringup],
    )

    initial_pose_publisher = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='milton_final_project',
                executable='initial_pose_publisher',
                name='initial_pose_publisher',
                output='screen',
                arguments=[
                    '--x', '0.0',
                    '--y', '0.0',
                    '--yaw', '0.0',
                ],
            )
        ],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        qbot_platform_launch,
        qbot_lidar_tf_node,
        qbot_odometry_node,
        q_shutdown_node,
        q_shutdown_handler,
        delayed_nav2_bringup,
        initial_pose_publisher,
    ])
