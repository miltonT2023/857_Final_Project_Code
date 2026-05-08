import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    qbot_share = get_package_share_directory('qbot_platform')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    lidar_x_arg = DeclareLaunchArgument(
        'lidar_x',
        default_value='0.15',
        description='LiDAR x offset from base_link in meters.',
    )

    lidar_y_arg = DeclareLaunchArgument(
        'lidar_y',
        default_value='0.0',
        description='LiDAR y offset from base_link in meters.',
    )

    lidar_z_arg = DeclareLaunchArgument(
        'lidar_z',
        default_value='0.20',
        description='LiDAR z offset from base_link in meters.',
    )

    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='1.57079632679',
        description='LiDAR yaw offset from base_link in radians.',
    )

    cartographer_config_dir = os.path.join(
        qbot_share,
        'config',
    )

    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qbot_share, 'launch', 'qbot_platform_launch.py')
        )
    )

    qbot_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        output='screen',
        arguments=[
            '--x',
            lidar_x,
            '--y',
            lidar_y,
            '--z',
            lidar_z,
            '--yaw',
            lidar_yaw,
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'base_scan',
        ],
    )

    joystick_command_node = Node(
        package='qbot_platform',
        executable='command',
        name='joystickCommands',
        output='screen',
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory',
            cartographer_config_dir,
            '-configuration_basename',
            'qbot_platform_2d.lua',
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution',
            '0.05',
            '-publish_period_sec',
            '1.0',
        ],
    )

    return LaunchDescription([
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        lidar_yaw_arg,
        qbot_platform_launch,
        qbot_lidar_tf_node,
        joystick_command_node,
        cartographer_node,
        occupancy_grid_node,
    ])
