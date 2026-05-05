import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    qbot_share = get_package_share_directory('qbot_platform')

    cartographer_config_dir = os.path.join(
        qbot_share,
        'config',
    )

    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                qbot_share,
                'launch',
                'qbot_platform_launch.py',
            )
        )
    )

    qbot_lidar_tf_node = Node(
        package='qbot_platform',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen',
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
        qbot_platform_launch,
        qbot_lidar_tf_node,
        joystick_command_node,
        cartographer_node,
        occupancy_grid_node,
    ])
