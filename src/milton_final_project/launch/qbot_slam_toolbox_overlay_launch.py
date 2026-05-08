from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    web_viewer_port = LaunchConfiguration('web_viewer_port')
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=(
            '/home/nvidia/857_Final_Project_Code/install/'
            'milton_final_project/share/milton_final_project/config/'
            'qbot_slam_toolbox.yaml'
        ),
        description='SLAM Toolbox parameters file.',
    )

    web_viewer_port_arg = DeclareLaunchArgument(
        'web_viewer_port',
        default_value='8094',
        description='Port for the browser-based live map viewer.',
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Fuse wheel odometry and IMU with robot_localization EKF.',
    )

    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=(
            '/home/nvidia/857_Final_Project_Code/install/'
            'milton_final_project/share/milton_final_project/config/'
            'qbot_ekf.yaml'
        ),
        description='robot_localization EKF parameters file.',
    )

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

    qbot_odometry_node = Node(
        condition=UnlessCondition(use_ekf),
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
            'publish_tf': True,
        }],
    )

    qbot_wheel_odometry_node = Node(
        condition=IfCondition(use_ekf),
        package='milton_final_project',
        executable='qbot_odometry_node',
        name='qbot_wheel_odometry_node',
        output='screen',
        parameters=[{
            'speed_topic': '/qbot_speed_feedback',
            'odom_topic': '/wheel_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_rate_hz': 30.0,
            'publish_tf': False,
        }],
    )

    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    scan_sanitizer_node = Node(
        package='milton_final_project',
        executable='scan_sanitizer_node',
        name='scan_sanitizer_node',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_slam',
            'range_min': 0.30,
            'calibration_duration_sec': 16.0,
            'self_mask_max_range': 0.85,
            'self_mask_hit_ratio': 0.45,
            'speckle_filter_window': 0,
            'speckle_min_neighbors': 0,
            'speckle_max_range_delta': 0.0,
            'median_filter_window': 0,
            'median_max_range_delta': 0.0,
            'stamp_offset_sec': -0.35,
            'mask_file': (
                '/home/nvidia/857_Final_Project_Code/maps/'
                'qbot_lidar_filter.json'
            ),
        }],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'scan_topic': '/scan_slam'},
        ],
        remappings=[
            ('/map', '/slam_toolbox_map'),
            ('/map_metadata', '/slam_toolbox_map_metadata'),
        ],
    )

    live_map_web_viewer = Node(
        package='milton_final_project',
        executable='live_map_web_viewer',
        name='live_map_web_viewer',
        output='screen',
        parameters=[{
            'map_topic': '/slam_toolbox_map',
            'scan_topic': '/scan_slam',
            'odom_topic': '/odom',
            'host': '0.0.0.0',
            'port': web_viewer_port,
        }],
    )

    return LaunchDescription([
        params_arg,
        web_viewer_port_arg,
        use_ekf_arg,
        ekf_params_arg,
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        lidar_yaw_arg,
        qbot_lidar_tf_node,
        qbot_odometry_node,
        qbot_wheel_odometry_node,
        ekf_node,
        scan_sanitizer_node,
        slam_toolbox_node,
        live_map_web_viewer,
    ])
