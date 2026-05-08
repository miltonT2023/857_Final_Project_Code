import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('milton_final_project')
    qbot_share = get_package_share_directory('qbot_platform')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    use_web_viewer = LaunchConfiguration('use_web_viewer')
    web_viewer_port = LaunchConfiguration('web_viewer_port')
    lidar_viewer_port = LaunchConfiguration('lidar_viewer_port')
    use_camera_stream = LaunchConfiguration('use_camera_stream')
    camera_stream_port = LaunchConfiguration('camera_stream_port')
    camera_image_topic = LaunchConfiguration('camera_image_topic')
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_share,
            'config',
            'qbot_slam_toolbox.yaml',
        ),
        description='SLAM Toolbox parameters file.',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            package_share,
            'rviz',
            'qbot_slam_toolbox.rviz',
        ),
        description='RViz config for live SLAM viewing.',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz to view /map, /scan, /odom, and TF.',
    )

    use_web_viewer_arg = DeclareLaunchArgument(
        'use_web_viewer',
        default_value='true',
        description='Start browser-based live /map viewer.',
    )

    web_viewer_port_arg = DeclareLaunchArgument(
        'web_viewer_port',
        default_value='8090',
        description='Port for the browser-based live /map viewer.',
    )

    lidar_viewer_port_arg = DeclareLaunchArgument(
        'lidar_viewer_port',
        default_value='8091',
        description='Port for the browser-based LiDAR/filter viewer.',
    )

    use_camera_stream_arg = DeclareLaunchArgument(
        'use_camera_stream',
        default_value='true',
        description='Start RealSense color camera and browser stream.',
    )

    camera_stream_port_arg = DeclareLaunchArgument(
        'camera_stream_port',
        default_value='8095',
        description='Port for the browser-based camera stream.',
    )

    camera_image_topic_arg = DeclareLaunchArgument(
        'camera_image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic to stream while mapping.',
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='Fuse wheel odometry and IMU with robot_localization EKF.',
    )

    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(package_share, 'config', 'qbot_ekf.yaml'),
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

    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qbot_share, 'launch', 'qbot_platform_launch.py')
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        condition=IfCondition(use_camera_stream),
        launch_arguments={
            'align_depth.enable': 'true',
        }.items(),
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

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.08,
            'autorepeat_rate': 20.0,
        }],
    )

    joystick_command_node = Node(
        package='milton_final_project',
        executable='qbot_joy_cmd_vel_node',
        name='qbot_joy_cmd_vel_node',
        output='screen',
        parameters=[{
            'joy_topic': '/joy',
            'cmd_vel_topic': '/cmd_vel',
            'enable_button': 4,
            'reverse_button': 0,
            'steering_axis': 0,
            'throttle_axis': 5,
            'max_linear_speed': 0.30,
            'max_angular_speed': 0.50,
        }],
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
            'calibration_duration_sec': 0.0,
            'self_mask_max_range': 0.85,
            'self_mask_hit_ratio': 0.45,
            'speckle_filter_window': 0,
            'speckle_min_neighbors': 0,
            'speckle_max_range_delta': 0.0,
            'median_filter_window': 0,
            'median_max_range_delta': 0.0,
            'stamp_with_current_time': True,
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
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    live_map_web_viewer = Node(
        condition=IfCondition(use_web_viewer),
        package='milton_final_project',
        executable='live_map_web_viewer',
        name='live_map_web_viewer',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'scan_topic': '/scan_slam',
            'odom_topic': '/odom',
            'host': '0.0.0.0',
            'port': web_viewer_port,
            'map_dir': '/home/nvidia/857_Final_Project_Code/maps',
            'map_name_prefix': 'slam_toolbox_map',
            'start_label': 'robot_start',
            'start_aliases': 'start,home,original',
        }],
    )

    lidar_web_viewer = Node(
        condition=IfCondition(use_web_viewer),
        package='milton_final_project',
        executable='lidar_web_viewer',
        name='lidar_web_viewer',
        output='screen',
        parameters=[{
            'raw_scan_topic': '/scan',
            'filtered_scan_topic': '/scan_slam',
            'mask_file': (
                '/home/nvidia/857_Final_Project_Code/maps/'
                'qbot_lidar_filter.json'
            ),
            'host': '0.0.0.0',
            'port': lidar_viewer_port,
        }],
    )

    camera_web_stream = Node(
        condition=IfCondition(use_camera_stream),
        package='milton_final_project',
        executable='yolo_web_stream',
        name='mapping_camera_stream',
        output='screen',
        parameters=[
            {'image_topic': camera_image_topic},
            {'host': '0.0.0.0'},
            {'port': camera_stream_port},
        ],
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

    return LaunchDescription([
        params_arg,
        rviz_config_arg,
        use_rviz_arg,
        use_web_viewer_arg,
        web_viewer_port_arg,
        lidar_viewer_port_arg,
        use_camera_stream_arg,
        camera_stream_port_arg,
        camera_image_topic_arg,
        use_ekf_arg,
        ekf_params_arg,
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        lidar_yaw_arg,
        qbot_platform_launch,
        realsense_launch,
        qbot_lidar_tf_node,
        joy_node,
        joystick_command_node,
        qbot_odometry_node,
        qbot_wheel_odometry_node,
        ekf_node,
        scan_sanitizer_node,
        slam_toolbox_node,
        rviz_node,
        live_map_web_viewer,
        lidar_web_viewer,
        camera_web_stream,
        q_shutdown_node,
        q_shutdown_handler,
    ])
