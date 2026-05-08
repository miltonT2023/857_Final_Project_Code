import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
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


def labels_file_for_map(map_yaml_path):
    if not map_yaml_path:
        return ''
    base, _ = os.path.splitext(map_yaml_path)
    return f'{base}.labels.yaml'


def saved_initial_pose(map_yaml_path, label_name='robot_start'):
    labels_file = labels_file_for_map(map_yaml_path)
    pose = {'x': '0.0', 'y': '0.0', 'yaw': '0.0'}
    if not labels_file or not os.path.exists(labels_file):
        return pose

    current_name = None
    with open(labels_file, 'r', encoding='utf-8') as label_file:
        for raw_line in label_file:
            line = raw_line.rstrip()
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if line.startswith('  ') and stripped.endswith(':'):
                current_name = stripped[:-1]
                continue
            if current_name == label_name and line.startswith('    ') and ':' in stripped:
                key, value = stripped.split(':', 1)
                if key in pose:
                    pose[key] = value.strip()

    return pose


def create_initial_pose_publisher(context, *args, **kwargs):
    map_path = LaunchConfiguration('map').perform(context)
    initial_x = LaunchConfiguration('initial_x').perform(context)
    initial_y = LaunchConfiguration('initial_y').perform(context)
    initial_yaw = LaunchConfiguration('initial_yaw').perform(context)

    saved_pose = saved_initial_pose(map_path)
    if initial_x == '':
        initial_x = saved_pose['x']
    if initial_y == '':
        initial_y = saved_pose['y']
    if initial_yaw == '':
        initial_yaw = saved_pose['yaw']

    return [
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='milton_final_project',
                    executable='initial_pose_publisher',
                    name='initial_pose_publisher',
                    output='screen',
                    arguments=[
                        '--x', initial_x,
                        '--y', initial_y,
                        '--yaw', initial_yaw,
                    ],
                )
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory('milton_final_project')
    qbot_share = get_package_share_directory('qbot_platform')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_map_viewer = LaunchConfiguration('use_map_viewer')
    map_viewer_port = LaunchConfiguration('map_viewer_port')

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

    use_map_viewer_arg = DeclareLaunchArgument(
        'use_map_viewer',
        default_value='true',
        description='Start browser-based live navigation map viewer.',
    )

    map_viewer_port_arg = DeclareLaunchArgument(
        'map_viewer_port',
        default_value='8093',
        description='Port for the browser-based navigation map viewer.',
    )

    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        default_value='',
        description='Initial AMCL x pose in the map frame.',
    )

    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        default_value='',
        description='Initial AMCL y pose in the map frame.',
    )

    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw',
        default_value='',
        description='Initial AMCL yaw pose in the map frame.',
    )

    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qbot_share, 'launch', 'qbot_platform_launch.py')
        )
    )

    lidar_tf_node = Node(
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
            'publish_tf': True,
        }],
    )

    scan_sanitizer_node = Node(
        package='milton_final_project',
        executable='scan_sanitizer_node',
        name='navigation_scan_sanitizer_node',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_slam',
            'range_min': 0.30,
            'calibration_duration_sec': 0.0,
            'self_mask_max_range': 0.85,
            'self_mask_hit_ratio': 0.55,
            'stamp_with_current_time': True,
            'mask_file': (
                '/home/nvidia/857_Final_Project_Code/maps/'
                'qbot_lidar_filter.json'
            ),
        }],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    initial_pose_publisher = OpaqueFunction(
        function=create_initial_pose_publisher,
    )

    navigation_map_viewer = Node(
        condition=IfCondition(use_map_viewer),
        package='milton_final_project',
        executable='live_map_web_viewer',
        name='navigation_map_web_viewer',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'scan_topic': '/scan_slam',
            'odom_topic': '/odom',
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'host': '0.0.0.0',
            'port': map_viewer_port,
            'map_dir': DEFAULT_MAP_DIR,
            'map_name_prefix': 'slam_toolbox_map',
            'page_title': 'QBot Navigation Map',
            'show_map_actions': False,
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

    return LaunchDescription([
        map_arg,
        params_arg,
        use_map_viewer_arg,
        map_viewer_port_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        qbot_platform_launch,
        lidar_tf_node,
        qbot_odometry_node,
        scan_sanitizer_node,
        localization_launch,
        navigation_launch,
        initial_pose_publisher,
        navigation_map_viewer,
        q_shutdown_node,
        q_shutdown_handler,
    ])
