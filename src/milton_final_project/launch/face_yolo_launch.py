import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('qbot_platform'),
                'launch',
                'qbot_platform_launch.py',
            )
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
        launch_arguments={
            'align_depth.enable': 'true',
        }.items(),
    )

    yolo_node = Node(
        package='milton_final_project',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        additional_env={
            'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1',
        },
        parameters=[
            {'detection_model': 'yolov8n.pt'},
            {'image_topic': '/camera/color/image_raw'},
            {'depth_topic': '/camera/aligned_depth_to_color/image_raw'},
            {'confidence': 0.25},
        ],
    )

    face_display_node = Node(
        package='milton_final_project',
        executable='face_display_node',
        name='face_display_node',
        output='screen',
        additional_env={
            'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1',
        },
        parameters=[
            {'width': 1024},
            {'height': 600},
            {'fullscreen': True},
            {'show_help': False},
            {'initial_expression': 'neutral'},
            {'web_stream_enabled': True},
            {'web_stream_host': '0.0.0.0'},
            {'web_stream_port': 8080},
            {'stt_enabled': True},
            {'stt_backend': 'faster_whisper'},
            {'stt_model_size': 'base'},
            {'stt_device': 'auto'},
            {'stt_compute_type': 'auto'},
            {'stt_local_files_only': False},
            {
                'waiting_message':
                "Hello, Hi, I love butter. I'm the nagivation robot that helps you find a location or room."
            },
            {'response_duration_sec': 10.0},
        ],
    )

    light_controller_node = Node(
        package='milton_final_project',
        executable='light_controller_node',
        name='light_controller_node',
        output='screen',
    )

    lidar_person_tracker_node = Node(
        package='milton_final_project',
        executable='lidar_person_tracker_node',
        name='lidar_person_tracker_node',
        output='screen',
    )

    waiting_person_greeter_node = Node(
        package='milton_final_project',
        executable='waiting_person_greeter_node',
        name='waiting_person_greeter_node',
        output='screen',
    )

    return LaunchDescription([
        qbot_platform_launch,
        realsense_launch,
        yolo_node,
        lidar_person_tracker_node,
        waiting_person_greeter_node,
        face_display_node,
        light_controller_node,
    ])
