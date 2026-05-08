from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


LIBGOMP_PATH = '/lib/aarch64-linux-gnu/libgomp.so.1'


def generate_launch_description():
    fullscreen = LaunchConfiguration('fullscreen')
    web_port = LaunchConfiguration('web_port')
    stt_model_size = LaunchConfiguration('stt_model_size')
    stt_device = LaunchConfiguration('stt_device')
    stt_compute_type = LaunchConfiguration('stt_compute_type')

    face_display_node = Node(
        package='milton_final_project',
        executable='face_display_node',
        name='face_display_node',
        output='screen',
        prefix=f'env LD_PRELOAD={LIBGOMP_PATH}',
        additional_env={
            'LD_PRELOAD': [
                LIBGOMP_PATH,
                ':',
                EnvironmentVariable('LD_PRELOAD', default_value=''),
            ],
        },
        parameters=[
            {'width': 1024},
            {'height': 600},
            {'fullscreen': fullscreen},
            {'show_help': False},
            {'initial_expression': 'neutral'},
            {'web_stream_enabled': True},
            {'web_stream_host': '0.0.0.0'},
            {'web_stream_port': web_port},
            {'speak_phrases': True},
            {'speech_rate': 125},
            {'speech_volume': 0.65},
            {'speech_voice_id': 'gmw/en-us'},
            {'stt_enabled': True},
            {'stt_backend': 'faster_whisper'},
            {'stt_model_size': stt_model_size},
            {'stt_model_path': ''},
            {'stt_device': stt_device},
            {'stt_compute_type': stt_compute_type},
            {'stt_local_files_only': False},
            {
                'waiting_message': (
                    'I am the SEIC navigation robot. Please enter the person '
                    'or room you are trying to find.'
                ),
            },
            {'response_duration_sec': 10.0},
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='LD_PRELOAD',
            value=[
                LIBGOMP_PATH,
                ':',
                EnvironmentVariable('LD_PRELOAD', default_value=''),
            ],
        ),
        DeclareLaunchArgument('fullscreen', default_value='true'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('stt_model_size', default_value='base'),
        DeclareLaunchArgument('stt_device', default_value='auto'),
        DeclareLaunchArgument('stt_compute_type', default_value='auto'),
        face_display_node,
    ])
