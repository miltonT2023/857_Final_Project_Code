import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'milton_final_project'
package_root = os.path.abspath(os.path.dirname(__file__))
repo_root = os.path.abspath(os.path.join(package_root, '..', '..'))
assets_root = os.path.join(repo_root, 'assets')
data_root = os.path.join(repo_root, 'data')

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]

for root, _, files in os.walk(assets_root):
    if not files:
        continue

    relative_root = os.path.relpath(root, repo_root)
    install_root = os.path.join('share', package_name, relative_root)
    data_files.append(
        (
            install_root,
            [
                os.path.relpath(os.path.join(root, file_name), package_root)
                for file_name in files
            ],
        )
    )

for root, _, files in os.walk(data_root):
    if not files:
        continue

    relative_root = os.path.relpath(root, repo_root)
    install_root = os.path.join('share', package_name, relative_root)
    data_files.append(
        (
            install_root,
            [
                os.path.relpath(os.path.join(root, file_name), package_root)
                for file_name in files
            ],
        )
    )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='adelacruz@sfsu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_display_node = milton_final_project.face_display_node:main',
            'wayfinding_input_node = milton_final_project.wayfinding_input_node:main',
            'wayfinding_history_node = milton_final_project.wayfinding_history_node:main',
            'yolo_node = milton_final_project.yolo_node:main',
            'lidar_person_tracker_node = milton_final_project.lidar_person_tracker_node:main',
            'waiting_person_greeter_node = milton_final_project.waiting_person_greeter_node:main',
            'yolo_web_stream = milton_final_project.yolo_web_stream:main',
            'light_controller_node = milton_final_project.light_controller_node:main',
            'slam_keyboard_mapper_node = milton_final_project.slam_keyboard_mapper_node:main',
            'map_3d_viewer = milton_final_project.map_3d_viewer:main',
            'navigate_to_label = milton_final_project.navigate_to_label:main',
            'save_latest_map = milton_final_project.save_latest_map:main',
            'initial_pose_publisher = milton_final_project.initial_pose_publisher:main',
            'qbot_odometry_node = milton_final_project.qbot_odometry_node:main',
            'q_shutdown_node = milton_final_project.q_shutdown_node:main',
        ],
    },
)
