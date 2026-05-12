import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    nav2_dir = get_package_share_directory("nav2_bringup")

    maps_dir = "/home/nvidia/857_Final_Project_Code/maps"
    map_name = LaunchConfiguration("map_name")
    labels_file = LaunchConfiguration("labels_file")
    use_breadcrumb_return = LaunchConfiguration("use_breadcrumb_return")
    default_map = PythonExpression(["'", maps_dir, "/' + '", map_name, "' + '.yaml'"])
    default_labels_file = PythonExpression([
        "'", maps_dir, "/' + '", map_name, "' + '_labels.json'"
    ])
    params_file = os.path.join(package_dir, "config", "qbot_platform_slam_and_nav.yaml")

    map_name_arg = DeclareLaunchArgument("map_name", default_value="lab_map_new")
    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    labels_file_arg = DeclareLaunchArgument("labels_file", default_value=default_labels_file)
    use_breadcrumb_return_arg = DeclareLaunchArgument(
        "use_breadcrumb_return",
        default_value="true",
        description="Record sparse outbound breadcrumbs and use them for return-home.",
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, "launch", "qbot_platform_launch.py"))
    )

    lidar_tf_node = Node(
        package="qbot_platform",
        executable="fixed_lidar_frame",
        name="fixed_lidar_frame",
        output="screen",
    )

    wheel_odom_node = Node(
        package="qbot_platform",
        executable="wheel_odometry.py",
        name="wheel_odometry",
        output="screen",
        parameters=[
            {
                "imu_angular_velocity_scale": 0.970,
                "use_imu_yaw": False,
            }
        ],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "localization_launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": "False",
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )

    go_to_label_node = Node(
        package="qbot_platform",
        executable="go_to_label.py",
        name="go_to_label",
        output="screen",
        arguments=["--labels-file", labels_file],
    )

    breadcrumb_return_node = Node(
        condition=IfCondition(use_breadcrumb_return),
        package="qbot_platform",
        executable="breadcrumb_return.py",
        name="breadcrumb_return",
        output="screen",
    )

    return LaunchDescription(
        [
            map_name_arg,
            map_arg,
            labels_file_arg,
            use_breadcrumb_return_arg,
            base_launch,
            lidar_tf_node,
            wheel_odom_node,
            localization_launch,
            navigation_launch,
            go_to_label_node,
            breadcrumb_return_node,
        ]
    )
