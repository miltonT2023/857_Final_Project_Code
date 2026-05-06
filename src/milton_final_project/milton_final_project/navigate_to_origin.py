import argparse
import sys

import rclpy

from milton_final_project.navigate_to_label import NavigateToLabelNode


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Send a Nav2 goal to the map origin.',
    )
    parser.add_argument('--x', default=0.0, type=float)
    parser.add_argument('--y', default=0.0, type=float)
    parser.add_argument('--yaw', default=0.0, type=float)
    parser.add_argument(
        '--action-name',
        default='/navigate_to_pose',
        help='Nav2 NavigateToPose action name.',
    )
    parser.add_argument(
        '--wait-timeout',
        default=90.0,
        type=float,
        help='Seconds to wait for Nav2 to become available.',
    )
    parser.add_argument(
        '--current-pose-topic',
        default='/amcl_pose',
        help='Navigation pose topic to use for the AMCL initial pose.',
    )
    parser.add_argument(
        '--current-pose-timeout',
        default=10.0,
        type=float,
        help='Seconds to wait for the current navigation pose.',
    )
    parser.add_argument(
        '--current-odom-topic',
        default='/odom',
        help='Odometry topic to use when AMCL has not published a pose yet.',
    )
    parsed_args = parser.parse_args(
        args=sys.argv[1:] if args is None else args
    )

    rclpy.init()
    node = NavigateToLabelNode(
        'map_origin',
        '',
        parsed_args.action_name,
        parsed_args.wait_timeout,
        0.0,
        0.0,
        0.0,
        parsed_args.current_pose_topic,
        parsed_args.current_pose_timeout,
        parsed_args.current_odom_topic,
    )
    try:
        node.send_named_goal(
            'map origin',
            parsed_args.x,
            parsed_args.y,
            parsed_args.yaw,
        )
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
