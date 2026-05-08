import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer
from tf2_ros import LookupException
from tf2_ros import TransformException
from tf2_ros import TransformListener

from milton_final_project.map_3d_viewer import DEFAULT_MAP_DIR
from milton_final_project.map_3d_viewer import find_latest_map
from milton_final_project.map_3d_viewer import load_labels
from milton_final_project.map_3d_viewer import write_labels


class SaveRobotStartPoseNode(Node):
    def __init__(self, map_frame, robot_frame):
        super().__init__('save_robot_start_pose')
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_pose(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        last_error = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except (LookupException, TransformException) as exc:
                last_error = exc

        if last_error is not None:
            raise RuntimeError(
                f'Could not get TF {self.map_frame} -> {self.robot_frame}: '
                f'{last_error}'
            )
        raise RuntimeError(
            f'No TF {self.map_frame} -> {self.robot_frame} received within '
            f'{timeout_sec:.1f} seconds.'
        )


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def resolve_map_name(map_dir, map_path):
    if map_path:
        return os.path.basename(os.path.abspath(os.path.expanduser(map_path)))

    latest_map = find_latest_map(map_dir)
    if latest_map is None:
        raise RuntimeError(f'No saved .yaml map found in {map_dir}')
    return os.path.basename(latest_map)


def parse_label_names(label, aliases):
    names = [label]
    names.extend(
        name.strip()
        for name in aliases.split(',')
        if name.strip()
    )
    clean_names = []
    for name in names:
        if name not in clean_names:
            clean_names.append(name)
    return clean_names


def resolve_yaw(parsed_args, measured_yaw):
    if parsed_args.yaw is not None and parsed_args.yaw_deg is not None:
        raise ValueError('Use either --yaw or --yaw-deg, not both.')
    if parsed_args.yaw is not None:
        return parsed_args.yaw
    if parsed_args.yaw_deg is not None:
        return math.radians(parsed_args.yaw_deg)
    return measured_yaw


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Save the robot current map pose into the map labels file.',
    )
    parser.add_argument('--map-dir', default=DEFAULT_MAP_DIR)
    parser.add_argument(
        '--map',
        default='',
        help='Map yaml filename/path. Defaults to latest map in --map-dir.',
    )
    parser.add_argument('--label', default='robot_start')
    parser.add_argument(
        '--aliases',
        default='start,home,original',
        help='Comma-separated extra labels that point to the same start pose.',
    )
    parser.add_argument(
        '--keep-existing-labels',
        action='store_true',
        help='Keep old labels instead of resetting this map label file first.',
    )
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--robot-frame', default='base_link')
    parser.add_argument(
        '--yaw',
        type=float,
        default=None,
        help='Override saved start heading in radians.',
    )
    parser.add_argument(
        '--yaw-deg',
        type=float,
        default=None,
        help='Override saved start heading in degrees.',
    )
    parser.add_argument('--timeout', type=float, default=10.0)
    parsed_args = parser.parse_args(
        args=sys.argv[1:] if args is None else args
    )

    map_dir = os.path.abspath(os.path.expanduser(parsed_args.map_dir))
    map_name = resolve_map_name(map_dir, parsed_args.map)

    rclpy.init()
    node = SaveRobotStartPoseNode(
        parsed_args.map_frame,
        parsed_args.robot_frame,
    )
    try:
        transform = node.wait_for_pose(parsed_args.timeout)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        labels = (
            load_labels(map_dir, map_name)['locations']
            if parsed_args.keep_existing_labels
            else {}
        )
        start_pose = {
            'x': translation.x,
            'y': translation.y,
            'yaw': resolve_yaw(parsed_args, quaternion_to_yaw(rotation)),
        }
        label_names = parse_label_names(parsed_args.label, parsed_args.aliases)
        for label_name in label_names:
            labels[label_name] = dict(start_pose)
        write_labels(map_dir, map_name, labels)
        node.get_logger().info(
            f'Saved start labels {", ".join(label_names)} for {map_name}: '
            f'x={translation.x:.3f}, y={translation.y:.3f}, '
            f'yaw={start_pose["yaw"]:.3f}'
        )
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
