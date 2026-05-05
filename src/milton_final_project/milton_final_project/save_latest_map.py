import argparse
import math
import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


DEFAULT_MAP_DIR = '/home/nvidia/857_Final_Project_Code/maps'


class SaveLatestMapNode(Node):
    def __init__(self, map_topic):
        super().__init__('save_latest_map')
        self.latest_map = None
        self.map_topic = map_topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )

    def map_callback(self, msg):
        self.latest_map = msg

    def wait_for_map(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.latest_map is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.latest_map is not None


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def write_pgm(image_path, map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    data = map_msg.data

    with open(image_path, 'wb') as image_file:
        header = f'P5\n# CREATOR: milton_final_project\n{width} {height}\n255\n'
        image_file.write(header.encode('ascii'))

        for y in range(height - 1, -1, -1):
            row_start = y * width
            row = bytearray()
            for x in range(width):
                occupancy = data[row_start + x]
                if occupancy < 0:
                    row.append(205)
                elif occupancy >= 65:
                    row.append(0)
                elif occupancy <= 25:
                    row.append(254)
                else:
                    row.append(205)
            image_file.write(row)


def write_yaml(yaml_path, image_name, map_msg):
    origin = map_msg.info.origin
    yaw = quaternion_to_yaw(origin.orientation)
    yaml_text = (
        f'image: {image_name}\n'
        'mode: trinary\n'
        f'resolution: {map_msg.info.resolution:.6f}\n'
        f'origin: [{origin.position.x:.6f}, {origin.position.y:.6f}, {yaw:.6f}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )

    with open(yaml_path, 'w', encoding='utf-8') as yaml_file:
        yaml_file.write(yaml_text)


def save_map(map_msg, map_directory, map_name_prefix):
    os.makedirs(map_directory, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    map_path = os.path.join(map_directory, f'{map_name_prefix}_{timestamp}')
    image_path = f'{map_path}.pgm'
    yaml_path = f'{map_path}.yaml'

    write_pgm(image_path, map_msg)
    write_yaml(yaml_path, os.path.basename(image_path), map_msg)
    return yaml_path


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Save the latest OccupancyGrid map from a running mapping launch.',
    )
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--map-dir', default=DEFAULT_MAP_DIR)
    parser.add_argument('--map-name-prefix', default='mapped_area')
    parser.add_argument('--timeout', type=float, default=10.0)
    parsed_args = parser.parse_args(args=args)

    rclpy.init()
    node = SaveLatestMapNode(parsed_args.map_topic)
    try:
        node.get_logger().info(f'Waiting for map on {parsed_args.map_topic}...')
        if not node.wait_for_map(parsed_args.timeout):
            node.get_logger().error(
                f'No map received on {parsed_args.map_topic} within '
                f'{parsed_args.timeout:.1f} seconds.'
            )
            raise SystemExit(1)

        yaml_path = save_map(
            node.latest_map,
            os.path.abspath(os.path.expanduser(parsed_args.map_dir)),
            parsed_args.map_name_prefix,
        )
        node.get_logger().info(f'Map save complete: {yaml_path}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
