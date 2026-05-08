import argparse
import math
import os
import time

import cv2
import numpy as np
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


def filter_occupancy_data(
    map_msg,
    min_occupied_component_cells,
    close_kernel_cells,
):
    width = map_msg.info.width
    height = map_msg.info.height
    occupancy = np.array(map_msg.data, dtype=np.int16).reshape((height, width))
    occupied = (occupancy >= 65).astype(np.uint8)

    if close_kernel_cells >= 2:
        kernel_cells = close_kernel_cells
        if kernel_cells % 2 == 0:
            kernel_cells += 1
        horizontal_kernel = np.ones((1, kernel_cells), dtype=np.uint8)
        vertical_kernel = np.ones((kernel_cells, 1), dtype=np.uint8)
        horizontal = cv2.morphologyEx(
            occupied,
            cv2.MORPH_CLOSE,
            horizontal_kernel,
        )
        vertical = cv2.morphologyEx(
            occupied,
            cv2.MORPH_CLOSE,
            vertical_kernel,
        )
        occupied = np.maximum.reduce([occupied, horizontal, vertical])

    if min_occupied_component_cells > 1:
        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            occupied,
            connectivity=8,
        )
        cleaned = np.zeros_like(occupied)
        for component_id in range(1, component_count):
            area = stats[component_id, cv2.CC_STAT_AREA]
            if area >= min_occupied_component_cells:
                cleaned[labels == component_id] = 1
        occupied = cleaned

    filtered = occupancy.copy()
    removed_noise = (occupancy >= 65) & (occupied == 0)
    added_wall_fill = (occupancy < 65) & (occupied == 1)
    filtered[removed_noise] = -1
    filtered[added_wall_fill] = 100
    return filtered.reshape(-1).tolist()


def write_pgm(
    image_path,
    map_msg,
    clean_map=True,
    min_occupied_component_cells=4,
    close_kernel_cells=3,
):
    width = map_msg.info.width
    height = map_msg.info.height
    data = (
        filter_occupancy_data(
            map_msg,
            min_occupied_component_cells,
            close_kernel_cells,
        )
        if clean_map
        else map_msg.data
    )

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


def save_map(
    map_msg,
    map_directory,
    map_name_prefix,
    clean_map=True,
    min_occupied_component_cells=4,
    close_kernel_cells=3,
):
    os.makedirs(map_directory, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    map_path = os.path.join(map_directory, f'{map_name_prefix}_{timestamp}')
    image_path = f'{map_path}.pgm'
    yaml_path = f'{map_path}.yaml'

    write_pgm(
        image_path,
        map_msg,
        clean_map=clean_map,
        min_occupied_component_cells=min_occupied_component_cells,
        close_kernel_cells=close_kernel_cells,
    )
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
    parser.add_argument(
        '--raw',
        action='store_true',
        help='Save the raw occupancy grid without noise cleanup.',
    )
    parser.add_argument(
        '--min-occupied-component-cells',
        type=int,
        default=4,
        help='Drop occupied blobs smaller than this many cells.',
    )
    parser.add_argument(
        '--close-kernel-cells',
        type=int,
        default=3,
        help='Morphological close kernel size for filling tiny wall gaps.',
    )
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
            clean_map=not parsed_args.raw,
            min_occupied_component_cells=max(
                1,
                parsed_args.min_occupied_component_cells,
            ),
            close_kernel_cells=max(1, parsed_args.close_kernel_cells),
        )
        node.get_logger().info(f'Map save complete: {yaml_path}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
