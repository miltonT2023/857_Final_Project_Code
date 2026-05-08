import json
import math
import os
import statistics

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


class ScanSanitizerNode(Node):
    def __init__(self):
        super().__init__('scan_sanitizer_node')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_slam')
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 0.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('calibration_duration_sec', 8.0)
        self.declare_parameter('self_mask_max_range', 0.75)
        self.declare_parameter('self_mask_hit_ratio', 0.55)
        self.declare_parameter('speckle_filter_window', 0)
        self.declare_parameter('speckle_min_neighbors', 0)
        self.declare_parameter('speckle_max_range_delta', 0.0)
        self.declare_parameter('median_filter_window', 0)
        self.declare_parameter('median_max_range_delta', 0.0)
        self.declare_parameter('stamp_with_current_time', False)
        self.declare_parameter('stamp_offset_sec', 0.0)
        self.declare_parameter(
            'mask_file',
            '/home/nvidia/857_Final_Project_Code/maps/qbot_lidar_filter.json',
        )
        self.declare_parameter('mask_reload_period_sec', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.keep_angle_min = float(self.get_parameter('angle_min').value)
        self.keep_angle_max = float(self.get_parameter('angle_max').value)
        self.calibration_duration_sec = float(
            self.get_parameter('calibration_duration_sec').value
        )
        self.self_mask_max_range = float(
            self.get_parameter('self_mask_max_range').value
        )
        self.self_mask_hit_ratio = float(
            self.get_parameter('self_mask_hit_ratio').value
        )
        self.speckle_filter_window = max(
            0,
            int(self.get_parameter('speckle_filter_window').value),
        )
        self.speckle_min_neighbors = max(
            0,
            int(self.get_parameter('speckle_min_neighbors').value),
        )
        self.speckle_max_range_delta = max(
            0.0,
            float(self.get_parameter('speckle_max_range_delta').value),
        )
        self.median_filter_window = max(
            0,
            int(self.get_parameter('median_filter_window').value),
        )
        self.median_max_range_delta = max(
            0.0,
            float(self.get_parameter('median_max_range_delta').value),
        )
        self.stamp_offset_sec = float(
            self.get_parameter('stamp_offset_sec').value
        )
        self.stamp_with_current_time = bool(
            self.get_parameter('stamp_with_current_time').value
        )
        self.mask_file = self.get_parameter('mask_file').value
        self.mask_reload_period_sec = float(
            self.get_parameter('mask_reload_period_sec').value
        )
        self.calibration_started_at = self.get_clock().now()
        self.calibration_complete = self.calibration_duration_sec <= 0.0
        self.self_hit_counts = []
        self.self_total_counts = []
        self.beam_mask = set()
        self.ignore_regions = []
        self.last_mask_mtime = None
        self.last_mask_load_check = 0.0
        self.load_mask_file(force=True)

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Empty,
            '/scan_sanitizer/recalibrate',
            self.recalibrate_callback,
            10,
        )

        self.get_logger().info(
            f'Republishing sanitized scans: {input_topic} -> {output_topic}; '
            f'ignoring ranges below {self.range_min:.2f} m'
        )
        if not self.calibration_complete:
            self.get_logger().info(
                'Learning robot self-filter for '
                f'{self.calibration_duration_sec:.1f}s. Slowly rotate the '
                'robot so poles/cables stay fixed in robot frame while room '
                'features move.'
            )
        if self.mask_file:
            self.get_logger().info(f'Using LiDAR ignore mask: {self.mask_file}')

    def recalibrate_callback(self, _msg):
        self.calibration_started_at = self.get_clock().now()
        self.calibration_complete = self.calibration_duration_sec <= 0.0
        self.self_hit_counts = []
        self.self_total_counts = []
        self.beam_mask = set()
        self.load_mask_file(force=True)
        self.get_logger().info(
            'Restarting LiDAR self-filter learning for '
            f'{self.calibration_duration_sec:.1f}s.'
        )

    def scan_callback(self, msg):
        self.maybe_reload_mask_file()
        sanitized = LaserScan()
        sanitized.header = msg.header
        if self.stamp_with_current_time:
            stamp = self.get_clock().now()
        else:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        if self.stamp_offset_sec != 0.0:
            stamp += Duration(seconds=self.stamp_offset_sec)

        sanitized.header.stamp = stamp.to_msg()
        sanitized.angle_min = msg.angle_min
        sanitized.angle_increment = msg.angle_increment
        sanitized.time_increment = msg.time_increment
        sanitized.scan_time = msg.scan_time
        sanitized.range_min = max(self.range_min, msg.range_min)
        sanitized.range_max = (
            min(self.range_max, msg.range_max)
            if self.range_max > 0.0
            else msg.range_max
        )
        sanitized.intensities = msg.intensities

        self.update_self_mask_calibration(msg, sanitized.range_min)

        sanitized.ranges = []
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + msg.angle_increment * index
            if (
                not math.isfinite(value)
                or value < sanitized.range_min
                or value > sanitized.range_max
                or angle < self.keep_angle_min
                or angle > self.keep_angle_max
                or index in self.beam_mask
                or self.point_is_ignored(value, angle)
            ):
                sanitized.ranges.append(float('inf'))
            else:
                sanitized.ranges.append(value)
        sanitized.ranges = self.filter_speckles(sanitized.ranges)
        sanitized.ranges = self.filter_small_jitter(sanitized.ranges)
        if sanitized.ranges:
            sanitized.angle_max = (
                sanitized.angle_min
                + sanitized.angle_increment * float(len(sanitized.ranges) - 1)
            )
        else:
            sanitized.angle_max = msg.angle_max

        self.publisher.publish(sanitized)

    def filter_speckles(self, ranges):
        if (
            self.speckle_filter_window <= 0
            or self.speckle_min_neighbors <= 0
            or self.speckle_max_range_delta <= 0.0
        ):
            return ranges

        filtered = list(ranges)
        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue

            neighbors = 0
            start = max(0, index - self.speckle_filter_window)
            stop = min(len(ranges), index + self.speckle_filter_window + 1)
            for neighbor_index in range(start, stop):
                if neighbor_index == index:
                    continue
                neighbor_value = ranges[neighbor_index]
                if not math.isfinite(neighbor_value):
                    continue
                if abs(neighbor_value - value) <= self.speckle_max_range_delta:
                    neighbors += 1

            if neighbors < self.speckle_min_neighbors:
                filtered[index] = float('inf')

        return filtered

    def filter_small_jitter(self, ranges):
        if (
            self.median_filter_window < 3
            or self.median_filter_window % 2 == 0
            or self.median_max_range_delta <= 0.0
        ):
            return ranges

        radius = self.median_filter_window // 2
        filtered = list(ranges)
        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue

            start = max(0, index - radius)
            stop = min(len(ranges), index + radius + 1)
            neighborhood = [
                neighbor_value
                for neighbor_value in ranges[start:stop]
                if math.isfinite(neighbor_value)
            ]
            if len(neighborhood) < self.median_filter_window:
                continue

            median_value = statistics.median(neighborhood)
            if abs(median_value - value) <= self.median_max_range_delta:
                filtered[index] = float(median_value)

        return filtered

    def update_self_mask_calibration(self, msg, sanitized_range_min):
        if self.calibration_complete:
            return

        if not self.self_hit_counts:
            self.self_hit_counts = [0] * len(msg.ranges)
            self.self_total_counts = [0] * len(msg.ranges)

        for index, value in enumerate(msg.ranges):
            if index >= len(self.self_hit_counts):
                break
            if not math.isfinite(value):
                continue
            self.self_total_counts[index] += 1
            if sanitized_range_min <= value <= self.self_mask_max_range:
                self.self_hit_counts[index] += 1

        elapsed = (
            self.get_clock().now() - self.calibration_started_at
        ).nanoseconds / 1_000_000_000.0
        if elapsed < self.calibration_duration_sec:
            return

        self.calibration_complete = True
        learned_mask = set()
        for index, total_count in enumerate(self.self_total_counts):
            if total_count <= 0:
                continue
            hit_ratio = self.self_hit_counts[index] / float(total_count)
            if hit_ratio >= self.self_mask_hit_ratio:
                learned_mask.add(index)

        self.beam_mask = learned_mask
        self.save_mask_file()
        self.get_logger().info(
            f'Learned LiDAR self-filter: masking {len(self.beam_mask)} '
            f'of {len(self.self_total_counts)} beams with close persistent '
            f'returns <= {self.self_mask_max_range:.2f} m.'
        )

    def maybe_reload_mask_file(self):
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now_sec - self.last_mask_load_check < self.mask_reload_period_sec:
            return
        self.last_mask_load_check = now_sec
        self.load_mask_file()

    def load_mask_file(self, force=False):
        if not self.mask_file or not os.path.exists(self.mask_file):
            return
        try:
            mtime = os.path.getmtime(self.mask_file)
            if not force and mtime == self.last_mask_mtime:
                return
            with open(self.mask_file, 'r', encoding='utf-8') as file:
                payload = json.load(file)
        except (OSError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f'Could not load LiDAR mask file {self.mask_file}: {error}'
            )
            return

        regions = []
        for region in payload.get('regions', []):
            points = region.get('points', [])
            clean_points = []
            for point in points:
                try:
                    clean_points.append((float(point['x']), float(point['y'])))
                except (KeyError, TypeError, ValueError):
                    continue
            if len(clean_points) >= 3:
                regions.append(clean_points)

        self.ignore_regions = regions
        self.beam_mask = set(int(index) for index in payload.get('beam_indices', []))
        self.last_mask_mtime = mtime
        self.get_logger().info(
            f'Loaded LiDAR mask: {len(self.ignore_regions)} regions, '
            f'{len(self.beam_mask)} beam indices.'
        )

    def save_mask_file(self):
        if not self.mask_file:
            return
        regions = []
        if os.path.exists(self.mask_file):
            try:
                with open(self.mask_file, 'r', encoding='utf-8') as file:
                    regions = json.load(file).get('regions', [])
            except (OSError, json.JSONDecodeError):
                regions = []
        payload = {
            'regions': regions,
            'beam_indices': sorted(self.beam_mask),
        }
        try:
            os.makedirs(os.path.dirname(self.mask_file), exist_ok=True)
            with open(self.mask_file, 'w', encoding='utf-8') as file:
                json.dump(payload, file, indent=2, sort_keys=True)
            self.last_mask_mtime = os.path.getmtime(self.mask_file)
        except OSError as error:
            self.get_logger().warning(
                f'Could not save LiDAR mask file {self.mask_file}: {error}'
            )

    def point_is_ignored(self, range_value, angle):
        if not self.ignore_regions:
            return False
        x = range_value * math.cos(angle)
        y = range_value * math.sin(angle)
        return any(point_in_polygon(x, y, region) for region in self.ignore_regions)


def point_in_polygon(x, y, polygon):
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        crosses_y = (current_y > y) != (previous_y > y)
        if crosses_y:
            slope_x = (
                (previous_x - current_x) * (y - current_y)
                / (previous_y - current_y)
                + current_x
            )
            if x < slope_x:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
