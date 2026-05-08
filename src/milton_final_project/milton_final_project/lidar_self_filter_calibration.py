import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty


class LidarSelfFilterCalibration(Node):
    def __init__(self, cmd_vel_topic, angular_z, duration_sec, publish_rate_hz):
        super().__init__('lidar_self_filter_calibration')
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.recalibrate_publisher = self.create_publisher(
            Empty,
            '/scan_sanitizer/recalibrate',
            10,
        )
        self.angular_z = angular_z
        self.duration_sec = duration_sec
        self.period_sec = 1.0 / publish_rate_hz
        self.cmd_vel_topic = cmd_vel_topic

    def run(self):
        self.get_logger().info('Requesting scan_sanitizer_node recalibration.')
        for _ in range(5):
            self.recalibrate_publisher.publish(Empty())
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        self.get_logger().info(
            f'Rotating at angular.z={self.angular_z:.3f} rad/s for '
            f'{self.duration_sec:.1f}s while scan_sanitizer_node learns '
            'persistent self-obstacle beams.'
        )
        deadline = time.monotonic() + self.duration_sec
        twist = Twist()
        twist.angular.z = self.angular_z

        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period_sec)

        stop = Twist()
        for _ in range(10):
            self.publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period_sec)
        self.get_logger().info('Calibration spin complete. Published stop.')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Slowly rotate QBot while LiDAR self-filter calibrates.',
    )
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--angular-z', type=float, default=0.22)
    parser.add_argument('--duration', type=float, default=16.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parsed_args = parser.parse_args(
        args=sys.argv[1:] if args is None else args
    )

    rclpy.init()
    node = LidarSelfFilterCalibration(
        parsed_args.cmd_vel_topic,
        parsed_args.angular_z,
        parsed_args.duration,
        parsed_args.rate,
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
