import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self, x, y, yaw, publish_count, publish_period_sec):
        super().__init__('initial_pose_publisher')
        self.x = x
        self.y = y
        self.yaw = yaw
        self.publish_count = publish_count
        self.publish_period_sec = publish_period_sec
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos,
        )

    def publish_initial_pose(self):
        wait_deadline = time.monotonic() + 10.0
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < wait_deadline:
            self.get_logger().info('Waiting for AMCL to subscribe to /initialpose...')
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.3)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]

        self.get_logger().info(
            f'Publishing initial pose: x={self.x:.3f}, '
            f'y={self.y:.3f}, yaw={self.yaw:.3f}'
        )
        for _ in range(self.publish_count):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(self.publish_period_sec)


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish AMCL initial pose.')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--period', type=float, default=0.5)
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:] if args is None else args)

    rclpy.init()
    node = InitialPosePublisher(
        parsed_args.x,
        parsed_args.y,
        parsed_args.yaw,
        parsed_args.count,
        parsed_args.period,
    )
    try:
        node.publish_initial_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
