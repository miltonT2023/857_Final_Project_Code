import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


HELP_TEXT = """
Keyboard SLAM mapping controls:
  w: forward
  s: backward
  a: turn left
  d: turn right
  x or space: stop
  q: stop, save map, and quit
"""


class SlamKeyboardMapperNode(Node):
    def __init__(self):
        super().__init__('slam_keyboard_mapper_node')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.75)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'map_directory',
            os.path.join(os.getcwd(), 'maps'),
        )
        self.declare_parameter('map_name_prefix', 'mapped_area')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_directory = os.path.expanduser(
            str(self.get_parameter('map_directory').value)
        )
        self.map_name_prefix = str(self.get_parameter('map_name_prefix').value)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )
        self.latest_map = None
        self.current_twist = Twist()
        self.shutdown_requested = False
        self.shutdown_started = False
        self.exit_requested = False
        self.stop_burst_until = 0.0
        self.terminal_settings = None
        self.keyboard_stream = None
        self.close_keyboard_stream = False

        self.configure_terminal()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            f'Publishing keyboard drive commands to: {self.cmd_vel_topic}'
        )
        self.get_logger().info(f'Saving latest occupancy grid from: {self.map_topic}')
        self.get_logger().info(f'Maps will be saved in: {self.map_directory}')
        self.get_logger().info(HELP_TEXT.strip())

    def map_callback(self, msg):
        self.latest_map = msg

    def configure_terminal(self):
        if sys.stdin.isatty():
            self.keyboard_stream = sys.stdin
        else:
            try:
                self.keyboard_stream = open('/dev/tty', 'r')
                self.close_keyboard_stream = True
            except OSError:
                self.keyboard_stream = None

        if self.keyboard_stream is None:
            self.get_logger().warn(
                'Keyboard input needs an interactive terminal. '
                'Run this node from a terminal with output=screen.'
            )
            return

        self.terminal_settings = termios.tcgetattr(self.keyboard_stream)
        tty.setcbreak(self.keyboard_stream.fileno())
        self.get_logger().info('Keyboard input is connected.')

    def restore_terminal(self):
        if self.terminal_settings is not None and self.keyboard_stream is not None:
            termios.tcsetattr(
                self.keyboard_stream,
                termios.TCSADRAIN,
                self.terminal_settings,
            )

        self.terminal_settings = None
        if self.close_keyboard_stream and self.keyboard_stream is not None:
            self.keyboard_stream.close()
        self.keyboard_stream = None
        self.close_keyboard_stream = False

    def timer_callback(self):
        if not self.shutdown_requested:
            for key in self.read_keys():
                self.handle_key(key)

        if time.monotonic() < self.stop_burst_until:
            self.current_twist = Twist()

        self.cmd_vel_pub.publish(self.current_twist)

        if (
            self.shutdown_requested
            and not self.shutdown_started
            and time.monotonic() >= self.stop_burst_until
        ):
            self.shutdown_started = True
            self.get_logger().info('Robot stopped. Saving map now.')
            self.save_map()
            self.restore_terminal()
            self.exit_requested = True

    def read_keys(self):
        if self.keyboard_stream is None:
            return []

        keys = []
        readable, _, _ = select.select([self.keyboard_stream], [], [], 0.0)
        while readable:
            keys.append(self.keyboard_stream.read(1).lower())
            readable, _, _ = select.select([self.keyboard_stream], [], [], 0.0)

        return keys

    def handle_key(self, key):
        if key == 'w':
            self.set_motion(self.linear_speed, 0.0, 'forward')
        elif key == 's':
            self.set_motion(-self.linear_speed, 0.0, 'backward')
        elif key == 'a':
            self.set_motion(0.0, self.angular_speed, 'turn left')
        elif key == 'd':
            self.set_motion(0.0, -self.angular_speed, 'turn right')
        elif key in ('x', ' '):
            self.request_stop('stop')
        elif key == 'q':
            self.get_logger().info('Quit key received. Stopping robot now.')
            self.request_stop('quit stop', burst_sec=2.0)
            self.shutdown_requested = True
        else:
            self.get_logger().info(f'Ignoring unmapped key: {key}')

    def set_motion(self, linear_x, angular_z, label):
        self.stop_burst_until = 0.0
        self.current_twist = Twist()
        self.current_twist.linear.x = linear_x
        self.current_twist.angular.z = angular_z
        self.get_logger().info(f'Keyboard command: {label}')
        subscriber_count = self.count_subscribers(self.cmd_vel_topic)
        if subscriber_count == 0:
            self.get_logger().warn(
                f'No subscribers are currently listening on {self.cmd_vel_topic}. '
                'Check that qbot_platform started and the driver is armed.'
            )

    def request_stop(self, label, burst_sec=0.6):
        self.current_twist = Twist()
        self.stop_burst_until = time.monotonic() + burst_sec
        self.get_logger().info(f'Keyboard command: {label}')
        subscriber_count = self.count_subscribers(self.cmd_vel_topic)
        if subscriber_count == 0:
            self.get_logger().warn(
                f'No subscribers are currently listening on {self.cmd_vel_topic}. '
                'Check that qbot_platform started and the driver is armed.'
            )

    def stop_robot(self):
        self.current_twist = Twist()
        for _ in range(30):
            self.cmd_vel_pub.publish(self.current_twist)
            time.sleep(0.05)

    def save_map(self):
        if self.latest_map is None:
            self.get_logger().error(
                f'No map has been received on {self.map_topic}; nothing to save.'
            )
            return

        os.makedirs(self.map_directory, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        map_path = os.path.join(
            self.map_directory,
            f'{self.map_name_prefix}_{timestamp}',
        )
        image_path = f'{map_path}.pgm'
        yaml_path = f'{map_path}.yaml'

        self.write_pgm(image_path, self.latest_map)
        self.write_yaml(yaml_path, os.path.basename(image_path), self.latest_map)
        self.get_logger().info(f'Map save complete: {yaml_path}')

    def write_pgm(self, image_path, map_msg):
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

    def write_yaml(self, yaml_path, image_name, map_msg):
        origin = map_msg.info.origin
        yaw = self.quaternion_to_yaw(origin.orientation)
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

    def quaternion_to_yaw(self, quaternion):
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = SlamKeyboardMapperNode()

    try:
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.restore_terminal()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
