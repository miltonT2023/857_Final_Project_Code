import select
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty


SHUTDOWN_TOPIC = '/milton_final_project/shutdown'


class QShutdownNode(Node):
    def __init__(self):
        super().__init__('q_shutdown_node')
        self.shutdown_requested = threading.Event()
        self.create_subscription(
            Empty,
            SHUTDOWN_TOPIC,
            self.shutdown_callback,
            10,
        )
        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.keyboard_thread.start()
        self.get_logger().info(
            'Type q and press Enter in this terminal to stop '
            'navigation launch.'
        )

    def shutdown_callback(self, _msg):
        self.get_logger().info('Shutdown request received.')
        self.shutdown_requested.set()

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            return

        while rclpy.ok() and not self.shutdown_requested.is_set():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue

            command = sys.stdin.readline().strip().lower()
            if command == 'q':
                self.get_logger().info(
                    'q pressed. Shutting down navigation launch.'
                )
                self.shutdown_requested.set()
                return


def main(args=None):
    rclpy.init(args=args)
    node = QShutdownNode()
    try:
        while rclpy.ok() and not node.shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
