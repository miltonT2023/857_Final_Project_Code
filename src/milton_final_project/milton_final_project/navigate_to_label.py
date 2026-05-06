import argparse
import math
import os
import select
import sys
import threading
import time

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Empty


DEFAULT_LABELS_FILE = (
    '/home/nvidia/857_Final_Project_Code/maps/map_labels.yaml'
)
DEFAULT_MAP_DIR = '/home/nvidia/857_Final_Project_Code/maps'
LEGACY_LABELS_FILE_NAME = 'map_labels.yaml'
SHUTDOWN_TOPIC = '/milton_final_project/shutdown'


def find_latest_map(map_dir):
    if not os.path.isdir(map_dir):
        return None

    yaml_paths = [
        os.path.join(map_dir, name)
        for name in os.listdir(map_dir)
        if name.endswith('.yaml')
        and not name.endswith('.labels.yaml')
        and name != LEGACY_LABELS_FILE_NAME
    ]
    if not yaml_paths:
        return None

    return max(yaml_paths, key=os.path.getmtime)


def labels_file_for_map(map_yaml_path):
    base, _ = os.path.splitext(map_yaml_path)
    return f'{base}.labels.yaml'


def default_labels_file():
    latest_map = find_latest_map(DEFAULT_MAP_DIR)
    if latest_map is None:
        return DEFAULT_LABELS_FILE

    return labels_file_for_map(latest_map)


def load_labels(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f'Labels file not found: {path}')

    labels = {}
    current_name = None
    with open(path, 'r', encoding='utf-8') as label_file:
        for raw_line in label_file:
            line = raw_line.rstrip()
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if line.startswith('  ') and stripped.endswith(':'):
                current_name = stripped[:-1]
                labels[current_name] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                continue
            if line.startswith('    ') and current_name and ':' in stripped:
                key, value = stripped.split(':', 1)
                if key in ('x', 'y', 'yaw'):
                    labels[current_name][key] = float(value.strip())

    return labels


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return {
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def make_initial_pose(x, y, yaw, stamp):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.pose.orientation.w = math.cos(yaw * 0.5)
    pose.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
    ]
    return pose


class NavigateToLabelNode(Node):
    def __init__(
        self,
        label_name,
        labels_file,
        action_name,
        wait_timeout,
        initial_x,
        initial_y,
        initial_yaw,
        current_pose_topic='/amcl_pose',
        current_pose_timeout=10.0,
        current_odom_topic='/odom',
    ):
        super().__init__('navigate_to_label')
        self.label_name = label_name
        self.labels_file = labels_file
        self.action_name = action_name
        self.wait_timeout = wait_timeout
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_yaw = initial_yaw
        self.current_pose_topic = current_pose_topic
        self.current_pose_timeout = current_pose_timeout
        self.current_odom_topic = current_odom_topic
        self.current_robot_pose = None
        self.current_robot_pose_source = None
        initial_pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_qos,
        )
        self.shutdown_publisher = self.create_publisher(
            Empty,
            SHUTDOWN_TOPIC,
            10,
        )
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name,
        )
        self.current_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.current_pose_topic,
            self.current_pose_callback,
            10,
        )
        self.current_odom_subscription = self.create_subscription(
            Odometry,
            self.current_odom_topic,
            self.current_odom_callback,
            10,
        )
        self.shutdown_requested = False
        self.goal_x = None
        self.goal_y = None
        self.last_feedback_log_time = 0.0
        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.keyboard_thread.start()

    def current_pose_callback(self, msg):
        self.current_robot_pose = msg.pose
        self.current_robot_pose_source = self.current_pose_topic

    def current_odom_callback(self, msg):
        if self.current_robot_pose_source == self.current_pose_topic:
            return

        self.current_robot_pose = msg.pose
        self.current_robot_pose_source = self.current_odom_topic

    def send_goal(self):
        self.get_logger().info(f'Loading labels from: {self.labels_file}')
        labels = load_labels(self.labels_file)
        if self.label_name not in labels:
            names = ', '.join(sorted(labels)) or 'none'
            raise KeyError(
                f'Label "{self.label_name}" was not found. '
                f'Available labels: {names}'
            )

        pose = labels[self.label_name]
        self.send_named_goal(
            f'label "{self.label_name}"',
            float(pose['x']),
            float(pose['y']),
            float(pose.get('yaw', 0.0)),
        )

    def send_named_goal(self, goal_name, goal_x, goal_y, goal_yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_x = goal_x
        self.goal_y = goal_y
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        quaternion = yaw_to_quaternion(goal_yaw)
        goal.pose.pose.orientation.z = quaternion['z']
        goal.pose.pose.orientation.w = quaternion['w']

        self.get_logger().info(
            f'Sending {goal_name} goal: '
            f'x={goal_x:.3f}, y={goal_y:.3f}, yaw={goal_yaw:.3f}'
        )
        self.get_logger().info(
            f'Waiting for Nav2 action server: {self.action_name}'
        )
        if not self.wait_for_nav2_action_server():
            raise RuntimeError(
                f'Nav2 action server "{self.action_name}" is not available. '
                f'Visible action servers: {self.visible_action_names()}. '
                'Keep qbot_navigation_launch.py running in another terminal '
                'and wait until Nav2 finishes activating.'
            )
        self.wait_for_nav2_active()
        self.publish_initial_pose()
        time.sleep(1.0)

        send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        self.spin_until_future_or_shutdown(send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(
                'Nav2 rejected the navigation goal. Usually this means Nav2 '
                'is not fully active yet, AMCL has no initial pose, or the '
                'selected label is not reachable on the loaded map. Check '
                'lifecycle states with: ros2 lifecycle get /bt_navigator and '
                'check that the label is in free space on the map.'
            )

        self.get_logger().info('Goal accepted. Navigating...')
        result_future = goal_handle.get_result_async()
        while not result_future.done() and rclpy.ok():
            if self.shutdown_requested:
                self.get_logger().info('Canceling current navigation goal.')
                cancel_future = goal_handle.cancel_goal_async()
                self.spin_until_future_or_shutdown(
                    cancel_future,
                    allow_shutdown=True,
                )
                raise RuntimeError('Navigation stopped by q.')
            rclpy.spin_once(self, timeout_sec=0.1)
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Arrived at {goal_name}. Exiting.'
            )
            return

        raise RuntimeError(
            f'Navigation finished without arriving. Status: {result.status}'
        )

    def feedback_callback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_log_time < 1.0:
            return
        self.last_feedback_log_time = now

        feedback = feedback_msg.feedback
        current_x = feedback.current_pose.pose.position.x
        current_y = feedback.current_pose.pose.position.y

        if self.goal_x is None or self.goal_y is None:
            self.get_logger().info('Navigation feedback received.')
            return

        straight_line_distance = math.hypot(
            self.goal_x - current_x,
            self.goal_y - current_y,
        )
        self.get_logger().info(
            f'Straight-line distance to label: {straight_line_distance:.2f} m'
        )

    def wait_for_nav2_action_server(self):
        deadline = time.monotonic() + self.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.raise_if_shutdown_requested()
            if self.action_client.wait_for_server(timeout_sec=1.0):
                return True

            remaining = max(0.0, deadline - time.monotonic())
            self.get_logger().info(
                f'Still waiting for {self.action_name}. '
                f'Visible actions: {self.visible_action_names()}. '
                f'Time left: {remaining:.0f}s'
            )

        return False

    def visible_action_names(self):
        action_service_suffixes = (
            '/_action/cancel_goal',
            '/_action/get_result',
            '/_action/send_goal',
        )
        action_names = set()
        for service_name, _ in self.get_service_names_and_types():
            for suffix in action_service_suffixes:
                if service_name.endswith(suffix):
                    action_names.add(service_name[:-len(suffix)])

        return ', '.join(sorted(action_names)) or 'none'

    def wait_for_nav2_active(self):
        lifecycle_nodes = [
            '/amcl',
            '/bt_navigator',
            '/planner_server',
            '/controller_server',
        ]
        deadline = time.monotonic() + self.wait_timeout
        pending = set(lifecycle_nodes)

        while rclpy.ok() and pending and time.monotonic() < deadline:
            self.raise_if_shutdown_requested()
            for node_name in list(pending):
                state = self.lifecycle_state(node_name)
                if state == 'active':
                    pending.remove(node_name)
                elif state:
                    self.get_logger().info(
                        f'Waiting for {node_name} to become active. '
                        f'Current: {state}'
                    )
                else:
                    self.get_logger().info(
                        f'Waiting for lifecycle service from {node_name}'
                    )
            if pending:
                time.sleep(1.0)

        if pending:
            names = ', '.join(sorted(pending))
            raise RuntimeError(
                f'Nav2 is not fully active yet. Still waiting on: {names}. '
                f'Check one with: ros2 lifecycle get {sorted(pending)[0]}'
            )

    def lifecycle_state(self, node_name):
        client = self.create_client(GetState, f'{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=0.25):
            self.destroy_client(client)
            return None

        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self.destroy_client(client)
        if not future.done() or future.result() is None:
            return None

        return future.result().current_state.label

    def publish_initial_pose(self):
        self.use_current_pose_as_initial_pose()
        self.get_logger().info(
            f'Publishing initial pose before goal: x={self.initial_x:.3f}, '
            f'y={self.initial_y:.3f}, yaw={self.initial_yaw:.3f}'
        )
        wait_deadline = time.monotonic() + 10.0
        while (
            self.initial_pose_publisher.get_subscription_count() == 0
            and time.monotonic() < wait_deadline
        ):
            self.raise_if_shutdown_requested()
            self.get_logger().info(
                'Waiting for AMCL to subscribe to /initialpose...'
            )
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.3)

        for _ in range(20):
            self.raise_if_shutdown_requested()
            msg = make_initial_pose(
                self.initial_x,
                self.initial_y,
                self.initial_yaw,
                self.get_clock().now().to_msg(),
            )
            self.initial_pose_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)

    def use_current_pose_as_initial_pose(self):
        self.get_logger().info(
            'Waiting for current robot pose from '
            f'{self.current_pose_topic} or {self.current_odom_topic}...'
        )
        deadline = time.monotonic() + self.current_pose_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.raise_if_shutdown_requested()
            if self.current_robot_pose is not None:
                pose = self.current_robot_pose.pose
                self.initial_x = pose.position.x
                self.initial_y = pose.position.y
                self.initial_yaw = quaternion_to_yaw(pose.orientation)
                self.get_logger().info(
                    'Using current robot pose from '
                    f'{self.current_robot_pose_source} as initial pose: '
                    f'x={self.initial_x:.3f}, '
                    f'y={self.initial_y:.3f}, '
                    f'yaw={self.initial_yaw:.3f}'
                )
                return

            rclpy.spin_once(self, timeout_sec=0.2)

        raise RuntimeError(
            'No current robot pose received on '
            f'{self.current_pose_topic} or {self.current_odom_topic}. '
            'Make sure qbot_navigation_launch.py is running.'
        )

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            return

        while rclpy.ok() and not self.shutdown_requested:
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue

            command = sys.stdin.readline().strip().lower()
            if command == 'q':
                self.get_logger().info(
                    'q pressed. Requesting navigation launch shutdown.'
                )
                self.request_shutdown()
                return

    def request_shutdown(self):
        self.shutdown_requested = True
        for _ in range(5):
            self.shutdown_publisher.publish(Empty())
            time.sleep(0.05)

    def raise_if_shutdown_requested(self):
        if self.shutdown_requested:
            raise RuntimeError('Navigation stopped by q.')

    def spin_until_future_or_shutdown(self, future, allow_shutdown=False):
        while not future.done() and rclpy.ok():
            if self.shutdown_requested and not allow_shutdown:
                raise RuntimeError('Navigation stopped by q.')
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Send a Nav2 goal from maps/map_labels.yaml.',
    )
    parser.add_argument(
        'label',
        help='Label name, for example entrance or room_101.',
    )
    parser.add_argument(
        '--labels-file',
        default=None,
        help='Path to a map-specific .labels.yaml file.',
    )
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
    parser.add_argument('--initial-x', default=0.0, type=float)
    parser.add_argument('--initial-y', default=0.0, type=float)
    parser.add_argument('--initial-yaw', default=0.0, type=float)
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
    labels_file = parsed_args.labels_file or default_labels_file()

    rclpy.init()
    node = NavigateToLabelNode(
        parsed_args.label,
        labels_file,
        parsed_args.action_name,
        parsed_args.wait_timeout,
        parsed_args.initial_x,
        parsed_args.initial_y,
        parsed_args.initial_yaw,
        parsed_args.current_pose_topic,
        parsed_args.current_pose_timeout,
        parsed_args.current_odom_topic,
    )
    try:
        node.send_goal()
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
