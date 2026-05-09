#!/usr/bin/env python3
"""Send a Nav2 goal using a saved map label."""

import argparse
import json
import math
import re
import sys
from pathlib import Path

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


def workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "maps").is_dir():
            return parent
    return Path("/home/nvidia/857_Final_Project_Code")


ROOT = workspace_root()
DEFAULT_LABELS_FILE = ROOT / "maps" / "lab_map_new_labels.json"
LAST_LABEL_FILE = ROOT / "maps" / "last_navigation_label.json"

ALIASES = {
    "dr zhang": "xiaorong zhang",
    "dr. zhang": "xiaorong zhang",
    "zhang": "xiaorong zhang",
    "313": "seic 313",
}


def normalize(text):
    return re.sub(r"\s+", " ", text.strip().lower())


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_labels(path):
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    labels = data.get("labels", [])
    if not isinstance(labels, list):
        raise ValueError(f"{path} does not contain a labels list")
    return labels


def label_display(label):
    detail = label.get("detail")
    if detail:
        return f"{label.get('name', '<unnamed>')} ({detail})"
    return label.get("name", "<unnamed>")


def list_labels(labels):
    for label in labels:
        world = label.get("world") or {}
        x = world.get("x")
        y = world.get("y")
        if x is None or y is None:
            continue
        print(f"{label_display(label)} -> x={float(x):.3f}, y={float(y):.3f}")


def find_label(labels, query):
    wanted = ALIASES.get(normalize(query), normalize(query))

    candidates = []
    for label in labels:
        names = [
            label.get("name", ""),
            label.get("detail", ""),
            label.get("kind", ""),
        ]
        normalized_names = [normalize(name) for name in names if name]
        if wanted in normalized_names:
            return label
        if any(wanted in name for name in normalized_names):
            candidates.append(label)

    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        names = ", ".join(label_display(label) for label in candidates)
        raise ValueError(f"Multiple labels match {query!r}: {names}")
    raise ValueError(f"No saved label matches {query!r}")


class LabelNavigator(Node):
    def __init__(self, action_name):
        super().__init__("go_to_label")
        self.action_name = action_name
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.labels = []
        self.server_timeout_sec = 10.0
        self.active_goal_label = None

    def go_to(self, label, timeout_sec):
        world = label.get("world") or {}
        x = float(world["x"])
        y = float(world["y"])
        yaw = float(label.get("yaw", 0.0))
        qz, qw = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Waiting for Nav2 action server, then going to {label_display(label)} "
            f"at x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )
        if not self.client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Nav2 action server {self.action_name} is not available")

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the goal")

        self.get_logger().info("Goal accepted. Waiting for result.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            raise RuntimeError("Nav2 did not return a result")
        return result.status

    def listen_for_labels(self, labels, topic, timeout_sec):
        self.labels = labels
        self.server_timeout_sec = timeout_sec

        self.get_logger().info(f"Waiting for Nav2 action server {self.action_name}.")
        if not self.client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Nav2 action server {self.action_name} is not available")

        self.create_subscription(String, topic, self.label_callback, 10)
        self.get_logger().info(f"Listening for label names on {topic}.")

    def label_callback(self, msg):
        query = msg.data.strip()
        if not query:
            return
        if self.active_goal_label is not None:
            self.get_logger().warning(
                f"Ignoring {query!r}; already navigating to {self.active_goal_label}."
            )
            return

        try:
            label = find_label(self.labels, query)
            goal = self.build_goal(label)
        except Exception as exc:
            self.get_logger().error(str(exc))
            return

        self.active_goal_label = label_display(label)
        self.get_logger().info(f"Received {query!r}; going to {self.active_goal_label}.")
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(lambda future: self.handle_goal_response(future, label))

    def build_goal(self, label):
        world = label.get("world") or {}
        x = float(world["x"])
        y = float(world["y"])
        yaw = float(label.get("yaw", 0.0))
        qz, qw = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def handle_goal_response(self, future, label):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal")
            self.active_goal_label = None
            return

        self.get_logger().info("Goal accepted. Waiting for result.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self.handle_result(future, label))

    def handle_result(self, future, label):
        result = future.result()
        if result is None:
            self.get_logger().error("Nav2 did not return a result")
            self.active_goal_label = None
            return

        save_last_label(label, result.status)
        self.get_logger().info(
            f"Navigation finished with status {result.status}. Saved {LAST_LABEL_FILE}."
        )
        self.active_goal_label = None


def save_last_label(label, status):
    output = {
        "name": label.get("name"),
        "kind": label.get("kind"),
        "detail": label.get("detail"),
        "world": label.get("world"),
        "status": status,
    }
    LAST_LABEL_FILE.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send Nav2 to a label from maps/lab_map_new_labels.json."
    )
    parser.add_argument("label", nargs="?", help="Label name or alias")
    parser.add_argument("--list", action="store_true", help="List saved labels and exit")
    parser.add_argument(
        "--listen",
        action="store_true",
        help="Subscribe to labels explicitly; this is the default with no label argument",
    )
    parser.add_argument(
        "--topic",
        default="/label",
        help="std_msgs/String topic to listen to",
    )
    parser.add_argument(
        "--labels-file",
        default=str(DEFAULT_LABELS_FILE),
        help="Path to a *_labels.json file",
    )
    parser.add_argument(
        "--action-name",
        default="/navigate_to_pose",
        help="Nav2 NavigateToPose action name",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the Nav2 action server",
    )
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    labels_file = Path(args.labels_file).expanduser()
    labels = load_labels(labels_file)

    if args.list:
        list_labels(labels)
        return 0

    rclpy.init()
    node = LabelNavigator(args.action_name)
    try:
        if args.listen or not args.label:
            node.listen_for_labels(labels, args.topic, args.server_timeout)
            rclpy.spin(node)
        else:
            label = find_label(labels, args.label)
            status = node.go_to(label, args.server_timeout)
            save_last_label(label, status)
            node.get_logger().info(
                f"Navigation finished with status {status}. Saved {LAST_LABEL_FILE}."
            )
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
