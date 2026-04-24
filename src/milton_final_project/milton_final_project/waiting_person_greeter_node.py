import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class WaitingPersonGreeterNode(Node):
    def __init__(self):
        super().__init__('waiting_person_greeter_node')

        self.declare_parameter('motion_topic', '/lidar/motion_target')
        self.declare_parameter('person_target_topic', '/yolo/person_target')
        self.declare_parameter('state_topic', '/robot/light_state')
        self.declare_parameter('user_input_topic', '/wayfinding/user_input')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('expression_topic', '/face/expression')
        self.declare_parameter('message_topic', '/face/message')
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('motion_timeout_sec', 0.8)
        self.declare_parameter('person_timeout_sec', 0.8)
        self.declare_parameter('idle_search_delay_sec', 90.0)
        self.declare_parameter('stable_detection_sec', 0.2)
        self.declare_parameter('align_tolerance_deg', 6.0)
        self.declare_parameter('stop_distance_ft', 0.5)
        self.declare_parameter('angular_gain', 1.4)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.18)
        self.declare_parameter('greeting_cooldown_sec', 12.0)
        self.declare_parameter(
            'greeting_message',
            'Hello there. I can help you find a room or person.',
        )
        self.declare_parameter('greeting_expression', 'happy')
        self.declare_parameter('require_yolo_for_turn', True)

        motion_topic = self.get_parameter('motion_topic').value
        person_target_topic = self.get_parameter('person_target_topic').value
        state_topic = self.get_parameter('state_topic').value
        user_input_topic = self.get_parameter('user_input_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        expression_topic = self.get_parameter('expression_topic').value
        message_topic = self.get_parameter('message_topic').value

        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.motion_timeout_sec = float(self.get_parameter('motion_timeout_sec').value)
        self.person_timeout_sec = float(self.get_parameter('person_timeout_sec').value)
        self.idle_search_delay_sec = float(
            self.get_parameter('idle_search_delay_sec').value
        )
        self.stable_detection_sec = float(
            self.get_parameter('stable_detection_sec').value
        )
        self.align_tolerance_deg = float(
            self.get_parameter('align_tolerance_deg').value
        )
        self.stop_distance_m = (
            float(self.get_parameter('stop_distance_ft').value) * 0.3048
        )
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.min_angular_speed = float(
            self.get_parameter('min_angular_speed').value
        )
        self.greeting_cooldown_sec = float(
            self.get_parameter('greeting_cooldown_sec').value
        )
        self.greeting_message = self.get_parameter('greeting_message').value
        self.greeting_expression = self.get_parameter('greeting_expression').value
        self.require_yolo_for_turn = bool(
            self.get_parameter('require_yolo_for_turn').value
        )

        self.current_state = 'waiting'
        self.latest_motion_target = None
        self.latest_motion_time = None
        self.latest_person_target = None
        self.latest_person_time = None
        self.person_seen_since = None
        self.last_greet_time = None
        self.last_cmd_was_nonzero = False
        self.last_user_input_time = self.get_clock().now()

        self.motion_sub = self.create_subscription(
            String,
            motion_topic,
            self.motion_callback,
            10,
        )
        self.person_target_sub = self.create_subscription(
            String,
            person_target_topic,
            self.person_target_callback,
            10,
        )
        self.state_sub = self.create_subscription(
            String,
            state_topic,
            self.state_callback,
            10,
        )
        self.user_input_sub = self.create_subscription(
            String,
            user_input_topic,
            self.user_input_callback,
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.expression_pub = self.create_publisher(String, expression_topic, 10)
        self.message_pub = self.create_publisher(String, message_topic, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Subscribed to motion topic: {motion_topic}')
        self.get_logger().info(f'Subscribed to person target topic: {person_target_topic}')
        self.get_logger().info(f'Subscribed to state topic: {state_topic}')
        self.get_logger().info(f'Subscribed to user input topic: {user_input_topic}')
        self.get_logger().info(f'Publishing turn commands to: {cmd_vel_topic}')

    def state_callback(self, msg: String):
        self.current_state = msg.data.strip() or 'waiting'
        if self.current_state != 'waiting':
            self.person_seen_since = None
            self.latest_motion_target = None
            self.latest_motion_time = None
            self.latest_person_target = None
            self.latest_person_time = None
            self.stop_robot()

    def user_input_callback(self, msg: String):
        if msg.data.strip():
            self.last_user_input_time = self.get_clock().now()
            self.person_seen_since = None
            self.stop_robot()

    def motion_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring invalid motion payload.')
            return

        self.latest_motion_target = payload
        self.latest_motion_time = self.get_clock().now()

    def person_target_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring invalid person target payload.')
            return

        self.latest_person_target = payload
        self.latest_person_time = self.get_clock().now()
        if payload.get('seen', False):
            if self.person_seen_since is None:
                self.person_seen_since = self.latest_person_time
        else:
            self.person_seen_since = None

    def payload_is_fresh(self, payload, payload_time, timeout_sec: float) -> bool:
        if payload is None or payload_time is None:
            return False
        age_sec = (self.get_clock().now() - payload_time).nanoseconds / 1e9
        return age_sec <= timeout_sec

    def greeting_ready(self) -> bool:
        if self.last_greet_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_greet_time).nanoseconds / 1e9
        return elapsed >= self.greeting_cooldown_sec

    def stop_robot(self):
        if not self.last_cmd_was_nonzero:
            return

        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.last_cmd_was_nonzero = False

    def idle_search_ready(self) -> bool:
        if self.current_state != 'waiting':
            return False

        idle_sec = (
            self.get_clock().now() - self.last_user_input_time
        ).nanoseconds / 1e9
        return idle_sec >= self.idle_search_delay_sec

    def publish_command(self, linear_x: float = 0.0, angular_z: float = 0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.last_cmd_was_nonzero = (
            abs(linear_x) > 1e-6 or abs(angular_z) > 1e-6
        )

    def publish_turn(self, angular_z: float):
        # Camera image x-offset and base rotation use opposite sign conventions.
        self.publish_command(angular_z=-angular_z)

    def publish_greeting(self):
        expression = String()
        expression.data = self.greeting_expression
        self.expression_pub.publish(expression)

        message = String()
        message.data = self.greeting_message
        self.message_pub.publish(message)
        self.last_greet_time = self.get_clock().now()

    def current_motion_target(self):
        if not self.payload_is_fresh(
            self.latest_motion_target,
            self.latest_motion_time,
            self.motion_timeout_sec,
        ):
            return None
        if not self.latest_motion_target.get('seen', False):
            return None
        return self.latest_motion_target

    def current_person_target(self):
        if not self.payload_is_fresh(
            self.latest_person_target,
            self.latest_person_time,
            self.person_timeout_sec,
        ):
            return None
        if not self.latest_person_target.get('seen', False):
            return None
        return self.latest_person_target

    def control_loop(self):
        if self.current_state != 'waiting':
            self.stop_robot()
            return

        if not self.idle_search_ready():
            self.person_seen_since = None
            self.stop_robot()
            return

        person_target = self.current_person_target()
        if person_target is not None:
            if self.person_seen_since is None:
                self.person_seen_since = self.get_clock().now()

            stable_sec = (
                self.get_clock().now() - self.person_seen_since
            ).nanoseconds / 1e9
            angle_deg = float(person_target.get('angle_deg', 0.0))
            distance_m = person_target.get('distance_m')
            angle_rad = math.radians(angle_deg)

            if distance_m is not None and float(distance_m) <= self.stop_distance_m:
                self.stop_robot()
                return

            if stable_sec < self.stable_detection_sec:
                self.stop_robot()
                return

            if abs(angle_deg) <= self.align_tolerance_deg:
                self.stop_robot()
                if self.greeting_ready():
                    self.publish_greeting()
                return

            turn_speed = max(
                self.min_angular_speed,
                min(self.max_angular_speed, abs(angle_rad) * self.angular_gain),
            )
            self.publish_turn(math.copysign(turn_speed, angle_rad))
            return

        self.person_seen_since = None

        if self.require_yolo_for_turn:
            self.stop_robot()
            return

        motion_target = self.current_motion_target()
        if motion_target is None:
            self.stop_robot()
            return

        angle_rad = float(motion_target.get('angle_rad', 0.0))
        if abs(angle_rad) < math.radians(self.align_tolerance_deg):
            self.stop_robot()
            return

        turn_speed = max(
            self.min_angular_speed,
            min(self.max_angular_speed, abs(angle_rad) * self.angular_gain),
        )
        self.publish_turn(math.copysign(turn_speed, angle_rad))


def main(args=None):
    rclpy.init(args=args)
    node = WaitingPersonGreeterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
