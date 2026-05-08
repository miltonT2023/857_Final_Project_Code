import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelMinimumEnforcer(Node):
    def __init__(self):
        super().__init__('cmd_vel_minimum_enforcer')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_enforced')
        self.declare_parameter('min_linear_x', 0.06)
        self.declare_parameter('min_angular_z', 0.20)
        self.declare_parameter('command_epsilon', 0.01)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.min_angular_z = float(self.get_parameter('min_angular_z').value)
        self.command_epsilon = float(self.get_parameter('command_epsilon').value)

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info(
            'Relaying %s -> %s with minimum nonzero speeds: '
            'linear.x=%.3f m/s, angular.z=%.3f rad/s'
            % (input_topic, output_topic, self.min_linear_x, self.min_angular_z)
        )

    def command_callback(self, msg):
        twist = Twist()
        twist.linear.x = self.enforce_minimum(msg.linear.x, self.min_linear_x)
        twist.linear.y = msg.linear.y
        twist.linear.z = msg.linear.z
        twist.angular.x = msg.angular.x
        twist.angular.y = msg.angular.y
        twist.angular.z = self.enforce_minimum(msg.angular.z, self.min_angular_z)
        self.publisher.publish(twist)

    def enforce_minimum(self, value, minimum):
        magnitude = abs(value)
        if magnitude <= self.command_epsilon or magnitude >= minimum:
            return value
        return math.copysign(minimum, value)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMinimumEnforcer()
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
