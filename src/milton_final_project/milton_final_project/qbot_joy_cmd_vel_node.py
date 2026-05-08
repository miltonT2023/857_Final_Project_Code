from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class QBotJoyCmdVelNode(Node):
    def __init__(self):
        super().__init__('qbot_joy_cmd_vel_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('reverse_button', 0)
        self.declare_parameter('steering_axis', 0)
        self.declare_parameter('throttle_axis', 5)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 0.50)

        self.enable_button = int(self.get_parameter('enable_button').value)
        self.reverse_button = int(self.get_parameter('reverse_button').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.throttle_axis = int(self.get_parameter('throttle_axis').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )

        self.publisher = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.create_subscription(
            Joy,
            self.get_parameter('joy_topic').value,
            self.joy_callback,
            10,
        )

        self.get_logger().info(
            'Publishing joystick velocity commands. Hold LB to enable; '
            'right trigger drives forward; A reverses.'
        )

    def button_pressed(self, msg, index):
        return index < len(msg.buttons) and msg.buttons[index] == 1

    def axis_value(self, msg, index, default=0.0):
        if index >= len(msg.axes):
            return default
        return float(msg.axes[index])

    def joy_callback(self, msg):
        twist = Twist()
        if not self.button_pressed(msg, self.enable_button):
            self.publisher.publish(twist)
            return

        steering = -self.axis_value(msg, self.steering_axis)
        trigger = self.axis_value(msg, self.throttle_axis, default=1.0)
        throttle = max(0.0, min(1.0, (1.0 - trigger) * 0.5))

        if self.button_pressed(msg, self.reverse_button):
            throttle = -throttle

        twist.linear.x = self.max_linear_speed * throttle
        twist.angular.z = self.max_angular_speed * steering
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = QBotJoyCmdVelNode()
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
