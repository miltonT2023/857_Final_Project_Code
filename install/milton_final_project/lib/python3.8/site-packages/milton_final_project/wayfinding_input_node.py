from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .robot_interpreter import RobotInterpreter
from .seic_directory import SeicDirectory


class WayfindingInputNode(Node):
    def __init__(self):
        super().__init__('wayfinding_input_node')

        self.expression_pub = self.create_publisher(String, '/face/expression', 10)
        self.message_pub = self.create_publisher(String, '/face/message', 10)
        self.running = True
        self.interpreter = RobotInterpreter()
        self.directory = SeicDirectory()

        self.publish_message("Hi, I'm the navigation robot that helps you find a location or room.")
        self.publish_expression('confused')

        self.input_thread = Thread(target=self.read_loop, daemon=True)
        self.input_thread.start()
        self.get_logger().info('Wayfinding input ready. Type a destination in this terminal.')

    def publish_expression(self, expression: str):
        msg = String()
        msg.data = expression
        self.expression_pub.publish(msg)

    def publish_message(self, text: str):
        msg = String()
        msg.data = text
        self.message_pub.publish(msg)

    def read_loop(self):
        while rclpy.ok() and self.running:
            try:
                destination = input('Where would you like to go? ').strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                rclpy.shutdown()
                break

            if not destination:
                self.publish_expression('confused')
                self.publish_message("Hi, I'm the navigation robot that helps you find a location or room.")
                continue

            target = self.interpreter.extract_target(destination)
            match = self.directory.find_best_match(target or destination)
            self.publish_expression(self.directory.expression_for_match(match))
            self.publish_message(self.directory.build_response(match))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WayfindingInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
