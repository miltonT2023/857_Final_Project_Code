from threading import Thread
import time

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
        self.response_duration_sec = 10.0
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

            if self.interpreter.is_conversation_end(destination):
                ending = self.interpreter.ending_response(destination)
                self.publish_expression(ending['expression'])
                self.publish_message(ending['message'])
                time.sleep(2.0)
                self.publish_expression('confused')
                self.publish_message(
                    "Hi, I'm the navigation robot that helps you find a location or room."
                )
                continue

            target = self.interpreter.extract_target(destination)
            match = self.directory.find_best_match(target or destination)
            self.publish_expression(self.directory.expression_for_match(match))
            self.publish_message(self.directory.build_response(match))

            if match.entry is None:
                continue

            destination_label = (
                match.entry.location if match.entry.kind == 'person' else match.entry.title
            )
            self.publish_message(
                f'{self.directory.build_response(match)} Do you need help getting to {destination_label}?'
            )

            try:
                answer = input('Do you need help getting there? (yes/no) ').strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                rclpy.shutdown()
                break

            if answer in {'yes', 'y', 'yeah', 'yep', 'sure', 'ok', 'okay'}:
                self.publish_expression('ready_to_go')
                self.publish_message(
                    f"Let's go. Going to navigation mode for {destination_label}."
                )
                time.sleep(self.response_duration_sec)
                self.publish_expression('confused')
                self.publish_message(
                    "Hi, I'm the navigation robot that helps you find a location or room."
                )
                continue

            self.publish_expression('happy')
            self.publish_message(
                'Okay. If you need anything else, ask me about another room or person.'
            )
            time.sleep(2.0)
            self.publish_expression('confused')
            self.publish_message(
                "Hi, I'm the navigation robot that helps you find a location or room."
            )

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
