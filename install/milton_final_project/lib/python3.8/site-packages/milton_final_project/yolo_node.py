import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('detection_model', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('confidence', 0.25)

        model_path = self.get_parameter('detection_model').value
        image_topic = self.get_parameter('image_topic').value
        self.confidence = self.get_parameter('confidence').value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded. Node is ready.')

        self.bridge = CvBridge()
        self.annotated_image_pub = self.create_publisher(
            Image,
            'yolo/annotated_image',
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Subscribed to camera topic: {image_topic}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=self.confidence, verbose=False)

        boxes = results[0].boxes
        detection_count = 0 if boxes is None else len(boxes)
        self.get_logger().info(
            f'Detected {detection_count} object(s)',
            throttle_duration_sec=1.0,
        )

        annotated_frame = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_image_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
