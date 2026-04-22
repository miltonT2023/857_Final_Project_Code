import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('detection_model', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter('confidence', 0.25)

        requested_model_path = self.get_parameter('detection_model').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.confidence = self.get_parameter('confidence').value
        model_path = self.resolve_model_path(requested_model_path)

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.person_class_id = self.find_person_class_id()
        self.get_logger().info('YOLO model loaded. Node is ready.')

        self.bridge = CvBridge()
        self.latest_depth_frame = None
        self.latest_depth_encoding = None
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
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Subscribed to camera topic: {image_topic}')
        self.get_logger().info(f'Subscribed to depth topic: {depth_topic}')

    def find_person_class_id(self):
        for class_id, class_name in self.model.names.items():
            if str(class_name).lower() == 'person':
                return int(class_id)
        raise ValueError('The loaded YOLO model does not include a person class.')

    def resolve_model_path(self, requested_model_path: str) -> str:
        resolved_path = self.find_model_path(requested_model_path)
        if resolved_path is not None:
            return str(resolved_path)

        fallback_model = 'yolov8n.pt'
        fallback_path = self.find_model_path(fallback_model)
        if fallback_path is not None:
            self.get_logger().warning(
                f'Model {requested_model_path!r} was not found. Falling back to {fallback_path}.'
            )
            return str(fallback_path)

        raise FileNotFoundError(
            f'Could not find requested model {requested_model_path!r} or fallback {fallback_model!r}.'
        )

    def find_model_path(self, model_path: str):
        candidate = Path(model_path).expanduser()
        if candidate.is_file():
            return candidate

        search_roots = [Path.cwd(), *Path(__file__).resolve().parents]
        for root in search_roots:
            candidate = root / model_path
            if candidate.is_file():
                return candidate

        return None

    def depth_callback(self, msg):
        self.latest_depth_encoding = msg.encoding
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough',
        )

    def depth_value_to_meters(self, depth_value):
        if self.latest_depth_encoding == '32FC1':
            return float(depth_value)
        return float(depth_value) / 1000.0

    def estimate_distance_meters(self, xyxy, frame_shape):
        if self.latest_depth_frame is None:
            return None

        depth_frame = self.latest_depth_frame
        if depth_frame.shape[:2] != frame_shape[:2]:
            return None

        x1, y1, x2, y2 = [int(value) for value in xyxy]
        x1 = max(0, min(frame_shape[1] - 1, x1))
        x2 = max(0, min(frame_shape[1] - 1, x2))
        y1 = max(0, min(frame_shape[0] - 1, y1))
        y2 = max(0, min(frame_shape[0] - 1, y2))

        if x2 <= x1 or y2 <= y1:
            return None

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        sample_radius = max(2, min((x2 - x1) // 8, (y2 - y1) // 8, 6))

        depth_patch = depth_frame[
            max(0, center_y - sample_radius):min(frame_shape[0], center_y + sample_radius + 1),
            max(0, center_x - sample_radius):min(frame_shape[1], center_x + sample_radius + 1),
        ]
        if depth_patch.size == 0:
            return None

        valid_depths = depth_patch[np.isfinite(depth_patch)]
        valid_depths = valid_depths[valid_depths > 0]
        if valid_depths.size == 0:
            return None

        distance_meters = self.depth_value_to_meters(np.median(valid_depths))
        if distance_meters <= 0:
            return None
        return distance_meters

    def annotate_person_detections(self, frame, results):
        boxes = results[0].boxes
        if boxes is None:
            return frame

        for box in boxes:
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            distance_meters = self.estimate_distance_meters(box.xyxy[0].tolist(), frame.shape)
            label = 'Person'
            if distance_meters is not None:
                label = f'Person | {distance_meters:.2f} m away'

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (102, 255, 178),
                2,
            )

            text_origin = (max(8, x1), max(24, y1 - 10))
            cv2.putText(
                frame,
                label,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                label,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (102, 255, 178),
                2,
                cv2.LINE_AA,
            )
        return frame

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(
            frame,
            conf=self.confidence,
            classes=[self.person_class_id],
            verbose=False,
        )

        boxes = results[0].boxes
        detection_count = 0 if boxes is None else len(boxes)
        self.get_logger().info(
            f'Detected {detection_count} person(s)',
            throttle_duration_sec=1.0,
        )

        annotated_frame = frame.copy()
        annotated_frame = self.annotate_person_detections(annotated_frame, results)
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
