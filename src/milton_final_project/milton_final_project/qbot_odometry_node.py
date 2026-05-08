import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class QBotOdometryNode(Node):
    def __init__(self):
        super().__init__('qbot_odometry_node')

        self.declare_parameter('speed_topic', '/qbot_speed_feedback')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('publish_tf', True)

        self.speed_topic = self.get_parameter('speed_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_stamp = None
        self.lock = threading.Lock()

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            TwistStamped,
            self.speed_topic,
            self.speed_feedback_callback,
            10,
        )

        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / publish_rate_hz, self.publish_odometry)

        self.get_logger().info(
            'Publishing odom from %s as %s -> %s'
            % (self.speed_topic, self.odom_frame, self.base_frame)
        )

    def speed_feedback_callback(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        with self.lock:
            if self.last_stamp is not None:
                dt = (stamp - self.last_stamp).nanoseconds / 1_000_000_000.0
                if 0.0 < dt < 1.0:
                    delta_distance = msg.twist.linear.x * dt
                    delta_yaw = msg.twist.angular.z * dt
                    mid_yaw = self.yaw + delta_yaw * 0.5
                    self.x += delta_distance * math.cos(mid_yaw)
                    self.y += delta_distance * math.sin(mid_yaw)
                    self.yaw = math.atan2(
                        math.sin(self.yaw + delta_yaw),
                        math.cos(self.yaw + delta_yaw),
                    )

            self.linear_velocity = msg.twist.linear.x
            self.angular_velocity = msg.twist.angular.z
            self.last_stamp = stamp

    def publish_odometry(self):
        with self.lock:
            if self.last_stamp is None:
                return

            x = self.x
            y = self.y
            yaw = self.yaw
            linear_velocity = self.linear_velocity
            angular_velocity = self.angular_velocity
            stamp = self.last_stamp.to_msg()

        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.08
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        odom.twist.covariance[0] = 0.03
        odom.twist.covariance[35] = 0.04
        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = QBotOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
