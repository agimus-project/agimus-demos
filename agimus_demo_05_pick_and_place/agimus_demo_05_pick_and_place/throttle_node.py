import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
# import time

"""
ros2 bag record
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/color/points
/camera/depth/camera_info
/camera2/color/image_raw
/camera2/color/camera_info
/happypose/detections
/joint_states
/mpc_debug
/mpc_input
/control
"""


class ThrottleNode(Node):
    def __init__(self):
        super().__init__("image_throttle_node")
        self.sub1 = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.sub2 = self.create_subscription(
            Image,
            "/camera2/color/image_raw",
            self.callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.sub3 = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.callback3,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.pub1 = self.create_publisher(
            Image,
            "/camera/color/image_raw_throttled",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.pub2 = self.create_publisher(
            Image,
            "/camera2/color/image_raw_throttled",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.pub3 = self.create_publisher(
            PointCloud2,
            "/camera/depth/color/points_throttled",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.last_pub_time1 = self.get_clock().now()
        self.last_pub_time2 = self.get_clock().now()
        self.last_pub_time3 = self.get_clock().now()
        self.interval1 = 1.0 / 10.0  # 10 Hz target
        self.interval2 = 1.0 / 10.0  # 10 Hz target
        self.interval3 = 1.0 / 5.0  # 10 Hz target

    def callback1(self, msg):
        now = self.get_clock().now()
        if (now - self.last_pub_time1).nanoseconds / 1e9 >= self.interval1:
            self.pub1.publish(msg)
            self.last_pub_time1 = now

    def callback2(self, msg):
        now = self.get_clock().now()
        if (now - self.last_pub_time2).nanoseconds / 1e9 >= self.interval2:
            self.pub2.publish(msg)
            self.last_pub_time2 = now

    def callback3(self, msg):
        now = self.get_clock().now()
        if (now - self.last_pub_time3).nanoseconds / 1e9 >= self.interval3:
            self.pub3.publish(msg)
            self.last_pub_time3 = now


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
