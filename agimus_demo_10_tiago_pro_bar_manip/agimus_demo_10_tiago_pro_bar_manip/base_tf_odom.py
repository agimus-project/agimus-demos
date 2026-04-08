import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class TfBasePub(Node):
    def __init__(self):
        super().__init__("tf_base_publisher")

        # Parameters
        self.declare_parameter("odom_topic", "/mobile_base_controller/odom")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_footprint")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos,
        )

        self.get_logger().info(
            f"TfBasePub ready — republishing '{self.odom_topic}' "
            f"as TF: '{self.parent_frame}' → '{self.child_frame}'"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Receives an Odometry message and broadcasts it as a TF transform."""

        t = TransformStamped()

        # Use the stamp from the incoming message for time consistency
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation (quaternion passthrough)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBasePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
