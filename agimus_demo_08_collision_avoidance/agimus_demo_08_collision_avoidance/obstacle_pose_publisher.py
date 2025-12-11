import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from rclpy.exceptions import ParameterException
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Vector3,
    Pose,
    Transform,
    TransformStamped,
)


class ObstaclePosePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_pose_publisher")

        self._obstacle_pose_msg = Pose(
            position=Point(x=0.5, y=0.0, z=0.2),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )
        self.declare_parameter("arm_id", "fer")

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._obstacle_pose_pub = self.create_publisher(Pose, "obstacle_capsule_0", 10)

        self._pose_publisher_timer = self.create_timer(
            0.1, self._pose_publisher_timer_cb
        )
        self.arm_id = self.get_parameter("arm_id").get_parameter_value().string_value
        self.get_logger().info("Node started.")

    def _pose_publisher_timer_cb(self) -> None:
        self._obstacle_pose_pub.publish(self._obstacle_pose_msg)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.arm_id + "_link0"
        transform.child_frame_id = "obstacle"
        transform.transform = Transform(
            translation=Vector3(
                x=self._obstacle_pose_msg.position.x,
                y=self._obstacle_pose_msg.position.y,
                z=self._obstacle_pose_msg.position.z,
            ),
            rotation=self._obstacle_pose_msg.orientation,
        )
        self._tf_broadcaster.sendTransform([transform])


def main(args=None):
    rclpy.init(args=args)
    try:
        obstacle_pose_publisher_node = ObstaclePosePublisher()
        rclpy.spin(obstacle_pose_publisher_node)
        obstacle_pose_publisher_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
