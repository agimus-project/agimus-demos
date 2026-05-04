import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class CameraTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("camera_target_publisher")

        self.declare_parameter("parent_frame", "pylone_link")
        self.declare_parameter("target_frame", "camera_target")
        self.declare_parameter("publish_rate", 10.0)

        self._parent_frame: str = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self._target_frame: str = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self._publish_rate: float = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self._translation = (-1.082, 0.299, -0.8)
        self._rotation = (1.0, 0.0, 0.0, 0.0)

        self._broadcaster = TransformBroadcaster(self)

        period = 1.0 / max(self._publish_rate, 1.0)
        self._timer = self.create_timer(period, self._timer_cb)
        self.get_logger().info(
            f"Publishing hard-coded target '{self._target_frame}' in '{self._parent_frame}'"
        )

    def _timer_cb(self) -> None:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._parent_frame
        msg.child_frame_id = self._target_frame
        msg.transform.translation.x = float(self._translation[0])
        msg.transform.translation.y = float(self._translation[1])
        msg.transform.translation.z = float(self._translation[2])
        msg.transform.rotation.x = float(self._rotation[0])
        msg.transform.rotation.y = float(self._rotation[1])
        msg.transform.rotation.z = float(self._rotation[2])
        msg.transform.rotation.w = float(self._rotation[3])
        self._broadcaster.sendTransform(msg)


def main() -> None:
    rclpy.init()
    node = CameraTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
