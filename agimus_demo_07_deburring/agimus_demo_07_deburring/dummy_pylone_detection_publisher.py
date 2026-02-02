import math
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from rclpy.exceptions import ParameterException
from geometry_msgs.msg import (
    Quaternion,
    Vector3,
    Transform,
    TransformStamped,
)


class DummyPyloneDetectionPublisher(Node):
    def __init__(self):
        super().__init__("dummy_pylone_detection_publisher")

        self.declare_parameter("parent_frame_id", "fer_link0")
        self.declare_parameter("child_frame_id", "pylone_link")
        self.declare_parameter("xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("xyzw", [1.0, 0.0, 0.0, 0.0])
        # Execute a dummy linear sine along Y xis with rotation around Z
        # axis to see if the pose is being updated correctly in the system
        self.declare_parameter("test_sine_motion", False)

        self._parent_frame_id = (
            self.get_parameter("parent_frame_id").get_parameter_value().string_value
        )
        self._child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self._pose = self.get_parameter("xyz").get_parameter_value().double_array_value
        self._quat = pin.Quaternion(
            np.array(
                self.get_parameter("xyzw").get_parameter_value().double_array_value
            )
        )
        self._pose_y_init = self._pose[1]

        self._test_sine_motion = (
            self.get_parameter("test_sine_motion").get_parameter_value().bool_value
        )

        self._transform = TransformStamped()
        self._transform.header.stamp = self.get_clock().now().to_msg()
        self._transform.header.frame_id = self._parent_frame_id
        self._transform.child_frame_id = self._child_frame_id
        self._transform.transform = Transform(
            translation=Vector3(**dict(zip("xyz", self._pose))),
            rotation=Quaternion(**dict(zip("xyzw", self._quat.coeffs()))),
        )

        self._dummy_motion_cnt = 0
        self._publish_frequency = 0.25

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_publisher_timer = self.create_timer(
            self._publish_frequency, self._tf_publisher_timer_cb
        )

        self.get_logger().info("Node started.")

    def _tf_publisher_timer_cb(self) -> None:
        self._transform.header.stamp = self.get_clock().now().to_msg()

        if self._test_sine_motion:
            # Move it by 4 cm every 2 seconds
            self._transform.transform.translation.y = (
                self._pose_y_init
                + math.sin(self._dummy_motion_cnt * self._publish_frequency) * 0.02
            )
            self._transform.transform.translation.y = (
                self._pose_y_init
                + math.sin(self._dummy_motion_cnt * self._publish_frequency) * 0.02
            )
            q = self._quat * pin.Quaternion(
                pin.rpy.rpyToMatrix(
                    np.array(
                        [
                            0.0,
                            0.0,
                            math.sin(self._dummy_motion_cnt * self._publish_frequency)
                            * np.pi
                            / 32,
                        ]
                    )
                )
            )
            self._transform.transform.rotation = Quaternion(
                **dict(zip("xyzw", q.coeffs()))
            )
            self._dummy_motion_cnt += 1

        self._tf_broadcaster.sendTransform([self._transform])


def main(args=None):
    rclpy.init(args=args)
    try:
        dummy_pylone_detection_publisher_node = DummyPyloneDetectionPublisher()
        rclpy.spin(dummy_pylone_detection_publisher_node)
        dummy_pylone_detection_publisher_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
