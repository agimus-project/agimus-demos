import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException

MOCAP_WORLD_FRAME_NAME = "world_mocap"
MOCAP_BASE_FRAME_NAME = "tiago_base/base"
MOCAP_BAR_FRAME_NAME = "cube/base"
WORLD_FRAME_NAME = "world"
BAR_FRAME_NAME = "bar_base_link"


class MocapWorldPub(Node):
    """Repositions the origin of the mocap to the first position of
    the base of the robot to match the origin of the path planning
    """

    def __init__(self):
        super().__init__("tf_mocap_world_node")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self._timer_listener = self.create_timer(0.01, self._cb_listener)

        self._got_init_world_mocap_2_base_mocap = False

    def _cb_listener(self):
        if not self._got_init_world_mocap_2_base_mocap:
            try:
                t = self._tf_buffer.lookup_transform(
                    MOCAP_BASE_FRAME_NAME, MOCAP_WORLD_FRAME_NAME, rclpy.time.Time()
                )

                t.header.frame_id = WORLD_FRAME_NAME  # parent frame
                t.child_frame_id = MOCAP_WORLD_FRAME_NAME  # child frame

                self.get_logger().info(f"transform {t}")
                self.tf_static_broadcaster.sendTransform(t)
                self._got_init_world_mocap_2_base_mocap = True

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {MOCAP_WORLD_FRAME_NAME} to {MOCAP_BASE_FRAME_NAME}: {ex}"
                )
                self._got_init_world2base_mocap = False

                return


def main(args=None):
    rclpy.init(args=args)
    node = MocapWorldPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
