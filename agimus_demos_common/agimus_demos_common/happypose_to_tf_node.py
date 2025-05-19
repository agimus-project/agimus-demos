import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import Detection2DArray
from agimus_controller_ros.ros_utils import (
    transform_to_se3,
    se3_to_transform,
    pose_to_se3,
)


def map_object_id(obj_id, dataset="tless"):
    num_part = obj_id.split("_")[1]
    return f"{dataset}-obj_{int(num_part):06d}"


class HappyposeToTf(Node):
    """Main class implementing ROS node redirecting /robot_description topic
    to parameter of a node."""

    def __init__(self):
        super().__init__("happypose_to_tf_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.vision_client = self.create_subscription(
            Detection2DArray,
            "/happypose/detections",
            self.vision_callback,
            qos_profile=qos_profile_system_default,
        )
        self.declare_parameter("base_name", "support_link")

    @property
    def _base_frame(self) -> str:
        return self.get_parameter("base_name").value

    def vision_callback(self, vision_msg: Detection2DArray) -> None:
        if vision_msg.detections == []:
            return
        image_stamp = vision_msg.detections[0].header.stamp
        camera_name = vision_msg.detections[0].header.frame_id
        base_frame = self._base_frame
        try:
            wMc = transform_to_se3(
                self.tf_buffer.lookup_transform(
                    base_frame, camera_name, image_stamp
                ).transform
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not get camera pose: {ex}")
            return
        ts = list()
        for pose_detection in vision_msg.detections:
            cMo = pose_to_se3(pose_detection.results[0].pose.pose)
            wMo = wMc * cMo
            t = TransformStamped()
            object_name = pose_detection.results[0].hypothesis.class_id

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = image_stamp
            t.header.frame_id = base_frame
            t.child_frame_id = object_name
            t.transform = se3_to_transform(wMo)
            ts.append(t)

        # Send the transformations
        self.tf_broadcaster.sendTransform(ts)


def main(args=None) -> int:
    rclpy.init(args=args)
    happypose_to_tf = HappyposeToTf()
    try:
        rclpy.spin(happypose_to_tf)
    except KeyboardInterrupt:
        pass
    happypose_to_tf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
