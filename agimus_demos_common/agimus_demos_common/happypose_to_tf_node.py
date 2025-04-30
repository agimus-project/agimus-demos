import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import Detection2DArray


def map_object_id(obj_id, dataset="tless"):
    num_part = obj_id.split("_")[1]
    return f"{dataset}-obj_{int(num_part):06d}"


class HappyposeToTf(Node):
    """Main class implementing ROS node redirecting /robot_description topic
    to parameter of a node."""

    def __init__(self):
        super().__init__("wait_for_non_zero_joints_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.vision_client = self.create_subscription(
            Detection2DArray,
            "/happypose/detections",
            self.vision_callback,
            qos_profile=qos_profile_system_default,
        )
        self.base_frame = "support_link"
        self.get_logger().info(
            "Node initialized, waiting for '/joint_states' to be published..."
        )

    def vision_callback(self, vision_msg: Detection2DArray) -> None:
        if vision_msg.detections == []:
            return
        for pose_detection in vision_msg.detections.results:
            t = TransformStamped()
            object_name = f"obj_{int(pose_detection.id):06d}"
            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = object_name
            t.transform.translation.x = self.start_obj_pose[0]
            t.transform.translation.y = self.start_obj_pose[1]
            t.transform.translation.z = self.start_obj_pose[2]
            t.transform.rotation.x = self.start_obj_pose[3]
            t.transform.rotation.y = self.start_obj_pose[4]
            t.transform.rotation.z = self.start_obj_pose[5]
            t.transform.rotation.w = self.start_obj_pose[6]

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)


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
