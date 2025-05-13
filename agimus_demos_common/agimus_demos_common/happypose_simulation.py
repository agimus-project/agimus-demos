import rclpy
import rclpy.parameter
import rclpy.time
import builtin_interfaces
from rclpy.qos import qos_profile_system_default
from rclpy.executors import ExternalShutdownException
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import geometry_msgs.msg

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import pinocchio
import eigenpy
import numpy as np

from agimus_controller_ros.ros_utils import transform_to_se3


def se3_to_pose(M: pinocchio.SE3) -> geometry_msgs.msg.Pose:
    t = geometry_msgs.msg.Pose()
    t.position.x = M.translation[0]
    t.position.y = M.translation[1]
    t.position.z = M.translation[2]

    q = eigenpy.Quaternion(M.rotation)
    t.orientation.w = q.w
    t.orientation.x = q.x
    t.orientation.y = q.y
    t.orientation.z = q.z
    return t


class HappyposeSimulation(rclpy.node.Node):
    PARAMETERS = (
        ("camera_name", "camera", "Name of camera in TF"),
        ("base_name", "base", "Name of the frame wrt which the object pose is expressed."),
        ("object_pose_in_base_txyz", [0.0, 0.0, 0.0], "Translation of the object"),
        ("object_pose_in_base_qxyzw", [0.0, 0.0, 0.0, 1.0], "Orientation of the object as a quaternion"),
        ("object_id", "tless-obj_000021", "Object ID as published by Happypose."),
        ("happypose_time", 0.2, "Computation time of happypose. Used both as publication delay and rate"),
    )

    def __init__(self):
        super().__init__("happypose_simulation")

        for name, value, _ in self.PARAMETERS:
            self.declare_parameter(name, value)

        self._bMc_tf = None

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            "/happypose/detections",
            qos_profile_system_default,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._timer = self.create_timer(
            self.get_parameter("happypose_time").value, self.publish
        )

    @property
    def _camera_name(self) -> str:
        return self.get_parameter("camera_name").value

    @property
    def _base_name(self) -> str:
        return self.get_parameter("base_name").value

    @property
    def _object_id(self) -> str:
        return self.get_parameter("object_id").value

    @property
    def _bMo(self) -> pinocchio.SE3:
        xyz = self.get_parameter("object_pose_in_base_txyz").value
        q = self.get_parameter("object_pose_in_base_qxyzw").value
        return pinocchio.XYZQUATToSE3(np.array(xyz + q))

    def _publish_detection(
        self, cMo: pinocchio.SE3, stamp: builtin_interfaces.msg.Time
    ):
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self._object_id
        hypothesis.pose.pose = se3_to_pose(cMo)

        detection = Detection2D(results=[hypothesis])
        detection.header.stamp = stamp
        detection.header.frame_id = self._camera_name
        detections = Detection2DArray(detections=[detection])

        self.detection_pub.publish(detections)

    def publish(self):
        # at T + DT, publish cMo(T) = cMb(T) * bMo with the expected time stamp.
        if self._bMc_tf is not None:
            bMc = transform_to_se3(self._bMc_tf.transform)
            self._publish_detection(
                bMc.inverse() * self._bMo, self._bMc_tf.header.stamp
            )

        # Get the pose of the camera in base frame at T: bMc(T).
        try:
            self._bMc_tf = self.tf_buffer.lookup_transform(
                self._base_name, self._camera_name, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"{ex}", throttle_duration_sec=5.0)


def main():
    import sys

    help = "--help" in sys.argv or "-h" in sys.argv
    try:
        rclpy.init()
        node = HappyposeSimulation()
        if help:
            print("This node can be configured with the following ROS node parameters:")
            for name, value, doc in HappyposeSimulation.PARAMETERS:
                print(f"- {name} [{value}]\n  {doc}")
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
