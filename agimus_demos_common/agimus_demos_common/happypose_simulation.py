import rclpy
import rclpy.time
import builtin_interfaces
from rclpy.qos import qos_profile_system_default
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import pinocchio
import numpy as np
import typing as T

from agimus_controller_ros.ros_utils import (
    transform_msg_to_se3,
    se3_to_pose_msg,
)


def set_transform(
    node: rclpy.node.Node,
    object_id: str,
    camera_name: str,
    base_name: str,
    object_pose: T.Union[
        pinocchio.SE3, tuple[float, float, float, float, float, float, float]
    ],
) -> bool:
    """
    Set the transform published by the Happypose simulation node. This is the client-side function.

    Args:
    - node: a ROS node used for ROS communication.
    - object_pose: desired pose of the object with respect to base_name
    - camera_name: camera frame name
    - object_id: name of the object as published by happypose. E.g. tless-obj_000021

    Return:
    - a boolean that is true if everything in the operation was successful.
    """
    if isinstance(object_pose, pinocchio.SE3):
        object_pose = pinocchio.SE3ToXYZQUAT(object_pose).tolist()

    assert isinstance(object_pose, (list, tuple)) and len(object_pose) == 7

    set_param = node.create_client(
        SetParameters, "/happypose_simulation/set_parameters"
    )
    if not set_param.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn(f"Service {set_param.srv_name} not available.")
        return False

    request = SetParameters.Request(
        parameters=[
            Parameter(
                name="camera_name",
                type_=Parameter.Type.STRING,
                value=camera_name,
            ).to_parameter_msg(),
            Parameter(
                name="base_name",
                type_=Parameter.Type.STRING,
                value=base_name,
            ).to_parameter_msg(),
            Parameter(
                name="object_pose_in_base_txyz",
                type_=Parameter.Type.DOUBLE_ARRAY,
                value=object_pose[:3],
            ).to_parameter_msg(),
            Parameter(
                name="object_pose_in_base_qxyzw",
                type_=Parameter.Type.DOUBLE_ARRAY,
                value=object_pose[3:],
            ).to_parameter_msg(),
            Parameter(
                name="object_id",
                type_=Parameter.Type.STRING,
                value=object_id,
            ).to_parameter_msg(),
        ]
    )
    future = set_param.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
    result: SetParameters.Response = future.result()
    ret_val = True
    for req, resp in zip(request.parameters, result.results):
        if not resp.successful:
            node.get_logger().warn(f"Failed to set param {req.name}: {resp.reason}")
            ret_val = False
    return ret_val


class HappyposeSimulation(rclpy.node.Node):
    """
    This node simulates happypose detection topic.

    It publishes a single detection result of the object defined as param at a pose
    in the camera frame such that it corresponds to the frame passed as param in base frame.
    """

    PARAMETERS = (
        ("camera_name", "camera", "Name of camera in TF"),
        (
            "base_name",
            "base",
            "Name of the frame wrt which the object pose is expressed.",
        ),
        ("object_pose_in_base_txyz", [0.0, 0.0, 0.0], "Translation of the object"),
        (
            "object_pose_in_base_qxyzw",
            [0.0, 0.0, 0.0, 1.0],
            "Orientation of the object as a quaternion",
        ),
        ("object_id", "tless-obj_000021", "Object ID as published by Happypose."),
        (
            "happypose_time",
            0.2,
            "Computation time of happypose. Used both as publication delay and rate",
        ),
    )

    def __init__(self):
        super().__init__("happypose_simulation")

        for name, value, _ in self.PARAMETERS:
            self.declare_parameter(name, value)

        self._bMc_tf = None

        # self.detection_pub = self.create_publisher(
        #    Detection2DArray,
        #    "/happypose/detections",
        #    qos_profile_system_default,
        # )
        self.detection_pub = self.create_publisher(
            PoseStamped,
            "/object/detections",
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
        """Pose of the object wrt base"""
        xyz = self.get_parameter("object_pose_in_base_txyz").value
        q = self.get_parameter("object_pose_in_base_qxyzw").value
        return pinocchio.XYZQUATToSE3(np.array(xyz + q))

    def _publish_detection(
        self, cMo: pinocchio.SE3, stamp: builtin_interfaces.msg.Time
    ):
        """
        - cMo: pose of the object wrt camera
        """
        # hypothesis = ObjectHypothesisWithPose()
        # hypothesis.hypothesis.class_id = self._object_id
        # hypothesis.pose.pose = se3_to_pose_msg(cMo)
        #
        # detection = Detection2D(results=[hypothesis])
        # detection.header.stamp = stamp
        # detection.header.frame_id = self._camera_name
        # detections = Detection2DArray(detections=[detection])
        detection = PoseStamped()
        detection.header.stamp = stamp
        detection.header.frame_id = self._camera_name
        detection.pose = se3_to_pose_msg(cMo)
        self.detection_pub.publish(detection)

    def publish(self):
        # at T + DT, publish cMo(T) = cMb(T) * bMo with the expected time stamp.
        if self._bMc_tf is not None:
            bMc = transform_msg_to_se3(self._bMc_tf.transform)
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
    if help:
        print(HappyposeSimulation.__doc__)
        print("This node can be configured with the following ROS node parameters:")
        for name, value, doc in HappyposeSimulation.PARAMETERS:
            print(f"- {name} [{value}]\n  {doc}")
        return

    rclpy.init()
    node = HappyposeSimulation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
