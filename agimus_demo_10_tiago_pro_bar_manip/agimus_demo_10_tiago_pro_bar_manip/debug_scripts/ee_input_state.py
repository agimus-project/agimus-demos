#!/usr/bin/env python3

import os
import tempfile
import time
from pathlib import Path

import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String


_EE_FRAME_RIGHT = "gripper_right_grasping_link"
_EE_FRAME_LEFT = "gripper_left_grasping_link"

_URDF_FETCH_TIMEOUT_S = 10.0
_DEFAULT_JOINT_STATES_TOPIC = "/joint_states"


def _fetch_urdf_from_topic(node: Node, timeout: float = _URDF_FETCH_TIMEOUT_S) -> str:
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    urdf_str = [None]
    sub = node.create_subscription(
        String, "/robot_description", lambda m: urdf_str.__setitem__(0, m.data), qos
    )
    t0 = time.time()
    while urdf_str[0] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)

    if urdf_str[0] is None:
        raise RuntimeError(f"Timeout after {timeout}s of waiting /robot_description. ")
    node.get_logger().info("/robot_description received")
    return urdf_str[0]


def _build_pin_models_from_robot_description(node: Node, package_dirs):
    urdf_str = _fetch_urdf_from_topic(node)
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f_:
        f_.write(urdf_str)
        tmp = f_.name
    try:
        return pin.buildModelsFromUrdf(tmp, package_dirs=package_dirs)
    finally:
        os.unlink(tmp)


def _q_from_js(model, js_msg):
    q = pin.neutral(model)
    positions = dict(zip(js_msg.name, js_msg.position))
    for jid in range(1, model.njoints):
        jname = model.names[jid]
        val = positions.get(jname, positions.get(f"tiago_pro/{jname}"))
        if val is not None and model.joints[jid].nq == 1:
            q[model.joints[jid].idx_q] = val
    return q


def _se3_to_pose_stamped(T: pin.SE3, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    t = T.translation
    q = pin.Quaternion(T.rotation)
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
        float(t[0]),
        float(t[1]),
        float(t[2]),
    )
    msg.pose.orientation.x = float(q.x)
    msg.pose.orientation.y = float(q.y)
    msg.pose.orientation.z = float(q.z)
    msg.pose.orientation.w = float(q.w)
    return msg


class EEPosePublisher(Node):
    def __init__(self):
        super().__init__("ee_pose_publisher")

        self.declare_parameter("joint_states_topic", _DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter("frame_id", "base_footprint")
        joint_states_topic = self.get_parameter("joint_states_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.get_logger().info("Building pinocchio model from /robot_description ...")
        from ament_index_python.packages import get_package_share_directory

        tiago_desc = Path(get_package_share_directory("tiago_pro_description"))
        self.model, _collision_model, _visual_model = (
            _build_pin_models_from_robot_description(
                self, package_dirs=[str(tiago_desc.parent.parent)]
            )
        )
        self.data = self.model.createData()
        self.ee_id_right = self.model.getFrameId(_EE_FRAME_RIGHT)
        self.ee_id_left = self.model.getFrameId(_EE_FRAME_LEFT)

        self._pub_right = self.create_publisher(PoseStamped, "ee_pose/right", 10)
        self._pub_left = self.create_publisher(PoseStamped, "ee_pose/left", 10)

        self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_cb,
            10,
        )

        self.get_logger().info(
            f"EEPosePublisher ready — '{joint_states_topic}' → FK → "
            f"'ee_pose/right' + 'ee_pose/left' (frame_id='{self.frame_id}')"
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        q = _q_from_js(self.model, msg)
        pin.framesForwardKinematics(self.model, self.data, q)

        stamp = self.get_clock().now().to_msg()
        T_right = self.data.oMf[self.ee_id_right]
        T_left = self.data.oMf[self.ee_id_left]

        self._pub_right.publish(_se3_to_pose_stamped(T_right, self.frame_id, stamp))
        self._pub_left.publish(_se3_to_pose_stamped(T_left, self.frame_id, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = EEPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
