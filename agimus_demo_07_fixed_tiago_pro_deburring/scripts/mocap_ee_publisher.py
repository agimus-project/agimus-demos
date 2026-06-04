#!/usr/bin/env python3
"""
Real-time EE pose publisher from Qualisys mocap.

Computes T_ee_in_base = T_base_mocap⁻¹ · T_ee_mocap and publishes:
  - TF frame  : base_link → mocap_ee
  - Topic     : /mocap_ee_pose  (geometry_msgs/PoseStamped)

Usage:
    python3 scripts/mocap_ee_publisher.py
    python3 scripts/mocap_ee_publisher.py --mocap-ip 140.93.1.100 --rate 100
"""

import argparse
import glob
import sys
import time

import numpy as np
import pinocchio as pin

for _p in sorted(
    glob.glob("/home/gepetto/ros2_ws/install/*/lib/python3*/site-packages")
    + glob.glob("/home/gepetto/agimus_deps_ws/install/*/lib/python3*/site-packages")
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qualisys import QualisysClient

_QUALISYS_IP    = "140.93.1.100"
# QTM body IDs (must match the Qualisys server's body list order)
_MOCAP_BODIES   = {"pylone": 0, "tiago_endEffector": 2, "tiago_base": 1}
_BASE_IDX       = 2  # local index of tiago_base
_EE_IDX         = 1  # local index of tiago_endEffector
_BASE_FRAME     = "base_link"
_MOCAP_EE_FRAME = "mocap_ee"


def _mocap_se3(qc: QualisysClient, idx: int) -> pin.SE3:
    pos  = qc.getPositions()[idx]
    quat = qc.getOrientationQuats()[idx]
    return pin.XYZQUATToSE3(np.concatenate([pos, quat]))


def _se3_to_transform_stamped(T, stamp, parent, child) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp    = stamp
    msg.header.frame_id = parent
    msg.child_frame_id  = child
    msg.transform.translation.x = float(T.translation[0])
    msg.transform.translation.y = float(T.translation[1])
    msg.transform.translation.z = float(T.translation[2])
    q = pin.Quaternion(T.rotation)
    msg.transform.rotation.x = float(q.x)
    msg.transform.rotation.y = float(q.y)
    msg.transform.rotation.z = float(q.z)
    msg.transform.rotation.w = float(q.w)
    return msg


def _se3_to_pose_stamped(T, stamp, frame) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame
    msg.pose.position.x = float(T.translation[0])
    msg.pose.position.y = float(T.translation[1])
    msg.pose.position.z = float(T.translation[2])
    q = pin.Quaternion(T.rotation)
    msg.pose.orientation.x = float(q.x)
    msg.pose.orientation.y = float(q.y)
    msg.pose.orientation.z = float(q.z)
    msg.pose.orientation.w = float(q.w)
    return msg


class MocapEEPublisher(Node):
    """Publishes the EE pose from mocap as TF and PoseStamped."""

    def __init__(self, mocap_ip: str, rate_hz: float):
        super().__init__("mocap_ee_publisher")
        self._tf_broadcaster = TransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseStamped, "/mocap_ee_pose", 10)

        self.get_logger().info(f"Connecting to Qualisys at {mocap_ip} …")
        self._qc = QualisysClient(ip=mocap_ip, bodies=_MOCAP_BODIES)
        time.sleep(1.0)
        self.get_logger().info("Mocap connected. Publishing mocap_ee at %.0f Hz." % rate_hz)

        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        positions = self._qc.getPositions()    # shape (N, 3) — single copy
        quats     = self._qc.getOrientationQuats()  # shape (N, 4) — single copy

        # Skip if either marker is not tracked (NaN position)
        if np.any(np.isnan(positions[_BASE_IDX])) or np.any(np.isnan(positions[_EE_IDX])):
            return

        T_base = pin.XYZQUATToSE3(np.concatenate([positions[_BASE_IDX], quats[_BASE_IDX]]))
        T_ee   = pin.XYZQUATToSE3(np.concatenate([positions[_EE_IDX],   quats[_EE_IDX]]))
        T_ee_in_base = T_base.inverse() * T_ee

        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(
            _se3_to_transform_stamped(T_ee_in_base, stamp, _BASE_FRAME, _MOCAP_EE_FRAME)
        )
        self._pose_pub.publish(
            _se3_to_pose_stamped(T_ee_in_base, stamp, _BASE_FRAME)
        )

    def destroy_node(self) -> None:
        self._qc.stop()
        super().destroy_node()


def main():
    import signal

    parser = argparse.ArgumentParser(description="Publish mocap EE pose as TF + PoseStamped.")
    parser.add_argument("--mocap-ip", default=_QUALISYS_IP, help="Qualisys server IP")
    parser.add_argument("--rate", type=float, default=100.0, help="Publish rate in Hz")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MocapEEPublisher(mocap_ip=args.mocap_ip, rate_hz=args.rate)

    def _shutdown(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
