#!/usr/bin/env python3
"""
Mocap MPC corrector node.

Intercepts mpc_input messages from the HPP orchestrator, computes the
translation error between FK and mocap at the current configuration, and
applies that correction to every target EE pose in the trajectory.

Correction formula (translation only):
    delta_t = T_FK.translation - T_mocap.translation
    target_corrected.position += delta_t

This compensates for a constant FK bias (joint calibration offset or
marker placement). The MPC receives targets that, when reached via FK,
put the real EE at the intended world position.

Subscriptions:
    mpc_input          (agimus_msgs/MpcInput)  — from HPP orchestrator
    /mocap_ee_pose     (geometry_msgs/PoseStamped) — from mocap_ee_publisher
    /joint_states      (sensor_msgs/JointState)

Publications:
    mpc_input_corrected (agimus_msgs/MpcInput)  — to agimus_controller
"""

import glob
import os
import sys
import tempfile
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from agimus_msgs.msg import MpcInput
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

_EE_FRAME_KW = "gripper_right_tool_holder"


class MocapMpcCorrectorNode(Node):

    def __init__(self):
        super().__init__("mocap_mpc_corrector")

        self._model = None
        self._data  = None
        self._ee_id = None
        self._q_current: np.ndarray | None = None
        self._mocap_se3: pin.SE3 | None = None   # latest EE pose from mocap
        self._mocap_new: bool = False             # True when a new mocap msg arrived
        self._last_valid_correction: pin.SE3 | None = None  # frozen when mocap lost

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/robot_description", self._urdf_cb, qos_latched
        )
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(PoseStamped, "/mocap_ee_pose", self._mocap_cb, 10)
        _qos_mpc = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(MpcInput, "mpc_input", self._mpc_input_cb, _qos_mpc)
        self._pub = self.create_publisher(MpcInput, "mpc_input_corrected", _qos_mpc)

        self.get_logger().info("Mocap MPC corrector ready — waiting for robot_description …")

    # ── Model loading ─────────────────────────────────────────────────────────

    def _urdf_cb(self, msg: String) -> None:
        if self._model is not None:
            return
        with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f:
            f.write(msg.data)
            tmp = f.name
        model = pin.buildModelFromUrdf(tmp)
        os.unlink(tmp)
        data   = model.createData()
        ee_id  = None
        for i, frame in enumerate(model.frames):
            if _EE_FRAME_KW in frame.name:
                ee_id = i
                break
        if ee_id is None:
            self.get_logger().error(f"Frame '{_EE_FRAME_KW}' not found in model.")
            return
        self._model = model
        self._data  = data
        self._ee_id = ee_id
        self.get_logger().info(
            f"Model loaded — EE frame: '{model.frames[ee_id].name}' (id={ee_id})"
        )

    # ── State callbacks ───────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        if self._model is None:
            return
        js_map = dict(zip(msg.name, msg.position))
        q = pin.neutral(self._model)
        for jid in range(1, self._model.njoints):
            jname = self._model.names[jid]
            val = js_map.get(jname) or js_map.get(jname.replace("tiago_pro/", ""))
            if val is not None and self._model.joints[jid].nq == 1:
                q[self._model.joints[jid].idx_q] = val
        self._q_current = q

    def _mocap_cb(self, msg: PoseStamped) -> None:
        t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        self._mocap_se3 = pin.XYZQUATToSE3(np.concatenate([t, q]))
        self._mocap_new = True

    # ── Correction ────────────────────────────────────────────────────────────

    def _fk_se3(self) -> pin.SE3 | None:
        if self._model is None or self._q_current is None:
            return None
        pin.forwardKinematics(self._model, self._data, self._q_current)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._ee_id].copy()

    def _mpc_input_cb(self, msg: MpcInput) -> None:
        if not msg.ee_inputs:
            self._pub.publish(msg)
            return

        if self._mocap_new:
            T_fk = self._fk_se3()
            if T_fk is not None:
                # Left-side SE3 correction: T_correction = T_fk * T_mocap⁻¹
                # When applied to a target T: T_corrected = T_correction * T_target
                # At convergence FK(q*) = T_corrected, real EE = T_mocap(q*) = T_target ✓
                self._last_valid_correction = T_fk * self._mocap_se3.inverse()
            self._mocap_new = False

        if self._last_valid_correction is None:
            self._pub.publish(msg)
            return

        T_correction = self._last_valid_correction

        for ee_input in msg.ee_inputs:
            t = np.array([ee_input.pose.position.x,
                          ee_input.pose.position.y,
                          ee_input.pose.position.z])
            q = np.array([ee_input.pose.orientation.x,
                          ee_input.pose.orientation.y,
                          ee_input.pose.orientation.z,
                          ee_input.pose.orientation.w])
            T_target    = pin.XYZQUATToSE3(np.concatenate([t, q]))
            T_corrected = T_correction * T_target
            q_out       = pin.Quaternion(T_corrected.rotation)

            ee_input.pose.position.x    = float(T_corrected.translation[0])
            ee_input.pose.position.y    = float(T_corrected.translation[1])
            ee_input.pose.position.z    = float(T_corrected.translation[2])
            ee_input.pose.orientation.x = float(q_out.x)
            ee_input.pose.orientation.y = float(q_out.y)
            ee_input.pose.orientation.z = float(q_out.z)
            ee_input.pose.orientation.w = float(q_out.w)

        self._pub.publish(msg)


def main(args=None):
    import signal

    rclpy.init(args=args)
    node = MocapMpcCorrectorNode()

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
