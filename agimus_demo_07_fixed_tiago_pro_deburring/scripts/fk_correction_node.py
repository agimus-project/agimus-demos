#!/usr/bin/env python3
"""
FK correction node.

Intercepts mpc_input messages and applies a static SE3 correction
computed once at startup from the difference between FK(absolute_position)
and FK(motor_position).

Correction formula:
    T_correction = FK(q_motor) * FK(q_abs)⁻¹
    T_corrected  = T_correction * T_target

This compensates for the systematic offset between the absolute encoder
zero-calibration and the URDF joint zero-calibration (motor encoder based),
in both translation and rotation.

Subscriptions:
    mpc_input                                              (agimus_msgs/MpcInput)
    /joint_states                                          (sensor_msgs/JointState)
    /joint_torque_state_broadcaster/dynamic_joint_states   (control_msgs/DynamicJointState)
    /robot_description                                     (std_msgs/String)

Publications:
    mpc_input_corrected  (agimus_msgs/MpcInput)
"""

import glob
import os
import sys
import tempfile

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
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState
from std_msgs.msg import String

_EE_FRAME_KW = "gripper_right_tool_holder"


class FKCorrectionNode(Node):

    def __init__(self):
        super().__init__("fk_correction_node")

        self._model       = None
        self._data        = None
        self._ee_id       = None
        self._q_motor     = None
        self._q_abs       = None
        self._correction  = None   # SE3, set once

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos_mpc = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(String, "/robot_description", self._urdf_cb, qos_latched)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(
            DynamicJointState,
            "/joint_torque_state_broadcaster/dynamic_joint_states",
            self._djs_cb, 10,
        )
        self.create_subscription(MpcInput, "mpc_input", self._mpc_cb, qos_mpc)
        self._pub = self.create_publisher(MpcInput, "mpc_input_corrected", qos_mpc)

        self.get_logger().info("FK correction node ready — waiting for robot_description …")

    # ── Model loading ─────────────────────────────────────────────────────────

    def _urdf_cb(self, msg: String) -> None:
        if self._model is not None:
            return
        with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f:
            f.write(msg.data)
            tmp = f.name
        model = pin.buildModelFromUrdf(tmp)
        os.unlink(tmp)
        data  = model.createData()
        ee_id = None
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
        self.get_logger().info(f"Model loaded — EE frame '{model.frames[ee_id].name}'")

    # ── Joint state callbacks ─────────────────────────────────────────────────

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
        self._q_motor = q
        self._try_compute_correction()

    def _djs_cb(self, msg: DynamicJointState) -> None:
        if self._model is None:
            return
        q = pin.neutral(self._model)
        for i, jname in enumerate(msg.joint_names):
            iv   = msg.interface_values[i]
            imap = dict(zip(iv.interface_names, iv.values))
            val  = imap.get("absolute_position")
            if val is None:
                continue
            full_name = f"tiago_pro/{jname}"
            for name in (full_name, jname):
                try:
                    jid = self._model.getJointId(name)
                    if jid < self._model.njoints and self._model.joints[jid].nq == 1:
                        q[self._model.joints[jid].idx_q] = val
                        break
                except Exception:
                    pass
        self._q_abs = q
        self._try_compute_correction()

    # ── Correction computation ───────────────────────────────────────────────

    def _fk_ee(self, q: np.ndarray) -> pin.SE3:
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._ee_id].copy()

    def _try_compute_correction(self) -> None:
        if self._q_motor is None or self._q_abs is None:
            return
        T_motor = self._fk_ee(self._q_motor)
        T_abs   = self._fk_ee(self._q_abs)
        # T_correction = T_motor * T_abs⁻¹
        # At convergence FK(q_motor) = T_correction * T_target → FK(q_abs) = T_target ✓
        self._correction = T_motor * T_abs.inverse()
        dt_mm = np.round((T_abs.translation - T_motor.translation) * 1000, 2)
        self.get_logger().info(
            f"FK correction: δt = {dt_mm} mm",
            throttle_duration_sec=5.0,
        )

    # ── MPC interception ──────────────────────────────────────────────────────

    def _mpc_cb(self, msg: MpcInput) -> None:
        if self._correction is None or not msg.ee_inputs:
            self._pub.publish(msg)
            return

        for ee_input in msg.ee_inputs:
            t = np.array([ee_input.pose.position.x,
                          ee_input.pose.position.y,
                          ee_input.pose.position.z])
            q = np.array([ee_input.pose.orientation.x,
                          ee_input.pose.orientation.y,
                          ee_input.pose.orientation.z,
                          ee_input.pose.orientation.w])
            T_corrected = self._correction * pin.XYZQUATToSE3(np.concatenate([t, q]))
            q_out = pin.Quaternion(T_corrected.rotation)

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
    node = FKCorrectionNode()

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
