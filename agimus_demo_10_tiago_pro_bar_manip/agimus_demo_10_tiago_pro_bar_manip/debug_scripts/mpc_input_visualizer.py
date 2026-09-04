#!/usr/bin/env python3
"""
Plays the trajectory sent to the /mpc_input topic in Viser
"""

import os
import tempfile
import sys
import numpy as np

import pinocchio as pin
from pinocchio.visualize import ViserVisualizer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from agimus_msgs.msg import MpcInput

MOVING_JOINTS = [
    "torso_lift_joint",
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
]


class MpcInputViz(Node):
    def __init__(self):
        super().__init__("mpc_input_viz")
        self._joint_state = None
        self._viz = None
        self._map_joints = None

        self.create_subscription(
            MpcInput,
            "/mpc_input",
            self._cb_mpc_input,
            QoSProfile(depth=1000, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )

        qos_rd = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/robot_description", self._cb_robot_description, qos_rd
        )

        qos_js = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, qos_js)

        self.get_logger().info("Waiting for /robot_description and /joint_states …")

    def _cb_joints(self, msg: JointState):
        self._joint_state = msg

    def _cb_robot_description(self, msg: String):
        if self._viz is not None:
            return
        self.get_logger().info(
            "/robot_description received — building robot model & visualizer..."
        )
        self._buildRobot(msg.data)

    def _buildRobot(self, urdf_str: str):
        with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
            f.write(urdf_str)
            tmp_path = f.name
        try:
            model, collision_model, visual_model = pin.buildModelsFromUrdf(
                tmp_path, root_joint=None, verbose=False
            )
            self._map_joints = self._get_joint_ids(model)

        finally:
            os.unlink(tmp_path)

        try:
            self._viz = ViserVisualizer(model, collision_model, visual_model)
            self._viz.initViewer(open=False)

        except ImportError as err:
            self.get_logger().error(
                "Error while initializing the viewer. It seems you should install viser"
            )
            print(err)
            sys.exit(0)

        self._viz.loadViewerModel()
        self._neutral_q0 = pin.neutral(model)
        self._viz.display(self._neutral_q0)
        self._viz.displayVisuals(True)
        self.get_logger().info("Successfully loaded the model from the URDF")

    def _cb_mpc_input(self, msg: String):
        if self._viz is None:
            return
        q = self._build_pin_q_from_mpc_input(msg.q)
        self.get_logger().info(f"{q}")
        self._viz.display(q)
        return 0

    def _get_joint_ids(self, model):
        q_ids = []
        for name in MOVING_JOINTS:
            joint_number = model.getJointId(name)
            joint_idx = model.idx_qs[joint_number]
            q_ids.append(joint_idx)
        self.get_logger().info(f"{q_ids}")
        return q_ids

    def _build_pin_q_from_mpc_input(self, q: np.ndarray):
        q_full = self._neutral_q0.copy()
        for i in range(0, len(self._map_joints)):
            q_full[self._map_joints[i]] = q[i]
        return q_full


def main():
    rclpy.init()
    node = MpcInputViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
