#!/usr/bin/env python3
"""
Debug script: publish a traj for the torso lift joint (0.05 → 0.3 m, 5 s) on /mpc_input.
Every other joint are freeze
"""

import copy
import os
import tempfile
import threading
import numpy as np
from collections import deque

import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from agimus_msgs.msg import MpcInput
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller_ros.ros_utils import weighted_traj_point_to_mpc_msg

POS_START = 0.05
POS_END = 0.30
TRAJ_DURATION = 5.0
OCP_DT = 0.01

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

LEFT_TOOL_NAME = "gripper_left_grasping_link"
RIGHT_TOOL_NAME = "gripper_right_grasping_link"

_W = {
    "w_q": [
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
    ],
    "w_dq": [0.001] * 15,
    "w_ddq": [0.001] * 15,
    "w_effort": [
        0.0,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
        0.001,
    ],
    "w_frame_translation": [10.0, 10.0, 10.0],
    "w_frame_rotation": [5.0, 5.0, 5.0],
    "w_collision_avoidance": 0.0,
}


class TorsoLiftMpcNode(Node):
    def __init__(self):
        super().__init__("torso_lift_mpc_traj")

        self._robot_model = None
        self._robot_data = None
        self._left_frame_id = None
        self._right_frame_id = None
        self._torso_idx = None
        self._joint_state = None

        self._buffer_lock = threading.Lock()
        self._buffer: deque[WeightedTrajectoryPoint] = deque()

        self._mpc_pub = self.create_publisher(
            MpcInput,
            "/mpc_input",
            QoSProfile(depth=1000, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(OCP_DT, self._publish_mpc_input_cb)

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
        if self._robot_model is not None and not self._buffer:
            self._fill_buffer()

    def _cb_robot_description(self, msg: String):
        if self._robot_model is not None:
            return
        self.get_logger().info("/robot_description received — building robot model...")
        self._buildRobot(msg.data)
        self.get_logger().info(
            f"Model ready | nq={self._robot_model.nq} | "
            f"torso_idx={self._torso_idx} | "
            f"left_fid={self._left_frame_id} right_fid={self._right_frame_id}"
        )
        if self._joint_state is not None:
            self._fill_buffer()

    def _buildRobot(self, urdf_str: str):
        with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
            f.write(urdf_str)
            tmp_path = f.name
        try:
            full_model, _, _ = pin.buildModelsFromUrdf(
                tmp_path, root_joint=None, verbose=False
            )
        finally:
            os.unlink(tmp_path)

        full_names = full_model.names.tolist()
        self.get_logger().info(f"Full model joints ({len(full_names)}): {full_names}")

        locked_ids = []
        for full in full_names:
            if full == "universe":
                continue
            joint_name = full.removeprefix("tiago_pro/")
            if joint_name not in MOVING_JOINTS:
                locked_ids.append(int(full_model.getJointId(full)))

        q_ref = pin.neutral(full_model)
        self._robot_model = pin.buildReducedModel(full_model, locked_ids, q_ref)
        self._robot_data = self._robot_model.createData()

        reduced_names = self._robot_model.names.tolist()
        self.get_logger().info(
            f"Reduced model joints ({len(reduced_names)}): {reduced_names}"
        )

        for jid in range(1, self._robot_model.njoints):
            jname = self._robot_model.names[jid].removeprefix("tiago_pro/")
            if jname == "torso_lift_joint":
                self._torso_idx = self._robot_model.joints[jid].idx_q
                break

        self._left_frame_id = self._get_frame_id(LEFT_TOOL_NAME)
        self._right_frame_id = self._get_frame_id(RIGHT_TOOL_NAME)

    def _get_frame_id(self, short_name: str) -> int:
        for candidate in [f"tiago_pro/{short_name}", short_name]:
            fid = self._robot_model.getFrameId(candidate)
            if fid < self._robot_model.nframes:
                self.get_logger().info(f"  Frame '{candidate}' → id={fid}")
                return fid
        available = [
            self._robot_model.frames[i].name for i in range(self._robot_model.nframes)
        ]
        raise RuntimeError(f"Frame '{short_name}' is missing. Available:\n{available}")

    def _build_q_from_joint_states(self) -> np.ndarray:
        q = pin.neutral(self._robot_model)

        js = self._joint_state
        name_to_pos = dict(zip(js.name, js.position))

        for jid in range(1, self._robot_model.njoints):
            jname = self._robot_model.names[jid].removeprefix("tiago_pro/")
            j = self._robot_model.joints[jid]

            if jname in name_to_pos:
                value = name_to_pos[jname]
                if j.nq == 1:
                    q[j.idx_q] = value
                elif j.nq == 2:
                    q[j.idx_q] = np.cos(value)
                    q[j.idx_q + 1] = np.sin(value)
            else:
                self.get_logger().warn(
                    f"Joint '{jname}' missing in /joint_states — remain at neutral",
                    throttle_duration_sec=5.0,
                )

        return q

    def _fill_buffer(self):
        q_init = self._build_q_from_joint_states()
        weights = self._build_traj_weights()
        n_points = int(TRAJ_DURATION / OCP_DT)

        pos_start = q_init[self._torso_idx]
        self.get_logger().info(
            f"Traj torso : {pos_start:.4f} → {POS_END:.4f} m ({n_points} pts)"
        )

        for i in range(n_points):
            alpha = i / max(n_points - 1, 1)
            q = q_init.copy()
            q[self._torso_idx] = pos_start + alpha * (POS_END - pos_start)
            pt = self._convert_point(i, q, weights)
            with self._buffer_lock:
                self._buffer.append(pt)

    def _convert_point(
        self, id: int, q_robot: np.ndarray, weights: TrajectoryPointWeights
    ) -> WeightedTrajectoryPoint:
        pin.framesForwardKinematics(self._robot_model, self._robot_data, q_robot)

        traj_point = TrajectoryPoint(
            id=id,
            time_ns=0,
            robot_configuration=list(q_robot),
            robot_velocity=list(np.zeros_like(q_robot)),
            robot_acceleration=list(np.zeros(len(q_robot))),
            robot_effort=list(np.zeros(len(q_robot))),
            forces={},
            end_effector_poses={
                LEFT_TOOL_NAME: copy.copy(self._robot_data.oMf[self._left_frame_id]),
                RIGHT_TOOL_NAME: copy.copy(self._robot_data.oMf[self._right_frame_id]),
            },
        )
        return WeightedTrajectoryPoint(point=copy.copy(traj_point), weights=weights)

    def _build_traj_weights(self) -> TrajectoryPointWeights:
        return TrajectoryPointWeights(
            w_robot_configuration=np.array(_W["w_q"]),
            w_robot_velocity=np.array(_W["w_dq"]),
            w_robot_acceleration=np.array(_W["w_ddq"]),
            w_robot_effort=np.array(_W["w_effort"]),
            w_end_effector_poses={
                LEFT_TOOL_NAME: np.concatenate(
                    [
                        np.array(_W["w_frame_translation"]),
                        np.array(_W["w_frame_rotation"]),
                    ]
                ),
                RIGHT_TOOL_NAME: np.concatenate(
                    [
                        np.array(_W["w_frame_translation"]),
                        np.array(_W["w_frame_rotation"]),
                    ]
                ),
            },
            w_end_effector_velocities={},
            w_forces={},
            w_collision_avoidance=_W["w_collision_avoidance"],
        )

    def _publish_mpc_input_cb(self):
        with self._buffer_lock:
            if not self._buffer:
                return
            pt = self._buffer[0] if len(self._buffer) == 1 else self._buffer.popleft()

        self._mpc_pub.publish(weighted_traj_point_to_mpc_msg(pt))


def main():
    rclpy.init()
    node = TorsoLiftMpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
