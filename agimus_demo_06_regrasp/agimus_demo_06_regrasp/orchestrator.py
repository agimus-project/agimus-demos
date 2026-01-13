"""Implement the agimus_demo_06_regrasp Orchestrator"""

from dataclasses import dataclass
import numpy as np
from typing import Tuple

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray

from agimus_demo_06_regrasp.franka_gripper_client import FrankaGripperClient

from agimus_demo_06_regrasp.manipulation_rrt_planner import ManipulationPlanner
from agimus_demo_06_regrasp.async_subscriber import AsyncSubscriber
from agimus_demo_06_regrasp.trajectory_publisher import TrajectoryPublisher
from agimus_demo_06_regrasp.utils import (
    normalize_quaternion,
    multiply_poses,
    XYZQuatType,
    get_traj_points_from_path,
)
from hpp.corbaserver.manipulation import loadServerPlugin, Robot


def map_object_id(obj_id, dataset="tless"):
    num_part = obj_id.split("_")[1]
    return f"{dataset}-obj_{int(num_part):06d}"


def get_hardcoded_initial_object_pose(object_name: str) -> list[float]:
    """Return initial object position in world frame."""
    if object_name == "obj_20":
        return [0.1, -0.2, 0.85, -np.sqrt(2) / 2, 0.0, np.sqrt(2) / 2, 0.0]
    if object_name == "obj_21":
        return [-0.12, -0.2, 0.85, 0.0, 0.0, 0.0, 1.0]
    # elif object_name == "obj_23":
    #     return [0.1, -0.23, 0.9, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_23":  # happypose
        return [
            0.04143410548567772,
            -0.14208067953586578,
            0.539716362953186,
            0.07858735553072632,
            0.03146105167416045,
            -0.712108373698452,
            0.6969475514873564,
        ]
    elif object_name == "obj_26":
        return [0.2, -0.15, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "cont_grasp_net_obj":
        return [0.2, -0.15, 0.85, 0.0, 0.0, 0.0, 1.0]
    else:
        raise ValueError(f"Object {object_name} not found")


def get_hardcoded_final_object_pose(object_name: str) -> list[float]:
    """Return desired object position in world frame."""
    obj_id = int(object_name[-2:])
    if int(obj_id) in [1, 2, 3, 4, 11, 12, 13, 14, 15, 16]:
        return [-0.15, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]
    elif int(obj_id) in [19, 20, 23, 24]:
        return [0.25, 0.0, 0.18, 0.0, 0.0, 0.0, 1.0]
    else:
        return [0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 1.0]
    # else:
    #     raise ValueError(f"Object {object_name} not found")


def get_goal_box_pose(obj_id: str) -> list[float]:
    # small objects
    if int(obj_id) in [1, 2, 3, 4, 11, 12, 13, 14, 15, 16]:
        print("Grasping a small object")
        return [0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0]
    # plugs and stuff
    elif int(obj_id) in [19, 20, 23, 24]:
        print("Grasping a plug")
        return [0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0]
    else:
        print("Grasping other stuff")
        return [0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0]


@dataclass
class OrchestratorParams:
    """Orchestrator parameters."""

    max_holding_force: float = 40.0
    use_hardcoded_poses: bool = True
    use_smoothing_at_waypoints: bool = True
    ocp_horizon: int = 40


class Orchestrator(object):
    """Orchestrator of demo agimus_demo_06_regrasp"""

    def __init__(self):
        self._node = Node("regrasp")
        self.param = OrchestratorParams()

        self.franka_gripper_cient = FrankaGripperClient(self._node)
        self.default_object_name = "cont_grasp_net_obj"
        self.use_sim = (
            self._node.get_parameter("use_sim_time").get_parameter_value().bool_value
        )
        self.use_hardcoded_poses = self.param.use_hardcoded_poses
        self.smooth = self.param.use_smoothing_at_waypoints

        self.trajectory_publisher = TrajectoryPublisher(self._node)

        self.state_client = AsyncSubscriber(
            self._node,
            JointState,
            "/joint_states",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.target_client = AsyncSubscriber(
            self._node,
            Pose,
            "/target_object",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if not self.use_hardcoded_poses:
            self.vision_client = AsyncSubscriber(
                self._node,
                Detection2DArray,
                "/happypose/detections",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        self.open_gripper()
        # if hppcorbaserver is running in a separate script
        loadServerPlugin("corbaserver", "manipulation-corba.so")
        loadServerPlugin("corbaserver", "bin_picking.so")

    def get_best_object_pose(
        self,
        detection_msg: Detection2DArray,
        object_name: str,
        metric: str = "confidence",  # 'confidence', 'closeness'
    ) -> list[float]:
        """Get best (according to metric) pose from the happypose prediction"""
        # TODO: change the map if we want to use YCBV
        filtered_detections = [
            (d, d.results[0].hypothesis.score, d.results[0].pose.pose.position.z)
            for d in detection_msg.detections
            if d.results[0].hypothesis.class_id == map_object_id(object_name)
        ]
        if len(filtered_detections) == 0:
            raise ValueError(
                f"in get best pose No object with id {object_name}, all present ids: {[d.results[0].hypothesis.class_id for d in detection_msg.detections]}"
            )
        if metric == "confidence":
            detection = max(filtered_detections, key=lambda pair: pair[1])[0]
        elif metric == "closeness":
            detection = min(filtered_detections, key=lambda pair: pair[2])[0]
        else:
            raise ValueError(f"Unknown metric for best object pose {metric}")
        pose: Pose = detection.results[0].pose.pose
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def get_object_start_and_goal_pose(
        self, object_name: str, q: XYZQuatType, robot: Robot
    ) -> Tuple[float, float]:
        """Return start and goal pose of the object in world frame."""
        obj_in_world_start_pose = None
        if self.use_hardcoded_poses:
            # TEMP fix: just hardcode pose from happypose
            obj_in_cam_start_pose = get_hardcoded_initial_object_pose(object_name)
            robot.setCurrentConfig(q)
            # TODO: change from hardcoded robot name
            cam_in_world_pose = robot.getLinkPosition(
                linkName="panda/camera_color_optical_frame"
            )
            obj_in_world_start_pose = normalize_quaternion(
                multiply_poses(cam_in_world_pose, obj_in_cam_start_pose)
            )
        else:
            # REAL setup
            print("waiting for vision")
            object_detections = self.vision_client.wait_for_future()
            print("Got vision")
            obj_in_cam_start_pose = self.get_best_object_pose(
                object_detections, object_name, metric="closeness"
            )
            robot.setCurrentConfig(q)
            # TODO: change from hardcoded robot name
            cam_in_world_pose = robot.getLinkPosition(
                linkName="panda/camera_color_optical_frame"
            )
            obj_in_world_start_pose = normalize_quaternion(
                multiply_poses(cam_in_world_pose, obj_in_cam_start_pose)
            )
        if obj_in_world_start_pose is None:
            raise ValueError(f"No {object_name} object detected")
        # TODO: think of how to change these hardcoded values
        # obj_23
        rotated90 = multiply_poses(
            obj_in_world_start_pose, [0.0, 0.0, 0.0, 0.7071068, 0.0, 0.0, 0.7071068]
        )
        rotated180 = multiply_poses(
            rotated90, [0.0, 0.0, 0.0, 0.7071068, 0.0, 0.0, 0.7071068]
        )
        # obj_20
        # rotated90 = multiply_poses(
        #     obj_in_world_start_pose, [0.0, 0.0, 0.0, 0.0, 0.0, 0.7071068, 0.7071068]
        # )
        # rotated180 = multiply_poses(
        #     rotated90, [0.0, 0.0, 0.0, 0.0, 0.0, 0.7071068, 0.7071068]
        # )
        return obj_in_world_start_pose, rotated90, rotated180

    def open_gripper(self):
        self.franka_gripper_cient.send_goal(position=0.039, max_effort=10.0)
        # TODO: change it to something normal
        # time.sleep(0.05)

    def close_gripper(self):
        self.franka_gripper_cient.send_goal(
            position=0.0, max_effort=self.param.max_holding_force
        )
        # TODO: change it to something normal
        # time.sleep(0.05)

    def grasp(self):
        self.franka_gripper_cient.grasp()
        # TODO: change it to something normal
        # time.sleep(1.0)

    def publish(self, path_vector, smooth_at_len=[], dt=0.01):
        traj = get_traj_points_from_path(path_vector, dt=dt)
        # TODO: get this from OCP params somehow
        traj += [
            traj[-1]
        ] * self.param.ocp_horizon  # add points in the end of len of OCP horizon
        if len(smooth_at_len) > 0:
            smoothing_indices = [int(el / dt) for el in smooth_at_len]
            self.trajectory_publisher.publish(traj, smoothing_indices=smoothing_indices)
        else:
            self.trajectory_publisher.publish(traj)

    def regrasp(self, object_name: str):
        current_robot_state = self.state_client.wait_for_future()
        # Test case to verify that method works for a very simple task
        # start_pose = get_hardcoded_initial_object_pose(object_name)
        # goal_pose = start_pose.copy()
        # goal_pose[1] += 0.1

        # Dummy hpp_q_init for getting FK to work
        hpp_q_init = list(current_robot_state.position) + [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self.planner = ManipulationPlanner(object_name)
        # self.hpp_client = HPPInterface(
        #     object_name=object_name, use_spline_gradient_based_opt=False
        # )
        start_obj_pose, rotated_90, rotate_180 = self.get_object_start_and_goal_pose(
            object_name=object_name, q=hpp_q_init, robot=self.planner.robot
        )

        hpp_q_init = list(current_robot_state.position) + start_obj_pose
        hpp_q_goal = list(current_robot_state.position) + rotate_180

        is_solved, path_sequences = self.planner.solve(hpp_q_init, hpp_q_goal)
        print(f"Manipulation plan is found: {is_solved}")
        if is_solved:
            input("Solution found! ready to execute it?")
            for path, object_moves, waypoints_at_len in path_sequences:
                if self.smooth:
                    self.publish(path, waypoints_at_len)
                else:
                    self.publish(path)
                # assuming object moves always goes 0-1-0-1-0-1
                if not object_moves:
                    if self.use_sim:
                        self.close_gripper()  # for simulation
                    else:
                        self.grasp()  # for hardware robot
                else:
                    self.open_gripper()
                # This input provides some delay for robot to grasp
                input("Continue to the next path?")
