"""Implement the agimus_demo_05_pick_and_place Orchestrator"""

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pinocchio as pin
import time
import typing as T

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_system_default
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray, Detection2D

from agimus_demo_05_pick_and_place.franka_gripper_client import FrankaGripperClient

from agimus_demo_05_pick_and_place.hpp_client import (
    HPPInterface,
    get_traj_points_from_path,
)
from agimus_demo_05_pick_and_place.async_subscriber import AsyncSubscriber
from agimus_demo_05_pick_and_place.trajectory_publisher import TrajectoryPublisher


def map_object_id(obj_id, dataset="tless"):
    num_part = obj_id.split("_")[1]
    return f"{dataset}-obj_{int(num_part):06d}"


def multiply_poses(pose1: list[float], pose2: list[float]) -> list[float]:
    p1 = pin.XYZQUATToSE3(pose1)
    p2 = pin.XYZQUATToSE3(pose2)
    return pin.SE3ToXYZQUAT(p1 * p2)


def get_hardcoded_initial_object_pose(object_name: str) -> T.Tuple[str, list[float]]:
    """Return initial object position in world frame."""
    frame = "panda/support_link"  # world frame
    if object_name == "obj_21":
        return frame, [-0.12, -0.2, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_23":
        return frame, [0.0, -0.23, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_25":
        return frame, [0.1, -0.17, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_26":
        return frame, [0.2, -0.15, 0.85, 0.0, 0.0, 0.0, 1.0]
    else:
        raise ValueError(f"Object {object_name} not found")


def get_hardcoded_final_object_pose(object_name: str) -> list[float]:
    """Return desired object position in destination box frame."""
    if object_name == "obj_21":
        return "dest_box/base_link", [-0.12, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_23":
        return "dest_box/base_link", [0.0, -0.03, 0.1, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_25":
        return "dest_box/base_link", [0.0, -0.05, 0.1, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_26":
        return "dest_box/base_link", [0.15, 0.05, 0.1, 0.0, 0.0, 0.0, 1.0]
    else:
        raise ValueError(f"Object {object_name} not found")


@dataclass
class OrchestratorParams:
    """Orchestrator parameters."""

    max_holding_force: float = 30.0
    parking_configuration: npt.NDArray = np.zeros(0)
    destination_configuration: npt.NDArray = np.zeros(0)


class Orchestrator(object):
    """Orchestrator of demo agimus_demo_05_pick_and_place"""

    def __init__(self):
        self._node = Node("pick_and_place")
        self.param = OrchestratorParams()

        self.franka_gripper_cient = FrankaGripperClient(self._node)
        self.default_object_name = "obj_23"

        self.is_simulation = False

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
        self.vision_client = AsyncSubscriber(
            self._node,
            Detection2DArray,
            "/happypose/detections",
            qos_profile_system_default,
        )
        self.open_gripper()

    def get_most_confident_object_pose(
        self, detection_msg: Detection2DArray, object_name: str
    ) -> T.Tuple[str, list[float]]:
        # TODO: change the map if we want to use YCBV
        filtered_detections = [
            (d, d.results[0].hypothesis.score)
            for d in detection_msg.detections
            if d.results[0].hypothesis.class_id == map_object_id(object_name)
        ]
        if len(filtered_detections) == 0:
            return "", None
        detection: Detection2D = max(filtered_detections, key=lambda pair: pair[1])[0]
        pose: Pose = detection.results[0].pose.pose
        return (
            detection.header.frame_id,
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        )

    def set_temporary_hpp_q_init(self, pose):
        """Useful to get correct transformation between robot's frames."""
        q_tmp = (
            pose
            + [0, 0, 1.0, 0, 0, 0, 1]
            + self.hpp_client.default_obstacle_pose
            + self.hpp_client.default_obstacle2_pose
        )
        self.hpp_client.robot.setCurrentConfig(q_tmp)

    def get_object_start_and_goal_pose(
        self, object_name: str, use_hardcoded_poses: bool
    ) -> T.Tuple[T.Tuple[str, float], T.Tuple[str, float]]:
        """Return start and goal pose of the object in world frame."""
        obj_start_pose = None
        if use_hardcoded_poses:
            # TEMP fix: just hardcode pose from happypose
            obj_start_pose = get_hardcoded_initial_object_pose(object_name)
        else:
            # REAL setup, TODO: fix communication error when happy pose is running
            print("waiting for obj pose")
            object_detections = self.vision_client.wait_for_future()
            print("got obj pose")
            obj_start_pose = self.get_most_confident_object_pose(
                object_detections, object_name
            )
            obj_start_pose[0] = "panda/" + obj_start_pose[0]

        if obj_start_pose[1] is None:
            raise ValueError(f"No {object_name} object detected")
        obj_goal_pose = get_hardcoded_final_object_pose(object_name=object_name)
        return obj_start_pose, obj_goal_pose

    def open_gripper(self):
        self.franka_gripper_cient.send_goal(position=0.039, max_effort=10.0)
        # TODO: change it to something normal
        time.sleep(0.05)

    def close_gripper(self):
        if self.is_simulation:
            self.franka_gripper_cient.send_goal(
                position=0.04, max_effort=self.param.max_holding_force
            )
            # TODO: change it to something normal
            time.sleep(0.05)
        else:
            self.franka_gripper_cient.grasp()
            # TODO: change it to something normal
            time.sleep(1.0)

    def publish(self, path_vector):
        traj = get_traj_points_from_path(path_vector)
        # TODO: get this from OCP params somehow
        traj += [traj[-1]] * 40  # OCP horizon
        self.trajectory_publisher.publish(traj)

    def go_to(
        self,
        desired_configuration: T.List[float],
        enable_visualization_in_gepetto_gui: bool = True,
    ):
        self.hpp_client = HPPInterface(
            object_name=self.default_object_name, use_spline_gradient_based_opt=False
        )
        current_robot_state = self.state_client.wait_for_future()
        traj = self.hpp_client.plan_free_motion(
            list(current_robot_state.position), desired_configuration
        )
        if enable_visualization_in_gepetto_gui:
            self.v = self.hpp_client.vf.createViewer()
            self.v(self.hpp_client.q_init)
            input("Trajectory computed. Ready to move. Press Enter to start motion...")
        self.publish(traj)

    def pick_and_place(
        self,
        object_name: str,
        use_hardcoded_poses: bool = False,
        enable_visualization_in_gepetto_gui: bool = True,
    ):
        current_robot_state = self.state_client.wait_for_future()
        q_init = list(current_robot_state.position)

        self.hpp_client = HPPInterface(
            object_name=object_name,
            use_spline_gradient_based_opt=False,
        )
        start_obj_pose, goal_obj_pose = self.get_object_start_and_goal_pose(
            object_name, use_hardcoded_poses=use_hardcoded_poses
        )
        self.hpp_client.set_relative_start_obj_pose(
            start_obj_pose[1], q_init, start_obj_pose[0]
        )
        self.hpp_client.set_goal_obj_pose(goal_obj_pose[0], goal_obj_pose[1][:3])

        grasp_path, placing_path, freefly_path = self.hpp_client.plan_pick_and_place(
            list(current_robot_state.position)
        )

        if enable_visualization_in_gepetto_gui:
            self.v = self.hpp_client.vf.createViewer()
            self.v(self.hpp_client.q_init)
            input("Trajectory computed. Ready to move. Press Enter to start motion...")

        self.open_gripper()
        self.open_gripper()
        self.publish(grasp_path)
        if placing_path is not None:
            # TODO: check automatically
            self.close_gripper()
            self.publish(placing_path)
            self.open_gripper()
            self.publish(freefly_path)
        # Commented out since restart does not work properly (corba crashes)
        # self.hpp_client.restart()
        # del self.hpp_client

    # def go_to_ee(self, target_ee):
    #     current_robot_state = self.state_client.wait_for_new_state()
    #     trajectory = self.hpp_client.plan_ee(current_robot_state.position, target_ee)
    #     self.trajectory_publisher.publish(trajectory)

    # def go_to_parking_pose(self):
    #     self.go_to(self.param.parking_configuration)

    # def go_to_destination_pose(self):
    #     self.go_to(self.param.destination_configuration)

    # def go_to_pre_grasp(self):
    #     new_target_pose = self.target_client.wait_for_new_target_pose()
    #     trajectory = self.go_to_ee(new_target_pose)
    #     self.trajectory_publisher(trajectory)
