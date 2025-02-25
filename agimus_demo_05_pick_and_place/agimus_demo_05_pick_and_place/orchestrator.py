"""Implement the agimus_demo_05_pick_and_place Orchestrator"""

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pinocchio as pin
import time
import re

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray

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


def hardcoded_config() -> list[float]:
    """
    Place the ros2 topic echo /happypose/detections output and get a list
    """
    str_pose = """
        position:
          x: 0.07415620982646942
          y: -0.0037332407664507627
          z: 0.4192313551902771
        orientation:
          x: 0.5784230576928513
          y: 0.770402139589831
          z: 0.2623295141316406
          w: 0.05559180051464918
    """
    float_values = list(map(float, re.findall(r"[-+]?\d*\.\d+|\d+", str_pose)))

    return float_values


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
        self.object_name = "obj_23"

        self.hpp_client = HPPInterface(object_name=self.object_name)
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
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def get_most_confident_object_pose(
        self, detection_msg: Detection2DArray
    ) -> list[float]:
        # TODO: change the map if we want to use YCBV
        filtered_detections = [
            (d, d.results[0].hypothesis.score)
            for d in detection_msg.detections
            if d.results[0].hypothesis.class_id == map_object_id(self.object_name)
        ]
        if len(filtered_detections) == 0:
            return
        detection = max(filtered_detections, key=lambda pair: pair[1])[0]
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

    def open_gripper(self):
        self.franka_gripper_cient.send_goal(position=0.039, max_effort=10.0)
        # TODO: change it to something normal
        time.sleep(0.05)

    def close_gripper(self):
        self.franka_gripper_cient.send_goal(
            position=0.0, max_effort=self.param.max_holding_force
        )
        # TODO: change it to something normal
        time.sleep(0.05)

    def grasp(self):
        self.franka_gripper_cient.grasp()
        # TODO: change it to something normal
        time.sleep(1.0)

    def publish(self, path_vector):
        traj = get_traj_points_from_path(path_vector)
        # TODO: get this from OCP params somehow
        traj += [traj[-1]] * 40  # OCP horizon
        self.trajectory_publisher.publish(traj)

    def go_to(self, desired_configuration):
        current_robot_state = self.state_client.wait_for_future()
        backup_goal_pose = self.hpp_client.goal_obj_pose.copy()
        self.hpp_client.goal_obj_pose = self.hpp_client.start_obj_pose.copy()
        traj = self.hpp_client.plan(
            list(current_robot_state.position), desired_configuration
        )
        self.publish(traj)
        self.hpp_client.restart()
        self.hpp_client.goal_obj_pose = backup_goal_pose.copy()

    def pick_and_place(self):
        current_robot_state = self.state_client.wait_for_future()
        # TEMP fix: just hardcode pose from happypose
        # obj_in_cam_pose = hardcoded_config()
        # REAL setup, TODO: fix communication error when happy pose is running
        object_detections = self.vision_client.wait_for_future()
        obj_in_cam_pose = self.get_most_confident_object_pose(object_detections)
        if obj_in_cam_pose is None:
            raise ValueError(f"No {self.object_name} object detected")
        hpp_q_init = (
            list(current_robot_state.position)
            + self.hpp_client.start_obj_pose
            + self.hpp_client.default_obstacle_pose
        )
        self.hpp_client.robot.setCurrentConfig(hpp_q_init)
        # TODO: change from hardcoded robot name
        cam_in_world_pose = self.hpp_client.robot.getLinkPosition(
            linkName="panda/camera_color_optical_frame"
        )
        obj_in_world_pose = multiply_poses(cam_in_world_pose, obj_in_cam_pose)
        # TODO: make this better
        obj_in_world_pose[3:] = obj_in_world_pose[3:] / np.linalg.norm(
            obj_in_world_pose[3:]
        )
        self.hpp_client.start_obj_pose = list(obj_in_world_pose)
        grasp_path, placing_path, freefly_path = self.hpp_client.plan(
            list(current_robot_state.position)
        )

        self.open_gripper()
        self.publish(grasp_path)
        if placing_path is not None:
            # self.close_gripper()  # for simulation
            self.grasp()  # for hardware robot
            self.publish(placing_path)
            self.open_gripper()
            self.publish(freefly_path)

        self.hpp_client.restart()

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
