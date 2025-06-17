"""Implement the agimus_demo_06_regrasp Orchestrator"""

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pinocchio as pin
import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState, PointCloud2
from vision_msgs.msg import Detection2DArray
from contact_graspnet_msgs.srv import GetSceneGrasps

from agimus_demo_06_regrasp.franka_gripper_client import FrankaGripperClient

from agimus_demo_06_regrasp.hpp_client import (
    HPPInterface,
    get_traj_points_from_path,
)

from agimus_demo_06_regrasp.manipulation_rrt_planner import ManipulationPlanner
from agimus_demo_06_regrasp.async_subscriber import AsyncSubscriber
from agimus_demo_06_regrasp.trajectory_publisher import TrajectoryPublisher
from agimus_demo_06_regrasp.utils import (
    normalize_quaternion,
    posemsg2mat,
    graspnet_to_handle,
    inverse_pose,
    multiply_poses,
    XYZQuatType,
)
from agimus_demo_06_regrasp.poincloud_utils import read_points_xyz_rgb
from hpp.corbaserver.manipulation import loadServerPlugin


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

    max_holding_force: float = 30.0
    parking_configuration: npt.NDArray = np.zeros(0)
    destination_configuration: npt.NDArray = np.zeros(0)


class Orchestrator(object):
    """Orchestrator of demo agimus_demo_06_regrasp"""

    def __init__(self):
        self._node = Node("regrasp")
        self.param = OrchestratorParams()

        self.franka_gripper_cient = FrankaGripperClient(self._node)
        self.default_object_name = "cont_grasp_net_obj"
        self.use_hardcoded_poses = True
        self.use_contact_graspnet = False
        self.use_pointcloud = False
        self.use_sim = True
        self.smooth = False

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
        if self.use_pointcloud:
            self.pointcloud_client = AsyncSubscriber(
                self._node,
                PointCloud2,
                "/camera/depth/color/points",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        if self.use_contact_graspnet:
            self.grasps_client = self._node.create_client(
                GetSceneGrasps, "contact_graspnet/get_scene_grasps"
            )
        self.open_gripper()
        # if hppcorbaserver is running in a separate script
        loadServerPlugin("corbaserver", "manipulation-corba.so")
        loadServerPlugin("corbaserver", "bin_picking.so")

        if self.use_contact_graspnet:
            self.detected_grasps = self.get_all_grasps()
            current_robot_state = self.state_client.wait_for_future()
            self.grasp_q = list(current_robot_state.position)

    def get_all_grasps(self) -> dict[list[tuple[np.array, float]]]:
        """Get all grasps from the graspnet service"""
        self.grasps_client.wait_for_service()
        self._node.get_logger().info("Graspnet service is available, calling...")
        request = GetSceneGrasps.Request()
        future = self.grasps_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        self._node.get_logger().info("Graspnet service response received!")
        resp: GetSceneGrasps.GetSceneGrasps.Response = future.result()
        scene_grasps = resp.scene_grasps

        object_nb = len(scene_grasps.object_types)
        all_grasps = {}
        for i in range(object_nb):
            object_type = scene_grasps.object_types[i]
            object_id = f"{i}_{object_type}"

            # grasps of the i-th object
            grasps_i = scene_grasps.object_grasps[i]

            all_grasps[object_id] = [
                (posemsg2mat(grasp), score)
                for grasp, score in zip(grasps_i.grasps, grasps_i.scores)
            ]

        return all_grasps

    def get_best_object_pose(
        self,
        detection_msg: Detection2DArray,
        object_name: str,
        metric: str = "confidence",  # 'confidence', 'closeness'
    ) -> list[float]:
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

    def process_cont_graspnet_grasps(self, object_to_pick: str):
        """
        This function iterates over grasps detected with ContactGraspNet and adds corresponding handles
        """
        grasp_hpp_q = self.grasp_q + self.hpp_client.start_obj_pose
        self.hpp_client.robot.setCurrentConfig(grasp_hpp_q)
        # TODO: change from hardcoded robot name
        cam_in_world_pose = self.hpp_client.robot.getLinkPosition(
            linkName="panda/camera_color_optical_frame"
        )

        possible_grasps = self.detected_grasps[object_to_pick]
        # sort and leave only 20 best grasps
        possible_grasps = sorted(possible_grasps, key=lambda x: x[1], reverse=True)[:10]
        print(f"object {object_to_pick} has {len(possible_grasps)} grasps")
        # take the first grasp as identity and place the object there
        # identity handles are defined in the srdf file
        # express other handles in this frame
        ref_grasp_pose, _ = possible_grasps[0]
        obj_in_world_pose = graspnet_to_handle(
            pin.XYZQUATToSE3(cam_in_world_pose), pin.SE3(ref_grasp_pose)
        )
        for i, (grasp, _) in enumerate(possible_grasps[1:]):
            handle_in_world_pose = graspnet_to_handle(
                pin.XYZQUATToSE3(cam_in_world_pose), pin.SE3(grasp)
            )
            # convert to be expressed in the object frame

            self.hpp_client.add_handle(
                multiply_poses(inverse_pose(obj_in_world_pose), handle_in_world_pose),
                str(i),
            )
        return obj_in_world_pose

    def select_object_to_pick(self) -> str:
        """The first object to pick is the one that is the closest to the camera on z-axis"""
        closest_dist = np.inf
        object_to_pick = None
        for k, v in self.detected_grasps.items():
            print(f"Object {k} has {len(v)} grasps")
            for grasp, _ in v:
                if grasp[2, 3] < closest_dist:
                    closest_dist = grasp[2, 3]
                    object_to_pick = k
        print("Picking up object", object_to_pick, "with distance", closest_dist)
        return object_to_pick

    def get_object_start_and_goal_pose(
        self, object_name: str, q: XYZQuatType
    ) -> Tuple[float, float]:
        """Return start and goal pose of the object in world frame."""
        obj_in_world_start_pose = None
        if self.use_hardcoded_poses:
            # TEMP fix: just hardcode pose from happypose
            obj_in_cam_start_pose = get_hardcoded_initial_object_pose(object_name)
            self.hpp_client.robot.setCurrentConfig(q)
            # TODO: change from hardcoded robot name
            cam_in_world_pose = self.hpp_client.robot.getLinkPosition(
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
            self.hpp_client.robot.setCurrentConfig(q)
            # TODO: change from hardcoded robot name
            cam_in_world_pose = self.hpp_client.robot.getLinkPosition(
                linkName="panda/camera_color_optical_frame"
            )
            obj_in_world_start_pose = normalize_quaternion(
                multiply_poses(cam_in_world_pose, obj_in_cam_start_pose)
            )
        if obj_in_world_start_pose is None:
            raise ValueError(f"No {object_name} object detected")
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

    def set_pointcloud(self):
        pointcloud_msg = self.pointcloud_client.wait_for_future()
        points, colors, depth_frame = read_points_xyz_rgb(pointcloud_msg, "panda/")
        current_robot_state = self.state_client.wait_for_future()
        q_robot = list(current_robot_state.position)
        self.hpp_client.set_point_cloud(
            q_robot=q_robot,
            camera_frame_name=depth_frame,
            points=points.tolist(),
            colors=colors.tolist(),
        )

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

    def publish(self, path_vector, smooth_at_len=[], dt=0.01):
        traj = get_traj_points_from_path(path_vector, dt=dt)
        # TODO: get this from OCP params somehow
        traj += [traj[-1]] * 40  # OCP horizon
        if len(smooth_at_len) > 0:
            smoothing_indices = [int(el / dt) for el in smooth_at_len]
            self.trajectory_publisher.publish(traj, smoothing_indices=smoothing_indices)
        else:
            self.trajectory_publisher.publish(traj)

    def go_to(self, desired_configuration):
        self.hpp_client = HPPInterface(
            object_name=self.default_object_name, use_spline_gradient_based_opt=False
        )
        current_robot_state = self.state_client.wait_for_future()
        backup_goal_pose = self.hpp_client.goal_obj_pose.copy()
        self.hpp_client.goal_obj_pose = self.hpp_client.start_obj_pose.copy()
        traj = self.hpp_client.plan(
            list(current_robot_state.position), desired_configuration
        )
        self.publish(traj)
        # Commented out since restart does not work properly (corba crashes)
        # self.hpp_client.restart()
        self.hpp_client.goal_obj_pose = backup_goal_pose.copy()
        # del self.hpp_client

    def test_manip_plan(self, object_name: str):
        current_robot_state = self.state_client.wait_for_future()
        # start_pose = get_hardcoded_initial_object_pose(object_name)
        # goal_pose = start_pose.copy()
        # goal_pose[1] += 0.1

        hpp_q_init = list(current_robot_state.position) + [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self.hpp_client = HPPInterface(
            object_name=object_name, use_spline_gradient_based_opt=False
        )
        start_obj_pose, rotated_90, rotate_180 = self.get_object_start_and_goal_pose(
            object_name=object_name, q=hpp_q_init
        )

        self.planner = ManipulationPlanner(object_name)

        hpp_q_init = list(current_robot_state.position) + start_obj_pose
        hpp_q_goal = list(current_robot_state.position) + rotate_180

        is_solved, path_sequences = self.planner.solve(hpp_q_init, hpp_q_goal)
        print(f"Manipulation plan solved: {is_solved}")
        if is_solved:
            input("Solution found! ready to play?")
            for path, object_moves, waypoints_at_len in path_sequences:
                if self.smooth:
                    self.publish(path, waypoints_at_len)
                else:
                    self.publish(path)
                # assuming object moves always goes 0-1-0-1-0-1
                # doing it in the wrong place to have some delay
                if not object_moves:
                    if self.use_sim:
                        self.close_gripper()  # for simulation
                    else:
                        self.grasp()  # for hardware robot
                else:
                    self.open_gripper()
                input("Continue to next path?")
        # hpp_q_init = list(current_robot_state.position) + rotated_90
        # hpp_q_goal = list(current_robot_state.position) + rotate_180

        # is_solved, full_path = self.planner.solve(hpp_q_init, hpp_q_goal)
        # print(f"Manipulation plan solved: {is_solved}")
        # if is_solved:
        #     self.publish(full_path)

    def regrasp(self, object_name: str):
        """Main function that is called in the orchestrator
        Object start pose comes either from happypose or hardcoded value
        Object is [picked up and] rotated by 90 degreed around x axis (try both ways?)
        Repeat the rotation
        In the end we should achieve full 180 degrees rotation

        Args:
            object_name (str): _description_
        """
        current_robot_state = self.state_client.wait_for_future()
        # fake config to compute forward kinematics for camera
        hpp_q_init = list(current_robot_state.position) + [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self.hpp_client = HPPInterface(
            object_name=object_name, use_spline_gradient_based_opt=False
        )
        start_obj_pose, rotated_90, rotate_180 = self.get_object_start_and_goal_pose(
            object_name=object_name, q=hpp_q_init
        )
        print(start_obj_pose)
        print(rotated_90)
        print(rotate_180)

        # first pick_and_place, rotated by 90 degreed around x axis
        self.pick_and_place(
            object_name,
            start_obj_pose,
            rotated_90,
            goal_handles=["part/goal_handle_side_top"],
            # goal_handles=["part/goal_handle_first"],  # obj_20
        )
        # first pick_and_place, rotated by 180 degreed around x axis
        self.pick_and_place(
            object_name,
            rotated_90,
            rotate_180,
            goal_handles=["part/goal_handle_side_right"],
            # goal_handles=["part/goal_handle_second"],  # obj_20
        )

    def pick_and_place(
        self,
        object_name: str,
        start_obj_pose: XYZQuatType,
        goal_obj_pose: XYZQuatType,
        goal_handles: list[str],
    ):
        current_robot_state = self.state_client.wait_for_future()

        if self.use_pointcloud:
            self.set_pointcloud()
        self.hpp_client = HPPInterface(
            object_name=object_name, use_spline_gradient_based_opt=False
        )
        self.hpp_client.start_obj_pose = start_obj_pose
        self.hpp_client.goal_obj_pose = goal_obj_pose
        self.v = self.hpp_client.vf.createViewer()

        hpp_q_init = list(current_robot_state.position) + self.hpp_client.start_obj_pose
        self.hpp_client.robot.setCurrentConfig(hpp_q_init)
        input("Look on the robot in gepetto-viewer (start config)")

        hpp_q_init = list(current_robot_state.position) + goal_obj_pose
        self.hpp_client.robot.setCurrentConfig(hpp_q_init)
        input("Look on the robot in gepetto-viewer (goal config)")

        hpp_q_init = list(current_robot_state.position) + self.hpp_client.start_obj_pose
        self.hpp_client.robot.setCurrentConfig(hpp_q_init)

        grasp_path, placing_path, freefly_path = self.hpp_client.plan_pick_and_place(
            list(current_robot_state.position), goal_handles=goal_handles
        )

        self.open_gripper()
        self.open_gripper()
        self.publish(grasp_path)
        if placing_path is not None:
            # TODO: check automatically
            if self.use_sim:
                self.close_gripper()  # for simulation
            else:
                self.grasp()  # for hardware robot
            self.publish(placing_path)
            self.open_gripper()
            self.publish(freefly_path)  #
