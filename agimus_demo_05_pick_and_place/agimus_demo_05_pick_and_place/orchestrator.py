"""Implement the agimus_demo_05_pick_and_place Orchestrator"""

from typing import Tuple
from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pinocchio as pin
import time
import typing as T


from tf2_ros import StaticTransformBroadcaster
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_system_default
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray, Detection2D

from agimus_demo_05_pick_and_place.franka_gripper_client import FrankaGripperClient

from agimus_demo_05_pick_and_place.hpp_client import (
    HPPInterface,
    get_q_dq_ddq_arrays_from_path,
)
from agimus_demo_05_pick_and_place.async_subscriber import AsyncSubscriber
from agimus_controller_ros.simple_trajectory_publisher import (
    SimpleTrajectoryPublisher,
)

from agimus_controller_ros.ros_utils import pose_msg_to_se3, se3_to_transform_msg


def map_object_id(obj_id, dataset="tless"):
    num_part = obj_id.split("_")[1]
    return f"{dataset}-obj_{int(num_part):06d}"


def get_most_confident_object_pose(
    detection_msg: Detection2DArray, object_name: str, dataset_name: str
) -> Tuple[str, list[float]]:
    # TODO: change the map if we want to use YCBV
    filtered_detections = [
        (d, d.results[0].hypothesis.score)
        for d in detection_msg.detections
        if d.results[0].hypothesis.class_id
        == map_object_id(object_name, dataset=dataset_name)
    ]
    if len(filtered_detections) == 0:
        return ["", None]
    detection: Detection2D = max(filtered_detections, key=lambda pair: pair[1])[0]
    pose: Pose = detection.results[0].pose.pose
    return [
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
    ]


def get_hardcoded_initial_object_pose(object_name: str) -> T.Tuple[str, list[float]]:
    """Return initial object position in world frame."""
    frame = "panda/support_link"  # world frame
    if object_name == "obj_21":
        return frame, [-0.12, -0.2, 0.85, 0.0, 0.0, 0.0, 1.0]
    if object_name == "obj_22":
        return frame, [-0.1, -0.17, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_23":
        return frame, [0.0, -0.23, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_25":
        return frame, [0.1, -0.17, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_26":
        return frame, [0.2, -0.15, 0.85, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_31":
        return frame, [0.18, -0.14, 0.95, 0.0, 0.0, 0.707, 0.707]
    else:
        raise ValueError(f"Object {object_name} not found")


def get_hardcoded_final_object_pose(object_name: str) -> list[float]:
    """Return desired object position in destination box frame."""
    if object_name in ["obj_01", "obj_02"]:  # lamp base
        return "dest_box/base_link", [0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 1.0]
    elif object_name in [  # multisocket plugs
        "obj_19",
        "obj_20",
        "obj_21",
        "obj_22",
        "obj_23",
    ]:
        return "dest_box/base_link", [0.4, -0.55, 0.075, 0.0, 0.0, 0.0, 1.0]
        # return "dest_box/base_link", [0.1, 0.0, 0.03, 0.0, 0.0, 0.0, 1.0]
    elif object_name in ["obj_25", "obj_26"]:  # switches
        return "dest_box/base_link", [0.05, 0.0, 0.03, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_03":
        return "dest_box/base_link", [0.05, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0]
    elif object_name == "obj_31":
        return "dest_box/base_link", [0.05, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0]
    else:
        raise ValueError(f"Object {object_name} not found")


def get_in_fer_link0_M_support_link():
    in_fer_link0_M_support_link = pin.SE3.Identity()
    in_fer_link0_M_support_link.translation = np.array([0.563, -0.165, -0.780])
    in_fer_link0_M_support_link.rotation[0, 0] = -1.0
    in_fer_link0_M_support_link.rotation[1, 1] = -1.0
    return in_fer_link0_M_support_link


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

        self.source_bin_pose = [0.5, 0.3, 0.9, 0.0, 0.0, 0.0, 1.0]
        self.destination_bin_pose = [-0.1, -0.1, 0.9, 0.0, 0.0, 0.0, 1.0]
        self.min_opening_for_grasp = 0.01

        self.franka_gripper_cient = FrankaGripperClient(self._node)
        self.default_object_name = "obj_23"
        self.set_hardcoded_q0_start_and_above_source_bin()
        self.is_simulation = (
            self._node.get_parameter("use_sim_time").get_parameter_value().bool_value
        )
        self._node.declare_parameter("dataset_name", "tless")
        self.dataset_name = (
            self._node.get_parameter("dataset_name").get_parameter_value().string_value
        )
        self._node.declare_parameter("vision_type", "apriltag_det")
        self.vision_type = (
            self._node.get_parameter("vision_type").get_parameter_value().string_value
        )
        self.arm_id = (
            self._node.get_parameter("arm_id").get_parameter_value().string_value
        )

        self.object_to_grasp_name = None
        self.start_obj_pose = None
        self.goal_obj_pose = None

        self.trajectory_publisher = SimpleTrajectoryPublisher()
        self.dt = self.trajectory_publisher.dt

        self.gripper_reader = AsyncSubscriber(
            self._node,
            JointState,
            "/fer_gripper/joint_states",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

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
        if self.vision_type in ["simulate_happypose", "happypose"]:
            self.vision_client = AsyncSubscriber(
                self._node,
                Detection2DArray,
                "/happypose/detections",
                qos_profile_system_default,
            )
        elif self.vision_type in ["simulate_apriltag_det", "apriltag_det"]:
            self.vision_client = AsyncSubscriber(
                self._node,
                PoseStamped,
                "/object/detections",
                qos_profile_system_default,
            )
        else:
            raise RuntimeError(f"Unknown vision type, got {self.vision_type}")
        self.tf_broadcaster = StaticTransformBroadcaster(self._node)
        self.open_gripper()
        rclpy.spin_until_future_complete(
            self.trajectory_publisher, self.trajectory_publisher.future_init_done
        )

        self.in_fer_link0_M_support_link = get_in_fer_link0_M_support_link()

    def set_source_bin_pose(self, pose):
        self.source_bin_pose = pose

    def set_destination_bin_pose(self, pose):
        self.destination_bin_pose = pose

    def set_hardcoded_q0_start_and_above_source_bin(self):
        # self.q0_start = [
        #     -0.3619834760502907,
        #     -1.3575006398318104,
        #     0.969610481368033,
        #     -2.6028532848927295,
        #     0.2040785081450368,
        #     1.9436352693107668,
        #     0.6423896937386857,
        #     0.0,
        #     0.0,
        # ]

        # Prague config
        self.q0_start = [
            -0.07989926975547221,
            0.16054953411490003,
            -0.4331556367121411,
            -1.6853466114741846,
            0.040477269951448534,
            1.866718797171213,
            0.3947644127864415,
            0.0,
            0.0,
        ]

        # these q0_2 and q0_3 are poses that we used where we had good estimation from happypose
        # as the robot was able to see multiple faces of the objects.
        self.q0_2 = [
            0.23478670612576777,
            0.6708581179127536,
            -0.24280409489251267,
            -1.230202247770209,
            0.16490062651369303,
            1.3221025101878985,
            0.8068517371820269,
            0.03894394636154175,
            0.03894394636154175,
        ]
        self.q0_3 = [
            0.30284322343977793,
            -0.17057967255660944,
            -0.19719527408742066,
            -2.146916439625255,
            -0.12145666446852621,
            1.8278849533134025,
            0.9324160086682614,
            0.038980063050985336,
            0.038980063050985336,
            0.008928003512008873,
        ]

        # this configuration is used as an intermediate configuration above the box, we use it
        #  because it's helps hpp find better planning when doing the picking of the objects in the box.
        self.q_above_source_bin = [
            -0.37749851551808805,
            -0.24527851252273686,
            0.37860498360790074,
            -2.390846227478563,
            0.07986644285218328,
            2.1525887422066345,
            0.6495647583792291,
            0.03897743672132492,
            0.03897743672132492,
        ]

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
        if use_hardcoded_poses:
            # TEMP fix: just hardcode pose from happypose
            obj_start_pose = get_hardcoded_initial_object_pose(object_name)
        else:
            # REAL setup, TODO: fix communication error when happy pose is running
            print("waiting for obj pose")
            if self.vision_type in ["simulate_happypose", "happypose"]:
                object_detections = self.vision_client.wait_for_future()
                obj_start_pose = get_most_confident_object_pose(
                    object_detections, object_name, dataset_name=self.dataset_name
                )
            elif self.vision_type in ["simulate_apriltag_det", "apriltag_det"]:
                apriltag_detections: PoseStamped = self.vision_client.wait_for_future()
                obj_start_pose = [
                    apriltag_detections.header.frame_id,
                    pin.SE3ToXYZQUAT(pose_msg_to_se3(apriltag_detections.pose)),
                ]
            else:
                raise RuntimeError(f"Unknown vision type, got {self.vision_type}")
            print("got obj pose")

            obj_start_pose[0] = "panda/" + obj_start_pose[0]

        if obj_start_pose[1] is None:
            raise ValueError(f"No {object_name} object detected")
        obj_goal_pose = get_hardcoded_final_object_pose(object_name=object_name)
        return obj_start_pose, obj_goal_pose

    def open_gripper(self):
        """Open the gripper."""
        self.franka_gripper_cient.send_goal(position=0.039, max_effort=10.0)
        # TODO: change it to something normal
        # time.sleep(0.05)

    def close_gripper(self):
        """Close the gripper."""
        if self.is_simulation:
            self.franka_gripper_cient.send_goal(
                position=0.04, max_effort=self.param.max_holding_force
            )
            # TODO: change it to something normal
            time.sleep(0.05)
        else:
            self.franka_gripper_cient.grasp()
            # TODO: change it to something normal
            # time.sleep(2.0)

    def add_trajectory_to_publish(
        self, path_vector, visual_servoing_time_range=None
    ) -> None:
        """Transform hpp path into a trajectory to publish with possibility to use visual servoing."""
        q_array, dq_array, ddq_array = get_q_dq_ddq_arrays_from_path(
            path_vector, dt=self.dt
        )
        # TODO: get this from OCP params somehow
        horizon_size = 40
        multiplier = 1
        # add complete horizon at the end of the trajectory of the last point
        q_array += [q_array[-1]] * multiplier * horizon_size  # OCP horizon
        dq_array += [dq_array[-1]] * multiplier * horizon_size  # OCP horizon
        ddq_array += [ddq_array[-1]] * multiplier * horizon_size  # OCP horizon

        # find trajectory points idx range where to apply visual servoing
        # The parameter is the trajectory name in trajectory_weights_params.yaml
        if visual_servoing_time_range is None:
            visual_servoing_idx_range = [0, 0]
        else:
            visual_servoing_idx_range = [
                int(t / self.dt) for t in visual_servoing_time_range
            ]
            if visual_servoing_time_range[1] == path_vector.length():
                visual_servoing_idx_range[1] += multiplier * horizon_size

        # convert arrays in list of trajectory points
        trajectory = (
            self.trajectory_publisher.trajectory.build_trajectory_from_q_dq_ddq_arrays(
                q_array, dq_array, ddq_array
            )
        )

        # pass the trajectory to the trajectory publisher
        if self.trajectory_publisher.params.trajectory_name == "generic_trajectory":
            self.trajectory_publisher.add_trajectory(trajectory)
        elif (
            self.trajectory_publisher.params.trajectory_name
            == "generic_visual_servoing_trajectory"
        ):
            # kz: this assumes start_obj_pose is in a particular frame (support link)
            in_support_link_M_object = pin.XYZQUATToSE3(
                self.hpp_client.start_obj_pose.copy()
            )
            in_fer_link0_M_object = (
                self.in_fer_link0_M_support_link * in_support_link_M_object
            )
            print(f"starting visual servoing for the pose {in_fer_link0_M_object}...")

            self.trajectory_publisher.add_visual_servoing_trajectory(
                trajectory,
                visual_servoing_idx_range,
                pin.SE3ToXYZQUAT(in_fer_link0_M_object),
            )

    def go_to(
        self,
        desired_configuration: T.List[float],
        enable_visualization_in_gepetto_gui: bool = True,
    ) -> None:
        """Publish an hpp trajectory to go to desired configuration"""
        self.hpp_client = HPPInterface(
            object_name=self.default_object_name,
            dataset_name=self.dataset_name,
            use_spline_gradient_based_opt=False,
            source_bin_pose=self.source_bin_pose,
            destination_bin_pose=self.destination_bin_pose,
        )
        current_robot_state = self.state_client.wait_for_future()
        traj = self.hpp_client.plan_free_motion(
            list(current_robot_state.position),
            desired_configuration,
            consider_object=False,
        )
        if enable_visualization_in_gepetto_gui:
            self.v = self.hpp_client.vf.createViewer()
            self.v(self.hpp_client.q_init)
            # input("Trajectory computed. Ready to move. Press Enter to start motion...")

        self.add_trajectory_to_publish(traj)
        rclpy.spin_until_future_complete(
            self.trajectory_publisher, self.trajectory_publisher.future_trajectory_done
        )

    def publish_transform_in_tf(
        self, parent_frame, child_frame, transform: pin.SE3, stamp
    ) -> None:
        """Publish desired static transform, mainly used to decide what object to track."""
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform = se3_to_transform_msg(transform)

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def pick_and_place(
        self,
        object_name: str,
        use_hardcoded_poses: bool = False,
        enable_visualization_in_gepetto_gui: bool = True,
    ):
        """Plan with hpp a pick and place path and publish it."""
        # current_robot_state = self.state_client.wait_for_future()
        # q_init = list(current_robot_state.position)

        self.hpp_client = HPPInterface(
            object_name=object_name,
            dataset_name=self.dataset_name,
            use_spline_gradient_based_opt=False,
            source_bin_pose=self.source_bin_pose,
            destination_bin_pose=self.destination_bin_pose,
        )
        self.publish_transform_in_tf(
            parent_frame=map_object_id(object_name, dataset=self.dataset_name),
            child_frame="current_object",
            transform=pin.SE3.Identity(),
            stamp=self._node.get_clock().now().to_msg(),
        )

        self.object_to_grasp_name = object_name

        grasped = False
        while not grasped:
            current_robot_state = self.state_client.wait_for_future()
            q_init = list(current_robot_state.position)
            start_obj_pose, goal_obj_pose = self.get_object_start_and_goal_pose(
                object_name, use_hardcoded_poses=use_hardcoded_poses
            )

            self.hpp_client.set_relative_start_obj_pose(
                start_obj_pose[1], q_init, start_obj_pose[0]
            )
            self.hpp_client.set_goal_obj_pose(goal_obj_pose[0], goal_obj_pose[1][:3])
            grasp_path, placing_path, freefly_path = (
                self.hpp_client.plan_pick_and_place(
                    q_init=list(current_robot_state.position),
                    # q_above_source_bin=self.q_above_source_bin,
                )
            )

            if enable_visualization_in_gepetto_gui:
                # self.v = self.hpp_client.vf.createViewer()
                self.hpp_client.v(self.hpp_client.q_init)
                # input("Trajectory computed. Ready to move. Press Enter to start motion...")

            self.open_gripper()
            self.open_gripper()
            time_pre_grasp = grasp_path.pathAtRank(
                grasp_path.numberPaths() - 1
            ).length()

            if (
                self.trajectory_publisher.params.visual_servoing_enabled
            ):  # with visual servoing setup in config file
                print("visual servoing enabled")
                self.add_trajectory_to_publish(
                    grasp_path,
                    visual_servoing_time_range=[
                        grasp_path.length() - time_pre_grasp,
                        grasp_path.length(),
                    ],
                )
            else:  # no visual servoing
                print("visual servoing disabled")
                self.add_trajectory_to_publish(
                    grasp_path,
                )
            rclpy.spin_until_future_complete(
                self.trajectory_publisher,
                self.trajectory_publisher.future_trajectory_done,
            )
            self.close_gripper()
            # time.sleep(0.5)

            # check if gripper is holding anything ()
            gripper_state = self.gripper_reader.wait_for_future()
            # Since both go 0 → 0.04, total opening is their sum
            opening = gripper_state.position[0] + gripper_state.position[1]
            if opening < self.min_opening_for_grasp:
                print("Failure to GRASP")
                # retry
                self.go_to(self.q0_start)  # TODO: infer this automatically
                rclpy.spin_until_future_complete(
                    self.trajectory_publisher,
                    self.trajectory_publisher.future_trajectory_done,
                )
                # time.sleep(2.)
            else:
                print(
                    f"GRASPED, {opening}: {gripper_state.position[0]}. {gripper_state.position[1]}"
                )
                grasped = True

        if self.vision_type in ["simulate_apriltag_det", "apriltag_det"]:
            apriltag_detections: PoseStamped = self.vision_client.wait_for_future()

            self.publish_transform_in_tf(
                parent_frame=apriltag_detections.header.frame_id,
                child_frame="current_object",
                transform=pose_msg_to_se3(apriltag_detections.pose),
                stamp=self._node.get_clock().now().to_msg(),
            )

        if placing_path is not None:
            # TODO: check automatically

            time_pre_grasp = placing_path.pathAtRank(0).length()
            self.add_trajectory_to_publish(placing_path)
            rclpy.spin_until_future_complete(
                self.trajectory_publisher,
                self.trajectory_publisher.future_trajectory_done,
            )
            self.open_gripper()
            self.add_trajectory_to_publish(freefly_path)
            rclpy.spin_until_future_complete(
                self.trajectory_publisher,
                self.trajectory_publisher.future_trajectory_done,
            )

    def calibrate(
        self,
        box_handles: list[str],
        enable_visualization_in_gepetto_gui: bool = True,
        n_samples: int = 1000,
    ) -> None:
        """
        Publish a plan made by hpp to setup the place of the boxes in the scene.
           You have to chose among handles of the boxes and the robot will pass by them
           to correct their position.
        """
        current_robot_state = self.state_client.wait_for_future()
        q_init = list(current_robot_state.position)

        self.hpp_client = HPPInterface(
            object_name="obj_26",
            use_spline_gradient_based_opt=False,
            arm_id=self.arm_id,
        )

        paths = self.hpp_client.plan_calib_motion(
            q_init,
            box_handles,
            n_samples=n_samples,
        )

        if enable_visualization_in_gepetto_gui:
            self.v = self.hpp_client.vf.createViewer()
            input("Trajectory computed. Ready to move. Press Enter to start motion...")

        self.open_gripper()
        self.open_gripper()
        for path in paths:
            if path is None:
                input("Press Enter to close the gripper...")
                self.close_gripper()
                input("Press Enter to open the gripper and continue...")
                self.open_gripper()
            else:
                self.add_trajectory_to_publish(path)
                rclpy.spin_until_future_complete(
                    self.trajectory_publisher,
                    self.trajectory_publisher.future_trajectory_done,
                )

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