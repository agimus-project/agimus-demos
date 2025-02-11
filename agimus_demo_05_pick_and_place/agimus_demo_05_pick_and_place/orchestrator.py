"""Implement the agimus_demo_05_pick_and_place Orchestrator"""

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from agimus_demo_05_pick_and_place.franka_gripper_client import FrankaGripperClient
# from agimus_demo_05_pick_and_place.hpp_client import HPPInterface
from agimus_demo_05_pick_and_place.script_hpp import HPPInterface, concatenatePaths, get_traj_points_from_path
from agimus_demo_05_pick_and_place.async_subscriber import AsyncSubscriber
from agimus_demo_05_pick_and_place.trajectory_publisher import TrajectoryPublisher


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
        object_name = "obj_23"
        robot_init = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4, 0.035, 0.035]

        self.hpp_client = HPPInterface(
            robot_configuration = robot_init,
            object_name=object_name,
            desired_location = [0.0, -0.1, 1.0]
        )
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

    def open_gripper(self):
        self.franka_gripper_cient.send_goal(
            position=0.04, max_effort=self.param.max_holding_force
        )

    def close_gripper(self):
        self.franka_gripper_cient.send_goal(position=0.0, max_effort=10.0)

    def go_to(self, desired_configuration):
        current_robot_state = self.state_client.wait_for_new_state()
        trajectory = self.hpp_client.plan(
            current_robot_state.position, desired_configuration
        )
        self.trajectory_publisher.publish(trajectory)

    def pick_and_place(self):
        current_robot_state = self.state_client.wait_for_future()
        q_init, grasp_path, placing_path, freefly_path = self.hpp_client.plan(current_robot_state.position)
        path_vector = concatenatePaths([grasp_path, placing_path, freefly_path])
        path_len = path_vector.length()
        configs = [path_vector.call(t)[0] for t in np.linspace(0, path_len, 100) ]
        # pickle.dump(configs, open("q_init.pkl", "wb"))
        # traj_publisher = TrajectoryPublisher(node=)
        traj1 = get_traj_points_from_path(grasp_path)
        traj2 = get_traj_points_from_path(placing_path)
        traj3 = get_traj_points_from_path(freefly_path)
        self.trajectory_publisher.publish(traj1)
        self.close_gripper()
        self.trajectory_publisher.publish(traj2)
        self.open_gripper()
        self.trajectory_publisher.publish(traj3)

    # def go_to_ee(self, target_ee):
    #     current_robot_state = self.state_client.wait_for_new_state()
    #     trajectory = self.hpp_client.plan_ee(current_robot_state.position, target_ee)
    #     self.trajectory_publisher.publish(trajectory)

    def go_to_parking_pose(self):
        self.go_to(self.param.parking_configuration)

    def go_to_destination_pose(self):
        self.go_to(self.param.destination_configuration)

    # def go_to_pre_grasp(self):
    #     new_target_pose = self.target_client.wait_for_new_target_pose()
    #     trajectory = self.go_to_ee(new_target_pose)
    #     self.trajectory_publisher(trajectory)
