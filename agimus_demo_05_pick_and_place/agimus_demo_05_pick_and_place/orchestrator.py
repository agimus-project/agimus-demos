"""Implement the agimus_demo_05_pick_and_place Orchestrator"""

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from agimus_demo_05_pick_and_place.franka_gripper_client import FrankaGripperClient
from agimus_demo_05_pick_and_place.hpp_client import HPPInterface
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
        self.hpp_client = HPPInterface()
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

    def go_to_ee(self, target_ee):
        current_robot_state = self.state_client.wait_for_new_state()
        trajectory = self.hpp_client.plan_ee(current_robot_state.position, target_ee)
        self.trajectory_publisher.publish(trajectory)

    def go_to_parking_pose(self):
        self.go_to(self.param.parking_configuration)

    def go_to_destination_pose(self):
        self.go_to(self.param.destination_configuration)

    def go_to_pre_grasp(self):
        new_target_pose = self.target_client.wait_for_new_target_pose()
        trajectory = self.go_to_ee(new_target_pose)
        self.trajectory_publisher(trajectory)
