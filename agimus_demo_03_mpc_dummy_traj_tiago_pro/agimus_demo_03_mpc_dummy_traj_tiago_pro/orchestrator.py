import time

from agimus_controller_ros.simple_trajectory_publisher import (
    SimpleTrajectoryPublisher,
)
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agimus_demo_03_mpc_dummy_traj_tiago_pro.async_subscriber import AsyncSubscriber
from agimus_demo_03_mpc_dummy_traj_tiago_pro.monolithe_hpp_square_motion import (
    plan_trajectory,
)


class Orchestrator(object):
    """Orchestrator of demo agimus_demo_03_mpc_dummy_traj_tiago_pro."""

    def __init__(self):
        self._node = Node("orchestrator")

        self._trajectory_reach = None
        self._trajectory = None
        self._trajectory_publisher = SimpleTrajectoryPublisher()
        self.dt = self._trajectory_publisher.dt

        self.state_client = AsyncSubscriber(
            self._node,
            JointState,
            "/joint_states",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def plan(self):
        if self._trajectory_reach is None and self._trajectory is None:
            current_robot_state = self.state_client.wait_for_future()
            q0 = [0, 0, 0, 0, 0, 0, 1] + current_robot_state.position.tolist()
            print(q0)
            self._trajectory_reach, self._trajectory = plan_trajectory(q0, self.dt)
            self.planned_once = True

    def execute_square(self):
        self.plan()
        self._trajectory_publisher.add_trajectory(self._trajectory_reach)
        self._trajectory_publisher.add_trajectory(self._trajectory)
        rclpy.spin_until_future_complete(
            self._trajectory_publisher,
            self._trajectory_publisher.future_trajectory_done,
        )

    def resend_trajectory(self):
        self._trajectory_publisher.add_trajectory(self._trajectory)
        rclpy.spin_until_future_complete(
            self._trajectory_publisher,
            self._trajectory_publisher.future_trajectory_done,
        )

    def continuously_send_trajectory(self):
        self.execute_square()
        while rclpy.ok():
            self.resend_trajectory()
            time.sleep(4.0)
