from agimus_controller_ros.ros_utils import pose_msg_to_se3, se3_to_transform_msg
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
        
        self._trajectory_publisher = SimpleTrajectoryPublisher()
        self.dt = self.trajectory_publisher.dt

        self.state_client = AsyncSubscriber(
            self._node,
            JointState,
            "/joint_states",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        
        
    def execute_square(self):
        current_robot_state = self.state_client.wait_for_future()
        trajectory = plan_trajectory(current_robot_state, self.dt)
        self.trajectory_publisher.add_trajectory(trajectory)
        rclpy.spin_until_future_complete(
            self.trajectory_publisher, self.trajectory_publisher.future_trajectory_done
        )
