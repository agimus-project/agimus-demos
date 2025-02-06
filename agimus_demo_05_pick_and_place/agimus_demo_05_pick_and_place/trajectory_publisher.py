from rclpy.node import Node
from agimus_msgs.msg import MpcInput
import numpy as np


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.publisher_ = self.create_publisher(MpcInput, "mpc_input_topic", 10)

    def publish(self, trajectory):
        """Publishes an MpcInput message with provided data."""
        for point in trajectory:
            msg = MpcInput()
            msg.q = point.q
            msg.w_q = np.ones(len(point.q)).tolist()
            msg.qdot = point.qdot
            msg.w_qdot = np.ones(len(point.qdot)).tolist()
            msg.qddot = point.qddot
            msg.w_qddot = np.ones(len(point.qddot)).tolist()
            msg.robot_effort = point.effort
            msg.w_robot_effort = np.ones(len(point.effort)).tolist()
            msg.pose = point.pose
            msg.w_pose = [1.0] * 6  # Example weight for pose

            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Published MpcInput: q={point.q}, qdot={point.qdot}, qddot={point.qddot}, effort={point.effort}"
            )
