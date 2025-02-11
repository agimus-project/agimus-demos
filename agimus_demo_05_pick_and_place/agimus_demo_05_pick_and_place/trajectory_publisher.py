from rclpy.node import Node
from agimus_msgs.msg import MpcInput
import numpy as np


class TrajectoryPublisher(object):
    def __init__(self, node: Node):
        self._node = node
        self._publisher = self._node.create_publisher(MpcInput, "mpc_input_topic", 10)

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

            self._publisher.publish(msg)
            self._node.get_logger().info(
                f"Published MpcInput: q={point.q}, "
                f"qdot={point.qdot}, qddot={point.qddot}, "
                f"effort={point.effort}"
            )
