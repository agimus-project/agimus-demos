from copy import deepcopy
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Pose


class TargetClient(Node):
    def __init__(self):
        super().__init__('target_client')

        # Joint states subscriber
        self.subscription = self.create_subscription(
            Pose,
            '/target_object',
            self.callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )
        self.current_object_pose = None
        self.future = Future()

    def callback(self, msg):
        """Callback function to update the current joint state."""
        self.current_state = msg
        # Complete the future if it was waiting.
        if not self.future.done():
            self.future.set_result(msg)
            self.future = Future()  # Reset future for next call

    def wait_for_new_target_pose(self):
        """Blocks until a new joint state is received."""
        rclpy.spin_until_future_complete(self, self.future)
        return deepcopy(self.future.result())
