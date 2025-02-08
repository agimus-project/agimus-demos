from copy import deepcopy
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState


class StateClient(Node):
    def __init__(self):
        super().__init__("state_client")

        # Joint states subscriber
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.current_state = None
        self.future = Future()

    def joint_state_callback(self, msg):
        """Callback function to update the current joint state."""
        self.current_state = msg

        # Complete the future if it was waiting
        if not self.future.done():
            self.future.set_result(msg)
            self.future = Future()  # Reset future for next call

    def wait_for_new_state(self):
        """Blocks until a new joint state is received."""
        rclpy.spin_until_future_complete(self, self.future)
        return deepcopy(self.future.result())
