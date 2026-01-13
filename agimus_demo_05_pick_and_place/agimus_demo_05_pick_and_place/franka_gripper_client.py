import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp


class FrankaGripperClient(object):
    def __init__(self, node: Node):
        self._node = node
        self._client = ActionClient(
            self._node, GripperCommand, "/fer_gripper/gripper_action"
        )
        self._action_client = ActionClient(self._node, Grasp, "/fer_gripper/grasp")

    def send_goal(self, position: float, max_effort: float):
        """Sends a goal to the GripperCommand action server."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self._node.get_logger().info("Waiting for Franka gripper action server...")
        self._client.wait_for_server()

        self._node.get_logger().info(
            f"Sending goal: position={position}, max_effort={max_effort}"
        )
        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def grasp(self, width: float = 0.0, speed: float = 0.04, force: int = 10.0):
        self._node.get_logger().info(
            "Waiting for Franka gripper action server to be available..."
        )
        self._action_client.wait_for_server()

        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        # goal_msg.epsilon.inner = 0.1
        goal_msg.epsilon.outer = 0.1

        self._node.get_logger().info("Sending goal to close the gripper...")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.fake_feedback_callback
        )
        # 1️⃣ Wait until goal is accepted
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self._node.get_logger().error("Grasp goal was rejected")
            return False

        self._node.get_logger().info("Grasp goal accepted, waiting for result...")

        # 2️⃣ Wait until gripper finishes closing
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)

        result = result_future.result().result

        self._node.get_logger().info(f"Grasp finished: success={result.success}")
        return result.success

    def goal_response_callback(self, future):
        """Handles the response when the goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info("Goal rejected.")
            return

        self._node.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handles the final result of the action."""
        result = future.result().result
        self._node.get_logger().info(
            f"Action completed. Reached position: {result.position}"
        )

    def feedback_callback(self, feedback_msg):
        """Handles feedback from the action server."""
        self._node.get_logger().info(
            f"Feedback: Position = {feedback_msg.feedback.position}"
        )

    def fake_feedback_callback(self, feedback_msg):
        """Handles feedback from the action server."""
        self._node.get_logger().info("Feedback: Position = ")
