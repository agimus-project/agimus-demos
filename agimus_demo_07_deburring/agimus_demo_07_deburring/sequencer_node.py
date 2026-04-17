import sys
import time
from tqdm import tqdm
from typing import Any

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ParameterException
from rclpy.parameter import Parameter

from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger

from action_msgs.msg import GoalStatus
from agimus_msgs.action import DeburringPlanner

try:
    from bigpose_msgs.srv import GetTransformStamped

    with_bigpose_msgs = True
except ImportError:
    with_bigpose_msgs = False


class SequencerNode(Node):
    def __init__(self, use_vision: bool = False):
        super().__init__("sequencer_node")

        # Actions

        self._deburring_action_client = ActionClient(
            self, DeburringPlanner, "/deburring_path_planner_node/plan_deburring"
        )

        # Service clients

        self._set_param_cli = self.create_client(
            SetParametersAtomically,
            "/deburring_path_planner_node/set_parameters_atomically",
        )

        self._detect_pylone_pose_cli = None
        self._refine_pylone_pose_cli = None
        if use_vision:
            self._detect_pylone_pose_cli = self.create_client(
                Trigger,
                "/bigpose_ros/detect",
            )
            self._refine_pylone_pose_cli = self.create_client(
                GetTransformStamped,
                "/bigpose_ros/refine",
            )

        self._progress_bar: tqdm | None = None
        self._last_handle_name: str | None = None
        self._is_planning = False
        self._last_progress = 0.0

        self.get_logger().info("Node started.")

    def _call_service_client(
        self, service_client: Client, request: Any, timeout: float = 5.0
    ) -> Any:
        """Waits for service server and calls it with a given request and spins the
            node to finish.

        Args:
            service_client (Client): Service client used for waiting.

        Raises:
            RuntimeError: Service server is not reachable.
        """
        start = time.time()
        ready = False
        while time.time() - start < timeout and not ready:
            ready = service_client.service_is_ready()
            if not ready:
                self.get_logger().debug(
                    f"Waiting for service {service_client.srv_name}...",
                    throttle_duration_sec=1.0,
                )
            time.sleep(0.1)

        if not ready:
            raise RuntimeError(f"Service {service_client.srv_name} is not ready!")

        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def refine_pylone_pose(self, retires: int = 5) -> None:
        """Refines pose of the pylone.

        Args:
            retires (int, optional): How many times refinement can be retried. Defaults to 5.

        Raises:
            RuntimeError: Service server is not reachable.
        """
        for _ in range(retires):
            res = self._call_service_client(
                self._refine_pylone_pose_cli, GetTransformStamped.Request()
            )
            if res.success:
                break
            self.get_logger().warn(
                f"Failed to refine pylone pose. Reason: '{res.message}'."
            )

    def detect_pylone_pose(self) -> None:
        """Detects pose of the pylne

        Raises:
            RuntimeError: Service server is not reachable.
        """
        self._call_service_client(self._detect_pylone_pose_cli, Trigger.Request())

    def set_weights_value(self, value: list[float]) -> None:
        """Sets parameter `weights.deburring_motion.w_desired_force` of external
        node.

        Args:
            value (list[float]): 3 element list containing weights to be set for deburring force.

        Raises:
            RuntimeError: Service server is not reachable.
        """
        request = SetParametersAtomically.Request(
            parameters=[
                Parameter(
                    "weights.deburring_motion.w_desired_force",
                    Parameter.Type.DOUBLE_ARRAY,
                    value,
                ).to_parameter_msg()
            ]
        )
        self._call_service_client(self._set_param_cli, request)

    def plan_to_handle(self, handle: str, do_plan: bool, do_insertion: bool) -> None:
        """Triggers planning to a handle action and waits for it's execution.

        Args:
            handle (str): Name of the handle to which robot has to move.
            do_insertion (bool): Whether to plan the motion or skip to insertion.
            do_insertion (bool): Whether to perform insertion and deburring motion
                during asked movement.

        Raises:
            RuntimeError: Failed to reach the action server.
            RuntimeError: Action was rejected.
            RuntimeError: Action execution failed.
        """
        goal_msg = DeburringPlanner.Goal(
            handle_name=handle,
            do_plan=do_plan,
            do_insertion=do_insertion,
        )
        self._deburring_action_client.wait_for_server()

        self._is_planning = True
        self._last_handle_name = handle
        self._last_progress = 0.0
        self._progress_bar = tqdm(
            total=1,
            desc=f"Planning to: {self._last_handle_name}",
            unit="%",
            leave=False,
        )

        future = self._deburring_action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )

        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(
                f"Action {self._deburring_action_client.action_name} does not respond!",
            )

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Action call was rejected!")

        self.get_logger().debug("Target goal accepted.")

        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)

        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            msg = "Action failed!"
            error_msg = future.result().result.error_msg
            if error_msg != "":
                msg += f" Error message: '{error_msg}'."
            raise RuntimeError(msg)

        self._progress_bar.close()

        self.get_logger().debug("Action succeed.")

    def _feedback_callback(self, feedback_msg: DeburringPlanner.Feedback) -> None:
        """Prints ongoing statistics about action. Prints planning status
            as well as later motion execution status.

        Args:
            feedback_msg (DeburringPlanner.Feedback): Received action feedback.
        """
        currently_planning = feedback_msg.feedback.planing
        if not currently_planning:
            if self._is_planning != currently_planning:
                self._progress_bar.close()
                self._progress_bar = tqdm(
                    total=100,
                    desc=f"Moving to handle: {self._last_handle_name}",
                    unit="%",
                    leave=False,
                )
            feedback = feedback_msg.feedback.progress
            delta = int((feedback - self._last_progress) * 100)
            self._progress_bar.update(delta)
            self._last_progress = feedback
        else:
            self._progress_bar.update(int(currently_planning))

        self._is_planning = currently_planning


def check_for_yes(prompt) -> bool:
    return input(prompt + " [Y/n]: ").lower() != "n"


def main(args=None) -> int:
    if args is None:
        args = sys.argv
    if "--help" in args or "-h" in args:
        print("Simple helper script automating selection of holes to sequence.")
        print(
            "  usage: ros2 run agimus_demo_07_deburring "
            + "sequencer_node <demo material type> <use active vision>"
        )
        print("  available materials: plastic, metal")
        print("  use activie vision is activated with `--use-vision`")
        print("  example: ros2 run agimus_demo_07_deburring sequencer_node metal true")
        return 0

    use_vision = "--use-vision" in args
    if (
        "--ros-args" not in args
        and len(args) != 2 + int(use_vision)
        or len({"metal", "plastic"} & set(args)) == 0
    ):
        print("Incorrect parameters! Use -h or --help for more information.")
        return 1

    rclpy.init(args=args)

    if not with_bigpose_msgs and use_vision:
        print("Could not find package bigpose_msgs. Exiting.")
        return 1

    if "metal" in args:
        # Order of values is: (force weights, handle name, whether or not refine the pose)
        weighted_handles = [
            ([0.000021, 0.000021, 0.000021], "hole_left_31", True),
            ([0.000030, 0.000030, 0.000030], "hole_left_16", True),
            ([0.000007, 0.000007, 0.000007], "hole_left_inside_43", True),
            ([0.000015, 0.000015, 0.000015], "hole_left_12", True),
        ]
    else:
        weighted_handles = [
            ([0.000009, 0.000009, 0.000009], "hole_left_41", True),
            ([0.000009, 0.000009, 0.000009], "hole_left_01", True),
            ([0.000009, 0.000009, 0.000009], "hole_left_04", True),
            ([0.000002, 0.000002, 0.000002], "hole_back_left_4", True),
            ([0.000002, 0.000002, 0.000002], "hole_back_right_2", False),
            ([0.000009, 0.000009, 0.000009], "hole_left_inside_33", True),
            ([0.0000015, 0.0000015, 0.0000015], "hole_back_left_8", True),
        ]

    try:
        sequencer_node = SequencerNode(use_vision)

        if use_vision and check_for_yes("Do you want to detect the object?"):
            sequencer_node.detect_pylone_pose()
            sequencer_node.refine_pylone_pose()

        for weight, handle_name, refine in weighted_handles:
            prompt = (
                f"Next handle is '{handle_name}'. Do you want to execute this hole?"
            )
            if not check_for_yes(prompt):
                continue
            try:
                sequencer_node.set_weights_value(weight)
                if refine and use_vision:
                    sequencer_node.plan_to_handle(handle_name, True, False)
                    if refine:
                        input("Press enter to refine.")
                        sequencer_node.refine_pylone_pose()
                        while not check_for_yes("Is refinement acceptable?"):
                            sequencer_node.refine_pylone_pose()
                    sequencer_node.plan_to_handle(handle_name, False, True)
                else:
                    sequencer_node.plan_to_handle(handle_name, True, True)
            except RuntimeError as err:
                print(f"Error. {str(err)}. Skipping the handle!")

        print("Sequencing done, stopping the node")

        sequencer_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()

    return 0


if __name__ == "__main__":
    main()
