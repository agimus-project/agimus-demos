import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ParameterException
from rclpy.parameter import Parameter
from rclpy.task import Future

from rcl_interfaces.srv import SetParametersAtomically

from agimus_msgs.action import DeburringPlanner


class SequencerNode(Node):
    def __init__(self):
        super().__init__("sequencer_node")

        self._deburring_action_client = ActionClient(
            self, DeburringPlanner, "/plan_deburring"
        )

        self._set_param_cli = self.create_client(
            SetParametersAtomically,
            "/deburring_path_planner_node/set_parameters_atomically",
        )

        self.get_logger().info("Node started.")

    def _set_params(self, parameters: list[Parameter], timeout: float = 5.0) -> None:
        start = time.time()
        ready = False
        while time.time() - start < timeout and not ready:
            ready = self._set_param_cli.service_is_ready()
            time.sleep(0.1)

        if not ready:
            raise RuntimeError(f"Service {self._set_param_cli.srv_name} is not ready!")

        req = SetParametersAtomically.Request(
            parameters=[param.to_parameter_msg() for param in parameters]
        )
        future = self._set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_weights_value(self, value: list[float]) -> None:
        self._set_params(
            [
                Parameter(
                    "weights.deburring_motion.w_desired_force",
                    Parameter.Type.DOUBLE_ARRAY,
                    value,
                )
            ]
        )

    def plan_to_handle(self, handle: str, do_insertion: bool) -> Future:
        goal_msg = DeburringPlanner.Goal(
            handle_name=handle,
            do_insertion=do_insertion,
        )
        self._deburring_action_client.wait_for_server()

        future = self._deburring_action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

        return future

    def _goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Target goal rejected. Reason: '{goal_handle.error_msgs}'"
            )
            return

        self.get_logger().info("Target goal accepted.")

        self._get_result_future = goal_handle.get_result_async()

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {format(feedback.progress)}")


def main(args=None) -> int:
    if args is None:
        args = sys.argv
    if "--help" in args or "-h" in args:
        print("Simple helper script automating selection of holes to sequence.")
        print(
            "  usage: ros2 run agimus_demo_07_deburring "
            + "sequencer_node <demo material type>"
        )
        print("  available materials: plastic, metal")
        print("  example: ros2 run agimus_demo_07_deburring sequencer_node metal")
        return 0

    if len(args) != 2 or len({"metal", "plastic"} & set(args)) == 0:
        print("Incorrect parameters! Use -h or --help for more information.")
        return 1

    rclpy.init(args=args)

    if "metal" in args:
        weighted_handles = [
            ([0.000009, 0.000009, 0.000009], "hole_left_31"),
            ([0.000007, 0.000007, 0.000007], "hole_left_16"),
            ([0.0000045, 0.0000045, 0.0000045], "hole_left_inside_43"),
            ([0.000007, 0.000007, 0.000007], "hole_left_12"),
        ]
    else:
        weighted_handles = [
            ([0.000009, 0.000009, 0.000009], "hole_left_41"),
            ([0.000009, 0.000009, 0.000009], "hole_left_01"),
            ([0.000009, 0.000009, 0.000009], "hole_left_04"),
            ([0.000002, 0.000002, 0.000002], "hole_back_left_4"),
            ([0.000002, 0.000002, 0.000002], "hole_back_right_2"),
            ([0.000009, 0.000009, 0.000009], "hole_left_inside_33"),
            ([0.0000015, 0.0000015, 0.0000015], "hole_back_left_8"),
        ]

    try:
        sequencer_node = SequencerNode()

        for weight, handle_name in weighted_handles:
            if (
                input(
                    f"Next handle is '{handle_name}'. Do you want to execute this hole? [Y/n]: "
                ).lower()
                == "n"
            ):
                continue
            sequencer_node.set_weights_value(weight)
            future = sequencer_node.plan_to_handle(handle_name, False)
            rclpy.spin_until_future_complete(sequencer_node, future)
            input("Please refine the pose")
            future = sequencer_node.plan_to_handle(handle_name, True)
            rclpy.spin_until_future_complete(sequencer_node, future)

        print("Sequencing done, stopping the node")

        sequencer_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()

    return 0


if __name__ == "__main__":
    main()
