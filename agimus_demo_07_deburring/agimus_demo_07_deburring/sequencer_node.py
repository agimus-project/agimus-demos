import time

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterException
from rclpy.parameter import Parameter

from rcl_interfaces.srv import SetParametersAtomically

from std_msgs.msg import String


class SequencerNode(Node):
    def __init__(self):
        super().__init__("sequencer_node")

        self._handle_publisher = self.create_publisher(String, "/target_handle", 10)

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

    def set_handle_name(self, handle: str) -> None:
        self._handle_publisher.publish(String(data=handle))
        rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    weighted_handles = [
        ([0.000009, 0.000009, 0.000009], "hole_left_41"),
        ([0.000009, 0.000009, 0.000009], "hole_left_01"),
        ([0.000009, 0.000009, 0.000009], "hole_left_04"),
        ([0.000002, 0.000002, 0.000002], "hole_back_left_4"),
        ([0.000002, 0.000002, 0.000002], "hole_back_right_2"),
        ([0.000009, 0.000009, 0.000009], "hole_left_inside_33"),
        ([0.0000015, 0.0000015, 0.0000015], "hole_back_left_8"),
        # Metal
        # ([0.000009, 0.000009, 0.000009], "hole_left_31"),
        # ([0.000009, 0.000009, 0.000009], "hole_left_16"),
        # ([0.000009, 0.000009, 0.000009], "hole_left_inside_43"),
        # ([0.000009, 0.000009, 0.000009], "hole_left_12"),
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
            sequencer_node.set_handle_name(handle_name)

        print("Sequencing done, stopping the node")

        sequencer_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
