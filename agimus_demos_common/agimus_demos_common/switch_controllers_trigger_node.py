import rclpy
from rclpy.node import Node
import subprocess


class SwitchControllersTriggerNode(Node):
    def __init__(self):
        super().__init__("switch_controllers_trigger")
        self.get_logger().info("Press Enter to switch controllers...")
        self.wait_for_user_input()

    def wait_for_user_input(self):
        input("Press Enter to activate controllers...")
        self.switch_controllers()

    def switch_controllers(self):
        try:
            subprocess.run(
                [
                    "ros2",
                    "control",
                    "switch_controllers",
                    "--deactivate",
                    "arm_right_controller",
                    "--activate",
                    "arm_right_1_joint_inertia_aping_controller",
                    "linear_feedback_cshaping_controller",
                    "arm_right_2_joint_inertia_shaping_controller",
                    "arm_right_3_joint_inertia_shaping_controller",
                    "arm_right_4_joint_inertia_shaping_controller",
                    "arm_right_5_joint_inertia_shaping_controller",
                    "arm_right_6_joint_inertia_shaping_controller",
                    "arm_right_7_joint_inertia_shontroller",
                    "joint_state_estimator",
                ],
                check=True,
            )
            self.get_logger().info("Controllers switched successfully.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to switch controllers: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SwitchControllersTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
