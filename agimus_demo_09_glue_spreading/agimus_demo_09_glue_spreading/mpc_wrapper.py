from aligator_mpc import MPC, Params, PatternGenerator, TestTrajs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pathlib import Path

class AligatorMPC(Node):
    def __init__(self):
        super().__init__('aligator_mpc')
        self.get_logger().info('Aligator MPC starting...')
        self.declare_parameter('config',"clearly a wrong value!!")
        config_path = self.get_parameter('config').value

        self.get_logger().info(config_path)


        # config_path = Path("config/mpc.yaml")
        # parameters = Params(config_path)

        # Waypoints ================================================================
        # patternGen = PatternGenerator([0.5,0.5,0], (0.6,0,0.3))
        # positions = patternGen.generate_pattern('zigzag_curve',stride=0.05)

        # test_trajs = TestTrajs()
        # start = [-0.5, -0.5, 0]
        # end = [1, 2, 1]
        # positions = test_trajs.line(start, end)
        # # positions = test_trajs.sine(start_point=start,length=1,period=0.05,amplitude=0.1, dist_between_points=0.01, sine_axis="Y", ampl_axis="X")




        # mpc = MPC(positions, parameters)


        # rclpy.wait_for_message.wait_for_message(msg_type= JointState,
        #                        node= "/joint_state_publisher",
        #                        topic = "/franka/joint_states ",
        #                        )

        self.get_logger().info('End of Setup')

def main(args=None):
    rclpy.init(args=args)

    node = AligatorMPC()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

