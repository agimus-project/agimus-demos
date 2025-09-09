from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from agimus_demo_07_deburring.hpp_client import HPPInterface
from agimus_demo_07_deburring.async_trajectory_publisher import AsyncTrajectoryPublisher
from agimus_demos_common.orchestrator.async_subscriber import AsyncSubscriber
from agimus_controller_ros.simple_trajectory_publisher import GenericTrajectory

from agimus_demo_07_deburring.orchestrator_parameters import (
    orchestrator_parameters,
)  # noqa: E402


class Orchestrator:
    """Orchestrator of demo agimus_demo_07_deburring"""

    def __init__(self):
        self._node = Node("deburring_orchestrator")

        self.is_simulation = False

        try:
            self._param_listener = orchestrator_parameters.ParamListener(self._node)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self._node.get_logger().error(str(e))
            return

        self.trajectory_publisher = AsyncTrajectoryPublisher(
            self._node,
            self._params.mpc_input_topic_name,
            self._params.buffer_size_topic_name,
            self._params.update_frequency,
            self._params.buffer_size,
        )
        self._trajectory_generator = GenericTrajectory(
            self._params.ee_frame_name,
            self._params.w_q,
            self._params.w_qdot,
            self._params.w_qddot,
            self._params.w_robot_effort,
            self._params.w_pose,
            w_collision_avoidance=0.0,
        )

        self.state_client = AsyncSubscriber(
            self._node,
            JointState,
            "/joint_states",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.robot_description_client = AsyncSubscriber(
            self._node,
            String,
            "/robot_description",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.robot_srdf_description_client = AsyncSubscriber(
            self._node,
            String,
            "/robot_srdf_description",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.target_client = AsyncSubscriber(
            self._node,
            Pose,
            "/target_object",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=True
        )

        # self._urdf = self.robot_description_client.wait_for_future()
        # self._srdf = self.robot_srdf_description_client.wait_for_future()

        self._urdf = "/home/gepetto/ros2_ws/src/panda.urdf"
        self._srdf = "/home/gepetto/ros2_ws/src/panda.srdf"

        self._hpp_interface = HPPInterface(
            self._urdf,
            self._srdf,
            self._params.pylone_used_handles,
            self._params.gripper_name,
        )

    def create_viewer(self):
        self._hpp_interface.create_viewer()

    def update_initial_configuration(self):
        self._hpp_interface.update_initial_configuration()

    def solve(self):
        self._hpp_interface.solve()

    def play_path(self):
        self._hpp_interface.play_path()

    def perform_motion(self):
        for handle_name in self._params.pylone_used_handles:
            input("Press enter to continue")
            self._hpp_interface.generate_pregrasp(handle_name)
            self._hpp_interface.play_path()
            input("Press enter to continue")
            self._hpp_interface.generate_enter_hole(handle_name)
            self._hpp_interface.play_path()
            # input("Press enter to continue")
            # self._hpp_interface.generate_retract(handle_name)
            # self._hpp_interface.play_path()

    def update_environment(self):
        # joint_states = self.state_client.wait_for_future()
        # # Update joint states of the robot
        # for name, position in zip(joint_states.name, joint_states.position):
        #     self._hpp_interface.update_joint_state(name, position)
        # # Update pose of the pylone from ROS transforms
        # try:
        #     transform_msg = self._tf_buffer.lookup_transform(
        #         self._params.world_frame, self._params.pylone_frame, rclpy.time.Time()
        #     )
        #     pose = pin.SE3ToXYZQUAT(transform_msg_to_se3(transform_msg.transform))
        #     self._hpp_interface.update_pylone_pose(pose[:3], pose[3:])
        # except TransformException as ex:
        #     print(
        #         "Pylone pose was not updated! "
        #         f"Could not transform {self._params.world_frame} "
        #         f"to {self._params.pylone_frame}: {ex}."
        #     )
        q = [0.0, 0.1865, 0.0, -2.4, 0.0, 2.5915, 0.7863]
        for i, position in enumerate(q):
            self._hpp_interface.update_joint_state(f"fer_joint{i + 1}", position)
        self._hpp_interface.update_pylone_pose([0.5, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0])

    # def publish(self, path_vector):
    #     q_array, dq_array, ddq_array = get_q_dq_ddq_arrays_from_path(path_vector)
    #     self.trajectory_publisher.set_new_trajectory(
    #         self._trajectory_generator.build_trajectory_from_q_dq_ddq_arrays(
    #             q_array, dq_array, ddq_array
    #         )
    #     )
