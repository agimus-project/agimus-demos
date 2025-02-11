
from rclpy.node import Node
from agimus_msgs.msg import MpcInput
import numpy as np
from std_msgs.msg import String
import pinocchio as pin
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose
import time


class TrajectoryPublisher(object):
    def __init__(self, node: Node):
        self._node = node
        self.pin_model = None
        self.pin_data = None
        self.ee_frame_id = None
        self.ee_frame_name = "fer_joint7"

        self._publisher = self._node.create_publisher(MpcInput, "/mpc_input", 10)
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscriber_robot_description_ = self._node.create_subscription(
            String,
            "/robot_description",
            self.robot_description_callback,
            qos_profile=qos_profile,
        )

    def robot_description_callback(self, msg: String) -> None:
        """Create the models of the robot from the urdf string."""
        print("Robot description callback is called.")
        self.pin_model = pin.buildModelFromXML(msg.data)
        self.pin_data = self.pin_model.createData()
        self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame_name)
        self._node.get_logger().warn(
            f"Model loaded, pin_model.nq = {self.pin_model.nq}"
        )

    def publish(self, trajectory: list):
        """Publishes an MpcInput message with provided data."""

        while self.pin_model is None:
            self._node.get_logger().warn("Robot model is not loaded...")
            time.sleep(0.5)

        self._node.get_logger().warn(f"Publishing traj of len {len(trajectory)}")
        for point in trajectory:
            q = np.concatenate([point.robot_configuration, np.zeros(2)])
            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            pin.updateFramePlacement(self.pin_model, self.pin_data, self.ee_frame_id)

            ee_pose = self.pin_data.oMf[self.ee_frame_id]
            xyz_quatxyzw = pin.SE3ToXYZQUAT(ee_pose)

            # print(q)
            u = pin.computeGeneralizedGravity(self.pin_model, self.pin_data, q)

            msg = MpcInput()
            msg.w_q = [1.0] * 7
            msg.w_qdot = [1e-2] * 7
            msg.w_qddot = [1e-6] * 7
            msg.w_robot_effort = [1e-4] * 7
            msg.w_pose = [1.0] * 6

            msg.q = list(point.robot_configuration)
            msg.qdot = list(point.robot_velocity)
            msg.qddot = list(point.robot_acceleration)

            msg.robot_effort = list(u[: len(msg.qddot)])

            pose = Pose()
            pose.position.x = xyz_quatxyzw[0]
            pose.position.y = xyz_quatxyzw[1]
            pose.position.z = xyz_quatxyzw[2]
            pose.orientation.x = xyz_quatxyzw[3]
            pose.orientation.y = xyz_quatxyzw[4]
            pose.orientation.z = xyz_quatxyzw[5]
            pose.orientation.w = xyz_quatxyzw[6]
            msg.pose = pose
            msg.ee_frame_name = self.ee_frame_name

            self._publisher.publish(msg)
            time.sleep(0.01)  # TODO how to do?
            # self._node.get_logger().info(f'Published MpcInput: q={msg.q}, qdot={msg.qdot}, qddot={msg.qddot}, effort={msg.robot_effort}')
