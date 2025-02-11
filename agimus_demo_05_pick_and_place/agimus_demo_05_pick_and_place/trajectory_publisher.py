from rclpy.node import Node
from agimus_msgs.msg import MpcInput
import numpy as np
from std_msgs.msg import String
import pinocchio as pin
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.ee_frame_name = "fer_joint7"
        self.publisher_ = self.create_publisher(MpcInput, "/mpc_input", 10)
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscriber_robot_description_ = self.create_subscription(
            String,
            "/robot_description",
            self.robot_description_callback,
            qos_profile=qos_profile,
        )

    def robot_description_callback(self, msg: String) -> None:
        """Create the models of the robot from the urdf string."""
        self.robot_description_msg = msg

    def load_models(self):
        """Callback to get robot description and store to object"""
        self.pin_model = pin.buildModelFromXML(self.robot_description_msg.data)
        self.pin_data = self.pin_model.createData()
        self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame_name)
        self.q = self.q0.copy()
        self.dq = np.zeros_like(self.q)
        self.ddq = np.zeros_like(self.q)
        self.get_logger().warn(f"Model loaded, pin_model.nq = {self.pin_model.nq}")


    def publish(self, trajectory: list):
        """Publishes an MpcInput message with provided data."""
        for point in trajectory:
            ee_pose = self.pin_data.oMf[self.ee_frame_id]
            xyz_quatxyzw = pin.SE3ToXYZQUAT(ee_pose)

            u = pin.computeGeneralizedGravity(self.pin_model, self.pin_data, self.q0)

            msg = MpcInput()
            msg.q = point.robot_configuration
            msg.w_q = [1.0] * len(msg.q)
            msg.qdot = point.robot_velocity
            msg.w_qdot = [1e-2] * len(msg.qdot)
            msg.qddot = point.robot_acceleration
            msg.w_qddot = [1e-6] * len(msg.qddot)
            msg.robot_effort = u[:len(msg.qddot)]
            msg.w_robot_effort = np.ones(len(point.effort)).tolist()
            pose = Pose()
            pose.position.x = xyz_quatxyzw[0]
            pose.position.y = xyz_quatxyzw[1]
            pose.position.z = xyz_quatxyzw[2]
            pose.orientation.x = xyz_quatxyzw[3]
            pose.orientation.y = xyz_quatxyzw[4]
            pose.orientation.z = xyz_quatxyzw[5]
            pose.orientation.w = xyz_quatxyzw[6]
            msg.pose = pose
            msg.w_pose = [1.0] * 6  # Example weight for pose

            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Published MpcInput: q={point.q}, qdot={point.qdot}, qddot={point.qddot}, effort={point.effort}"
            )
