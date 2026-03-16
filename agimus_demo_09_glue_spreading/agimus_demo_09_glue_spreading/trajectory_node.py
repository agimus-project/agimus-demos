import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import pinocchio as pin
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


from agimus_demo_09_glue_spreading.mpcTrajectoryUtils import (
    PatternGenerator,
    SplineGenerator,
)
import yaml
from pathlib import Path


class TrajectoryMPC(Node):
    REF_FRAME = "world"
    TARGET_FRAME = "fr3_hand_tcp"
    TRAJ_TYPE = "zigzag"

    def __init__(self):
        super().__init__("trajectory_MPC_node")
        self.get_logger().info("Trajectory MPC node has been started.")
        self.declare_parameter(
            "plate_param",
            "/home/gepetto/ros2_ws/src/agimus-demos/agimus_demo_09_glue_spreading/config/plate.yaml",
        )
        # Load plate configuration
        config_path = Path(
            self.get_parameter("plate_param").get_parameter_value().string_value
        )
        with open(config_path, "r") as file:
            self.plate_config = yaml.safe_load(file)["plate"]
        self.plate_width = self.plate_config["glue_surface"]["width"]
        self.plate_length = self.plate_config["glue_surface"]["length"]
        self.plate_height = self.plate_config["glue_surface"]["height"]
        self.plate_center = self.plate_config["glue_surface"]["xyz"]
        self.plate_name_link = self.plate_config["name"] + "_base_link"
        self.glue_stride = self.plate_config["glue_surface"]["stride"]
        # Generate pattern waypoints
        object = {
            "width": self.plate_width,
            "length": self.plate_length,
            "height": self.plate_height,
            "center": self.plate_center,
        }

        self.pattern_generator = PatternGenerator(object)
        self.waypoints = self.pattern_generator.generate_pattern(
            "zigzag_curve", stride=self.glue_stride
        )
        self.original_waypoints = np.copy(self.waypoints)

        # TF2 setup
        self.tf_object_buffer = Buffer()
        self.tf_object_listener = TransformListener(
            self.tf_object_buffer, self, spin_thread=True
        )
        self.last_tf = [None, None]  # To store the last transforms
        self.tf = [None, None]  # To store the latest transforms

        # Publisher setup
        self.traj_marker_publisher = self.create_publisher(
            Marker, "trajectory_marker", 10
        )

        # self.traj_sender=self.create_publisher(Traj, 'mpc_trajectory', 10)
        # self.traj_timer = self.create_timer(0.1, self.send_trajectory)

        # Wait for TF availability
        self.wait_for_transform(self.REF_FRAME, self.plate_name_link)
        self.wait_for_transform(self.REF_FRAME, self.TARGET_FRAME)

        # Setup Spline Generator
        # Get the target frame transform
        trans, quat = self.get_tf(self.REF_FRAME, self.TARGET_FRAME)
        start_pos = np.array([trans.x, trans.y, trans.z])
        quat_pin = pin.Quaternion(np.array([quat.x, quat.y, quat.z, quat.w]))
        rot_matrix = quat_pin.toRotationMatrix()
        start_ori = pin.utils.matrixToRpy(rot_matrix)
        self.spline = SplineGenerator(start_pos, start_ori, waypoints=self.waypoints)

        # Setup timer to update trajectory
        self.update_transform_timer = self.create_timer(0.2, self.update_plate_tf)
        self.trajectory_timer = self.create_timer(0.5, self.transform_traj)
        self.timer_marker_traj = self.create_timer(0.5, self.display_trajectory_rviz)

        self.get_logger().info("Trajectory MPC node initialization complete.")

    def wait_for_transform(self, parent_frame, child_frame):
        while not self.tf_object_buffer.can_transform(
            parent_frame,
            child_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        ):
            self.get_logger().info(
                f"Waiting for TF between {parent_frame} and {child_frame}..."
            )
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("TF between base_link and object_frame is available.")

    def get_tf(self, parent_frame, child_frame):
        try:
            if not self.tf_object_buffer.can_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.25),
            ):
                self.get_logger().warn(
                    f"TF entre {parent_frame} et {child_frame} indisponible."
                )
                return
            trans: TransformStamped = self.tf_object_buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            return [
                trans.transform.translation,
                trans.transform.rotation,
            ]  # Store translation and rotation
        except Exception as e:
            self.get_logger().warn(f"Error getting TF: {e}")
            return [None, None]

    def update_plate_tf(self):
        self.tf[0], self.tf[1] = self.get_tf(self.REF_FRAME, self.plate_name_link)

    def transform_traj(self):
        try:
            trans, quat = self.tf[0], self.tf[1]
            if trans is None or quat is None:
                self.get_logger().error("TF data is incomplete.")
                return

            # First TF: just store
            if self.last_tf == [None, None]:
                self.last_tf = [trans, quat]
                self.spline.waypoints = np.copy(self.original_waypoints)
                self.spline.transform(trans, quat)
                return

            # Compare TF values properly
            if self.tf_changed(trans, quat, self.last_tf[0], self.last_tf[1]):
                # Reset BEFORE applying transformation
                self.spline.waypoints = np.copy(self.original_waypoints)
                self.spline.transform(trans, quat)
                self.last_tf = [trans, quat]

        except Exception as e:
            self.get_logger().error(f"Failed to update waypoints: {e}")

    def tf_changed(self, t1, q1, t2, q2):
        return (
            abs(t1.x - t2.x) > 1e-6
            or abs(t1.y - t2.y) > 1e-6
            or abs(t1.z - t2.z) > 1e-6
            or abs(q1.x - q2.x) > 1e-6
            or abs(q1.y - q2.y) > 1e-6
            or abs(q1.z - q2.z) > 1e-6
            or abs(q1.w - q2.w) > 1e-6
        )

    def display_trajectory_rviz(self):
        """
        Affiche la trajectoire dans RViz en utilisant des markers de type LINE_STRIP.
        """
        if not self.spline.start_traj:
            self.get_logger().warn("Aucune trajectoire à afficher.")
            return

        marker = Marker()
        marker.header.frame_id = (
            self.REF_FRAME
        )  # ou "map", "base_link", selon ton robot
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        # Ligne continue
        marker.scale.x = 0.01  # épaisseur de la ligne (en mètres)

        # Couleur (RGBA)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # vert opaque

        # Ajout des points de la trajectoire
        t = 0
        while t < self.spline.t_total:
            xyz = self.spline.get_interpolated_pose(t)
            p = Point()
            p.x, p.y, p.z = xyz[0], xyz[1], xyz[2]
            marker.points.append(p)
            t = t + 0.02
        self.traj_marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
