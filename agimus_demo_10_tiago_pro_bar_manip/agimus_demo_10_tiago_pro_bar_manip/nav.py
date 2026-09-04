#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path, Odometry

import tf2_ros
import tf2_geometry_msgs
import pinocchio as pin
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__("trajectory_follower")

        # ---- Paramètres ----
        self.declare_parameter("path_topic", "/trajectory")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/mobile_base_controller/odom")
        self.declare_parameter("traj_marker_topic", "/marker_traj")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("kx", 1.0)
        self.declare_parameter("ky", 3.0)
        self.declare_parameter("ktheta", 2.0)

        self.declare_parameter("v_nominal", 0.3)  # m/s
        self.declare_parameter("v_max", 0.6)  # m/s, norme max de (Vx, Vy)
        self.declare_parameter("w_max", 1.5)  # rad/s

        self.declare_parameter("lookahead_index", 2)  # points d'avance
        self.declare_parameter("goal_tolerance", 0.1)  # m

        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.traj_marker_topic = self.get_parameter("traj_marker_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.control_rate_hz = self.get_parameter("control_rate_hz").value

        self.kx = self.get_parameter("kx").value
        self.ky = self.get_parameter("ky").value
        self.ktheta = self.get_parameter("ktheta").value

        self.v_nominal = self.get_parameter("v_nominal").value
        self.v_max = self.get_parameter("v_max").value
        self.w_max = self.get_parameter("w_max").value

        self.lookahead_index = self.get_parameter("lookahead_index").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value

        # ---- State ----
        self.current_path = None  # (x, y, theta)
        self.current_index = 0
        self.base_pose = None
        self.base_twist = (0.0, 0.0, 0.0)
        self.base_pose_stamp = None
        self.cmd = Twist()
        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- ROS I/O ----
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/vel_cmd_marker", 10)
        self.traj_marker_pub = self.create_publisher(Marker, self.traj_marker_topic, 10)

        odom_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, odom_qos
        )

        period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(period, self.control_loop)
        self.timer_marker = self.create_timer(period, self.publish_cmd_marker)
        self.timer_traj_marker = self.create_timer(1.0, self.publish_traj_marker)

        self.get_logger().info(
            f"TrajectoryFollower (holonome) prêt : path='{self.path_topic}' -> "
            f"cmd_vel='{self.cmd_vel_topic}', TF {self.world_frame}->{self.robot_frame}"
        )

    def path_callback(self, msg: Path):
        # ---- Empty traj or too short----
        if len(msg.poses) < 2:
            if len(msg.poses) == 0:
                self.get_logger().warn("Empty traj received")
            else:
                self.get_logger().warn("Traj too short")
            self.current_path = None
            self.current_index = 0
            self.publish_twist(0.0, 0.0, 0.0)
            return

        # ---- Checking traj ref frame
        source_frame = msg.header.frame_id or self.world_frame
        transform = None
        if source_frame != self.world_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                self.current_path = None
                self.current_index = 0
                self.publish_twist(0.0, 0.0, 0.0)
                return

        pts = []
        for pose_stamped in msg.poses:
            pose = pose_stamped.pose
            if transform is not None:
                ps_in = PoseStamped()
                ps_in.header.frame_id = source_frame
                ps_in.header.stamp = msg.header.stamp
                ps_in.pose = pose
                ps_out = tf2_geometry_msgs.do_transform_pose(ps_in, transform)
                pose = ps_out.pose if hasattr(ps_out, "pose") else ps_out

            p = pose.position
            q = pose.orientation
            quat = pin.Quaternion(q.w, q.x, q.y, q.z)
            _, _, yaw = pin.rpy.matrixToRpy(quat.toRotationMatrix())
            pts.append((p.x, p.y, yaw))

        self.current_path = pts
        self.current_index = 0
        self.get_logger().info(
            f"Traj received : {len(pts)} points "
            f"(ref frame '{source_frame}' -> '{self.world_frame}')."
        )

        self.publish_traj_marker()

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quat = pin.Quaternion(q.w, q.x, q.y, q.z)
        _, _, theta = pin.rpy.matrixToRpy(quat.toRotationMatrix())
        self.base_pose = (p.x, p.y, theta)

        t = msg.twist.twist
        self.base_twist = (t.linear.x, t.linear.y, t.angular.z)

        self.base_pose_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

    def _predicted_base_pose(self):
        if self.base_pose is None:
            return None
        if self.base_pose_stamp is None:
            return self.base_pose

        now = self.get_clock().now()
        dt = (now - self.base_pose_stamp).nanoseconds * 1e-9
        dt = min(max(dt, 0.0), 0.2)

        x, y, theta = self.base_pose
        vx_b, vy_b, w = self.base_twist

        # Vel in robot frame
        dx = (vx_b * math.cos(theta) - vy_b * math.sin(theta)) * dt
        dy = (vx_b * math.sin(theta) + vy_b * math.cos(theta)) * dt
        dtheta = w * dt

        return x + dx, y + dy, normalize_angle(theta + dtheta)

    def find_target(self, x, y):
        path = self.current_path
        n = len(path)

        best_i = self.current_index
        best_d = math.hypot(path[best_i][0] - x, path[best_i][1] - y)
        for i in range(self.current_index, min(self.current_index + 20, n)):
            d = math.hypot(path[i][0] - x, path[i][1] - y)
            if d < best_d:
                best_d = d
                best_i = i
        self.current_index = best_i

        target_i = min(best_i + self.lookahead_index, n - 1)
        xd, yd, thetad = path[target_i]

        i0 = max(target_i - 1, 0)
        i1 = min(target_i + 1, n - 1)
        seg_len = math.hypot(path[i1][0] - path[i0][0], path[i1][1] - path[i0][1])

        STATIONARY_EPS = 1e-3  # m
        if seg_len < STATIONARY_EPS:
            wd = 0.0
            vd = 0.0
        else:
            dtheta = normalize_angle(path[i1][2] - path[i0][2])
            dt = seg_len / max(self.v_nominal, 1e-3)
            wd = dtheta / dt
            vd = self.v_nominal

        is_goal = (target_i == n - 1) and (
            math.hypot(path[-1][0] - x, path[-1][1] - y) < self.goal_tolerance
        )
        return xd, yd, thetad, vd, wd, is_goal

    def control_loop(self):
        if self.current_path is None:
            self.publish_twist(0.0, 0.0, 0.0)
            return

        if self.base_pose is None:
            self.get_logger().warn(
                "No odometrie received yet on '%s'" % self.odom_topic,
                throttle_duration_sec=2.0,
            )
            return
        x, y, theta = self._predicted_base_pose()

        xd, yd, thetad, vd, wd, is_goal = self.find_target(x, y)

        if is_goal:
            self.publish_twist(0.0, 0.0, 0.0)
            return

        dx, dy = xd - x, yd - y
        ex = math.cos(theta) * dx + math.sin(theta) * dy
        ey = -math.sin(theta) * dx + math.cos(theta) * dy
        etheta = normalize_angle(thetad - theta)

        # ---- Command rules
        vx = vd * math.cos(etheta) + self.kx * ex
        vy = vd * math.sin(etheta) + self.ky * ey
        w = wd + self.ktheta * math.sin(etheta)

        speed = math.hypot(vx, vy)
        if speed > self.v_max and speed > 1e-6:
            scale = self.v_max / speed
            vx *= scale
            vy *= scale

        w = max(min(w, self.w_max), -self.w_max)

        self.publish_twist(vx, vy, w)

    def publish_twist(self, vx: float, vy: float, w: float):
        self.cmd.linear.x = vx
        self.cmd.linear.y = vy
        self.cmd.angular.z = w
        self.cmd_pub.publish(self.cmd)

    def publish_cmd_marker(self):
        if self.cmd is None or self.base_pose is None:
            return
        x, y, _ = self.base_pose

        marker = Marker()

        marker.header.frame_id = self.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "cmd_vel"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = [
            Point(x=0.0, y=0.0, z=0.1),
            Point(x=self.cmd.linear.x, y=self.cmd.linear.y, z=0.1),
        ]

        marker.scale.x = 0.03
        marker.scale.y = 0.08
        marker.scale.z = 0.12

        if self.cmd.linear.x >= 0.0:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.color.a = 1.0
        marker.lifetime.sec = 0

        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_traj_marker(self):
        if self.current_path is None:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "trajectory"
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.02

        marker.color.r = 0.1
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.points = [Point(x=px, y=py, z=0.0) for (px, py, _) in self.current_path]

        marker.lifetime.sec = 0

        self.traj_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
