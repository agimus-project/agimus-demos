"""
HPP Action Server for TIAGo Pro bimanual bar handling.

Provides a ROS2 action server for HPP planning requests.

Action: /hpp/plan_bar_handling (PlanBarGrasp)
    Goal:   gripper (string), handle (string)
    Result: success (bool), path_id (int), message (string)
    Feedback: status (string), elapsed_time (float)

"""

import os
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from test_agimus_type.action import PlanBarGrasp

from agimus_demo_10_tiago_pro_bar_manip.traj.hpp_traj import (
    HPPPathGenerator,
    BaseObject,
)


class HPPActionServer(Node):
    """
    ROS2 Action Server wrapping HPPPathGenerator.

    Subscribes:
        /joint_states
        /tf
    Action Server:
        /hpp/plan_bar_handling  (PlanBarGrasp)
    """

    def __init__(self):
        super().__init__("hpp_action_server")

        # ── Internal state ────────────────────────────────────────────────────
        self._joint_state: JointState | None = None
        self._odom: Odometry | None = None
        self._bar_pose: PoseStamped | None = None
        self._plate_pose: PoseStamped | None = None
        self._hpp_path = None
        self._planning = False

        # ── Callback group pour permettre planification + spin simultanés ─────
        self._cb_group = ReentrantCallbackGroup()

        # ── ROS subscribers ───────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._cb_joints,
            qos,
            callback_group=self._cb_group,
        )
        # self.create_subscription(
        #     Odometry,    "/mobile_base_controller/odom",         self._cb_odom,       qos,
        #     callback_group=self._cb_group
        # )
        self.create_timer(0.5, self._cb_object_pose, callback_group=self._cb_group)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._world_frame = "world"

        # ── Action server ─────────────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            PlanBarGrasp,
            "/orchestrator/plan_bar_handling",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        # ── HPP setup ─────────────────────────────────────────────────────────
        pkg = get_package_share_directory("agimus_demo_10_tiago_pro_bar_manip")
        tiago_pkg = get_package_share_directory("tiago_pro_description")
        moveit_pkg = get_package_share_directory("tiago_pro_moveit_config")

        self.get_logger().info("Initializing HPPPathGenerator ...")
        self._hpp = HPPPathGenerator(
            handle_config=os.path.join(
                pkg, "config/planning", "handles_configurations.yaml"
            ),
            robot_description=os.path.join(tiago_pkg, "robots", "tiago_pro.urdf.xacro"),
            robot_description_srdf=os.path.join(
                moveit_pkg, "config", "srdf", "tiago_pro.srdf.xacro"
            ),
            handle_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/reinforcement_bar.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/reinforcement_bar.srdf",
                name="reinforcement_bar",
            ),
            plate_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/plate.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/plate.srdf",
                name="plate",
            ),
            table_object=BaseObject(
                root_joint_type="anchor",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/table.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/table.srdf",
                name="table",
            ),
            ocp_dt=0.1,
            robot_name="tiago_pro",
            logger=self.get_logger(),
        )
        self.get_logger().info("HPPPathGenerator ready — action server started.")

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _cb_joints(self, msg):
        self._joint_state = msg

    # def _cb_odom(self, msg):      self._odom        = msg
    def _cb_object_pose(self):
        self._bar_pose = self._lookup_pose("table_link", "bar_base_link")
        self._plate_pose = self._lookup_pose("table_link", "plate_base_link")
        self._odom = self._lookup_pose("table_link", "base_footprint")

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _goal_cb(self, goal_request):
        # DEBUG : Vérifier ici si ROS voit les données
        self.get_logger().info(
            f"Goal Request - Gripper: '{goal_request.gripper}', Handle: '{goal_request.handle}'"
        )

        if not goal_request.gripper or not goal_request.handle:
            self.get_logger().error("REJECTED: Received empty strings in Goal Callback")
            return GoalResponse.REJECT

        if self._planning:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._planning = True
        feedback_msg = PlanBarGrasp.Feedback()
        result_msg = PlanBarGrasp.Result()
        t_start = time.time()

        gripper_name = goal_handle.request.gripper
        handle_name = goal_handle.request.handle
        self.get_logger().info(f"Executing plan for: {gripper_name} / {handle_name}")

        # 1. Waiting to get robot state
        timeout = 5.0
        deadline = time.time() + timeout
        while time.time() < deadline:
            if all(
                m is not None
                for m in [
                    self._joint_state,
                    self._odom,
                    self._bar_pose,
                    self._plate_pose,
                ]
            ):
                break
            # Do nothing, spin_once handle by the MultiThreaded Executor
            time.sleep(0.1)
        else:
            result_msg.success = False
            result_msg.message = "Timeout waiting for robot state."
            goal_handle.abort()
            self._planning = False
            return result_msg

        # 2. Build current environment config
        q_init = self._build_q_init()
        if q_init is None:
            result_msg.success = False
            result_msg.message = "Failed to build q_init."
            goal_handle.abort()
            self._planning = False
            return result_msg

        # 3. Planning
        path_result = [None]

        def _plan_worker(g, h, q):
            try:
                path_result[0] = self._hpp.plan_grasp(gripper=g, handle=h, q_init=q)
            except Exception as e:
                self.get_logger().error(f"HPP Thread Error: {e}")

        plan_thread = threading.Thread(
            target=_plan_worker, args=(gripper_name, handle_name, q_init)
        )
        plan_thread.start()

        while plan_thread.is_alive():
            if goal_handle.is_cancel_requested:
                plan_thread.join()
                result_msg.success = False
                result_msg.message = "Cancelled."
                goal_handle.canceled()
                self._planning = False
                return result_msg

            feedback_msg.status = "Planning in progress..."
            feedback_msg.elapsed_time = time.time() - t_start
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)

        plan_thread.join()

        # 5. Résultat
        if path_result[0] is None:
            result_msg.success = False
            result_msg.message = "Planning failed."
            goal_handle.abort()
        else:
            result_msg.success = True
            result_msg.message = "Success."
            result_msg.path_id = self._hpp._ps.numberPaths() - 1
            goal_handle.succeed()

        self._planning = False
        return result_msg

    # ── State helpers ─────────────────────────────────────────────────────────

    def _lookup_pose(self, parent_frame: str, child_frame: str) -> list | None:
        """
        Lookup transform parent_frame -> child_frame via tf2.
        Returns [x, y, z, qx, qy, qz, qw] or None if not available.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            # Normalize the quat
            quat = np.array([ro.x, ro.y, ro.z, ro.w])
            quat /= np.linalg.norm(quat)  # normalization
            return [tr.x, tr.y, tr.z, quat[0], quat[1], quat[2], quat[3]]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed for {child_frame}: {e}")
            return None

    def _odom_to_base(self, odom: Odometry) -> list:
        p, o = odom.pose.pose.position, odom.pose.pose.orientation
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        theta = np.arctan2(siny, cosy)
        return [p.x, p.y, np.cos(theta), np.sin(theta)]

    def _tf_to_base_odom(self, tf: list[float]) -> list:
        x = tf[0]
        y = tf[1]
        qx = tf[3]
        qy = tf[4]
        qz = tf[5]
        qw = tf[6]
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = np.arctan2(siny, cosy)
        return [x, y, np.cos(theta), np.sin(theta)]

    def _build_q_init(self) -> list | None:
        if any(
            m is None
            for m in [self._joint_state, self._odom, self._bar_pose, self._plate_pose]
        ):
            return None

        robot = self._hpp.robot
        q = robot.getCurrentConfig()
        # Tiago whole body
        # Others joints
        for i, joint_name in enumerate(self._joint_state.name):
            full = f"tiago_pro/{joint_name}"
            if full in robot.jointNames:
                rank = robot.rankInConfiguration[full]
                value = self._joint_state.position[i]
                # Wheel are defined as continuous joint (so2 object)
                nq = robot.getJointConfigSize(full)
                if nq == 2:
                    q[rank] = np.cos(value)
                    q[rank + 1] = np.sin(value)
                elif nq == 1:  # Joint standard
                    q[rank] = value
        # Base
        r = robot.rankInConfiguration["tiago_pro/root_joint"]
        q[r : r + 4] = self._tf_to_base_odom(self._odom)

        # Table
        # r_table = robot.rankInConfiguration["table/root_joint"]
        # q[r_table:r_table+7] = self._table_pose

        # Barre
        r_bar = robot.rankInConfiguration["reinforcement_bar/root_joint"]
        q[r_bar : r_bar + 7] = self._bar_pose

        # Plate
        r_plate = robot.rankInConfiguration["plate/root_joint"]
        q[r_plate : r_plate + 7] = self._plate_pose
        # _, q, _ = self._hpp._cg.applyNodeConstraints("free", q)
        return q


# ── Entrypoint ────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = HPPActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    node = HPPActionServer()
