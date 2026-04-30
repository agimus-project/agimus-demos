import os
import time

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from test_agimus_type.action import PlanBarGrasp
from agimus_demo_10_tiago_pro_bar_manip.traj.hpp_traj import (
    BaseObject,
    HPPPathGenerator,
)
from agimus_demo_10_tiago_pro_bar_manip.rostools import process_xacro


def _patch_srdf(srdf_str: str) -> str:
    """Inject disabled collision pairs into SRDF string."""
    i = srdf_str.find("</robot>")
    assert i != -1, "SRDF string does not contain </robot>"
    pairs = [
        ("base_link", "wheel_front_left_link"),
        ("base_link", "wheel_front_right_link"),
        ("base_link", "wheel_rear_left_link"),
        ("base_link", "wheel_rear_right_link"),
        ("gripper_left_screw_left_link", "gripper_left_fingertip_left_link"),
        ("gripper_right_screw_left_link", "gripper_right_fingertip_left_link"),
    ]
    insert = "".join(
        f'  <disable_collisions link1="{l1}" link2="{l2}" reason="Never"/>\n'
        for l1, l2 in pairs
    )
    return srdf_str[:i] + insert + "</robot>"


class HPPActionServer(Node):
    def __init__(self):
        super().__init__("hpp_action_server")

        self._joint_state = None
        self._odom = None
        self._bar_pose = None
        self._plate_pose = None

        # == Planning state =================================================
        self._planning = False
        self._traj_pick = None
        self._traj_place = None
        self._q_after_grasp = None

        self._cb_group = ReentrantCallbackGroup()
        self._hpp: HPPPathGenerator | None = None
        self.get_logger().info("Waiting for /robot_description …")

        qos_rd = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._urdf_str: str | None = None
        self.create_subscription(
            String,
            "/robot_description",
            self._cb_robot_description,
            qos_rd,
            callback_group=self._cb_group,
        )

        # == /joint_states ==================================================
        from rclpy.qos import QoSReliabilityPolicy as Rel

        qos_js = QoSProfile(depth=10, reliability=Rel.BEST_EFFORT)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._cb_joints,
            qos_js,
            callback_group=self._cb_group,
        )

        # == TF2 ============================================================
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self.create_timer(0.5, self._cb_object_pose, callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            PlanBarGrasp,
            "/orchestrator/plan_bar_handling",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

    # == /robot_description callback ===========================================

    def _cb_robot_description(self, msg: String):
        if self._hpp is not None:
            return
        self._urdf_str = msg.data
        self.get_logger().info("robot_description received — building HPP …")
        self._init_hpp()

    def _init_hpp(self):
        urdf_str = self._urdf_str

        # Pinocchio model (still built from the raw URDF string)
        robot_model = pin.buildModelFromXML(urdf_str)
        self.get_logger().info(f"Pinocchio model built: {robot_model.nq} dof")

        # == SRDF string =====================================================
        moveit_pkg = get_package_share_directory("tiago_pro_moveit_config")
        srdf_raw = os.path.join(moveit_pkg, "config", "srdf", "tiago_pro.srdf.xacro")

        srdf_string = process_xacro(
            str(srdf_raw),
            "end_effector_left:=pal-pro-gripper",
            "end_effector_right:=pal-pro-gripper",
        )
        srdf_str = _patch_srdf(srdf_string)

        # == Package paths for objects =======================================
        pkg = get_package_share_directory("agimus_demo_10_tiago_pro_bar_manip")

        self._hpp = HPPPathGenerator(
            urdf_str=urdf_str,
            srdf_str=srdf_str,
            robot_model=robot_model,
            handle_config=os.path.join(
                pkg, "config/planning", "handles_configurations.yaml"
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
        self.get_logger().info("HPPPathGenerator ready — action server active.")

    # == ROS callbacks =========================================================

    def _cb_joints(self, msg):
        self._joint_state = msg

    def _cb_object_pose(self):
        self._bar_pose = self._lookup_pose("table_link", "bar_base_link")
        self._plate_pose = self._lookup_pose("table_link", "plate_base_link")
        self._odom = self._lookup_pose("table_link", "base_footprint")
        self._bar_goal_pose = self._lookup_pose("table_link", "bar_goal_pose")

    # == Action callbacks ======================================================

    def _goal_cb(self, goal_request):
        if self._hpp is None:
            self.get_logger().error("REJECTED: HPP not ready yet")
            return GoalResponse.REJECT

        action_type = goal_request.action_type
        gripper = goal_request.gripper
        handle = goal_request.handle
        self.get_logger().info(
            f"Goal — action_type='{action_type}' gripper='{gripper}' handle='{handle}'"
        )

        if not action_type or not gripper or not handle:
            self.get_logger().error("REJECTED: empty field")
            return GoalResponse.REJECT
        if action_type not in ("grasp", "place"):
            self.get_logger().error(f"REJECTED: unknown action_type '{action_type}'")
            return GoalResponse.REJECT
        if action_type == "place" and self._q_after_grasp is None:
            self.get_logger().error("REJECTED: no grasp planned yet")
            return GoalResponse.REJECT
        if self._planning:
            self.get_logger().warn("REJECTED: already planning")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._planning = True
        result_msg = PlanBarGrasp.Result()
        action_type = goal_handle.request.action_type
        gripper_name = goal_handle.request.gripper
        handle_name = goal_handle.request.handle

        self.get_logger().info(
            f"Executing '{action_type}' — {gripper_name} / {handle_name}"
        )

        if not self._wait_for_state(timeout=5.0):
            result_msg.success = False
            result_msg.message = "Timeout waiting for robot state."
            goal_handle.abort()
            self._planning = False
            return result_msg

        try:
            if action_type == "grasp":
                result_msg.success, result_msg.message = self._plan_grasp(
                    gripper_name, handle_name
                )
            else:
                result_msg.success, result_msg.message = self._plan_place(
                    gripper_name, handle_name
                )
        except RuntimeError as e:
            self.get_logger().warn(f"Plan failed: {str(e)}")
            result_msg.success = False
            result_msg.message = str(e)
        except Exception as e:
            import traceback

            self.get_logger().error(f"CRITICAL BUG:\n{traceback.format_exc()}")
            result_msg.success = False
            result_msg.message = f"Internal Software Error: {str(e)}"

        if result_msg.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._planning = False
        return result_msg

    # == Planning helpers ======================================================

    def _plan_grasp(self, gripper, handle):
        q_init = self._build_q_init()
        if q_init is None:
            return False, "Failed to build q_init."

        traj, q_end = self._hpp.plan_grasp(
            gripper=gripper, handle=handle, q_init=q_init
        )
        if traj is None:
            return False, "Grasp planning failed."

        self._traj_pick = traj
        self._q_after_grasp = q_end
        self.get_logger().info("Grasp planned")
        return True, "Grasp planned successfully."

    def _plan_place(self, gripper, handle):
        q_init_place = list(self._q_after_grasp)
        r = self._hpp.robot.rankInConfiguration["tiago_pro/root_joint"]
        q_init_place[r] = 3.0

        target_bar_pose = [1.2, 0.0, 0.67, 0.0, 0.0, 0.0, 1.0]

        traj, _ = self._hpp.plan_place(
            gripper=gripper,
            handle=handle,
            q_init=q_init_place,
            target_bar_pose=target_bar_pose,
        )
        if traj is None:
            return False, "Place planning failed."

        self._traj_place = traj
        self.get_logger().info("Place planned")
        return True, "Place planned successfully"

    # == State helpers =========================================================

    def _wait_for_state(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if all(
                m is not None
                for m in [
                    self._joint_state,
                    self._odom,
                    self._bar_pose,
                    self._plate_pose,
                    self._bar_goal_pose,
                ]
            ):
                return True
            time.sleep(0.05)
        return False

    def _lookup_pose(self, parent_frame, child_frame):
        try:
            t = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            quat = np.array([ro.x, ro.y, ro.z, ro.w])
            quat /= np.linalg.norm(quat)
            return [tr.x, tr.y, tr.z, quat[0], quat[1], quat[2], quat[3]]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed for {child_frame}: {e}")
            return None

    def _tf_to_base_odom(self, tf):
        x, y = tf[0], tf[1]
        qx, qy, qz, qw = tf[3], tf[4], tf[5], tf[6]
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        return [x, y, np.cos(np.arctan2(siny, cosy)), np.sin(np.arctan2(siny, cosy))]

    def _build_q_init(self):
        if any(
            m is None
            for m in [self._joint_state, self._odom, self._bar_pose, self._plate_pose]
        ):
            return None
        robot = self._hpp.robot
        q = list(pin.neutral(robot.model()))
        for i, joint_name in enumerate(self._joint_state.name):
            full = f"tiago_pro/{joint_name}"
            if full in robot.rankInConfiguration:
                rank = robot.rankInConfiguration[full]
                nq = robot.model().joints[robot.model().getJointId(full)].nq
                value = self._joint_state.position[i]
                if nq == 2:
                    q[rank] = np.cos(value)
                    q[rank + 1] = np.sin(value)
                elif nq == 1:
                    q[rank] = value
        r = robot.rankInConfiguration["tiago_pro/root_joint"]
        q[r : r + 4] = self._tf_to_base_odom(self._odom)
        r_bar = robot.rankInConfiguration["reinforcement_bar/root_joint"]
        q[r_bar : r_bar + 7] = self._bar_pose
        r_plate = robot.rankInConfiguration["plate/root_joint"]
        q[r_plate : r_plate + 7] = self._plate_pose
        return q


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
    main()
