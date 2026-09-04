"""
Orchestrator of the bimanual bar manipulation on TiagoPro

Uses a ros action server to launch the planning and executing:
    ros2 action send_goal /orchestrator/plan_bar_handling test_agimus_type/action/PlanBarPick "{task: 'pick', gripper: 'tiago_pro/left', handle: 'reinforcement_bar/left'}"

The available options are:
    - tasks:
        pick
        place
    - gripper:
        tiago_pro/left
        tiago_pro/right
    - handle:
        reinforcement_bar/left
        reinforcement_bar/right

After the action is call the server will launch the plan with HPP, sample the trajectory, and publishes it to the /mpc_input topic

Launched with the bringup.launch.py file of the same package

"""

import os
import time
import copy
import yaml
import threading
import numpy as np
import pinocchio as pin
from pathlib import Path as FilePath
from collections import deque
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from agimus_msgs.msg import MpcInput
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller_ros.ros_utils import weighted_traj_point_to_mpc_msg
from test_agimus_type.action import (
    PlanBarPick,
)  # TODO change name to  remove "pick" specifics
from agimus_demo_10_tiago_pro_bar_manip.traj.hpp_traj import (
    BaseObject,
    HPPPathGenerator,
)
from agimus_demo_10_tiago_pro_bar_manip.planner.path_planner import PlanningCancelled
from agimus_demo_10_tiago_pro_bar_manip.rostools import process_xacro

GRIPPER_OPEN_POSITION = 0.07
GRIPPER_CLOSED_POSITION = 0.0
GRIPPER_MOTION_DURATION = 1.5
LEFT_GRIPPER_RELEASE_SERVICE = "/gripper_left_grasper_srv/release"
LEFT_GRIPPER_GRASP_SERVICE = "/gripper_left_grasper_srv/grasp"
RIGHT_GRIPPER_RELEASE_SERVICE = "/gripper_right_grasper_srv/release"
RIGHT_GRIPPER_GRASP_SERVICE = "/gripper_right_grasper_srv/grasp"

BASE_TRAJECTORY_TOPIC = "/trajectory"
BASE_TRAJECTORY_FRAME = "world"


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


class GoalCancelled(Exception):
    """Raised internally when the action goal is cancelled while a planned
    trajectory is being drained to the MPC / while waiting for EE
    convergence — as opposed to PlanningCancelled, which covers the HPP
    planning phase.
    """

    pass


class HPPActionServer(Node):
    def __init__(self):
        super().__init__("hpp_action_server")

        # == Internal State =================================================
        self._joint_state = None
        self._odom = None
        self._bar_pose = None
        self._plate_pose = None
        self._table_pose = None

        # == Planning state =================================================
        self._path = None
        self._planning = False
        self._q_after_pick = None
        self._trajectory_buffer_lock = threading.Lock()
        self._trajectory_buffer = deque()

        # == OCP-related variables ==========================================
        self._ocp_dt = 0.01

        # == Robot state ====================================================
        self._robot_model = None
        self._robot_data = None
        self._nq = None
        self._nv = None

        self.declare_parameter("orchestrator_hpp_config", "Unconfigured path")
        config_file_path = FilePath(self.get_parameter("orchestrator_hpp_config").value)

        with open(config_file_path, "r") as f:
            _config_file = yaml.load(f, Loader=yaml.SafeLoader)

        self._r_config = _config_file["robot_configuration"]
        self._weights_dict = _config_file["weights"]
        self._ee_placement_tolerances = _config_file["ee_placement_tolerances"]
        self._left_tool_frame_id_name = self._r_config["left_tool"]
        self._left_tool_frame_id_pin_frame = None
        self._right_tool_frame_id_name = self._r_config["right_tool"]
        self._right_tool_frame_id_pin_frame = None

        # == HPP ============================================================
        self._cb_group = ReentrantCallbackGroup()
        self._hpp: HPPPathGenerator | None = None
        self.get_logger().info("Waiting for /robot_description …")

        # == ROS ============================================================
        # == Subscribers

        # robot_description
        qos_rd = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "/robot_description",
            self._cb_robot_description,
            qos_rd,
            callback_group=self._cb_group,
        )

        # joint_states
        qos_js = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._cb_joints,
            qos_js,
            callback_group=self._cb_group,
        )
        # self.create_timer(0.5, self._cb_object_pose, callback_group=self._cb_group) # TODO ?keep?

        # == Publishers
        # Output of the orchestrator and input of the MPC
        self._mpc_input_pub = self.create_publisher(
            MpcInput,
            "/mpc_input",
            qos_profile=QoSProfile(
                depth=1000,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            ),
        )
        self._mpc_input_publisher_timer = self.create_timer(
            self._ocp_dt,
            self._publish_mpc_input_cb,  # TODO change ocp_dt to be from a yaml
        )

        self._base_traj_pub = self.create_publisher(
            Path,
            BASE_TRAJECTORY_TOPIC,
            10,
        )

        # self._traj_pub = self.create_publisher(
        #     JointTrajectory,
        #     "/joint_trajectory_controller/joint_trajectory",
        #     qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        # )

        self._traj_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        # == TF2 ============================================================
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self.create_timer(0.5, self._cb_object_pose, callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            PlanBarPick,
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
        self.get_logger().info("/robot_description received — building HPP...")
        self._init_hpp(urdf_str=msg.data)
        self._buildRobot()

    def _buildRobot(self):
        full_robot_model = self._hpp.robot.model().copy()

        locked_joint_ids = []
        for full in full_robot_model.names.tolist():
            if full == "universe":
                continue
            joint_name = full.removeprefix("tiago_pro/")
            if joint_name not in self._r_config["moving_joints"]:
                joint_id = int(full_robot_model.getJointId(full))  # int natif Python
                locked_joint_ids.append(joint_id)

        q_ref = pin.neutral(full_robot_model)  # vecteur full nq, toujours disponible

        self._robot_model = pin.buildReducedModel(
            full_robot_model,
            locked_joint_ids,  # liste Python d'int natifs
            q_ref,
        )
        self._robot_data = self._robot_model.createData()

        self._left_tool_frame_id_pin_frame = self._robot_model.getFrameId(
            f"tiago_pro/{self._left_tool_frame_id_name}"
        )
        self._right_tool_frame_id_pin_frame = self._robot_model.getFrameId(
            f"tiago_pro/{self._right_tool_frame_id_name}"
        )
        self.get_logger().info(f"Pinocchio model built: {self._robot_model.nq} dof")

    def _init_hpp(self, urdf_str):
        # SRDF string
        moveit_pkg = get_package_share_directory("tiago_pro_moveit_config")
        srdf_raw = os.path.join(moveit_pkg, "config", "srdf", "tiago_pro.srdf.xacro")

        srdf_string = process_xacro(
            str(srdf_raw),
            "end_effector_left:=pal-pro-gripper",
            "end_effector_right:=pal-pro-gripper",
        )
        srdf_str = _patch_srdf(srdf_string)

        # Package paths for objects
        pkg = get_package_share_directory("agimus_demo_10_tiago_pro_bar_manip")

        self._hpp = HPPPathGenerator(
            urdf_str=urdf_str,
            srdf_str=srdf_str,
            hpp_config=os.path.join(
                pkg, "config/planning", "orchestrator_hpp_configuration.yaml"
            ),
            handle_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/standalone/reinforcement_bar.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/reinforcement_bar.srdf",
                name="reinforcement_bar",
            ),
            plate_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/standalone/plate.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/plate.srdf",
                name="plate",
            ),
            table_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/standalone/table.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/table.srdf",
                name="table",
            ),
            ocp_dt=0.01,
            robot_name="tiago_pro",
            logger=self.get_logger(),
        )
        self.get_logger().info("HPPPathGenerator ready — action server active.")

    # == ROS callbacks =========================================================

    def _cb_joints(self, msg):
        self._joint_state = msg

    def _cb_object_pose(self):
        self._bar_pose = self._lookup_pose("world", "bar_base_link")
        self._plate_pose = self._lookup_pose("world", "plate_base_link")
        self._odom = self._lookup_pose("world", "base_footprint")
        self._bar_goal_pose = self._lookup_pose("world", "bar_goal_pose")
        self._table_pose = self._lookup_pose("world", "table_link")

    def _call_grasper_service(
        self,
        service_name: list[str],
        action_label: list[str],
        timeout: list[float] = {5.0, 5.0},
    ) -> tuple:

        client_left = self.create_client(Empty, service_name[0])
        client_right = self.create_client(Empty, service_name[1])

        deadline = time.time() + timeout
        while time.time() < deadline:
            if client_left.wait_for_service(
                timeout_sec=min(0.2, self._ocp_dt)
            ) and client_right.wait_for_service(timeout_sec=min(0.2, self._ocp_dt)):
                break
        else:
            self.get_logger().info(
                f"One of the Gripper service '{service_name}' is not available."
            )
            return False, False

        future_left = client_left.call_async(Empty.Request())
        future_right = client_right.call_async(Empty.Request())

        while not future_left.done() and future_right.done():
            time.sleep(0.0001)

        if future_left.cancelled():
            self.get_logger().info(
                f"Gripper {action_label} request to '{service_name}' was cancelled."
            )
            return False

        exc = future_left.exception()
        if exc is not None:
            self.get_logger().info(
                f"Gripper {action_label} failed via '{service_name}': {exc}"
            )
            return False

        return True, True

    def open_gripper(
        self,
        position: float = GRIPPER_OPEN_POSITION,
        duration: float = GRIPPER_MOTION_DURATION,
        timeout: float = 5.0,
    ) -> bool:
        """Open the left gripper in Gazebo."""
        del position, duration
        return self._call_grasper_service(
            [LEFT_GRIPPER_RELEASE_SERVICE, RIGHT_GRIPPER_RELEASE_SERVICE],
            ["open", "open"],
            timeout=timeout,
        )

    def close_gripper(
        self,
        position: float = GRIPPER_CLOSED_POSITION,
        duration: float = GRIPPER_MOTION_DURATION,
        timeout: float = 5.0,
    ) -> bool:
        """Close the left gripper in Gazebo."""
        del position, duration
        return self._call_grasper_service(
            [LEFT_GRIPPER_GRASP_SERVICE, RIGHT_GRIPPER_GRASP_SERVICE],
            ["close", "close"],
            timeout=timeout,
        )

    # == Action callbacks ======================================================

    def _goal_cb(self, goal_request):

        if self._hpp is None:
            self.get_logger().error(
                "REJECTED: HPP hasn't been initialized, check if /robot_description is published"
            )
            return GoalResponse.REJECT
        if self._planning:
            self.get_logger().warn("REJECTED: already planning")
            return GoalResponse.REJECT

        task = goal_request.task
        gripper = goal_request.gripper
        handle = goal_request.handle
        self.get_logger().info(
            f"Goal — task='{task}' gripper='{gripper}' handle='{handle}'"
        )

        if not task or not gripper or not handle:
            self.get_logger().error("REJECTED: empty field(s) in action request")
            return GoalResponse.REJECT

        match task:
            case "pick":
                if self._robot_model is None:
                    self.get_logger().error(
                        "REJECTED: Robot description has not been received yet"
                    )
                    return GoalResponse.REJECT
            case "place":
                if self._q_after_pick is None:
                    self.get_logger().error("REJECTED: no pick planned")
                    return GoalResponse.REJECT
            case _:
                self.get_logger().error(
                    f"REJECTED: task '{task}' not in ['pick','place']"
                )
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """Plans the trajectory using HPP and executes each segment sequentially.
        Args:
            goal_handle (rclpy.action.server.ServerGoalHandle): ros2 action message
        Returns:
            test_agimus_type.action._plan_bar_pick.PlanBarPick_Result: return message
        """
        self._planning = True
        result_msg = PlanBarPick.Result()
        task = goal_handle.request.task
        gripper_name = goal_handle.request.gripper
        handle_name = goal_handle.request.handle

        self.get_logger().info(f"Executing '{task}' — {gripper_name} / {handle_name}")
        if not self._wait_for_state(goal_handle, timeout=5.0):
            result_msg.success = False
            result_msg.message = "Timeout waiting for robot state or cancelled."
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self._planning = False
            return result_msg

        def cancel_check():
            return goal_handle.is_cancel_requested

        # Feedback: planning
        feedback_msg = PlanBarPick.Feedback()
        feedback_msg.current_action = f'plan "{task}" path'
        feedback_msg.message = ""
        goal_handle.publish_feedback(feedback_msg)

        success = False
        message = ""
        traj = None

        try:
            match task:
                case "pick":
                    success, message, traj = self._plan_pick(
                        gripper_name, handle_name, cancel_check=cancel_check
                    )
                case "place":
                    success, message, traj = self._plan_place(
                        gripper_name, handle_name, cancel_check=cancel_check
                    )
                case _:
                    success = False
                    message = f"task '{task}' not in ['pick', 'place']"
                    self.get_logger().error(message)

        except PlanningCancelled as e:
            self.get_logger().info(f"Planning cancelled: {e}")
            if hasattr(self._hpp, "reset"):
                self._hpp.reset()  # Clean HPP
            self._q_after_pick = None  # Impossible to do a place after a cancel pick
            result_msg.success = False
            result_msg.message = "Cancelled by user request"
            goal_handle.canceled()
            self._planning = False
            return result_msg

        except RuntimeError as e:
            self.get_logger().warn(f"Plan failed: {str(e)}")
            success = False
            message = str(e)

        except Exception as e:
            import traceback

            self.get_logger().error(f"CRITICAL BUG:\n{traceback.format_exc()}")
            success = False
            message = f"Internal Software Error: {str(e)}"

        if not success:
            self.get_logger().warn(f"Planning failed: {message}")
            result_msg.success = False
            result_msg.message = message
            goal_handle.abort()
            self._planning = False
            return result_msg

        # Gripper actions per segment:
        # pick:  Approach=None, Pregrasp=open, Grasp=close, Preplace=None
        # place: Approach=None, Preplace=None, Place=open,  Retreat=None
        gripper_actions = {
            "pick": [None, "open", "close", None],
            "place": [None, None, "open", None],
        }

        self.get_logger().info(f"Planning succeeded — executing {len(traj)} segments")

        try:
            for seg_id, segment in enumerate(traj):
                if goal_handle.is_cancel_requested:
                    result_msg.success = False
                    result_msg.message = "Cancelled before segment start"
                    raise GoalCancelled(result_msg.message)
                    return result_msg

                gripper_action = gripper_actions[task][seg_id]

                feedback_msg = PlanBarPick.Feedback()
                feedback_msg.current_action = "execute"
                feedback_msg.message = (
                    f"Segment {seg_id}/{len(traj) - 1} — gripper={gripper_action}"
                )
                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(f"Segment {seg_id}: gripper={gripper_action}")

                ok = self._execute_segment_and_wait_ee(
                    traj=segment,
                    task=task,
                    gripper_action=gripper_action,
                    ee_tol_pos=self._ee_placement_tolerances["position"],
                    ee_tol_rot=self._ee_placement_tolerances["rotation"],
                    cancel_check=cancel_check,
                )
                # self.get_logger().info("Waiting 1 sec between segments ")
                # time.sleep(1)

                if not ok:
                    message = f"Segment {seg_id} failed"
                    self.get_logger().error(message)
                    result_msg.success = False
                    result_msg.message = message
                    goal_handle.abort()
                    self._planning = False
                    return result_msg

        except GoalCancelled as e:
            self.get_logger().info(f"Execution cancelled: {e}")
            self._purge_trajectories()  # Stopping MPC and Base
            if hasattr(self._hpp, "reset"):
                self._hpp.reset()  # Clean HPP
            self._q_after_pick = None  # Restart sequence
            result_msg.success = False
            result_msg.message = "Cancelled by user request"
            goal_handle.canceled()
            self._planning = False
            return result_msg

        self.get_logger().info("All segments completed successfully")
        result_msg.success = True
        result_msg.message = "Task completed successfully"
        goal_handle.succeed()
        self._planning = False
        return result_msg

    def _execute_segment_and_wait_ee(
        self,
        traj: list,
        task: str,
        gripper_action: str | None,  # "open" | "close" | None
        timeout_margin: float = 120.0,
        ee_tol_pos: float = 0.04,
        ee_tol_rot: float = 0.06,
        poll_period: float = 0.1,
        cancel_check=None,
        goal_handle=None,
    ) -> bool:
        weighted_trajectory = self._convert_path(traj, task)
        base_traj = self._base_trajectory(traj)
        num_points = len(weighted_trajectory)

        self.get_logger().info(f"Sending {num_points} points to MPC buffer...")
        with self._trajectory_buffer_lock:
            self._trajectory_buffer.extend(weighted_trajectory)

        self.send_base_traj(base_traj)

        traj_duration = num_points * self._ocp_dt
        deadline = time.time() + traj_duration + timeout_margin

        q_target = traj[-1]
        frames_to_check = [
            self._left_tool_frame_id_pin_frame,
            self._right_tool_frame_id_pin_frame,
        ]

        self.get_logger().info("Waiting for buffer drain + EE convergence...")

        frames_to_check = [
            self._left_tool_frame_id_pin_frame,
            self._right_tool_frame_id_pin_frame,
        ]
        traj_done = [False] * len(frames_to_check)

        last_log_state = [{"pos": None, "rot": None} for _ in frames_to_check]

        log_delta_pos = 0.001
        log_delta_rot = 0.01

        while time.time() < deadline:
            if cancel_check is not None and cancel_check():
                self.get_logger().info(
                    "Cancel requested — freezing trajectory buffer and stopping wait."
                )
                self._freeze_trajectory_buffer()
                raise GoalCancelled("Segment execution cancelled by user request")

            with self._trajectory_buffer_lock:
                buf_len = len(self._trajectory_buffer)

            if buf_len <= 1 and self._joint_state is not None:
                q_current = self._build_q_init()

                pos_errs, rot_errs = self._get_ee_pose_error(
                    q_current, q_target, frames_to_check
                )

                for i in range(len(frames_to_check)):
                    pos_c = pos_errs[i]
                    rot_c = rot_errs[i]
                    frame_name = frames_to_check[i]

                    if pos_c <= ee_tol_pos and rot_c <= ee_tol_rot:
                        traj_done[i] = True
                    else:
                        traj_done[i] = False

                        last_pos = last_log_state[i]["pos"]
                        last_rot = last_log_state[i]["rot"]

                        if (
                            last_pos is None
                            or abs(pos_c - last_pos) > log_delta_pos
                            or abs(rot_c - last_rot) > log_delta_rot
                        ):
                            self.get_logger().info(
                                f"Waiting for [{frame_name}] -> "
                                f"Pos Err: {pos_c:.3f}m (tol: {ee_tol_pos}) | "
                                f"Rot Err: {rot_c:.3f}rad (tol: {ee_tol_rot})"
                            )

                            last_log_state[i]["pos"] = pos_c
                            last_log_state[i]["rot"] = rot_c

                if all(traj_done):
                    self.get_logger().info("Target reached for all EE")
                    break

            time.sleep(poll_period)

        if not all(traj_done):
            for i in range(len(frames_to_check)):
                if not traj_done[i]:
                    self.get_logger().warn(
                        f"Timeout sur [{frames_to_check[i]}] : bloqué à "
                        f"Pos={pos_errs[i]:.3f}m, Rot={rot_errs[i]:.3f}rad"
                    )
            return False

        # Action gripper
        if gripper_action == "open":
            return self.open_gripper()
        elif gripper_action == "close":
            return self.close_gripper()
        return True

    def _extract_q_robot(self, q_hpp: list) -> np.ndarray:
        """Extracts the native Pinocchio configuration from the full HPP state"""
        robot = self._hpp.robot
        q_robot = np.zeros(self._robot_model.nq)

        # Iterate over the native Pinocchio model joints
        for joint_name in self._robot_model.names:
            if joint_name == "universe":
                continue
            if joint_name in robot.rankInConfiguration:
                rank = robot.rankInConfiguration[joint_name]
                joint_id = self._robot_model.getJointId(joint_name)
                idx_q = self._robot_model.joints[joint_id].idx_q
                nq = self._robot_model.joints[joint_id].nq
                for j in range(nq):
                    q_robot[idx_q + j] = q_hpp[rank + j]

        return q_robot

    def _get_ee_pose_error(self, q_current, q_target, list_frame_id: list) -> tuple:
        """
        Compute pose Error between 2 config
        """
        q_cur_rob = self._extract_q_robot(q_current)
        q_tgt_rob = self._extract_q_robot(q_target)

        pin_model = self._robot_model
        pin_data = pin_model.createData()

        M_current = []
        M_target = []

        pos_err_list = []
        rot_err_list = []

        pin.forwardKinematics(pin_model, pin_data, q_cur_rob)
        pin.updateFramePlacements(pin_model, pin_data)
        for frame_id in list_frame_id:
            M_current.append(pin_data.oMf[frame_id].copy())

        pin.forwardKinematics(pin_model, pin_data, q_tgt_rob)
        pin.updateFramePlacements(pin_model, pin_data)
        for frame_id in list_frame_id:
            M_target.append(pin_data.oMf[frame_id].copy())

        for idx in range(len(list_frame_id)):
            err = pin.log6(M_current[idx].inverse() * M_target[idx])

            pos_err = np.linalg.norm(err.linear)
            rot_err = np.linalg.norm(err.angular)

            pos_err_list.append(float(pos_err))
            rot_err_list.append(float(rot_err))

        return pos_err_list, rot_err_list

    # == Planning helpers ======================================================

    def _plan_pick(self, gripper, handle, cancel_check=None):
        """Plans the pick trajectory using the self._hpp object

        Args:
            gripper (str): name of the gripper
            handle (str): name of the handle on the picked object
            cancel_check: optional zero-arg callable; if it returns True during
                planning, PlanningCancelled is raised (propagated from HPP).

        Returns:
            Bool, str, List: success, message, trajectory
        """
        q_init = self._build_q_init()
        if q_init is None:
            return False, "Failed to build q_init", None

        traj = self._hpp.plan_pick(
            gripper=gripper, handle=handle, q_init=q_init, cancel_check=cancel_check
        )
        if traj is None or any(seg is None for seg in traj):
            return False, "Pick planning failed", None

        self._q_after_pick = list(traj[-1][-1])
        return True, "Pick planned successfully", traj

    def _plan_place(self, gripper, handle, cancel_check=None):
        """Plans the place trajectory using the self._hpp object

        Args:
            gripper (str): name of the gripper
            handle (str): name of the handle on the placed object
            cancel_check: optional zero-arg callable; if it returns True during
                planning, PlanningCancelled is raised (propagated from HPP).

        Returns:
            Bool, str, List: success, message, trajectory
        """

        q_init_place = list(self._q_after_pick)
        r = self._hpp.robot.rankInConfiguration["tiago_pro/root_joint"]
        hx = self._hpp.robot.rankInConfiguration["reinforcement_bar/root_joint"]

        q_init_place[r] += 1.0
        q_init_place[hx] += 1.0

        target_bar_pose = self._bar_goal_pose

        traj = self._hpp.plan_place(
            gripper=gripper,
            handle=handle,
            q_init=q_init_place,
            target_bar_pose=target_bar_pose,
            cancel_check=cancel_check,
        )
        if traj is None or any(seg is None for seg in traj):
            return False, "Place planning failed.", None

        return True, "Place planned successfully", traj

    # == Running helpers ======================================================

    def _publish_mpc_input_cb(self) -> None:
        # Skip if buffer is empty
        with self._trajectory_buffer_lock:
            _buffer_len = len(self._trajectory_buffer)
        if not _buffer_len:
            return

        # TODO: see if the check is necessary for us
        # if self._buffer_size >= self._params.ocp_buffer_size:
        #     return

        def _get_traj_point() -> WeightedTrajectoryPoint:
            if len(self._trajectory_buffer) == 1:
                return self._trajectory_buffer[0]
            else:
                return self._trajectory_buffer.popleft()

        with self._trajectory_buffer_lock:
            mpc_input_msg = weighted_traj_point_to_mpc_msg(_get_traj_point())

        self._mpc_input_pub.publish(mpc_input_msg)

    # == State helpers ========================================================

    def _wait_for_state(self, goal_handle=None, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            # ---> Check for cancel <---
            if goal_handle and goal_handle.is_cancel_requested:
                return False
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
        for i, joint_name in enumerate(
            self._joint_state.name
        ):  # TODO duplicated code find way to merge
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
        r_table = robot.rankInConfiguration["table/root_joint"]
        q[r_table : r_table + 7] = self._table_pose
        return q

    # == Path helpers =========================================================

    def _convert_path(self, trajectory, task):
        """Converts the hpp path into an agimus-controller compatible one
        Args:
            trajectory list: list of q
        Returns:
            [TrajectoryPoint]: trajectory converted in a list of TrajectoryPoint
        """

        while self._robot_model is None:
            self.get_logger().info("No robot description yet")
            time.sleep(0.5)

        weights = self._build_traj_weights(task)
        converted_trajectory = [
            self._convert_point(i, trajectory[i], weights)
            for i in range(len(trajectory))
        ]

        return converted_trajectory

    def _base_trajectory(self, trajectory):
        robot = self._hpp.robot
        base_joint_name = "tiago_pro/root_joint"
        rank = robot.rankInConfiguration[base_joint_name]
        q_base = []
        for i in range(len(trajectory)):
            q_base.append(trajectory[i][rank : rank + 4])  # x,y,cos(theta),sin(theta)
        return q_base

    def _freeze_trajectory_buffer(self) -> None:
        """Collapse the pending MPC buffer down to a single 'hold position' point
        instead of emptying it.
        """
        with self._trajectory_buffer_lock:
            if self._trajectory_buffer:
                hold_point = self._trajectory_buffer[0]
                self._trajectory_buffer.clear()
                self._trajectory_buffer.append(hold_point)

    def send_base_traj(self, base_traj: list) -> None:
        if not base_traj:
            self.get_logger().warn("send_base_traj: traj empty")
            return

        stamp = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.frame_id = BASE_TRAJECTORY_FRAME
        path_msg.header.stamp = stamp

        for q_base in base_traj:
            assert len(q_base) == 4

            x, y, cos_t, sin_t = (
                float(q_base[0]),
                float(q_base[1]),
                float(q_base[2]),
                float(q_base[3]),
            )
            theta = float(np.arctan2(sin_t, cos_t))

            pose = PoseStamped()
            pose.header.frame_id = BASE_TRAJECTORY_FRAME
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            R = pin.rpy.rpyToMatrix(0.0, 0.0, theta)
            quat = pin.Quaternion(R)

            pose.pose.orientation.x = quat.x
            pose.pose.orientation.y = quat.y
            pose.pose.orientation.z = quat.z
            pose.pose.orientation.w = quat.w

            path_msg.poses.append(pose)

        self._base_traj_pub.publish(path_msg)

    def _convert_point(
        self, id: int, q_env: list, weights: TrajectoryPointWeights
    ) -> TrajectoryPoint:
        """Converts a point from HPP to agimus controller format
        Args:
            i (int) : point number
            q (np.array): joint values
            task (string): name of the task used to build the correct weights
        Returns:
            WeightedTrajectoryPoint: point with weights in the agimus-controller format
        """

        # select only tiago
        if any(
            m is None
            for m in [self._joint_state, self._odom, self._bar_pose, self._plate_pose]
        ):
            return None
        robot = self._hpp.robot
        q_robot = []
        for full in self._robot_model.names.tolist():
            joint_name = full.removeprefix("tiago_pro/")
            if joint_name in self._r_config["moving_joints"]:
                rank = robot.rankInConfiguration[full]
                value = q_env[rank]
                # nq = len(robot.getJointConfig(full))
                # if nq == 2:
                #     q_robot.append(np.cos(value))
                #     q_robot.append(np.sin(value))
                # elif nq == 1:
                q_robot.append(value)

        # ! Kept the way to add the base root when we'll add it
        # r = robot.rankInConfiguration["tiago_pro/root_joint"]
        # q[r : r + 4] = self._tf_to_base_odom(self._odom)

        pin.framesForwardKinematics(
            self._robot_model, self._robot_data, np.asarray(q_robot)
        )  # ! unused in the message cause there is no vel and acc and effort?

        traj_point = TrajectoryPoint(
            id=id,
            time_ns=0,
            robot_configuration=q_robot,
            robot_velocity=np.zeros_like(q_robot),
            robot_acceleration=np.zeros(len(q_robot)),  # self._nv
            robot_effort=np.zeros(len(q_robot)),  # self._nv
            forces={},
            end_effector_poses={
                self._left_tool_frame_id_name: copy.copy(
                    self._robot_data.oMf[self._left_tool_frame_id_pin_frame]
                ),
                self._right_tool_frame_id_name: copy.copy(
                    self._robot_data.oMf[self._right_tool_frame_id_pin_frame]
                ),
            },
        )
        # self.get_logger().info(f"ee pose :{traj_point.end_effector_poses}")

        return WeightedTrajectoryPoint(point=copy.copy(traj_point), weights=weights)

    def _build_traj_weights(self, task: str):
        """Builds a TrajectoryPointWeights for a given task.
        Task should be defined in the config yaml `self._weights_dict`
        Args:
            task (str): task type : ['pick', 'place']
        """
        return TrajectoryPointWeights(
            w_robot_configuration=np.array(self._weights_dict[task]["w_q"]),
            w_robot_velocity=np.array(self._weights_dict[task]["w_dq"]),
            w_robot_acceleration=np.array(self._weights_dict[task]["w_ddq"]),
            w_robot_effort=np.array(self._weights_dict[task]["w_effort"]),
            w_end_effector_poses={
                self._left_tool_frame_id_name: np.concatenate(
                    (
                        np.asarray(self._weights_dict[task]["w_frame_translation"]),
                        np.asarray(self._weights_dict[task]["w_frame_rotation"]),
                    )
                ),
                self._right_tool_frame_id_name: np.concatenate(
                    (
                        np.asarray(self._weights_dict[task]["w_frame_translation"]),
                        np.asarray(self._weights_dict[task]["w_frame_rotation"]),
                    )
                ),
            },
            w_end_effector_velocities={},
            w_forces={},
            w_collision_avoidance=self._weights_dict[task]["w_collision_avoidance"],
        )

    def _purge_trajectories(self):
        """Purge all trajectory buffers to immediately stop the robot."""
        self.get_logger().info("Purging trajectory buffers (Cancel requested)...")

        # 1. Freeze the MPC buffer instead of clearing completely to avoid crash
        self._freeze_trajectory_buffer()

        # 2. Stop the base by sending an empty path
        empty_path = Path()
        empty_path.header.frame_id = BASE_TRAJECTORY_FRAME
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self._base_traj_pub.publish(empty_path)


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
