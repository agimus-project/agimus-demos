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

# TODO : Move the HPP to its own python file

"""

import os
import time
import copy
import yaml
import threading
import numpy as np
import pinocchio as pin
from pathlib import Path
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

        # == Internal State =================================================
        self._joint_state = None
        self._odom = None
        self._bar_pose = None
        self._plate_pose = None

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
        config_file_path = Path(self.get_parameter("orchestrator_hpp_config").value)

        with open(config_file_path, "r") as f:
            _config_file = yaml.load(f, Loader=yaml.SafeLoader)

        self._r_config = _config_file["robot_configuration"]
        self._weights_dict = _config_file["weights"]
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
        self._buildRobot(urdf_str=msg.data)
        self.get_logger().info("/robot_description received — building HPP...")
        self._init_hpp(urdf_str=msg.data)

    def _buildRobot(self, urdf_str):
        """Builds the pinocchio robot model from the urdf and
        sets `_right_tool_frame_id_pin_frame` & `_left_tool_frame_id_pin_frame`
        Args:
            urdf_str (str): urdf string
        """
        self._robot_model = pin.buildModelFromXML(urdf_str)
        self._robot_data = self._robot_model.createData()
        self._nq = self._robot_model.nq
        self._nv = self._robot_model.nv
        self._left_tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._left_tool_frame_id_name
        )
        self._right_tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._right_tool_frame_id_name
        )
        self.get_logger().info(f"Pinocchio model built: {self._robot_model.nq} dof")

    def _init_hpp(self, urdf_str):
        if self._robot_model is None:
            raise ValueError("Pinocchio robot model is empty!")

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
            robot_model=self._robot_model,
            config=os.path.join(
                pkg, "config/planning", "orchestrator_hpp_configuration.yaml"
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
            ocp_dt=0.01,
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
        """Plans the trajectory using HPP and adds it to the buffer

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

        if not self._wait_for_state(timeout=5.0):
            result_msg.success = False
            result_msg.message = "Timeout waiting for robot state."
            goal_handle.abort()
            self._planning = False
            return result_msg

        # Planning
        # update the action server feedback to match current action
        feedback_msg = PlanBarPick.Feedback()
        feedback_msg.current_action = f'plan "{task}" path'
        feedback_msg.message = ""
        goal_handle.publish_feedback(feedback_msg)

        message = None
        success = False
        traj = None
        try:
            match task:
                case "pick":
                    success, message, traj = self._plan_pick(gripper_name, handle_name)
                case "place":
                    success, message, traj = self._plan_place(gripper_name, handle_name)
                case _:
                    self.get_logger().error(
                        f"_execute_ plan : task '{task}' not in ['pick','place']"
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

        self.get_logger().info(message)

        if not success:
            goal_handle.abort()
            return
        else:
            # Sending traj to robot
            weighted_trajectory = self._convert_path(traj, task)
            with self._trajectory_buffer_lock:
                self._trajectory_buffer.extend(weighted_trajectory)

            feedback_msg = PlanBarPick.Feedback()
            feedback_msg.current_action = "Run"
            feedback_msg.message = "Trajectory sent to MPC"
            goal_handle.publish_feedback(feedback_msg)

        result_msg.success = success
        result_msg.message = message
        self._planning = False

        return result_msg

    # == Planning helpers ======================================================

    def _plan_pick(self, gripper, handle):
        """Plans the pick trajectory using the self._hpp object

        Args:
            gripper (str): name of the gripper
            handle (str): name of the handle on the picked object

        Returns:
            Bool, str, List: success, message, trajectory
        """
        q_init = self._build_q_init()
        if q_init is None:
            return False, "Failed to build q_init", None

        traj, q_end = self._hpp.plan_pick(gripper=gripper, handle=handle, q_init=q_init)
        if traj is None:
            return False, "Pick planning failed", None

        self._q_after_pick = q_end
        return True, "Pick planned successfully", traj

    def _plan_place(self, gripper, handle):
        """Plans the place trajectory using the self._hpp object

        Args:
            gripper (str): name of the gripper
            handle (str): name of the handle on the placed object

        Returns:
            Bool, str, List: success, message, trajectory
        """

        q_init_place = list(self._q_after_pick)
        r = self._hpp.robot.rankInConfiguration["tiago_pro/root_joint"]
        hx = self._hpp.robot.rankInConfiguration["reinforcement_bar/root_joint"]

        q_init_place[r] += 1.0
        q_init_place[hx] += 1.0

        target_bar_pose = [1.2, 0.0, 0.67, 0.0, 0.0, 0.0, 1.0]

        traj, _ = self._hpp.plan_place(
            gripper=gripper,
            handle=handle,
            q_init=q_init_place,
            target_bar_pose=target_bar_pose,
        )
        if traj is None:
            return False, "Place planning failed."

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

        for joint_name in self._robot_model.names.tolist():
            full = f"tiago_pro/{joint_name}"
            if joint_name in self._r_config["moving_joints"]:
                rank = robot.rankInConfiguration[full]
                value = q_env[rank]
                nq = len(robot.getJointConfig(full))
                if nq == 2:
                    q_robot.append(np.cos(value))
                    q_robot.append(np.sin(value))
                elif nq == 1:
                    q_robot.append(value)

        # ! Kept the way to add the base root when we'll add it
        # r = robot.rankInConfiguration["tiago_pro/root_joint"]
        # q[r : r + 4] = self._tf_to_base_odom(self._odom)

        # pin.framesForwardKinematics(self._robot_model, self._robot_data, np.asarray(q_robot)) # ! unused in the message cause there is no vel and acc and effort?

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
