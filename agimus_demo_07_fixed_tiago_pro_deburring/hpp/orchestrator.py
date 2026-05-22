"""
HPP deburring orchestrator for TIAGo Pro — fixed base (right arm only).

Provides an interactive interface for step-by-step planning and execution:
    o = Orchestrator()
    o.plan()             # run HPP planner (generates qpg, qg, p1, p2, p3)
    o.execute()          # sample + publish trajectory to MPC
    o.plan_and_execute() # both in sequence

Run via orchestrator_node.py (sources both ros2_config.sh and hpp_config.sh).
"""

import os
import subprocess
import sys
import glob
import tempfile
import time
import numpy as np
import pinocchio as pin
import yaml

# ── Make agimus_msgs / rclpy importable from HPP environment ──────────────────
for _p in sorted(
    glob.glob("/home/gepetto/ros2_ws/install/*/lib/python3*/site-packages")
    + glob.glob("/home/gepetto/agimus_deps_ws/install/*/lib/python3*/site-packages")
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pyhpp.manipulation import Device, urdf
from pyhpp.manipulation import Graph, Problem, TransitionPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut, SplineGradientBased_bezier3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from agimus_msgs.msg import MpcInput, MpcEEInput
from sensor_msgs.msg import JointState


# ── Constants ─────────────────────────────────────────────────────────────────

_HPP_DIR    = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR    = os.path.join(_HPP_DIR, "..")
ROBOT_SRDF  = os.path.join(_HPP_DIR, "tiago_pro.srdf")
PYLONE_SRDF = os.path.join(_HPP_DIR, "pylone.srdf")
PYLONE_URDF = os.path.join(_PKG_DIR, "urdf", "pylone.urdf")
GROUND_SRDF = os.path.join(_HPP_DIR, "ground.srdf")
GROUND_URDF = os.path.join(_PKG_DIR, "urdf", "ground.urdf")

_CFG_FILE = os.path.join(_PKG_DIR, "config", "hpp_orchestrator_params.yaml")
with open(_CFG_FILE) as _f:
    _cfg = yaml.safe_load(_f)

DT         = _cfg["trajectory"]["dt"]
TIME_SCALE = _cfg["trajectory"]["time_scale"]

_PYLONE_POSE_FILE = os.path.join(_PKG_DIR, "config", "pylone_pose.yaml")

_h           = _cfg["handle"]
HANDLE_LINK  = _h["link"]
HANDLE_NAME  = _h["name"]
HANDLE_POS   = np.array(_h["position"])
HANDLE_CLEAR = _h["clearance"]

LEFT_ARM_TUCK  = _cfg["tuck"]["left_arm"]
RIGHT_ARM_TUCK = _cfg["tuck"]["right_arm"]

_w          = _cfg["weights"]
W_Q         = np.array(_w["w_q"])
W_QDOT      = np.array(_w["w_qdot"])
W_QDDOT     = np.array(_w["w_qddot"])
W_EFFORT      = np.array(_w["w_effort"])
W_COLLISION   = _w["w_collision"]
W_FRAME_TRANS = np.array(_w["w_frame_trans"])
W_FRAME_ROT   = np.array(_w["w_frame_rot"])


class _TrajectoryPublisherNode(Node):
    """One-shot ROS2 node that publishes pre-computed MpcInput messages."""

    def __init__(self, messages: list):
        super().__init__("hpp_trajectory_publisher")
        self._messages = messages
        self._idx = 0
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(MpcInput, "mpc_input", qos)
        self._timer = self.create_timer(DT, self._publish_next)
        self.get_logger().info(
            f"Publishing {len(self._messages)} trajectory points at {1/DT:.0f} Hz …"
        )
        self._done = False

    def _publish_next(self):
        if self._idx >= len(self._messages):
            if not self._done:
                self.get_logger().info("Trajectory fully published.")
                self._done = True
            self._timer.cancel()
            return
        self._pub.publish(self._messages[self._idx])
        self._idx += 1


class Orchestrator:
    """
    Interactive orchestrator for HPP deburring planning and MPC execution.
    Fixed-base version: only the right arm (7 DOF) is actuated.

    Usage (in IPython):
        o = Orchestrator()
        o.plan()
        o.execute()
    """

    def __init__(self, ros_node: Node = None):
        self._ros_node = ros_node
        self.p1 = self.p2 = self.p3 = None
        self._messages = None

        print("Loading HPP model …")
        self._setup_model()
        print("Building constraint graph …")
        self._setup_graph()
        print("Orchestrator ready.  Call plan() to start.\n")

    # ── Model setup ───────────────────────────────────────────────────────────

    @staticmethod
    def _fetch_robot_urdf(timeout: float = 10.0) -> str:
        """Return the URDF string from /robot_description (transient_local topic)."""
        from std_msgs.msg import String
        node = rclpy.create_node("_hpp_urdf_fetcher")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        urdf_str = None

        def _cb(msg):
            nonlocal urdf_str
            urdf_str = msg.data

        node.create_subscription(String, "/robot_description", _cb, qos)
        t0 = time.time()
        while urdf_str is None and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()

        if urdf_str is None:
            raise RuntimeError(
                f"Timed out after {timeout}s waiting for /robot_description. "
                "Is the simulation running?"
            )
        return urdf_str

    def _setup_model(self):
        urdf_str = self._fetch_robot_urdf()
        _tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w")
        _tmp.write(urdf_str)
        _tmp.close()

        robot = Device("tiago_pro")
        # anchor = fixed base, only arm joints have DOF
        urdf.loadModel(
            robot, 0, "tiago_pro", "anchor",
            f"file://{_tmp.name}",
            ROBOT_SRDF,
            pin.SE3.Identity(),
        )
        os.unlink(_tmp.name)
        urdf.loadModel(
            robot, 0, "ground", "anchor",
            GROUND_URDF,
            GROUND_SRDF,
            pin.SE3.Identity(),
        )
        urdf.loadModel(
            robot, 0, "pylone", "freeflyer",
            PYLONE_URDF,
            PYLONE_SRDF,
            pin.SE3.Identity(),
        )

        self.robot = robot
        model = robot.model()
        self.model = model

        def _idx(name):
            return model.joints[model.getJointId(name)].idx_q

        self._left_arm_idx  = _idx("tiago_pro/arm_left_1_joint")
        self._right_arm_idx = _idx("tiago_pro/arm_right_1_joint")
        self._pylone_idx    = _idx("pylone/root_joint")

        with open(_PYLONE_POSE_FILE) as _pf:
            _pc = yaml.safe_load(_pf)
        _px = _pc["pylone_x"]
        _py = _pc["pylone_y"]
        _pz = _pc["pylone_z"]
        _pq = _pc.get("pylone_quat", [0.0, 0.0, 0.0, 1.0])

        self._set_pylone_bounds(_px, _py, _pz)

        # Add handle: Rx(-90°) so handle Z = world +Y (into the hole)
        _R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        robot.addHandle(
            HANDLE_LINK, HANDLE_NAME,
            pin.SE3(_R, HANDLE_POS),
            HANDLE_CLEAR,
            6 * [True],
        )
        robot.handles()[HANDLE_NAME].approachingDirection = np.array([0, 0, 1])

        li = self._left_arm_idx
        ri = self._right_arm_idx
        pi = self._pylone_idx

        self.q_init = pin.neutral(model).copy()
        self._left_arm_lock_values = list(LEFT_ARM_TUCK)
        self.q_init[li:li+7] = LEFT_ARM_TUCK
        self.q_init[ri:ri+7] = RIGHT_ARM_TUCK
        self.q_init[pi:pi+3] = [_px, _py, _pz]
        self.q_init[pi+3:pi+7] = _pq

        self._pin_data = model.createData()
        ee_frame_name = "tiago_pro/gripper_right_tool_holder"
        self._ee_frame_id = model.getFrameId(ee_frame_name)
        if self._ee_frame_id == model.nframes:
            raise RuntimeError(f"Frame '{ee_frame_name}' not found in model")

    def _set_pylone_bounds(self, x, y, z, margin: float = 0.001):
        """Lock pylone position with tight bounds so HPP cannot move it."""
        self.robot.setJointBounds("pylone/root_joint", [
            x - margin, x + margin,
            y - margin, y + margin,
            z - margin, z + margin,
            -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"),
        ])

    # ── Constraint graph setup ─────────────────────────────────────────────────

    def _setup_graph(self):
        robot = self.robot
        model = self.model

        problem = Problem(robot)
        graph   = Graph("robot", robot, problem)
        factory = ConstraintGraphFactory(graph)
        graph.maxIterations(40)
        graph.errorThreshold(1e-3)
        factory.setGrippers(["tiago_pro/gripper"])
        factory.setObjects(["pylone"], [[HANDLE_NAME]], [[]])
        factory.generate()

        self._transition_approach = graph.getTransition(
            f"tiago_pro/gripper > {HANDLE_NAME} | f_01"
        )
        self._transition_insert = graph.getTransition(
            f"tiago_pro/gripper > {HANDLE_NAME} | f_12"
        )

        _cts = ComparisonTypes()
        _cts[:] = [ComparisonType.EqualToZero]

        def _lock(joint_name, value):
            j = model.joints[model.getJointId(joint_name)]
            if j.nq == 2 and j.nv == 1:
                locked_val = np.array([np.cos(value), np.sin(value)])
            else:
                locked_val = np.array([value])
            return LockedJoint(robot, joint_name, locked_val, _cts)

        locked = []
        locked.append(_lock("tiago_pro/torso_lift_joint", 0.0))
        for wheel in ["wheel_front_left_joint", "wheel_front_right_joint",
                      "wheel_rear_left_joint",  "wheel_rear_right_joint"]:
            locked.append(_lock(f"tiago_pro/{wheel}", 0.0))
        for i, val in enumerate(self._left_arm_lock_values):
            locked.append(_lock(f"tiago_pro/arm_left_{i+1}_joint", val))
        for name in ["gripper_left_finger_joint",
                     "gripper_left_inner_finger_left_joint",
                     "gripper_left_fingertip_left_joint",
                     "gripper_left_inner_finger_right_joint",
                     "gripper_left_fingertip_right_joint",
                     "gripper_left_outer_finger_right_joint",
                     "gripper_right_tool_mount_joint"]:
            locked.append(_lock(f"tiago_pro/{name}", 0.0))
        locked.append(_lock("tiago_pro/head_1_joint", 0.0))
        locked.append(_lock("tiago_pro/head_2_joint", 0.0))

        graph.addNumericalConstraintsToGraph(locked)

        sm = SecurityMargins(problem, factory, ["tiago_pro", "pylone"], robot)
        sm.setSecurityMarginBetween("tiago_pro", "pylone", 0.05)
        sm.apply()

        for jname in model.names:
            if "tiago_pro" in jname:
                graph.setSecurityMarginForTransition(
                    self._transition_insert, jname, "pylone/root_joint", float("-inf")
                )

        graph.initialize()

        self.problem = problem
        self.graph   = graph

    # ── Planning ──────────────────────────────────────────────────────────────

    def plan(self, max_attempts: int = 50) -> bool:
        """Generate qpg (collision-free), qg, and plan p1, p2, p3."""
        shooter = self.problem.configurationShooter()
        qpg = None
        for i in range(max_attempts):
            q = shooter.shoot()
            res, q_cand, err = self.graph.generateTargetConfig(
                self._transition_approach, self.q_init, q
            )
            if not res:
                continue
            pv = self._transition_approach.pathValidation()
            res, _ = pv.validateConfiguration(q_cand)
            if not res:
                continue
            qpg = q_cand
            print(f"  qpg found at attempt {i}, err={err:.2e}")
            break

        if qpg is None:
            print(f"Failed to find collision-free qpg in {max_attempts} attempts.")
            return False

        res, qg, err = self.graph.generateTargetConfig(
            self._transition_insert, qpg, qpg
        )
        print(f"  qg: res={res}, err={err:.2e}")
        if not res:
            print("Failed to generate qg.")
            return False

        self.problem.constraintGraph(self.graph)
        planner = TransitionPlanner(self.problem)
        planner.maxIterations(1000)

        planner.setTransition(self._transition_approach)
        q_goal = np.zeros((1, self.robot.configSize()), order='F')
        q_goal[0, :] = qpg
        print("Planning p1 (approach) …")
        p1 = planner.planPath(self.q_init, q_goal, True)
        print("  p1 found.")

        shortcut   = RandomShortcut(self.problem)
        spline_opt = SplineGradientBased_bezier3(self.problem)

        try:
            for i in range(3):
                p1_new = shortcut.optimize(p1)
                tr_before = p1.timeRange()
                tr_after  = p1_new.timeRange()
                dt = (tr_before.second - tr_before.first) - (tr_after.second - tr_after.first)
                p1 = p1_new
                print(f"  p1 shortcut pass {i+1}/3: {tr_after.second - tr_after.first:.2f} s  (−{dt:.2f} s)")
                if dt < 1e-3:
                    break
        except Exception as e:
            print(f"  p1 shortcut failed: {e}")
        try:
            p1 = spline_opt.optimize(p1)
            tr = p1.timeRange()
            print(f"  p1 spline: {tr.second - tr.first:.2f} s")
        except Exception as e:
            print(f"  p1 spline optimisation failed: {e}")

        planner.setTransition(self._transition_insert)
        q_goal[0, :] = qg
        print("Planning p2 (insertion) …")
        p2 = planner.planPath(qpg, q_goal, True)
        print("  p2 found.")

        try:
            p2 = shortcut.optimize(p2)
            tr = p2.timeRange()
            print(f"  p2 shortcut: {tr.second - tr.first:.2f} s")
        except Exception as e:
            print(f"  p2 shortcut failed: {e}")
        try:
            p2 = spline_opt.optimize(p2)
            tr = p2.timeRange()
            print(f"  p2 spline: {tr.second - tr.first:.2f} s")
        except Exception as e:
            print(f"  p2 spline optimisation failed: {e}")

        p3 = p2.reverse()
        print("  p3 ready (retraction, reversed from optimised p2).")

        self.p1  = p1
        self.p2  = p2
        self.p3  = p3
        self.qpg = qpg
        self.qg  = qg
        return True

    # ── Path sampling ─────────────────────────────────────────────────────────

    def _extract_active_q(self, q_full):
        """Extract the 7 right-arm joint positions from a full HPP config."""
        q = np.array(q_full)
        ri = self._right_arm_idx
        return q[ri:ri+7].copy()

    def _active_velocity(self, q1, q2, dt):
        """Finite-difference velocity for the 7 right-arm joints."""
        return (q2 - q1) / dt

    def _sample_path(self, path):
        tr = path.timeRange()
        t_min, t_max = tr.first, tr.second
        n     = max(2, int((t_max - t_min) * TIME_SCALE / DT))
        times = np.linspace(t_min, t_max, n)
        q_list = [self._extract_active_q(path.eval(t)[0]) for t in times]
        q_arr  = np.array(q_list)

        dq_list = [self._active_velocity(q_arr[i], q_arr[i+1], DT)
                   for i in range(len(q_arr) - 1)]
        dq_list.append(dq_list[-1])
        dq_arr = np.array(dq_list)

        ddq_list = [(dq_arr[i+1] - dq_arr[i]) / DT
                    for i in range(len(dq_arr) - 1)]
        ddq_list.append(ddq_list[-1])
        ddq_arr = np.array(ddq_list)

        return q_arr, dq_arr, ddq_arr

    def _fk_ee(self, q_arm: np.ndarray) -> pin.SE3:
        q_full = pin.neutral(self.model)
        ri = self._right_arm_idx
        q_full[ri:ri + 7] = q_arm
        pin.forwardKinematics(self.model, self._pin_data, q_full)
        pin.updateFramePlacements(self.model, self._pin_data)
        return self._pin_data.oMf[self._ee_frame_id].copy()

    def _build_msg(self, q, dq, ddq, msg_id):
        msg = MpcInput()
        msg.id           = msg_id
        msg.q            = q.tolist()
        msg.qdot         = dq.tolist()
        msg.qddot        = ddq.tolist()
        msg.robot_effort = np.zeros(7).tolist()
        msg.w_q                   = W_Q.tolist()
        msg.w_qdot                = W_QDOT.tolist()
        msg.w_qddot               = W_QDDOT.tolist()
        msg.w_robot_effort        = W_EFFORT.tolist()
        msg.w_collision_avoidance = W_COLLISION

        T_ee = self._fk_ee(q)
        quat = pin.Quaternion(T_ee.rotation)
        ee_input = MpcEEInput()
        ee_input.frame_id = "gripper_right_tool_holder"
        ee_input.pose.position.x = float(T_ee.translation[0])
        ee_input.pose.position.y = float(T_ee.translation[1])
        ee_input.pose.position.z = float(T_ee.translation[2])
        ee_input.pose.orientation.x = float(quat.x)
        ee_input.pose.orientation.y = float(quat.y)
        ee_input.pose.orientation.z = float(quat.z)
        ee_input.pose.orientation.w = float(quat.w)
        ee_input.w_pose = list(np.concatenate([W_FRAME_TRANS, W_FRAME_ROT]))
        msg.ee_inputs = [ee_input]
        return msg

    def _build_messages(self, paths: list, n_hold: int = 200) -> list:
        """
        paths : list of (path, label) tuples to concatenate.
        n_hold: number of hold waypoints appended at the end.
        """
        msgs = []
        idx = 0
        for path, label in paths:
            q_arr, dq_arr, ddq_arr = self._sample_path(path)
            print(f"  {label}: {len(q_arr)} waypoints")
            for q, dq, ddq in zip(q_arr, dq_arr, ddq_arr):
                msgs.append(self._build_msg(q, dq, ddq, idx))
                idx += 1
        q_final  = msgs[-1].q
        dq_zero  = np.zeros(len(msgs[-1].qdot)).tolist()
        ddq_zero = np.zeros(len(msgs[-1].qddot)).tolist()
        for _ in range(n_hold):
            msg = self._build_msg(np.array(q_final), np.array(dq_zero), np.array(ddq_zero), idx)
            msgs.append(msg)
            idx += 1
        print(f"  {len(msgs)} MpcInput messages total ({n_hold} hold points appended).")
        return msgs

    # ── Execution ─────────────────────────────────────────────────────────────

    def execute(self, paths=None):
        """
        Sample and publish MpcInput messages to the controller.

        paths : list of Path objects to execute in sequence.
                Defaults to [p1, p2, p3].
        """
        if self.p1 is None:
            print("No path available — run plan() first.")
            return

        _labels = {id(self.p1): "p1 (approach)",
                   id(self.p2): "p2 (insertion)",
                   id(self.p3): "p3 (retraction)"}
        if paths is None:
            paths = [self.p1, self.p2, self.p3]
        named = [(p, _labels.get(id(p), f"path_{i+1}")) for i, p in enumerate(paths)]

        print("Sampling trajectories …")
        self._messages = self._build_messages(named)

        if self._ros_node is None:
            self._ros_node = _TrajectoryPublisherNode(self._messages)
        else:
            self._ros_node._messages = self._messages
            self._ros_node._idx      = 0
            self._ros_node._done     = False
            self._ros_node._timer    = self._ros_node.create_timer(
                DT, self._ros_node._publish_next
            )

        print("Publishing trajectory …")
        try:
            while not self._ros_node._done:
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                time.sleep(DT)
        except KeyboardInterrupt:
            print("\nExecution interrupted.")

    # ── Pose comparison ───────────────────────────────────────────────────────

    def _read_robot_config(self, timeout: float = 5.0):
        """Return a full HPP config vector from the current robot joint states."""
        _own_node = False
        if self._ros_node is None:
            self._ros_node = rclpy.create_node("hpp_read_config_node")
            _own_node = True

        joint_state = [None]
        sub = self._ros_node.create_subscription(
            JointState, "/joint_states",
            lambda m: joint_state.__setitem__(0, m), 10)

        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            if joint_state[0] is not None:
                break
        self._ros_node.destroy_subscription(sub)

        if _own_node:
            self._ros_node.destroy_node()
            self._ros_node = None

        if joint_state[0] is None:
            raise RuntimeError("Timeout reading robot state from /joint_states.")

        js_map = dict(zip(joint_state[0].name, joint_state[0].position))
        q = pin.neutral(self.model).copy()
        ri = self._right_arm_idx
        for i in range(1, 8):
            key = f"arm_right_{i}_joint"
            q[ri + i - 1] = js_map.get(key, js_map.get(f"tiago_pro/{key}", 0.0))
        return q

    def compare_pose(self, q_ref=None, timeout: float = 5.0):
        """
        Compare a reference configuration with the current robot state,
        at the gripper_right_tool_holder frame.

        q_ref : HPP Path object, full config vector, or None (defaults to qg).
                Examples:
                  o.compare_pose()      # vs qg (insertion goal)
                  o.compare_pose(o.p1)  # vs end of p1
        """
        if self._ros_node is None:
            print("No ROS node available — run execute() first.")
            return
        if q_ref is None:
            if self.qg is None:
                print("No qg available — run plan() first.")
                return
            q_ref = np.array(self.qg)
        elif hasattr(q_ref, "timeRange"):
            q_ref = np.array(q_ref.eval(q_ref.timeRange().second)[0])

        print("Reading current robot state …")
        try:
            q_actual = self._read_robot_config(timeout)
        except RuntimeError as e:
            print(f"compare_pose: {e}")
            return

        data_ref = self.model.createData()
        data_act = self.model.createData()
        pin.forwardKinematics(self.model, data_ref, q_ref)
        pin.updateFramePlacements(self.model, data_ref)
        pin.forwardKinematics(self.model, data_act, q_actual)
        pin.updateFramePlacements(self.model, data_act)

        T_ref = data_ref.oMf[self._ee_frame_id]
        T_act = data_act.oMf[self._ee_frame_id]
        delta = T_ref.inverse() * T_act
        pos_err_mm  = np.linalg.norm(delta.translation) * 1000.0
        rot_err_deg = np.degrees(np.linalg.norm(pin.log3(delta.rotation)))

        ri = self._right_arm_idx
        print(f"\n{'='*58}")
        print(f"  Pose comparison  (reference vs actual robot state)")
        print(f"{'='*58}")
        print(f"  EE planned [m] : {np.round(T_ref.translation, 4)}")
        print(f"  EE actual  [m] : {np.round(T_act.translation, 4)}")
        print(f"  Position error : {pos_err_mm:.1f} mm")
        print(f"  Rotation error : {rot_err_deg:.2f} °")
        print(f"\n  Per-joint error — right arm [rad / °]:")
        for i in range(7):
            e = q_ref[ri + i] - q_actual[ri + i]
            print(f"    arm_right_{i+1}_joint : {e:+.4f} rad  ({np.degrees(e):+.2f}°)")
        print(f"{'='*58}\n")

    # ── Visualisation ─────────────────────────────────────────────────────────

    def init_viewer(self, open: bool = True):
        from pyhpp_viser import Viewer
        self._viewer = Viewer(self.robot)
        self._viewer.initViewer(open=open, loadModel=True)
        self._viewer.setProblem(self.problem)
        self._viewer.setGraph(self.graph)
        self._viewer(self.q_init)
        print("Viser viewer ready.  Use o.view(q) or o.play(path).")

    def view(self, q=None):
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        q = np.array(q) if q is not None else self.q_init
        self._viewer(q)
        pin.forwardKinematics(self.model, self._pin_data, q)
        pin.updateFramePlacements(self.model, self._pin_data)
        T = self._pin_data.oMf[self._ee_frame_id]
        rpy = pin.rpy.matrixToRpy(T.rotation)
        print(f"EE position (xyz) [m]: {np.round(T.translation, 4).tolist()}")
        print(f"EE rotation (rpy) [°]: {np.round(np.degrees(rpy), 1).tolist()}")

    def play(self, path, n=100, dt=0.05):
        """Play a path in Viser by sampling n configurations."""
        import time as _time
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        try:
            self._viewer.loadPath(path)
        except Exception:
            pass
        t0 = path.timeRange().first
        tf = path.timeRange().second
        for i in range(n):
            t = t0 + i * (tf - t0) / (n - 1)
            q = path.eval(t)[0]
            self._viewer(q)
            _time.sleep(dt)

    # ── Robot state sync ──────────────────────────────────────────────────────

    def sync_from_robot(self, timeout: float = 5.0):
        """Update q_init from the current robot joint state (arm only)."""
        if self._ros_node is None:
            self._ros_node = rclpy.create_node("hpp_sync_node")
            _own_node = True
        else:
            _own_node = False

        joint_state = [None]
        joint_sub = self._ros_node.create_subscription(
            JointState, "/joint_states",
            lambda msg: joint_state.__setitem__(0, msg), 10
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            if joint_state[0] is not None:
                break

        self._ros_node.destroy_subscription(joint_sub)
        if _own_node:
            self._ros_node.destroy_node()
            self._ros_node = None

        if joint_state[0] is None:
            print("sync_from_robot: timeout — could not receive /joint_states")
            return

        js = joint_state[0]
        js_map = dict(zip(js.name, js.position))

        def _read_arm(side):
            return np.array([
                js_map.get(f"arm_{side}_{i}_joint",
                           js_map.get(f"tiago_pro/arm_{side}_{i}_joint", 0.0))
                for i in range(1, 8)
            ])

        ri = self._right_arm_idx
        li = self._left_arm_idx

        left_arm  = _read_arm("left")
        right_arm = _read_arm("right")

        self.q_init[ri:ri+7] = right_arm
        self.q_init[li:li+7] = left_arm

        self._left_arm_lock_values = left_arm.tolist()
        print("  Rebuilding constraint graph with synced left arm …")
        self._setup_graph()

        print(
            f"sync_from_robot: "
            f"right_arm={np.round(right_arm, 3).tolist()}  "
            f"left_arm={np.round(left_arm, 3).tolist()}"
        )

    # ── Pylone pose update ────────────────────────────────────────────────────

    def reload_pylone_pose(self) -> None:
        """Read pylone pose from config/pylone_pose.yaml and update q_init."""
        if not os.path.exists(_PYLONE_POSE_FILE):
            print(f"No pylone pose file found at {_PYLONE_POSE_FILE}.")
            print("Run scripts/localize_pylone.py first.")
            return
        with open(_PYLONE_POSE_FILE) as f:
            cfg = yaml.safe_load(f)
        t = [cfg["pylone_x"], cfg["pylone_y"], cfg["pylone_z"]]
        q = cfg.get("pylone_quat", [0.0, 0.0, 0.0, 1.0])
        self.update_pylone_pose(t, q)

    def update_pylone_pose(self, t: list, q: list = None) -> None:
        """Update pylone pose in q_init and display in Viser if open.

        t: [x, y, z] in base_link frame
        q: [qx, qy, qz, qw] orientation (default: identity)
        """
        t = np.array(t)
        q = np.array(q) if q is not None else np.array([0., 0., 0., 1.])
        self._set_pylone_bounds(t[0], t[1], t[2])
        pi = self._pylone_idx
        self.q_init[pi:pi+3] = t
        self.q_init[pi+3:pi+7] = q
        if hasattr(self, "_viewer"):
            self._viewer(self.q_init)
        print(f"Pylone pose updated: t={np.round(t, 4).tolist()}, q={np.round(q, 4).tolist()}.")
        if not hasattr(self, "_viewer"):
            print("Call o.init_viewer() to visualize.")

    # ── Mocap (Qualisys) ─────────────────────────────────────────────────────

    _QUALISYS_IP    = "140.93.1.100"
    _MOCAP_BODIES   = {"pylone": 0, "tiago_endEffector": 1, "tiago_base": 2}
    _MOCAP_BASE_IDX = 2  # tiago_base = reference frame

    def connect_mocap(self, ip: str = _QUALISYS_IP) -> None:
        """Start the Qualisys mocap subprocess.  Call this once before
        compare_mocap() or localize_pylone_from_mocap()."""
        _scripts_dir = os.path.join(_PKG_DIR, "scripts")
        if _scripts_dir not in sys.path:
            sys.path.insert(0, _scripts_dir)
        from qualisys import QualisysClient  # noqa: PLC0415
        self._qc = QualisysClient(ip=ip, bodies=self._MOCAP_BODIES)
        time.sleep(1.0)   # let the subprocess receive its first packet
        print(f"Mocap connected to {ip}.")

    def disconnect_mocap(self) -> None:
        """Stop the Qualisys subprocess."""
        if hasattr(self, "_qc"):
            self._qc.stop()
            del self._qc
            print("Mocap disconnected.")

    def _mocap_se3(self, idx: int) -> pin.SE3:
        """Return pinocchio SE3 for Qualisys body index *idx*."""
        pos  = self._qc.getPositions()[idx]       # (3,) in metres
        quat = self._qc.getOrientationQuats()[idx] # (4,) [qx qy qz qw]
        return pin.XYZQUATToSE3(np.concatenate([pos, quat]))

    def compare_mocap(self, timeout: float = 5.0) -> None:
        """Compare mocap poses vs robot FK, relative to base_link / tiago_base.

        Prints position error (mm) and rotation error (deg) for:
          • end effector  (tiago_endEffector  ↔  gripper_right_tool_holder)
          • pylone        (pylone             ↔  q_init pylone pose)

        Requires connect_mocap() and a running ROS node (execute() or
        rclpy.init() already called).
        """
        if not hasattr(self, "_qc"):
            print("No mocap client — call connect_mocap() first.")
            return

        print("Reading current robot joint state …")
        try:
            q_actual = self._read_robot_config(timeout)
        except RuntimeError as e:
            print(f"compare_mocap: {e}")
            return

        # ── Mocap relative poses (w.r.t. tiago_base) ──────────────────────
        T_mocap_base = self._mocap_se3(self._MOCAP_BASE_IDX)
        T_rel_ee_mocap  = T_mocap_base.inverse() * self._mocap_se3(1)  # tiago_endEffector
        T_rel_pyl_mocap = T_mocap_base.inverse() * self._mocap_se3(0)  # pylone

        # ── Robot poses ────────────────────────────────────────────────────
        T_rel_ee_robot = self._fk_ee(self._extract_active_q(q_actual))

        pi = self._pylone_idx
        T_rel_pyl_robot = pin.XYZQUATToSE3(
            np.concatenate([self.q_init[pi:pi+3], self.q_init[pi+3:pi+7]])
        )

        # ── Helpers ────────────────────────────────────────────────────────
        def _breakdown(T_a: pin.SE3, T_b: pin.SE3):
            """Return signed per-axis errors: (dt_mm[3], drpy_deg[3])."""
            delta   = T_a.inverse() * T_b
            dt_mm   = delta.translation * 1e3                          # [dx, dy, dz] mm
            drpy    = np.degrees(pin.utils.matrixToRpy(delta.rotation)) # [dr, dp, dy] deg
            return dt_mm, drpy

        def _print_body(label: str, T_mocap: pin.SE3, T_robot: pin.SE3) -> None:
            t_m  = T_mocap.translation
            t_r  = T_robot.translation
            rpy_m = np.degrees(pin.utils.matrixToRpy(T_mocap.rotation))
            rpy_r = np.degrees(pin.utils.matrixToRpy(T_robot.rotation))
            dt_mm, drpy = _breakdown(T_mocap, T_robot)
            norm_t = np.linalg.norm(dt_mm)
            norm_r = np.linalg.norm(drpy)
            flag   = "✓" if norm_t < 20 and norm_r < 5 else "!"

            print(f"\n  {label}")
            print(f"  {'':4s}{'':12s}  {'x':>9s}  {'y':>9s}  {'z':>9s}")
            print(f"  {'':4s}{'mocap [m]':12s}  {t_m[0]:>+9.4f}  {t_m[1]:>+9.4f}  {t_m[2]:>+9.4f}")
            print(f"  {'':4s}{'robot [m]':12s}  {t_r[0]:>+9.4f}  {t_r[1]:>+9.4f}  {t_r[2]:>+9.4f}")
            print(f"  {'':4s}{'Δ [mm]':12s}  {dt_mm[0]:>+9.2f}  {dt_mm[1]:>+9.2f}  {dt_mm[2]:>+9.2f}  (|Δ|={norm_t:.1f} mm)  {flag}")
            print(f"")
            print(f"  {'':4s}{'':12s}  {'roll':>9s}  {'pitch':>9s}  {'yaw':>9s}")
            print(f"  {'':4s}{'mocap [°]':12s}  {rpy_m[0]:>+9.2f}  {rpy_m[1]:>+9.2f}  {rpy_m[2]:>+9.2f}")
            print(f"  {'':4s}{'robot [°]':12s}  {rpy_r[0]:>+9.2f}  {rpy_r[1]:>+9.2f}  {rpy_r[2]:>+9.2f}")
            print(f"  {'':4s}{'Δ [°]':12s}  {drpy[0]:>+9.2f}  {drpy[1]:>+9.2f}  {drpy[2]:>+9.2f}  (|Δ|={norm_r:.2f}°)  {flag}")

        print(f"\n{'='*66}")
        print(f"  Mocap vs Robot — poses relative to base_footprint / tiago_base")
        print(f"{'='*66}")
        _print_body(
            "End effector  (tiago_endEffector ↔ gripper_right_tool_holder)",
            T_rel_ee_mocap, T_rel_ee_robot,
        )
        print(f"\n  {'-'*62}")
        _print_body(
            "Pylone  (mocap ↔ pose localisée)",
            T_rel_pyl_mocap, T_rel_pyl_robot,
        )
        print(f"\n{'='*66}\n")

    def localize_pylone_from_mocap(self) -> None:
        """Set the pylone pose in the orchestrator from the current mocap reading.

        Reads the mocap pose of 'pylone' relative to 'tiago_base', updates
        q_init and saves the result to config/pylone_pose.yaml.

        This is an alternative to the manual pointing procedure in
        scripts/localize_pylone.py.
        """
        if not hasattr(self, "_qc"):
            print("No mocap client — call connect_mocap() first.")
            return

        T_mocap_base = self._mocap_se3(self._MOCAP_BASE_IDX)
        T_mocap_pyl  = self._mocap_se3(0)
        T_rel        = T_mocap_base.inverse() * T_mocap_pyl

        t    = T_rel.translation.tolist()
        qpin = pin.Quaternion(T_rel.rotation)
        q    = [float(qpin.x), float(qpin.y), float(qpin.z), float(qpin.w)]

        self.update_pylone_pose(t, q)

        result = {
            "pylone_x":    round(t[0], 4),
            "pylone_y":    round(t[1], 4),
            "pylone_z":    round(t[2], 4),
            "pylone_quat": [round(v, 6) for v in q],
        }
        with open(_PYLONE_POSE_FILE, "w") as _f:
            yaml.dump(result, _f, default_flow_style=False)
        print(f"Pylone pose saved to {_PYLONE_POSE_FILE}")

    # ── Controller activation ─────────────────────────────────────────────────

    def activate_lfc(self) -> None:
        """Switch arm_right_controller → LFC + JSE. Call this when ready to move."""
        print("Activating LFC controllers …")
        subprocess.run(
            [
                "ros2", "control", "switch_controllers",
                "--deactivate", "arm_right_controller",
                "--activate",
                "linear_feedback_controller",
                "joint_state_estimator",
            ],
            check=True,
        )
        print("LFC controllers active.")

    def deactivate_lfc(self) -> None:
        """Switch LFC + JSE → arm_right_controller. Call this to return to position control."""
        print("Deactivating LFC controllers …")
        subprocess.run(
            [
                "ros2", "control", "switch_controllers",
                "--deactivate",
                "linear_feedback_controller",
                "joint_state_estimator",
                "--activate", "arm_right_controller",
            ],
            check=True,
        )
        print("arm_right_controller active.")

    # ── Combined ──────────────────────────────────────────────────────────────

    def plan_and_execute(self, max_attempts: int = 50):
        """Plan then immediately execute."""
        if self.plan(max_attempts=max_attempts):
            self.execute()
