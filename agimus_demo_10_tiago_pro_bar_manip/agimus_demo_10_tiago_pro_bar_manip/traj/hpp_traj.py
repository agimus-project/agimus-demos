from math import sqrt
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from pyhpp.manipulation import Device, Graph, Problem
from pyhpp.manipulation import urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint

from agimus_demo_10_tiago_pro_bar_manip.planner.path_planner import PathPlanner


class BaseObject:
    """Wrapper to describe a URDF/SRDF object for HPP."""

    def __init__(self, root_joint_type: str, urdf_path: str, srdf_path: str, name: str):
        self.rootJointType = root_joint_type
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


class HPPPathGenerator:
    def __init__(
        self,
        urdf_str: str,
        srdf_str: str,
        robot_model: pin.Model,
        handle_config: Path,
        handle_object: BaseObject,
        plate_object: BaseObject,
        table_object: BaseObject,
        ocp_dt: float,
        robot_name: str,
        logger,
    ) -> None:
        self._ocp_dt = ocp_dt
        self._robot_name = robot_name
        self._logger = logger
        self._robot_model = robot_model

        self._handle_object = handle_object
        self._plate_object = plate_object
        self._table_object = table_object

        with open(handle_config, "r") as f:
            self._handle_config_file = yaml.safe_load(f)
        self._handles_config: dict[str, list] = {}
        for gname, config in self._handle_config_file.get("grippers", {}).items():
            self._handles_config[gname] = config.get("handles", [])

        # ── HPP Device ────────────────────────────────────────────────────────
        robot = Device(f"{robot_name}-manip")
        urdf.loadModelFromString(
            robot,
            0,
            robot_name,
            "planar",
            urdf_str,
            srdf_str,
            pin.SE3.Identity(),
        )
        for obj in (handle_object, plate_object, table_object):
            urdf.loadModel(
                robot,
                0,
                obj.name,
                obj.rootJointType,
                obj.urdfFilename,
                obj.srdfFilename,
                pin.SE3.Identity(),
            )

        self.robot = robot
        self._pin_model = robot.model()

        # ── Joint bounds ──────────────────────────────────────────────────────
        self.robot.setJointBounds(
            f"{handle_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0.0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        self.robot.setJointBounds(
            f"{plate_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0.0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        self.robot.setJointBounds(
            f"{robot_name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, -1, 1, -1, 1],
        )

        # ── Problem & Graph ───────────────────────────────────────────────────
        self._problem = Problem(self.robot)
        self.graph = Graph("robot", self.robot, self._problem)
        self._problem.constraintGraph(self.graph)

        self.graph.errorThreshold(1e-2)
        self.graph.maxIterations(40)

        # ── Locked joints ─────────────────────────────────────────────────────
        self._locked_grippers = self._create_locked_grippers(robot_name)
        self._locked_head = self._create_locked_head(robot_name)
        self._locked_wheels = self._create_locked_wheels(robot_name)
        self._locked_plate = self._create_locked_plate()
        self._locked_base_mobility = self._create_locked_base_mobility(robot_name)
        self._locked_arms, self._locked_torso = self._create_locked_arms_and_torso(
            robot_name
        )

        # ── Grippers & handles ────────────────────────────────────────────────
        c = sqrt(2) / 2
        pose_gripper = pin.XYZQUATToSE3(
            np.array([0, 0, 0.19, 0, -c, 0, c], dtype=float)
        )
        pose_hd_left = pin.XYZQUATToSE3(
            np.array([0, 0.01, -0.25, 0, 0, -c, c], dtype=float)
        )
        pose_hd_right = pin.XYZQUATToSE3(
            np.array([0, 0.01, 0.25, 0, 0, -c, c], dtype=float)
        )
        self.define_gripper(
            self.robot,
            f"{robot_name}/arm_left_7_link",
            f"{robot_name}/left",
            pose_gripper,
        )
        self.define_gripper(
            self.robot,
            f"{robot_name}/arm_right_7_link",
            f"{robot_name}/right",
            pose_gripper,
        )
        self.define_handle(
            self.robot,
            f"{handle_object.name}/bar_base_link",
            f"{handle_object.name}/left",
            pose_hd_left,
        )
        self.define_handle(
            self.robot,
            f"{handle_object.name}/bar_base_link",
            f"{handle_object.name}/right",
            pose_hd_right,
        )

        # ── Build constraint graph & call graph.initialize() ─────────────────
        self._generate_constraint_graph()
        # graph.initialize() is the last call inside _generate_constraint_graph()

        self._logger.info("Constraint graph built and initialized.")

        self._path_planner = PathPlanner(self._problem, self.graph, self.robot)

        self.init_viewer()

    # ── Locked joint helpers ──────────────────────────────────────────────────

    def _make_cts(self, n: int) -> ComparisonTypes:
        cts = ComparisonTypes()
        cts[:] = [ComparisonType.EqualToZero] * n
        return cts

    def _lock(self, joint_name: str, value: list) -> LockedJoint:
        val = np.array(value, dtype=float)
        return LockedJoint(self.robot, joint_name, val, self._make_cts(len(val)))

    def _create_locked_grippers(self, robot_name):
        locked = []
        values = {
            f"{robot_name}/gripper_left_finger_joint": 0.05,
            f"{robot_name}/gripper_left_inner_finger_left_joint": -0.05,
            f"{robot_name}/gripper_left_fingertip_left_joint": 0.05,
            f"{robot_name}/gripper_left_finger_right_joint": 0.0,
            f"{robot_name}/gripper_left_inner_finger_right_joint": -0.05,
            f"{robot_name}/gripper_left_fingertip_right_joint": 0.05,
            f"{robot_name}/gripper_left_outer_finger_left_joint": -0.05,
            f"{robot_name}/gripper_left_outer_finger_right_joint": -0.05,
            f"{robot_name}/gripper_right_finger_joint": 0.05,
            f"{robot_name}/gripper_right_inner_finger_left_joint": -0.05,
            f"{robot_name}/gripper_right_fingertip_left_joint": 0.05,
            f"{robot_name}/gripper_right_finger_right_joint": 0.0,
            f"{robot_name}/gripper_right_inner_finger_right_joint": -0.05,
            f"{robot_name}/gripper_right_fingertip_right_joint": 0.05,
            f"{robot_name}/gripper_right_outer_finger_left_joint": -0.05,
            f"{robot_name}/gripper_right_outer_finger_right_joint": -0.05,
        }
        for joint, value in values.items():
            lj = self._lock(joint, [value])
            self._problem.setConstantRightHandSide(lj, False)
            locked.append(lj)
        return locked

    def _create_locked_head(self, robot_name):
        locked = []
        for joint, value in {
            f"{robot_name}/head_1_joint": 0.0,
            f"{robot_name}/head_2_joint": 0.0,
        }.items():
            lj = self._lock(joint, [value])
            self._problem.setConstantRightHandSide(lj, True)
            locked.append(lj)
        return locked

    def _create_locked_wheels(self, robot_name: str) -> list:
        locked = []
        for joint in [
            f"{robot_name}/wheel_front_left_joint",
            f"{robot_name}/wheel_front_right_joint",
            f"{robot_name}/wheel_rear_left_joint",
            f"{robot_name}/wheel_rear_right_joint",
        ]:
            lj = self._lock(joint, [1.0, 0.0])  # SO2: [cos(0), sin(0)]
            self._problem.setConstantRightHandSide(lj, True)
            locked.append(lj)
        return locked

    def _create_locked_plate(self):
        lj = self._lock(
            f"{self._plate_object.name}/root_joint",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self._problem.setConstantRightHandSide(lj, False)
        return [lj]

    def _create_locked_base_mobility(self, robot_name):
        lj = self._lock(f"{robot_name}/root_joint", [0.0, 0.0, 1.0, 0.0])
        self._problem.setConstantRightHandSide(lj, False)
        return [lj]

    def _create_locked_arms_and_torso(self, robot_name: str) -> tuple[list, list]:
        locked_arms, seen = [], set()
        for j in self.robot.model().names:
            if not j.startswith(f"{robot_name}/"):
                continue
            if (
                j.startswith(f"{robot_name}/head")
                or j.startswith(f"{robot_name}/wheel")
                or "gripper" in j
                or j == f"{robot_name}/torso_lift_joint"
                or j == f"{robot_name}/root_joint"
            ):
                continue
            if j in seen:
                continue
            seen.add(j)
            lj = self._lock(j, [0.0])
            self._problem.setConstantRightHandSide(lj, False)
            locked_arms.append(lj)
        lj_torso = self._lock(f"{robot_name}/torso_lift_joint", [0.0])
        self._problem.setConstantRightHandSide(lj_torso, False)
        return locked_arms, [lj_torso]

    # ── Constraint graph ──────────────────────────────────────────────────────

    def _generate_constraint_graph(self) -> None:
        handle_object_name = self._handle_object.name
        gripper_names = list(self._handles_config.keys())
        primary_gripper = gripper_names[0]
        primary_handle = self._handles_config[primary_gripper][0]["name"]

        bar_placement = self.graph.createPlacementConstraint(
            f"place_{handle_object_name}",
            [f"{handle_object_name}/bottom"],
            [
                f"{self._table_object.name}/reinforcment_bar_support",
                f"{self._plate_object.name}/top",
            ],
        )
        place_complement_obj = bar_placement[1]

        factory = ConstraintGraphFactory(self.graph)
        factory.setGrippers([primary_gripper])
        factory.environmentContacts(
            [
                f"{self._table_object.name}/reinforcment_bar_support",
                f"{self._plate_object.name}/top",
            ]
        )
        factory.setObjects(
            [handle_object_name],
            [[primary_handle]],
            [[f"{handle_object_name}/bottom"]],
        )
        factory.generate()

        # ── Bimanual secondary gripper ────────────────────────────────────────
        if len(gripper_names) == 2:
            secondary_gripper = gripper_names[1]
            secondary_handle = self._handles_config[secondary_gripper][0]["name"]
            # createGraspConstraint → [0]=grasp, [1]=complement, [2]=both
            sec_grasp_list = self.graph.createGraspConstraint(
                f"{secondary_gripper} grasps {secondary_handle}",
                secondary_gripper,
                secondary_handle,
            )
            sec_grasp_obj = sec_grasp_list[0]  # grasp constraint only
            sec_pregrasp_obj = self.graph.createPreGraspConstraint(
                f"{secondary_gripper} pregrasps {secondary_handle}",
                secondary_gripper,
                secondary_handle,
            )
            for node_name, c_obj in [
                (
                    f"{primary_gripper} > {primary_handle} | f_pregrasp",
                    sec_pregrasp_obj,
                ),
                (f"{primary_gripper} > {primary_handle} | f_intersec", sec_grasp_obj),
                (f"{primary_gripper} > {primary_handle} | f_preplace", sec_grasp_obj),
                (f"{primary_gripper} grasps {primary_handle}", sec_grasp_obj),
            ]:
                self.graph.addNumericalConstraintsToState(
                    self.graph.getState(node_name), [c_obj]
                )

        # ── Global constraints ────────────────────────────────────────────────
        self.graph.addNumericalConstraintsToGraph(
            self._locked_grippers + self._locked_head + self._locked_wheels
        )

        # ── States & transitions ──────────────────────────────────────────────
        state_unconstrained = self.graph.createState("unconstrained", False, 0)
        state_free = self.graph.getState("free")
        self.graph.createTransition(
            state_unconstrained, state_free, "project-on-free", 1, state_unconstrained
        )

        node_grasp = f"{primary_gripper} grasps {primary_handle}"
        state_grasp = self.graph.getState(node_grasp)

        t_transit_free = self.graph.createTransition(
            state_free, state_free, "transit_free", 1, state_free
        )
        self.graph.addNumericalConstraintsToTransition(
            t_transit_free,
            self._locked_arms + self._locked_torso + [place_complement_obj],
        )

        t_transit_grasp = self.graph.createTransition(
            state_grasp, state_grasp, "transit_grasp", 1, state_grasp
        )
        self.graph.addNumericalConstraintsToTransition(
            t_transit_grasp, self._locked_arms + self._locked_torso
        )

        for edge_name in [
            f"{primary_gripper} > {primary_handle} | f_01",
            f"{primary_gripper} > {primary_handle} | f_12",
            f"{primary_gripper} > {primary_handle} | f_23",
            "Loop | f",
            "Loop | 0-0",
        ]:
            self.graph.addNumericalConstraintsToTransition(
                self.graph.getTransition(edge_name), self._locked_base_mobility
            )

        self.graph.setWeight(self.graph.getTransition("Loop | f"), 1)
        self.graph.setWeight(self.graph.getTransition("Loop | 0-0"), 1)

        for e_name in self.graph.getTransitionNames():
            self.graph.addNumericalConstraintsToTransition(
                self.graph.getTransition(e_name), self._locked_plate
            )

        self.graph.initialize()

    # ── Public helpers ────────────────────────────────────────────────────────

    def define_gripper(
        self,
        robot,
        frame_name: str,
        gripper_name: str,
        pose: pin.SE3,
        clearance: float = 0.02,
    ):
        robot.addGripper(frame_name, gripper_name, pose, clearance)

    def define_handle(
        self,
        robot,
        link_name: str,
        handle_name: str,
        pose: pin.SE3,
        clearance: float = 0.05,
    ):
        robot.addHandle(link_name, handle_name, pose, clearance, 6 * [True])

    def sample_trajectory(self, path_obj) -> list | None:
        if path_obj is None:
            return None
        tr = path_obj.timeRange()
        t_min, t_max = tr.first, tr.second
        times = np.arange(t_min, t_max, self._ocp_dt)
        return [np.asarray(path_obj.eval(t)[0], dtype=np.float64) for t in times]

    def plan_grasp(self, gripper: str, handle: str, q_init: list):
        q = np.asarray(q_init, dtype=np.float64)
        self.view(q)
        result = self._path_planner.planPathtoBarHandling(
            gripper, handle, q, self._logger
        )
        if result is not None:
            self._logger.info("Path to bar grasping planned successfully.")
            if hasattr(self, "_viewer"):
                self._viewer.loadPath(result)
                self._logger.info("Trajectoire chargée dans le viewer.")
            traj = self.sample_trajectory(result)
            last_q = list(traj[-1]) if traj else None
            return traj, last_q
        else:
            self._logger.error("Path to bar grasping planning failed.")
            return None, None

    def plan_place(
        self, gripper: str, handle: str, q_init: list, target_bar_pose: list
    ):
        q = np.asarray(q_init, dtype=np.float64)
        self.view(q)
        result = self._path_planner.planPathtoBarPlacement(
            gripper, handle, q, target_bar_pose, self._logger
        )
        if result is not None:
            self._logger.info("Path to bar placement planned successfully.")
            if hasattr(self, "_viewer"):
                self._viewer.loadPath(result)
                self._logger.info("Trajectoire chargée dans le viewer.")
            traj = self.sample_trajectory(result)
            last_q = list(traj[-1]) if traj else None
            return traj, last_q
        else:
            self._logger.error("Path to bar placement planning failed.")
            return None, None

    # ── Visualisation ─────────────────────────────────────────────────────────

    def init_viewer(self, open: bool = False):
        from pyhpp_viser import Viewer

        self._viewer = Viewer(self.robot)
        self._viewer.initViewer(open=open, loadModel=True)
        self._viewer.setProblem(self._problem)
        self._viewer.setGraph(self.graph)
        print("Viser viewer ready.  Use o.view(q) or o.play(path).")

    def view(self, q=None):
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        self._viewer(q if q is not None else self.q_init)

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
