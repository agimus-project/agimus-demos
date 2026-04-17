from math import sqrt
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Constraints,
    ProblemSolver,
    Robot,
)
from hpp.gepetto.manipulation import ViewerFactory

from agimus_demo_10_tiago_pro_bar_manip.planner.path_planner import PathPlanner

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()


class BaseObject:
    """Wrapper to describe a URDF/SRDF object for HPP."""

    def __init__(self, root_joint_type: str, urdf_path: str, srdf_path: str, name: str):
        self.rootJointType = root_joint_type
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


class HPPPathGenerator:
    """
    Wraps HPP manipulation planning for the Tiago Pro bimanual demo.

    Parameters
    ----------
    urdf_str : str
        Robot URDF as a plain string (already processed, no xacro).
    srdf_str : str
        Robot SRDF as a plain string (already processed, no xacro).
    robot_model : pin.Model
        Pinocchio model built from urdf_str by the caller.
    handle_config : Path
        YAML file with handle/gripper configuration.
    handle_object, plate_object, table_object : BaseObject
        HPP object descriptors.
    ocp_dt : float
        Sampling time step for trajectory extraction.
    robot_name : str
        HPP joint name prefix, e.g. "tiago_pro".
    logger :
        ROS2 logger or None.
    """

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

        # Handle/gripper config
        with open(handle_config, "r") as f:
            self._handle_config_file = yaml.safe_load(f)
        self._handles_config: dict[str, list] = {}
        for gname, config in self._handle_config_file.get("grippers", {}).items():
            self._handles_config[gname] = config.get("handles", [])

        # ── HPP Robot ───────────────────────────────────────────────────────
        # urdf_str must NOT contain "file://" prefixes
        Robot.urdfString = urdf_str.replace("file://", "")
        Robot.srdfString = srdf_str

        robot = Robot("tiago_pro-manip", robot_name, rootJointType="planar")
        # SRDF already injected via Robot.srdfString above;
        # if you need to patch collision pairs do it in the caller before passing srdf_str.

        # ── Problem Solver ─────────────────────────────────────────────────
        self._ps = ProblemSolver(robot)
        self._ps.loadPlugin("manipulation-spline-gradient-based.so")
        self._ps.setErrorThreshold(1e-2)
        self._ps.setMaxIterProjection(40)
        self._ps.setParameter("StatesPathFinder/maxDepth", 2)
        self._ps.addPathOptimizer("RandomShortcut")
        self._ps.addPathOptimizer("SimpleTimeParameterization")
        self._ps.setParameter("SimpleTimeParameterization/order", 2)
        self._ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)

        # ── Viewer / object loading ────────────────────────────────────────
        vf = ViewerFactory(self._ps)
        vf.loadObjectModel(handle_object, handle_object.name)
        vf.loadObjectModel(plate_object, plate_object.name)
        vf.loadObjectModel(table_object, table_object.name)
        self._v = vf.createViewer()

        # ── Joint bounds ───────────────────────────────────────────────────
        robot.setJointBounds(
            f"{handle_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        robot.setJointBounds(
            f"{plate_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        robot.setJointBounds(
            f"{robot_name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, -1, 1, -1, 1],
        )
        self.robot = robot

        # ── Locked joints ──────────────────────────────────────────────────
        self._locked_grippers = self._create_locked_grippers(robot_name)
        self._locked_head = self._create_locked_head(robot_name)
        self._locked_wheels = self._create_locked_wheels(robot_name)
        self._locked_plate = self._create_locked_plate()
        self._locked_base_mobility = self._create_locked_base_mobility(robot_name)
        self._locked_arms, self._locked_torso = self._create_locked_arms_and_torso(
            robot, robot_name
        )

        # ── Grippers & handles ─────────────────────────────────────────────
        c = sqrt(2) / 2
        self.define_gripper(
            robot,
            f"{robot_name}/arm_left_7_link",
            f"{robot_name}/left",
            [0, 0, 0.19, 0, -c, 0, c],
        )
        self.define_gripper(
            robot,
            f"{robot_name}/arm_right_7_link",
            f"{robot_name}/right",
            [0, 0, 0.19, 0, -c, 0, c],
        )
        self.define_handle(
            robot,
            f"{handle_object.name}/bar_base_link",
            f"{handle_object.name}/left",
            [0, 0.01, -0.25, 0, 0, -c, c],
        )
        self.define_handle(
            robot,
            f"{handle_object.name}/bar_base_link",
            f"{handle_object.name}/right",
            [0, 0.01, 0.25, 0, 0, -c, c],
        )

        # ── Constraint graph ───────────────────────────────────────────────
        gripper_names = list(self._handles_config.keys())
        self._cg = self._generate_constraint_graph(
            robot, "graph", robot_name, gripper_names
        )
        self._logger.info("Constraint graph built")

        self._path_planner = PathPlanner(self._ps, self._cg)

    # ── Private helpers (unchanged from previous version) ──────────────────

    def _create_locked_grippers(self, robot_name):
        locked = []
        values = {
            f"{robot_name}/gripper_left_finger_joint": 0.05,
            f"{robot_name}/gripper_left_inner_finger_left_joint": -0.05,
            f"{robot_name}/gripper_left_fingertip_left_joint": 0.05,
            f"{robot_name}/gripper_left_finger_right_joint": 0,
            f"{robot_name}/gripper_left_inner_finger_right_joint": -0.05,
            f"{robot_name}/gripper_left_fingertip_right_joint": 0.05,
            f"{robot_name}/gripper_left_outer_finger_left_joint": -0.05,
            f"{robot_name}/gripper_left_outer_finger_right_joint": -0.05,
            f"{robot_name}/gripper_right_finger_joint": 0.05,
            f"{robot_name}/gripper_right_inner_finger_left_joint": -0.05,
            f"{robot_name}/gripper_right_fingertip_left_joint": 0.05,
            f"{robot_name}/gripper_right_finger_right_joint": 0,
            f"{robot_name}/gripper_right_inner_finger_right_joint": -0.05,
            f"{robot_name}/gripper_right_fingertip_right_joint": 0.05,
            f"{robot_name}/gripper_right_outer_finger_left_joint": -0.05,
            f"{robot_name}/gripper_right_outer_finger_right_joint": -0.05,
        }
        for joint, value in values.items():
            name = f"locked_{joint}"
            self._ps.createLockedJoint(name, joint, [value])
            locked.append(name)
        return locked

    def _create_locked_head(self, robot_name):
        locked = []
        for joint, value in {
            f"{robot_name}/head_1_joint": 0,
            f"{robot_name}/head_2_joint": 0,
        }.items():
            name = f"locked_{joint}"
            self._ps.createLockedJoint(name, joint, [value])
            locked.append(name)
        return locked

    def _create_locked_wheels(self, robot_name):
        locked = []
        for joint in [
            f"{robot_name}/wheel_front_left_joint",
            f"{robot_name}/wheel_front_right_joint",
            f"{robot_name}/wheel_rear_left_joint",
            f"{robot_name}/wheel_rear_right_joint",
        ]:
            name = f"locked_{joint}"
            self._ps.createLockedJoint(name, joint, [1, 0])
            self._ps.setConstantRightHandSide(name, True)
            locked.append(name)
        return locked

    def _create_locked_plate(self):
        joint = f"{self._plate_object.name}/root_joint"
        name = f"locked_{joint}"
        self._ps.createLockedJoint(name, joint, [0, 0, 0, 0, 0, 0, 1])
        self._ps.setConstantRightHandSide(name, False)
        return [name]

    def _create_locked_base_mobility(self, robot_name):
        name = "locked_base"
        self._ps.createLockedJoint(name, f"{robot_name}/root_joint", [0, 0, 1, 0])
        self._ps.setConstantRightHandSide(name, False)
        return [name]

    def _create_locked_arms_and_torso(self, robot, robot_name):
        locked_arms, seen = [], set()
        for j in robot.jointNames:
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
            name = f"locked_{j}"
            self._ps.createLockedJoint(name, j, [0])
            self._ps.setConstantRightHandSide(name, False)
            locked_arms.append(name)
        self._ps.createLockedJoint(
            "locked_torso", f"{robot_name}/torso_lift_joint", [0]
        )
        self._ps.setConstantRightHandSide("locked_torso", False)
        return locked_arms, ["locked_torso"]

    def _generate_constraint_graph(self, robot, graph_name, robot_name, gripper_names):
        handle_object_name = self._handle_object.name
        primary_gripper = gripper_names[0]
        primary_handle = self._handles_config[primary_gripper][0]["name"]

        cg = ConstraintGraph(robot, graph_name)
        factory = ConstraintGraphFactory(cg)
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

        if len(gripper_names) == 2:
            secondary_gripper = gripper_names[1]
            secondary_handle = self._handles_config[secondary_gripper][0]["name"]
            cg.createGrasp(
                f"{secondary_gripper} grasps {secondary_handle}",
                secondary_gripper,
                secondary_handle,
            )
            cg.createPreGrasp(
                f"{secondary_gripper} pregrasps {secondary_handle}",
                secondary_gripper,
                secondary_handle,
            )
            for node, constraint in [
                (
                    f"{primary_gripper} > {primary_handle} | f_pregrasp",
                    f"{secondary_gripper} pregrasps {secondary_handle}",
                ),
                (
                    f"{primary_gripper} > {primary_handle} | f_intersec",
                    f"{secondary_gripper} grasps {secondary_handle}",
                ),
                (
                    f"{primary_gripper} > {primary_handle} | f_preplace",
                    f"{secondary_gripper} grasps {secondary_handle}",
                ),
                (
                    f"{primary_gripper} grasps {primary_handle}",
                    f"{secondary_gripper} grasps {secondary_handle}",
                ),
            ]:
                cg.addConstraints(
                    node=node, constraints=Constraints(numConstraints=[constraint])
                )

        cg.addConstraints(
            graph=True,
            constraints=Constraints(
                numConstraints=self._locked_grippers
                + self._locked_head
                + self._locked_wheels
            ),
        )

        cg.createNode("unconstrained")
        cg.createEdge("unconstrained", "free", "project-on-free", 1, "unconstrained")

        node_grasp = f"{primary_gripper} grasps {primary_handle}"
        cg.createEdge("free", "free", "transit_free", 1, "free")
        cg.addConstraints(
            edge="transit_free",
            constraints=Constraints(
                numConstraints=self._locked_arms
                + self._locked_torso
                + [f"place_{handle_object_name}/complement"]
            ),
        )
        cg.createEdge(node_grasp, node_grasp, "transit_grasp", 1, node_grasp)
        cg.addConstraints(
            edge="transit_grasp",
            constraints=Constraints(
                numConstraints=self._locked_arms + self._locked_torso
            ),
        )
        for edge in [
            f"{primary_gripper} > {primary_handle} | f_01",
            f"{primary_gripper} > {primary_handle} | f_12",
            f"{primary_gripper} > {primary_handle} | f_23",
            "Loop | f",
            "Loop | 0-0",
        ]:
            cg.addConstraints(
                edge=edge,
                constraints=Constraints(numConstraints=self._locked_base_mobility),
            )

        cg.setWeight("Loop | f", 1)
        cg.setWeight("Loop | 0-0", 1)

        for e in cg.edges.keys():
            cg.addConstraints(
                edge=e, constraints=Constraints(numConstraints=self._locked_plate)
            )

        cg.initialize()
        return cg

    # ── Public helpers ─────────────────────────────────────────────────────

    def define_gripper(self, robot, frame_name, gripper_name, pose, clearance=0.02):
        robot.client.manipulation.robot.addGripper(
            frame_name, gripper_name, pose, clearance
        )

    def define_handle(self, robot, link_name, handle_name, pose, clearance=0.05):
        robot.client.manipulation.robot.addHandle(
            link_name, handle_name, pose, clearance, 6 * [True]
        )

    def sample_trajectory(self, path_obj) -> list | None:
        if path_obj is None:
            return None
        length = path_obj.length()
        dt = self._ocp_dt
        n = max(2, int(np.ceil(length / dt)))
        return [list(path_obj.call(min(i * dt, length))[0]) for i in range(n)]

    # ── Planning API ───────────────────────────────────────────────────────

    def plan_grasp(self, gripper: str, handle: str, q_init: list):
        result = self._path_planner.planPathtoBarHandling(
            gripper, handle, q_init, self._v, self._logger
        )
        traj = self.sample_trajectory(result)
        if result is not None:
            result.deleteThis()
        last_q = list(traj[-1]) if traj else None
        return traj, last_q

    def plan_place(
        self, gripper: str, handle: str, q_init: list, target_bar_pose: list
    ):
        result = self._path_planner.planPathtoBarPlacement(
            gripper, handle, q_init, target_bar_pose, self._v, self._logger
        )
        traj = self.sample_trajectory(result)
        if result is not None:
            result.deleteThis()
        last_q = list(traj[-1]) if traj else None
        return traj, last_q
