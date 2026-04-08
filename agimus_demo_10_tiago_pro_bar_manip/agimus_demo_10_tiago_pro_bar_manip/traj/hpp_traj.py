from math import sqrt
from pathlib import Path

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
from agimus_demo_10_tiago_pro_bar_manip.rostools import process_xacro

from agimus_demo_10_tiago_pro_bar_manip.planner.path_planner import PathPlanner

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()


class BaseObject(object):
    """Wrapper to describe a URDF/SRDF object for HPP."""

    def __init__(
        self,
        root_joint_type: str,
        urdf_path: str,
        srdf_path: str,
        name: str,
    ):
        self.rootJointType = root_joint_type  # HPP expects this attribute name
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


class HPPPathGenerator:
    """
    Wraps HPP manipulation planning for the Tiago Pro bimanual demo.

    Parameters
    ----------
    bar_placement_config_path : Path
        YAML file describing the bar placement (handles + optional q_guesses).
    robot_description : Path
        Path to the robot URDF xacro file.
    robot_description_srdf : Path
        Path to the robot SRDF xacro file.
    environment_description : Path
        (Currently unused – reserved for future environment URDF loading.)
    handle_object : BaseObject
        The object that carries the handles (reinforcement bar).
    plate_object : BaseObject
        The plate object (fixed on the table).
    table_object : BaseObject
        The table object (static environment).
    ocp_dt : float
        Time step used by the MPC / OCP downstream.
    tool_frame_id : str
        Name of the tool frame in the Pinocchio model.
    robot_model : pin.Model
        Pinocchio model of the robot (without objects).
    robot_name : str
        Prefix used in HPP joint names, e.g. ``"tiago_pro"``.
    gripper_names : list[str]
        List of gripper names (without prefix).  The *first* entry is used
        for the TransitionPlanner; the second (if present) is the bimanual
        helper gripper.
    """

    def __init__(
        self,
        handle_config: Path,
        robot_description: Path,
        robot_description_srdf: Path,
        handle_object: BaseObject,
        plate_object: BaseObject,
        table_object: BaseObject,
        ocp_dt: float,
        robot_name: str,
        logger,
    ) -> None:
        # ------------------------------------------------------------------ #
        # Basic attributes
        # ------------------------------------------------------------------ #
        self._ocp_dt = ocp_dt
        self._robot_name = robot_name
        self._logger = logger

        # Objects
        self._handle_object = handle_object
        self._plate_object = plate_object
        self._table_object = table_object

        # Load bar-placement config (handles + optional q_guesses per handle)
        with open(handle_config, "r") as stream:
            self._handle_config_file = yaml.safe_load(stream)

        # Build {handle_name: q_guess_list} from YAML
        # Expected YAML structure:
        #   handles:
        #     - name: left
        #       q_guess: [0.0, ...]   # optional
        #     - name: right
        self._handles_config: dict[str, list] = {}
        for gripper_name, config in self._handle_config_file.get(
            "grippers", {}
        ).items():
            self._handles_config[gripper_name] = config.get("handles", [])
        # ------------------------------------------------------------------ #
        # Build & configure the HPP robot
        # ------------------------------------------------------------------ #
        robot = self._process_Robot(robot_description, robot_description_srdf)

        # ------------------------------------------------------------------ #
        # Problem Solver
        # ------------------------------------------------------------------ #
        self._ps = ProblemSolver(robot)
        self._ps.loadPlugin("manipulation-spline-gradient-based.so")
        self._ps.setErrorThreshold(1e-2)
        self._ps.setMaxIterProjection(40)
        self._ps.setParameter("StatesPathFinder/maxDepth", 2)
        self._ps.addPathOptimizer("RandomShortcut")
        self._ps.addPathOptimizer("SimpleTimeParameterization")
        self._ps.setParameter("SimpleTimeParameterization/order", 2)
        self._ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)

        # ------------------------------------------------------------------ #
        # Viewer (loads object models into HPP)
        # ------------------------------------------------------------------ #
        vf = ViewerFactory(self._ps)
        vf.loadObjectModel(self._handle_object, self._handle_object.name)
        vf.loadObjectModel(self._plate_object, self._plate_object.name)
        vf.loadObjectModel(self._table_object, self._table_object.name)
        self._v = vf.createViewer()

        # ------------------------------------------------------------------ #
        # Joint bounds
        # ------------------------------------------------------------------ #
        robot.setJointBounds(
            f"{self._handle_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        robot.setJointBounds(
            f"{self._plate_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        # robot.setJointBounds(
        #     "table/root_joint",[-5.0, 5.0, -5.0, 5.0, 0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        # )
        robot.setJointBounds(
            f"{robot_name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, -1, 1, -1, 1],
        )
        self.robot = robot

        # ------------------------------------------------------------------ #
        # Locked joints
        # ------------------------------------------------------------------ #
        self._locked_grippers = self._create_locked_grippers(robot_name)
        self._locked_head = self._create_locked_head(robot_name)
        self._locked_wheels = self._create_locked_wheels(robot_name)
        self._locked_plate = self._create_locked_plate()
        # self._locked_table =self._create_locked_table()
        self._locked_base_mobility = self._create_locked_base_mobility(robot_name)
        self._locked_arms, self._locked_torso = self._create_locked_arms_and_torso(
            robot, robot_name
        )

        # ------------------------------------------------------------------ #
        # Grippers
        # ------------------------------------------------------------------ #
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

        # Handles on handle object
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

        # ------------------------------------------------------------------ #
        # Constraint graph
        # ------------------------------------------------------------------ #
        gripper_names = list(self._handles_config.keys())
        self._cg = self._generate_constraint_graph(
            robot, "graph", robot_name, gripper_names
        )
        self._logger.info("Constraint graph built")

        # # ------------------------------------------------------------------ #
        # # PathPlanner (TransitionPlanner wrapper)
        # # ------------------------------------------------------------------ #
        # primary_gripper = gripper_names[0]
        self._path_planner = PathPlanner(self._ps, self._cg)

    # ---------------------------------------------------------------------- #
    # Private helpers
    # ---------------------------------------------------------------------- #

    def _process_Robot(self, urdf_xacro: Path, srdf_xacro: Path) -> Robot:
        """Build and return the HPP Robot from xacro files."""
        Robot.urdfString = process_xacro(
            str(urdf_xacro),
            "end_effector_left:=pal-pro-gripper",
            "end_effector_right:=pal-pro-gripper",
        ).replace("file://", "")
        Robot.srdfString = ""

        srdf_string = process_xacro(
            str(srdf_xacro),
            "end_effector_left:=pal-pro-gripper",
            "end_effector_right:=pal-pro-gripper",
        )

        # Patch SRDF: remove </robot>, add disabled collision pairs, close tag
        i = srdf_string.find("</robot>")
        assert i != -1, "SRDF string does not contain </robot>"
        srdf_string = srdf_string[:i]
        for l1, l2 in [
            ("base_link", "wheel_front_left_link"),
            ("base_link", "wheel_front_right_link"),
            ("base_link", "wheel_rear_left_link"),
            ("base_link", "wheel_rear_right_link"),
            ("gripper_left_screw_left_link", "gripper_left_fingertip_left_link"),
            ("gripper_right_screw_left_link", "gripper_right_fingertip_left_link"),
        ]:
            srdf_string += (
                f'  <disable_collisions link1="{l1}" link2="{l2}" reason="Never"/>\n'
            )
        srdf_string += "</robot>"

        robot = Robot("tiago_pro-manip", "tiago_pro", rootJointType="planar")
        robot.client.manipulation.robot.insertRobotSRDFModelFromString(
            "tiago_pro", srdf_string
        )
        return robot

    def _create_locked_grippers(self, robot_name: str) -> list[str]:
        locked = []
        gripper_values = {
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
        for joint, value in gripper_values.items():
            name = f"locked_{joint}"
            self._ps.createLockedJoint(name, joint, [value])
            locked.append(name)
        return locked

    def _create_locked_head(self, robot_name: str) -> list[str]:
        locked = []
        for joint, value in {
            f"{robot_name}/head_1_joint": 0,
            f"{robot_name}/head_2_joint": 0,
        }.items():
            name = f"locked_{joint}"
            self._ps.createLockedJoint(name, joint, [value])
            locked.append(name)
        return locked

    def _create_locked_wheels(self, robot_name: str) -> list[str]:
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

    def _create_locked_plate(self) -> list[str]:
        joint = f"{self._plate_object.name}/root_joint"
        name = f"locked_{joint}"
        self._ps.createLockedJoint(name, joint, [0.6, 0, 0.66, 0, 0, 0, 1])
        self._ps.setConstantRightHandSide(name, True)
        return [name]

    def _create_locked_table(self) -> list[str]:
        joint = "table/root_joint"
        name = f"locked_{joint}"
        self._ps.createLockedJoint(name, joint, [0, 0, 0, 0, 0, 0, 1])
        self._ps.setConstantRightHandSide(name, True)
        return [name]

    def _create_locked_base_mobility(self, robot_name: str) -> list[str]:
        """Lock base en transit avec barre — suiverait q_init par défaut"""
        name = "locked_base"
        self._ps.createLockedJoint(name, f"{robot_name}/root_joint", [0, 0, 1, 0])
        self._ps.setConstantRightHandSide(name, False)  # Flexible sur orientation
        return [name]

    def _create_locked_arms_and_torso(
        self, robot: Robot, robot_name: str
    ) -> tuple[list[str], list[str]]:
        """Create locked-joint constraints for all arm joints and the torso."""
        locked_arms = []
        seen = set()
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

        # Torso handled separately so it can be unlocked independently
        self._ps.createLockedJoint(
            "locked_torso", f"{robot_name}/torso_lift_joint", [0]
        )
        self._ps.setConstantRightHandSide("locked_torso", False)
        locked_torso = ["locked_torso"]

        return locked_arms, locked_torso

    def _generate_constraint_graph(
        self,
        robot: Robot,
        graph_name: str,
        robot_name: str,
        gripper_names: list[str],
    ) -> ConstraintGraph:
        """Build and initialise the HPP constraint graph."""
        handle_object_name = self._handle_object.name
        primary_gripper = f"{gripper_names[0]}"
        primary_handle = self._handles_config[primary_gripper][0][
            "name"
        ]  # e.g. "left" or "right"

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
            [[f"{primary_handle}"]],
            [[f"{handle_object_name}/bottom"]],
        )
        factory.generate()

        # --- Bimanual constraints (second gripper) -------------------------
        if len(gripper_names) == 2:
            secondary_gripper = f"{gripper_names[1]}"
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
                    node=node,
                    constraints=Constraints(numConstraints=[constraint]),
                )

        # --- Global constraints (applied to the whole graph) ---------------
        cg.addConstraints(
            graph=True,
            constraints=Constraints(
                numConstraints=(
                    self._locked_grippers
                    + self._locked_head
                    + self._locked_wheels
                    + self._locked_plate
                    # + self._locked_table
                )
            ),
        )

        # --- Additional nodes & edges (unconstrained / project-on-free) ----
        cg.createNode("unconstrained")
        cg.createEdge("unconstrained", "free", "project-on-free", 1, "unconstrained")

        # --- Transit edges -------------------------------------------------
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
        cg.addConstraints(
            edge=f"{primary_gripper} > {primary_handle} | f_01",
            constraints=Constraints(numConstraints=self._locked_base_mobility),
        )
        cg.addConstraints(
            edge=f"{primary_gripper} > {primary_handle} | f_12",
            constraints=Constraints(numConstraints=self._locked_base_mobility),
        )
        cg.addConstraints(
            edge=f"{primary_gripper} > {primary_handle} | f_23",
            constraints=Constraints(numConstraints=self._locked_base_mobility),
        )

        # --- Lock base mobility on loop edges ------------------------------
        cg.addConstraints(
            edge="Loop | f",
            constraints=Constraints(numConstraints=self._locked_base_mobility),
        )
        cg.addConstraints(
            edge="Loop | 0-0",
            constraints=Constraints(numConstraints=self._locked_base_mobility),
        )

        cg.setWeight("Loop | f", 1)
        cg.setWeight("Loop | 0-0", 1)

        # -- Base placement constraints on graspng and releasing the bar ----------------

        print(f"[HPPPathGenerator] constraint graph edge count: {len(cg.edges)}")
        cg.initialize()
        return cg

    # ---------------------------------------------------------------------- #
    # Public helpers (previously standalone methods)
    # ---------------------------------------------------------------------- #

    def define_gripper(
        self,
        robot: Robot,
        frame_name: str,
        gripper_name: str,
        gripper_frame_pose: list[float],
        clearance: float = 0.02,
    ) -> None:
        """Register a gripper on the robot model."""
        robot.client.manipulation.robot.addGripper(
            frame_name, gripper_name, gripper_frame_pose, clearance
        )

    def define_handle(
        self,
        robot: Robot,
        link_name: str,
        handle_name: str,
        handle_pose: list[float],
        clearance: float = 0.05,
    ) -> None:
        """Register a handle on an object link."""
        robot.client.manipulation.robot.addHandle(
            link_name,
            handle_name,
            handle_pose,
            clearance,
            6 * [True],
        )

    def update_joint_values(self, q_now: list[float]) -> None:
        """Update the robot's current configuration in HPP."""
        self.robot.setCurrentConfig(q_now)

    def update_arm_joint_values(self, q_now: list[float]) -> None:
        """Update only the arm joints' values in HPP, keeping the rest unchanged."""
        q_current = self.robot.getCurrentConfig()
        r = self.robot.rankInConfiguration[self._robot_name + "/root_joint"]
        q_current[r : r + 4] = q_now[r : r + 4]  # Update base (planar) joints
        # Update arm joints (assuming they are contiguous and start after the base joints)
        arm_joint_names = [
            j for j in self.robot.jointNames if j.startswith(f"{self._robot_name}/arm")
        ]
        for joint_name in arm_joint_names:
            idx = self.robot.rankInConfiguration[joint_name]
            q_current[idx] = q_now[idx]
        self.robot.setCurrentConfig(q_current)

    # ---------------------------------------------------------------------- #
    # Planning API
    # ---------------------------------------------------------------------- #

    def plan_grasp(
        self,
        gripper: str,
        handle: str,
        q_init: list,
    ):
        """Plan path from q_init to pregrasp configuration for *handle*."""
        return self._path_planner.planPathtoBarHandling(
            gripper, handle, q_init, self._v, self._logger
        )

    def plan_path_to_place(
        self,
        gripper: str,
        handle: str,
        q_init: list,
        q_goal: list,
        grasp_constraints: list[str],
        q_guesses: list = [],
    ):
        """Plan full pregrasp -> grasp -> preplace -> place path for *handle*."""
        return self._path_planner.planPathToPlace(
            gripper, handle, q_init, q_goal, grasp_constraints, q_guesses
        )
