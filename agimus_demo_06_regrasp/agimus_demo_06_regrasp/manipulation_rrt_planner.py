from hpp.corbaserver import shrinkJointRange, wrap_delete
from hpp.corbaserver.manipulation import (
    Robot,
    newProblem,
    ProblemSolver,
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Rule,
)

from hpp.gepetto.manipulation import ViewerFactory
from hpp.rostools import process_xacro, retrieve_resource

from agimus_demo_06_regrasp.utils import (
    BaseObject,
    get_obj_goal_handles,
    hack_for_ros2_support_in_hpp,
    path_move_object,
    get_path_grasp_sequences,
)


class ManipulationPlanner:
    """
    Manipulation RRT planner inspired from
    https://github.com/humanoid-path-planner/test-hpp/blob/3f38d7b3a6ef0ea351f6d9ef370aeb04e70228c6/script/test_ur5.py
    and
    https://github.com/agimus-project/guided_tamp_benchmark

    """

    def __init__(
        self,
        object_name: str,
        robot_grippers: list[str] = ["panda/panda_gripper"],
        default_object_bounds: list[float] = [-1.0, 1.5, -1.0, 1.0, 0.0, 2.2],
        gripper_open_value: float = 0.04,
        package_location: str = "package://agimus_demo_06_regrasp",
        verbose: bool = True,
    ):
        """
        :param object_name: Name of the object to manipulate (e.g., "tless_20")
        :param robot_grippers: List of gripper names to use for manipulation
        :param default_object_bounds: Default bounds for the manipulated object
        :param gripper_open_value: Value to set the gripper in open position
        :param verbose: If True, print additional information during execution
        """
        # Ensure compatibility with ROS2
        hack_for_ros2_support_in_hpp()
        self.object_name = object_name
        self.robot_grippers = robot_grippers
        self.default_object_bounds = default_object_bounds
        self.gripper_open_value = gripper_open_value
        self.package_location = package_location
        self.verbose = verbose

        self.create_problem()
        self.create_graph()

    def wd(self, o):
        return wrap_delete(o, self.ps.client.basic._tools)

    def create_problem(self):
        """Create a new problem in HPP"""
        Client().problem.resetProblem()
        newProblem()
        # Load robot
        Robot.urdfString = process_xacro(
            self.package_location + "/urdf/demo.urdf.xacro"
        )
        Robot.srdfString = ""
        self.robot = Robot("robot", "panda", rootJointType="anchor")
        shrinkJointRange(self.robot, [f"panda/fer_joint{i}" for i in range(1, 8)], 0.95)
        # set problem
        self.ps = ProblemSolver(self.robot)
        self.ps.selectPathPlanner("M-RRT")
        # self.ps.addPathOptimizer("RandomShortcut")
        self.ps.addPathOptimizer("Graph-RandomShortcut")
        self.ps.addPathOptimizer("EnforceTransitionSemantic")
        # add time parametrization for smooth velocities
        self.ps.addPathOptimizer("SimpleTimeParameterization")
        self.ps.setParameter("SimpleTimeParameterization/order", 2)
        self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.7)
        self.ps.setParameter("SimpleTimeParameterization/safety", 0.95)

        # Add path projector to avoid discontinuities
        self.ps.selectPathProjector("Progressive", 0.05)
        self.ps.selectPathValidation("Graph-Progressive", 0.01)

        # create viewer factory to debug with gepetto-gui
        self.vf = ViewerFactory(self.ps)

        # Load manipulated object
        self.manip_object = BaseObject(
            urdf_path=retrieve_resource(
                f"{self.package_location}/urdf/tless/{self.object_name}.urdf"
            ),
            srdf_path=retrieve_resource(
                f"{self.package_location}/srdf/tless/{self.object_name}.srdf"
            ),
            name="tless_20",
        )
        self.vf.loadObjectModel(self.manip_object, self.manip_object.name)

        self.robot.setJointBounds(
            f"{self.manip_object.name}/root_joint", self.default_object_bounds
        )
        print("Part loaded")
        self.robot.client.manipulation.robot.insertRobotSRDFModel(
            "panda",
            retrieve_resource("package://agimus_demo_06_regrasp/srdf/demo.srdf"),
        )

        # Lock gripper in open position.
        self.ps.createLockedJoint(
            "locked_finger_1", "panda/fer_finger_joint1", [self.gripper_open_value]
        )
        self.ps.createLockedJoint(
            "locked_finger_2", "panda/fer_finger_joint2", [self.gripper_open_value]
        )
        self.ps.setConstantRightHandSide("locked_finger_1", True)
        self.ps.setConstantRightHandSide("locked_finger_2", True)

    def create_graph(self):
        """Create the constraint graph for manipulation"""
        # Get handle of the objects
        self.object_handles, self.goal_handles = get_obj_goal_handles(
            prefix=self.manip_object.name + "/",
            srdf_path=self.manip_object.srdfFilename,
        )

        # create constraint graph
        rules = [
            Rule([".*"], [".*"], True),
        ]
        self.cg = ConstraintGraph(self.robot, graphName="constraint_graph")
        self.factory = ConstraintGraphFactory(self.cg)
        self.factory.setGrippers(self.robot_grippers)
        self.factory.environmentContacts(
            [
                "panda/foam_block_contact",
            ]
        )
        self.factory.setObjects(
            [self.manip_object.name],
            [self.object_handles],
            [["tless_20/lower_upper_z", "tless_20/lower_y"]],
        )
        self.factory.setRules(rules)
        self.factory.generate()
        self.cg.initialize()

    def clear_roadmap(self):
        """Clear problem solver roadmap and erase all paths"""
        self.ps.clearRoadmap()
        for i in range(self.ps.numberPaths() - 1, -1, -1):
            self.ps.erasePath(i)
        self.ps.resetGoalConfigs()

    def solve(self, q_start, q_goal) -> bool:
        """Solve planning problem with classical hpp"""
        self.vf.createViewer()  # Create gepetto-gui viewer
        self.robot.setCurrentConfig(q_start)
        input("Press Enter to continue (q_init before projection) ...")
        res, q_start, err = self.cg.applyNodeConstraints("free", q_start)
        assert res, f"Failed to apply node constraints on start configuration: {err}"
        self.robot.setCurrentConfig(q_start)
        input("Press Enter to continue (q_init after projection) ...")
        self.robot.setCurrentConfig(q_goal)
        input("Press Enter to continue (q_goal before projection) ...")
        res, q_goal, err = self.cg.applyNodeConstraints("free", q_goal)
        assert res, f"Failed to apply node constraints on goal configuration: {err}"
        self.robot.setCurrentConfig(q_goal)
        input("Press Enter to continue (q_goal after projection) ...")

        self.robot.setCurrentConfig(q_start)
        self.clear_roadmap()
        self.ps.setInitialConfig(q_start)
        self.ps.addGoalConfig(q_goal)
        try:
            self.ps.solve()
            path = self.ps.client.basic.problem.getPath(self.ps.numberPaths() - 1)

            flat_path = path.flatten()
            # grasp_path_idxs = [0]
            # placing_path_idxs = []
            # freefly_path_idxs = []

            for idx in range(1, flat_path.numberPaths()):
                if path_move_object(path.pathAtRank(idx)):
                    print(f"moving object at {idx}")
                else:
                    print(f"static object at {idx}")
            path_seq = get_path_grasp_sequences(
                path,
                self.wd(self.wd(self.ps.client.basic.problem.getProblem()).robot()),
            )
            for path, _, _ in path_seq:
                self.ps.client.basic.problem.addPath(path)

            return True, path_seq
        except BaseException as e:
            if self.verbose:
                print(e)
            return False, None
