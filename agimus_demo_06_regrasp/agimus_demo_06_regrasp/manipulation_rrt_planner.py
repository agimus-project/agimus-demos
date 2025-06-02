from hpp.corbaserver import shrinkJointRange
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
)


class ManipulationPlanner:
    """
    Manipulation RRT planner inspired from
    https://github.com/humanoid-path-planner/test-hpp/blob/3f38d7b3a6ef0ea351f6d9ef370aeb04e70228c6/script/test_ur5.py
    and
    https://github.com/agimus-project/guided_tamp_benchmark

    """

    def __init__(self, object_name: str, object_handles: list[str]):
        self.object_handles = object_handles
        self.robot_grippers = ["panda/panda_gripper"]
        Client().problem.resetProblem()
        # create problem
        newProblem()

        # Load robot
        package_location = "package://agimus_demo_06_regrasp"
        Robot.urdfString = process_xacro(package_location + "/urdf/demo.urdf.xacro")
        Robot.srdfString = ""
        self.robot = Robot("robot", "panda", rootJointType="anchor")
        shrinkJointRange(self.robot, [f"panda/fer_joint{i}" for i in range(1, 8)], 0.95)
        # Load manipulated object
        self.manip_object = BaseObject(
            urdf_path=retrieve_resource(
                f"{package_location}/urdf/tless/{object_name}.urdf"
            ),
            srdf_path=retrieve_resource(
                f"{package_location}/srdf/tless/{object_name}.srdf"
            ),
            name="part",
        )

        # set problem
        self.ps = ProblemSolver(self.robot)
        self.ps.addPathOptimizer("EnforceTransitionSemantic")
        # add time parametrization for smooth velocities
        self.ps.addPathOptimizer("SimpleTimeParameterization")
        self.ps.setParameter("SimpleTimeParameterization/order", 2)
        self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)
        self.ps.setParameter("SimpleTimeParameterization/safety", 0.95)

        # Add path projector to avoid discontinuities
        self.ps.selectPathProjector("Progressive", 0.05)
        self.ps.selectPathValidation("Graph-Progressive", 0.01)

        # create viewer factory to debug with gepetto-gui
        self.vf = ViewerFactory(self.ps)

        # load the object
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

        # Add handle of the objects
        self.handles, self.goal_handles = get_obj_goal_handles(
            prefix=self.manip_object.name + "/",
            srdf_path=self.manip_object.srdfFilename,
        )

        # create constraint graph

        rules = [
            Rule([".*"], [".*"], True),
        ]
        self.cg = ConstraintGraph(self.robot, graphName=self.graph_name)
        self.factory = ConstraintGraphFactory(self.cg)
        self.factory.setGrippers(self.robot_grippers)
        self.factory.environmentContacts(["panda/foam_block_contact"])
        self.factory.setObjects(
            [self.manip_object.name],
            self.object_handles,
            [f"{self.manip_object.name}/{self.manip_object.name}_surface"],
        )
        self.factory.setRules(rules)
        self.factory.generate()
        self.cg.initialize()

        # validate graph
        cproblem = self.ps.hppcorba.problem.getProblem()
        cgraph = cproblem.getConstraintGraph()
        cgraph.initialize()

    def clear_roadmap(self):
        """Clear problem solver roadmap and erase all paths"""
        self.ps.clearRoadmap()
        for i in range(self.ps.numberPaths() - 1, -1, -1):
            self.ps.erasePath(i)
        self.ps.resetGoalConfigs()

    def solve(self, q_start, q_goal) -> bool:
        """Solve planning problem with classical hpp"""
        q_start, q_goal = self.get_start_goal()
        self.clear_roadmap()
        self.ps.setInitialConfig(q_start)
        self.ps.addGoalConfig(q_goal)
        try:
            self.ps.addPathOptimizer("RandomShortcut")
            self.ps.solve()
            return True
        except BaseException as e:
            if self.verbose:
                print(e)
            return False
