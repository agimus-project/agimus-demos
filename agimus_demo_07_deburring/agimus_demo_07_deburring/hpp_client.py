import pathlib
import typing as T
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import (
    Robot,
    ConstraintGraph,
    ConstraintGraphFactory,
    ProblemSolver,
    Rule,
)
from hpp.gepetto.manipulation import ViewerFactory

from math import ceil

from agimus_demos_common.orchestrator.utils import (
    BaseObject,
)
from agimus_demos_common.orchestrator.corba import CorbaServer
from hpp.rostools import retrieve_resource
from hpp.corbaserver import wrap_delete as wd


XYZQuatType: T.TypeAlias = T.Tuple[float, float, float, float, float, float, float]


corba = None


class HPPInterface:
    """This interface assumes that there is one object to manipulate and one moving obstacle.
    Both the object and the obstacle are encoded in the space
    Graph creation is specific to this setup
    """

    def __init__(
        self,
        robot_urdf_string: str = "",
        robot_srdf_string: str = "",
        used_handles: list[str] = [],
        gripper_name: str = "",
    ):
        self._used_handles = used_handles

        package_location = "package://agimus_demo_07_deburring"
        Robot.urdfString = pathlib.Path(robot_urdf_string).read_text()
        Robot.srdfString = pathlib.Path(robot_srdf_string).read_text()

        # TODO move urdf and srdf to params
        self._pylone_object = BaseObject(
            urdf_path=retrieve_resource(f"{package_location}/urdf/pylone.urdf"),
            srdf_path=retrieve_resource(f"{package_location}/srdf/pylone.srdf"),
            name="pylone",
        )

        # Init corbaserver
        global corba
        corba = CorbaServer()

        self._robot_name = "panda"
        self._gripper_name = f"{self._robot_name}/{gripper_name}"
        self._robot = Robot("robot", self._robot_name, rootJointType="anchor")
        self._q_init = self._robot.getCurrentConfig()

        self._robot.client.manipulation.robot.insertRobotSRDFModel(
            self._robot_name,
            retrieve_resource("package://agimus_demo_07_deburring/srdf/demo.srdf"),
        )

        self._ps = ProblemSolver(self._robot)

        self.vf = ViewerFactory(self._ps)
        self.vf.loadObjectModel(self._pylone_object, self._pylone_object.name)
        self.v = None

        # TODO move bounds to the params
        self._robot.setJointBounds(
            f"{self._pylone_object.name}/root_joint", [-1.0, 1.0, -1.0, 1.0, 0.0, 2.2]
        )
        shrinkJointRange(
            self._robot, [f"{self._robot_name}/fer_joint{i}" for i in range(1, 8)], 0.95
        )

        self._ps.setErrorThreshold(1e-3)
        self._ps.setMaxIterProjection(40)

        for i in range(1, 3):
            self._ps.createLockedJoint(
                f"locked_finger_{i}", f"{self._robot_name}/fer_finger_joint{i}", [0.0]
            )
            self._ps.setConstantRightHandSide(f"locked_finger_{i}", True)

        self._ps.addPathOptimizer("RandomShortcut")

        self._ps.addPathOptimizer("SimpleTimeParameterization")
        self._ps.setParameter("SimpleTimeParameterization/order", 2)
        self._ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.5)
        self._ps.setParameter("SimpleTimeParameterization/safety", 0.95)

        self.cg = ConstraintGraph(self._robot, "graph")
        self.factory = ConstraintGraphFactory(self.cg)
        self.factory.setGrippers([self._gripper_name])
        self.factory.setObjects([self._pylone_object.name], [self._used_handles], [[]])
        self.factory.setRules([Rule([".*"], [".*"], True)])
        self.factory.generate()
        self.cg.initialize()

        cproblem = wd(self._ps.hppcorba.problem.getProblem())
        self._cgraph = wd(cproblem.getConstraintGraph())
        crobot = wd(cproblem.robot())
        cdistance = wd(self._ps.hppcorba.problem.createDistance("Weighed", cproblem))
        croadmap = wd(
            self._ps.client.manipulation.problem.createRoadmap(cdistance, crobot)
        )
        croadmap.constraintGraph(self._cgraph)
        self._transition_planner = wd(
            self._ps.client.basic.problem.createPathPlanner(
                "TransitionPlanner", cproblem, croadmap
            )
        )
        self._transition_planner.setPathProjector("Progressive", 0.02)
        self._transition_planner.maxIterations(100)

    # def get_path(self):
    #     self._robot.client.basic.robot.getLinksPosition(q, [frame_name])

    def _sample_state_on_transition_target(
        self,
        q_from: list[float],
        transition: str,
        max_iter: int = 1,
    ) -> list[float]:
        for _ in range(max_iter):
            succ, q1, _ = self.cg.generateTargetConfig(
                transition, q_from, self._robot.shootRandomConfig()
            )
            if succ:
                res, _ = self._robot.isConfigValid(q1)  # Collision checking
                if res:
                    return q1
        raise RuntimeError(
            "HPPInterface: Failed to sample collision free final configuration "
            f"for transition '{transition}' after {max_iter} iterations!"
        )

    def _generate_transition(self, transition: str):
        q_final = self._sample_state_on_transition_target(
            q_from=self._q_init,
            transition=transition,
            max_iter=100,
        )
        self._ps.setInitialConfig(self._q_init)
        self._ps.resetGoalConfigs()
        self._ps.addGoalConfig(q_final.copy())
        self._ps.solve()

        self._q_init = q_final.copy()

        # paths = self._ps.numberPaths()
        # # print(self._ps.numberPaths())
        # p_len = self._ps.pathLength(paths-1)
        # self._ps.getTimeOutPathPlanning()
        # print(self._ps.configAtParam(paths-1, 0.0))
        # print(self._ps.configAtParam(paths-1, 0.5 * p_len))
        # print(self._ps.configAtParam(paths-1, p_len))

    def _generate_valid_config_for_handle(
        self, edge, q_init, q_guesses=[], max_iter=100
    ):
        configurations = q_guesses + [
            self.robot.shootRandomConfig() for _ in range(max_iter)
        ]
        for qrand in configurations:
            res, qpg, _ = self.graph.generateTargetConfig(
                edge + " | f_01", q_init, qrand
            )
            if not res or not self.robot.configIsValid(qpg):
                continue
            res, qg, _ = self.graph.generateTargetConfig(edge + " | f_12", qpg, qpg)
            if not res or not self.robot.configIsValid(qg):
                continue
            return qpg, qg
        raise RuntimeError(
            f"Filed to compute a valid configuration for edge: '{edge}'."
        )

    def _generate_valid_config_for_handle(
        self, edge, q_init=None, q_guesses=[], max_iter=100
    ):
        def projectPreGraspConfig(edge, q_init, qInput):
            # Perform projection
            res, projConfig, err = self.graph.generateTargetConfig(edge, q_init, qInput)
            # self.graph.getConfigErrorForEdgeTarget(e, q0, qres)

            # Log the results
            return res, projConfig

        # Loop to generate pre-grasp configurations
        configurations = q_guesses + [
            self.robot.shootRandomConfig() for _ in range(max_iter)
        ]
        for q_rand in configurations:
            res, projConfig = projectPreGraspConfig(edge + " | f_01", q_init, q_rand)
            if res:
                valid_res, _ = self.robot.isConfigValid(projConfig)
                if valid_res:
                    return True, projConfig  # Both projection and validation succeeded
                else:
                    continue  # Skip to the next iteration if validation fails
            else:
                continue  # Skip to the next iteration if projection fails
        raise RuntimeError(
            f"Filed to compute a valid configuration for edge: '{edge}'."
        )

    # def generate_path_for_handle(self, handle: str, max_iter: int = 100):
    #     # generate configurations
    #     edge = tool_gripper + " > " + handle
    #     for _ in range(max_iter):
    #         res, qpg, qg = self._generate_valid_config_for_handle(
    #             edge, self._q_init, [self._q_init], max_iter
    #         )
    #         if not res:
    #             continue
    #         # from qinit to pregrasp
    #         self._transition_planner.setEdge(self._cgraph.edges[edge + " | f_01"])
    #         try:
    #             p1 = self._transition_planner.planPath(
    #                 self._q_init, [qpg], resetRoadmap=True
    #             )
    #         except:
    #             p1 = None
    #         if not p1:
    #             continue
    #         # from pregrasp to grasp
    #         self._transition_planner.setEdge(self._cgraph.edges[edge + " | f_12"])
    #         res, p2, _ = self._transition_planner.directPath(qpg, qg, True)
    #         if not res or not p2:
    #             p2 = None
    #             continue
    #         # back to pregrasp
    #         self._transition_planner.setEdge(self._cgraph.edges[edge + " | f_12"])
    #         res, p3, _ = self._transition_planner.directPath(qg, qpg, True)
    #         if not res or not p3:
    #             p3 = None
    #             continue
    #         return [p1, p2, p3]
    #     raise RuntimeError(f"Filed to compute a path for handle: '{handle}'.")

    def discretize_path(self, path, dt: float) -> list:
        n_steps = ceil(path.length() / dt)
        return [path.call(dt * t) for t in n_steps]

    def restart(self):
        """This needs to be improved"""
        corba.restart()
        self.setup_problem()

    def update_joint_state(self, joint_name: str, position: float):
        rank = self._robot.rankInConfiguration[f"{self._robot_name}/{joint_name}"]
        self._q_init[rank] = position

    def update_pylone_pose(self, pose: list[float], quat: list[float]):
        rank = self._robot.rankInConfiguration[f"{self._pylone_object.name}/root_joint"]
        self._q_init[rank : rank + 3] = pose
        self._q_init[rank + 3 : rank + 7] = quat

    def create_viewer(self):
        self.v = self.vf.createViewer()

    def update_initial_configuration(self):
        self._ps.setInitialConfig(self._q_init)
        self.v(self._q_init)

    def play_path(self, path):
        from hpp.gepetto import PathPlayer

        pp = PathPlayer(self.v)
        pp(path)
