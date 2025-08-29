import pathlib

from hpp.corbaserver import wrap_delete as wd
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    loadServerPlugin,
    ProblemSolver,
    Rule,
    Robot,
)  # noqa: F811

from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory

from hpp.rostools import retrieve_resource

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()


class BaseObject(object):
    rootJointType = "freeflyer"

    def __init__(self, urdf_path: str, srdf_path: str, name: str):
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


# Load the robot
package_location = "package://agimus_demo_07_deburring"
Robot.urdfString = pathlib.Path("/home/gepetto/ros2_ws/src/panda.urdf").read_text()
Robot.srdfString = pathlib.Path("/home/gepetto/ros2_ws/src/panda.srdf").read_text()

# Load the object
pylone_object = BaseObject(
    urdf_path=retrieve_resource(f"{package_location}/urdf/pylone.urdf"),
    srdf_path=retrieve_resource(f"{package_location}/srdf/pylone.srdf"),
    name="pylone",
)
# Define which handles to use
pylone_used_handles = ["pylone/hole_02", "pylone/hole_03", "pylone/hole_01"]


robot = Robot("robot", "panda", rootJointType="anchor")


robot.client.manipulation.robot.insertRobotSRDFModel(
    "panda",
    retrieve_resource("package://agimus_demo_07_deburring/srdf/demo.srdf"),
)

ps = ProblemSolver(robot)

vf = ViewerFactory(ps)
vf.loadObjectModel(pylone_object, pylone_object.name)
v = vf.createViewer()

# TODO check what bounds have to be set here
robot.setJointBounds(
    f"{pylone_object.name}/root_joint", [-1.0, 1.0, -1.0, 1.0, 0.0, 2.2]
)
shrinkJointRange(robot, [f"panda/fer_joint{i}" for i in range(1, 8)], 0.95)

# Set problem solver params
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

# TODO fingers tips are not locking
for i in range(1, 3):
    ps.createLockedJoint(f"locked_finger_{i}", f"panda/fer_finger_joint{i}", [0.0])
    ps.setConstantRightHandSide(f"locked_finger_{i}", True)

# Load plugins
ps.addPathOptimizer("RandomShortcut")

ps.addPathOptimizer("SimpleTimeParameterization")
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.5)
ps.setParameter("SimpleTimeParameterization/safety", 0.95)

cg = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory(cg)
factory.setGrippers(["panda/panda_gripper"])
factory.setObjects([pylone_object.name], [pylone_used_handles], [[]])
factory.setRules([Rule([".*"], [".*"], True)])
factory.generate()
cg.initialize()


# Set robot state
q_init = robot.getCurrentConfig()
q = [0.0, 0.1865, 0.0, -2.4, 0.0, 2.5915, 0.7863]
for i, position in enumerate(q):
    rank = robot.rankInConfiguration[f"panda/fer_joint{i + 1}"]
    q_init[rank] = position

rank = robot.rankInConfiguration[f"{pylone_object.name}/root_joint"]
q_init[rank : rank + 3] = [1.2, -0.5, 0.0]
q_init[rank + 3 : rank + 7] = [1.0, 0.0, 0.0, 0.0]

ps.setInitialConfig(q_init)
v(q_init)

# TODO generate order of the transitions and corresponding paths
# TODO obtain a discrete path, that is discretized with dt and not number of samples
# TODO obtain joint position, end effector position, joint velocity and optionally joint accelerations
# TODO get information which executed path corresponds to which part of motion to adjust MPC gains


# Current implementation


def sample_state_on_transition_target(
    q_from: list[float],
    transition: str,
    max_iter: int = 1,
) -> list[float]:
    for _ in range(max_iter):
        succ, q1, _ = cg.generateTargetConfig(
            transition, q_from, robot.shootRandomConfig()
        )
        if succ:
            res, _ = robot.isConfigValid(q1)  # Collision checking
            if res:
                return q1
    raise RuntimeError("Failed to find feasible configuration!")


def generate_transition(transition: str, q_init: list[float]):
    q_final = sample_state_on_transition_target(
        q_from=q_init,
        transition=transition,
        max_iter=100,
    )
    ps.setInitialConfig(q_init)
    ps.resetGoalConfigs()
    ps.addGoalConfig(q_final.copy())
    ps.solve()

    return q_final.copy()


pp = PathPlayer(v)
for handle_name in pylone_used_handles:
    input("Press enter to continue")
    # Generate pregrasp
    q_init = generate_transition(f"panda/panda_gripper > {handle_name} | f_01", q_init)
    pp(ps.numberPaths() - 1)
    input("Press enter to continue")
    # Generate grasp
    q_init = generate_transition(f"panda/panda_gripper > {handle_name} | f_12", q_init)
    pp(ps.numberPaths() - 1)


# New implementation


# Create Transition Planner
cproblem = wd(ps.hppcorba.problem.getProblem())
cgraph = wd(cproblem.getConstraintGraph())
crobot = wd(cproblem.robot())
cdistance = wd(ps.hppcorba.problem.createDistance("Weighed", cproblem))
croadmap = wd(ps.client.manipulation.problem.createRoadmap(cdistance, crobot))
croadmap.constraintGraph(cgraph)
transitionPlanner = wd(
    ps.client.basic.problem.createPathPlanner("TransitionPlanner", cproblem, croadmap)
)
transitionPlanner.setPathProjector("Progressive", 0.02)
transitionPlanner.maxIterations(100)

# transition planner is the same as ins tate planner
# when using graph class there is a dict to go from edges and nodes to IDs
transitionPlanner.setEdge(cgraph.edges["Loop | f"])

# tip. Compute grasp motion before computing motion to a new configuration
# calling reverse on a path does not always work, it is better to do it with directPath
# there is no need to concatenate path

# Everywhere in the demo with UR10 replace inStatePlanner with TransitionPlanner


# # Path discretization
# p1 = transitionPlanner.planPath(qinit, [qpg], resetRoadmap=True)
# # If parametyrization is time this is time
# max_time = p1.length()
# # Gives a discrete path
# p1.call(max_time)

# form the python file provided look at:
# - generatePathToConifg and add pregrasp and grasp
# - traverse handles


# def generatePathForHandle(handle, qinit=None, NrandomConfig=100, step=3):
#     qinit = self.checkQInit(qinit)
#     # generate configurations
#     edge = tool_gripper + " > " + handle
#     for nTrial in range(NrandomConfig):
#         res, qpg, qg = self.generateValidConfigForHandle(
#             handle=handle,
#             qinit=qinit,
#             qguesses=[qinit],
#             NrandomConfig=NrandomConfig,
#             step=step,
#         )
#         if not res:
#             continue
#         # build path
#         # from qinit to pregrasp
#         self.inStatePlanner.setEdge(edge + " | f_01")
#         try:
#             p1 = transitionPlanner.planPath(qinit, [qpg], resetRoadmap=True)
#         except:
#             p1 = None
#         if not p1:
#             continue
#         if step < 2:
#             return p1
#         # from pregrasp to grasp
#         self.inStatePlanner.setEdge(edge + " | f_12")
#         res, p2, msg = transitionPlanner.directPath(qpg, qg, True)
#         if not res:
#             p2 = None
#         if not p2:
#             continue
#         # Return concatenation
#         if step < 3:
#             return concatenatePaths([p1, p2])
#         # back to pregrasp
#         p3 = self.wd(p2.reverse())
#         return concatenatePaths([p1, p2, p3])
#     raise RuntimeError("failed to compute a path.")
