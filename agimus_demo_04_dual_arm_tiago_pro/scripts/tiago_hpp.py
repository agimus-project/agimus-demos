from math import pi, sqrt
from rostools import process_xacro
from hpp.corbaserver import loadServerPlugin, wrap_delete as wd
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Constraints,
    ProblemSolver,
    Robot
)
from hpp.gepetto.manipulation import ViewerFactory
from viewer import displayGripper, displayHandle

# Load Tiago pro robot and Reinforcment bar
class ReinforcmentBar:
    rootJointType = "freeflyer"
    urdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/urdf/reinforcment_bar.urdf"
    srdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/srdf/reinforcment_bar.srdf"

#Robot.urdfFilename = (
#    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf"
#)
#Robot.srdfFilename = ""

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

urdf_xacro = "package://tiago_pro_description/robots/tiago_pro.urdf.xacro"
srdf_xacro = "package://tiago_pro_moveit_config/config/srdf/tiago_pro.srdf.xacro"
Robot.urdfString = process_xacro(urdf_xacro, "end_effector_left:=pal-pro-gripper",
    "end_effector_right:=pal-pro-gripper").replace("file://", "")
Robot.srdfString = ""
srdfString = process_xacro(srdf_xacro, "end_effector_left:=pal-pro-gripper",
                           "end_effector_right:=pal-pro-gripper")
# Additional collision pairs to remove
# remove </robot> from file
i = srdfString.find("</robot>")
assert(i!=-1)
srdfString = srdfString[:i]
for l1, l2 in [("base_link", "wheel_front_left_link"),
    ("base_link", "wheel_front_right_link") ,
    ("base_link", "wheel_rear_left_link"),
    ("base_link", "wheel_rear_right_link"),
    ("gripper_left_screw_left_link", "gripper_left_fingertip_left_link"),
    ("gripper_right_screw_left_link", "gripper_right_fingertip_left_link")
    ]:
    srdfString += f'  <disable_collisions link1="{l1}" link2="{l2}" reason="Never"/>\n'
srdfString += "</robot>"
robot = Robot("tiago_pro-manip", "tiago_pro", rootJointType = "planar")
robot.client.manipulation.robot.insertRobotSRDFModelFromString("tiago_pro", srdfString)
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
vf.loadObjectModel(ReinforcmentBar, "reinforcment_bar")

# Set joint bounds
robot.setJointBounds("reinforcment_bar/root_joint", [-3,3,-3,3,0,2,-1,1,-1,1,-1,1,-1,1])
robot.setJointBounds("tiago_pro/root_joint", [-3, 3, -3, 3, -1, 1, -1, 1]) 

# Define grippers and handles
c = sqrt(2)/2
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_left_7_link", "tiago_pro/left",
    [0, 0, .19, 0, -c, 0, c], .02)
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_right_7_link", "tiago_pro/right",
    [0, 0, .19, 0, -c, 0, c], .02)
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link", "reinforcment_bar/left",
    [0, 0.01, -.25, 0, 0, -c, c], .05, 6*[True])
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link", "reinforcment_bar/right",
    [0, 0.01, .25, 0, 0, -c, c], .05, 6*[True])

# Create lists of locked joints
ps.createLockedJoint("locked_tiago_pro/root_joint", "tiago_pro/root_joint",
                     [0, 0, 1, 0])
ps.setConstantRightHandSide("locked_tiago_pro/root_joint", False)
lockedGrippers = {
    'tiago_pro/gripper_left_finger_joint': .05,
    'tiago_pro/gripper_left_inner_finger_left_joint': -.05,
    'tiago_pro/gripper_left_fingertip_left_joint': .05,
    'tiago_pro/gripper_left_finger_right_joint': 0,
    'tiago_pro/gripper_left_inner_finger_right_joint': -.05,
    'tiago_pro/gripper_left_fingertip_right_joint': .05,
    'tiago_pro/gripper_left_outer_finger_left_joint': -.05,
    'tiago_pro/gripper_left_outer_finger_right_joint': -.05,
    'tiago_pro/gripper_right_finger_joint': .05,
    'tiago_pro/gripper_right_inner_finger_left_joint': -.05,
    'tiago_pro/gripper_right_fingertip_left_joint': .05,
    'tiago_pro/gripper_right_finger_right_joint': 0,
    'tiago_pro/gripper_right_inner_finger_right_joint': -.05,
    'tiago_pro/gripper_right_fingertip_right_joint': .05,
    'tiago_pro/gripper_right_outer_finger_left_joint': -.05,
    'tiago_pro/gripper_right_outer_finger_right_joint': -.05,
    }
locked_grippers = list()
for j, v in lockedGrippers.items():
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [v])
    locked_grippers.append(constraint)

lockedHead = {"tiago_pro/head_1_joint" : 0,
              "tiago_pro/head_2_joint" : 0}
locked_head = list()
for j, v in lockedHead.items():
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [v])
    locked_head.append(constraint)

locked_arms_and_torso = list()
for j in filter(lambda s:s.startswith("tiago_pro/") and
    not s.startswith("tiago_pro/head") and
    not s.startswith("tiago_pro/wheel") and
    s != "tiago_pro/root_joint", robot.jointNames):
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [0])
    locked_arms_and_torso.append(constraint)
    ps.setConstantRightHandSide(constraint, False)

locked_wheels = list()
for j in ["tiago_pro/wheel_front_left_joint", "tiago_pro/wheel_front_right_joint",
    "tiago_pro/wheel_rear_left_joint", "tiago_pro/wheel_rear_right_joint"]:
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [1, 0])
    locked_wheels.append(constraint)
    ps.setConstantRightHandSide(constraint, True)

# Create constraint graph
cg = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory (cg)
factory.setGrippers (["tiago_pro/left"])
factory.setObjects (["reinforcment_bar"], [["reinforcment_bar/left"]], [[]])
factory.generate ()
# # Add a transition to move the base keeping all other joints fixed
# cg.createEdge("free", "free", "move_base", 1, "free")
# # Add a transition to move the base while holding the bar
# n = "tiago_pro/left grasps reinforcment_bar/left"
# cg.createEdge(n, n, "move_base_holding_bar", 1, n)
# Lock grippers and head all the time
cg.addConstraints(graph=True,
    constraints = Constraints(numConstraints = locked_grippers + locked_head +
    locked_wheels))
# When moving the arms, do not move the base
# for e in ['Loop | f', 'Loop | 0-0', 'tiago_pro/left < reinforcment_bar/left | 0-0',
#     'tiago_pro/left > reinforcment_bar/left | f_01',
#     'tiago_pro/left > reinforcment_bar/left | f_12',
#     'tiago_pro/left < reinforcment_bar/left | 0-0_10',
#     'tiago_pro/left < reinforcment_bar/left | 0-0_21',]:
#     cg.addConstraints(edge = e,
#         constraints = Constraints(numConstraints = ["locked_tiago_pro/root_joint"]))
# When moving the base, do not move the arms and torso
# cg.addConstraints(edge = "move_base_holding_bar", constraints = Constraints(
#     numConstraints = locked_arms_and_torso
# ))
# cg.addConstraints(edge = "move_base", constraints = Constraints(
#     numConstraints = locked_arms_and_torso + ["reinforcment_bar/root_joint"]
#))
# Add other pregrasp constraints in pregrasp states
g = "tiago_pro/right"
h = "reinforcment_bar/right"
cg.createGrasp(f"{g} grasps {h}", g, h)
cg.createPreGrasp(f"{g} pregrasps {h}", g, h)
cg.addConstraints(node = "tiago_pro/left > reinforcment_bar/left | f_pregrasp",
    constraints = Constraints(
        numConstraints=["tiago_pro/right pregrasps reinforcment_bar/right"]))
cg.addConstraints(node = "tiago_pro/left grasps reinforcment_bar/left",
    constraints = Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]))
cg.initialize ()

# Set initial configuration
q0 = robot.getCurrentConfig()
r = robot.rankInConfiguration["reinforcment_bar/root_joint"]
q0[r:r+7] = [.7, 0, .6, c, 0, 0, c]

res, q_init, err = cg.applyNodeConstraints('free', q0)
q_goal = q_init[:]
q_goal[r:r+7] = [-.7, 0, .6, 0, c, c, 0]

# Load path optimizers
ps.loadPlugin('spline-gradient-based.so')
ps.addPathOptimizer("SplineGradientBased_bezier3")

ps.selectPathProjector("Progressive", .1)
ps.setInitialConfig(q_init)
#ps.addGoalConfig(q_goal)
#ps.selectPathPlanner("StatesPathFinder")
#ps.solve()
