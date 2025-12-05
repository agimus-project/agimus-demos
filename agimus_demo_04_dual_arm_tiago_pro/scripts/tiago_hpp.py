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
robot = Robot("tiago_pro-manip", "tiago_pro", rootJointType = "planar")
robot.client.manipulation.robot.insertRobotSRDFModelFromString("", srdfString)
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
    [0, 0, .18, 0, -c, 0, c], .1)
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_right_7_link", "tiago_pro/right",
    [0, 0, .18, 0, -c, 0, c], .1)
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link", "reinforcment_bar/left",
    [0, 0.01, -.25, 0, 0, -c, c], .1, 6*[True])
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link", "reinforcment_bar/right",
    [0, 0.01, .25, 0, 0, -c, c], .1, 6*[True])

# Set initial configuration
q0 = robot.getCurrentConfig()

# Lock gripper joints
lockedGrippers = {
    'tiago_pro/gripper_left_finger_joint': .1,
    'tiago_pro/gripper_left_inner_finger_left_joint': -.1,
    'tiago_pro/gripper_left_fingertip_left_joint': .1,
    'tiago_pro/gripper_left_finger_right_joint': 0,
    'tiago_pro/gripper_left_inner_finger_right_joint': -.1,
    'tiago_pro/gripper_left_fingertip_right_joint': .1,
    'tiago_pro/gripper_left_outer_finger_left_joint': -.1,
    'tiago_pro/gripper_left_outer_finger_right_joint': -.1,
    'tiago_pro/gripper_right_finger_joint': .1,
    'tiago_pro/gripper_right_inner_finger_left_joint': -.1,
    'tiago_pro/gripper_right_fingertip_left_joint': .1,
    'tiago_pro/gripper_right_finger_right_joint': 0,
    'tiago_pro/gripper_right_inner_finger_right_joint': -.1,
    'tiago_pro/gripper_right_fingertip_right_joint': .1,
    'tiago_pro/gripper_right_outer_finger_left_joint': -.1,
    'tiago_pro/gripper_right_outer_finger_right_joint': -.1,
    }
for j, v in lockedGrippers.items():
    ps.createLockedJoint(j, j, [v])

r = robot.rankInConfiguration["reinforcment_bar/root_joint"]
q0[r:r+7] = [.6, 0, .5, c, 0, 0, c]

# Create constraint graph
cg = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory (cg)
factory.setGrippers (["tiago_pro/left", "tiago_pro/right"])
factory.setObjects (["reinforcment_bar"], [["reinforcment_bar/left", "reinforcment_bar/right"]],
                    [[]])
possibleGrasps = {"tiago_pro/left" : ["reinforcment_bar/left"],
                  "tiago_pro/right": ["reinforcment_bar/right"]}
factory.setPossibleGrasps(possibleGrasps)
factory.generate ()
cg.addConstraints(graph=True, constraints = Constraints(numConstraints = lockedGrippers.keys()))
cg.initialize ()

res, q1, err = cg.applyNodeConstraints('free', q0)