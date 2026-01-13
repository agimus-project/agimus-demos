#!/usr/bin/env python
# Copyright (c) 2025 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

from math import sqrt
from rostools import process_xacro
from helper import Helper
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


# Load Tiago pro robot and Reinforcment bar
class Table:
    rootJointType = "anchor"
    urdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/urdf/table.urdf"
    srdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/srdf/table.srdf"


class Plate:
    rootJointType = "freeflyer"
    urdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/urdf/plate.urdf"
    srdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/srdf/plate.srdf"


class ReinforcmentBar:
    rootJointType = "freeflyer"
    urdfFilename = (
        "package://agimus_demo_04_dual_arm_tiago_pro/urdf/reinforcment_bar.urdf"
    )
    srdfFilename = (
        "package://agimus_demo_04_dual_arm_tiago_pro/srdf/reinforcment_bar.srdf"
    )


loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

urdf_xacro = "package://tiago_pro_description/robots/tiago_pro.urdf.xacro"
srdf_xacro = "package://tiago_pro_moveit_config/config/srdf/tiago_pro.srdf.xacro"
Robot.urdfString = process_xacro(
    urdf_xacro,
    "end_effector_left:=pal-pro-gripper",
    "end_effector_right:=pal-pro-gripper",
).replace("file://", "")
Robot.srdfString = ""
srdfString = process_xacro(
    srdf_xacro,
    "end_effector_left:=pal-pro-gripper",
    "end_effector_right:=pal-pro-gripper",
)
# Additional collision pairs to remove
# remove </robot> from file
i = srdfString.find("</robot>")
assert i != -1
srdfString = srdfString[:i]
for l1, l2 in [
    ("base_link", "wheel_front_left_link"),
    ("base_link", "wheel_front_right_link"),
    ("base_link", "wheel_rear_left_link"),
    ("base_link", "wheel_rear_right_link"),
    ("gripper_left_screw_left_link", "gripper_left_fingertip_left_link"),
    ("gripper_right_screw_left_link", "gripper_right_fingertip_left_link"),
]:
    srdfString += f'  <disable_collisions link1="{l1}" link2="{l2}" reason="Never"/>\n'
srdfString += "</robot>"
robot = Robot("tiago_pro-manip", "tiago_pro", rootJointType="planar")
robot.client.manipulation.robot.insertRobotSRDFModelFromString("tiago_pro", srdfString)
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
vf.loadObjectModel(Table, "table")
vf.loadObjectModel(Plate, "plate")
vf.loadObjectModel(ReinforcmentBar, "reinforcment_bar")

# ps.selectPathValidation("NoValidation", 1)

# Set joint bounds
robot.setJointBounds(
    "plate/root_joint", [-3, 3, -3, 3, 0, 2, -1, 1, -1, 1, -1, 1, -1, 1]
)
robot.setJointBounds(
    "reinforcment_bar/root_joint", [-3, 3, -3, 3, 0, 2, -1, 1, -1, 1, -1, 1, -1, 1]
)
robot.setJointBounds("tiago_pro/root_joint", [0.1, 2.6, -2, 0.05, -1, 1, -1, 1])

# Define grippers and handles
c = sqrt(2) / 2
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_left_7_link", "tiago_pro/left", [0, 0, 0.19, 0, -c, 0, c], 0.02
)
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_right_7_link", "tiago_pro/right", [0, 0, 0.19, 0, -c, 0, c], 0.02
)
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link",
    "reinforcment_bar/left",
    [0, 0.01, -0.25, 0, 0, -c, c],
    0.05,
    6 * [True],
)
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link",
    "reinforcment_bar/right",
    [0, 0.01, 0.25, 0, 0, -c, c],
    0.05,
    6 * [True],
)

# Create lists of locked joints
ps.createLockedJoint(
    "locked_tiago_pro/root_joint", "tiago_pro/root_joint", [0, 0, 1, 0]
)
ps.setConstantRightHandSide("locked_tiago_pro/root_joint", False)
lockedGrippers = {
    "tiago_pro/gripper_left_finger_joint": 0.05,
    "tiago_pro/gripper_left_inner_finger_left_joint": -0.05,
    "tiago_pro/gripper_left_fingertip_left_joint": 0.05,
    "tiago_pro/gripper_left_finger_right_joint": 0,
    "tiago_pro/gripper_left_inner_finger_right_joint": -0.05,
    "tiago_pro/gripper_left_fingertip_right_joint": 0.05,
    "tiago_pro/gripper_left_outer_finger_left_joint": -0.05,
    "tiago_pro/gripper_left_outer_finger_right_joint": -0.05,
    "tiago_pro/gripper_right_finger_joint": 0.05,
    "tiago_pro/gripper_right_inner_finger_left_joint": -0.05,
    "tiago_pro/gripper_right_fingertip_left_joint": 0.05,
    "tiago_pro/gripper_right_finger_right_joint": 0,
    "tiago_pro/gripper_right_inner_finger_right_joint": -0.05,
    "tiago_pro/gripper_right_fingertip_right_joint": 0.05,
    "tiago_pro/gripper_right_outer_finger_left_joint": -0.05,
    "tiago_pro/gripper_right_outer_finger_right_joint": -0.05,
}
locked_grippers = list()
for j, v in lockedGrippers.items():
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [v])
    locked_grippers.append(constraint)

lockedHead = {"tiago_pro/head_1_joint": 0, "tiago_pro/head_2_joint": 0}
locked_head = list()
for j, v in lockedHead.items():
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [v])
    locked_head.append(constraint)

locked_arms_and_torso = list()
for j in filter(
    lambda s: s.startswith("tiago_pro/")
    and not s.startswith("tiago_pro/head")
    and not s.startswith("tiago_pro/wheel")
    and s != "tiago_pro/root_joint",
    robot.jointNames,
):
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [0])
    locked_arms_and_torso.append(constraint)
    ps.setConstantRightHandSide(constraint, False)

locked_wheels = list()
for j in [
    "tiago_pro/wheel_front_left_joint",
    "tiago_pro/wheel_front_right_joint",
    "tiago_pro/wheel_rear_left_joint",
    "tiago_pro/wheel_rear_right_joint",
]:
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [1, 0])
    locked_wheels.append(constraint)
    ps.setConstantRightHandSide(constraint, True)

# Lock plate since for some reason, the factory does not.
ps.createLockedJoint(
    "locked_plate/root_joint", "plate/root_joint", [0, 0, 0, 0, 0, 0, 1]
)
ps.setConstantRightHandSide("locked_plate/root_joint", False)

# Create constraint graph
cg = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory(cg)
factory.setGrippers(["tiago_pro/left"])
factory.environmentContacts(["table/reinforcment_bar_support", "plate/top"])
factory.setObjects(
    ["reinforcment_bar"], [["reinforcment_bar/left"]], [["reinforcment_bar/bottom"]]
)
factory.generate()
# Add a state and transition to project on 'free' with static tiago_pro and plate
cg.createNode("unconstrained")
cg.createEdge("unconstrained", "free", "project-on-free", 1, "unconstrained")
# Lock wheels, head and grippers everywhere
cg.addConstraints(
    graph=True,
    constraints=Constraints(
        numConstraints=locked_grippers + locked_head + locked_wheels
    ),
)

# Add other pregrasp-grasp constraint in pregrasp|intersec|preplace states
g = "tiago_pro/right"
h = "reinforcment_bar/right"
cg.createGrasp(f"{g} grasps {h}", g, h)
cg.createPreGrasp(f"{g} pregrasps {h}", g, h)
cg.addConstraints(
    node="tiago_pro/left > reinforcment_bar/left | f_pregrasp",
    constraints=Constraints(
        numConstraints=["tiago_pro/right pregrasps reinforcment_bar/right"]
    ),
)
cg.addConstraints(
    node="tiago_pro/left > reinforcment_bar/left | f_intersec",
    constraints=Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]
    ),
)
cg.addConstraints(
    node="tiago_pro/left > reinforcment_bar/left | f_preplace",
    constraints=Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]
    ),
)
cg.addConstraints(
    node="tiago_pro/left grasps reinforcment_bar/left",
    constraints=Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]
    ),
)

# Lock plate in all transitions
for e in cg.edges.keys():
    cg.addConstraints(
        edge=e, constraints=Constraints(numConstraints=["locked_plate/root_joint"])
    )

cg.setWeight("Loop | f", 1)
cg.setWeight("Loop | 0-0", 1)
cg.initialize()

# Set initial configuration
q0 = robot.getCurrentConfig()
r = robot.rankInConfiguration["tiago_pro/root_joint"]
q0[r : r + 4] = [2.6, 0, -1, 0]
r = robot.rankInConfiguration["plate/root_joint"]
q0[r : r + 3] = [0.6, 0, 0.66]
r = robot.rankInConfiguration["reinforcment_bar/root_joint"]
q0[r : r + 7] = [
    1.20,
    -0.0009939583742700046,
    0.6680938848666721,
    0.12097379466237763,
    0.6966816640367284,
    0.6966816640367284,
    -0.12097379466237763,
]
res, q_init, err = cg.applyNodeConstraints("free", q0)
q_goal = q_init[:]
q_goal[r : r + 7] = [0.2, 0, 0.7, 0, c, c, 0]
res, q_goal, err = cg.generateTargetConfig("project-on-free", q_goal, q_goal)
assert res
# Load path optimizers
ps.loadPlugin("spline-gradient-based.so")
# ps.addPathOptimizer("SplineGradientBased_bezier3")

ps.selectPathProjector("Progressive", 0.1)
ps.addPathOptimizer("RandomShortcut")
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 20)
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

helper = Helper(ps, cg)
t = ps.solve()
helper.optimizePath(ps.numberPaths() - 1)
