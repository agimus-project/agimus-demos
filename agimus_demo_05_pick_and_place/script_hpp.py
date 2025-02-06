# Copyright 2022 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from math import pi, sqrt
from hpp.corbaserver import loadServerPlugin, shrinkJointRange
from hpp.corbaserver.manipulation import Robot, newProblem, ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from bin_picking import BinPicking
from logging import getLogger
from pyquaternion import Quaternion
import os
import numpy as np

import time
import ast
import pickle

def concatenatePaths(paths):
    if len(paths) == 0: return None
    p = paths[0].asVector()
    for q in paths[1:]:
        assert(p.end() == q.initial())
        p.appendPath(q)
    return p

logger = getLogger(__name__)

print("[START]")
print(
    "To avoid crash during constrain graph building, RESTART the hppcorbaserver process once in a while."
)

connectedToRos = False

Tless_object = "tless-obj_000001"

# __________________________START_OF_GRAPH_GENERATION_________________________

package_location = os.getcwd()

print("reading generic URDF")
from hpp.rostools import process_xacro, retrieve_resource

Robot.urdfString = process_xacro(package_location + "/urdf/demo.urdf.xacro")
Robot.srdfString = ""


class Box:
    urdfFilename = package_location + "/urdf/big_box.urdf"
    srdfFilename = package_location + "/srdf/big_box.srdf"
    rootJointType = "freeflyer"


class TLess:
    if Tless_object == "tless-obj_000001":
        urdfFilename = package_location + "/urdf/obj_01.urdf"
        srdfFilename = package_location + "/srdf/obj_01.srdf"
        rootJointType = "freeflyer"



defaultContext = "corbaserver"
loadServerPlugin(defaultContext, "manipulation-corba.so")
loadServerPlugin(defaultContext, "bin_picking.so")
newProblem()

robot = Robot("robot", "panda", rootJointType="anchor")
robot.opticalFrame = "camera_color_optical_frame"
shrinkJointRange(robot, [f"panda/panda_joint{i}" for i in range(1, 8)], 0.95)
ps = ProblemSolver(robot)

ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)
ps.setParameter("SimpleTimeParameterization/safety", 0.95)
# ps.setParameter("timeOut", 1.)]
ps.setTimeOutPathPlanning(1.0)


# Add path projector to avoid discontinuities
ps.selectPathProjector("Progressive", 0.05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)
vf.loadObjectModel(TLess, "part")
vf.loadObjectModel(Box, "box")

robot.setJointBounds("part/root_joint", [-1.0, 1.5, -1.0, 1.0, 0.0, 2.2])
robot.setJointBounds("box/root_joint", [-1.0, 1.0, -1.0, 1.0, 0.0, 1.8])

print("Part and box loaded")
robot.client.manipulation.robot.insertRobotSRDFModel(
    "panda", package_location + "/srdf/demo.srdf"
)

# Remove collisions between object and self collision geometries
srdfString = '<robot name="demo">'
for i in range(1, 8):
    srdfString += f'<disable_collisions link1="panda_link{i}_sc" link2="part/base_link" reason="handled otherwise"/>'
srdfString += '<disable_collisions link1="panda_hand_sc" link2="part/base_link" reason="handled otherwise"/>'
srdfString += "</robot>"
robot.client.manipulation.robot.insertRobotSRDFModelFromString("panda", srdfString)

# Discretize handles
ps.client.manipulation.robot.addGripper(
    "panda/support_link",
    "goal/gripper1",
    [1.05, 0.0, 1.02, 0, sqrt(2) / 2, 0, sqrt(2) / 2],
    0.0,
)
ps.client.manipulation.robot.addGripper(
    "panda/support_link",
    "goal/gripper2",
    [1.05, 0.0, 1.02, 0, -sqrt(2) / 2, 0, sqrt(2) / 2],
    0.0,
)
ps.client.manipulation.robot.addHandle(
    "part/base_link",
    "part/center1",
    [0, 0, 0, 0, sqrt(2) / 2, 0, sqrt(2) / 2],
    0.03,
    3 * [True] + [False, True, True],
)
ps.client.manipulation.robot.addHandle(
    "part/base_link",
    "part/center2",
    [0, 0, 0, 0, -sqrt(2) / 2, 0, sqrt(2) / 2],
    0.03,
    3 * [True] + [False, True, True],
)

# Lock gripper in open position.
ps.createLockedJoint("locked_finger_1", "panda/panda_finger_joint1", [0.035])
ps.createLockedJoint("locked_finger_2", "panda/panda_finger_joint2", [0.035])
ps.setConstantRightHandSide("locked_finger_1", True)
ps.setConstantRightHandSide("locked_finger_2", True)

# Add handle of the objects (obj_tless-000001 : 16/16/16/16  obj_tless-000023 : 3/3/4/4)
handles = list()
if Tless_object == "tless-obj_000001":
    handles += ["part/lateral_top_%03i" % i for i in range(16)]
    handles += ["part/lateral_bottom_%03i" % i for i in range(16)]
    handles += ["part/top_%03i" % i for i in range(16)]
    handles += ["part/bottom_%03i" % i for i in range(16)]
if Tless_object == "tless-obj_000023":
    handles += ["part/lateral_top_%03i" % i for i in range(3)]
    handles += ["part/lateral_bottom_%03i" % i for i in range(3)]
    handles += ["part/top_%03i" % i for i in range(4)]
    handles += ["part/bottom_%03i" % i for i in range(4)]

binPicking = BinPicking(ps)
binPicking.objects = ["part", "box"]
binPicking.robotGrippers = ["panda/panda_gripper"]
binPicking.goalGrippers = [
    "goal/gripper1",
    "goal/gripper2",
]
binPicking.goalHandles = [
    "part/center1",
    "part/center2",
]
binPicking.handles = handles
binPicking.graphConstraints = ["locked_finger_1", "locked_finger_2"]


def disable_collision():
    srdf_disable_collisions = """<robot>\n"""
    srdf_disable_collisions_fmt = (
        """  <disable_collisions link1="{}" link2="{}" reason=""/>\n"""
    )
    srdf_disable_collisions += srdf_disable_collisions_fmt.format(
        "box/base_link", "part/base_link"
    )
    srdf_disable_collisions += "</robot>"
    robot.client.manipulation.robot.insertRobotSRDFModelFromString(
        "", srdf_disable_collisions
    )


disable_collision()

build_time_start = time.time()
print("Building constraint graph")
binPicking.buildGraph()
build_time_stop = time.time()
building_time = build_time_stop - build_time_start
print("The graph took ", building_time, "s to build.")

q0 = [
    0,
    -pi / 4,
    0,
    -3 * pi / 4,
    0,
    pi / 2,
    pi / 4,
    0.035,
    0.035,
    0,
    0,
    1.2,
    0,
    0,
    0,
    1,
    0,
    0,
    0.761,
    0,
    0,
    0,
    1,
]

# Create effector
print("Building effector.")
binPicking.buildEffectors([f"box/base_link_{i}" for i in range(5)], q0)

print("Generating goal configurations.")
binPicking.generateGoalConfigs(q0)


def split_path(path):
    path = path.flatten()
    grasp_path_idxs = [0]
    placing_path_idxs = []
    freefly_path_idxs = []

    for idx in range(1, path.numberPaths()):
        if path_move_object(path.pathAtRank(idx)):
            placing_path_idxs.append(idx)
        else:
            if len(placing_path_idxs) == 0:
                grasp_path_idxs.append(idx)
            else:
                freefly_path_idxs.append(idx)
    grasp_path = concatenatePaths([path.pathAtRank(idx) for idx in grasp_path_idxs])
    placing_path = concatenatePaths([path.pathAtRank(idx) for idx in placing_path_idxs])
    freefly_path = concatenatePaths([path.pathAtRank(idx) for idx in freefly_path_idxs])
    return grasp_path, placing_path, freefly_path


def path_move_object(path):
    object_init_pose = np.array(path.initial()[9:12])
    object_end_pose = np.array(path.end()[9:12])
    eps = 1e-4
    if np.linalg.norm(object_end_pose - object_init_pose) < eps:
        return False
    else:
        return True


# ___________________________END_OF_GRAPH_GENERATION__________________________


def GrabAndDrop(robot, ps, binPicking, q_init):

    # ____________GETTING_THE_POSE____________

    # quaternion is X, Y, Z, W

    # if acq_type == "input_config":
    #     print("[INFO] Input the config.")
    #     data_input = input("Enter the XYZQUAT : ")
    #     data_input = ast.literal_eval(data_input)
    #     quat = Quaternion(data_input[3], data_input[4], data_input[5], data_input[6])
    #     quat = quat.normalised
    #     q_input = [
    #         data_input[0],
    #         data_input[1],
    #         data_input[2],
    #         quat[0],
    #         quat[1],
    #         quat[2],
    #         quat[3],
    #     ]
    #     q_init[9:16], wMo = q_input, None

    # if acq_type == "ros_bridge_config":
    #     print("[INFO] Make sure the /happypose/detections ros topic exist !")
    #     input("Press [ENTER] to proceed ...")
    #     print("Gathering poses")
    #     pose = vision_listener.in_world_pose_object
    #     quat = Quaternion(
    #         [
    #             pose.orientation.x,
    #             pose.orientation.y,
    #             pose.orientation.z,
    #             pose.orientation.w,
    #         ]
    #     )
    #     quat = quat.normalised
    #     q_bridge = [
    #         pose.position.x,
    #         pose.position.y,
    #         pose.position.z,
    #         quat[0],
    #         quat[1],
    #         quat[2],
    #         quat[3],
    #     ]
    #     q_init[9:16], wMo = q_bridge, None

    # if (
    #     acq_type != "test_config"
    #     and acq_type != "input_config"
    #     and acq_type != "ros_bridge_config"
    #     and acq_type != "given_config"
    # ):
    #     raise ValueError("acquistion type for q_init is unknown.")
    # ___________________________________________
    
    poses = np.array(q_init[9:16])

    print(q_init)
    print("\nPose of the object : \n", poses, "\n")

    found, msg = robot.isConfigValid(q_init)

    # Resolving the path to the object
    if found:
        print("[INFO] Object found with no collision")
        print("Solving ...")
        res = False
        res, p = binPicking.solve(q_init)
        print("p", p)
        grasp_path, placing_path, freefly_path = split_path(p)
        if res:
            ps.client.basic.problem.addPath(grasp_path)
            ps.client.basic.problem.addPath(placing_path)
            ps.client.basic.problem.addPath(freefly_path)
            print("Path generated.")
        else:
            raise Warning(f"Robot q_init isn't a valid configuration {msg}")
        return q_init, p

    else:
        print("[INFO] Object found but not collision free")
        print("Trying solving without playing path for simulation ...")

    return q_init, None


# ____________________________________________________________________________


if __name__ == "__main__":
    """ 
    To run from command line:
    cd src/agimus-demos/agimus_demo_04_hpp_mpc
    ROS_PACKAGE_PATH=`pwd` hppcorbaserver
    """
    print("Script HPP ready !")
    q_init = q0 = [
        0,
        -pi / 4,
        0,
        -3 * pi / 4,
        0,
        pi / 2,
        pi / 4,
        0.035,
        0.035,
        0,
        0,
        1.2,
        0,
        0,
        0,
        1,
        0,
        0,
        0.761,
        0,
        0,
        0,
        1,
    ]
    res, q_init, err = binPicking.graph.applyNodeConstraints("free", q_init)

    list_of_cam_pose = np.zeros(shape=(10, 4, 4))
    list_of_images = np.zeros(shape=(10, 720, 1280, 3), dtype=np.uint8)
    list_of_q = np.zeros(shape=(10, 7))

    # Type of data acquisition
    test_config = "test_config"
    input_config = "input_config"
    ros_bridge_config = "ros_bridge_config"
    given_config = "given_config :"
    q_init, path_id = GrabAndDrop(robot, ps, binPicking, q_init)
    path_len = ps.pathLength(path_id)
    configs = [ps.configAtParam(path_id, t) for t in np.linspace(0, path_len, 100) ]
    pickle.dump(configs, open("q_init.pkl", "wb"))