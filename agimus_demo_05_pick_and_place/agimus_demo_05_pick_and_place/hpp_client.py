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

from math import sqrt
from agimus_demo_05_pick_and_place.corba import CorbaServer
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import Robot, newProblem, ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from agimus_demo_05_pick_and_place.bin_picking import BinPicking
import numpy as np
from pathlib import Path

import time

from agimus_demo_05_pick_and_place.utils import (
    split_path,
    BaseObject,
    get_obj_goal_handles,
)
from hpp.rostools import process_xacro
from agimus_controller.trajectory import TrajectoryPoint


class HPPInterface:
    """This interface assumes that there is one object to manipulate and one moving obstacle.
    Both the object and the obstacle are encoded in the space
    Graph creation is specific to this setup
    """

    def __init__(
        self,
        object_name: str = "obj_01",
        robot_urdf_string: str = "",
        robot_srdf_string: str = "",
        start_obj_pose: list[float] = [0.0, 0.1, 0.9, 0.0, 0.0, 0.0, 1.0],
        goal_obj_pose: list[float] = [0.0, -0.3, 0.95, 0.0, 0.0, 0.0, 1.0],
    ):
        self.start_obj_pose = start_obj_pose
        self.goal_obj_pose = goal_obj_pose

        self.default_obstacle_pose = [-0.99, -0.99, 0.761, 0.0, 0.0, 0.0, 1.0]
        self.default_object_bounds = [-1.0, 1.5, -1.0, 1.0, 0.0, 2.2]
        # TODO: maybe this should be a parameter
        package_location = Path(__file__).parent
        urdf_string = (
            process_xacro(str(package_location / "urdf/demo.urdf.xacro"))
            if robot_urdf_string == ""
            else robot_urdf_string
        )
        Robot.urdfString = urdf_string
        Robot.srdfString = robot_srdf_string

        self.manip_object = BaseObject(
            urdf_path=str(package_location / f"urdf/{object_name}.urdf"),
            srdf_path=str(package_location / f"srdf/{object_name}.srdf"),
            name="part",
        )
        self.obstacle_object = BaseObject(
            urdf_path=str(package_location / "urdf/big_box.urdf"),
            srdf_path=str(package_location / "srdf/big_box.srdf"),
            name="box",
        )
        # Init corbaserver
        self.corba = CorbaServer()
        self.setup_problem()

    def set_robot(self):
        self.robot = Robot("robot", "panda", rootJointType="anchor")
        # self.robot.opticalFrame = "camera_color_optical_frame"
        # TODO: get joint names automatically
        shrinkJointRange(
            self.robot, [f"panda/panda_joint{i}" for i in range(1, 8)], 0.95
        )

    def set_problem(self):
        # Setup problem solver and parameters
        self.ps = ProblemSolver(self.robot)
        self.ps.addPathOptimizer("EnforceTransitionSemantic")
        self.ps.addPathOptimizer("SimpleTimeParameterization")
        self.ps.setParameter("SimpleTimeParameterization/order", 2)
        self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)
        self.ps.setParameter("SimpleTimeParameterization/safety", 0.95)

        # Add path projector to avoid discontinuities
        self.ps.selectPathProjector("Progressive", 0.05)
        self.ps.selectPathValidation("Graph-Progressive", 0.01)

        vf = ViewerFactory(self.ps)
        # load the object
        vf.loadObjectModel(self.manip_object, self.manip_object.name)
        self.robot.setJointBounds(
            f"{self.manip_object.name}/root_joint", self.default_object_bounds
        )

        # load moving obstacle
        vf.loadObjectModel(self.obstacle_object, self.obstacle_object.name)
        self.robot.setJointBounds(
            f"{self.obstacle_object.name}/root_joint", self.default_object_bounds
        )
        print("Part and box loaded")
        # TODO: think about this, maybe as a parameter
        self.robot.client.manipulation.robot.insertRobotSRDFModel(
            "panda",
            str(Path(__file__).parent / "srdf/demo.srdf"),
        )
        # Remove collisions between object and self collision geometries
        # TODO: get link names automatically
        srdfString = '<robot name="demo">'
        for i in range(1, 8):
            srdfString += f'<disable_collisions link1="panda_link{i}_sc" link2="{self.manip_object.name}/base_link" reason="handled otherwise"/>'
        srdfString += f'<disable_collisions link1="panda_hand_sc" link2="{self.manip_object.name}/base_link" reason="handled otherwise"/>'
        srdfString += "</robot>"
        self.robot.client.manipulation.robot.insertRobotSRDFModelFromString(
            "panda", srdfString
        )

        # Create robot gripper handle, x-axis is facing down (for pregrasp)
        self.ps.client.manipulation.robot.addGripper(
            "panda/support_link",
            "goal/gripper",
            self.goal_obj_pose[:3] + [0, sqrt(2) / 2, 0, sqrt(2) / 2],
            0.0,
        )

        # Lock gripper in open position.
        # TODO: pass this as a parameter?
        self.ps.createLockedJoint(
            "locked_finger_1", "panda/panda_finger_joint1", [0.035]
        )
        self.ps.createLockedJoint(
            "locked_finger_2", "panda/panda_finger_joint2", [0.035]
        )
        self.ps.setConstantRightHandSide("locked_finger_1", True)
        self.ps.setConstantRightHandSide("locked_finger_2", True)
        # Add handle of the objects
        self.handles, self.goal_handles = get_obj_goal_handles(
            prefix=self.manip_object.name + "/",
            srdf_path=self.manip_object.srdfFilename,
        )

    def setup_problem(self):
        newProblem()
        self.set_robot()
        self.set_problem()

    def restart(self):
        self.corba.reset_problem()
        self.setup_problem()

    def plan(self, q_init: list[float], q_goal: list[float] = None):
        object_static = np.isclose(self.start_obj_pose, self.goal_obj_pose).all()
        self.q_init = q_init + self.start_obj_pose + self.default_obstacle_pose
        if q_goal is None:
            q_goal = q_init.copy()
        self.q_goal = q_goal + self.goal_obj_pose + self.default_obstacle_pose

        self.binPicking = BinPicking(self.ps)
        self.binPicking.objects = [self.manip_object.name, self.obstacle_object.name]
        self.binPicking.robotGrippers = ["panda/panda_gripper"]
        self.binPicking.goalGrippers = ["goal/gripper"]
        self.binPicking.goalHandles = self.goal_handles
        self.binPicking.handles = self.handles
        self.binPicking.graphConstraints = ["locked_finger_1", "locked_finger_2"]

        # TODO: restructure this
        def disable_collision():
            srdf_disable_collisions = """<robot>\n"""
            srdf_disable_collisions_fmt = (
                """  <disable_collisions link1="{}" link2="{}" reason=""/>\n"""
            )
            srdf_disable_collisions += srdf_disable_collisions_fmt.format(
                "box/base_link", "part/base_link"
            )
            srdf_disable_collisions += "</robot>"
            self.robot.client.manipulation.robot.insertRobotSRDFModelFromString(
                "", srdf_disable_collisions
            )

        disable_collision()

        build_time_start = time.time()
        print("Building constraint graph")
        self.binPicking.buildGraph()
        build_time_stop = time.time()
        building_time = build_time_stop - build_time_start
        print("The graph took ", building_time, "s to build.")

        # Create effector
        if not object_static:
            print("Building effector.")  # this hardcodes sequence of steps
            self.binPicking.buildEffectors(
                [f"box/base_link_{i}" for i in range(5)], self.q_init
            )

            print("Generating goal configurations.")
            self.binPicking.generateGoalConfigs(self.q_goal)

        res, q_init, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_init
        )
        assert res, f"Robot q_init isn't a valid configuration {err}"
        poses = np.array(q_init[9:16])

        print(q_init)
        print("\nPose of the object : \n", poses, "\n")

        found, msg = self.robot.isConfigValid(q_init)

        # Resolving the path to the object
        if found:
            print("[INFO] Object found with no collision")
            print("Solving ...")
            res = False
            if object_static:
                self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.05)
                p, res, _ = self.binPicking.transitionPlanner.directPath(
                    q_init, self.q_goal, True
                )
                print("Achieve to create a direct path : ", res)
                if not res:
                    p = self.binPicking.transitionPlanner.planPath(
                        q_init,
                        [
                            self.q_goal,
                        ],
                        True,
                    )
                print("Free path", p)
                return p
            else:
                res, p = self.binPicking.solve(q_init)
                print("Grasping path", p)
                if res:
                    grasp_path, placing_path, freefly_path = split_path(p)
                    self.ps.client.basic.problem.addPath(grasp_path)
                    self.ps.client.basic.problem.addPath(placing_path)
                    self.ps.client.basic.problem.addPath(freefly_path)
                    print("Path generated.")
                    return grasp_path, placing_path, freefly_path
                else:
                    print("No solution")
                    return None

        else:
            print("[INFO] Object found but not collision free")
            print("Trying solving without playing path for simulation ...")
            return


# ____________________________________________________________________________
def get_traj_points_from_path(hpp_path, robot_ndof=7, dt=0.01):
    total_time = hpp_path.length()
    print(total_time)
    T = int(total_time / dt)
    traj_point_list = []
    for iter in range(T):
        iter_time = total_time * iter / (T - 1)  # iter * dt

        traj_point_list.append(
            TrajectoryPoint(
                robot_configuration=np.array(hpp_path.call(iter_time)[0][:robot_ndof]),
                robot_velocity=np.array(hpp_path.derivative(iter_time, 1)[:robot_ndof]),
                robot_acceleration=np.array(
                    hpp_path.derivative(iter_time, 2)[:robot_ndof]
                ),
                end_effector_poses={"link0": np.eye(4)},
            )
        )
    return traj_point_list
