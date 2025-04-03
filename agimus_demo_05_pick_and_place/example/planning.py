import numpy as np
from agimus_demo_05_pick_and_place.hpp_client import HPPInterface
from agimus_demo_05_pick_and_place.orchestrator import (
    get_hardcoded_initial_object_pose,
    multiply_poses,
)
import pinocchio.liegroups
import eigenpy
import random

eigenpy.seed(random.randint(0, 1000))

def random_quaternion():
    so3 = pinocchio.liegroups.SO3()
    return so3.random().tolist()

scenario = 2
if scenario == 0:
    object_name = "obj_21"
    obj_in_world_pose = get_hardcoded_initial_object_pose(object_name)
    robot_init_config = [
        0.0,
        -0.7862140995395444,
        0.0,
        -2.373050927881054,
        0.0,
        1.5711160116981167,
        0.785398163397438,
        0.04,
        0.04,
    ]
    goal_obj_pose = [0., 0., 0.15]
elif scenario == 1:
    object_name = "obj_23"
    robot_init_config = [
        -0.022136991300342374,
        -0.26823348731009766,
        0.07020840263646232,
        -2.423302171238089,
        -0.1399733059944589,
        2.2062745136949746,
        0.9638526762326558,
        0.03892720118165016,
        0.03892720118165016,
    ]
    obj_in_world_pose = [
        -0.01679423395981995,
        -0.18984389827298917,
        0.8252371533471734,
        -0.002136521717574758,
        0.997926546804224,
        0.061979655231819336,
        -0.017221056753064356,
    ]

    goal_obj_pose = [0., 0., 0.15]
    #     0.05,
    #     -0.2,
    #     0.761,
    #     0.0,
    #     0.0,
    #     0.0,
    #     1.0,
    #     0.5,
    #     0.2,
    #     0.761,
    #     0.0,
    #     0.0,
    #     0.0,
    #     1.0,
    # ]
elif scenario == 2:
    object_name = "obj_23"
    robot_init_config = [
        -0.022136991300342374,
        -0.26823348731009766,
        0.07020840263646232,
        -2.423302171238089,
        -0.1399733059944589,
        2.2062745136949746,
        0.9638526762326558,
        0.03892720118165016,
        0.03892720118165016,
    ]
    obj_in_world_pose = [
        -0.01679423395981995,
        -0.18984389827298917,
        0.8752371533471734,
    ] + random_quaternion()

    goal_obj_pose = [0., 0., 0.15]
# hpp_q_init = (
#     robot_init_config + hpp_client.start_obj_pose + hpp_client.default_obstacle_pose + hpp_client.default_obstacle2_pose
# )
# hpp_client.robot.setCurrentConfig(hpp_q_init)
# cam_in_world_pose = hpp_client.robot.getLinkPosition(
#     linkName="panda/camera_color_optical_frame"
# )
# obj_in_world_pose = multiply_poses(cam_in_world_pose, obj_in_cam_pose)
# TODO: make this better

hpp_client = HPPInterface(object_name)
obj_in_world_pose[3:] = obj_in_world_pose[3:] / np.linalg.norm(obj_in_world_pose[3:])
hpp_client.start_obj_pose = list(obj_in_world_pose)
hpp_client.goal_obj_pose = goal_obj_pose
output = hpp_client.plan_pick_and_place(robot_init_config)

v = hpp_client.vf.createViewer()
v (hpp_client.q_init)