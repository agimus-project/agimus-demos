import numpy as np
from agimus_demo_05_pick_and_place.hpp_client import HPPInterface
from agimus_demo_05_pick_and_place.orchestrator import hardcoded_config, multiply_poses

object_name = "obj_21"
hpp_client = HPPInterface(object_name)

obj_in_cam_pose = hardcoded_config(object_name)
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
hpp_q_init = (
    robot_init_config + hpp_client.start_obj_pose + hpp_client.default_obstacle_pose
)
hpp_client.robot.setCurrentConfig(hpp_q_init)
cam_in_world_pose = hpp_client.robot.getLinkPosition(
    linkName="panda/camera_color_optical_frame"
)
obj_in_world_pose = multiply_poses(cam_in_world_pose, obj_in_cam_pose)
# TODO: make this better
obj_in_world_pose[3:] = obj_in_world_pose[3:] / np.linalg.norm(obj_in_world_pose[3:])
hpp_client.start_obj_pose = list(obj_in_world_pose)
grasp_path, placing_path, freefly_path = hpp_client.plan(robot_init_config)
