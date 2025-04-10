import numpy as np
from agimus_demo_05_pick_and_place.hpp_client import HPPInterface
from agimus_demo_05_pick_and_place.orchestrator import (
    get_hardcoded_initial_object_pose,
    multiply_poses,
)
from agimus_demo_05_pick_and_place import graspit_script_db
import plyfile
import numpy as np
import pathlib

obj_id = "21"
object_name = f"obj_{obj_id}"
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

goal_obj_pose = [0.0, 0.0, 0.15]
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
# hpp_q_init = (
#     robot_init_config + hpp_client.start_obj_pose + hpp_client.default_obstacle_pose + hpp_client.default_obstacle2_pose
# )
# hpp_client.robot.setCurrentConfig(hpp_q_init)
# cam_in_world_pose = hpp_client.robot.getLinkPosition(
#     linkName="panda/camera_color_optical_frame"
# )
# obj_in_world_pose = multiply_poses(cam_in_world_pose, obj_in_cam_pose)
# TODO: make this better

# dir = pathlib.Path("/home/gepetto/ros2_ws/src/t-less_toolkit/t-less_v2/test_kinect/01/ply")
# filename = (dir / "0000.ply").absolute().as_posix()
# plydata = plyfile.PlyData.read(filename)
# points = plydata["vertex"].data[['x','y','z']].tolist()
# colors = [(r/255,g/255,b/255,1.0) for r,g,b in plydata["vertex"].data[['red','green','blue']]]

hpp_client = HPPInterface(
    object_name,                          
    source_bin_pose=[-0.9, -0.9, 0.761, 0.0, 0.0, 0.0, 1.0],
)

import pickle
import data_parser

with open("datas/recorded_scene01b.pkl", "rb") as f:
    data = pickle.load(f)

points, colors, depth_frame = data_parser.read_points_xyz_rgb(data, "panda/")
q_robot = data_parser.get_robot_config(data, hpp_client.robot, "panda")
cMo, camera_frame = data_parser.get_object_pose(
    data, f"tless-obj_0000{obj_id}", "panda/"
)
hpp_client.set_point_cloud(
    q_robot=q_robot,
    camera_frame_name=depth_frame,
    points=points.tolist(),
    colors=colors.tolist(),
)
wMc = hpp_client.get_robot_link_position(q_robot, camera_frame)
wMo = multiply_poses(wMc, cMo)

wMo[3:] = wMo[3:] / np.linalg.norm(wMo[3:])
hpp_client.start_obj_pose = list(wMo)
hpp_client.goal_obj_pose = goal_obj_pose

grasps = graspit_script_db.load_from_csv(f"datas/graspit_obj{obj_id}.csv")
# gui = v.client.gui
# if gui.nodeExists("part_grasps"):
#     gui.deleteNode("part_grasps", True)
# gui.createGroup("part_grasps")
# gui.addToGroup("part_grasps", "robot/part/base_link")
graspit_handles = list()
for i, grasp in enumerate(grasps):
    name = f"part_grasps/grasp_{i}"
    # gui.addXYZaxis(name, [1, 0, 0, 1], 0.001, 0.01)
    #  grasp.translation -= [0,0,1]
    # gui.applyConfiguration(name, grasp.pose_list)
    graspit_handles.append(f"graspit_{i}")
    hpp_client.ps.client.manipulation.robot.addHandle(
        "part/base_link",
        graspit_handles[-1],
        grasp.pose_list,
        0.05,
        [
            True,
        ]
        * 6,
    )
# gui.refresh()
hpp_client.handles = graspit_handles

q0 = hpp_client.robot.getCurrentConfig()
q0[:len(robot_init_config)] = robot_init_config
grasps_config = list()
for i in range(len(grasps)):
    node_name = f"panda/panda_gripper grasps graspit_{i}"
    ok, q, err = graph.applyNodeConstraints(node_name, q0)
    grasps_config.append(q)

# output = hpp_client.plan_pick_and_place(robot_init_config)
# hpp_client.ps.client.basic.obstacle.loadPoint

# v.client.gui.addMesh("pointcloud", filename)
# v.client.gui.addCurve("pointcloud", points, [1,0,0,1])
# v.client.gui.setCurveMode("pointcloud", "POINTS")
# v.client.gui.setCurveColors("pointcloud", colors)
# v.client.gui.addToGroup("pointcloud", "scene_hpp_")
# v (hpp_client.q_init)

# TODO
# For this script to run fine, one needs to increase
# giopMaxMsgSize in file /opt/openrobots/etc/omniORB.cfg
# The default value is 2097152. Multiplying by 10 worked for me.
