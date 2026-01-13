import numpy as np
import pinocchio as pin

from robomeshcat import Robot, Scene, Object
from example_robot_data.robots_loader import PandaLoader as RobLoader
import example_robot_data as robex
from pathlib import Path
import time


robot = robex.load("panda")
model = robot.model
data = model.createData()

q = pin.neutral(model)
ee_frame_name = "panda_hand"  # change to your frame name
ee_frame_id = model.getFrameId(ee_frame_name)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
word_to_ee = data.oMf[ee_frame_id]
ee_to_gripper = pin.XYZQUATToSE3(
    np.array([0, 0, 0.103, 0, -0.7071067811865476, 0, 0.7071067811865476])
)

obj_to_default_handle = pin.XYZQUATToSE3(np.array([0, 0, 0, 0, 0, 0, 1.0]))
# side_B_zn_yp
obj_to_base_handle = pin.XYZQUATToSE3(
    np.array([-0.05, -0.019, 0.0, 0.0, 0.7071067811865475, 0.0, 0.7071067811865476])
)
obj_to_base_handle_rotated = pin.XYZQUATToSE3(
    np.array([-0.05, 0.019, 0.0, 0.0, 0.7071067811865475, 0.0, 0.7071067811865476])
)
turn_45 = pin.SE3(
    pin.utils.rpyToMatrix(np.array([0, -np.pi / 4, 0])), np.array([0.0, 0.0, 0.0])
)
turn_135 = pin.SE3(
    pin.utils.rpyToMatrix(np.array([0, -3 * np.pi / 4, 0])), np.array([0.0, 0.0, 0.0])
)
turn_around_x = pin.SE3(
    pin.utils.rpyToMatrix(np.array([np.pi, 0, 0])), np.array([0.0, 0.0, 0.0])
)

obj_to_base_handle_45deg = obj_to_base_handle * turn_45
obj_to_base_handle_135deg = obj_to_base_handle * turn_135
# rotate 180
obj_to_base_handle_45deg_rotated = turn_around_x * obj_to_base_handle_rotated * turn_45
obj_to_base_handle_135deg_rotated = (
    turn_around_x * obj_to_base_handle_rotated * turn_135
)

# side_A_zn_yn
obj_to_side_A_zn_yn = pin.XYZQUATToSE3(
    np.array([0.05, -0.019, 0.0, 0.7071067811865476, 0.0, -0.7071067811865475, 0.0])
)
obj_to_side_A_zn_yn_rotated = pin.XYZQUATToSE3(
    np.array([0.05, 0.019, 0.0, 0.7071067811865476, 0.0, -0.7071067811865475, 0.0])
)
obj_to_side_A_zn_yn_45deg = obj_to_side_A_zn_yn * turn_45
obj_to_side_A_zn_yn_135deg = obj_to_side_A_zn_yn * turn_135
# rotate 180
obj_to_side_A_zn_yn_45deg_rotated = (
    turn_around_x * obj_to_side_A_zn_yn_rotated * turn_45
)
obj_to_side_A_zn_yn_135deg_rotated = (
    turn_around_x * obj_to_side_A_zn_yn_rotated * turn_135
)

all_handles = [
    obj_to_default_handle,
    obj_to_base_handle,
    obj_to_base_handle_45deg,
    obj_to_base_handle_45deg_rotated,
    obj_to_base_handle_135deg,
    obj_to_base_handle_135deg_rotated,
    obj_to_side_A_zn_yn,
    obj_to_side_A_zn_yn_45deg,
    obj_to_side_A_zn_yn_135deg,
    obj_to_side_A_zn_yn_45deg_rotated,
    obj_to_side_A_zn_yn_135deg_rotated,
]

all_handles_names = [
    "default_handle",
    "side_B_zn_yp",
    "side_B_zn_yp_45deg",
    "side_B_zn_yp_45deg_rotated",
    "side_B_zn_yp_135deg",
    "side_B_zn_yp_135deg_rotated",
    "side_A_zn_yn",
    "side_A_zn_yn_45deg",
    "side_A_zn_yn_135deg",
    "side_A_zn_yn_45deg_rotated",
    "side_A_zn_yn_135deg_rotated",
]

# print(obj_to_side_B_zn_yp_handle * obj_to_side_B_zn_yp_45deg_handle.inverse())
# exit(66)
# handle = gripper (they match)
scene = Scene()
rob = Robot(
    urdf_path=RobLoader().df_path,
    mesh_folder_path=Path(RobLoader().model_path).parent.parent,
    name="visual",
)
scene.add_robot(rob)


obj_file = "/home/ros/agimus_dev_container/agimus-demos/agimus_demo_05_pick_and_place/urdf/tless/obj_23.obj"
obj = Object.create_mesh(path_to_mesh=Path(obj_file), scale=1e-3, name="obj")
scene.add_object(obj)


with scene.animation(fps=1):  # start the animation with the current scene
    for p, name in zip(all_handles, all_handles_names):
        obj.pose = word_to_ee * ee_to_gripper * p.inverse()
        x, y, z, qx, qy, qz, qw = pin.SE3ToXYZQUAT(p).tolist()
        print(f"""    <handle name="{name}" clearance="0.03">
        <position xyz="{x} {y} {z}" xyzw="{qx} {qy} {qz} {qw}"/>
        <link name="base_link"/>
    </handle>""")
        scene.render()


time.sleep(5)  # we need some time to send the animation to the browser
