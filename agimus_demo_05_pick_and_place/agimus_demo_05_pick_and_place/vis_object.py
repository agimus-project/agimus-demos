import numpy as np
from pathlib import Path
import example_robot_data as erd
from robomeshcat import Object, Scene, Robot
import pickle
import pinocchio as pin

" Vis object from .ply file "
# scene = Scene()
# scene.add_object(Object.create_mesh(path_to_mesh=Path(__file__).parent / 'urdf/obj_23.ply', scale=1e-3, name='obj'))

"Vis hpp traj from q_init.pkl"
obj_name = "obj_23"
robot = erd.load("panda")
robot_pose = np.eye(4)
robot_pose[:3, 3] = [0.563, -0.1655, 0.78]
robot_pose[:3, :3] = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])

hpp_configs = pickle.load(open(Path(__file__).parent / "q_init.pkl", "rb"))
scene = Scene()
moving_object = Object.create_mesh(
    path_to_mesh=Path(__file__).parent / f"urdf/{obj_name}.ply", scale=1e-3, name="obj"
)
scene.add_object(moving_object)

print(Path(robot.urdf))
rob = Robot(
    urdf_path=robot.urdf,
    mesh_folder_path=Path(robot.urdf).parent.parent.parent.parent.parent,
    name="panda",
)
scene.add_robot(rob)
rob.pose = robot_pose

with scene.animation(fps=1):  # start the animation with the current scene
    for hpp_q in hpp_configs:
        rob[:] = hpp_q[:9]
        moving_object.pose = pin.XYZQUATToSE3(hpp_q[9 : 9 + 7]).homogeneous

        scene.render()

scene.render_image()
